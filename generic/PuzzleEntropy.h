//
//  PuzzleEntropy.hpp
//  The Witness Editor
//
//  Created by Samarium on 2023-07-21.
//  Copyright © 2023 MovingAI. All rights reserved.
//
#ifndef HOG2_GENERIC_PUZZLE_ENTROPY_H
#define HOG2_GENERIC_PUZZLE_ENTROPY_H

#include <numeric>
#include <vector>
#include "PuzzleInferenceRule.h"
#include "SearchEnvironment.h"
#include "vectorCache.h"

struct EntropyInfo
{
    double value;
    unsigned depth;
};

template<class State, class Action>
class Entropy
{
protected:
    static constexpr double inf = std::numeric_limits<double>::max();
    vectorCache<Action> actCache;
    vectorCache<EntropyInfo> entropyInfoCache;
    vectorCache<double> doubleCache;
    bool isRelative = false;

    static void Softmin(const std::vector<double> &vars, std::vector<double> &ret)
    {
        double sum = std::accumulate(vars.begin(), vars.end(), 0.0, [](double r, double i) {
            return r + std::exp(-i);
        });
        ret.reserve(vars.size());
        std::transform(vars.begin(), vars.end(), std::back_inserter(ret), [&](double i) {
            return std::exp(-i) / sum;
        });
    }

    static double KlDivergence(const std::vector<double> &p, const std::vector<double> &q)
    {
        if (p.size() != q.size())
            throw std::invalid_argument("Input vectors must be of the same size.");
        double divergence = 0.0;
        for (auto i = 0; i < p.size(); ++i)
        {
            if (p[i] != 0.0 && q[i] != 0.0)
                divergence += p[i] * std::log2(p[i] / q[i]);
        }
        return divergence;
    }

    double ImmediateEntropy(const std::vector<Action> &actions,
                            const std::vector<double> &childEntropy,
                            std::optional<Action> prevAction)
    {
        auto size = actions.size();
        if (!isRelative)
            return std::log2(static_cast<double>(size));
        std::vector<double> &childDist = *doubleCache.getItem();
        Softmin(childEntropy, childDist);
        std::vector<double> &expectedDist = *doubleCache.getItem();
        expectedDist.resize(size);
        std::generate(expectedDist.begin(), expectedDist.end(), [
            size = static_cast<double>(size)
        ]() { return 1 / size; });
        if (size > 1 && prevAction.has_value() &&
            std::find(actions.begin(), actions.end(), prevAction.value()) != actions.end())
        {
            for (auto i = 0; i < size; ++i)
            {
                if (actions[i] == prevAction.value())
                    expectedDist[i] += probShift;
                else
                    expectedDist[i] -= probShift / (size - 1);
            }
        }
        auto kl = KlDivergence(childDist, expectedDist);
        doubleCache.returnItem(&childDist);
        doubleCache.returnItem(&expectedDist);
        return kl;
    }

public:
    PuzzleInferenceRuleSet<State, Action> ruleSet;
    double probShift = 0.0;

    auto& SetRelative(bool val)
    {
        this->isRelative = val;
        return *this;
    }

    auto& SetShift(double val)
    {
        this->probShift = val;
        return *this;
    }

    virtual EntropyInfo Calculate(const SearchEnvironment<State, Action> &env, State &state, unsigned lookahead,
                                  std::optional<Action> prevAction)
    {
        if (env.GoalTest(state))
            return { 0.0, 0 };
        std::vector<Action> &allActions = *actCache.getItem();
        env.GetActions(state, allActions);
        ruleSet.FilterActions(env, state, allActions);
        if (allActions.empty())
        {
            actCache.returnItem(&allActions);
            return { inf, 0 };
        }
        std::vector<EntropyInfo> &children = *entropyInfoCache.getItem();
        for (const auto &action: allActions)
        {
            env.ApplyAction(state, action);
            children.emplace_back(Calculate(env, state, (lookahead > 0) ? lookahead - 1 : 0, action));
            env.UndoAction(state, action);
        }
        for (auto it = children.begin(); it != children.end(); )
        {
            auto &info = *it;
            if (info.value == 0 && info.depth < lookahead)
                return { 0.0, info.depth + 1 };
            if (info.value == inf && info.depth < lookahead)
                it = children.erase(it);
            else
                ++it;
        }
        EntropyInfo entropyInfo{};
        if (std::all_of(children.begin(), children.end(), [](EntropyInfo &info) {
            return info.value == 0;
        }))
            entropyInfo = { 0.0, children[0].depth + 1 };
        else if (std::all_of(children.begin(), children.end(), [](EntropyInfo &info) {
            return info.value == inf;
        }))
            entropyInfo = { inf, children[0].depth + 1 };
        else
        {
            auto &minChild = *std::min_element(children.begin(), children.end(),
                                               [](EntropyInfo &info1, EntropyInfo &info2) {
                return info1.value < info2.value;
            });
            auto &childrenEntropy = *doubleCache.getItem();
            childrenEntropy.reserve(children.size());
            std::transform(children.begin(), children.end(), std::back_inserter(childrenEntropy),
                           [](EntropyInfo &info){ return info.value; });
            entropyInfo = { minChild.value + ImmediateEntropy(allActions, childrenEntropy, prevAction),
                            minChild.depth + 1 };
            doubleCache.returnItem(&childrenEntropy);
        }
        entropyInfoCache.returnItem(&children);
        actCache.returnItem(&allActions);
        return entropyInfo;
    }
    
    virtual EntropyInfo CalculateDeadEndEntropy(const SearchEnvironment<State, Action> &env, State &state,
                                                unsigned lookahead)
    {
        return { 0.0, 0 };
    }
};

#endif /* HOG2_GENERIC_PUZZLE_ENTROPY_H */
