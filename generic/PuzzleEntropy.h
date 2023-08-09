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
#include "SearchEnvironment.h"
#include "vectorCache.h"
#include "PuzzleInferenceRule.h"

struct EntropyInfo
{
    double entropy;
    unsigned depth;
};

template<class State, class Action>
class Entropy
{
protected:
    static constexpr double inf = std::numeric_limits<double>::max();
    vectorCache<Action> actCache;
    vectorCache<EntropyInfo> entropyInfoCache;
    bool isRelative = false;

    static std::vector<double> Softmin(const std::vector<double> &vars)
    {
        double sum = std::accumulate(vars.begin(), vars.end(), 0.0, [](double r, double i) {
            return r + std::exp(-i);
        });
        auto ret = std::vector<double>();
        ret.reserve(vars.size());
        std::transform(vars.begin(), vars.end(), std::back_inserter(ret), [&](double i) {
            return std::exp(-i) / sum;
        });
        return ret;
    }

    static double KlDivergence(const std::vector<double> &p, const std::vector<double> &q)
    {
        if (p.size() != q.size())
            throw std::invalid_argument("Input vectors must be of the same size.");
        double divergence = 0.0;
        for (size_t i = 0; i < p.size(); ++i)
        {
            if (p[i] != 0.0 && q[i] != 0.0)
                divergence += p[i] * std::log(p[i] / q[i]);
        }
        return divergence;
    }

    double ImmediateEntropy(const std::vector<Action> &actions,
                            const std::vector<double> &childEntropy) const
    {
        auto size = static_cast<double>(actions.size());
        if (!isRelative)
            return std::log2(size);
        auto dist = Softmin(childEntropy);
        auto uniform = std::vector<double>(actions.size());
        std::generate(uniform.begin(), uniform.end(), [&]() { return 1 / size; });
        return KlDivergence(dist, uniform);
    }

    virtual void FilterActions(const SearchEnvironment<State, Action> &env, State &state,
                               std::vector<Action> &actions) const = 0;

public:

    auto& SetRelative(bool val)
    {
        this->isRelative = val;
        return *this;
    }

    virtual EntropyInfo Calculate(const SearchEnvironment<State, Action> &env, State &state, unsigned lookAhead)
    {
        if (env.GoalTest(state))
            return { 0.0, 0 };
        std::vector<Action> &allActions = *actCache.getItem();
        env.GetActions(state, allActions);
        FilterActions(env, state, allActions);
        if (allActions.empty())
        {
            actCache.returnItem(&allActions);
            return { inf, 0 };
        }
        std::vector<EntropyInfo> &childEntropyInfo = *entropyInfoCache.getItem();
        for (auto &action: allActions)
        {
            env.ApplyAction(state, action);
            childEntropyInfo.emplace_back(Calculate(env, state, (lookAhead > 0) ? lookAhead - 1 : 0));
            env.UndoAction(state, action);
        }
        for (auto it = childEntropyInfo.begin(); it != childEntropyInfo.end(); )
        {
            auto &info = *it;
            if (info.entropy == 0 && info.depth < lookAhead)
                return { 0.0, info.depth + 1 };
            if (info.entropy == inf && info.depth < lookAhead)
                it = childEntropyInfo.erase(it);
            else
                ++it;
        }
        EntropyInfo entropyInfo;
        if (std::all_of(childEntropyInfo.begin(), childEntropyInfo.end(), [](EntropyInfo &info) {
            return info.entropy == 0;
        }))
            entropyInfo = { 0.0, childEntropyInfo[0].depth + 1 };
        else if (std::all_of(childEntropyInfo.begin(), childEntropyInfo.end(), [](EntropyInfo &info) {
            return info.entropy == inf;
        }))
            entropyInfo = { inf, childEntropyInfo[0].depth + 1 };
        else
        {
            auto &min_childEntropyInfo = *std::min_element(childEntropyInfo.begin(), childEntropyInfo.end(),
                                       [](EntropyInfo &info1, EntropyInfo &info2){
                return info1.entropy < info2.entropy;
            });
            auto childEntropy = std::vector<double>();
            childEntropy.reserve(childEntropyInfo.size());
            std::transform(childEntropyInfo.begin(), childEntropyInfo.end(), std::back_inserter(childEntropy),
                           [](EntropyInfo &info){ return info.entropy; });
            entropyInfo = { min_childEntropyInfo.entropy + ImmediateEntropy(allActions, childEntropy),
                            min_childEntropyInfo.depth + 1 };
        }
        actCache.returnItem(&allActions);
        return entropyInfo;
    }
};

#endif /* HOG2_GENERIC_PUZZLE_ENTROPY_H */
