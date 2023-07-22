//
//  PuzzleEntropy.hpp
//  The Witness Editor
//
//  Created by Samarium on 2023-07-21.
//  Copyright © 2023 MovingAI. All rights reserved.
//

#ifndef HOG2_GENERIC_PUZZLE_ENTROPY
#define HOG2_GENERIC_PUZZLE_ENTROPY

#include <vector>
#include "SearchEnvironment.h"
#include "vectorCache.h"

template<class State, class Action>
class Entropy
{
protected:
    static std::vector<double> Softmin(const std::vector<double> &vars)
    {
        auto ret = std::vector<double>(vars.size());
        double sum = 0.0;
        std::for_each(vars.begin(), vars.end(), [&](double i) { sum += std::exp(-i); });
        size_t i = 0;
        std::generate(ret.begin(), ret.end(), [&]() { return std::exp(-vars[i++]) / sum;});
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

    static constexpr double inf = std::numeric_limits<double>::max();
    vectorCache<WitnessAction> actCache;
    vectorCache<double> entropyCache;
public:
    double MinimumUniformSolutionEntropy(SearchEnvironment<State, Action> &env, State &state)
    {
        if (env.GoalTest(state))
            return 0.0;
        std::vector<Action> &allActions = *actCache.getItem();
        env.GetActions(state, allActions);
        if (allActions.empty())
        {
            actCache.returnItem(&allActions);
            return inf;
        }
        std::vector<double> &childEntropy = *entropyCache.getItem();
        for (auto &action: allActions)
        {
            env.ApplyAction(state, action);
            childEntropy.emplace_back(this->MinimumUniformSolutionEntropy(env, state));
            env.UndoAction(state, action);
        }
        double ret;
        if (std::all_of(childEntropy.begin(), childEntropy.end(), [](int i) { return i == 0; } ))
            ret = 0.0;
        else if(std::all_of(childEntropy.begin(), childEntropy.end(), [](double i) { return i == inf; }))
            ret = inf;
        else
            ret = *std::min_element(childEntropy.begin(), childEntropy.end())
                + std::log2(static_cast<double>(allActions.size()));
        actCache.returnItem(&allActions);
        entropyCache.returnItem(&childEntropy);
        return ret;
    }

    double RelativeMUSE(SearchEnvironment<State, Action> &env, State &state)
    {
        if (env.GoalTest(state))
            return 0.0;
        std::vector<WitnessAction> &allActions = *actCache.getItem();
        env.GetActions(state, allActions);
        if (allActions.empty())
        {
            actCache.returnItem(&allActions);
            return inf;
        }
        std::vector<double> &childEntropy = *entropyCache.getItem();
        for (auto &action: allActions) {
            env.ApplyAction(state, action);
            childEntropy.emplace_back(this->RelativeMUSE(state));
            env.UndoAction(state, action);
        }
        double ret;
        if (std::all_of(childEntropy.begin(), childEntropy.end(), [](int i) { return i == 0; } ))
            ret = 0.0;
        else if(std::all_of(childEntropy.begin(), childEntropy.end(), [](double i) { return i == inf; }))
            ret = inf;
        else
        {
            auto dist = Entropy<State, Action>::Softmin(childEntropy);
            auto uniform = std::vector<double>(allActions.size());
            std::generate(uniform.begin(), uniform.end(), [&]() { return 1 / static_cast<double>(allActions.size()); });
            ret = *std::min_element(childEntropy.begin(), childEntropy.end())
                + Entropy<State, Action>::KlDivergence(dist, uniform);
        }
        actCache.returnItem(&allActions);
        entropyCache.returnItem(&childEntropy);
        return ret;
    }
};

#endif /* HOG2_GENERIC_PUZZLE_ENTROPY */
