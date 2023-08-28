//
//  WitnessInferenceRule.h
//  The Witness Editor
//
//  Created by Samarium on 2023-07-31.
//  Copyright © 2023 MovingAI. All rights reserved.
//
#ifndef THE_WITNESS_EDITOR_INCLUDE_WITNESS_INFERENCE_RULE_H
#define THE_WITNESS_EDITOR_INCLUDE_WITNESS_INFERENCE_RULE_H

#include "PuzzleInferenceRule.h"
#include "Witness.h"

static bool CompareSC(const WitnessRegionConstraint &a, const WitnessRegionConstraint &b)
{
    return a.t == kSeparation && b.t == kSeparation && a != b;
}

template<int width, int height>
ActionType SeparationRule(const SearchEnvironment<WitnessState<width, height>, WitnessAction> &env,
                          const WitnessState<width, height> &state, const WitnessAction &action)
{
    if (state.path.empty())
        return UNKNOWN;
    const auto witness = dynamic_cast<const Witness<width, height>*>(&env);
    int currX = state.path.back().first;
    int currY = state.path.back().second;
    switch (action) {
        case kUp:
        {
            if (currX < width && currY < height
                && CompareSC(witness->GetRegionConstraint(currX - 1, currY),
                             witness->GetRegionConstraint(currX, currY)))
                return MUST_TAKE;
            break;
        }
        case kRight:
        {
            if (currY > 0 && currY < height
                && CompareSC(witness->GetRegionConstraint(currX, currY),
                             witness->GetRegionConstraint(currX, currY - 1)))
                return MUST_TAKE;
            break;
        }
        case kDown:
        {
            if (currX < width && currY > 0
                && CompareSC(witness->GetRegionConstraint(currX - 1, currY - 1),
                             witness->GetRegionConstraint(currX, currY - 1)))
                return MUST_TAKE;
            break;
        }
        case kLeft:
        {
            if (currY > 0 && currY < height
                && CompareSC(witness->GetRegionConstraint(currX - 1, currY - 1),
                             witness->GetRegionConstraint(currX - 1, currY)))
                return MUST_TAKE;
            break;
        }
        default:
            break;
    }
    return UNKNOWN;
}

template <int width, int height>
ActionType PathConstraintRule(const SearchEnvironment<WitnessState<width, height>, WitnessAction> &env,
                              const WitnessState<width, height> &state, const WitnessAction &action)
{
    if (state.path.empty())
        return UNKNOWN;
    const auto witness = dynamic_cast<const Witness<width, height>*>(&env);
    int currX = state.path.back().first;
    int currY = state.path.back().second;
    switch (action) {
        case kUp:
        {
            if (witness->GetMustCrossConstraint(false, currX, currY))
                return MUST_TAKE;
            if (witness->GetCannotCrossConstraint(false, currX, currY))
                return CANNOT_TAKE;
            break;
        }
        case kRight:
        {
            if (witness->GetMustCrossConstraint(true, currX, currY))
                return MUST_TAKE;
            if (witness->GetCannotCrossConstraint(true, currX, currY))
                return CANNOT_TAKE;
            break;
        }
        case kDown:
        {
            if (witness->GetMustCrossConstraint(false, currX, currY - 1))
                return MUST_TAKE;
            if (witness->GetCannotCrossConstraint(false, currX, currY - 1))
                return CANNOT_TAKE;
            break;
        }
        case kLeft:
        {
            if (witness->GetMustCrossConstraint(true, currX - 1, currY))
                return MUST_TAKE;
            if (witness->GetCannotCrossConstraint(true, currX - 1, currY))
                return CANNOT_TAKE;
        }
        default:
            break;
    }
    return UNKNOWN;
}

static void CalculateDistances(const std::pair<int, int> &pos, const std::vector<std::pair<int, int>> &goals, std::vector<double> &dists)
{
    for (size_t i = 0; i < goals.size(); ++i)
    {
        auto &g = goals[i];
        dists[i] = std::sqrt(std::pow(g.first - pos.first, 2.0) + std::pow(g.second - pos.second, 2.0));
    }
}

template <int width, int height>
ActionType TowardsGoalRule(const SearchEnvironment<WitnessState<width, height>, WitnessAction> &env,
                           const WitnessState<width, height> &state, const WitnessAction &action)
{
    if (state.path.empty())
        return UNKNOWN;
    if (action == kEnd)
        return MUST_TAKE;
    if (!state.hitTheWall() || state.isAlongTheWall())
        return UNKNOWN;
    const auto witness = dynamic_cast<const Witness<width, height>*>(&env);
    auto &currPos = state.path.back();
    auto nextPos = currPos;
    switch (action) {
        case kUp:
        {
            ++nextPos.second;
            break;
        }
        case kRight:
        {
            ++nextPos.first;
            break;
        }
        case kDown:
        {
            --nextPos.second;
            break;
        }
        case kLeft:
        {
            --nextPos.first;
            break;
        }
        default:
            return UNKNOWN;
    }
    auto currDists = std::vector<double>(witness->goal.size());
    auto nextDists = std::vector<double>(witness->goal.size());
    CalculateDistances(currPos, witness->goal, currDists);
    CalculateDistances(nextPos, witness->goal, nextDists);
    for (size_t i = 0; i < nextDists.size(); ++i)
        nextDists[i] -= currDists[i];
    if (std::all_of(nextDists.begin(), nextDists.end(), [](double d){ return d > 0; }))
        return CANNOT_TAKE;
    return UNKNOWN;
}

#endif /* THE_WITNESS_EDITOR_INCLUDE_WITNESS_INFERENCE_RULE_H */
