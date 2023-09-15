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
            if (currX > 0 && currX < width
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
            if (currX > 0 && currX < width
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
    if (witness->start.size() > 1 || witness->goal.size() > 1)
        return UNKNOWN;
    std::pair<int, int> start = witness->start[0];
    if (start.first != 0 && start.second != 0)
        return UNKNOWN;
    std::pair<int, int> goal = witness->goal[0];
    int currX = state.path.back().first;
    int currY = state.path.back().second;
    if (currX == 0 || currX == width) // vertical wall
    {
        if (currY < goal.second) // should choose kUP
        {
            if (start.second > currY && start.second <= goal.second) // unless the start is in the middle
                return (action == kDown) ? MUST_TAKE : CANNOT_TAKE;
            return (action == kUp) ? MUST_TAKE : CANNOT_TAKE;
        }
        else // should choose kDown
        {
            if (start.second < currY && start.second >= goal.second)
                return (action == kUp) ? MUST_TAKE : CANNOT_TAKE;
            return (action == kDown) ? MUST_TAKE : CANNOT_TAKE;
        }
    }
    else // horizontal wall
    {
        if (currX < goal.first) // should choose kRight
        {
            if (start.first > currX && start.first <= goal.first)
                return (action == kLeft) ? MUST_TAKE : CANNOT_TAKE;
            return (action == kRight) ? MUST_TAKE : CANNOT_TAKE;
        }
        else // should choose kLeft
        {
            if (start.first < currX && start.first >= goal.first)
                return (action == kRight) ? MUST_TAKE : CANNOT_TAKE;
            return (action == kLeft) ? MUST_TAKE : CANNOT_TAKE;
        }
    }
}

template<int width, int height>
std::vector<std::function<ActionType(const SearchEnvironment<WitnessState<width, height>, WitnessAction>&,
                                     const WitnessState<width, height>&, const WitnessAction&)>>
witnessInferenceRules =
{
    &SeparationRule<width, height>,
    &PathConstraintRule<width, height>,
    &TowardsGoalRule<width, height>
};

#endif /* THE_WITNESS_EDITOR_INCLUDE_WITNESS_INFERENCE_RULE_H */
