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

static inline bool CompareSC(const WitnessRegionConstraint &a, const WitnessRegionConstraint &b)
{
    return a.type == kSeparation && b.type == kSeparation && a != b;
}

template<int width, int height>
ActionType SeparationRule(const SearchEnvironment<WitnessState<width, height>, WitnessAction> &env,
                          const WitnessState<width, height> &state, const WitnessAction &action)
{
    if (state.path.empty())
        return UNKNOWN;
    const auto witness = dynamic_cast<const Witness<width, height> *>(&env);
    const auto &[currX, currY] = state.path.back();
    switch (action)
    {
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

template<int width, int height>
ActionType PathConstraintRule(const SearchEnvironment<WitnessState<width, height>, WitnessAction> &env,
                              const WitnessState<width, height> &state, const WitnessAction &action)
{
    if (state.path.empty())
        return UNKNOWN;
    const auto witness = dynamic_cast<const Witness<width, height> *>(&env);
    const auto &[currX, currY] = state.path.back();
    switch (action)
    {
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
            break;
        }
        default:
            break;
    }
    return UNKNOWN;
}

template<int width, int height>
ActionType TowardsGoalRule(const SearchEnvironment<WitnessState<width, height>, WitnessAction> &env,
                           const WitnessState<width, height> &state, const WitnessAction &action)
{
    if (state.path.empty())
        return UNKNOWN;
    if (action == kEnd)
        return MUST_TAKE;
    if (!state.HitTheWall() || state.IsAlongTheWall())
        return UNKNOWN;
    const auto witness = dynamic_cast<const Witness<width, height> *>(&env);
    if (witness->start.size() > 1 || witness->goal.size() > 1)
        return UNKNOWN;
    std::pair<int, int> start = witness->start[0];
    if (start.first != 0 && start.second != 0)
        return UNKNOWN;
    std::pair<int, int> goal = witness->goal[0];
    const auto &[currX, currY] = state.path.back();
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
ActionType RegionCompletionRule(const SearchEnvironment<WitnessState<width, height>, WitnessAction> &env,
                                WitnessState<width, height> &state, const WitnessAction &action)
{
    if (state.path.empty())
        return UNKNOWN;
    const auto witness = dynamic_cast<const Witness<width, height> *>(&env);
    witness->ApplyAction(state, action);
    bool regionSatisfied = true;
    auto [x, y] = state.path.back();
    if ((state.HitTheWall() && !state.IsAlongTheWall()) ||
        witness->goalMap[witness->GetPathIndex(x, y)] != 0)
        regionSatisfied = witness->RegionTest(state);
    witness->UndoAction(state, action);
    return regionSatisfied ? UNKNOWN : CANNOT_TAKE;
}

template<int width, int height>
ActionType AloneThePathRule(const SearchEnvironment<WitnessState<width, height>, WitnessAction> &env,
                            WitnessState<width, height> &state, const WitnessAction &action)
{
    if (state.path.empty())
        return UNKNOWN;
    const auto witness = dynamic_cast<const Witness<width, height> *>(&env);
    witness->ApplyAction(state, action);
    bool satisfied = witness->PathTest(state);
    witness->UndoAction(state, action);
    return satisfied ? UNKNOWN : CANNOT_TAKE;
}

template<int width, int height>
static void GetTriangles(const Witness<width, height> &env, const WitnessState<width, height> &state,
                         const WitnessAction &action, unsigned num, std::vector<std::pair<int, int>> &pos)
{
    const auto &[currX, currY] = state.path.back();
    switch (action)
    {
        case kUp:
        {
            if (currX > 0)
            {
                const auto &constraint = env.GetRegionConstraint(currX - 1, currY);
                if (constraint.type == kTriangle && constraint.parameter == num)
                    pos.emplace_back(currX - 1, currY);
            }
            if (currX < width)
            {
                const auto &constraint = env.GetRegionConstraint(currX, currY);
                if (constraint.type == kTriangle && constraint.parameter == num)
                    pos.emplace_back(currX, currY);
            }
            break;
        }
        case kRight:
        {
            if (currY > 0)
            {
                const auto &constraint = env.GetRegionConstraint(currX, currY - 1);
                if (constraint.type == kTriangle && constraint.parameter == num)
                    pos.emplace_back(currX, currY - 1);
            }
            if (currY < height)
            {
                const auto &constraint = env.GetRegionConstraint(currX, currY);
                if (constraint.type == kTriangle && constraint.parameter == num)
                    pos.emplace_back(currX, currY);
            }
            break;
        }
        case kDown:
        {
            if (currX > 0)
            {
                const auto &constraint = env.GetRegionConstraint(currX - 1, currY - 1);
                if (constraint.type == kTriangle && constraint.parameter == num)
                    pos.emplace_back(currX - 1, currY - 1);
            }
            if (currX < width)
            {
                const auto &constraint = env.GetRegionConstraint(currX, currY - 1);
                if (constraint.type == kTriangle && constraint.parameter == num)
                    pos.emplace_back(currX, currY - 1);
            }
            break;
        }
        case kLeft:
        {
            if (currY > 0)
            {
                auto constraint = env.GetRegionConstraint(currX - 1, currY - 1);
                if (constraint.type == kTriangle && constraint.parameter == num)
                    pos.emplace_back(currX - 1, currY - 1);
            }
            if (currY < height)
            {
                auto constraint = env.GetRegionConstraint(currX - 1, currY);
                if (constraint.type == kTriangle && constraint.parameter == num)
                    pos.emplace_back(currX - 1, currY);
            }
            break;
        }
        default:
            break;
    }
}

template<int width, int height>
static unsigned CountOccupiedEdges(const WitnessState<width, height> &state, const std::pair<int, int> &pos)
{
    return static_cast<unsigned>(state.OccupiedEdge(pos.first, pos.second, pos.first, pos.second + 1)) +
           static_cast<unsigned>(state.OccupiedEdge(pos.first, pos.second + 1, pos.first + 1, pos.second + 1)) +
           static_cast<unsigned>(state.OccupiedEdge(pos.first + 1, pos.second + 1, pos.first + 1, pos.second)) +
           static_cast<unsigned>(state.OccupiedEdge(pos.first + 1, pos.second, pos.first, pos.second));
}

template<int width, int height>
ActionType OneTriangleRule(const SearchEnvironment<WitnessState<width, height>, WitnessAction> &env,
                           WitnessState<width, height> &state, const WitnessAction &action)
{
    if (state.path.empty())
        return UNKNOWN;
    const auto witness = dynamic_cast<const Witness<width, height> *>(&env);
    std::vector<std::pair<int, int>> pos;
    GetTriangles(*witness, state, action, 1, pos);
    witness->ApplyAction(state, action);
    for (auto &p: pos)
    {
        if (CountOccupiedEdges(state, p) > 1)
            return CANNOT_TAKE;
    }
    witness->UndoAction(state, action);
    return UNKNOWN;
}

enum WitnessInferenceRule {
    kSeparationRule,
    kPathConstraintRule,
    kTowardsGoalRule,
    kRegionCompletionRule,
    kAloneThePathRule,
    kOneTriangleRule,
//    kTwoTrianglesRule,
//    kThreeTrianglesRule,
    kInferenceRuleCount [[maybe_unused]]
};

inline std::ostream &operator<<(std::ostream &os, WitnessInferenceRule wir)
{
    switch (wir)
    {
        case kSeparationRule:
            return os << "SeparationRule";
        case kPathConstraintRule:
            return os << "PathConstraintRule";
        case kTowardsGoalRule:
            return os << "TowardsGoalRule";
        case kRegionCompletionRule:
            return os << "RegionCompletionRule";
        case kAloneThePathRule:
            return os << "AloneThePathRule";
        case kOneTriangleRule:
            return os << "OneTriangleRule";
        default:
            return os;
    }
}

template<int width, int height>
std::unordered_map<int,
        std::function<ActionType(const SearchEnvironment<WitnessState<width, height>, WitnessAction> &,
                                 WitnessState<width, height> &, const WitnessAction &)>>
witnessInferenceRules =
{
    { kSeparationRule, std::function(SeparationRule<width, height>) },
    { kPathConstraintRule, std::function(PathConstraintRule<width, height>) },
    { kTowardsGoalRule, std::function(TowardsGoalRule<width, height>) },
    { kRegionCompletionRule, std::function(RegionCompletionRule<width, height>) },
    { kAloneThePathRule, std::function(AloneThePathRule<width, height>) },
    { kOneTriangleRule, std::function(OneTriangleRule<width, height>) },
};

#endif /* THE_WITNESS_EDITOR_INCLUDE_WITNESS_INFERENCE_RULE_H */
