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

template<int width, int height>
class SeparationRule: public InferenceRule<WitnessState<width, height>, WitnessAction>
{
private:
    using State = WitnessState<width, height>;
    using Action = WitnessAction;
    
    SeparationRule() = default;

    static bool Compare(const WitnessRegionConstraint &a, const WitnessRegionConstraint &b)
    {
        return a.t == kSeparation && b.t == kSeparation && a != b;
    }
    
public:
    SeparationRule(const SeparationRule&) = delete;
    void operator=(const SeparationRule&) = delete;
    
    static auto& GetInstance()
    {
        static SeparationRule<width, height> instance;
        return instance;
    }
    
    void UpdateActionLogics(const SearchEnvironment<State, Action> &env, const State &state,
                            std::unordered_map<Action, ActionType> &logics) const override
    {
        if (state.path.empty())
            return;
        const auto witness = dynamic_cast<const Witness<width, height>*>(&env);
        int currX = state.path.back().first;
        int currY = state.path.back().second;
        
        if (logics.find(kUp) != logics.end()
            && currX < width && currY < height
            && Compare(witness->GetRegionConstraint(currX - 1, currY),
                       witness->GetRegionConstraint(currX, currY)))
            logics[kUp] = MUST_TAKE;
        
        if (logics.find(kRight) != logics.end()
            && currY > 0 && currY < height
            && Compare(witness->GetRegionConstraint(currX, currY),
                       witness->GetRegionConstraint(currX, currY - 1)))
            logics[kRight] = MUST_TAKE;
        
        if (logics.find(kDown) != logics.end()
            && currX < width && currY > 0
            && Compare(witness->GetRegionConstraint(currX - 1, currY - 1),
                       witness->GetRegionConstraint(currX, currY - 1)))
            logics[kDown] = MUST_TAKE;
        
        if (logics.find(kLeft) != logics.end()
            && currY > 0 && currY < height
            && Compare(witness->GetRegionConstraint(currX - 1, currY - 1),
                       witness->GetRegionConstraint(currX - 1, currY)))
            logics[kLeft] = MUST_TAKE;
    }
};

template <int width, int height>
class PathConstraintRule: public InferenceRule<WitnessState<width, height>, WitnessAction>
{
private:
    using State = WitnessState<width, height>;
    using Action = WitnessAction;
    
    PathConstraintRule() = default;
public:
    PathConstraintRule(const PathConstraintRule&) = delete;
    void operator=(const PathConstraintRule&) = delete;
    
    static auto& GetInstance()
    {
        static PathConstraintRule instance;
        return instance;
    }
    
    void UpdateActionLogics(const SearchEnvironment<State, Action> &env, const State &state,
                            std::unordered_map<Action, ActionType> &logics) const override
    {
        if (state.path.empty() || logics.empty())
            return;
        const auto witness = dynamic_cast<const Witness<width, height>*>(&env);
        int currX = state.path.back().first;
        int currY = state.path.back().second;
        
        if (logics.find(kUp) != logics.end())
        {
            if (witness->GetMustCrossConstraint(false, currX, currY))
                logics[kUp] = MUST_TAKE;
            if (witness->GetCannotCrossConstraint(false, currX, currY))
                logics[kUp] = CANNOT_TAKE;
        }
        
        if (logics.find(kRight) != logics.end())
        {
            if (witness->GetMustCrossConstraint(true, currX, currY))
                logics[kRight] = MUST_TAKE;
            if (witness->GetCannotCrossConstraint(true, currX, currY))
                logics[kRight] = CANNOT_TAKE;
        }
        
        if (logics.find(kDown) != logics.end())
        {
            if (witness->GetMustCrossConstraint(false, currX, currY - 1))
                logics[kDown] = MUST_TAKE;
            if (witness->GetCannotCrossConstraint(false, currX, currY - 1))
                logics[kDown] = CANNOT_TAKE;
        }
        
        if (logics.find(kLeft) != logics.end())
        {
            if (witness->GetMustCrossConstraint(true, currX - 1, currY))
                logics[kLeft] = MUST_TAKE;
            if (witness->GetCannotCrossConstraint(true, currX - 1, currY))
                logics[kLeft] = CANNOT_TAKE;
        }
    }
};

#endif /* THE_WITNESS_EDITOR_INCLUDE_WITNESS_INFERENCE_RULE_H */
