//
//  WitnessInferenceRule.h
//  The Witness Editor
//
//  Created by Samarium on 2023-07-31.
//  Copyright © 2023 MovingAI. All rights reserved.
//
#ifndef THE_WITNESS_EDITOR_INCLUDE_WITNESS_INFERENCE_RULE_H
#define THE_WITNESS_EDITOR_INCLUDE_WITNESS_INFERENCE_RULE_H

#include <variant>
#include "Globals.h"
#include "PuzzleInferenceRule.h"

template<int width, int height>
class SeparationRule: public InferenceRule<WitnessState<width, height>, WitnessAction> {
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
    
    void UpdateActionLogics(const SearchEnvironment<State, Action> &puzzle, const State &state,
                            std::unordered_map<Action, ActionType> &logics) const override
    {
        if (state.path.size() == 0)
            return;
        const auto witness = dynamic_cast<const Witness<width, height>*>(&puzzle);
        int currX = state.path.back().first;
        int currY = state.path.back().second;
        if (currX > 0 && currX < width && currY < height)
        {
            const WitnessRegionConstraint &a = witness->regionConstraints[currX - 1][currY];
            const WitnessRegionConstraint &b = witness->regionConstraints[currX][currY];
            if (logics.find(kUp) != logics.end() && Compare(a, b))
                logics[kUp] = MUST_TAKE;
        }
        if (currX < width && currY > 0 && currY < height)
        {
            const WitnessRegionConstraint &a = witness->regionConstraints[currX][currY];
            const WitnessRegionConstraint &b = witness->regionConstraints[currX][currY - 1];
            if (logics.find(kRight) != logics.end() && Compare(a, b))
                logics[kRight] = MUST_TAKE;
        }
        if (currX > 0 && currX < width && currY > 0)
        {
            const WitnessRegionConstraint &a = witness->regionConstraints[currX - 1][currY - 1];
            const WitnessRegionConstraint &b = witness->regionConstraints[currX][currY - 1];
            if (logics.find(kDown) != logics.end() && Compare(a, b))
                logics[kDown] = MUST_TAKE;
        }
        if (currX > 0 && currY > 0 && currY < height)
        {
            const WitnessRegionConstraint &a = witness->regionConstraints[currX - 1][currY - 1];
            const WitnessRegionConstraint &b = witness->regionConstraints[currX - 1][currY];
            if (logics.find(kLeft) != logics.end() && Compare(a, b))
                logics[kLeft] = MUST_TAKE;
        }
    }
};

template <int width, int height>
class WitnessPuzzleInferenceRuleSet: public PuzzleInferenceRuleSet<WitnessState<width, height>, WitnessAction>
{
private:
    using State = WitnessState<width, height>;
    using Action = WitnessAction;
    
public:
    std::vector<InferenceRule<State, Action>*> rules;

    void UpdateActionLogics(const SearchEnvironment<State, Action> &env, const State &state,
                            std::unordered_map<Action, ActionType> &logics) const override
    {
        std::for_each(rules.begin(), rules.end(), [&](auto rule) {
            rule->UpdateActionLogics(env, state, logics);
        });
    }
};

#endif /* THE_WITNESS_EDITOR_INCLUDE_WITNESS_INFERENCE_RULE_H */
