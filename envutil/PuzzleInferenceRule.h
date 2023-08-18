//
//  WitnessPuzzleInferenceRule.h
//  The Witness Editor
//
//  Created by Samarium on 2023-07-26.
//  Copyright © 2023 MovingAI. All rights reserved.
//
#ifndef HOG2_ENV_UTIL_PUZZLE_INFERENCE_RULE_H
#define HOG2_ENV_UTIL_PUZZLE_INFERENCE_RULE_H

#include <algorithm>
#include <unordered_map>
#include <vector>
#include "SearchEnvironment.h"

enum ActionType
{
    CANNOT_TAKE,
    MUST_TAKE,
    UNKNOWN,
    INVALID
};

template<class State, class Action>
class PuzzleInferenceRuleSet {
protected:
    virtual void UpdateActionLogics(const SearchEnvironment<State, Action> &env, const State &state,
                                    std::unordered_map<Action, ActionType> &logics) const
    {
        std::for_each(rules.begin(), rules.end(), [&](auto &rule) {
            std::for_each(logics.begin(), logics.end(), [&](auto &logic) {
                ActionType t = rule(env, state, logic.first);
                if (t == UNKNOWN)
                    return;
                if (logic.second != UNKNOWN && logic.second != t)
                    logic.second = INVALID;
                else logic.second = t;
            });
        });
    }
    
public:
    std::vector<std::function<ActionType(const SearchEnvironment<State, Action>&, const State&, const Action&)>> rules;
    std::unordered_map<Action, ActionType> logics;
    
    virtual void FilterActions(const SearchEnvironment<State, Action> &env, const State &state,
                               std::vector<Action> &actions)
    {
        logics.clear();
        std::transform(actions.begin(), actions.end(), std::inserter(logics, logics.end()), [](const Action &action) {
            return std::make_pair(action, UNKNOWN);
        });
        UpdateActionLogics(env, state, logics);
        if (std::count_if(logics.begin(), logics.end(), [](const auto &logic) {
            return logic.second == MUST_TAKE;
        }) > 1 || std::count_if(logics.begin(), logics.end(), [](const auto &logic) {
            return logic.second == INVALID;
        }) > 0)
        {
            actions.clear();
            return;
        }
        for (auto it = actions.begin(); it != actions.end(); )
        {
            Action &action = *it;
            switch (logics[action]) {
                case MUST_TAKE:
                {
                    actions.clear();
                    actions.push_back(action);
                    return;
                }
                case CANNOT_TAKE:
                {
                    it = actions.erase(it);
                    break;
                }
                default:
                    it++;
            }
        }
    }
};

#endif /* HOG2_ENV_UTIL_PUZZLE_INFERENCE_RULE_H */
