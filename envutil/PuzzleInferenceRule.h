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
#include <utility>

#include "SearchEnvironment.h"
#include "vectorCache.h"

enum ActionType
{
    CANNOT_TAKE,
    MUST_TAKE,
    UNKNOWN,
    INVALID
};

/**
 * @see https://stackoverflow.com/a/35920804
 */
template<typename T, typename... U>
size_t GetFunctionAddress(std::function<T(U...)> f) {
    typedef T(fnType)(U...);
    fnType ** fnPointer = f.template target<fnType*>();
    return (size_t) *fnPointer;
}

template<class State, class Action>
class PuzzleInferenceRuleSet {
    using InferenceRule = std::function<ActionType(const SearchEnvironment<State, Action>&, State&, const Action&)>;

protected:
    vectorCache<InferenceRule> rulesCache;
    mutable std::unordered_map<Action, ActionType> logics = {};

    virtual void UpdateActionLogics(const SearchEnvironment<State, Action> &env, State &state,
                                    std::unordered_map<Action, ActionType> &logics) const
    {
        std::for_each(enabledRules.begin(), enabledRules.end(), [&](auto &rule) {
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
    std::vector<InferenceRule>& disabledRules;
    std::vector<InferenceRule>& enabledRules;

    PuzzleInferenceRuleSet(): disabledRules(*rulesCache.getItem()), enabledRules(*rulesCache.getItem()) { }

    ~PuzzleInferenceRuleSet()
    {
        rulesCache.returnItem(&disabledRules);
        rulesCache.returnItem(&enabledRules);
    }

    virtual void SetRules(const std::vector<InferenceRule> &rules)
    {
        enabledRules.reserve(rules.size());
        enabledRules.insert(enabledRules.end(), rules.begin(), rules.end());
        disabledRules.reserve(rules.size());
    }

    virtual bool DisableRule(const InferenceRule &rule)
    {
        auto it = std::find_if(enabledRules.begin(), enabledRules.end(), [&](const auto &r) {
            return GetFunctionAddress(r) == GetFunctionAddress(rule);
        });
        if (it != enabledRules.end())
        {
            disabledRules.push_back(std::move(*it));
            enabledRules.erase(it);
            return true;
        }
        return false;
    }

    virtual bool EnableRule(const InferenceRule &rule)
    {
        auto it = std::find_if(disabledRules.begin(), disabledRules.end(), [&](const auto &r) {
            return GetFunctionAddress(r) == GetFunctionAddress(rule);
        });
        if (it != disabledRules.end())
        {
            enabledRules.push_back(std::move(*it));
            disabledRules.erase(it);
            return true;
        }
        return false;
    }

    virtual void DisableAllRules()
    {
        if (!enabledRules.empty())
        {
            disabledRules.insert(disabledRules.end(), enabledRules.begin(), enabledRules.end());
            enabledRules.clear();
        }
    }

    virtual void EnableAllRules()
    {
        if (!disabledRules.empty())
        {
            enabledRules.insert(enabledRules.end(), disabledRules.begin(), disabledRules.end());
            disabledRules.clear();
        }
    }

    virtual void FilterActions(const SearchEnvironment<State, Action> &env, State &state,
                               std::vector<Action> &actions) const
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
