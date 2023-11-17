//
//  WitnessPuzzleEntropy.hpp
//  The Witness Editor
//
//  Created by Junwen Shen on 2023-11-16.
//  Copyright © 2023 MovingAI. All rights reserved.
//

#ifndef THE_WITNESS_EDITOR_INCLUDE_WITNESS_PUZZLE_ENTROPY_H
#define THE_WITNESS_EDITOR_INCLUDE_WITNESS_PUZZLE_ENTROPY_H

#include "PuzzleEntropy.h"
#include "PuzzleInferenceRule.h"
#include "Witness.h"

template<int width, int height>
class WitnessPuzzleEntropy : public Entropy<WitnessState<width, height>, WitnessAction>
{
    using H = Entropy<WitnessState<width, height>, WitnessAction>;
public:
    
    EntropyInfo CalculateDeadEndEntropy(const Witness<width, height> &env,
                                        WitnessState<width, height> &state,
                                        unsigned lookahead)
    {
        const auto &head = state.path.back();
        if (env.GoalTest(state))
            return { H::inf, 0 };
        else if (!state.path.empty() && (
                 std::find(env.goal.cbegin(), env.goal.cend(), head) != env.goal.cend() ||
            env.goalMap[env.GetPathIndex(head.first, head.second)] != 0))
            return { 0.0, 0 };
        std::vector<WitnessAction> &allActions = *H::actCache.getItem();
        env.GetActions(state, allActions);
        if (allActions.empty())
        {
            H::actCache.returnItem(&allActions);
            return { 0.0, 0 };
        }
        H::ruleSet.UpdateActionLogics(env, state, allActions);
        auto &logics = H::ruleSet.logics;
        if (std::count_if(logics.begin(), logics.end(), [](const auto &logic) {
            return logic.second == MUST_TAKE;
        }) > 1 || std::count_if(logics.begin(), logics.end(), [](const auto &logic) {
            return logic.second == INVALID;
        }) > 0)
        {
            H::actCache.returnItem(&allActions);
            return { 0.0, 0 };
        }
        std::vector<EntropyInfo> &children = *H::entropyInfoCache.getItem();
        for (const auto &action: allActions)
        {
            env.ApplyAction(state, action);
            children.emplace_back(logics[action] == CANNOT_TAKE ? EntropyInfo{ 0.0, 0 } :
                    CalculateDeadEndEntropy(env, state, (lookahead > 0) ? lookahead - 1 : 0));
            env.UndoAction(state, action);
        }
        EntropyInfo entropyInfo{};
        if (std::all_of(children.begin(), children.end(), [](EntropyInfo &info) {
            return info.value == H::inf;
        }))
            entropyInfo = { H::inf, children[0].depth + 1 };
        else
        {
            for (auto it = children.begin(); it != children.end(); )
            {
                if (it->value == H::inf)
                    it = children.erase(it);
                else
                    ++it;
            }
            auto &maxChild = *std::max_element(children.begin(), children.end(),
                                               [](EntropyInfo &info1, EntropyInfo &info2) {
                                                   return info1.value < info2.value;
                                               });
            entropyInfo = { maxChild.value + std::log2(static_cast<double>(allActions.size())),
                            maxChild.depth + 1 };
        }
        H::entropyInfoCache.returnItem(&children);
        H::actCache.returnItem(&allActions);
        return entropyInfo;
    }
};

#endif /* THE_WITNESS_EDITOR_INCLUDE_WITNESS_PUZZLE_ENTROPY_H */
