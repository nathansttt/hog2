//
//  WitnessPuzzleEntropy.h
//  The Witness Editor
//
//  Created by Samarium on 2023-08-08.
//  Copyright © 2023 MovingAI. All rights reserved.
//
#ifndef THE_WITNESS_EDITOR_INCLUDE_WITNESS_PUZZLE_ENTROPY_H
#define THE_WITNESS_EDITOR_INCLUDE_WITNESS_PUZZLE_ENTROPY_H

#include "PuzzleEntropy.h"
#include "Witness.h"
#include "WitnessInferenceRule.h"

template <int width, int height>
class WitnessPuzzleEntropy: public Entropy<WitnessState<width, height>, WitnessAction>
{
private:
    using State = WitnessState<width, height>;
    using Action = WitnessAction;

    void FilterActions(const SearchEnvironment<State, Action> &env, State &state,
                       std::vector<Action> &actions) const override
    {
        ruleSet.FilterActions(env, state, actions);
    }

public:
    WitnessPuzzleInferenceRuleSet<width, height> ruleSet;
};

#endif /* THE_WITNESS_EDITOR_INCLUDE_WITNESS_PUZZLE_ENTROPY_H */
