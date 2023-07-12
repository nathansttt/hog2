//
//  EntropyUtil.h
//  The Witness Editor
//
//  Created by Samarium on 2023-06-29.
//  Copyright © 2023 MovingAI. All rights reserved.
//

#ifndef THE_WITNESS_EDITOR_INCLUDE_ENTROPY_UTIL
#define THE_WITNESS_EDITOR_INCLUDE_ENTROPY_UTIL

#include "Globals.h"
#include "SolutionUtil.h"

template<int width, int height>
double MinimumUniformSolutionEntropy(const Witness<width, height> &witness,
              WitnessState<width, height> &state)
{
    auto puzzle = witness;
    if (puzzle.GoalTest(state))
        return 0.0;
    std::vector<WitnessState<width, height>> allSolutions;
    std::vector<WitnessAction> allActions;
    std::vector<double> childEntropy;
    puzzle.GetActions(state, allActions);
//    std::cout << "action size: " << allActions.size() << std::endl;
    if (allActions.empty())
        return  std::numeric_limits<double>::infinity();
    double immediateEntropy = std::log2(static_cast<double>(allActions.size()));
//    std::cout << "immedEnt: " << immediateEntropy << std::endl;
    for (auto action: allActions) {
        puzzle.ApplyAction(state, action);
        childEntropy.emplace_back(MinimumUniformSolutionEntropy(puzzle, state));
        puzzle.UndoAction(state, action);
    }
//    std::cout << "childEnts: ";
//    for (const auto &i : childEntropy)
//        std::cout << i << " ";
//    std::cout << std::endl;
    if (std::all_of(childEntropy.begin(), childEntropy.end(), [](int i) { return i == 0; } ))
        return 0.0;
    return *std::min_element(childEntropy.begin(), childEntropy.end()) + immediateEntropy;
}

#endif /* THE_WITNESS_EDITOR_INCLUDE_ENTROPY_UTIL */
