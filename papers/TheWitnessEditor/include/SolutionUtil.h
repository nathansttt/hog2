//
//  SolutionUtil.h
//  The Witness Editor
//
//  Created by Samarium on 2023-06-29.
//  Copyright © 2023 MovingAI. All rights reserved.
//

#ifndef THE_WITNESS_EDITOR_INCLUDE_SOLUTION_UTIL_H
#define THE_WITNESS_EDITOR_INCLUDE_SOLUTION_UTIL_H

#include "Globals.h"
#include "Timer.h"

void GetAllSolutions();

int CountSolutions(const Witness <puzzleWidth, puzzleHeight> &w,
                   const std::vector <WitnessState<puzzleWidth, puzzleHeight>> &allSolutions,
                   int &len, uint64_t limit);

int CountSolutions(const Witness <puzzleWidth, puzzleHeight> &w,
                   const std::vector <WitnessState<puzzleWidth, puzzleHeight>> &allSolutions,
                   std::vector<int> &solutions,
                   const std::vector<int> &forbidden, int &len, uint64_t limit);

template<int puzzleWidth, int puzzleHeight>
void DFS(const Witness <puzzleWidth, puzzleHeight> &w, WitnessState <puzzleWidth, puzzleHeight> &s, // NOLINT
         std::vector <WitnessState<puzzleWidth, puzzleHeight>> &puzzles)
{
    std::vector <WitnessAction> acts;

    if (w.GoalTest(s))
    {
        puzzles.push_back(s);
        return;
    }

    w.GetActions(s, acts);
    for (auto &a: acts)
    {
        w.ApplyAction(s, a);
        DFS(w, s, puzzles);
        w.UndoAction(s, a);
    }
}

template<int puzzleWidth, int puzzleHeight>
void GetAllSolutions(
        const Witness <puzzleWidth, puzzleHeight> &w, std::vector <WitnessState<puzzleWidth, puzzleHeight>> &puzzles)
{
    WitnessState <puzzleWidth, puzzleHeight> s;
    s.Reset();
    Timer t;
    t.StartTimer();
    DFS(w, s, puzzles);
    t.EndTimer();
    printf("%lu solutions found in %1.2fs\n", puzzles.size(), t.GetElapsedTime());
}

template<int puzzleWidth, int puzzleHeight>
void GetAllSolutions(std::vector <WitnessState<puzzleWidth, puzzleHeight>> &puzzles)
{
    Witness <puzzleWidth, puzzleHeight> w;
    GetAllSolutions(w, puzzles);
}

#endif /* THE_WITNESS_EDITOR_INCLUDE_SOLUTION_UTIL_H */
