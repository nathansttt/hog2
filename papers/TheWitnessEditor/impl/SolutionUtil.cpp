#include "SolutionUtil.h"

void GetAllSolutions()
{
    //	std::vector<WitnessState<puzzleWidth, puzzleHeight>> puzzles;
    //	GetAllSolutions(puzzles);

    std::vector<WitnessState<2, 2>> p1;
    GetAllSolutions(p1);
    std::vector<WitnessState<2, 3>> p2;
    GetAllSolutions(p2);
    std::vector<WitnessState<3, 3>> p3;
    GetAllSolutions(p3);
    std::vector<WitnessState<3, 4>> p4;
    GetAllSolutions(p4);
    std::vector<WitnessState<4, 4>> p5;
    GetAllSolutions(p5);
    std::vector<WitnessState<4, 5>> p6;
    GetAllSolutions(p6);
    std::vector<WitnessState<5, 5>> p7;
    GetAllSolutions(p7);
}

void UpdateSolutionIndicies()
{
    currentSolutionIndices.clear();
    for (size_t i = 0; i < allSolutions.size(); i++)
    {
        auto &solution = allSolutions[i];
        if (witness.GoalTest(solution))
        {
            currentSolutionIndices.emplace_back(i);
        }
    }
    gEntropy = GetCurrentEntropy(witness);
}
