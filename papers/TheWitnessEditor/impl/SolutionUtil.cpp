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

int CountSolutions(const Witness<puzzleWidth, puzzleHeight> &wp,
    const std::vector<WitnessState<puzzleWidth, puzzleHeight>> &allSolutions,
                   int &len, uint64_t limit)
{
    int count = 0;
    for (const auto &i : allSolutions)
    {
        if (wp.GoalTest(i))
        {
            len = (int)i.path.size();
            count++;
        }
        if (count > limit) break;
    }
    return count;
}

int CountSolutions(const Witness<puzzleWidth, puzzleHeight> &w,
    const std::vector<WitnessState<puzzleWidth, puzzleHeight>> &allSolutions, std::vector<int> &solutions,
    const std::vector<int> &forbidden, int &len, uint64_t limit)
{
    solutions.resize(0);
    int count = 0;
    for (int x : forbidden)
    {
        if (w.GoalTest(allSolutions[x])) return 0;
    }
    for (int x = 0; x < allSolutions.size(); x++)
    {
        if (w.GoalTest(allSolutions[x]))
        {
            len = (int)allSolutions[x].path.size();
            count++;
            solutions.push_back(x);
        }
        if (count > limit) break;
    }
    return count;
}
