/*
 *  $Id: Driver.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Modified by Junwen Shen on 06/29/23.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */
#include "Driver.h"

#include "Globals.h"
#include "Puzzles.h"
#include "SolutionUtil.h"
#include "WitnessInferenceRule.h"

bool recording = false;
bool parallel = false;
bool drawEditor = false;
int selectTetrisPiece = 0;
// 2x2 + 5 (interesting)
// 2x5 + 5 [201/200][149][137/139][82][64][50]
// 3x4 + 5 [481/471][463!][399!][374! - 373][260][150][146][126]
unsigned long currBoard = 0;

Witness<puzzleWidth, puzzleHeight> witness;
Entropy<WitnessState<puzzleWidth, puzzleHeight>, WitnessAction> entropy;
InteractiveWitnessState<puzzleWidth, puzzleHeight> iws;
std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;

std::vector<TetrisItem> gTetrisPieces = {};

static void InitTetrisPieces()
{
    for (unsigned i = 1; i <= 24; ++i)
    {
        if (i <= 9)
        {
            Graphics::point p = {(float)(-1.0f + i * 0.2), -0.75f};
            gTetrisPieces.push_back({static_cast<int>(i), p, 0.1f});
        }
        else if (i <= 18)
        {
            Graphics::point p = {(float)(-1.0f + (i - 9) * 0.2), -0.55f};
            gTetrisPieces.push_back({static_cast<int>(i), p, 0.1f});
        }
        else
        {
            Graphics::point p = {(float)(-1.0f + (i - 18) * 0.2), -0.35f};
            gTetrisPieces.push_back({static_cast<int>(i), p, 0.1f});
        }
    }
}

static void AddInferenceRule()
{
    entropy.ruleSet.rules = witnessInferenceRules<puzzleWidth, puzzleHeight>;
}

static void InitPuzzle()
{
    k87fxsr();
    for (size_t i = 0; i < allSolutions.size(); i++)
    {
        auto &solution = allSolutions[i];
        if (witness.GoalTest(solution)) {
            currentSolutionIndices.emplace_back(i);
        }
    }
    std::sort(currentSolutionIndices.begin(), currentSolutionIndices.end(), [&](size_t a, size_t b) {
        return entropy.SetRelative(gUseRelativeEntropy).Calculate(witness, allSolutions[a], gLookahead).value >
            entropy.SetRelative(gUseRelativeEntropy).Calculate(witness, allSolutions[b], gLookahead).value;
    });
    gNumSolutions = currentSolutionIndices.size();
    AddInferenceRule();
    gEntropy = GetCurrentEntropy(witness);
}


/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
    InstallKeyboardHandler(WitnessKeyboardHandler, "Solve", "Solve current board", kAnyModifier, 'v');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Test", "Test constraints", kAnyModifier, 't');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Record", "Record a movie", kAnyModifier, 'r');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Save", "Save current puzzle as svg", kAnyModifier, 's');
    InstallKeyboardHandler(
        WitnessKeyboardHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Prev Board", "Jump to next found board.", kAnyModifier, '[');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Next Board", "Jump to prev found board", kAnyModifier, ']');
    InstallKeyboardHandler(
        WitnessKeyboardHandler, "Prev 100 Board", "Jump to next 100 found board.", kAnyModifier, '{');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Next 100 Board", "Jump to prev 100 found board", kAnyModifier, '}');

    InstallKeyboardHandler(WitnessKeyboardHandler, "Editor", "Open editor", kAnyModifier, 'e');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Selection", "Open Tetris pieces panel", kAnyModifier, 'x');

    InstallCommandLineHandler(WitnessCLHandler, "-run", "-run", "Runs pre-set experiments.");
    InstallCommandLineHandler(WitnessCLHandler, "-test", "-test", "Basic test with MD heuristic");

    InstallWindowHandler(WitnessWindowHandler);
    InstallMouseClickHandler(
        WitnessClickHandler, static_cast<tMouseEventType>(kMouseMove | kMouseUp | kMouseDown | kMouseDrag));
}

int main(int argc, char *argv[])
{
    InitTetrisPieces();
    GetAllSolutions(allSolutions);
    InitPuzzle();
    InstallHandlers();
    std::cout << "size: " << sizeof(iws) << std::endl;
    RunHOGGUI(argc, argv, 1280, 640);
    return 0;
}
