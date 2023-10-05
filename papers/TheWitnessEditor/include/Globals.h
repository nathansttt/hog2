//
//  Globals.h
//  The Witness Editor
//
//  Created by Samarium on 2023-06-29.
//  Copyright © 2023 MovingAI. All rights reserved.
//
#ifndef THE_WITNESS_EDITOR_INCLUDE_GLOBALS_H
#define THE_WITNESS_EDITOR_INCLUDE_GLOBALS_H

#include "Witness.h"
#include "WitnessInferenceRule.h"
#include "PuzzleEntropy.h"

constexpr int puzzleWidth = 4;
constexpr int puzzleHeight = 4;
const int minSolutions = 5000;
constexpr double inf = std::numeric_limits<double>::max();
extern unsigned long currBoard;
extern Witness<puzzleWidth, puzzleHeight> witness;
extern Entropy<WitnessState<puzzleWidth, puzzleHeight>, WitnessAction> entropy;
extern Witness<puzzleWidth, puzzleHeight> editor;
extern InteractiveWitnessState<puzzleWidth, puzzleHeight> iws;
extern std::vector<Witness<puzzleWidth, puzzleHeight>> best;
extern std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
extern std::vector<size_t> currentSolutionIndices;
extern size_t gNumSolutions;
extern bool solved;
extern double gEntropy;
extern unsigned gLookahead;
extern bool gWithReplacement;
extern unsigned gSuggestedLocation;

struct RegionConstraintItem {
    WitnessRegionConstraint constraint;
    Graphics::point c;
    float radius{};
};

extern std::vector<RegionConstraintItem> gRegionConstraintItems;

struct PathConstraintItem {
    WitnessPathConstraintType constraint{};
    Graphics::point c;
    float radius{};
};

extern std::vector<PathConstraintItem> gPathConstraintItems;

struct TetrisItem {
    int parameter{};
    Graphics::point c;
    float radius{};
};

extern std::vector<TetrisItem> gTetrisPieces;

struct ColorItem {
    rgbColor color;
    Graphics::point c;
    float radius{};
};

extern std::vector<ColorItem> gProvidedColors;

extern int gSelectedEditorItem;
extern unsigned gSelectedTetrisItem;
extern unsigned gSelectedColor;
extern bool recording;
extern bool parallel;
extern bool drawEditor;
extern int selectTetrisPiece;
extern Graphics::point cursor;
extern int cursorViewport;
extern bool gUseRelativeEntropy;
extern std::vector <std::function<ActionType(
        const SearchEnvironment<WitnessState<puzzleWidth, puzzleHeight>, WitnessAction> &,
        WitnessState<puzzleWidth, puzzleHeight> &, const WitnessAction &)>> gInferenceRules;

inline double GetCurrentEntropy(const Witness<puzzleWidth, puzzleHeight> &env)
{
    return entropy.SetRelative(gUseRelativeEntropy).Calculate(env, iws.ws, gLookahead).value;
}

#endif /* THE_WITNESS_EDITOR_INCLUDE_GLOBALS_H */
