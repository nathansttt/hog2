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
#include "PuzzleEntropy.h"

constexpr int puzzleWidth = 4;
constexpr int puzzleHeight = 4;
const int minSolutions = 5000;
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
extern double gMuse;

struct RegionConstraintItem {
    WitnessRegionConstraint constraint;
    Graphics::point c;
    float radius{};
};

extern std::vector <RegionConstraintItem> gRegionConstraintItems;

struct PathConstraintItem {
    WitnessPathConstraintType constraint{};
    Graphics::point c;
    float radius{};
};

extern std::vector <PathConstraintItem> gPathConstraintItems;

struct TetrisItem {
    int parameter{};
    Graphics::point c;
    float radius{};
};

extern std::vector <TetrisItem> gTetrisPieces;

struct ColorItem {
    rgbColor color;
    Graphics::point c;
    float radius{};
};

extern std::vector <ColorItem> gProvidedColors;

extern int gSelectedEditorItem;
extern unsigned gSelectedTetrisItem;
extern unsigned gSelectedColor;
extern bool recording;
extern bool parallel;
extern bool drawEditor;
extern int selectTetrisPiece;
extern Graphics::point cursor;
extern int cursorViewport;

#endif /* THE_WITNESS_EDITOR_INCLUDE_GLOBALS_H */
