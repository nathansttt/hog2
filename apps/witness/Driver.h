/*
 *  $Id: Driver.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Modified by Junwen Shen on 06/01/23.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */
#include "Common.h"
#include "Timer.h"
#include "Witness.h"

constexpr int puzzleWidth = 4;
constexpr int puzzleHeight = 4;
const int minSolutions = 5000;
extern unsigned long currBoard;
extern Witness<puzzleWidth, puzzleHeight> witness;
extern Witness<puzzleWidth, puzzleHeight> editor;
extern InteractiveWitnessState<puzzleWidth, puzzleHeight> iws;
extern std::vector<Witness<puzzleWidth, puzzleHeight>> best;

struct EditorItem
{
    WitnessRegionConstraint constraint;
    Graphics::point c;
    float radius{};
};

extern std::vector<EditorItem> gEditorItems;

struct TetrisItem
{
    int parameter{};
    Graphics::point c;
    float radius{};
};

extern std::vector<TetrisItem> gTetrisPieces;

struct ColorItem
{
    rgbColor color;
    Graphics::point c;
    float radius{};
};

extern std::vector<ColorItem> gProvidedColors;

extern int gSelectedEditorItem;
extern unsigned gSelectedTetrisItem;
extern unsigned gSelectedColor;

void GetAllSolutions();
int CountSolutions(const Witness<puzzleWidth, puzzleHeight> &w,
    const std::vector<WitnessState<puzzleWidth, puzzleHeight>> &allSolutions, int &len, int limit);
int CountSolutions(const Witness<puzzleWidth, puzzleHeight> &w,
    const std::vector<WitnessState<puzzleWidth, puzzleHeight>> &allSolutions, std::vector<int> &solutions,
    const std::vector<int> &forbidden, int &len, int limit);

void Load(uint64_t which);
void ExamineMustCross(int count);
void ExamineMustCrossAndRegions(int crossCount, int regionCount);
void ExamineMustCrossAnd3Regions(int crossCount, int regionCount);
void ExamineTetris(int count);
void ExamineTriangles(int count);
void ExamineRegionsAndStars(int count);
void ParallelExamine(int count);

template <int puzzleWidth, int puzzleHeight>
void DFS(const Witness<puzzleWidth, puzzleHeight> &w, WitnessState<puzzleWidth, puzzleHeight> &s, // NOLINT
    std::vector<WitnessState<puzzleWidth, puzzleHeight>> &puzzles)
{
    std::vector<WitnessAction> acts;

    if (w.GoalTest(s))
    {
        puzzles.push_back(s);
        return;
    }

    w.GetActions(s, acts);
    for (auto &a : acts)
    {
        w.ApplyAction(s, a);
        DFS(w, s, puzzles);
        w.UndoAction(s, a);
    }
}

template <int puzzleWidth, int puzzleHeight>
void GetAllSolutions(
    const Witness<puzzleWidth, puzzleHeight> &w, std::vector<WitnessState<puzzleWidth, puzzleHeight>> &puzzles)
{
    WitnessState<puzzleWidth, puzzleHeight> s;
    s.Reset();
    Timer t;
    t.StartTimer();
    DFS(w, s, puzzles);
    t.EndTimer();
    printf("%lu solutions found in %1.2fs\n", puzzles.size(), t.GetElapsedTime());
}

template <int puzzleWidth, int puzzleHeight>
void GetAllSolutions(std::vector<WitnessState<puzzleWidth, puzzleHeight>> &puzzles)
{
    Witness<puzzleWidth, puzzleHeight> w;
    GetAllSolutions(w, puzzles);
}

extern bool recording;
extern bool parallel;
extern bool drawEditor;
extern int selectTetrisPiece;
extern Graphics::point cursor;
extern int cursorViewport;

void WitnessWindowHandler(unsigned long windowID, tWindowEventType eType);
void WitnessFrameHandler(unsigned long windowID, unsigned int viewport, void *data);
void WitnessKeyboardHandler(unsigned long windowID, tKeyboardModifier mod, char key);
int WitnessCLHandler(char *argument[], int maxNumArgs);
bool WitnessClickHandler(unsigned long windowID, int viewport, int x, int y, point3d p, tButtonType, tMouseEventType e);
void InstallHandlers();
