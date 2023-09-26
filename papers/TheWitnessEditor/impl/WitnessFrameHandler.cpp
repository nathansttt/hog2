#include "Driver.h"
#include "SolutionUtil.h"

std::vector<RegionConstraintItem> gRegionConstraintItems = {
    {{kSeparation, 0, Colors::cyan}, Graphics::point{-0.75, -0.7}, 0.1},
    {{kStar, 0, Colors::cyan}, Graphics::point{-0.5, -0.7},  0.1},
    {{kTetris, 10, {0.862745098f, 0.6549019608f, 0.0f}}, Graphics::point{-0.25, -0.7}, 0.05},
    {{kNegativeTetris, 10, {0.2196078431f, 0.3607843137f, 0.8705882353f}}, Graphics::point{-0.05, -0.7}, 0.05},
    {{kTriangle, 1, Colors::orange}, Graphics::point{-0.75, -0.5}, 0.05},
    {{kTriangle, 2, Colors::orange}, Graphics::point{-0.5, -0.5}, 0.05},
    {{kTriangle, 3, Colors::orange}, Graphics::point{-0.2, -0.5}, 0.05},
};

std::vector<PathConstraintItem> gPathConstraintItems = {
    {kNoPathConstraint, Graphics::point{0.1, -0.5}, 0.075},
    {kMustCross, Graphics::point{0.3, -0.5}, 0.075},
    {kCannotCross, Graphics::point{0.5, -0.5}, 0.075}
};

bool drawEditor = false;
int gSelectedEditorItem = -1;
unsigned gSelectedTetrisItem = 0;
unsigned gSelectedColor = 0;
static Graphics::point gLastPosition = Graphics::point{-1, -1};
std::vector<size_t> currentSolutionIndices = {};
size_t gNumSolutions = 0;
double gEntropy = 0.0;
bool gUseRelativeEntropy = true;
unsigned gLookahead = 0;

std::vector<ColorItem> gProvidedColors = {
    {Colors::red, Graphics::point{-0.75, -0.2}, 0.1},
    {Colors::orange, Graphics::point{-0.6, -0.2}, 0.1},
    {{0.862745098f, 0.6549019608f, 0.0f}, Graphics::point{-0.45, -0.2}, 0.1},
    {Colors::blue, Graphics::point{-0.3, -0.2}, 0.1},
    {Colors::cyan, Graphics::point{-0.15, -0.2}, 0.1},
    {Colors::magenta, Graphics::point{0.0, -0.2}, 0.1},
    {Colors::pink, Graphics::point{0.15, -0.2}, 0.1},
    {Colors::black, Graphics::point{0.3, -0.2}, 0.1},
};

Witness<puzzleWidth, puzzleHeight> editor;

static size_t GetNumValidSolutions(bool isAdding)
{
    size_t ret = 0;
    if (isAdding)
    {
        for (const size_t &i: currentSolutionIndices)
        {
            if (editor.GoalTest(allSolutions[i]))
                ++ret;
        }
    }
    else
    {
        for (const auto &solution: allSolutions)
        {
            if (editor.GoalTest(solution))
                ++ret;
        }
    }
    return ret;
}

static void DrawGameViewport(unsigned long windowID)
{
    Graphics::Display &display = GetContext(windowID)->display;
    if (!drawEditor)
    {
        witness.Draw(display);
        iws.IncrementTime();
        witness.Draw(display, iws);
        if (solved)
        {
            display.DrawText("Solved!", Graphics::point{1, 1}, Colors::black, 0.075,
                             Graphics::textAlignRight, Graphics::textBaselineBottom);
        }
    }
    else
    {
        editor.Draw(display);
        display.DrawText("# of solutions: ", Graphics::point{0.75, 0.9}, Colors::black, 0.075,
                         Graphics::textAlignRight, Graphics::textBaselineBottom);
        display.DrawText("Entropy: ", Graphics::point{0.7, 1}, Colors::black, 0.07,
                         Graphics::textAlignRight, Graphics::textBaselineBottom);
        display.DrawText("Suggestion: ", Graphics::point{-1, 1}, Colors::black, 0.075,
                         Graphics::textAlignLeft, Graphics::textBaselineBottom);
        if (gSelectedEditorItem != -1 && cursorViewport == 0)
        {
            if (gSelectedEditorItem < gRegionConstraintItems.size())
            {
                bool cursorInPuzzle = false;
                WitnessRegionConstraint constraint = gRegionConstraintItems[gSelectedEditorItem].constraint;
                for (unsigned i = 0; i < editor.regionConstraintLocations.size(); ++i)
                {
                    const auto &location = editor.regionConstraintLocations[i];
                    Graphics::point p = location.first;
                    if (PointInRect(cursor, location.second))
                    {
                        int x = editor.GetRegionFromX(i);
                        int y = editor.GetRegionFromY(i);
                        if (p != gLastPosition)
                        {
                            bool isAdding;
                            if (constraint == editor.GetRegionConstraint(x, y))
                            {
                                editor.RemoveRegionConstraint(x, y);
                                isAdding = false;
                            }
                            else
                            {
                                editor.AddRegionConstraint(x, y, constraint);
                                isAdding = true;
                            }
                            gNumSolutions = GetNumValidSolutions(isAdding);
                            gEntropy = GetCurrentEntropy(editor);
                        }
                        display.FrameRect(location.second, (gNumSolutions > 0) ? Colors::gray : Colors::red, 0.01);
                        editor.DrawRegionConstraint(display, constraint, p);
                        gLastPosition = p;
                        cursorInPuzzle = true;
                        break;
                    }
                }
                if (!cursorInPuzzle)
                    editor.DrawRegionConstraint(display, constraint, cursor);
                if (gSuggestedLocation != std::numeric_limits<unsigned>::max())
                {
                    int x = editor.GetRegionFromX(gSuggestedLocation);
                    int y = editor.GetRegionFromY(gSuggestedLocation);
                    display.DrawText((std::to_string(x) + ", " + std::to_string(y)).c_str(),
                                     Graphics::point{-0.57, 1}, Colors::black, 0.075,
                                     Graphics::textAlignLeft, Graphics::textBaselineBottom);
                    if (editor.GetRegionConstraint(x, y).t == kNoRegionConstraint)
                        display.FrameRect(editor.regionConstraintLocations[gSuggestedLocation].second, Colors::green, 0.01);
                }
                else
                    display.DrawText("None", Graphics::point{-0.57, 1}, Colors::black, 0.075,
                                     Graphics::textAlignLeft, Graphics::textBaselineBottom);
            }
            else
            {
                for (unsigned i = 0; i < editor.pathConstraintLocations.size(); ++i)
                {
                    const auto &location = editor.pathConstraintLocations[i];
                    if (PointInRect(cursor, location.second) &&
                        i != puzzleWidth * (puzzleHeight + 1) + (puzzleWidth + 1) * puzzleHeight)
                    {
                        WitnessPathConstraintType constraint =
                                gPathConstraintItems[gSelectedEditorItem - gRegionConstraintItems.size()]
                                        .constraint;
                        if (location.first != gLastPosition) {
                            bool isAdding = false;
                            if (constraint == editor.pathConstraints[i])
                                editor.pathConstraints[i] = kNoPathConstraint;
                            else
                            {
                                editor.pathConstraints[i] = constraint;
                                if (constraint != kNoPathConstraint)
                                    isAdding = true;
                            }
                            gNumSolutions = GetNumValidSolutions(isAdding);
                            gEntropy = GetCurrentEntropy(editor);
                        }
                        display.FrameRect(location.second, (gNumSolutions > 0) ? Colors::gray : Colors::red, 0.01);
                        gLastPosition = location.first;
                        break;
                    }
                }
                if (gSuggestedLocation != std::numeric_limits<unsigned>::max())
                {
                    auto p = editor.GetPathLocation(gSuggestedLocation);
                    display.DrawText((((p.t == 0) ? "horizontal, " : (p.t == 1) ? "vertical, " : "vertex, ") +
                                      std::to_string(p.x) + ", " +
                                      std::to_string(p.y)).c_str(),
                                     Graphics::point{-0.57, 1}, Colors::black, 0.075,
                                     Graphics::textAlignLeft, Graphics::textBaselineBottom);
                    display.FrameRect(editor.pathConstraintLocations[gSuggestedLocation].second, Colors::green, 0.01);
                }
                else
                    display.DrawText("None", Graphics::point{-0.57, 1}, Colors::black, 0.075,
                                     Graphics::textAlignLeft, Graphics::textBaselineBottom);
            }
            display.DrawText(std::to_string(gNumSolutions).c_str(), Graphics::point{0.9, 0.9}, Colors::black, 0.075,
                             Graphics::textAlignRight, Graphics::textBaselineBottom);
        }
        else
        {
            gLastPosition = Graphics::point{-1, -1};
            display.DrawText(std::to_string(currentSolutionIndices.size()).c_str(), Graphics::point{0.9, 0.9},
                             Colors::black, 0.075, Graphics::textAlignRight, Graphics::textBaselineBottom);
            gEntropy = GetCurrentEntropy(witness);
        }
        display.DrawText((gEntropy != inf) ? to_string_with_precision(gEntropy, 2).c_str() : "inf",
                         Graphics::point{0.9, 1}, Colors::black, 0.075, Graphics::textAlignRight, Graphics::textBaselineBottom);
    }
}

static void FrameLightgrayRect(Graphics::Display& display, int viewport, const Graphics::rect& rect)
{
    if (cursorViewport == viewport && PointInRect(cursor, rect))
        display.FrameRect(rect, Colors::lightgray, 0.01);
}

static void DrawEditorViewport(unsigned long windowID)
{
    if (!drawEditor) return;
    Graphics::Display &display = GetContext(windowID)->display;

    display.FillRect({-1.0f, -1.0f, 1.0f, 1.0f}, Colors::gray);

    display.DrawText("Select a constraint", Graphics::point{-0.8, -0.83}, Colors::black, 0.05);
    for (unsigned i = 0; i < gRegionConstraintItems.size(); ++i)
    {
        RegionConstraintItem &item = gRegionConstraintItems[i];
        editor.DrawRegionConstraint(display, item.constraint, item.c);
        if (i == gSelectedEditorItem)
        {
            if (i < gRegionConstraintItems.size() - 2)
                display.FrameRect({item.c, item.radius + 0.01f}, Colors::white, 0.01);
            else if (i == gRegionConstraintItems.size() - 2)
                display.FrameRect({item.c.x - item.radius - 0.03f, item.c.y - item.radius - 0.01f,
                                   item.c.x + item.radius + 0.03f, item.c.y + item.radius + 0.01f},
                                  Colors::white, 0.01);
            else
                display.FrameRect({item.c.x - item.radius - 0.06f, item.c.y - item.radius - 0.01f,
                                   item.c.x + item.radius + 0.06f, item.c.y + item.radius + 0.01f},
                                  Colors::white, 0.01);
        }
        else if (cursorViewport == 1 && PointInRect(cursor, {item.c, item.radius + 0.01f}))
        {
            if (i < gRegionConstraintItems.size() - 2)
                display.FrameRect({item.c, item.radius + 0.01f}, Colors::lightgray, 0.01);
            else if (i == gRegionConstraintItems.size() - 2)
                display.FrameRect({item.c.x - item.radius - 0.03f, item.c.y - item.radius - 0.01f,
                                   item.c.x + item.radius + 0.03f, item.c.y + item.radius + 0.01f},
                                  Colors::lightgray, 0.01);
            else
                display.FrameRect({item.c.x - item.radius - 0.06f, item.c.y - item.radius - 0.01f,
                                   item.c.x + item.radius + 0.06f, item.c.y + item.radius + 0.01f},
                                  Colors::lightgray, 0.01);
        }
    }
    for (unsigned i = 0; i < gPathConstraintItems.size(); ++i)
    {
        PathConstraintItem &item = gPathConstraintItems[i];
        if (i == gSelectedEditorItem - gRegionConstraintItems.size())
            display.FillRect({item.c, item.radius + 0.01f}, Colors::green);
        else if (cursorViewport == 1 && PointInRect(cursor, {item.c, item.radius + 0.01f}))
            display.FillRect({item.c, item.radius + 0.01f}, Colors::lightgray);
        else
            display.FillRect({item.c, item.radius + 0.01f}, Colors::white);
        display.FillRect({item.c.x - item.radius, item.c.y - 0.025f, item.c.x + item.radius, item.c.y + 0.025f},
                         witness.drawColor);
        if (item.constraint == kCannotCross)
            display.FillRect({item.c, 0.025}, witness.backColor);
        else if (item.constraint == kMustCross)
            display.FillNGon(item.c, 0.025, 6, 30, witness.backColor);
    }
    display.DrawText("Select a color", Graphics::point{-0.8, -0.3}, Colors::black, 0.05);
    Graphics::rect rc;
    for (const auto &item: gProvidedColors)
    {
        rc = {item.c, 0.05f};
        display.FillRect(rc, item.color);
        FrameLightgrayRect(display, 1, rc);
    }
    display.DrawText("Clear All", Graphics::point{0.5, -0.15}, Colors::black, 0.05);
    rc = {0.49, -0.22, 0.7, -0.14};
    FrameLightgrayRect(display, 1, rc);
    display.DrawText("Use relative entropy: ", Graphics::point{-0.8, -0.04}, Colors::black, 0.05);
    display.DrawText((gUseRelativeEntropy) ? "true" : "false", Graphics::point{-0.32, -0.04},
                     (gUseRelativeEntropy) ? Colors::lightgreen : Colors::lightred, 0.05);
    rc = (gUseRelativeEntropy) ? Graphics::rect{-0.33, -0.10, -0.225, -0.03} : Graphics::rect{-0.33, -0.10, -0.21, -0.03};
    FrameLightgrayRect(display, 1, rc);
    display.DrawText("Lookahead steps: ", Graphics::point{-0.15, -0.04}, Colors::black, 0.05);
    display.DrawText(std::to_string(gLookahead).c_str(), Graphics::point{0.27, -0.04}, Colors::black, 0.05);
    rc = {0.26, -0.10, 0.31, -0.03};
    FrameLightgrayRect(display, 1, rc);
    display.DrawText("Replacement: ", Graphics::point{0.38, -0.04}, Colors::black, 0.05);
    display.DrawText((gWithReplacement) ? "true" : "false", Graphics::point{0.71, -0.04},
                     (gWithReplacement) ? Colors::lightgreen : Colors::lightred, 0.05);
    rc = (gWithReplacement) ? Graphics::rect{0.7, -0.10, 0.805, -0.03} : Graphics::rect{0.7, -0.10, 0.82, -0.03};
    FrameLightgrayRect(display, 1, rc);
}

static void DrawTetrisPiecesViewport(unsigned long windowID)
{
    if (!drawEditor) return;
    Graphics::Display &display = GetContext(windowID)->display;
    display.FillRect({-1.0f, -1.0f, 1.0f, 1.0f}, Colors::bluegray);
    for (const auto &item: gTetrisPieces)
    {
        WitnessRegionConstraintType t = (selectTetrisPiece == 2) ? kNegativeTetris : kTetris;
        WitnessRegionConstraint constraint = {.t = t, .parameter = item.parameter, .c = Colors::white};
        editor.DrawRegionConstraint(display, constraint, item.c);
        if (cursorViewport == 2 && PointInRect(cursor, {item.c, item.radius}))
        {
            display.FrameRect({item.c, item.radius}, Colors::lightgray, 0.01);
        }
    }
}

void WitnessFrameHandler(unsigned long windowID, unsigned int viewport, void * /*data*/)
{
    switch (viewport)
    {
        case 0:
            DrawGameViewport(windowID);
            break;
        case 1:
            editor = witness;
            DrawEditorViewport(windowID);
            break;
        case 2:
            DrawTetrisPiecesViewport(windowID);
            break;
        default:
            break;
    }
}
