//#include "Graphics.h"
#include "../Driver.h"

std::vector <RegionConstraintItem> gRegionConstraintItems = {
        {{kRegion,         0,  Colors::white}, Graphics::point{-0.75, -0.7}, 0.1},
        {{kStar,           0,  Colors::white}, Graphics::point{-0.5, -0.7},  0.1},
        {{kTetris,         10, Colors::white}, Graphics::point{-0.25, -0.7}, 0.05},
        {{kNegativeTetris, 10, Colors::white}, Graphics::point{-0.05, -0.7}, 0.05},
        {{kTriangle,       1,  Colors::white}, Graphics::point{-0.75, -0.5}, 0.05},
        {{kTriangle,       2,  Colors::white}, Graphics::point{-0.5, -0.5},  0.05},
        {{kTriangle,       3,  Colors::white}, Graphics::point{-0.2, -0.5},  0.05},
};

std::vector <PathConstraintItem> gPathConstraintItems = {{kNoConstraint, Graphics::point{0.1, -0.5}, 0.075},
                                                         {kMustCross,    Graphics::point{0.3, -0.5}, 0.075},
                                                         {kCannotCross,  Graphics::point{0.5, -0.5}, 0.075}};

int gSelectedEditorItem = -1;
unsigned gSelectedTetrisItem = 0;
unsigned gSelectedColor = 0;

std::vector <ColorItem> gProvidedColors = {
        {Colors::white,   Graphics::point{-0.75, -0.15}, 0.1},
        {Colors::red,     Graphics::point{-0.6, -0.15},  0.1},
        {Colors::green,   Graphics::point{-0.45, -0.15}, 0.1},
        {Colors::blue,    Graphics::point{-0.3, -0.15},  0.1},
        {Colors::yellow,  Graphics::point{-0.15, -0.15}, 0.1},
        {Colors::cyan,    Graphics::point{0.0, -0.15},   0.1},
        {Colors::magenta, Graphics::point{0.15, -0.15},  0.1},
        {Colors::black,   Graphics::point{0.3, -0.15},   0.1},
};

Witness <puzzleWidth, puzzleHeight> editor;

static void DrawGameViewport(unsigned long windowID)
{
    Graphics::Display &display = GetContext(windowID)->display;
    witness.Draw(display);
    if (!drawEditor)
    {
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
        if (gSelectedEditorItem != -1 && cursorViewport == 0)
        {
            if (gSelectedEditorItem < gRegionConstraintItems.size())
            {
                for (const auto &location: witness.regionConstraintLocations)
                {
                    Graphics::point p = location.first;
                    if (PointInRect(cursor, location.second))
                    {
                        display.FrameRect({p, 0.1}, Colors::gray, 0.01);
                        witness.DrawRegionConstraint(
                                display, gRegionConstraintItems[gSelectedEditorItem].constraint, p);
                    }
                }
            }
            else
            {
                for (unsigned i = 0; i < witness.pathConstraintLocations.size() - 1; ++i)
                {
                    if (PointInRect(cursor, witness.pathConstraintLocations[i].second) &&
                        i != puzzleWidth * (puzzleHeight + 1) + (puzzleWidth + 1) * puzzleHeight)
                    {
                        display.FrameRect(witness.pathConstraintLocations[i].second, Colors::gray, 0.01);
                    }
                }
            }
        }
    }
}

static void DrawEditorViewport(unsigned long windowID)
{
    if (!drawEditor) return;
    Graphics::Display &display = GetContext(windowID)->display;

    display.FillRect({-1.0f, -1.0f, 1.0f, 1.0f}, Colors::gray);

    display.DrawText("Select a constraint", Graphics::point{-0.8, -0.82}, Colors::black, 0.05);
    for (unsigned i = 0; i < gRegionConstraintItems.size(); ++i)
    {
        RegionConstraintItem &item = gRegionConstraintItems[i];
        editor.DrawRegionConstraint(display, item.constraint, item.c);
        if (i == gSelectedEditorItem)
        {
            display.FrameRect({item.c, item.radius + 0.01f}, Colors::white, 0.01);
        }
    }
    for (unsigned i = 0; i < gPathConstraintItems.size(); ++i)
    {
        PathConstraintItem &item = gPathConstraintItems[i];
        if (i == gSelectedEditorItem - gRegionConstraintItems.size())
            display.FillCircle(item.c, item.radius + 0.01f, Colors::green);
        else
            display.FillCircle(item.c, item.radius + 0.01f, Colors::white);
        display.FillRect(
                {item.c.x - item.radius, item.c.y - 0.025f, item.c.x + item.radius, item.c.y + 0.025f},
                witness.drawColor);
        if (item.constraint == kCannotCross)
            display.FillRect({item.c, 0.025}, witness.backColor);
        else if (item.constraint == kMustCross)
            display.FillNGon(item.c, 0.025, 6, 30, witness.backColor);
    }
    display.DrawText("Select a color", Graphics::point{-0.8, -0.25}, Colors::black, 0.05);
    for (auto &item: gProvidedColors)
    {
        display.FillRect({item.c, 0.05f}, item.color);
    }
    display.DrawText("Clear All", Graphics::point{0.5, -0.15}, Colors::black, 0.05);
}

static void DrawTetrisPiecesViewport(unsigned long windowID)
{
    if (!drawEditor) return;
    Graphics::Display &display = GetContext(windowID)->display;
    display.FillRect({-1.0f, -1.0f, 1.0f, 1.0f}, Colors::bluegray);
    //    printf("selected tetris piece: %d\n", selectTetrisPiece);
    for (auto &item: gTetrisPieces)
    {
        WitnessRegionConstraintType t = kTetris;
        if (selectTetrisPiece == 2)
        {
            t = kNegativeTetris;
        }
        WitnessRegionConstraint constraint = {.t = t, .parameter = item.parameter, .c = Colors::white};
        editor.DrawRegionConstraint(display, constraint, item.c);
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
    {
        DrawEditorViewport(windowID);
        editor = witness;
        break;
    }
    case 2:
    {
        DrawTetrisPiecesViewport(windowID);
        break;
    }
    default:
        break;
    }
    Graphics::Display &display = GetContext(windowID)->display;
    if (gSelectedEditorItem != -1 && viewport == cursorViewport)
    {
        if (gSelectedEditorItem < gRegionConstraintItems.size())
        {
            if (gSelectedEditorItem == 2 || gSelectedEditorItem == 3)
            {
                if (gSelectedTetrisItem != 0)
                {
                    witness.DrawRegionConstraint(
                            display, gRegionConstraintItems[gSelectedEditorItem].constraint, cursor);
                }
            }
            else
            {
                witness.DrawRegionConstraint(display, gRegionConstraintItems[gSelectedEditorItem].constraint, cursor);
            }
        }
    }
}
