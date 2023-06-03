//#include "Graphics.h"
#include "../Driver.h"

std::vector<EditorItem> gEditorItems = {
    {{kRegion, 0, Colors::white}, Graphics::point{-0.75, -0.7}, 0.1},
    {{kStar, 0, Colors::white}, Graphics::point{-0.5, -0.7}, 0.1},
    {{kTetris, 10, Colors::white}, Graphics::point{-0.25, -0.7}, 0.05},
    {{kNegativeTetris, 10, Colors::white}, Graphics::point{-0.05, -0.7}, 0.05},
    {{kTriangle, 1, Colors::white}, Graphics::point{-0.75, -0.5}, 0.05},
    {{kTriangle, 2, Colors::white}, Graphics::point{-0.5, -0.5}, 0.05},
    {{kTriangle, 3, Colors::white}, Graphics::point{-0.2, -0.5}, 0.05},
};

int gSelectedEditorItem = -1;
unsigned gSelectedTetrisItem = 0;
unsigned gSelectedColor = 0;

std::vector<ColorItem> gProvidedColors = {
    {Colors::white, Graphics::point{-0.75, -0.15}, 0.1},
    {Colors::red, Graphics::point{-0.6, -0.15}, 0.1},
    {Colors::green, Graphics::point{-0.45, -0.15}, 0.1},
    {Colors::blue, Graphics::point{-0.3, -0.15}, 0.1},
    {Colors::yellow, Graphics::point{-0.15, -0.15}, 0.1},
    {Colors::cyan, Graphics::point{0.0, -0.15}, 0.1},
    {Colors::magenta, Graphics::point{0.15, -0.15}, 0.1},
    {Colors::black, Graphics::point{0.3, -0.15}, 0.1},
};

Witness<puzzleWidth, puzzleHeight> editor;

static void DrawGameViewport(unsigned long windowID)
{
    Graphics::Display &display = GetContext(windowID)->display;
    witness.Draw(display);
    if (!drawEditor)
    {
        iws.IncrementTime();
        witness.Draw(display, iws);
    }
    else
    {
        if (gSelectedEditorItem != -1 && cursorViewport == 0)
        {
            for (const auto &location : witness.regionConstraintLocations)
            {
                Graphics::point p = location.first;
                if (PointInRect(cursor, location.second))
                {
                    display.FrameRect({p, 0.1}, Colors::gray, 0.01);
                    witness.DrawRegionConstraint(display, gEditorItems[gSelectedEditorItem].constraint, p);
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
    for (unsigned i = 0; i < gEditorItems.size(); ++i)
    {
        EditorItem &item = gEditorItems[i];
        editor.DrawRegionConstraint(display, item.constraint, item.c);
        if (i == gSelectedEditorItem)
        {
            display.FrameRect({item.c, item.radius + 0.01f}, Colors::white, 0.01);
        }
    }
    display.DrawText("Select a color", Graphics::point{-0.8, -0.25}, Colors::black, 0.05);
    for (auto &item : gProvidedColors)
    {
        display.FillRect({item.c, 0.05f}, item.color);
    }
}

static void DrawTetrisPiecesViewport(unsigned long windowID)
{
    if (!drawEditor) return;
    Graphics::Display &display = GetContext(windowID)->display;
    display.FillRect({-1.0f, -1.0f, 1.0f, 1.0f}, Colors::bluegray);
    //    printf("selected tetris piece: %d\n", selectTetrisPiece);
    for (auto &item : gTetrisPieces)
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
        if (gSelectedEditorItem == 2 || gSelectedEditorItem == 3)
        {
            if (gSelectedTetrisItem != 0)
            {
                witness.DrawRegionConstraint(display, gEditorItems[gSelectedEditorItem].constraint, cursor);
            }
        }
        else
        {
            witness.DrawRegionConstraint(display, gEditorItems[gSelectedEditorItem].constraint, cursor);
        }
    }
}
