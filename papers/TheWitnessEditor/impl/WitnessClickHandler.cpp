#include "Driver.h"
#include "Globals.h"

Graphics::point cursor = Graphics::point{};
int cursorViewport = 0;
bool solved = false;
bool gWithReplacement = false;
unsigned gSuggestedLocation = std::numeric_limits<unsigned>::max();

static double MaximizedEntropy(const WitnessRegionConstraint &constraint)
{
    double ret = -1.0;
    if (gWithReplacement)
    {
        for (unsigned i = 0; i < witness.regionConstraintLocations.size(); ++i)
        {
            int x = witness.GetRegionFromX(i);
            int y = witness.GetRegionFromY(i);
            if (constraint == witness.GetRegionConstraint(x, y))
            {
                witness.RemoveRegionConstraint(x, y);
                double e = GetCurrentEntropy(witness);
                if (e > ret && e != inf)
                {
                    ret = e;
                    gSuggestedLocation = i;
                }
                witness.AddRegionConstraint(x, y, constraint);
            }
            else
            {
                auto c = WitnessRegionConstraint(editor.GetRegionConstraint(x, y));
                witness.AddRegionConstraint(x, y, constraint);
                double e = GetCurrentEntropy(witness);
                if (e > ret && e != inf)
                {
                    ret = e;
                    gSuggestedLocation = i;
                }
                witness.RemoveRegionConstraint(x, y);
                witness.AddRegionConstraint(x, y, c);
            }
        }
    }
    else
    {
        for (unsigned i = 0; i < witness.regionConstraintLocations.size(); ++i)
        {
            int x = witness.GetRegionFromX(i);
            int y = witness.GetRegionFromY(i);
            if (witness.GetRegionConstraint(x, y).t == kNoRegionConstraint)
            {
                witness.AddRegionConstraint(x, y, constraint);
                double e = GetCurrentEntropy(witness);
                if (e > ret && e != inf)
                {
                    ret = e;
                    gSuggestedLocation = i;
                }
                witness.RemoveRegionConstraint(x, y);
            }
        }
    }
    return ret;
}

static double MaximizedEntropy(const WitnessPathConstraintType &constraint)
{
    double ret = -1.0;
    if (gWithReplacement)
    {
        for (unsigned i = 1; i < witness.pathConstraintLocations.size() - 1; ++i)
        {
            if (constraint == witness.pathConstraints[i])
            {
                witness.pathConstraints[i] = kNoPathConstraint;
                double e = GetCurrentEntropy(editor);
                if (e > ret && e != inf)
                {
                    ret = e;
                    gSuggestedLocation = i;
                }
                witness.pathConstraints[i] = constraint;
            }
            else
            {
                auto p = witness.pathConstraints[i];
                witness.pathConstraints[i] = constraint;
                double e = GetCurrentEntropy(editor);
                if (e > ret && e != inf)
                {
                    ret = e;
                    gSuggestedLocation = i;
                }
                witness.pathConstraints[i] = p;
            }
        }
    }
    else
    {
        for (unsigned i = 1; i < witness.pathConstraintLocations.size() - 1; ++i)
        {
            if (witness.pathConstraints[i] == kNoPathConstraint)
            {
                witness.pathConstraints[i] = constraint;
                double e = GetCurrentEntropy(editor);
                if (e > ret && e != inf)
                {
                    ret = e;
                    gSuggestedLocation = i;
                }
                witness.pathConstraints[i] = kNoPathConstraint;
            }
        }
    }
    return ret;
}

bool WitnessClickHandler(unsigned long windowID, int viewport, int /*x*/, int /*y*/, point3d p, tButtonType,
                         tMouseEventType e)
{
    switch (viewport)
    {
    case 0:
    {
        if (!drawEditor)
        {
            if (e == kMouseUp)
            {
                if (witness.Click(p, iws))
                {
                    if (witness.GoalTest(iws.ws))
                    {
                        printf("Solved!\n");
                        solved = true;
                    }
                    else
                    {
                        printf("Invalid solution\n");
                        iws.Reset();
                    }
                }
            }
            if (e == kMouseMove)
            {
                witness.Move(p, iws);
                //		if (iws.ws.path.size() > 0)
                //			std::cout << iws.ws.path.back().first << ", " << iws.ws.path.back().second << "\n";
            }
        }
        else
        {
            if (e == kMouseDown)
            {
                if (gSelectedEditorItem != -1 && viewport == cursorViewport)
                {
                    if (gSelectedEditorItem < gRegionConstraintItems.size())
                    {
                        for (unsigned i = 0; i < witness.regionConstraintLocations.size(); ++i)
                        {
                            if (PointInRect(p, witness.regionConstraintLocations[i].second))
                            {
                                int x = witness.GetRegionFromX(i);
                                int y = witness.GetRegionFromY(i);
                                WitnessRegionConstraint constraint = gRegionConstraintItems[gSelectedEditorItem].constraint;
                                if (constraint == witness.GetRegionConstraint(x, y))
                                    witness.RemoveRegionConstraint(x, y);
                                else
                                    witness.AddRegionConstraint(x, y, constraint);
                                break;
                            }
                        }
                    }
                    else
                    {
                        for (unsigned i = 0; i < witness.pathConstraintLocations.size() - 1; ++i)
                        {
                            if (PointInRect(p, witness.pathConstraintLocations[i].second) &&
                                i != puzzleWidth * (puzzleHeight + 1) + (puzzleWidth + 1) * puzzleHeight)
                            {
                                printf("Selected Location: %d\n", i);
                                WitnessPathConstraintType constraint =
                                        gPathConstraintItems[gSelectedEditorItem - gRegionConstraintItems.size()]
                                                .constraint;
                                printf("Selected Constraint: %d\n", constraint);
                                if (constraint == editor.pathConstraints[i])
                                    witness.pathConstraints[i] = kNoPathConstraint;
                                else
                                    witness.pathConstraints[i] = constraint;
                                break;
                            }
                        }
                    }
                    UpdateSolutionIndicies();
                }
            }
        }
        break;
    }
    case 1:
    {
        if (e == kMouseDown)
        {
            bool selected = false;
            for (unsigned i = 0; i < gRegionConstraintItems.size(); i++)
            {
                if (PointInRect(p, {gRegionConstraintItems[i].c, gRegionConstraintItems[i].radius}))
                {
                    gSelectedEditorItem = static_cast<int>(i);
                    if (gSelectedEditorItem == 2)
                        selectTetrisPiece = 1;
                    else if (gSelectedEditorItem == 3)
                        selectTetrisPiece = 2;
                    else
                        selectTetrisPiece = 0;
                    WitnessKeyboardHandler(windowID, kAnyModifier, 'x');
                    selected = true;
                    double e = MaximizedEntropy(gRegionConstraintItems[i].constraint);
                    std::cout << "max entropy: "
                        << ((e == inf) ? "inf" : to_string_with_precision(e, 2)) << std::endl;
                    break;
                }
            }
            for (unsigned i = 0; i < gPathConstraintItems.size(); ++i)
            {
                if (PointInRect(p, {gPathConstraintItems[i].c, gPathConstraintItems[i].radius}))
                {
                    gSelectedEditorItem = static_cast<int>(i + gRegionConstraintItems.size());
                    selected = true;
                    double e = MaximizedEntropy(gPathConstraintItems[i].constraint);
                    std::cout << "max entropy: "
                        << ((e == inf) ? "inf" : to_string_with_precision(e, 2))  << std::endl;
                    break;
                }
            }
            bool selectColor = false;
            for (unsigned i = 0; i < gProvidedColors.size(); ++i)
            {
                if (PointInRect(p, {gProvidedColors[i].c, gProvidedColors[i].radius}))
                {
                    gSelectedColor = i;
                    selectColor = true;
                    printf("Selected Color: %d\n", gSelectedColor);
                    gRegionConstraintItems[0].constraint.c = gProvidedColors[i].color;
                    gRegionConstraintItems[1].constraint.c = gProvidedColors[i].color;
                    break;
                }
            }
            if (!selected && !selectColor)
            {
                gSelectedEditorItem = -1;
            }
            if (PointInRect(p, Graphics::rect{0.49, -0.22, 0.7, -0.14}))
            {
                witness.Reset();
            }
            if (PointInRect(p, Graphics::rect{-0.33, -0.10, -0.225, -0.04}))
            {
                gUseRelativeEntropy ^= true;
                gEntropy = GetCurrentEntropy(witness);
            }
            if (PointInRect(p, Graphics::rect{0.31, -0.10, 0.36, -0.03}))
            {
                switch (gLookahead) {
                    case 0:
                        gLookahead = 1;
                        break;
                    case 1:
                        gLookahead = 2;
                        break;
                    default:
                        gLookahead = 0;
                        break;
                }
                gEntropy = GetCurrentEntropy(witness);
            }
            if (PointInRect(p, Graphics::rect{0.7, -0.10, 0.805, -0.03}))
            {
                gWithReplacement ^= true;
            }
        }
        break;
    }
    case 2:
    {
        if (e == kMouseDown)
        {
            for (unsigned i = 0; i < gTetrisPieces.size(); ++i)
            {
                if (PointInRect(p, {gTetrisPieces[i].c, gTetrisPieces[i].radius}))
                {
                    gSelectedTetrisItem = i + 1;
                    printf("Selected Tetris: %d\n", gSelectedTetrisItem);
                    if (selectTetrisPiece == 1)
                        gRegionConstraintItems[2].constraint.parameter = static_cast<int>(gSelectedTetrisItem);
                    else
                        gRegionConstraintItems[3].constraint.parameter = static_cast<int>(gSelectedTetrisItem);
                    selectTetrisPiece = 0;
                    WitnessKeyboardHandler(windowID, kAnyModifier, 'x');
                }
            }
        }
    }
    default:
        break;
    }
    cursor = p;
    cursorViewport = viewport;
    if (e == kMouseDrag) // ignore movement with mouse button down
        return false;

    // Don't need any other mouse support
    return true;
}
