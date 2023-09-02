#include "Driver.h"
#include "Globals.h"

Graphics::point cursor = Graphics::point{};
int cursorViewport = 0;
bool solved = false;

static void UpdateSolutionIndices() {
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
                                unsigned y = i % puzzleWidth;
                                unsigned x = (i - y) / puzzleWidth;
                                WitnessRegionConstraint constraint = gRegionConstraintItems[gSelectedEditorItem].constraint;
                                if (constraint == witness.GetRegionConstraint(x, y))
                                    witness.ClearConstraint(x, y);
                                else
                                    switch (constraint.t) {
                                        case kSeparation:
                                            witness.AddSeparationConstraint(x, y, constraint.c);
                                            break;
                                        case kStar:
                                            witness.AddStarConstraint(x, y, constraint.c);
                                            break;
                                        case kTetris:
                                            witness.AddTetrisConstraint(x, y, constraint.parameter);
                                            break;
                                        case kNegativeTetris:
                                            witness.AddNegativeTetrisConstraint(x, y, constraint.parameter);
                                            break;
                                        case kTriangle:
                                            witness.AddTriangleConstraint(x, y, constraint.parameter);
                                            break;
                                        case kEraser:
                                            break;
                                        default: // kNoRegionConstraint, kRegionConstraintCount
                                            break;
                                    }
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
                                if (constraint == witness.pathConstraints[i])
                                    witness.pathConstraints[i] = kNoPathConstraint;
                                else
                                    witness.pathConstraints[i] = constraint;
                                break;
                            }
                        }
                    }
                    UpdateSolutionIndices();
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
                    break;
                }
            }
            for (unsigned i = 0; i < gPathConstraintItems.size(); ++i)
            {
                if (PointInRect(p, {gPathConstraintItems[i].c, gPathConstraintItems[i].radius}))
                {
                    gSelectedEditorItem = static_cast<int>(i + gRegionConstraintItems.size());
                    selected = true;
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
            printf("Selected Constraint: %d\n", gSelectedEditorItem);
            if (PointInRect(p, {Graphics::point{0.7, -0.15}, 0.25}))
            {
                witness.Reset();
            }
            if (PointInRect(p, Graphics::rect{-0.33, -0.10, -0.225, -0.04}))
            {
                gUseRelativeEntropy ^= true;
                gEntropy = GetCurrentEntropy(witness);
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
