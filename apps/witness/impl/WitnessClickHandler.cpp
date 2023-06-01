#include "../Driver.h"

Graphics::point cursor;
int cursorViewport = 0;

bool WitnessClickHandler(unsigned long windowID, int viewport, int /*x*/, int /*y*/, point3d p, tButtonType,
                         tMouseEventType e) {
    switch (viewport) {
        case 0: {
            if (!drawEditor) {
                if (e == kMouseUp) {
                    if (witness.Click(p, iws)) {  // found goal
                        if (witness.GoalTest(iws.ws)) {
                            printf("Solved!\n");
                        } else {
                            printf("Invalid solution\n");
                            iws.Reset();
                        }
                    }
                }
                if (e == kMouseMove) {
                    witness.Move(p, iws);
                    //		if (iws.ws.path.size() > 0)
                    //			std::cout << iws.ws.path.back().first << ", " << iws.ws.path.back().second << "\n";
                }
            } else {
                if (e == kMouseDown) {
                    if (gSelectedEditorItem != -1 && viewport == cursorViewport) {
                        for (unsigned i = 0; i < witness.regionConstraintLocations.size(); ++i) {
                            if (PointInRect(p, witness.regionConstraintLocations[i].second)) {
                                unsigned y = i % puzzleWidth;
                                unsigned x = (i - y) / puzzleWidth;
                                if (gEditorItems[gSelectedEditorItem].constraint == witness.constraints[x][y]) {
                                    witness.constraints[x][y] = {.t = kNone, .parameter = 0, .c = Colors::white};
                                } else {
                                    witness.constraints[x][y] = gEditorItems[gSelectedEditorItem].constraint;
                                }
                            }
                        }
                    }
                }
            }
            break;
        }
        case 1: {
            if (e == kMouseDown) {
                bool selected = false;
                for (unsigned i = 0; i < gEditorItems.size(); i++) {
                    if (PointInRect(p, {gEditorItems[i].c, gEditorItems[i].radius})) {
                        gSelectedEditorItem = static_cast<int>(i);
                        if (gSelectedEditorItem == 2) {
                            selectTetrisPiece = 1;
                        } else if (gSelectedEditorItem == 3) {
                            selectTetrisPiece = 2;
                        } else {
                            selectTetrisPiece = 0;
                        }
                        WitnessKeyboardHandler(windowID, kAnyModifier, 'x');
                        selected = true;
                        break;
                    }
                }
                if (!selected) {
                    gSelectedEditorItem = -1;
                }
                printf("Selected Constraint: %d\n", gSelectedEditorItem);
                for (unsigned i = 0; i < gProvidedColors.size(); ++i) {
                    if (PointInRect(p, {gProvidedColors[i].c, gProvidedColors[i].radius})) {
                        gSelectedColor = i;
                        printf("Selected Color: %d\n", gSelectedColor);
                        gEditorItems[0].constraint.c = gProvidedColors[i].color;
                        gEditorItems[1].constraint.c = gProvidedColors[i].color;
                        break;
                    }
                }
            }
            break;
        }
        case 2: {
            if (e == kMouseDown) {
                for (unsigned i = 0; i < gTetrisPieces.size(); ++i) {
                    if (PointInRect(p, {gTetrisPieces[i].c, gTetrisPieces[i].radius})) {
                        gSelectedTetrisItem = i + 1;
                        printf("Selected Tetris: %d\n", gSelectedTetrisItem);
                        if (selectTetrisPiece == 1) {
                            gEditorItems[2].constraint.parameter = static_cast<int>(gSelectedTetrisItem);
                        } else {
                            gEditorItems[3].constraint.parameter = static_cast<int>(gSelectedTetrisItem);
                        }
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
    if (e == kMouseDrag)  // ignore movement with mouse button down
        return false;

    // Don't need any other mouse support
    return true;
}
