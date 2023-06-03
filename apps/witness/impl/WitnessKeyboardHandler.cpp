#include "../Driver.h"
#include "SVGUtil.h"
#include "FileUtil.h"

void WitnessKeyboardHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
    if (drawEditor && (key != 'e') && (key != 'x')) return;
    switch (key)
    {
    case 't':
        ParallelExamine(5);
        //			ExamineMustCross(numRequiredPieces);
        //			w.ClearTetrisConstraints();
        //			ExamineTetris(4);
        //			ExamineMustCrossAndRegions(numRequiredPieces, numSeparationPieces);
        //			ExamineMustCrossAnd3Regions(numRequiredPieces, numSeparationPieces);
        //			ExamineTriangles(6);
        // ExamineTriangles(puzzleHeight*puzzleWidth);
        //			ExamineRegionsAndStars(0);
        break;
    case 'v':
    {
        std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
        GetAllSolutions(witness, allSolutions);
        if (!allSolutions.empty())
        {
            iws.ws = allSolutions[0];
            iws.currState = InteractiveWitnessState<puzzleWidth, puzzleHeight>::kWaitingRestart;
        }
    }
    break;
    case 's':
    {
        Graphics::Display d;
        // d.FillRect({-1, -1, 1, 1}, Colors::darkgray);
        witness.Draw(d);
        witness.Draw(d, iws);
        std::string fname = "/Users/nathanst/Desktop/SVG/witness_";
        int count = 0;
        while (FileExists(fname + std::to_string(count) + ".svg"))
        {
            count++;
        }
        printf("Save to '%s'\n", (fname + std::to_string(count) + ".svg").c_str());
        MakeSVG(d, (fname + std::to_string(count) + ".svg").c_str(), 400, 400, 0, witness.SaveToHashString().c_str());

        {
            int wide, high;
            witness.GetDimensionsFromHashString(witness.SaveToHashString(), wide, high);
        }
    }
    break;
    case 'r':
        recording = !recording;
        break;
    case '\t':
        if (mod != kShiftDown)
            SetActivePort(windowID, (GetActivePort(windowID) + 1) % GetNumPorts(windowID));
        else
        {
            SetNumPorts(windowID, 1 + (GetNumPorts(windowID) % MAXPORTS));
        }
        break;
    case '[':
        if (!best.empty())
        {
            currBoard = (currBoard + (int)(best.size()) - 1) % best.size();
            Load(currBoard);
            printf("%lu of %lu\n", currBoard + 1, best.size());
        }
        break;
    case ']':
        if (!best.empty())
        {
            currBoard = (currBoard + 1) % best.size();
            Load(currBoard); //, numRequiredPieces, numSeparationPieces);
            printf("%lu of %lu\n", currBoard + 1, best.size());
        }
        break;
    case '{':
        if (!best.empty())
        {
            currBoard = (currBoard + (int)(100 * best.size()) - 100) % best.size();
            Load(currBoard);
            printf("%lu of %lu\n", currBoard + 1, best.size());
        }
        break;
    case '}':
        if (!best.empty())
        {
            currBoard = (currBoard + 100) % best.size();
            Load(currBoard); //, numRequiredPieces, numSeparationPieces);
            printf("%lu of %lu\n", currBoard + 1, best.size());
        }
        break;
    case 'o':
        if (iws.ws.path.empty())
        {
            iws.ws.path.emplace_back(0, 0);
            iws.ws.path.emplace_back(0, 1);
            iws.ws.path.emplace_back(1, 1);
        }
        else
        {
            iws.Reset();
        }
        break;
    case 'e':
    { // open editor
        if (!drawEditor)
        {
            drawEditor = true;
            MoveViewport(windowID, 1, {0.0f, -1.0f, 1.0f, 1.0f});
        }
        else
        {
            MoveViewport(windowID, 1, {1.0f, -1.0f, 2.0f, 1.0f});
            MoveViewport(windowID, 2, {1.0f, 0.0f, 2.0f, 2.0f});
            drawEditor = false;
            gSelectedEditorItem = -1;
        }
        break;
    }
    case 'x':
    {
        if (drawEditor)
        {
            if (selectTetrisPiece != 0)
            {
                MoveViewport(windowID, 2, {0.0f, 0.0f, 1.0f, 2.0f});
            }
            else
            {
                MoveViewport(windowID, 2, {1.0f, 0.0f, 2.0f, 2.0f});
            }
        }
        break;
    }
    default:
        break;
    }
}
