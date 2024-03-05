#include "Driver.h"
#include "ExamineUtil.h"
#include "FileUtil.h"
#include "Globals.h"
#include "SolutionUtil.h"
#include "SVGUtil.h"

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
        if (!currentSolutionIndices.empty())
        {
            iws.ws = allSolutions[currentSolutionIndices[0]];
            iws.currState = InteractiveWitnessState<puzzleWidth, puzzleHeight>::kWaitingRestart;
            solved = true;
        }
        break;
    }
    case 's':
    {
        auto ret = std::string(witness);
//        std::cout << witness.SaveToHashString() << std::endl;
//        Witness<puzzleWidth, puzzleHeight>().LoadFromHashString(witness.SaveToHashString());
        std::cout << ret << std::endl;
//        std::istringstream iss(ret);
//        Witness<puzzleWidth, puzzleHeight>().Deserialize(iss);
        break;
    }
    case 'r':
    {
        iws.Reset();
        solved = false;
        break;
    }
    case '\t':
        break;
    case '[':
        if (!best.empty())
        {
            currBoard = (currBoard + (int) (best.size()) - 1) % best.size();
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
            currBoard = (currBoard + (int) (100 * best.size()) - 100) % best.size();
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
    {
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
    }
    case 'e':
    { // open editor
        if (!drawEditor)
        {
            drawEditor = true;
            iws.Reset();
            solved = false;
            UpdateSolutionIndicies();
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
