#include "Driver.h"
#include "ExamineUtil.h"
#include "FileUtil.h"
#include "Globals.h"
#include "SolutionUtil.h"
#include "SVGUtil.h"

unsigned gSolutionIndex = 0;

void WitnessKeyboardHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
    if (drawEditor && key != 'e' && key != 'x' && !std::isdigit(key)) return;
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
            iws.ws = allSolutions[currentSolutionIndices[(gSolutionIndex++) % currentSolutionIndices.size()]];
            iws.currState = InteractiveWitnessState<puzzleWidth, puzzleHeight>::kWaitingRestart;
            solved = true;
        }
        break;
    }
    case 's':
    {
        auto path = std::filesystem::temp_directory_path().string() + std::string("editor.svg");
        Graphics::Display &display = GetContext(windowID)->display;
        MakeSVG(display, path.c_str(), 600, 600, 0);
        std::cout << "Saved to " << path << std::endl;
        break;
    }
    case 'w':
    {
        submitTextToBuffer(std::string(witness).c_str());
        break;
    }
    case 'l':
    {
#ifdef __EMSCRIPTEN__
        auto str = std::string(getTextBuffer());
        std::stringstream iss(str);
        auto w = Witness<puzzleWidth, puzzleHeight>();
        iss >> w;
        witness = w;
#endif
        break;
    }
    case 'r':
    {
        iws.Reset();
        solved = false;
        break;
    }
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
            UpdateSolutionIndices();
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
    case '0':
    {
        if (entropy.ruleSet.disabled.empty())
        {
            entropy.ruleSet.DisableAllRules();
            std::cout << "All inference rules are disabled." << std::endl;
        }
        else
        {
            entropy.ruleSet.EnableAllRules();
            std::cout << "All inference rules are enabled." << std::endl;
        }
        gEntropy = GetCurrentEntropy(witness);
        break;
    }
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    {
        int index = std::stoi(&key) - 1;
        std::cout << static_cast<WitnessInferenceRule>(index);
        if (!entropy.ruleSet.DisableRule(index))
        {
            (void) entropy.ruleSet.EnableRule(index);
            std::cout << " enabled" << std::endl;
        }
        else std::cout << " disabled" << std::endl;
        gEntropy = GetCurrentEntropy(witness);
        break;
    }
    default:
        break;
    }
}
