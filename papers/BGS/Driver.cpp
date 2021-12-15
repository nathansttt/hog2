/*
 * This code demonstrates IBEX variants of BTS and BGS and can be used to duplicate
 * many of the experiments found in that paper.
 */
#include <cstring>
#include "Common.h"
#include "Driver.h"
#include "Timer.h"
#include "TemplateAStar.h"
#include "GraphEnvironment.h"
#include "GraphInconsistencyInstances.h"
#include "ImprovedBGS.h"
#include "Map2DEnvironment.h"
#include "ScenarioLoader.h"

const int kHeuristic = GraphSearchConstants::kTemporaryLabel;

bool recording = false;
Graph *g;
GraphEnvironment *ge;

class GraphDistHeuristic : public Heuristic<graphState> {
public:
	double HCost(const graphState &a, const graphState &b) const
	{
		return g->GetNode(a)->GetLabelL(kHeuristic);
	}
};

GraphDistHeuristic h;

int main(int argc, char* argv[])
{
	setvbuf(stdout, NULL, _IONBF, 0);
	
	InstallHandlers();
	RunHOGGUI(argc, argv, 640, 640);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Test", "Basic test with MD heuristic", kAnyModifier, 't');
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');

//	InstallKeyboardHandler(WeightedTest, "Weighted STP Test", "Test the STP with weights", kNoModifier, 'w');
//	InstallKeyboardHandler(STPTest, "STP Test", "Test the STP PDBs", kNoModifier, 'd');
//	InstallKeyboardHandler(BuildSTP_PDB, "Build STP PDBs", "Build PDBs for the STP", kNoModifier, 'a');

	InstallCommandLineHandler(MyCLHandler, "-polygraph", "-polygraph <size> <algorithm>", "Runs worst-case inconsistency graph");
	InstallCommandLineHandler(MyCLHandler, "-grid", "-polygraph <map> <scenario> <algorithm>", "Runs the algorithm on the scenario file with Octile heuristic");
	
	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler);
}


void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		SetNumPorts(windowID, 1);
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	display.DrawText("No visual display", {0, 0}, Colors::black, 0.1, Graphics::textAlignLeft, Graphics::textBaselineMiddle);
}

void runProblemSet(char *theMap, char *scenario, char *algorithm)
{
	TemplateAStar<xyLoc, tDirection, MapEnvironment> searcher;
	IncrementalBGS<xyLoc, tDirection>  ibex;

	ScenarioLoader s(scenario);
	Map *map = new Map(theMap);
	MapEnvironment e(map);
	Timer t;
	std::vector<xyLoc> path;
	
	for (int x = 0; x < s.GetNumExperiments(); x++)
	{
		xyLoc from = xyLoc(s.GetNthExperiment(x).GetStartX(), s.GetNthExperiment(x).GetStartY());
		xyLoc to = xyLoc(s.GetNthExperiment(x).GetGoalX(), s.GetNthExperiment(x).GetGoalY());

		if (strcmp(algorithm, "astar") == 0)
		{
			t.StartTimer();
			searcher.GetPath(&e, from, to, path);
			t.EndTimer();
		}

		if (strcmp(algorithm, "bgs") == 0)
		{
			t.StartTimer();
			ibex.GetPath(&e, from, to, &e, path);
			t.EndTimer();
		}
		/*
		if (fgreater(fabs(e.GetPathLength(path)-s.GetNthExperiment(x).GetDistance()), 0.01))
		{
			std::cout << "From: " << from << " to " << to << "\n";
			printf("Found solution %d length %f; expected %f difference %f\n", map->GetTerrainType(from.x, from.y), e.GetPathLength(path), s.GetNthExperiment(x).GetDistance(),
				  e.GetPathLength(path)-s.GetNthExperiment(x).GetDistance());
			exit(1);
		}
		*/
		printf("%f\t%llu\t%f\n", e.GetPathLength(path), searcher.GetNodesExpanded(), t.GetElapsedTime());
	}
	
	exit(0);
}



int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-grid") == 0)
	{
		if (maxNumArgs > 3)
		{
			runProblemSet(argument[1], argument[2], argument[3]);
			exit(0);
		}
	}
	if (strcmp(argument[0], "-polygraph") == 0)
	{
		std::vector<graphState> path;
		IncrementalBGS<graphState, graphMove>  ibex;

		int instanceSize = 10;
		if (maxNumArgs >= 3)
			instanceSize = atoi(argument[1]);
		g = GraphInconsistencyExamples::GetPolyGraph(instanceSize);
		ge = new GraphEnvironment(g);
		ge->SetDirected(true);
	
		graphState from, to;
		from = 0;
		to = g->GetNumNodes()-1;
		//astar.SetUseBPMX(BPMX?1:0);
		//astar.InitializeSearch(ge, from, to, path);
		ibex.GetPath(ge, from, to, &h, path);
		printf("%d\t%llu\n", instanceSize, ibex.GetNodesExpanded());
	}
	
	exit(0);
	return 2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 't': break;
		case 'r': recording = !recording; break;
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
			break;
		case '\t':
			break;
		case 'p':
		{}
			break;
		case 'o':
		{}
			break;
		default:
			break;
	}
}

bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	return false;
}

