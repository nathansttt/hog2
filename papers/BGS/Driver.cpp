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
#include "ImprovedBGS2.h"
#include "IncrementalBGS.h"
#include "Map2DEnvironment.h"
#include "ScenarioLoader.h"
#include <map>
#include "IBEX.h"

const int kHeuristic = GraphSearchConstants::kTemporaryLabel;

bool recording = false;

/***** Graph Tests ******/

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

/******* DH Tests ********/


struct dh {
	std::vector<double> depths;
	xyLoc startLoc;
};

class DifferentialHeuristic : public Heuristic<xyLoc> {
public:
	double HCost(const xyLoc &a, const xyLoc &b) const
	{
		//return distance(a, b);
		double v = e->HCost(a, b);
		int a_hash = e->GetStateHash(a);
		int b_hash = e->GetStateHash(b);
		int x  = (a.x+a.y)%(values.size());
		v = std::max(v, fabs(values[x].depths[a_hash]-values[x].depths[b_hash]));
		return v;
	/*	for (int x = 0; x < values.size(); x++)
			v = std::max(v, fabs(values[x].depths[a_hash]-values[x].depths[b_hash]));
		return v;
	*/
	}
	// Doesn't include the baseline heuristic
	double DHCost(const xyLoc &a, const xyLoc &b) const
	{
		//return distance(a, b);
		double v = 0;
		for (int x = 0; x < values.size(); x++)
			v = std::max(v, fabs(values[x].depths[e->GetStateHash(a)]-values[x].depths[e->GetStateHash(b)]));
		return v;
	}
	void Clear()
	{ values.resize(0); }
	void Add()
	{
		std::vector<xyLoc> path;
		TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
		astar.SetStopAfterGoal(false);
		xyLoc where;
		if (values.size() == 0)
		{ // find first open location
		    Map *m = e->GetMap();
			for (int y = 0; y < m->GetMapHeight(); y++)
			{
				for (int x = 0; x < m->GetMapWidth(); x++)
				{
					if (m->GetTerrainType(x, y) == kGround)
					{
						where.x = x;
						where.y = y;
						y = m->GetMapHeight();
						x = m->GetMapWidth();
						astar.InitializeSearch(e, where, where, path);
					}
				}
			}
		}
		else { // get furthest point
			astar.InitializeSearch(e, values[0].startLoc, values[0].startLoc, path);
			for (int x = 1; x < values.size(); x++)
				astar.AddAdditionalStartState(values[x].startLoc);
		}

		while (astar.GetNumOpenItems() > 0)
		{
			where = astar.CheckNextNode();
			astar.DoSingleSearchStep(path);
		}
		
		std::cout << "Adding DH at " << where << "\n";
		dh newDH;
		newDH.startLoc = where;
		newDH.depths.resize(e->GetMaxHash());
		astar.SetStopAfterGoal(false);
		astar.GetPath(e, where, where, path);
		
		for (int x = 0; x < astar.GetNumItems(); x++)
		{
			double cost;
			xyLoc v = astar.GetItem(x).data;
			if (!astar.GetClosedListGCost(v, cost)) {
				//printf("Error reading depth from closed list!\n");
			}
			else {
				int hash = e->GetStateHash(v);
				newDH.depths[hash] = cost;
			}
		}
		values.push_back(newDH);
	}
	MapEnvironment *e;
	std::vector<dh> values;
};

DifferentialHeuristic DH;



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
	InstallCommandLineHandler(MyCLHandler, "-grid", "-grid <map> <scenario> <algorithm>", "Runs the algorithm on the scenario file with Octile heuristic");
	InstallCommandLineHandler(MyCLHandler, "-inconsistent", "-inconsistent <map> <scenario> <algorithm>", "Runs the algorithm on the scenario file with a compressed DH heuristic");
	
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
	ImprovedBGS2<xyLoc, tDirection>  bgs;
	IncrementalBGS<xyLoc, tDirection>  ibex;
	IBEX::IBEX<xyLoc, tDirection, MapEnvironment, false> i(2, 8, 2, false);
	
	ScenarioLoader s(scenario);
	Map *map = new Map(theMap);
	MapEnvironment e(map);
	Timer t;
	std::vector<xyLoc> path;
	std::vector<tDirection> path2;
	
	for (int x = 0; x < s.GetNumExperiments(); x++)
	{
		if (s.GetNthExperiment(x).GetDistance() == 0)
			continue;
		
		xyLoc from = xyLoc(s.GetNthExperiment(x).GetStartX(), s.GetNthExperiment(x).GetStartY());
		xyLoc to = xyLoc(s.GetNthExperiment(x).GetGoalX(), s.GetNthExperiment(x).GetGoalY());

		if (strcmp(algorithm, "astar") == 0)
		{
			t.StartTimer();
			searcher.GetPath(&e, from, to, path);
			t.EndTimer();
			printf("result: %f\t%llu\t%f\n", e.GetPathLength(path), searcher.GetNodesExpanded(), t.GetElapsedTime());
		}

		if (strcmp(algorithm, "bgs") == 0)
		{
			t.StartTimer();
			bgs.GetPath(&e, from, to, &e, path);
			bgs.SetK(1);
			t.EndTimer();
			printf("result: %f\t%llu\t%f\n", e.GetPathLength(path), bgs.GetNodesExpanded(), t.GetElapsedTime());
		}

		if (strcmp(algorithm, "ibex") == 0)
		{
			t.StartTimer();
			ibex.GetPath(&e, from, to, &e, path);
			t.EndTimer();
			printf("result: %f\t%llu\t%f\n", e.GetPathLength(path), ibex.GetNodesExpanded(), t.GetElapsedTime());
		}

		if (strcmp(algorithm, "IBEX") == 0)
		{
			t.StartTimer();
			i.GetPath(&e, from, to, path2);
			t.EndTimer();
			printf("result: %f\t%llu\t%f\n", e.GetPathLength(path), i.GetNodesExpanded(), t.GetElapsedTime());
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
	}
	
	exit(0);
}



void runProblemSetInconsistent(char *theMap, char *scenario, char *algorithm, char *K)
{
	TemplateAStar<xyLoc, tDirection, MapEnvironment> searcher;
	IncrementalBGS<xyLoc, tDirection>  ibex;
	IBEX::IBEX<xyLoc, tDirection, MapEnvironment, false> i(2, 8, 2, false);
	searcher.SetReopenNodes(true);
	
	ScenarioLoader s(scenario);
	Map *map = new Map(theMap);
	MapEnvironment e(map);
	DH.e = &e;
	for (int x = 0; x < 10; x++)
		DH.Add();
	Timer t;
	std::vector<xyLoc> path;
	std::vector<tDirection> path2;
	
	for (int x = 0; x < s.GetNumExperiments(); x++)
	{
		if (s.GetNthExperiment(x).GetDistance() == 0)
			continue;

		xyLoc from = xyLoc(s.GetNthExperiment(x).GetStartX(), s.GetNthExperiment(x).GetStartY());
		xyLoc to = xyLoc(s.GetNthExperiment(x).GetGoalX(), s.GetNthExperiment(x).GetGoalY());

		if (strcmp(algorithm, "astar") == 0)
		{
			searcher.SetHeuristic(&DH);
			t.StartTimer();
			searcher.GetPath(&e, from, to, path);
			t.EndTimer();
			printf("result: %f\t%llu\t%f\n", e.GetPathLength(path), searcher.GetNodesExpanded(), t.GetElapsedTime());
		}

		if (strcmp(algorithm, "bpmx") == 0)
		{
			searcher.SetHeuristic(&DH);
			searcher.SetUseBPMX(1);
			t.StartTimer();
			searcher.GetPath(&e, from, to, path);
			t.EndTimer();
			printf("result: %f\t%llu\t%f\n", e.GetPathLength(path), searcher.GetNodesExpanded(), t.GetElapsedTime());
		}

		if (strcmp(algorithm, "bgs-bpmx") == 0)
		{
			ImprovedBGS2<xyLoc, tDirection>  bgs;
			t.StartTimer();
			bgs.SetUseBPMX();
			bgs.SetK(std::stoi(K));
			bgs.GetPath(&e, from, to, &DH, path);
			t.EndTimer();
			printf("result: %f\t%llu\t%f\n", bgs.GetSolutionCost(), bgs.GetNodesExpanded(), t.GetElapsedTime());
		}

		if (strcmp(algorithm, "bgs") == 0)
		{
			ImprovedBGS2<xyLoc, tDirection>  bgs;
			t.StartTimer();
			bgs.SetK(std::stoi(K));
			bgs.GetPath(&e, from, to, &DH, path);
			t.EndTimer();
			printf("result: %f\t%llu\t%f\n", bgs.GetSolutionCost(), bgs.GetNodesExpanded(), t.GetElapsedTime());
		}

		if (strcmp(algorithm, "ibex") == 0)
		{
			t.StartTimer();
			ibex.GetPath(&e, from, to, &DH, path);
			t.EndTimer();
			printf("result: %f\t%llu\t%f\n", e.GetPathLength(path), ibex.GetNodesExpanded(), t.GetElapsedTime());
		}

		if (strcmp(algorithm, "IBEX") == 0)
		{
			t.StartTimer();
			i.GetPath(&e, &DH, from, to, path2);
			t.EndTimer();
			printf("result: %f\t%llu\t%f\n", e.GetPathLength(from, path2), i.GetNodesExpanded(), t.GetElapsedTime());
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
	}
	
	exit(0);
}

void RunPolygraph(int instanceSize, char *algorithm, char *K)
{
	std::vector<graphState> path;
	IncrementalBGS<graphState, graphMove>  ibex;
	std::vector<graphMove> path2;
	
	g = GraphInconsistencyExamples::GetPolyGraph(instanceSize);
	ge = new GraphEnvironment(g);
	ge->SetDirected(true);
	
	graphState from, to;
	from = 0;
	to = g->GetNumNodes()-1;
	//astar.SetUseBPMX(BPMX?1:0);
	//astar.InitializeSearch(ge, from, to, path);
	Timer t;

	if (strcmp(algorithm, "IBEX") == 0)
	{
		IBEX::IBEX<graphState, graphMove, GraphEnvironment, false> i(2, 5, 2, false);
		t.StartTimer();
		i.GetPath(ge, from, to, path2);
		t.EndTimer();
		printf("%s\t%d\t%llu\t%f\n", algorithm, instanceSize, i.GetNodesExpanded(), t.GetElapsedTime());
	}
	if (strcmp(algorithm, "astar") == 0)
	{
		TemplateAStar<graphState, graphMove, GraphEnvironment> astar;
		astar.SetReopenNodes(true);
		astar.SetHeuristic(&h);
		t.StartTimer();
		astar.GetPath(ge, from, to, path);
		t.EndTimer();
		printf("%s\t%d\t%llu\t%f\n", algorithm, instanceSize, astar.GetNodesExpanded(), t.GetElapsedTime());
	}
	if (strcmp(algorithm, "bgs") == 0)
	{
		ImprovedBGS2<graphState, graphMove>  bgs;
		t.StartTimer();
		bgs.GetPath(ge, from, to, &h, path);
		bgs.SetK(std::stoi(K));
		t.EndTimer();
		printf("%s\t%d\t%llu\t%f\n", algorithm, instanceSize, bgs.GetNodesExpanded(), t.GetElapsedTime());
	}
	if (strcmp(algorithm, "ibex") == 0)
	{
		t.StartTimer();
		ibex.GetPath(ge, from, to, &h, path);
		t.EndTimer();
		printf("%s\t%d\t%llu\t%f\n", algorithm, instanceSize, ibex.GetNodesExpanded(), t.GetElapsedTime());
	}
	
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
	if (strcmp(argument[0], "-inconsistent") == 0)
	{
		if (maxNumArgs > 4)
		{
			runProblemSetInconsistent(argument[1], argument[2], argument[3], argument[4]);
			exit(0);
		}
	}
	if (strcmp(argument[0], "-polygraph") == 0)
	{
		if (maxNumArgs >= 4)
		{
			RunPolygraph(atoi(argument[1]), argument[2], argument[3]);
		}
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

