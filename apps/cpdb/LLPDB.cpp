/*
 *  LLPDB.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/18/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include "LLPDB.h"
#include "Map2DEnvironment.h"
#include "MapSectorAbstraction.h"
#include "Common.h"
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include <deque>
#include "IDAStar.h"
#include "TemplateAStar.h"
#include "GraphCanonicalHeuristic.h"
#include "Propagation.h"
#include "ScenarioLoader.h"
#include "AStarDelay.h"
#include "MNPuzzle.h"
#include "FloydWarshall.h"
using namespace GraphSearchConstants;

void RunBigTest();
void RunTest(ScenarioLoader *sl);
void RunHeuristicTest(ScenarioLoader *sl);
void BuildMapCPDBs();
void BuildWithThreads();
void TestCPDB(char *scenario, char *pdb, int lower, int upper);
void TestMazeCPDBs();
void TestRoomCPDBs();

void BuildScenarioFiles();
void BuildScenarioFiles2();
void TestPDB();
void BuildMazeCPDBs();
void BuildRoomCPDBs();
void TestSTPDiff();
void TestSTPCanonical();
void DrawGraph(Graph *g);
void MoveGraph(Graph *g);

void TestAPSP();
void TestSmallMap();

void RunTest(ScenarioLoader *sl, GraphHeuristic *gcheur, Map *map, float minDist, float maxDist,
			 std::vector<int> &nodes, std::vector<float> &hcosts, std::vector<float> &time,
			 bool octile = false);

GraphCanonicalHeuristic *gch = 0;
GraphMapInconsistentHeuristic *gmih = 0;
GraphMapInconsistentHeuristic *gmih2 = 0;
Graph *stp = 0;
TemplateAStar<graphState, graphMove, GraphEnvironment> visual_astar;


bool mouseTracking;
int px1, py1, px2, py2;
int absType = 0;

std::vector<UnitMapSimulation *> unitSims;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv);
}


/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
void CreateSimulation(int id)
{
	Map *map;
	if (gDefaultMap[0] == 0)
	{
		map = new Map(256, 256);
//		//MakeMaze(map);
//		for (int x = 0; x < 100; x++)
//		{
//			char name[255];
//			sprintf(name, "maze_%d%d%d.map", x/100, (x%100)/10, (x%10));
		MakeMaze(map);
//		BuildRandomRoomMap(map, 8);
		map->scale(512, 512);
		map->setTileSet(kWinter);
//			map->save(name);
//		}
	}
	else {
		map = new Map(gDefaultMap);
		map->scale(512, 512);
		//map->save("/Users/nathanst/Development/hog2/maps/652 maps/map2_big.map");
	}
	
	unitSims.resize(id+1);
	unitSims[id] = new UnitMapSimulation(new MapEnvironment(map));
	unitSims[id]->SetStepType(kMinTime);
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kNoModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');
	
	InstallKeyboardHandler(MyPDBKeyHandler, "load PDB", "load PDB", kNoModifier, 'l');
	InstallKeyboardHandler(MyPDBKeyHandler, "Build PDB", "Build a sliding-tile PDB", kNoModifier, 'b');
	InstallKeyboardHandler(MyPDBKeyHandler, "Test PDB", "Test a sliding-tile PDB", kAnyModifier, 't');
	InstallKeyboardHandler(MyPDBKeyHandler, "Test PDB", "Run Big PDB test", kNoModifier, 'r');
	
	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	
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
		CreateSimulation(windowID);

//		char pdbname[255];
//		sprintf(pdbname, "CPDBs/maze_%d%d%d.map.%d_%d.pdb", 0, 0, 0, 5, 10);
//		gch = new GraphCanonicalHeuristic(pdbname);
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if ((windowID < unitSims.size()) && (unitSims[windowID] == 0))
		return;
	
	if (viewport == 0)
	{
		unitSims[windowID]->StepTime(1.0/30.0);
	}
	if (stp)
		DrawGraph(stp);
	else if (gch)
		gch->OpenGLDraw();
	else
		unitSims[windowID]->OpenGLDraw();
	
	if (gmih)
	{
		gmih->OpenGLDraw();
		visual_astar.OpenGLDraw();
	}
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	strncpy(gDefaultMap, argument[1], 1024);
	return 2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '\t':
			if (mod != kShiftDown)
				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
			else
			{
				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
			}
			break;
		case 'p': unitSims[windowID]->SetPaused(!unitSims[windowID]->GetPaused()); break;
		case 'o':
			if (unitSims[windowID]->GetPaused())
			{
				unitSims[windowID]->SetPaused(false);
				unitSims[windowID]->StepTime(1.0/30.0);
				unitSims[windowID]->SetPaused(true);
			}
			break;
		case ']': absType = (absType+1)%3; break;
		case '[': absType = (absType+4)%3; break;
			//		case '{': unitSim->setPaused(true); unitSim->offsetDisplayTime(-0.5); break;
			//		case '}': unitSim->offsetDisplayTime(0.5); break;
		default:
			break;
	}
}


void MyPDBKeyHandler(unsigned long, tKeyboardModifier mod, char key)
{
	if (key == 'l')
	{
		//TestSmallMap();
		//TestAPSP();

		//TestSTPCanonical();
		
		//delete gch;
		//gch = new GraphCanonicalHeuristic("CPDBs/8room_000.map.pdb");
		gch = new GraphCanonicalHeuristic("CPDBs/8room_002.map.2_10.o.pdb");
		
		//BuildScenarioFiles();
		//BuildScenarioFiles2();
	}
	if (key == 'b')
	{
		BuildMazeCPDBs();
		BuildRoomCPDBs();
		//BuildMapCPDBs();
	}
	if (key == 't')
	{
		if (gmih == 0)
		{
			if (gch)
			{
				gmih = new GraphMapInconsistentHeuristic(gch->GetMap(), gch->GetGraph());
			}
			else {
				Map *m = unitSims[0]->GetEnvironment()->GetMap();
				
				gmih = new GraphMapInconsistentHeuristic(m, GraphSearchConstants::GetGraph(m));
			}
			gmih->UseSmartPlacement(true);
		}
		gmih->AddHeuristic();
		if (mod == kShiftDown)
		{
			MapEnvironment *menv = unitSims[0]->GetEnvironment();
			Map *m = menv->GetMap();
			Graph *g = GraphSearchConstants::GetGraph(m);
			GraphEnvironment *env = new GraphEnvironment(g, gmih);

			std::vector<graphState> path;
			for (int x = 0; x < 1; x++)
			{
				node *n1, *n2;
				n1 = g->GetRandomNode();
				n2 = g->GetRandomNode();
				graphState s1 = n1->GetNum();
				graphState g1 = n2->GetNum();
//				mabs->GetTileFromNode(n1, px1, py1);
//				mabs->GetTileFromNode(n2, px2, py2);
				visual_astar.GetPath(env, s1, g1, path);
				printf("%d\t%d\t%d\t%d\t%1.5f\n", n1->GetLabelL(kMapX), n1->GetLabelL(kMapY),
					   n2->GetLabelL(kMapX), n2->GetLabelL(kMapY), env->GetPathLength(path));
			}
		}
//		TestMazeCPDBs();
//		TestRoomCPDBs();
//		TestPDB();
	}
	if (key == 'r')
	{
		TestRoomCPDBs();
//		if (stp)
//			MoveGraph(stp);
//		else
//			RunBigTest();
	}
}


void TestPDB()
{
	std::vector<std::vector<int> > nodes(26);
	std::vector<std::vector<float> > time(26);
	std::vector<std::vector<float> > hcosts(26);

	int xx = 0;
	char name[255];
	sprintf(name, "scenarios/rooms/8room_%d%d%d.map.txt",  xx/100, (xx/10)%10, xx%10);
	printf("Loading scenario %s\n", name);
	ScenarioLoader *sl = new ScenarioLoader(name);
	Map *map = new Map(sl->GetNthExperiment(0).GetMapName());
	map->scale(sl->GetNthExperiment(0).GetXScale(), sl->GetNthExperiment(0).GetXScale());
	Graph *graph = GraphSearchConstants::GetGraph(map);

	GraphMapInconsistentHeuristic gdh(map, graph);
	gdh.SetNumUsedHeuristics(1000);
	gdh.SetMode(kMax);
	gdh.UseSmartPlacement(true);
	
	int which = 0;
	printf("Solving with %d heuristics\n", gdh.GetNumHeuristics());
	RunTest(sl, &gdh, map, 256, 512, nodes[which], hcosts[which], time[which], true);
	for (int x = 0; x < 25; x++)
	{
		which++;
		for (int y = 0; y < 5; y++)
			gdh.AddHeuristic();
		printf("Solving with %d heuristics\n", gdh.GetNumHeuristics());
		RunTest(sl, &gdh, map, 256, 512, nodes[which], hcosts[which], time[which]);
	}

	double optimal = 0;
	int total = 0;
	for (int x = 0; x < sl->GetNumExperiments(); x++)
	{
		Experiment e = sl->GetNthExperiment(x);
		if ((e.GetDistance() < 256) || (e.GetDistance() > 512))
			continue;
		optimal += e.GetDistance();
		total++;
	}

	printf("# states\tnodes\thcost\ttime\n");
	double averageHCost;
	double averageTime;
	double averageNodes;
	for (unsigned int x = 0; x < 26; x++)
	{
		averageHCost = 0;
		averageTime = 0;
		averageNodes = 0;
		for (unsigned int y = 0; y < nodes[x].size(); y++)
		{
			averageTime += time[x][y];
			averageHCost += hcosts[x][y];
			averageNodes += nodes[x][y];
		}
		averageTime /= nodes[x].size();
		averageHCost /= nodes[x].size();
		averageNodes /= nodes[x].size();
		printf("%d\t%1.2f\t%1.2f\t%1.4f\n", x*5, averageNodes, averageHCost, averageTime);
	}
	printf("%d\t%1.2f\t%1.2f\t%1.4f\n", graph->GetNumNodes(), optimal/total, optimal/total, 0);
}


bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	//TemplateAStar<armAngles, armRotations, RoboticArm> astar;
	return false;
}

void BuildMapCPDBs()
{
	for (int x = 12; x <= 711; x++)
	{
		char name[255];
		sprintf(name, "scenarios/path/AR0%d%d%dSR.map.txt", x/100, (x/10)%10, x%10);
		ScenarioLoader *sl = new ScenarioLoader(name);
		if (sl->GetNumExperiments() == 0)
			continue;
		
		delete gch;
		gch = 0;
		Map *m = new Map(sl->GetNthExperiment(0).GetMapName());
		m->scale(sl->GetNthExperiment(0).GetXScale(), 
				 sl->GetNthExperiment(0).GetYScale());
		gch = new GraphCanonicalHeuristic(m, 1);
		
		char pdbname[255];
		sprintf(pdbname, "CPDBs/AR0%d%d%dSR.map.pdb", x/100, (x/10)%10, x%10);
		gch->save(pdbname);		
	}
}

void BuildMazeCPDBs()
{
	for (int w = 5; w <= 20; w+=15)
	{
		for (int x = 0; x < 5; x++)
		{
			for (int y = 1; y <= 5; y++)
			{
				printf("Building maze %d size %d\n", x, y);
				char name[255];
				sprintf(name, "scenarios/mazes/maze_%d%d%d.map.txt", x/100, (x/10)%10, x%10);
				ScenarioLoader *sl = new ScenarioLoader(name);
				if (sl->GetNumExperiments() == 0)
					continue;
				
				delete gch;
				gch = 0;
				Map *m = new Map(sl->GetNthExperiment(0).GetMapName());
				m->scale(sl->GetNthExperiment(0).GetXScale(), 
						 sl->GetNthExperiment(0).GetYScale());
				gch = new GraphCanonicalHeuristic(m, y, w);
				
				char pdbname[255];
				sprintf(pdbname, "CPDBs/maze_%d%d%d.map.%d_%d.o.pdb", x/100, (x/10)%10, x%10, y, w);
				gch->save(pdbname);		
			}
		}
	}
}

void BuildRoomCPDBs()
{
	for (int w = 5; w <= 20; w+=15)
	{
		for (int x = 0; x < 5; x++)
		{
			for (int y = 1; y <= 5; y++)
			{
				printf("Building room %d\n", x);
				char name[255];
				sprintf(name, "scenarios/rooms/8room_%d%d%d.map.txt",  x/100, (x/10)%10, x%10);
				ScenarioLoader *sl = new ScenarioLoader(name);
				if (sl->GetNumExperiments() == 0)
					continue;
				
				delete gch;
				gch = 0;
				Map *m = new Map(sl->GetNthExperiment(0).GetMapName());
				m->scale(sl->GetNthExperiment(0).GetXScale(), 
						 sl->GetNthExperiment(0).GetYScale());
				gch = new GraphCanonicalHeuristic(m, y, w);
				
				char pdbname[255];
				sprintf(pdbname, "CPDBs/8room_%d%d%d.map.%d_%d.o.pdb", x/100, (x/10)%10, x%10, y, w);
				gch->save(pdbname);		
				printf(pdbname);
				//RunHeuristicTest(sl);
			}
		}
	}
}

void TestMazeCPDBs()
{
	TestCPDB("scenarios/mazes/maze_%d%d%d.map.txt",
			 "CPDBs/maze_%d%d%d.map.%d_%d.o.pdb",
			 512, 768);
//	TestCPDB("scenarios/mazes/maze_%d%d%d.map.txt",
//			 "CPDBs/maze_%d%d%d.map.%d_%d.pdb",
//			 756, 768);
}

void TestRoomCPDBs()
{
	TestCPDB("scenarios/rooms/8room_%d%d%d.map.txt",
			 "CPDBs/8room_%d%d%d.map.%d_%d.o.pdb",
			 256, 512);
//	TestCPDB("scenarios/rooms/8room_%d%d%d.map.txt",
//			 "CPDBs/8room_%d%d%d.map.%d_%d.pdb",
//			 500, 512);
}

void TestCPDB(char *scenario, char *pdb, int lower, int upper)
{
	std::vector<std::vector<float> > times(8);
	std::vector<std::vector<float> > hcosts(8);
	std::vector<std::vector<int> > nodes(8);
	
	for (int w = 10; w <= 20; w*=20)
	{
		for (int x = 0; x <= 4; x++)
		{
			char name[255];
			sprintf(name, scenario,  x/100, (x/10)%10, x%10);
			printf("Loading scenario %s\n", name);
			ScenarioLoader *sl = new ScenarioLoader(name);
			if (sl->GetNumExperiments() == 0)
				continue;

			for (int y = 1; y <= 5; y++)
			{
				printf("Loading pdb %d\n", y);
				char pdbname[255];
				sprintf(pdbname, pdb, x/100, (x/10)%10, x%10, y, w);
				if (gch == 0)
					gch = new GraphCanonicalHeuristic(pdbname);
				else
					gch->load(pdbname);
			
//				freopen("/Users/nathanst/Desktop/CH.txt","a+",stdout);
				RunTest(sl, gch, gch->GetMap(), lower, upper, nodes[y], hcosts[y], times[y]);
			}
//			freopen("/Users/nathanst/Desktop/octile.txt","a+",stdout);
//			RunTest(sl, gch, gch->GetMap(), lower, upper, nodes[0], hcosts[0], times[0], true);
//			for (int y = 1; y <= 1; y++)
//			{
//				GraphMapInconsistentHeuristic gdh(gch->GetMap(), gch->GetGraph());
//				gdh.SetNumUsedHeuristics(1000);
//				gdh.SetMode(kMax);
//				gdh.UseSmartPlacement((y==0)?false:true);
//				for (int t = 0; t < w; t++)
//					gdh.AddHeuristic();
//				freopen("/Users/nathanst/Desktop/DH.txt","a+",stdout);
//				RunTest(sl, &gdh, gch->GetMap(), lower, upper, nodes[6+y], hcosts[6+y], times[6+y]);
//			}
			delete sl;
		}
	}
	printf("# states\tnodes\thcost\ttime\n");
	std::vector<double> averageHCost(8);
	std::vector<double> averageTime(8);
	std::vector<double> averageNodes(8);
	for (unsigned int x = 0; x < 8; x++)
	{
		for (unsigned int y = 0; y < nodes[x].size(); y++)
		{
			averageTime[x] += times[x][y];
			averageHCost[x] += hcosts[x][y];
			averageNodes[x] += nodes[x][y];
		}
		averageTime[x] /= times[x].size();
		averageHCost[x] /= hcosts[x].size();
		averageNodes[x] /= nodes[x].size();
		printf("%d\t%1.2f\t%1.2f\t%1.4f\n", x, averageNodes[x], averageHCost[x], averageTime[x]);
	}
}

void RunTest(ScenarioLoader *sl, GraphHeuristic *gcheur, Map *map, float minDist, float maxDist,
			 std::vector<int> &nodes, std::vector<float> &hcosts, std::vector<float> &time, bool octile)
{
	Graph *g = gcheur->GetGraph();
	GraphMapHeuristic gmh(map, g);
	GraphEnvironment env1(g, gcheur);
	GraphEnvironment env2(g, &gmh);
	
	std::vector<graphState> thePath;
	TemplateAStar<graphState, graphMove, GraphEnvironment> astar;
	astar.SetUseBPMX(false);
	for (int x = 0; x < sl->GetNumExperiments(); x++)
	{
		Experiment e = sl->GetNthExperiment(x);
		if ((e.GetDistance() < minDist) || (e.GetDistance() > maxDist))
			continue;
		env1.SetDirected(false);
		
		graphState start, goal;
		start = map->getNodeNum(e.GetStartX(), e.GetStartY());
		goal = map->getNodeNum(e.GetGoalX(), e.GetGoalY());
		
		if (octile)
			hcosts.push_back(env2.HCost(start, goal));
		else
			hcosts.push_back(env1.HCost(start, goal));
		
		Timer t;
		t.startTimer();
		if (octile)
			astar.GetPath(&env2, start, goal, thePath);
		else
			astar.GetPath(&env1, start, goal, thePath);
		time.push_back(t.endTimer());
		nodes.push_back(astar.GetNodesExpanded());
	}
}


void RunBigTest()
{
	for (int y = 200; y < 2000; y+=200)
	{
		printf("------- %d -------\n", y);
		for (int x = 0; x <= 5; x++) // only have 2 rooms right now
		{
			printf("------- %d -------\n", x);
			char name[255];
			sprintf(name, "scenarios/rooms/8room_%d%d%d.map.txt", x/100, (x/10)%10, x%10);
			//sprintf(name, "scenarios/mazes/maze_%d%d%d.map.txt", x/100, (x/10)%10, x%10);
			//sprintf(name, "scenarios/path/AR0%d%d%dSR.map.txt", x/100, (x/10)%10, x%10);
			ScenarioLoader *sl = new ScenarioLoader(name);
			if (sl->GetNumExperiments() == 0)
				continue;
			
			char pdbname[255];
			//sprintf(pdbname, "CPDBs/AR0%d%d%dSR.map.pdb", x/100, (x/10)%10, x%10);
			//sprintf(pdbname, "CPDBs/maze_%d%d%d.map.%d.pdb", x/100, (x/10)%10, x%10, y);
			//sprintf(pdbname, "CPDBs/8room_%d%d%d.map.pdb", x/100, (x/10)%10, x%10);
			sprintf(pdbname, "CPDBs/8room_%d%d%d.map.%d.pdb", x/100, (x/10)%10, x%10, y);
			
			delete gch;
			gch = 0;
			gch = new GraphCanonicalHeuristic(pdbname);

			Graph *g = gch->GetGraph();
			//GraphMapInconsistentHeuristic::HN = 1+y*y/(g->GetNumNodes());
			delete gmih;
			delete gmih2;
			gmih = new GraphMapInconsistentHeuristic(gch->GetMap(), gch->GetGraph());
			gmih->UseSmartPlacement(true);
			gmih2 = new GraphMapInconsistentHeuristic(gch->GetMap(), gch->GetGraph());
			gmih2->UseSmartPlacement(false);
			for (int t = 0; t < gch->GetNumEntries()/g->GetNumNodes(); t++)
			{
				gmih->AddHeuristic();
				gmih2->AddHeuristic();
			}
			gmih->SetNumUsedHeuristics(gch->GetNumEntries()/g->GetNumNodes());
			gmih2->SetNumUsedHeuristics(gch->GetNumEntries()/g->GetNumNodes());
			gmih->SetMode(kMax);
			gmih2->SetMode(kMax);
			
			//RunHeuristicTest(sl);
			//RunTest(sl);
		}
	}
}

void RunTest(ScenarioLoader *sl)
{
	Graph *g = gch->GetGraph();

	GraphMapHeuristic gmh(gch->GetMap(), gch->GetGraph());
	
	GraphEnvironment *env1 = new GraphEnvironment(g, gch);
	GraphEnvironment *env2 = new GraphEnvironment(g, &gmh);
	GraphEnvironment *env3 = new GraphEnvironment(g, gmih);
	GraphEnvironment *env4 = new GraphEnvironment(g, gmih2);

	env1->SetDirected(true);
	env2->SetDirected(true);
	env3->SetDirected(true);
	env4->SetDirected(true);
	
	double sum1 = 0, sum2 = 0, sum3 = 0, sum4 = 0, sum5 = 0;
	double cnt = 0;
	for (int x = 0; x < sl->GetNumExperiments(); x++)
	{
		Experiment e = sl->GetNthExperiment(x);
		if (e.GetDistance() < 500)
			continue;
//		printf("%s\t%1.2f\t", e.GetMapName(), e.GetDistance());

		graphState start, goal;

		start = gch->GetMap()->getNodeNum(e.GetStartX(), e.GetStartY());
		goal = gch->GetMap()->getNodeNum(e.GetGoalX(), e.GetGoalY());
		
		std::vector<graphState> thePath;
		AStarDelay delay(0);
		TemplateAStar<graphState, graphMove, GraphEnvironment> astar;
		
		//astar.GetPath(env2, g, start, goal, thePath);
		astar.SetUseBPMX(false);
		astar.GetPath(env2, start, goal, thePath);
//		printf("%d\t",
//			   astar.GetNodesExpanded());//, p->GetNodesReopened());
		sum1 += astar.GetNodesExpanded();
		
		//astar.GetPath(env1, g, start, goal, thePath);
		astar.SetUseBPMX(false);
		astar.GetPath(env1, start, goal, thePath);
//		printf("%d\t",
//			   astar.GetNodesExpanded());
		sum2 += astar.GetNodesExpanded();


//		astar.SetUseBPMX(true);
//		astar.GetPath(env1, start, goal, thePath);
//		printf("%d\t",
//			   astar.GetNodesExpanded());
//		sum3 += astar.GetNodesExpanded();

		astar.SetUseBPMX(false);
		astar.GetPath(env4, start, goal, thePath);
//		printf("%ld\t",
//			   delay.GetNodesExpanded());
		sum4 += astar.GetNodesExpanded();

		astar.SetUseBPMX(false);
		astar.GetPath(env3, start, goal, thePath);
//		printf("%d\n",
//			   astar.GetNodesExpanded());
		sum5 += astar.GetNodesExpanded();
		
		sum3 += thePath.size();
		cnt++;
	}
	printf("##\t\t\t\t\t%f\t%f\t%f\t%f\t%f\n", sum1/cnt, sum2/cnt, sum4/cnt, sum5/cnt, sum3/cnt);
	delete env1;
	delete env2;
	delete env3;
	delete env4;
}

void *doThreadedModel2(void *data);
void *doThreadedModel1(void *data);

void BuildWithThreads()
{
	pthread_t one, two;
	pthread_create(&one, NULL, doThreadedModel1, (void**)0);
	pthread_create(&two, NULL, doThreadedModel2, (void**)0);
}

void *doThreadedModel1(void *)
{
	//BuildScenarioFiles();
	return 0;
}

void *doThreadedModel2(void *)
{
	//BuildScenarioFiles2();
	return 0;
}

void BuildScenarioFiles()
{
	std::vector<graphState> thePath;
	char name[255];
	for (int x = 13; x < 100; x++)
	{
		sprintf(name, "scenarios/mazes/maze_%d%d%d.map.txt", x/100, (x/10)%10, x%10);
		ScenarioLoader *sl = new ScenarioLoader(name);	
		std::vector<int> counts(256);
		int total = 10*64;
		double len;
		sprintf(name, "maps/mazes/maze_%d%d%d.map", x/100, (x/10)%10, x%10);
		Map *m;
		m = new Map(name);
		m->scale(512, 512);
		Graph *g = GraphSearchConstants::GetGraph(m);
		
		TemplateAStar<graphState, graphMove, GraphEnvironment> astar;

		graphState start, goal;
		GraphMapHeuristic gmh(m, g);
		GraphEnvironment *env2 = new GraphEnvironment(g, &gmh);
		astar.SetUseBPMX(false);

		while (total > 0)
		{
			bool found = false;
			while (!found)
			{
				astar.GetPath(env2, start = g->GetRandomNode()->GetNum(),
							  goal = g->GetRandomNode()->GetNum(), thePath);
				len = env2->GetPathLength(thePath);
				if ((len > 512) && (len <= 768) && (counts[(int)(len/4)] < 10))
				{
					found = true;
					continue;
				}
				while (len > 512)
				{
					len -= env2->GCost(thePath[thePath.size()-1], thePath[thePath.size()-2]);
					thePath.pop_back();
					//len = env2->GetPathLength(thePath);
					if ((len > 512) && (len <= 768) && (counts[(int)(len/4)] < 10))
					{
						found = true;
						break;
					}
				}
			}
			counts[(int)(len/4)]++;
			printf("%s [%d left] Found path in bucket %d length %1.2f (%ld, %ld) to (%ld, %ld)\n",
				   name, total, (int)(len/4), len,
				   g->GetNode(thePath[0])->GetLabelL(kMapX),g->GetNode(thePath[0])->GetLabelL(kMapY),
				   g->GetNode(thePath.back())->GetLabelL(kMapX),g->GetNode(thePath.back())->GetLabelL(kMapY));
			total--;
			Experiment exp(g->GetNode(thePath[0])->GetLabelL(kMapX),g->GetNode(thePath[0])->GetLabelL(kMapY),
						   g->GetNode(thePath.back())->GetLabelL(kMapX),g->GetNode(thePath.back())->GetLabelL(kMapY),
						   512,512,(int)(len/4),len,string(name));
			sl->AddExperiment(exp);
		}
		sprintf(name, "scenarios/mazes/maze_%d%d%d.map.txt", x/100, (x/10)%10, x%10);
		sl->Save(name);

		delete sl;
		delete g;
		delete m;
		delete env2;
	}
}

void BuildScenarioFiles2()
{
	std::vector<graphState> thePath;
	char name[255];
	for (int x = 0; x < 100; x++)
	{
		ScenarioLoader *sl = new ScenarioLoader();	
		std::vector<int> counts(128);
		int total = 10*128;
		double len;
		sprintf(name, "maps/rooms/8room_%d%d%d.map", x/100, (x/10)%10, x%10);
		Map *m;
		m = new Map(name);
		m->scale(512, 512);
		Graph *g = GraphSearchConstants::GetGraph(m);
		
		TemplateAStar<graphState, graphMove, GraphEnvironment> astar;
		
		graphState start, goal;
		GraphMapHeuristic gmh(m, g);
		GraphEnvironment *env2 = new GraphEnvironment(g, &gmh);
		astar.SetUseBPMX(false);
		
		while (total > 0)
		{
			bool found = false;
			while (!found)
			{
				astar.GetPath(env2, start = g->GetRandomNode()->GetNum(),
							  goal = g->GetRandomNode()->GetNum(), thePath);
				len = env2->GetPathLength(thePath);
				if (thePath.size() == 0)
					continue;
				if ((len > 0) && (len <= 512) && (counts[(int)(len/4)] < 10))
				{
					found = true;
					continue;
				}
				while (thePath.size() > 2)
				{
					len -= env2->GCost(thePath[thePath.size()-1], thePath[thePath.size()-2]);
					thePath.pop_back();
					//len = env2->GetPathLength(thePath);
					if ((len > 0) && (len <= 512) && (counts[(int)(len/4)] < 10))
					{
						found = true;
						break;
					}
				}
			}
			counts[(int)(len/4)]++;
			printf("%s [%d left] Found path in bucket %d length %1.2f (%ld, %ld) to (%ld, %ld)\n",
				   name, total, (int)(len/4), len,
				   g->GetNode(thePath[0])->GetLabelL(kMapX),g->GetNode(thePath[0])->GetLabelL(kMapY),
				   g->GetNode(thePath.back())->GetLabelL(kMapX),g->GetNode(thePath.back())->GetLabelL(kMapY));
			total--;
			Experiment exp(g->GetNode(thePath[0])->GetLabelL(kMapX),g->GetNode(thePath[0])->GetLabelL(kMapY),
						   g->GetNode(thePath.back())->GetLabelL(kMapX),g->GetNode(thePath.back())->GetLabelL(kMapY),
						   512,512,(int)(len/4),len,string(name));
			sl->AddExperiment(exp);
		}
		sprintf(name, "scenarios/rooms/8room_%d%d%d.map.txt", x/100, (x/10)%10, x%10);
		sl->Save(name);
		
		delete sl;
		delete g;
		delete m;
		delete env2;
	}
}

void TestSTPDiff()
{
	std::vector<std::pair<int, int> > problems;
	MNPuzzle p(3, 3);
	Graph *g = p.GetGraph();
	while (problems.size() < 3000)
	{
		int a = g->GetRandomNode()->GetNum();
		int b = g->GetRandomNode()->GetNum();
		MNPuzzleState s1(3, 3), s2(3, 3);
		p.GetStateFromHash(s1, a);
		p.GetStateFromHash(s2, b);
		if ((p.GetParity(s1) == p.GetParity(s2)) && (p.GetParity(s2) == 0) && (p.HCost(s1, s2) > 12))
			problems.push_back(std::pair<int, int>(a, b));
	}
	stp = g;
	GraphPuzzleDistanceHeuristic gpdh(p, g, 0);
	GraphEnvironment ge(g, &gpdh);
	gpdh.UseSmartPlacement(true);
	ge.SetDirected(false);
	TemplateAStar<graphState, graphMove, GraphEnvironment> astar;
	std::vector<graphState> thePath;

	//if (0)
	for (int x = 0; x <= 25; x++)
	{
		int nodes = 0;
		int touched = 0;
		double hvalues = 0;
		double time = 0;
		Timer t;
		for (unsigned int y = 0; y < problems.size(); y++)
		{
			graphState start, goal;
			start = problems[y].first;
			goal = problems[y].second;
			t.startTimer();
			astar.GetPath(&ge, start, goal, thePath);
			time += t.endTimer();
			nodes += astar.GetNodesExpanded();
			touched += astar.GetNodesTouched();
			hvalues += ge.HCost(start, goal);
			//printf("%d   %d\n", y, astar.GetNodesExpanded());
//			if (0 == y%100)
//				printf("%d ", y);
		}
		printf("%d\t%1.2f\t%1.2f\t%1.2f\t%1.4f\n", gpdh.GetNumHeuristics(), (double)nodes/problems.size(), (double)touched/problems.size(), (double)hvalues/problems.size(), time/problems.size());
		if (x == 20)
			break;
		for (int y = 0; y < 10; y++)
		{
			node *n = 0;
			while (1)
			{
				n = g->GetRandomNode();
				int b = n->GetNum();
				MNPuzzleState s1(3, 3);
				p.GetStateFromHash(s1, b);
				if (p.GetParity(s1) == 0)
					break;
			}
			gpdh.AddHeuristic(n);
		}
//		if (gpdh.GetNumHeuristics() == 0)
//			gpdh.AddHeuristic();
	}
}

void TestSTPCanonical()
{
	std::vector<std::pair<int, int> > problems;
	MNPuzzle p(3, 3);
	Graph *g = p.GetGraph();

	while (problems.size() < 3000)
	{
		int a = g->GetRandomNode()->GetNum();
		int b = g->GetRandomNode()->GetNum();
		MNPuzzleState s1(3, 3), s2(3, 3);
		p.GetStateFromHash(s1, a);
		p.GetStateFromHash(s2, b);
		if ((p.GetParity(s1) == p.GetParity(s2)) && (p.GetParity(s2) == 0) && (p.HCost(s1, s2) > 12))
			problems.push_back(std::pair<int, int>(a, b));
	}
	
	stp = g;
	GraphPuzzleDistanceHeuristic gpdh(p, g, 0);
	GraphCanonicalHeuristic gstdpch(g, &gpdh, 2, 10);
	GraphEnvironment ge(g, &gstdpch);
	ge.SetDirected(false);

	TemplateAStar<graphState, graphMove, GraphEnvironment> astar;
	std::vector<graphState> thePath;
	astar.SetUseBPMX(true);
	
	{
		int nodes = 0;
		int touched = 0;
		double hvalues = 0;
		double time = 0;
		Timer t;
		for (unsigned int y = 0; y < problems.size(); y++)
		{
			graphState start, goal;
			start = problems[y].first;
			goal = problems[y].second;
			t.startTimer();
			astar.GetPath(&ge, start, goal, thePath);
			time += t.endTimer();
			nodes += astar.GetNodesExpanded();
			touched += astar.GetNodesTouched();
			hvalues += ge.HCost(start, goal);
			//printf("%d   %d\n", y, astar.GetNodesExpanded());
			//			if (0 == y%100)
			//				printf("%d ", y);
		}
		printf("%d\t%1.2f\t%1.2f\t%1.2f\t%1.4f\n", gpdh.GetNumHeuristics(), (double)nodes/problems.size(), (double)touched/problems.size(), (double)hvalues/problems.size(), time/problems.size());
	}
}


void MoveGraph(Graph *g)
{
//	for (int i = 0 ; i < g->GetNumEdges() ; i++)
//	{
//		edge *e = g->GetEdge(i);
//
//		double vx = g->GetNode(e->getTo())->GetLabelF(GraphSearchConstants::kXCoordinate) - g->GetNode(e->getFrom())->GetLabelF(GraphSearchConstants::kXCoordinate);
//		double vy = g->GetNode(e->getTo())->GetLabelF(GraphSearchConstants::kYCoordinate) - g->GetNode(e->getFrom())->GetLabelF(GraphSearchConstants::kYCoordinate);
//		double vz = g->GetNode(e->getTo())->GetLabelF(GraphSearchConstants::kZCoordinate) - g->GetNode(e->getFrom())->GetLabelF(GraphSearchConstants::kZCoordinate);
//		double len = sqrt(vx * vx + vy * vy + vz * vz);
//		len = (len == 0) ? .0001 : len;
//		double f = (e->GetWeight()/300.0 - len) / (len * 3);
//		double dx = f * vx;
//		double dy = f * vy;
//		double dz = f * vz;
//		//kTemporaryLabel
//		nodes[e.to].dx += dx;
//		nodes[e.to].dy += dy;
//		nodes[e.from].dx += -dx;
//		nodes[e.from].dy += -dy;
//	}
//	
//	for (int i = 0 ; i < nnodes ; i++) {
//		Node n1 = nodes[i];
//		double dx = 0;
//		double dy = 0;
//		
//		for (int j = 0 ; j < nnodes ; j++) {
//			if (i == j) {
//				continue;
//			}
//			Node n2 = nodes[j];
//			double vx = n1.x - n2.x;
//			double vy = n1.y - n2.y;
//			double len = vx * vx + vy * vy;
//			if (len == 0) {
//				dx += Math.random();
//				dy += Math.random();
//			} else if (len < 100*100) {
//				dx += vx / len;
//				dy += vy / len;
//			}
//		}
//		double dlen = dx * dx + dy * dy;
//		if (dlen > 0) {
//			dlen = Math.sqrt(dlen) / 2;
//			n1.dx += dx / dlen;
//			n1.dy += dy / dlen;
//		}
//	}
//	
//	Dimension d = getSize();
//	for (int i = 0 ; i < nnodes ; i++) {
//		Node n = nodes[i];
//		if (!n.fixed) {
//			n.x += Math.max(-5, Math.min(5, n.dx));
//			n.y += Math.max(-5, Math.min(5, n.dy));
//		}
//		if (n.x < 0) {
//			n.x = 0;
//		} else if (n.x > d.width) {
//			n.x = d.width;
//		}
//		if (n.y < 0) {
//			n.y = 0;
//		} else if (n.y > d.height) {
//			n.y = d.height;
//		}
//		n.dx /= 2;
//		n.dy /= 2;
//	}
//	
//	
//	
//	
//	for (int t = 0; t < g->GetNumNodes(); t++)
//	{
//		node *n = g->GetNode(t);
//		neighbor_iterator ni = n->getNeighborIter();
//		for (long tmp = n->nodeNeighborNext(ni); tmp != -1; tmp = n->nodeNeighborNext(ni))
//		{
//			double x, y, z;
//			x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
//			y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
//			z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
//			
//			double x1, y1, z1;
//			node *nb = g->GetNode(tmp);
//			x1 = nb->GetLabelF(GraphSearchConstants::kXCoordinate);
//			y1 = nb->GetLabelF(GraphSearchConstants::kYCoordinate);
//			z1 = nb->GetLabelF(GraphSearchConstants::kZCoordinate);
//
//			double len =sqrt((x-x1)*(x-x1) + (y-y1)*(y-y1) + (z-z1)*(z-z1));
////			if (1)
////			{
////				// now move n to be 1/600 away from nb!
////				n->SetLabelF(GraphSearchConstants::kXCoordinate, 0.5*x + 0.5*x1);
////				n->SetLabelF(GraphSearchConstants::kYCoordinate, 0.5*y + 0.5*y1);
////				n->SetLabelF(GraphSearchConstants::kZCoordinate, 0.5*z + 0.5*z1);
////			}
////			else {
////				n->SetLabelF(GraphSearchConstants::kXCoordinate, x+0.5*(x-x1));
////				n->SetLabelF(GraphSearchConstants::kYCoordinate, y+0.5*(y-y1));
////				n->SetLabelF(GraphSearchConstants::kZCoordinate, z+0.5*(z-z1));
//			n->SetLabelF(GraphSearchConstants::kXCoordinate, x+(len-1.0/600.0)*(x-x1));
//			n->SetLabelF(GraphSearchConstants::kYCoordinate, y+(len-1.0/600.0)*(y-y1));
//			n->SetLabelF(GraphSearchConstants::kZCoordinate, z+(len-1.0/600.0)*(z-z1));
//			//			}
//		}
//	}	
}

void DrawGraph(Graph *g)
{
	glBegin(GL_LINES);
	glColor3f(0, 1.0, 0);
	
	for (int t = 0; t < g->GetNumNodes(); t++)
	{
		node *n = g->GetNode(t);
		neighbor_iterator ni = n->getNeighborIter();
		for (long tmp = n->nodeNeighborNext(ni); tmp != -1; tmp = n->nodeNeighborNext(ni))
		{
			double x, y, z;
			x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
			y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
			z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
			
			double x1, y1, z1;
			node *nb = g->GetNode(tmp);
			x1 = nb->GetLabelF(GraphSearchConstants::kXCoordinate);
			y1 = nb->GetLabelF(GraphSearchConstants::kYCoordinate);
			z1 = nb->GetLabelF(GraphSearchConstants::kZCoordinate);

			glVertex3f(x, y, z);
			glVertex3f(x1, y1, z1);
		}
	}	
	glEnd();
}

void TestAPSP()
{
	std::vector<std::vector<double> > lengths;
	double cnt = 0;
	std::vector<double> buckets;
	while (1)
	{
		cnt += 1;
		for (int s = 8; s <= 64; s+=4)
		{
			Map *m = new Map(s, s);
			MakeMaze(m);
			Graph *g = GetGraph(m);
			
			FloydWarshall(g, lengths);
			double mVal = 0;
			for (unsigned x = 0; x < lengths.size(); x++)
			{
				for (unsigned y = 0; y < lengths.size(); y++)
				{
	//				printf("%1.1f ", lengths[x][y]);
					if (lengths[x][y] > mVal)
						mVal = lengths[x][y];
				}
	//			printf("\n");
			}
			if ((s-8)/4 >= buckets.size())
				buckets.resize((s-8)/4+1);
			buckets[(s-8)/4] += mVal;
			printf("States\t%d\tWidth\t%f\n", g->GetNumNodes(), mVal);
			delete g;
			delete m;
		}
		printf("Updated averages:\n");
		for (int x = 0; x < buckets.size(); x++)
		{
			printf("%d\t%f\n", 8+x*4, buckets[x]/cnt);
		}
	}
}

void TestSmallMap()
{
	std::vector<std::pair<int, int> > problems;
	Map *m = new Map(300, 300);
	//BuildRandomRoomMap(m, 30);
	MakeMaze(m);
	m->scale(600, 600);
	Graph *g = GetGraph(m);

	GraphMapInconsistentHeuristic gmih(m, g);
	GraphEnvironment ge(g, &gmih);
	ge.SetDirected(true);
	while (problems.size() < 200)
	{
		graphState a = g->GetRandomNode()->GetNum();
		graphState b = g->GetRandomNode()->GetNum();
		problems.push_back(std::pair<int, int>(a, b));
	}
	TemplateAStar<graphState, graphMove, GraphEnvironment> astar;
	std::vector<graphState> thePath;
	
	gmih.UseSmartPlacement(true);
	for (int x = 0; x < 10; x++)
	{
		printf("Adding heuristic %d\n", x);
		gmih.AddHeuristic();
		printf("Done adding heuristic %d\n", x);
	}
	
	for (int x = 10; x <= 10; x++)
	{
		gmih.SetNumUsedHeuristics(x);
		
		int nodes = 0;
		int touched = 0;
		double hvalues = 0;
		for (unsigned int y = 0; y < problems.size(); y++)
		{
			graphState start, goal;
			start = problems[y].first;
			goal = problems[y].second;
			astar.GetPath(&ge, start, goal, thePath);
			nodes += astar.GetNodesExpanded();
			touched += astar.GetNodesTouched();
			hvalues += ge.HCost(start, goal);
			printf("%d   %d\n", y, astar.GetNodesExpanded());
//			if (0 == y%100)
//				printf("%d ", y);
		}
		printf("%d\t%f\t%f\t%f\n", x, (double)nodes/problems.size(), (double)touched/problems.size(), (double)hvalues/problems.size());
	}
	
}
