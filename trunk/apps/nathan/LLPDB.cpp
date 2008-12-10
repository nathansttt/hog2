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
#include "MapQuadTreeAbstraction.h"
#include "Common.h"
#include "Sample.h"
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
void BuildCPDBs();
void BuildWithThreads();

void BuildScenarioFiles();
void BuildScenarioFiles2();
void TestPDB();
void BuildMazeCPDBs();
void BuildRoomCPDBs();
void TestSTP();
void DrawGraph(Graph *g);
void MoveGraph(Graph *g);

void TestAPSP();
void TestSmallMap();

GraphCanonicalHeuristic *gch = 0;
GraphMapInconsistentHeuristic *gmih = 0;
Graph *stp = 0;
//void BuildPDB(Map *map);
//void GetCenters(MapQuadTreeAbstraction *msa, std::vector<int> &centers);
//void GetPDBValues(MapQuadTreeAbstraction *msa, const std::vector<int> &centers,
//				  std::vector<std::vector<double> > &lengths);


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
		map = new Map(12, 12);
//		//MakeMaze(map);
//		for (int x = 0; x < 100; x++)
//		{
//			char name[255];
//			sprintf(name, "maze_%d%d%d.map", x/100, (x%100)/10, (x%10));
		MakeMaze(map);
//		map->scale(512, 512);
//			map->save(name);
//		}
	}
	else
		map = new Map(gDefaultMap);
	
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
	InstallKeyboardHandler(MyPDBKeyHandler, "Test PDB", "Test a sliding-tile PDB", kNoModifier, 't');
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
		CreateSimulation(windowID);
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
		unitSims[windowID]->OpenGLDraw(windowID);
	
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


void MyPDBKeyHandler(unsigned long, tKeyboardModifier, char key)
{
	if (key == 'l')
	{
		//TestSmallMap();
		TestAPSP();

		//TestSTP();
		
		//delete gch;
		//gch = new GraphCanonicalHeuristic("CPDBs/8room_000.map.pdb");
		
		//BuildScenarioFiles();
		//BuildScenarioFiles2();
	}
	if (key == 'b')
	{
		BuildMazeCPDBs();
		//BuildRoomCPDBs();
		//BuildCPDBs();
	}
	if (key == 't')
	{
		srandom(1042);
		for (int x = 0; x < 20; x++)
		{
			TestPDB();
		}
	}
	if (key == 'r')
	{
		if (stp)
			MoveGraph(stp);
		else
			RunBigTest();
	}
}


void TestPDB()
{
	Graph *g = gch->GetGraph();
	GraphMapHeuristic gmh(gch->GetMap(), gch->GetGraph());

	GraphEnvironment *env1 = new GraphEnvironment(g, gch);
	GraphEnvironment *env2 = new GraphEnvironment(g, &gmh);
	env1->SetDirected(false);
	env2->SetDirected(false);
	
	node *from = g->GetRandomNode();
	node *to = g->GetRandomNode();
	std::vector<graphState> thePath;
	Prop astar(PROP_BPMX);
	//TemplateAStar<graphState, graphMove, GraphEnvironment> astar;

	graphState start, goal;
	start = from->GetNum();
	goal = to->GetNum();
	astar.GetPath(env2, g, start, goal, thePath);
	//astar.GetPath(env2, start, goal, thePath);
	printf("%d\t%d\t%d\t",
		   astar.GetNodesExpanded(), astar.GetNodesTouched(), (int)thePath.size());//, p->GetNodesReopened());

	astar.GetPath(env1, g, start, goal, thePath);
	//astar.GetPath(env1, start, goal, thePath);
	printf("%d\t%d\t%d\n",
		   astar.GetNodesExpanded(), astar.GetNodesTouched(), (int)thePath.size());//, p->GetNodesReopened());
}


bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	return false;
}

void BuildCPDBs()
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
		gch = new GraphCanonicalHeuristic(m);
		
		char pdbname[255];
		sprintf(pdbname, "CPDBs/AR0%d%d%dSR.map.pdb", x/100, (x/10)%10, x%10);
		gch->save(pdbname);		
	}
}

void BuildMazeCPDBs()
{
	for (int x = 0; x < 100; x++)
	{
		for (int y = 200; y < 2000; y+=200)
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
			gch = new GraphCanonicalHeuristic(m, y);
			
			char pdbname[255];
			sprintf(pdbname, "CPDBs/maze_%d%d%d.map.%d.pdb", x/100, (x/10)%10, x%10, y);
			gch->save(pdbname);		
		}
	}
}

void BuildRoomCPDBs()
{
	for (int x = 0; x < 100; x++)
	{
		for (int y = 200; y < 2000; y+=200)
		{
			printf("Building room %d\n", x);
			char name[255];
			sprintf(name, "scenarios/rooms/8room_%d%d%d.map.txt", x/100, (x/10)%10, x%10);
			ScenarioLoader *sl = new ScenarioLoader(name);
			if (sl->GetNumExperiments() == 0)
				continue;
			
			delete gch;
			gch = 0;
			Map *m = new Map(sl->GetNthExperiment(0).GetMapName());
			m->scale(sl->GetNthExperiment(0).GetXScale(), 
					 sl->GetNthExperiment(0).GetYScale());
			gch = new GraphCanonicalHeuristic(m, y);
			
			char pdbname[255];
			sprintf(pdbname, "CPDBs/8room_%d%d%d.map.%d.pdb", x/100, (x/10)%10, x%10, y);
			gch->save(pdbname);		
			printf(pdbname);
			RunHeuristicTest(sl);
		}
	}
}

void RunBigTest()
{
	for (int y = 200; y < 2000; y+=200)
	{
		printf("------- %d -------\n", y);
		for (int x = 0; x <= 3; x++)
		{
			printf("------- %d -------\n", x);
			char name[255];
			//sprintf(name, "scenarios/rooms/8room_%d%d%d.map.txt", x/100, (x/10)%10, x%10);
			sprintf(name, "scenarios/mazes/maze_%d%d%d.map.txt", x/100, (x/10)%10, x%10);
			//sprintf(name, "scenarios/path/AR0%d%d%dSR.map.txt", x/100, (x/10)%10, x%10);
			ScenarioLoader *sl = new ScenarioLoader(name);
			if (sl->GetNumExperiments() == 0)
				continue;
			
			char pdbname[255];
			//sprintf(pdbname, "CPDBs/AR0%d%d%dSR.map.pdb", x/100, (x/10)%10, x%10);
			sprintf(pdbname, "CPDBs/maze_%d%d%d.map.%d.pdb", x/100, (x/10)%10, x%10, y);
			//sprintf(pdbname, "CPDBs/8room_%d%d%d.map.pdb", x/100, (x/10)%10, x%10);
			//sprintf(pdbname, "CPDBs/8room_%d%d%d.map.%d.pdb", x/100, (x/10)%10, x%10, y);
			
			delete gch;
			gch = 0;
			gch = new GraphCanonicalHeuristic(pdbname);

			Graph *g = gch->GetGraph();
			GraphMapInconsistentHeuristic::HN = 1+y*y/(g->GetNumNodes());
			delete gmih;
			gmih = new GraphMapInconsistentHeuristic(gch->GetMap(), gch->GetGraph());

			//RunHeuristicTest(sl);
			RunTest(sl);
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
	
	double sum1 = 0, sum2 = 0, sum3 = 0, sum4 = 0, sum5 = 0;
	double cnt = 0;
	for (int x = 0; x < sl->GetNumExperiments(); x++)
	{
		Experiment e = sl->GetNthExperiment(x);
		if (e.GetDistance() < 500)
			continue;
//		printf("%s\t%1.2f\t", e.GetMapName(), e.GetDistance());

		env1->SetDirected(false);
		env2->SetDirected(false);
		env3->SetDirected(false);

		graphState start, goal;

		start = gch->GetMap()->getNodeNum(e.GetStartX(), e.GetStartY());
		goal = gch->GetMap()->getNodeNum(e.GetGoalX(), e.GetGoalY());
		
		std::vector<graphState> thePath;
		AStarDelay delay(0);
		TemplateAStar<graphState, graphMove, GraphEnvironment> astar;
		
		//astar.GetPath(env2, g, start, goal, thePath);
		astar.SetUseBPMX(false);
//		astar.GetPath(env2, start, goal, thePath);
//		printf("%d\t",
//			   astar.GetNodesExpanded());//, p->GetNodesReopened());
//		sum1 += astar.GetNodesExpanded();
		
		//astar.GetPath(env1, g, start, goal, thePath);
		astar.SetUseBPMX(false);
//		astar.GetPath(env1, start, goal, thePath);
//		printf("%d\t",
//			   astar.GetNodesExpanded());
//		sum2 += astar.GetNodesExpanded();


//		astar.SetUseBPMX(true);
//		astar.GetPath(env1, start, goal, thePath);
//		printf("%d\t",
//			   astar.GetNodesExpanded());
//		sum3 += astar.GetNodesExpanded();

//		delay.GetPath(env1, g, start, goal, thePath);
//		printf("%ld\t",
//			   delay.GetNodesExpanded());
//		sum4 += delay.GetNodesExpanded();

		astar.SetUseBPMX(false);
		astar.GetPath(env3, start, goal, thePath);
//		printf("%d\n",
//			   astar.GetNodesExpanded());
		sum5 += astar.GetNodesExpanded();
		
		cnt++;
	}
	printf("##\t\t\t\t\t%f\t%f\t%f\t%f\t%f\n", sum1/cnt, sum2/cnt, sum3/cnt, sum4/cnt, sum5/cnt);
	delete env1;
	delete env2;
}

void RunHeuristicTest(ScenarioLoader *sl)
{
	Graph *g = gch->GetGraph();
	GraphMapHeuristic gmh(gch->GetMap(), gch->GetGraph());
	
	GraphEnvironment *env1 = new GraphEnvironment(g, gch);
	GraphEnvironment *env2 = new GraphEnvironment(g, &gmh);
	GraphEnvironment *env3 = new GraphEnvironment(g, gmih);
	
	double sum1 = 0, sum2 = 0, sum3 = 0, sum4 = 0;
	double cnt = 0;
	for (int x = 0; x < sl->GetNumExperiments(); x++)
	{
		Experiment e = sl->GetNthExperiment(x);
		if (e.GetDistance() < 250)
			continue;
		env1->SetDirected(false);
		env2->SetDirected(false);
		
		graphState start, goal;
		
		start = gch->GetMap()->getNodeNum(e.GetStartX(), e.GetStartY());
		goal = gch->GetMap()->getNodeNum(e.GetGoalX(), e.GetGoalY());
		
		cnt++;
		sum1 += env1->HCost(start, goal);
		sum2 += env2->HCost(start, goal);
		sum4 += env3->HCost(start, goal);
		sum3 += e.GetDistance();
//		printf("%s\t%1.2f\t%f\t%f\t%f\n", e.GetMapName(), e.GetDistance(),
//			   env1->HCost(start, goal), env2->HCost(start, goal), env3->HCost(start, goal));
				
	}
	printf("##\t\t\t\t\t%f\t%f\t%f\t%f\n", sum1/cnt, sum2/cnt, sum3/cnt, sum4/cnt);
	delete env1;
	delete env2;
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

void TestSTP()
{
	std::vector<std::pair<int, int> > problems;
	MNPuzzle p(3, 3);
	Graph *g = p.GetGraph();
	stp = g;
	GraphPuzzleDistanceHeuristic gpdh(p, g, 0);
	GraphEnvironment ge(g, &gpdh);
	ge.SetDirected(false);
	while (problems.size() < 1000)
	{
		int a = g->GetRandomNode()->GetNum();
		int b = g->GetRandomNode()->GetNum();
		MNPuzzleState s1(3, 3), s2(3, 3);
		p.GetStateFromHash(s1, a);
		p.GetStateFromHash(s2, b);
		if ((p.GetParity(s1) == p.GetParity(s2)) && (p.GetParity(s2) == 0) && (p.HCost(s1, s2) > 14))
			problems.push_back(std::pair<int, int>(a, b));
	}
	TemplateAStar<graphState, graphMove, GraphEnvironment> astar;
	std::vector<graphState> thePath;

	if (0)
	for (int x = 0; x <= 10; x++)
	{
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
			//printf("%d   %d\n", y, astar.GetNodesExpanded());
//			if (0 == y%100)
//				printf("%d ", y);
		}
		printf("%d\t%f\t%f\t%f\n", gpdh.GetNumHeuristics(), (double)nodes/problems.size(), (double)touched/problems.size(), (double)hvalues/problems.size());
		if (x == 10)
			break;
		for (int y = gpdh.GetNumHeuristics(); y > 0; y--)
			gpdh.AddHeuristic();
		if (gpdh.GetNumHeuristics() == 0)
			gpdh.AddHeuristic();
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
	Map *m = new Map(36, 36);
	MakeMaze(m);
	m->scale(72, 72);
	Graph *g = GetGraph(m);

	GraphMapInconsistentHeuristic gmih(m, g);
	GraphEnvironment ge(g, &gmih);
	ge.SetDirected(false);
	while (problems.size() < 2000)
	{
		graphState a = g->GetRandomNode()->GetNum();
		graphState b = g->GetRandomNode()->GetNum();
		problems.push_back(std::pair<int, int>(a, b));
	}
	TemplateAStar<graphState, graphMove, GraphEnvironment> astar;
	std::vector<graphState> thePath;
	
	for (int x = 0; x <= g->GetNumNodes(); x++)
	{
		GraphMapInconsistentHeuristic::HN = x;
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
			//printf("%d   %d\n", y, astar.GetNodesExpanded());
			//			if (0 == y%100)
			//				printf("%d ", y);
		}
		printf("%d\t%f\t%f\t%f\n", x, (double)nodes/problems.size(), (double)touched/problems.size(), (double)hvalues/problems.size());
	}
	
}
