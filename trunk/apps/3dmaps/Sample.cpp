/*
 * $Id: sample.cpp,v 1.23 2006/11/01 23:33:56 nathanst Exp $
 *
 *  sample.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Copyright 2005 Nathan Sturtevant, University of Alberta. All rights reserved.
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

#include "Common.h"
#include "Sample.h"
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include "Map2DEnvironment.h"
#include "RandomUnits.h"
#include "TemplateAStar.h"
#include "GraphEnvironment.h"
#include "MapSectorAbstraction.h"
#include "GraphRefinementEnvironment.h"
#include "ScenarioLoader.h"
#include "BFS.h"
#include "Map3DGrid.h"
#include "MinimalSectorAbstraction.h"

bool mouseTracking = false;
bool runningSearch1 = false;
bool runningSearch2 = false;
bool gEdit = true;
int px1, py1, pz1;//, px2, py2;
int absType = 0;
int mazeSize = 40;
int gStepsPerFrame = 4;
double searchWeight = 0;
bool screenShot = false;
bool addPoints = true;

std::vector<UnitMapSimulation *> unitSims;

TemplateAStar<graphState, graphMove, GraphEnvironment> astar;

TemplateAStar<xyLoc, tDirection, MapEnvironment> a1;
TemplateAStar<xyLoc, tDirection, MapEnvironment> a2;
MapEnvironment *ma1 = 0;
MapEnvironment *ma2 = 0;
GraphDistanceHeuristic *gdh = 0;

MapSectorAbstraction *msa;

std::vector<xyLoc> path;

Map3DGrid *m3d = 0;
#include "BitMap.h"

int main(int argc, char* argv[])
{
//	BitMapPic p("/Users/nathanst/Desktop/Untitled.bmp");
//	for (int x = 0; x < p.GetWidth(); x++)
//	{
//		for (int y = 0; y < p.GetHeight(); y++)
//		{
//			uint8_t r, g, b, a;
//			p.GetPixel(x, y, r, g, b, a);
//			if ((2*g > 3*r) && (2*g > 3*b) && (abs(r-b) < 75))
//				p.SetPixel(x, y, 255, 255, 255, a);
//			else
//				p.SetPixel(x, y, 0, 0, 0, a);
//		}
//	}
//	p.Save("/Users/nathanst/Desktop/New.bmp");
//	exit(0);
	InstallHandlers();
//	int h = 0;
//	for (int x = 0; x < 20; x++)
//	{
//		if (0 == x%2)
//		{
//			for (int y = 0; y < 20; y++)
//			{
//				m3d.AddPoint(x, y, h++);
//			}
//		}
//		else {
//			for (int y = 20-1; y >= 0; y--)
//			{
//				m3d.AddPoint(x, y, h);
//			}
//		}
//	}
//	m3d.Rebuild();

//	for (int x = 19; x >= 0; x--)
//	{
//		for (int y = 0; y < 20; y++)
//		{
//			bool added = false;
//			for (int z = 0; z < 10; z++)
//			{
//				if (abs(x-y) < z)
//				{
//					m3d.AddPoint(19-x, y, 10-z);
//					added = true;
//					break;
//				}
//			}
//			if (!added)
//				m3d.AddPoint(19-x, y, 0);
//		}
//	}

	//	for (int x = 0; x < 10; x++)
//	{
//		for (int y = 0; y < 10; y++)
//		{
//			m3d.AddPoint(x, y, 5);
//		}
//	}
	
	RunHOGGUI(argc, argv);
}


void MakeStaircase()
{
	delete m3d;
	m3d = new Map3DGrid(10, 10, 8);
	int height = 0;
	int dir = 1;
	for (int x = 0; x < 10; x++)
		for (int y = 2; y < 10; y++)
			m3d->AddPoint(x, y, height);
	
	for (int t = 0; t < 5; t++)
	{
		for (int x = 0; x < 10; x++)
		{
			m3d->AddPoint(x, 0, height);
			m3d->AddPoint(x, 1, height++);
		}
		for (int x = 9; x >= 0; x--)
		{
			m3d->AddPoint(x, 2, height);
			m3d->AddPoint(x, 3, height++);
		}
	}

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
		map = new Map(mazeSize, mazeSize);
		MakeMaze(map, 10);
//		map->Scale(mazeSize, mazeSize);
	}
	else {
		map = new Map(gDefaultMap);
		//map->Scale(512, 512);
	}
	
	if (0 && gDefaultMap[0] == 0)
	{
		MakeStaircase();
//		m3d = new Map3DGrid(map, 16);
		//m3d->AddMap(map, 0);
		m3d->SetDrawGrid(true);
		m3d->PrintStats();
		//m3d->AddMap(map, 50);
	}
	else {
		m3d = new Map3DGrid(map, 16);
	}
	map->SetTileSet(kWinter);
	msa = new MapSectorAbstraction(map, 8);
	msa->ToggleDrawAbstraction(0);
	//msa->ToggleDrawAbstraction(2);
	//msa->ToggleDrawAbstraction(3);
	unitSims.resize(id+1);
	unitSims[id] = new UnitSimulation<xyLoc, tDirection, MapEnvironment>(new MapEnvironment(map));
	unitSims[id]->SetStepType(kMinTime);
	m3d->SetDrawGrid(true);
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Edit", "Toggle editing", kAnyModifier, 'e');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kNoModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Change weight", "Change the search weight", kNoModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');
	
	InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a randomly moving unit", kShiftDown, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, '1');
	
	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	InstallCommandLineHandler(MyCLHandler, "-memory", "-memory <map> <sectors>", "Measures the memory used by a particular map.");
	InstallCommandLineHandler(MyCLHandler, "-cut", "-cut <map> <sectors>", "put a 100 cell gash across the middle of the map");
	InstallCommandLineHandler(MyCLHandler, "-speed", "-speed <map> <sectors>", "Measures the speed of successor generation on a particular map.");

	InstallWindowHandler(MyWindowHandler);
	
	InstallMouseClickHandler(MyClickHandler);
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
		
		delete ma1;
		ma1 = 0;
		delete ma2;
		ma2 = 0;
		delete gdh;
		gdh = 0;
		delete unitSims[windowID];
		unitSims[windowID] = 0;
		runningSearch1 = false;
		runningSearch2 = false;
		mouseTracking = false;
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		CreateSimulation(windowID);
		SetNumPorts(windowID, 1);
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (m3d)
		m3d->OpenGLDraw();
}

void doExport()
{
	Map *map = new Map(gDefaultMap);
	map->Scale(512, 512);
	msa = new MapSectorAbstraction(map, 8);
	msa->ToggleDrawAbstraction(1);
	Graph *g = msa->GetAbstractGraph(1);
	printf("g\n%d %d\n", g->GetNumNodes(), g->GetNumEdges());
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		node *n = g->GetNode(x);
		int x1, y1;
		msa->GetTileFromNode(n, x1, y1);
		printf("%d %d %d\n", x, x1, y1);
	}
	for (int x = 0; x < g->GetNumEdges(); x++)
	{
		edge *e = g->GetEdge(x);
		printf("%d %d\n", e->getFrom(), e->getTo());//, (int)(100.0*e->GetWeight())); // %d 0
	}
	exit(0);
}

void buildProblemSet()
{
	ScenarioLoader s;
	printf(gDefaultMap); printf("\n");
	Map map(gDefaultMap);
	Graph *g = GraphSearchConstants::GetGraph(&map);
	GraphDistanceHeuristic gdh(g);
	gdh.SetPlacement(kFarPlacement);
	// make things go fast; we're doing tons of searches, so use a good heuristic
	for (unsigned int x = 0; x < 30; x++)
		gdh.AddHeuristic();
	GraphEnvironment *ge = new GraphEnvironment(&map, g, &gdh);
	ge->SetDirected(false);
	
	std::vector<std::vector<Experiment> > experiments;
	for (unsigned int x = 0; x < 100000; x++)
	{
		if (0==x%100)
		{ printf("\r%d", x); fflush(stdout); }
		node *s1 = g->GetRandomNode();
		node *g1 = g->GetRandomNode();
//		uint64_t nodesExpanded = 0;
//		printf("%d\t%d\t",  s1->GetNum(), g1->GetNum());
		graphState gs1, gs2;
		gs1 = s1->GetNum();
		gs2 = g1->GetNum();
		std::vector<graphState> thePath;
		astar.GetPath(ge, gs1, gs2, thePath);
//		printf("%d\n", (int)ge->GetPathLength(thePath));

		if (experiments.size() <= ge->GetPathLength(thePath)/4)
			experiments.resize(ge->GetPathLength(thePath)/4+1);
		if (experiments[ge->GetPathLength(thePath)/4].size() < 10)
		{
			Experiment e(s1->GetLabelL(GraphSearchConstants::kMapX), s1->GetLabelL(GraphSearchConstants::kMapY),
						 g1->GetLabelL(GraphSearchConstants::kMapX), g1->GetLabelL(GraphSearchConstants::kMapY),
						 map.GetMapWidth(), map.GetMapHeight(), ge->GetPathLength(thePath)/4, ge->GetPathLength(thePath), gDefaultMap);
			experiments[ge->GetPathLength(thePath)/4].push_back(e);
		}
		bool done = true;
		for (unsigned int y = 0; y < experiments.size(); y++)
		{
			if (experiments[y].size() != 10)
			{ done = false; break; }
		}
		if (done) break;
	}
	for (unsigned int x = 1; x < experiments.size(); x++)
	{
		if (experiments[x].size() != 10)
			break;
		for (unsigned int y = 0; y < experiments[x].size(); y++)
			s.AddExperiment(experiments[x][y]);
	}
	printf("\n");
	char name[255];
	sprintf(name, "%s.scen", gDefaultMap);
	s.Save(name);
	exit(0);
}


void runProblemSet2(char *problems, int multiplier)
{
	Map *map = new Map(gDefaultMap);
	map->Scale(512, 512);
	msa = new MapSectorAbstraction(map, 8, multiplier);
	
	Graph *g = msa->GetAbstractGraph(1);
	GraphAbstractionHeuristic gah1(msa, 1);
	GraphDistanceHeuristic localGDH(g);
	localGDH.SetPlacement(kAvoidPlacement);
	for (unsigned int x = 0; x < 10; x++)
		localGDH.AddHeuristic();

	GraphHeuristicContainer ghc(g);
	
	GraphRefinementEnvironment env1(msa, 1, &localGDH);
	GraphRefinementEnvironment env2(msa, 1, 0);
//	ghc.AddHeuristic(&localGDH);
//	ghc.AddHeuristic(&gah1);
	env1.SetDirected(false);
	
	FILE *f = fopen(problems, "r");
	if (f == 0)
	{
		printf("Cannot open file: '%s'\n", problems);
		exit(0);
	}
	Timer t;
	printf("len\tnodes\ttoucht\tlen\ttime\tdiff_n\tdiff_t\tdiff_l\ttime\n");
	while (!feof(f))
	{
		int from, to, cost;
		if (fscanf(f, "%d\t%d\t%d\n", &from, &to, &cost) != 3)
			break;
		node *s1 = g->GetNode(from);
		node *g1 = g->GetNode(to);
		graphState gs, gg;
		gs = s1->GetNum();
		gg = g1->GetNum();
		std::vector<graphState> thePath;
		t.StartTimer();
		astar.GetPath(&env2, gs, gg, thePath);
		t.EndTimer();
		printf("%d\t", cost);
		printf("%llu\t%llu\t%1.2f\t%e\t",
			   astar.GetNodesExpanded(), astar.GetNodesTouched(),
			   env1.GetPathLength(thePath), t.GetElapsedTime());
		t.StartTimer();
		astar.GetPath(&env1, gs, gg, thePath);
		t.EndTimer();
		printf("%llu\t%llu\t%1.2f\t%e",
			   astar.GetNodesExpanded(), astar.GetNodesTouched(),
			   env1.GetPathLength(thePath), t.GetElapsedTime());
		printf("\n");
	}
	fclose(f);
	exit(0);
}

void runProblemSet(char *problems, int multiplier)
{
	Map *map = new Map(gDefaultMap);
	map->Scale(512, 512);
	msa = new MapSectorAbstraction(map, 8, multiplier);

	Graph *g = msa->GetAbstractGraph(1);
	GraphAbstractionHeuristic gah2(msa, 2);
	GraphAbstractionHeuristic gah1(msa, 1);
	
	GraphRefinementEnvironment env2(msa, 2, &gah2);
	GraphRefinementEnvironment env1(msa, 1, &gah1);
	env1.SetDirected(false);
	env2.SetDirected(false);
	
	FILE *f = fopen(problems, "r");
	if (f == 0)
	{
		printf("Cannot open file: '%s'\n", problems);
		exit(0);
	}
	printf("len\tlvl2n\tlvl2nt\tlvl2len\tlvl2tim\tlvl1nf\tlvl1ntf\tlvl1tn\tlvl1tt\tlvl1len_f\ttot\ttott\ttot_len\n");
	Timer t;
	while (!feof(f))
	{
		int from, to, cost;
		if (fscanf(f, "%d\t%d\t%d\n", &from, &to, &cost) != 3)
			break;
		node *s1 = g->GetNode(from);
		node *g1 = g->GetNode(to);
		node *s2 = msa->GetParent(s1);
		node *g2 = msa->GetParent(g1);
		uint64_t nodesExpanded = 0;
		uint64_t nodesTouched = 0;
		double totalTime = 0;
		//		printf("Searching from %d to %d in level 1; %d to %d in level 2\n",
		//			   s1->GetNum(), g1->GetNum(), s2->GetNum(), g2->GetNum());
		graphState gs1, gs2;
		gs1 = s2->GetNum();
		gs2 = g2->GetNum();
		std::vector<graphState> thePath;
		std::vector<graphState> abstractPath;
		t.StartTimer();
		astar.GetPath(&env2, gs1, gs2, abstractPath);
		totalTime = t.EndTimer();
		printf("%d\t", cost);
		printf("%llu\t%llu\t%1.2f\t%f\t", astar.GetNodesExpanded(), astar.GetNodesTouched(), env2.GetPathLength(abstractPath), totalTime);
		if (abstractPath.size() == 0)
		{
			printf("%llu\t%llu\t%llu\t%llu\t%1.2f\t%f\t", (uint64_t)0, (uint64_t)0, astar.GetNodesExpanded(), astar.GetNodesTouched(), 0.0, 0.0);
			printf("%llu\t%llu\t%1.2f\t%f\t%d\t%d\n", astar.GetNodesExpanded(), astar.GetNodesTouched(), 0.0, 0.0, 0, 0);
//			printf("\n");
			continue;
		}

		nodesExpanded += astar.GetNodesExpanded();
		nodesTouched += astar.GetNodesTouched();

		env1.SetPlanningCorridor(abstractPath, 2);
		gs1 = s1->GetNum();
		gs2 = g1->GetNum();
		t.StartTimer();
		astar.GetPath(&env1, gs1, gs2, thePath);
		t.EndTimer();
		printf("%llu\t%llu\t%llu\t%llu\t%1.2f\t%f\t",
			   astar.GetNodesExpanded(), astar.GetNodesTouched(),
			   astar.GetNodesExpanded()+nodesExpanded, astar.GetNodesTouched()+nodesTouched,
			   env1.GetPathLength(thePath), totalTime+t.GetElapsedTime());
		
		int abstractStart = 0;
		gs1 = s1->GetNum();
		double totalLength = 0;
		int refineAmt = 2;
		int refinedPathNodes = 0;
		do { // not working yet -- fully check!
			env1.SetPlanningCorridor(abstractPath, 2, abstractStart);
			gs2 = g1->GetNum();
			if (abstractPath.size()-abstractStart > refineAmt)
			{
				env1.SetUseAbstractGoal(true, 2);
				gs2 = abstractPath[abstractStart+refineAmt];
			}
			else {
				env1.SetUseAbstractGoal(false, 0);
			}
			t.StartTimer();
			astar.GetPath(&env1, gs1, gs2, thePath);
			t.EndTimer();
			refinedPathNodes += thePath.size();
			totalTime+=t.GetElapsedTime();
			abstractStart += refineAmt;
			gs1 = thePath.back();
			
			nodesExpanded += astar.GetNodesExpanded();
			nodesTouched += astar.GetNodesTouched();
			totalLength += env1.GetPathLength(thePath);
			if (thePath.back() == gs2)
				break;
		} while (thePath.back() != g1->GetNum());
		
//		printf("%llu\t%llu\t%1.2f\t", astar.GetNodesExpanded(), astar.GetNodesTouched(), env1.GetPathLength(thePath));
		thePath.resize(0);
		printf("%llu\t%llu\t%1.2f\t%f\t%d\t%d\n", nodesExpanded, nodesTouched, totalLength, totalTime, abstractPath.size(), refinedPathNodes);
		
//		gs1 = s1->GetNum();
//		gs2 = g1->GetNum();
//		env1.SetPlanningCorridor(abstractPath, 2);
//		astar.GetPath(&env1, gs1, gs2, thePath);
//		printf("%llu\t%1.2f\n", astar.GetNodesExpanded(), env1.GetPathLength(thePath));
	}
	fclose(f);
	exit(0);
}


int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp( argument[0], "-map" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		strncpy(gDefaultMap, argument[1], 1024);
		Map map(argument[1]);
//		map.Scale(512, 512);
		map.Save(argument[2]);
//		//buildProblemSet();
//		//doExport();
		exit(0);
		return 2;
	}
	else if (strcmp(argument[0], "-memory" ) == 0 )
	{
		if (maxNumArgs <= 2) exit(0);
		Map map(argument[1]);
		map.Trim();
		MinimalSectorAbstraction msabs(&map, atoi(argument[2]));
		Map3DGrid m3g(&map, atoi(argument[2]));
		printf("Previously: abstraction: %d map: %d\n", msabs.GetAbstractionBytesUsed(), map.GetMapWidth()*map.GetMapHeight());
		printf("New: abstraction: %d map: %d\n", m3g.GetAbstractionBytesUsed(), m3g.GetGridBytesUsed());
		exit(0);
		return 3;
	}
	else if (strcmp(argument[0], "-cut" ) == 0 )
	{
		if (maxNumArgs <= 2) exit(0);
		Map map(argument[1]);
		Map3DGrid m3g(&map, atoi(argument[2]));
		Timer t;
		t.StartTimer();
		for (int x = 1; x < 100; x++)
		{
			bool doRepair = ((map.GetMapWidth()/2-50+x)%gSectorSize)==(gSectorSize-1);
			m3g.RemovePoint(map.GetMapWidth()/2-50+x, map.GetMapHeight()/2, 0, doRepair);
		}
		t.EndTimer();
		printf("Cut time: %1.6f\n", t.GetElapsedTime());
		t.StartTimer();
		m3g.AddPoint(map.GetMapWidth()/2, map.GetMapHeight()/2, 0);
		for (int x = 1; x < 50; x++)
		{
			if ((map.GetMapWidth()/2-x >= 0) && (map.GetMapWidth()/2+x < map.GetMapWidth()))
			{
				m3g.AddPoint(map.GetMapWidth()/2-x, map.GetMapHeight()/2, 0);
				m3g.AddPoint(map.GetMapWidth()/2+x, map.GetMapHeight()/2, 0);
			}
		}
		t.EndTimer();
		printf("Add time: %1.6f\n", t.GetElapsedTime());
		exit(0);
		return 3;
	}
	else if (strcmp(argument[0], "-speed" ) == 0 )
	{
		Map map(argument[1]);
		MinimalSectorAbstraction msabs(&map, atoi(argument[2]));
		Map3DGrid m3g(&map, atoi(argument[2]));

		state3d s;
		Timer t;
		double oldTime = 0, newTime = 0;
		int cnt = 0;
		std::vector<state3d> n1;
		srandom(34);
		for (int x = 0; x < map.GetMapWidth(); x++)
		{
			for (int y = 0; y < map.GetMapHeight(); y++)
			{
				if (m3g.FindNearState(x, y, 0, s) != -1)
				{
					t.StartTimer();
					m3g.GetSuccessors(s, n1);
					newTime += t.EndTimer();
					cnt++;
//					if (n1.size() == 0)
//						continue;
//					for (int x = 0; x < 0; x++)
//					{
//						s = n1[random()%n1.size()];
//						m3g.GetSuccessors(s, n1);
//						if (n1.size() == 0)
//						{
//							assert(!"No successors to state");
//							break;
//						}
//					}
//					x = map.GetMapWidth();
//					y = map.GetMapHeight();
				}
			}
		}
		printf("Time: %d points New: %1.7f time; ", cnt, newTime);

		MapEnvironment me(&map);
		std::vector<xyLoc> n2;
		srandom(34);
		t.StartTimer();
		for (int x = 0; x < map.GetMapWidth(); x++)
		{
			for (int y = 0; y < map.GetMapHeight(); y++)
			{
				if (map.GetTerrainType(x, y) != kGround)
					continue;
				xyLoc s(x, y);
				t.StartTimer();
				me.GetSuccessors(s, n2);
				oldTime += t.EndTimer();

//				if (n2.size() == 0)
//					continue;
//				for (int x = 0; x < 0; x++)
//				{
//					me.GetSuccessors(n2[random()%n2.size()], n2);
//				}
//				x = map.GetMapWidth();
//				y = map.GetMapHeight();
			}
		}
		printf("Old: %1.7f time\n", oldTime);
		exit(0);
		return 3;
	}
	return 2; //ignore typos
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 'w':
			if (searchWeight == 0)
				searchWeight = 1.0;
			else if (searchWeight == 1.0)
				searchWeight = 10.0;
			else if (searchWeight == 10.0)
				searchWeight = 0;
			break;
		case '[': if (gStepsPerFrame > 2) gStepsPerFrame /= 2; break;
		case ']': gStepsPerFrame *= 2; break;
		case 'e': gEdit = !gEdit; break;
		case '\t':
			m3d->SetDrawGrid(!m3d->GetDrawGrid());
//			if (mod != kShiftDown)
//				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
//			else
//			{
//				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
//			}
			break;
		case 'p': unitSims[windowID]->SetPaused(!unitSims[windowID]->GetPaused()); break;
		case 'o':
		{
			MakeStaircase();
		}
			break;
		default:
			break;
	}
}

void MyRandomUnitKeyHandler(unsigned long, tKeyboardModifier , char)
{
	addPoints = !addPoints;
	printf("Set add poitns to: %s\n", addPoints?"true":"false");
	return;
	
	static uint64_t average1=0, average2 = 0;
	static int count = 0;
	Graph *g = msa->GetAbstractGraph(1);
	GraphAbstractionHeuristic gah2(msa, 2);
	GraphAbstractionHeuristic gah1(msa, 1);

	GraphRefinementEnvironment env2(msa, 2, &gah2);
	GraphRefinementEnvironment env1(msa, 1, &gah1);
	env1.SetDirected(false);
	env2.SetDirected(false);

	for (unsigned int x = 0; x < 1; x++)
	{
//		node *s1 = g->GetRandomNode();
//		node *g1 = g->GetRandomNode();
//		node *s2 = msa->GetParent(s1);
//		node *g2 = msa->GetParent(g1);
//		int from, to, cost;
//		if (fscanf(f, "%d\t%d\t%d\n", &from, &to, &cost) != 3)
//			break;
		node *s1 = g->GetRandomNode();//g->GetNode(from);
		node *g1 = g->GetRandomNode();//g->GetNode(to);
		node *s2 = msa->GetParent(s1);
		node *g2 = msa->GetParent(g1);
		uint64_t nodesExpanded = 0;
		uint64_t nodesTouched = 0;
		//		printf("Searching from %d to %d in level 1; %d to %d in level 2\n",
		//			   s1->GetNum(), g1->GetNum(), s2->GetNum(), g2->GetNum());
		graphState gs1, gs2;
		gs1 = s2->GetNum();
		gs2 = g2->GetNum();
		std::vector<graphState> thePath;
		std::vector<graphState> abstractPath;
		astar.GetPath(&env2, gs1, gs2, abstractPath);
		//printf("Abstract length %d\n", abstractPath.size());
		//		printf("%d\t", cost);
		printf("%llu\t%llu\t%1.2f\t", astar.GetNodesExpanded(), astar.GetNodesTouched(), env2.GetPathLength(abstractPath));
		if (abstractPath.size() == 0)
			break;

		nodesExpanded += astar.GetNodesExpanded();
		nodesTouched += astar.GetNodesTouched();
		
		env1.SetPlanningCorridor(abstractPath, 2);
		gs1 = s1->GetNum();
		gs2 = g1->GetNum();
		astar.GetPath(&env1, gs1, gs2, thePath);
		printf("Full\t%llu\t%1.2f\t", astar.GetNodesExpanded(), env1.GetPathLength(thePath));
//		for (unsigned int x = 0; x < thePath.size(); x++)
//			printf("{%d}", thePath[x]);
		
		int abstractStart = 0;
		gs1 = s1->GetNum();
		double totalLength = 0;
		int refineAmt = 5;
		do { // not working yet -- fully check!
			env1.SetPlanningCorridor(abstractPath, 2, abstractStart);
			gs2 = g1->GetNum();
			if (abstractPath.size()-abstractStart > refineAmt)
			{
				env1.SetUseAbstractGoal(true, 2);
				gs2 = abstractPath[abstractStart+refineAmt];
				//printf("Using abstract goal of %d\n", abstractStart+refineAmt);
			}
			else {
				env1.SetUseAbstractGoal(false, 0);
			}
			astar.GetPath(&env1, gs1, gs2, thePath);
			abstractStart += refineAmt;
			gs1 = thePath.back();
			
//			for (unsigned int x = 0; x < thePath.size(); x++)
//				printf("(%d)", thePath[x]);
			
			nodesExpanded += astar.GetNodesExpanded();
			nodesTouched += astar.GetNodesTouched();
			totalLength += env1.GetPathLength(thePath);
			if (thePath.back() == gs2)
				break;
		} while (thePath.back() != g1->GetNum());
		
		printf("Part\t%llu\t%llu\t%1.2f\t", astar.GetNodesExpanded(), astar.GetNodesTouched(), totalLength);
		thePath.resize(0);
		printf("Tot\t%llu\t%llu\n", nodesExpanded, nodesTouched);
	}
}


void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	static int y = 18;
	static int x = 0;
	static bool add = false;

	if (m3d)
	{
		Timer timer;
		timer.StartTimer();
		//for (int t = 0; t < m3d->GetWidth(); t++)
		{
			if (add)
				m3d->AddPoint(x, 18, 0);
			else
				m3d->RemovePoint(x, y, 0);
			x = (x+1)%m3d->GetWidth();
			if (x == 0)
				add=!add;
		}
		timer.EndTimer();
//		y+=2;
		printf("%1.2fµs elapsed\n", 1000000*timer.GetElapsedTime());
	}
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	if (!gEdit)
		return false;
	mouseTracking = false;
	state3d s;
	if (button == kRightButton)
	{
		switch (mType)
		{
			case kMouseDown:
				m3d->GetPointFromCoordinate(loc, px1, py1, pz1);
				//unitSims[windowID]->GetEnvironment()->GetMap()
				printf("Mouse down at (%d, %d, %d)\n", px1, py1, pz1);
				if (addPoints)
				{
					if (-1 == m3d->FindNearState(px1, py1, 0, s))
						m3d->AddPoint(px1, py1, 0);
				}
				else {
					m3d->RemovePoint(px1, py1, 0);
				}

				break;
			case kMouseDrag:
				m3d->GetPointFromCoordinate(loc, px1, py1, pz1);
				//mouseTracking = true;
				//unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px2, py2);
				printf("Mouse tracking at (%d, %d, %d)\n", px1, py1, pz1);
				if (addPoints)
				{
					if (-1 == m3d->FindNearState(px1, py1, 0, s))
						m3d->AddPoint(px1, py1, 0);
				}
				else {
					m3d->RemovePoint(px1, py1, 0);
				}
				break;
			case kMouseUp:
				m3d->GetPointFromCoordinate(loc, px1, py1, pz1);
				printf("Mouse tracking at (%d, %d, %d)\n", px1, py1, pz1);
				if (addPoints)
				{
					if (-1 == m3d->FindNearState(px1, py1, 0, s))
						m3d->AddPoint(px1, py1, 0);
				}
				else {
					m3d->RemovePoint(px1, py1, 0);
				}
				//			{
//				if ((px1 == -1) || (px2 == -1))
//					break;
//				unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px2, py2);
//				printf("Searching from (%d, %d) to (%d, %d)\n", px1, py1, px2, py2);
//				
//				if (ma1 == 0)
//				{
//					ma1 = new MapEnvironment(unitSims[windowID]->GetEnvironment()->GetMap());
////					gdh = new GraphMapInconsistentHeuristic(ma1->GetMap(), GraphSearchConstants::GetGraph(ma1->GetMap()));
////					gdh->SetPlacement(kAvoidPlacement);
////					ma1->SetGraphHeuristic(gdh);
////					for (int x = 0; x < 10; x++)
////						gdh->AddHeuristic();
////					((GraphMapInconsistentHeuristic*)gdh)->SetNumUsedHeuristics(10);
////					((GraphMapInconsistentHeuristic*)gdh)->SetMode(kMax);
//				}
//				if (ma2 == 0)
//					ma2 = new MapEnvironment(unitSims[windowID]->GetEnvironment()->GetMap());
//				
//				a1.SetStopAfterGoal(true);
//				a2.SetStopAfterGoal(true);
//				//a2.SetWeight(1.8);
//				xyLoc s1;
//				xyLoc g1;
//				s1.x = px1; s1.y = py1;
//				g1.x = px2; g1.y = py2;
//				
//				a1.SetWeight(searchWeight);
//				a1.InitializeSearch(ma1, s1, g1, path);
//				//a2.InitializeSearch(ma2, s1, g1, path);
//				a1.SetUseBPMX(0);
//				runningSearch1 = true;
//				//runningSearch2 = true;
//				
//			}
				break;
		}
		return true;
	}
	return false;
}


