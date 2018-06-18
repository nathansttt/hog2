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
#include "Driver.h"
#include "Map2DEnvironment.h"
#include "TemplateAStar.h"
#include "OptimisticSearch.h"
#include "MapGenerators.h"
#include <string>

enum mode {
	kFindPathWA1 = 0,
	kFindPathWA11 = 1,
	kFindPathWA15 = 2,
	kFindPathWA30 = 3,
	kFindPathOptimistic11_15 = 4,
	kFindPathOptimistic11_3 = 5,
	kFindPathOptimistic15_3 = 6
};

MapEnvironment *me = 0;
TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
OptimisticSearch<xyLoc, tDirection, MapEnvironment> optimistic;
std::vector<xyLoc> path;

xyLoc start, goal;

mode m = kFindPathWA15;

int stepsPerFrame = 1;
bool recording = false;
bool running = false;
bool mapChanged = true;

void StartSearch();
Map *ReduceMap(Map *inputMap);

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1200, 1200);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
//	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Faster", "Run faster", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Slower", "Run slower", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "A*", "A*", kAnyModifier, '1');
	InstallKeyboardHandler(MyDisplayHandler, "wA*(1.1)", "wA*(1.1)", kAnyModifier, '2');
	InstallKeyboardHandler(MyDisplayHandler, "wA*(1.5)", "wA*(1.5)", kAnyModifier, '3');
	InstallKeyboardHandler(MyDisplayHandler, "wA*(3.0)", "wA*(3.0)", kAnyModifier, '4');
	InstallKeyboardHandler(MyDisplayHandler, "Optimistic", "Optimistic", kAnyModifier, '5');
	InstallKeyboardHandler(MyDisplayHandler, "Optimistic", "Optimistic", kAnyModifier, '6');
	InstallKeyboardHandler(MyDisplayHandler, "Optimistic", "Optimistic", kAnyModifier, '7');

	//InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	
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
		//glClearColor(0.99, 0.99, 0.99, 1.0);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		SetNumPorts(windowID, 1);
		
		Map *map = new Map(150, 150);
		//MakeRandomMap(map, 40);
		//MakeMaze(map, 25);
		BuildRandomRoomMap(map, 10, 60);
		
		//Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0011SR.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0012SR.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/lak303d.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/da2/ht_chantry.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/den201d.map");

		Map *m2 = ReduceMap(map);
		delete map;
		map = m2;

//		for (int x = 0; x < 300; x++)
//		for (int y = 0; y < 300; y++)
//		{
//			if (map->GetTerrainType(x, y) == kGround)
//				map->SetTerrainType(x, y, (random()%2)?kSwamp:kGround);
//		}
		
		map->SetTileSet(kWinter);
		me = new MapEnvironment(map);
		
		start.x = 0xFFFF;
		start.y = 0xFFFF;
	}
}

int frameCnt = 0;

void StepAlgorithms(int numSteps)
{
	if (running && (m == kFindPathOptimistic11_3 || m == kFindPathOptimistic15_3 || m == kFindPathOptimistic11_15))
	{
		if (path.size() == 0)
		{
			for (int x = 0; x < numSteps && path.size() == 0; x++)
				optimistic.DoSingleSearchStep(path);
			if (path.size() != 0)
			{
				std::string s = ": "+std::to_string(optimistic.GetNodesExpanded())+" nodes expanded";
				appendTextToBuffer(s.c_str());
			}
		}
	}
	else if (running)
	{
		if (path.size() == 0)
		{
			for (int x = 0; x < numSteps && path.size() == 0; x++)
				astar.DoSingleSearchStep(path);
			if (path.size() != 0)
			{
				std::string s = ": "+std::to_string(astar.GetNodesExpanded())+" nodes expanded";
				appendTextToBuffer(s.c_str());
			}
		}
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	
	if (mapChanged == true)
	{
		display.StartBackground();
		me->Draw(display);
		display.EndBackground();
		mapChanged = false;
	}

	
	if (start.x != 0xFFFF && start.y != 0xFFFF && !running)
	{
		me->SetColor(Colors::green);
		me->Draw(display, start);
		me->SetColor(Colors::red);
		me->DrawLine(display, start, goal, 10);
	}
	
	StepAlgorithms(stepsPerFrame);
	
	if (running && (m == kFindPathOptimistic11_3 || m == kFindPathOptimistic15_3 || m == kFindPathOptimistic11_15))
	{
		optimistic.Draw(display);

		if (path.size() != 0)
		{
			me->SetColor(Colors::green);
			for (int x = 1; x < path.size(); x++)
			{
				me->DrawLine(display, path[x-1], path[x], 5);
			}
		}
	}
	else if (running)
	{
		astar.Draw(display);
		
		if (path.size() != 0)
		{
			me->SetColor(Colors::green);
			for (int x = 1; x < path.size(); x++)
			{
				me->DrawLine(display, path[x-1], path[x], 5);
			}
		}
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
		case '[':
		{
			stepsPerFrame /= 2;
			break;
		}
		case ']':
		{
			if (stepsPerFrame < 32768)
				stepsPerFrame *= 2;
			if (stepsPerFrame == 0)
				stepsPerFrame = 1;
			break;
		}
		case 'r':
			recording = !recording;
			break;
		case '1': m = kFindPathWA1; submitTextToBuffer("Searching with A*"); StartSearch(); break;
		case '2': m = kFindPathWA11; submitTextToBuffer("Searching with wA*(1.1)"); StartSearch(); break;
		case '3': m = kFindPathWA15; submitTextToBuffer("Searching with wA*(1.5)"); StartSearch(); break;
		case '4': m = kFindPathWA30; submitTextToBuffer("Searching with wA*(3.0)"); StartSearch(); break;
		case '5':
			m = kFindPathOptimistic11_15;
			submitTextToBuffer("Searching with Optimistic(1.1, 1.5)");
			StartSearch();
			break;
		case '6': m = kFindPathOptimistic11_3; submitTextToBuffer("Searching with Optimistic(1.1, 3.0)"); StartSearch(); break;
		case '7': m = kFindPathOptimistic15_3; submitTextToBuffer("Searching with Optimistic(1.5, 3.0)"); StartSearch(); break;
		case 'p':
			stepsPerFrame = 0;
			break;
		case 'o':
		{
			StepAlgorithms(1);
		}
			break;
		case '?':
		{
		}
			break;
		case 's':
			break;
		case 'l':
			break;
		default:
			break;
	}
	
}

bool MyClickHandler(unsigned long , int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
	if (mType == kMouseDown)
	{
		switch (button)
		{
			case kRightButton: printf("Right button\n"); break;
			case kLeftButton: printf("Left button\n"); break;
			case kMiddleButton: printf("Middle button\n"); break;
		}
	}
	if (button != kLeftButton)
		return false;
	switch (mType)
	{
		case kMouseDown:
		{
			int x, y;
			me->GetMap()->GetPointFromCoordinate(loc, x, y);
			if (me->GetMap()->GetTerrainType(x, y) == kGround || me->GetMap()->GetTerrainType(x, y) == kSwamp)
			{
				start.x = x;
				start.y = y;
				goal = start;
				printf("Hit (%d, %d)\n", x, y);
				running = false;
			}
			return true;
		}
		case kMouseDrag:
		{
			int x, y;
			me->GetMap()->GetPointFromCoordinate(loc, x, y);
			if (me->GetMap()->GetTerrainType(x, y) == kGround || me->GetMap()->GetTerrainType(x, y) == kSwamp)
			{
				goal.x = x;
				goal.y = y;
			}
			break;
		}
		case kMouseUp:
		{
			int x, y;
			me->GetMap()->GetPointFromCoordinate(loc, x, y);
			if (me->GetMap()->GetTerrainType(x, y) == kGround || me->GetMap()->GetTerrainType(x, y) == kSwamp)
			{
				goal.x = x;
				goal.y = y;
				printf("UnHit (%d, %d)\n", x, y);
			}

			StartSearch();
			return true;
		}
	}
	return false;
}

void StartSearch()
{
	if (start == goal)
		return;
	if (m == kFindPathOptimistic11_15)
	{
		optimistic.SetWeight(1.5);
		optimistic.SetOptimalityBound(1.1);
		optimistic.InitializeSearch(me, start, goal, path);
		running = true;
	}
	else if (m == kFindPathOptimistic11_3)
	{
		optimistic.SetWeight(3.0);
		optimistic.SetOptimalityBound(1.1);
		optimistic.InitializeSearch(me, start, goal, path);
		running = true;
	}
	else if (m == kFindPathOptimistic15_3)
	{
		optimistic.SetWeight(3);
		optimistic.SetOptimalityBound(1.5);
		optimistic.InitializeSearch(me, start, goal, path);
		running = true;
	}
	else if (m == kFindPathWA1)
	{
		astar.SetWeight(1);
		astar.InitializeSearch(me, start, goal, path);
		running = true;
	}
	else if (m == kFindPathWA11)
	{
		astar.SetWeight(1.1);
		astar.InitializeSearch(me, start, goal, path);
		running = true;
	}
	else if (m == kFindPathWA15)
	{
		astar.SetWeight(1.5);
		astar.InitializeSearch(me, start, goal, path);
		running = true;
	}
	else if (m == kFindPathWA30)
	{
		astar.SetWeight(3.0);
		astar.InitializeSearch(me, start, goal, path);
		running = true;
	}
}


#include "GraphEnvironment.h"

int LabelConnectedComponents(Graph *g);

Map *ReduceMap(Map *inputMap)
{
	Graph *g = GraphSearchConstants::GetGraph(inputMap);
	
	int biggest = LabelConnectedComponents(g);
	
	Map *m = new Map(inputMap->GetMapWidth(), inputMap->GetMapHeight());
	for (int x = 0; x < inputMap->GetMapWidth(); x++)
	{
		for (int y = 0; y < inputMap->GetMapHeight(); y++)
		{
			if (inputMap->GetTerrainType(x, y) == kTrees)
				m->SetTerrainType(x, y, kTrees);
			else if (inputMap->GetTerrainType(x, y) == kWater)
				m->SetTerrainType(x, y, kWater);
			else m->SetTerrainType(x, y, kOutOfBounds);
		}
	}
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		if (g->GetNode(x)->GetLabelL(GraphSearchConstants::kTemporaryLabel) == biggest)
		{
			int theX, theY;
			theX = g->GetNode(x)->GetLabelL(GraphSearchConstants::kMapX);
			theY = g->GetNode(x)->GetLabelL(GraphSearchConstants::kMapY);
			if (g->GetNode(inputMap->GetNodeNum(theX+1, theY)) &&
				(g->GetNode(inputMap->GetNodeNum(theX+1, theY))->GetLabelL(GraphSearchConstants::kTemporaryLabel) == biggest) &&
				(!g->FindEdge(x, inputMap->GetNodeNum(theX+1, theY))))
			{
				m->SetTerrainType(theX, theY, kOutOfBounds);
			}
			else if (g->GetNode(inputMap->GetNodeNum(theX, theY+1)) &&
					 (g->GetNode(inputMap->GetNodeNum(theX, theY+1))->GetLabelL(GraphSearchConstants::kTemporaryLabel) == biggest) &&
					 (!g->FindEdge(x, inputMap->GetNodeNum(theX, theY+1))))
			{
				m->SetTerrainType(theX, theY, kOutOfBounds);
			}
			//			else if (g->GetNode(inputMap->GetNodeNum(theX+1, theY+1)) &&
			//					 (g->GetNode(inputMap->GetNodeNum(theX+1, theY+1))->GetLabelL(GraphSearchConstants::kTemporaryLabel) == biggest) &&
			//					 (!g->FindEdge(x, inputMap->GetNodeNum(theX+1, theY+1))))
			//			{
			//				m->SetTerrainType(theX, theY, kOutOfBounds);
			//			}
			else {
				if (inputMap->GetTerrainType(theX, theY) == kSwamp)
					m->SetTerrainType(theX, theY, kSwamp);
				else
					m->SetTerrainType(theX, theY, kGround);
			}
		}
		else if (inputMap->GetTerrainType(g->GetNode(x)->GetLabelL(GraphSearchConstants::kMapX),
										  g->GetNode(x)->GetLabelL(GraphSearchConstants::kMapY)) == kGround)
			m->SetTerrainType(g->GetNode(x)->GetLabelL(GraphSearchConstants::kMapX),
							  g->GetNode(x)->GetLabelL(GraphSearchConstants::kMapY), kTrees);
	}
	return m;
}

int LabelConnectedComponents(Graph *g)
{
	for (int x = 0; x < g->GetNumNodes(); x++)
		g->GetNode(x)->SetLabelL(GraphSearchConstants::kTemporaryLabel, 0);
	int group = 0;
	std::vector<int> groupSizes;
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		if (g->GetNode(x)->GetLabelL(GraphSearchConstants::kTemporaryLabel) == 0)
		{
			group++;
			groupSizes.resize(group+1);
			
			std::vector<unsigned int> ids;
			ids.push_back(x);
			while (ids.size() > 0)
			{
				unsigned int next = ids.back();
				ids.pop_back();
				if (g->GetNode(next)->GetLabelL(GraphSearchConstants::kTemporaryLabel) != 0)
					continue;
				groupSizes[group]++;
				g->GetNode(next)->SetLabelL(GraphSearchConstants::kTemporaryLabel, group);
				for (int y = 0; y < g->GetNode(next)->GetNumEdges(); y++)
				{
					edge *e = g->GetNode(next)->getEdge(y);
					if (g->GetNode(e->getFrom())->GetLabelL(GraphSearchConstants::kTemporaryLabel) == 0)
						ids.push_back(e->getFrom());
					if (g->GetNode(e->getTo())->GetLabelL(GraphSearchConstants::kTemporaryLabel) == 0)
						ids.push_back(e->getTo());
				}
			}
		}
	}
	int best = 0;
	for (unsigned int x = 1; x < groupSizes.size(); x++)
	{
		printf("%d states in group %d\n", groupSizes[x], x);
		if (groupSizes[x] > groupSizes[best])
			best = x;
	}
	printf("Keeping group %d\n", best);
	return best;
	//	kMapX;
}
