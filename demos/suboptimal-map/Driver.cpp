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
#include "CanonicalGrid.h"
#include "TemplateAStar.h"
#include "TextOverlay.h"
#include "OptimisticSearch.h"
#include "MapGenerators.h"
#include "MapOverlay.h"
#include <string>

enum mode {
	kFindPathWA1 = 0,
	kFindPathWA15 = 1,
	kFindPathWA30 = 2,
	kFindPathOptimistic = 3
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

void StartSearch();
Map *ReduceMap(Map *inputMap);

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1200, 1200);
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Clear", "Clear graph", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Help", "Draw help", kAnyModifier, '?');
	InstallKeyboardHandler(MyDisplayHandler, "Weight", "Toggle Dijkstra & A*", kAnyModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Save", "Save current graph", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Load", "Load last saved graph", kAnyModifier, 'l');

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
		
		Map *map = new Map(300, 300);
		//MakeRandomMap(map, 40);
		//MakeMaze(map, 25);
		BuildRandomRoomMap(map, 25, 60);
		
		//Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0011SR.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0012SR.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/lak303d.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/da2/ht_chantry.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/den201d.map");

		Map *m2 = ReduceMap(map);
		delete map;
		map = m2;

		map->SetTileSet(kWinter);
		me = new MapEnvironment(map);
		start.x = 0xFFFF;
		start.y = 0xFFFF;
	}
}

int frameCnt = 0;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	me->OpenGLDraw();
	
	if (start.x != 0xFFFF && start.y != 0xFFFF)
	{
		me->SetColor(0.0, 1.0, 0.0);
		me->OpenGLDraw(start);
	}
	
	if (running && m == kFindPathOptimistic)
	{
		optimistic.OpenGLDraw();

		if (path.size() == 0)
		{
			for (int x = 0; x < stepsPerFrame && path.size() == 0; x++)
				optimistic.DoSingleSearchStep(path);
		}
		else {
			me->SetColor(0, 1, 0);
			glLineWidth(10);
			for (int x = 1; x < path.size(); x++)
			{
				me->GLDrawLine(path[x-1], path[x]);
			}
			glLineWidth(1);
		}
	}
	else if (running && m != kFindPathOptimistic)
	{
		astar.OpenGLDraw();
		
		if (path.size() == 0)
		{
			for (int x = 0; x < stepsPerFrame && path.size() == 0; x++)
				astar.DoSingleSearchStep(path);
		}
		else {
			me->SetColor(0, 1, 0);
			glLineWidth(10);
			for (int x = 1; x < path.size(); x++)
			{
				me->GLDrawLine(path[x-1], path[x]);
			}
			glLineWidth(1);
		}
	}

//	if (recording && viewport == GetNumPorts(windowID)-1)
//	{
//		char fname[255];
//		sprintf(fname, "/Users/nathanst/Movies/tmp/astar-%d%d%d%d",
//				(frameCnt/1000)%10, (frameCnt/100)%10, (frameCnt/10)%10, frameCnt%10);
//		SaveScreenshot(windowID, fname);
//		printf("Saved %s\n", fname);
//		frameCnt++;
//		if (path.size() == 0)
//		{
//			MyDisplayHandler(windowID, kNoModifier, 'o');
//		}
//		else {
//			recording = false;
//		}
//	}
//	return;
	
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
		case '{':
		{
			stepsPerFrame /= 2;
			if (stepsPerFrame < 1)
				stepsPerFrame = 1;
			break;
		}
		case '}':
		{
			stepsPerFrame *= 2;
			break;
		}
		case ']':
		{
			m = mode((int(m)+1)%4);
			switch (m)
			{
				//case
				case kFindPathWA1: submitTextToBuffer("Find A*"); break;
				case kFindPathWA15: submitTextToBuffer("Find WA* w = 1.5"); break;
				case kFindPathWA30: submitTextToBuffer("Find WA* w = 3.0"); break;
				case kFindPathOptimistic: submitTextToBuffer("Find Optimistic Path!"); break;
			}
			if (start.x != 0xFFFF && start.y != 0xFFFF)
				StartSearch();
		}
			break;
		case '[':
		{
			m = mode((int(m)+3)%4);
			switch (m)
			{
				case kFindPathWA1: submitTextToBuffer("Find A*"); break;
				case kFindPathWA15: submitTextToBuffer("Find WA* w = 1.5"); break;
				case kFindPathWA30: submitTextToBuffer("Find WA* w = 3.0"); break;
				case kFindPathOptimistic: submitTextToBuffer("Find Optimistic Path!"); break;
			}
			if (start.x != 0xFFFF && start.y != 0xFFFF)
				StartSearch();
		}
			break;
		case '|':
		{
		}
			break;
		case 'w':
//			if (weight > 0.5)
//				weight = 0.0;
//			else
//				weight = 1.0;
//			astar.SetWeight(weight);
//			astar.InitializeSearch(ge, astar.start, astar.goal, path);
//			ShowSearchInfo();
//			
//			running = true;
			break;
		case 'r':
			recording = !recording;
			break;
		case '0':
//		case '1': edgeCost = 1.0; te.AddLine("Adding edges; New edges cost 1"); m = kAddEdges; break;
//		case '2': edgeCost = 2.0; te.AddLine("Adding edges; New edges cost 2"); m = kAddEdges; break;
//		case '3': edgeCost = 3.0; te.AddLine("Adding edges; New edges cost 3"); m = kAddEdges; break;
//		case '4': edgeCost = 4.0; te.AddLine("Adding edges; New edges cost 4"); m = kAddEdges; break;
//		case '5': edgeCost = 5.0; te.AddLine("Adding edges; New edges cost 5"); m = kAddEdges; break;
//		case '6': edgeCost = 6.0; te.AddLine("Adding edges; New edges cost 6"); m = kAddEdges; break;
//		case '7': edgeCost = 7.0; te.AddLine("Adding edges; New edges cost 7"); m = kAddEdges; break;
//		case '8': edgeCost = 8.0; te.AddLine("Adding edges; New edges cost 8"); m = kAddEdges; break;
//		case '9': edgeCost = 9.0; te.AddLine("Adding edges; New edges cost 9"); m = kAddEdges; break;
		case '\t':
			
			if (mod != kShiftDown)
				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
			else
			{
				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
			}
			break;
		case 'p':
			//running = !running;
			break;
		case 'o':
		{
			if (running)
			{
				optimistic.DoSingleSearchStep(path);
			}
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
			if (me->GetMap()->GetTerrainType(x, y) == kGround)
			{
				start.x = x;
				start.y = y;
				goal = start;
				printf("Hit (%d, %d)\n", x, y);
			}
			return true;
		}
		case kMouseDrag:
		{
			int x, y;
			me->GetMap()->GetPointFromCoordinate(loc, x, y);
			if (me->GetMap()->GetTerrainType(x, y) == kGround)
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
			if (me->GetMap()->GetTerrainType(x, y) == kGround)
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
	if (m == kFindPathOptimistic)
	{
		optimistic.InitializeSearch(me, start, goal, path);
		running = true;
	}
	else if (m == kFindPathWA1)
	{
		astar.SetWeight(1);
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
