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
#include "MapOverlay.h"
#include <string>

enum mode {
	kAddDH = 0,
	kShowHeuristicDiff = 1,
	kFindPath = 2
};

MapEnvironment *me = 0;
TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
std::vector<xyLoc> path;
std::vector<xyLoc> hd;
std::vector<xyLoc> farStates;

void DoHighwayDimension(xyLoc s);
void DoHighwayDimension1(xyLoc s);
void DoHighwayDimension2(CanonicalGrid::xyLoc s);
void RedoHDDisplay();

xyLoc start, goal;
int hdSearchType = 0;

mode m = kAddDH;

bool recording = false;
bool running = false;

int radius = 5;
int multiplier = 4;

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
		
		//Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0011SR.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0012SR.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/lak303d.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/da2/ht_chantry.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/den201d.map");
		Map *map = new Map("/Users/nathanst/hog2/maps/dao/hrt201d.map");

		map->SetTileSet(kWinter);
		me = new MapEnvironment(map);

		if ((0))
		{
			// draw basic map with grid and start state
			std::fstream svgFile;
			svgFile.open("/Users/nathanst/Desktop/transit.svg", std::fstream::out | std::fstream::trunc);
			svgFile << me->SVGHeader();
			svgFile << me->SVGDraw();
//			px1 = 19; py1 = 18;
//			me->SetColor(1, 1, 1);
//			svgFile << me->SVGLabelState({20, 20}, "S", 1);
			svgFile << "</svg>";
			svgFile.close();
		}
	}
}

int frameCnt = 0;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	me->OpenGLDraw();
	
	if (start.x != -1 && start.y != -1)
	{
		me->SetColor(0.0, 1.0, 0.0);
		me->OpenGLDraw(start);
	}

	me->SetColor(1.0, 0.0, 0.0);
	for (const xyLoc &l : hd)
		me->OpenGLDraw(l);

	me->SetColor(0.0, 0.0, 1.0);
	for (const xyLoc &l : farStates)
		me->OpenGLDraw(l);
	
	if (running)
	{
		astar.OpenGLDraw();

		if (path.size() == 0)
			astar.DoSingleSearchStep(path);
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
			break;
		}
		case ']':
		{
			m = mode((int(m)+1)%3);
			switch (m)
			{
				//case
				case kAddDH: submitTextToBuffer("Add DH"); break;
				case kShowHeuristicDiff: submitTextToBuffer("Show Heuristic Diff"); break;
				case kFindPath: submitTextToBuffer("Find Path!"); break;
			}

		}
			break;
		case '[':
		{
			switch (m)
			{
//				case kMoveNodes: m = kAddEdges; te.AddLine("Current mode: add edges"); break;
//				case kFindPath: m = kMoveNodes; te.AddLine("Current mode: moves nodes"); break;
//				case kAddNodes: m = kFindPath; te.AddLine("Current mode: find path"); break;
//				case kAddEdges: m = kAddNodes; te.AddLine("Current mode: add nodes"); break;
			}
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
			printf("Hit tab!\n");
			hdSearchType = (hdSearchType+1)%3;
			RedoHDDisplay();
			
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
				astar.DoSingleSearchStep(path);
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
				RedoHDDisplay();
//				if (m == kAddDH)
//					AddDH();
//				if (m == kShowHeuristicDiff)
//					ShowDiff();
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

			if (m == kFindPath)
			{
				astar.InitializeSearch(me, start, goal, path);
				//astar.SetHeuristic(&h);
				running = true;
			}
			return true;
		}
	}
	return false;
}

/*
 * Showing HD states based on individual searches to radius r
 */
void DoHighwayDimension(xyLoc s)
{
	hd.clear();
	farStates.clear();
	
	std::vector<xyLoc> v;
	TemplateAStar<xyLoc, tDirection, MapEnvironment> search;
	TemplateAStar<xyLoc, tDirection, MapEnvironment> search2;
	search.SetStopAfterGoal(false);
	ZeroHeuristic<xyLoc> z;
	search.SetHeuristic(&z);
	search.InitializeSearch(me, s, s, v);
	while (true)
	{
		double cost;
		bool success = search.GetOpenListGCost(search.CheckNextNode(), cost);
		if (!success || cost > 2*ROOT_TWO*radius*multiplier+1)
			break;
		if (search.DoSingleSearchStep(v))
			break;
	}
	Map *m = me->GetMap();
	for (int y = 0; y < m->GetMapHeight(); y++)
	{
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			if (y < s.y-multiplier*radius || y > s.y+multiplier*radius)
				continue;
			if ((x < s.x-multiplier*radius) || (x > s.x+multiplier*radius))
				continue;
			
			if (m->GetTerrainType(x, y) == kGround &&
				((x == s.x-multiplier*radius) ||
				 (x == s.x+multiplier*radius) ||
				 (y == s.y-multiplier*radius) ||
				 (y == s.y+multiplier*radius)))
			{
				xyLoc t(x, y);
				farStates.push_back(t);
				
				// these are fine, since they require longer paths to reach!
				if (search.GetStateLocation(t) == kOpenList)
				{
					std::cout << "Error: " << t << " found on open!\n";
					continue;
				}
				else if (search.GetStateLocation(t) == kNotFound)
				{
					std::cout << "Error: " << t << " not found!\n";
					continue;
				}
				search2.GetPath(me, s, t, v);
				//search.ExtractPathToStart(t, v);
				for (const xyLoc &l : v)
				{
					if (l.y < s.y-radius || l.y > s.y+radius)
						continue;
					if ((l.x < s.x-radius) || (l.x > s.x+radius))
						continue;

					if (l.x == s.x-radius || l.x == s.x+radius || l.y == s.y-radius || l.y == s.y+radius)
						hd.push_back(l);
				}
			}
		}
	}
}

/*
 * Showing HD states based on one dijkstra search to radius r
 * (Single tree is used to trace back all paths.)
 */
void DoHighwayDimension1(xyLoc s)
{
	hd.clear();
	farStates.clear();
	
	std::vector<xyLoc> v;
	TemplateAStar<xyLoc, tDirection, MapEnvironment> search;
	search.SetStopAfterGoal(false);
	ZeroHeuristic<xyLoc> z;
	search.SetHeuristic(&z);
	search.InitializeSearch(me, s, s, v);
	while (true)
	{
		double cost;
		bool success = search.GetOpenListGCost(search.CheckNextNode(), cost);
		if (!success || cost > 2*ROOT_TWO*radius*multiplier+1)
			break;
		if (search.DoSingleSearchStep(v))
			break;
	}
	Map *m = me->GetMap();
	for (int y = 0; y < m->GetMapHeight(); y++)
	{
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			if (y < s.y-multiplier*radius || y > s.y+multiplier*radius)
				continue;
			if ((x < s.x-multiplier*radius) || (x > s.x+multiplier*radius))
				continue;
			
			if (m->GetTerrainType(x, y) == kGround &&
				((x == s.x-multiplier*radius) ||
				 (x == s.x+multiplier*radius) ||
				 (y == s.y-multiplier*radius) ||
				 (y == s.y+multiplier*radius)))
			{
				xyLoc t(x, y);
				farStates.push_back(t);
				
				// these are fine, since they require longer paths to reach!
				if (search.GetStateLocation(t) == kOpenList)
				{
					std::cout << "Error: " << t << " found on open!\n";
					continue;
				}
				else if (search.GetStateLocation(t) == kNotFound)
				{
					std::cout << "Error: " << t << " not found!\n";
					continue;
				}
				
				search.ExtractPathToStart(t, v);
				for (const xyLoc &l : v)
				{
					if (l.y < s.y-radius || l.y > s.y+radius)
						continue;
					if ((l.x < s.x-radius) || (l.x > s.x+radius))
						continue;
					
					if (l.x == s.x-radius || l.x == s.x+radius || l.y == s.y-radius || l.y == s.y+radius)
						hd.push_back(l);
				}
			}
		}
	}
}

/*
 * Showing HD states based on canonical A* search to radius r
 * (Single tree is used to trace back all paths, but this won't impact the results, since there
 *  is a single canonical path for every goal state.)
 */
void DoHighwayDimension2(CanonicalGrid::xyLoc s)
{
	// draw basic map with grid and start state
	std::fstream svgFile;
	svgFile.open("/Users/nathanst/Desktop/transit.svg", std::fstream::out | std::fstream::trunc);
	svgFile << me->SVGHeader();
	svgFile << me->SVGDraw();
	svgFile << me->SVGDraw(xyLoc(s.x, s.y));
	
	
	
	hd.clear();
	farStates.clear();
	
	Map *m = me->GetMap();
	CanonicalGrid::CanonicalGrid g(m);
	
	std::vector<CanonicalGrid::xyLoc> v;
	TemplateAStar<CanonicalGrid::xyLoc, CanonicalGrid::tDirection, CanonicalGrid::CanonicalGrid> search;
	search.SetStopAfterGoal(false);
	ZeroHeuristic<CanonicalGrid::xyLoc> z;
	search.SetHeuristic(&z);
	search.InitializeSearch(&g, s, s, v);
	while (true)
	{
		double cost;
		bool success = search.GetOpenListGCost(search.CheckNextNode(), cost);
		if (!success || cost > 2*ROOT_TWO*radius*multiplier+1)
			break;
		if (search.DoSingleSearchStep(v))
			break;
	}
	for (int y = 0; y < m->GetMapHeight(); y++)
	{
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			if (y < s.y-multiplier*radius || y > s.y+multiplier*radius)
				continue;
			if ((x < s.x-multiplier*radius) || (x > s.x+multiplier*radius))
				continue;
			
			if (m->GetTerrainType(x, y) == kGround &&
				((x == s.x-multiplier*radius) ||
				 (x == s.x+multiplier*radius) ||
				 (y == s.y-multiplier*radius) ||
				 (y == s.y+multiplier*radius)))
			{
				CanonicalGrid::xyLoc t(x, y);
				farStates.push_back({t.x, t.y});
				
				// these are fine, since they require longer paths to reach!
				if (search.GetStateLocation(t) == kOpenList)
				{
					std::cout << "Error: " << t << " found on open!\n";
					continue;
				}
				else if (search.GetStateLocation(t) == kNotFound)
				{
					std::cout << "Error: " << t << " not found!\n";
					continue;
				}
				me->SetColor(0, 0, 1);
				svgFile << me->SVGDraw(xyLoc(x, y));
				search.ExtractPathToStart(t, v);
				for (const CanonicalGrid::xyLoc &l : v)
				{
					if (l.y < s.y-radius || l.y > s.y+radius)
						continue;
					if ((l.x < s.x-radius) || (l.x > s.x+radius))
						continue;
					
					if (l.x == s.x-radius || l.x == s.x+radius || l.y == s.y-radius || l.y == s.y+radius)
						hd.push_back({l.x, l.y});
				}
			}
		}
	}
	
	
	me->SetColor(0.5, 0.5, 0.5);
	for (int y = 0; y < m->GetMapHeight(); y++)
	{
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			if (y < s.y-radius || y > s.y+radius)
				continue;
			if ((x < s.x-radius) || (x > s.x+radius))
				continue;
			
			if (m->GetTerrainType(x, y) == kGround &&
				((x == s.x-radius) ||
				 (x == s.x+radius) ||
				 (y == s.y-radius) ||
				 (y == s.y+radius)))
			{
				svgFile << me->SVGDraw(xyLoc(x, y));
			}
		}
	}
	svgFile << "</svg>";
	svgFile.close();

}

void RedoHDDisplay()
{
	if (hdSearchType == 0)
	{
		submitTextToBuffer("Showing Canonical");
		CanonicalGrid::xyLoc l(start.x, start.y);
		DoHighwayDimension2(l);
	}
	else if (hdSearchType == 1)
	{
		submitTextToBuffer("Showing Regular (Individual searches)");
		DoHighwayDimension(start);
	}
	else {
		submitTextToBuffer("Showing Regular (Single dijkstra search)");
		DoHighwayDimension1(start);
	}
}
