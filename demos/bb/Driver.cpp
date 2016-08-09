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
CanonicalGrid::CanonicalGrid *grid;
MapOverlay *mo = 0;
MapOverlay *canmo = 0;
TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
std::vector<xyLoc> path;
std::vector<CanonicalGrid::xyLoc> path2;
void DrawJPSGoalArea(CanonicalGrid::tDirection dir, bool drawBound);
void DrawRegularGoalArea(tDirection dir, bool drawBound);

xyLoc start, goal;
bool showCanonical = false;

mode m = kAddDH;
void AddDH();

bool recording = false;
bool running = false;

CanonicalGrid::tDirection cDir = CanonicalGrid::kN;
tDirection dir = kN;

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
		for (int x = 0; x < values.size(); x++)
			v = std::max(v, fabs(values[x].depths[e->GetStateHash(a)]-values[x].depths[e->GetStateHash(b)]));
		return v;
	}
	MapEnvironment *e;
	std::vector<dh> values;
};

DifferentialHeuristic h;

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
	InstallKeyboardHandler(MyDisplayHandler, "Rotate dir", "Rotate direction", kAnyModifier, 'w');
//	InstallKeyboardHandler(MyDisplayHandler, "Save", "Save current graph", kAnyModifier, 's');
//	InstallKeyboardHandler(MyDisplayHandler, "Load", "Load last saved graph", kAnyModifier, 'l');

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
		
		//Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0012SR.map");
		Map *map = new Map("/Users/nathanst/hog2/maps/da2/ht_chantry.map");

		map->SetTileSet(kWinter);
		mo = new MapOverlay(map);
		
		me = new MapEnvironment(map);
		grid = new CanonicalGrid::CanonicalGrid(map);
		h.e = me;
		astar.SetHeuristic(&h);
		start.x = 0;
	}
}

int frameCnt = 0;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	me->OpenGLDraw();
	if (showCanonical)
		DrawJPSGoalArea(cDir, true);
	else
		DrawRegularGoalArea(dir, true);

//	for (int x = 0; x < h.values.size(); x++)
//	{
//		me->SetColor(1.0, 0, 1.0);
//		me->OpenGLDraw(h.values[x].startLoc);
//	}
//	
//	if (running)
//	{
//		astar.OpenGLDraw();
//
//		if (path.size() == 0)
//			astar.DoSingleSearchStep(path);
//		else {
//			me->SetColor(0, 1, 0);
//			glLineWidth(10);
//			for (int x = 1; x < path.size(); x++)
//			{
//				me->GLDrawLine(path[x-1], path[x]);
//			}
//			glLineWidth(1);
//		}
//
//	}
//	if (viewport == 0)
//	{
//		
//		if (running)
//		{
//			//astar.DoSingleSearchStep(path);
//			astar.OpenGLDraw();
//		}
//		
//		if (path.size() > 0)
//		{
//			me->SetColor(0, 1, 0);
//			glLineWidth(10);
//			for (int x = 1; x < path.size(); x++)
//			{
//				me->GLDrawLine(path[x-1], path[x]);
//			}
//			glLineWidth(1);
//		}
//	}

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
	glLineWidth(1);
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
			h.values.resize(0);
//			name[0] = 'a';
//			g->Reset();
//			te.AddLine("Current mode: add nodes");
//			m = kAddNodes;
//			path.resize(0);
//			running = false;
		}
			break;
		case 'w':
			if (dir == kN) { dir = kNE; cDir = CanonicalGrid::kNE; }
			else if (dir == kNE) { dir = kE; cDir = CanonicalGrid::kE; }
			else if (dir == kE) { dir = kSE; cDir = CanonicalGrid::kSE; }
			else if (dir == kSE) { dir = kS; cDir = CanonicalGrid::kS; }
			else if (dir == kS) { dir = kSW; cDir = CanonicalGrid::kSW; }
			else if (dir == kSW) { dir = kW; cDir = CanonicalGrid::kW; }
			else if (dir == kW) { dir = kNW; cDir = CanonicalGrid::kNW; }
			else if (dir == kNW) { dir = kN; cDir = CanonicalGrid::kN; }
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
			showCanonical = !showCanonical;
			if (showCanonical)
			{
				printf("Showing canonical reach\n");
				submitTextToBuffer("Canonical Reach");
			}
			else {
				printf("Showing regular reach\n");
				submitTextToBuffer("Regular Reach");
			}
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

void ShowDiff()
{
	mo->Clear();
	std::vector<xyLoc> p;
	TemplateAStar<xyLoc, tDirection, MapEnvironment> search;
	search.SetStopAfterGoal(false);
	search.GetPath(me, start, start, p);
	printf("%lld nodes expanded\n", search.GetNodesExpanded());
	
	for (int x = 0; x < search.GetNumItems(); x++)
	{
		double cost;
		xyLoc v = search.GetItem(x).data;
		if (!search.GetClosedListGCost(v, cost))
			printf("Error reading depth from closed list!\n");
		else {
			int hash = me->GetStateHash(v);
			//printf("(%d, %d): %f\n", v.x, v.y, cost);
			mo->SetOverlayValue(v.x, v.y, cost-h.HCost(start, v));
			//printf("Read value: %f\n", mo->GetOverlayValue(v.x, v.y));
		}
	}
}

void AddDH()
{
	mo->Clear();
	dh newDH;
	mo->SetTransparentValue(0);
	newDH.startLoc = start;
	newDH.depths.resize(me->GetMaxHash());
	std::vector<xyLoc> p;
	TemplateAStar<xyLoc, tDirection, MapEnvironment> search;
	search.SetStopAfterGoal(false);
	search.GetPath(me, start, start, p);
	
	for (int x = 0; x < search.GetNumItems(); x++)
	{
		double cost;
		xyLoc v = search.GetItem(x).data;
		if (!search.GetClosedListGCost(v, cost))
			printf("Error reading depth from closed list!\n");
		else {
			int hash = me->GetStateHash(v);
			newDH.depths[hash] = cost;
			//printf("(%d, %d): %f\n", v.x, v.y, cost);
			mo->SetOverlayValue(v.x, v.y, cost);
			//printf("Read value: %f\n", mo->GetOverlayValue(v.x, v.y));
		}
	}
	mo->SetColorMap(10);
	h.values.push_back(newDH);
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
				start.x = x;
				start.y = y;
			}
			break;
		}
		case kMouseUp:
		{
//			int x, y;
//			me->GetMap()->GetPointFromCoordinate(loc, x, y);
//			if (me->GetMap()->GetTerrainType(x, y) == kGround)
//			{
//				goal.x = x;
//				goal.y = y;
//				printf("UnHit (%d, %d)\n", x, y);
//			}
//
//			if (m == kFindPath)
//			{
//				astar.InitializeSearch(me, start, goal, path);
//				astar.SetHeuristic(&h);
//				running = true;
//			}
//			return true;
		}
	}
	return false;
}

void DrawRegularGoalArea(tDirection dir, bool drawBound)
{
	if (start.x == 0)
		return;
	std::string str;
	int minx = 1000, miny = 1000;
	int maxx = 0, maxy = 0;
	
	TemplateAStar<xyLoc, tDirection, MapEnvironment> regAstar;
	xyLoc cStart;
	cStart.x = start.x;
	cStart.y = start.y;
	
	regAstar.SetWeight(1.0);
	regAstar.SetStopAfterGoal(false);
	regAstar.GetPath(me, cStart, cStart, path);
	me->SetColor(0.0, 0.0, 1.0);
	me->OpenGLDraw(cStart);
	
	std::deque<xyLoc> queue;
	// draw everything
	me->ApplyAction(cStart, dir);
	queue.push_back(cStart);
	std::vector<xyLoc> v;
	std::vector<bool> visited(me->GetMap()->GetMapHeight()*me->GetMap()->GetMapWidth());
	while (!queue.empty())
	{
		xyLoc next = queue.front();
		queue.pop_front();
		if (visited[next.x+next.y*me->GetMap()->GetMapWidth()] == true)
			continue;
		
		visited[next.x+next.y*me->GetMap()->GetMapWidth()] = true;
		grid->SetColor(1.0, 0.5, 0.0);
		me->SetColor(1.0, 0.5, 0.0);
		AStarOpenClosedData<xyLoc> data;
		if (regAstar.GetClosedItem(next, data))
		{
			me->GetSuccessors(data.data, v);
			CanonicalGrid::xyLoc t(data.data.x, data.data.y);
			grid->OpenGLDraw(t);
			
			if (data.data.x > maxx)
				maxx = data.data.x;
			if (data.data.x < minx)
				minx = data.data.x;
			if (data.data.y > maxy)
				maxy = data.data.y;
			if (data.data.y < miny)
				miny = data.data.y;
			
			for (auto &s : v)
			{
				double g;
				regAstar.GetClosedListGCost(s, g);
				if (fequal(g, data.g+me->GCost(data.data, s)))
					queue.push_back(s);
			}
		}
	}
	if (drawBound)
	{
		GLdouble t, l, r, b, z, rad, tmp;
		//printf("Bounds: (%d, %d) to (%d, %d)\n", minx, miny, maxx, maxy);
		me->GetMap()->GetOpenGLCoord(minx, miny, l, t, z, rad);
		me->GetMap()->GetOpenGLCoord(maxx, maxy, r, b, z, rad);
		glColor3f(1.0, 1.0, 1.0);
		glLineWidth(4.0);
		OutlineRect(l-rad, t-rad, r+rad, b+rad, z-2*rad);
	}
}

void DrawJPSGoalArea(CanonicalGrid::tDirection dir, bool drawBound)
{
	if (start.x == 0)
		return;
	std::string str;
	int minx = 1000, miny = 1000;
	int maxx = 0, maxy = 0;
	TemplateAStar<CanonicalGrid::xyLoc, CanonicalGrid::tDirection, CanonicalGrid::CanonicalGrid> canAstar;
	CanonicalGrid::xyLoc cStart;
	cStart.x = start.x;
	cStart.y = start.y;
	
	canAstar.SetWeight(1.0);
	canAstar.SetStopAfterGoal(false);
	canAstar.GetPath(grid, cStart, cStart, path2);
	
	std::deque<CanonicalGrid::xyLoc> queue;
	// draw everything
	grid->ApplyAction(cStart, dir);
	//cStart.y--;
	queue.push_back(cStart);
	std::vector<CanonicalGrid::xyLoc> v;
	std::vector<xyLoc> l;
	while (!queue.empty())
	{
		CanonicalGrid::xyLoc next = queue.front();
		queue.pop_front();
		
		grid->SetColor(0.0, 0.5, 1.0);
		me->SetColor(0.0, 0.5, 1.0);
		AStarOpenClosedData<CanonicalGrid::xyLoc> data;
		if (canAstar.GetClosedItem(next, data))
		{
			grid->GetSuccessors(data.data, v);
			grid->OpenGLDraw(data.data);
			
			if (data.data.x > maxx)
				maxx = data.data.x;
			if (data.data.x < minx)
				minx = data.data.x;
			if (data.data.y > maxy)
				maxy = data.data.y;
			if (data.data.y < miny)
				miny = data.data.y;
			
			for (auto &s : v)
			{
				double g;
				canAstar.GetClosedListGCost(s, g);
				//printf("Cost in closed: %f; cost through parent %f (%f+%f)\n", g, data.g+grid->GCost(data.data, s), data.g, grid->GCost(data.data, s));
				if (fequal(g, data.g+grid->GCost(data.data, s)))
					queue.push_back(s);
			}
		}
	}
	if (drawBound)
	{
		GLdouble t, l, r, b, z, rad, tmp;
		//printf("Bounds: (%d, %d) to (%d, %d)\n", minx, miny, maxx, maxy);
		me->GetMap()->GetOpenGLCoord(minx, miny, l, t, z, rad);
		me->GetMap()->GetOpenGLCoord(maxx, maxy, r, b, z, rad);
		glColor3f(1.0, 1.0, 1.0);
		glLineWidth(4.0);
		OutlineRect(l-rad, t-rad, r+rad, b+rad, z-2*rad);
	}
}