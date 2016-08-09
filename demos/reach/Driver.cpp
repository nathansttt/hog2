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
MapOverlay *mo = 0;
MapOverlay *canmo = 0;
TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
std::vector<xyLoc> path;
void ComputeReach(std::vector<double> &reach);
void ComputeCanonicalReach(std::vector<double> &reach);
std::vector<double> regularReach;
std::vector<double> canonicalReach;

xyLoc start, goal;
bool showCanonical = false;

mode m = kAddDH;
void AddDH();

bool recording = false;
bool running = false;


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
		
		//Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0012SR.map");
		Map *map = new Map("/Users/nathanst/hog2/maps/dao/lak202d.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/lak303d.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/da2/ht_chantry.map");
		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/den201d.map");

		map->SetTileSet(kWinter);
		mo = new MapOverlay(map);
		
		me = new MapEnvironment(map);
		h.e = me;
		astar.SetHeuristic(&h);
		ComputeReach(regularReach);
		ComputeCanonicalReach(canonicalReach);

	
		{
			std::fstream svgFile;
			svgFile.open("/Users/nathanst/Desktop/reach-can.svg", std::fstream::out | std::fstream::trunc);
			svgFile << me->SVGHeader();
			svgFile << me->SVGDraw();
			svgFile << canmo->SVGDraw();
			svgFile << "</svg>";
			svgFile.close();
		}
		{
			std::fstream svgFile;
			svgFile.open("/Users/nathanst/Desktop/reach-reg.svg", std::fstream::out | std::fstream::trunc);
			svgFile << me->SVGHeader();
			svgFile << me->SVGDraw();
			svgFile << mo->SVGDraw();
			svgFile << "</svg>";
			svgFile.close();
		}

	}
}

int frameCnt = 0;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	me->OpenGLDraw();
	if (showCanonical)
		canmo->OpenGLDraw();
	else
		mo->OpenGLDraw();
	
//	if (start.x != -1 && start.y != -1)
//	{
//		me->SetColor(0.0, 1.0, 0.0);
//		me->OpenGLDraw(start);
//	}
//	if (goal.x != -1 && goal.y != -1)
//	{
//		me->SetColor(1.0, 0, 0.0);
//		me->OpenGLDraw(goal);
//	}
	
	for (int x = 0; x < h.values.size(); x++)
	{
		me->SetColor(1.0, 0, 1.0);
		me->OpenGLDraw(h.values[x].startLoc);
	}
	
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

//			m = mode((int(m)+1)%3);
//			switch (m)
//			{
//				//case
//				case kAddDH: submitTextToBuffer("Add DH"); break;
//				case kShowHeuristicDiff: submitTextToBuffer("Show Heuristic Diff"); break;
//				case kFindPath: submitTextToBuffer("Find Path!"); break;
//			}

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
				if (m == kAddDH)
					AddDH();
				if (m == kShowHeuristicDiff)
					ShowDiff();
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
				astar.SetHeuristic(&h);
				running = true;
			}
			return true;
		}
	}
	return false;
}

void ComputeReach(xyLoc start, TemplateAStar<xyLoc, tDirection, MapEnvironment> &search, std::vector<double> &reach)
{
	std::vector<xyLoc> neighbors;
	Map *m = me->GetMap();
	
	for (int i = 0; i < search.GetNumItems(); i++)
	{
		const AStarOpenClosedData<xyLoc> &d = search.GetItem(i);
		me->GetSuccessors(d.data, neighbors);
		bool foundParent = false;
		// check if we have a neighbor that has this state as a parent
		for (int x = 0; x < neighbors.size(); x++)
		{
			AStarOpenClosedData<xyLoc> n;
			if (search.GetClosedItem(neighbors[x], n))
			{
				if (n.parentID == i)
				{
					foundParent = true;
					break;
				}
			}
		}
		if (foundParent)
			continue;
		
		// trace from i back to the start state updating the reach of each state
		double perfectHCost = 0;
		AStarOpenClosedData<xyLoc> v = search.GetItem(i);
		int currParent = i;
		while (true)
		{
			int64_t nextParent = v.parentID;
			if (nextParent == kTAStarNoNode || nextParent == currParent)
				break;
			xyLoc next = search.GetItem(nextParent).data;
			perfectHCost += me->GCost(next, v.data);
			double r = std::min(perfectHCost, search.GetItem(nextParent).g);
			reach[me->GetStateHash(next)] = std::max(r, reach[me->GetStateHash(next)]);
			mo->SetOverlayValue(next.x, next.y, reach[me->GetStateHash(next)]);
			v = search.GetItem(nextParent);
			currParent = nextParent;
		}
	}
}

// Not using canonical ordering
void ComputeReach(std::vector<double> &reach)
{
	if (me == 0)
		return;
	Map *m = me->GetMap();
	delete mo;
	mo = 0;
	if (mo == 0)
	{
		mo = new MapOverlay(m);
		mo->SetColorMap(2);
		mo->SetTransparentValue(0.1);
	}
	reach.clear();
	reach.resize(m->GetMapWidth()*m->GetMapHeight());
	
	TemplateAStar<xyLoc, tDirection, MapEnvironment> search;
	std::vector<xyLoc> result;
	search.SetStopAfterGoal(false);
	for (int y = 0; y < m->GetMapHeight(); y++)
	{
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			if (m->GetTerrainType(x, y) == kGround)
			{
				xyLoc l(x, y);
				std::cout << l << "\n";
				search.GetPath(me, l, l, result);
				ComputeReach(l, search, reach);
			}
			else {
				mo->SetOverlayValue(x, y, 0.1);
			}
		}
	}
	
	for (int c = 0; c < 20; c++)
	{
		int x1, y1;
		int x2, y2;
		do {
			x1 = random()%m->GetMapWidth();
			y1 = random()%m->GetMapHeight();
			x2 = random()%m->GetMapWidth();
			y2 = random()%m->GetMapHeight();
		} while (m->GetTerrainType(x1, y1) != kGround || m->GetTerrainType(x2, y2) != kGround);
		xyLoc l1(x1, y1);
		xyLoc l2(x2, y2);
		search.SetStopAfterGoal(true);
		search.GetPath(me, l1, l2, result);
		for (int x = 0; x < result.size(); x++)
		{
			printf("%1.2f ", reach[me->GetStateHash(result[x])]);
		}
		printf("\n");
	}
}


void ComputeCanonicalReach(CanonicalGrid::xyLoc start,
						   TemplateAStar<CanonicalGrid::xyLoc,
						   CanonicalGrid::tDirection,
						   CanonicalGrid::CanonicalGrid> &search,
						   std::vector<double> &reach,
						   CanonicalGrid::CanonicalGrid &cge)
{
	std::vector<CanonicalGrid::xyLoc> neighbors;
	Map *m = me->GetMap();
	
	for (int i = 0; i < search.GetNumItems(); i++)
	{
		const AStarOpenClosedData<CanonicalGrid::xyLoc> &d = search.GetItem(i);
		cge.GetSuccessors(d.data, neighbors);
		bool foundParent = false;
		// check if we have a neighbor that has this state as a parent
		for (int x = 0; x < neighbors.size(); x++)
		{
			AStarOpenClosedData<CanonicalGrid::xyLoc> n;
			if (search.GetClosedItem(neighbors[x], n))
			{
				if (n.parentID == i)
				{
					foundParent = true;
					break;
				}
			}
		}
		if (foundParent)
			continue;
		
		// trace from i back to the start state updating the reach of each state
		double perfectHCost = 0;
		AStarOpenClosedData<CanonicalGrid::xyLoc> v = search.GetItem(i);
		int currParent = i;
		while (true)
		{
			int64_t nextParent = v.parentID;
			if (nextParent == kTAStarNoNode || nextParent == currParent)
				break;
			CanonicalGrid::xyLoc next = search.GetItem(nextParent).data;
			perfectHCost += cge.GCost(next, v.data);
			double r = std::min(perfectHCost, search.GetItem(nextParent).g);
			reach[cge.GetStateHash(next)] = std::max(r, reach[cge.GetStateHash(next)]);
			canmo->SetOverlayValue(next.x, next.y, reach[cge.GetStateHash(next)]);
			v = search.GetItem(nextParent);
			currParent = nextParent;
		}
	}
}

// Not using canonical ordering
void ComputeCanonicalReach(std::vector<double> &reach)
{
	if (me == 0)
		return;
	Map *m = me->GetMap();
	CanonicalGrid::CanonicalGrid cge(m);
	
	delete canmo;
	canmo = 0;
	if (canmo == 0)
	{
		canmo = new MapOverlay(m);
		canmo->SetColorMap(2);
		canmo->SetTransparentValue(0.1);
	}
	reach.clear();
	reach.resize(m->GetMapWidth()*m->GetMapHeight());
	
	TemplateAStar<CanonicalGrid::xyLoc, CanonicalGrid::tDirection, CanonicalGrid::CanonicalGrid> search;
	std::vector<CanonicalGrid::xyLoc> result;
	search.SetStopAfterGoal(false);
	for (int y = 0; y < m->GetMapHeight(); y++)
	{
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			if (m->GetTerrainType(x, y) == kGround)
			{
				CanonicalGrid::xyLoc l(x, y);
				std::cout << l << "\n";
				search.GetPath(&cge, l, l, result);
				ComputeCanonicalReach(l, search, reach, cge);
			}
			else {
				canmo->SetOverlayValue(x, y, 0.1);
			}
		}
	}
}