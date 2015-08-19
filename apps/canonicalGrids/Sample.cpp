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
#include "AStar.h"
#include "TemplateAStar.h"
#include "GraphEnvironment.h"
#include "MapSectorAbstraction.h"
#include "GraphRefinementEnvironment.h"
#include "ScenarioLoader.h"
#include "BFS.h"
#include "PEAStar.h"
#include "EPEAStar.h"
#include "MapGenerators.h"
#include "FPUtil.h"
#include "CanonicalGrid.h"

bool mouseTracking = false;
bool runningSearch1 = false;
bool runningSearch2 = false;
int px1 = 0, py1 = 0, px2 = 0, py2 = 0;
int absType = 0;
int mazeSize = 20;
int gStepsPerFrame = 1;
double searchWeight = 1;
bool reopenNodes = false;
bool screenShot = false;
bool recording = false;

std::vector<UnitMapSimulation *> unitSims;

TemplateAStar<graphState, graphMove, GraphEnvironment> astar;

std::vector<TemplateAStar<graphState, graphMove, GraphEnvironment> > astars;
CanonicalGrid::CanonicalGrid *grid;

TemplateAStar<xyLoc, tDirection, MapEnvironment> a1;
TemplateAStar<CanonicalGrid::xyLoc, CanonicalGrid::tDirection, CanonicalGrid::CanonicalGrid> a2;
MapEnvironment *ma1 = 0;
CanonicalGrid::CanonicalGrid *ma2 = 0;

std::vector<xyLoc> path;
std::vector<CanonicalGrid::xyLoc> path2;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1000, 1000);
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
		//ht_chantry.arl.map // den012d
		//map = new Map("/Users/nathanst/hog2/maps/dao/orz101d.map");
		//map = new Map("/Users/nathanst/hog2/maps/dao/orz107d.map");
		//map = new Map("/Users/nathanst/hog2/maps/dao/lak308d.map");
		//map = new Map("/Users/nathanst/hog2/maps/da2/ht_chantry.map");
		//map = new Map("/Users/nathanst/hog2/maps/da2/lt_backalley_g.map");
		
		//map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0011SR.map");
		//map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0012SR.map");

		map = new Map("/Users/nathanst/hog2/maps/random/random512-35-6.map");
		//map = new Map("/Users/nathanst/hog2/maps/rooms/8room_000.map");
		//map = new Map("/Users/nathanst/hog2/maps/mazes/maze512-16-0.map");
		
		//map = new Map("/Users/nathanst/hog2/maps/local/weight.map");
		//map = new Map("weight.map");
//		map = new Map(100, 100);
//		map->SetTerrainType(25, 25, kTrees);
//		map->SetTerrainType(75, 75, kTrees);
//		map->SetTerrainType(75, 25, kTrees);
//		map->SetTerrainType(25, 75, kTrees);
//		map = new Map(mazeSize, mazeSize);
//		MakeMaze(map, 5);
//		map->Scale(512, 512);
	}
	else {
		map = new Map(gDefaultMap);
		//map->Scale(512, 512);
	}
	map->SetTileSet(kWinter);

	unitSims.resize(id+1);
	unitSims[id] = new UnitSimulation<xyLoc, tDirection, MapEnvironment>(new MapEnvironment(map));
	unitSims[id]->SetStepType(kMinTime);
	SetNumPorts(id, 1);
	grid = new CanonicalGrid::CanonicalGrid(map);
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record the screen.", kNoModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kNoModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Change weight", "Change the search weight", kNoModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Reopen", "Toggle re-opening policy.", kNoModifier, 't');
	InstallKeyboardHandler(MyDisplayHandler, "Rotate Compression", "Rotate Compression being shown in heuristic", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Rotate Displayed Heuristic", "Rotate which heuristic is shown", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Reset Rotations", "Reset the current rotation/translation of the map.", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');
	
	InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a randomly moving unit", kShiftDown, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, '1');
	
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
		
		delete ma1;
		ma1 = 0;
		delete ma2;
		ma2 = 0;
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
		//SetZoom(windowID, 10);
	}

}

// 1. resize window to 1024x768 

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (viewport == 0)
	{
		unitSims[windowID]->StepTime(1.0/30.0);
	}

	CanonicalGrid::xyLoc gLoc(px1, py1);
	std::deque<CanonicalGrid::xyLoc> queue;
	queue.push_back(gLoc);
	std::vector<CanonicalGrid::xyLoc> v;
	std::vector<bool> visited(grid->GetMap()->GetMapHeight()*grid->GetMap()->GetMapWidth());
	while (!queue.empty())
	{
		grid->GetSuccessors(queue.front(), v);
		for (auto &s : v)
		{
			if (!visited[s.x+s.y*grid->GetMap()->GetMapWidth()])
			{
				grid->SetColor(0.0, 0.0, 0.0);
				queue.push_back(s);
			}
			else {
				grid->SetColor(1.0, 0.0, 0.0);
			}
			grid->GLDrawLine(queue.front(), s);
			visited[s.x+s.y*grid->GetMap()->GetMapWidth()] = true;
		}
		queue.pop_front();
	}
	
	unitSims[windowID]->OpenGLDraw();
	
	if (screenShot)
	{
		SaveScreenshot(windowID, gDefaultMap);
		exit(0);
	}

	if (mouseTracking)
	{
		glBegin(GL_LINES);
		glColor3f(1.0f, 0.0f, 0.0f);
		Map *m = unitSims[windowID]->GetEnvironment()->GetMap();
		GLdouble x, y, z, r;
		m->GetOpenGLCoord(px1, py1, x, y, z, r);
		glVertex3f(x, y, z-3*r);
		m->GetOpenGLCoord(px2, py2, x, y, z, r);
		glVertex3f(x, y, z-3*r);
		glEnd();
	}
	
	if ((ma1) && (viewport == 0)) // only do this once...
	{
		ma1->SetColor(0.0, 0.5, 0.0, 0.75);
		if (runningSearch1 && !unitSims[windowID]->GetPaused())
		{
			ma1->SetColor(0.0, 0.0, 1.0, 0.75);
			for (int x = 0; x < gStepsPerFrame; x++)
			{
				if (a1.DoSingleSearchStep(path))
				{
					printf("Solution: moves %d, length %f, %lld nodes, %u on OPEN\n",
						   (int)path.size(), ma1->GetPathLength(path), a1.GetNodesExpanded(),
						   a1.GetNumOpenItems());
					runningSearch1 = false;
					break;
				}
			}
		}
		a1.OpenGLDraw();
	}
//	if ((ma2) && viewport == 1)
//	{
//		ma2->SetColor(1.0, 0.0, 0.0, 0.5);
//		if (runningSearch2)
//		{
//			ma2->SetColor(1.0, 0.0, 1.0, 0.5);
//			for (int x = 0; x < gStepsPerFrame; x++)
//			{
//				if (a2.DoSingleSearchStep(path))
//				{
//					printf("Solution: moves %d, length %f, %lld nodes\n",
//						   (int)path.size(), ma1->GetPathLength(path), a2.GetNodesExpanded());
//					runningSearch2 = false;
//					break;
//				}
//			}
//		}
//		a2.OpenGLDraw();
//	}

	
	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		static int cnt = 0;
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/%d%d%d%d", (cnt/1000)%10, (cnt/100)%10, (cnt/10)%10, cnt%10);
		SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
		cnt++;
	}
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp( argument[0], "-map" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		strncpy(gDefaultMap, argument[1], 1024);
		return 2;
	}
	return 0;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '|': resetCamera(); break;
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
		case 't':
			reopenNodes = !reopenNodes;
			if (reopenNodes) printf("Will re-open nodes\n");
			if (!reopenNodes) printf("Will not re-open nodes\n");
			break;
		case 'w':
			if (searchWeight == 0)
				searchWeight = 1.0;
			else if (searchWeight == 1.0)
				searchWeight = 10.0;
			else if (searchWeight == 10.0)
				searchWeight = 0;
			printf("Search weight is %1.2f\n", searchWeight);
			break;
		case '[': if (gStepsPerFrame >= 2) gStepsPerFrame /= 2; break;
		case ']': gStepsPerFrame *= 2; break;
		case '{':
		case '}':
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
		{
			if (runningSearch1)
			{
				if (a1.DoSingleSearchStep(path))
				{
					printf("Solution: moves %d, length %f, %lld nodes\n",
						   (int)path.size(), ma1->GetPathLength(path), a1.GetNodesExpanded());
					runningSearch1 = false;
				}
			}
			if (runningSearch2)
			{
				if (a2.DoSingleSearchStep(path2))
				{
					printf("Solution: moves %d, length %f, %lld nodes\n",
						   (int)path.size(), ma1->GetPathLength(path), a2.GetNodesExpanded());
					runningSearch2 = false;
				}
			}
		}
			break;
		default:
			break;
	}
}

bool IsDirectHReachable(xyLoc a, xyLoc b, std::vector<std::vector<uint8_t> > &world)
{
	MapEnvironment *env = unitSims[0]->GetEnvironment();
	double h = env->HCost(a, b);
	int minx = std::min(a.x, b.x);
	int maxx = std::max(a.x, b.x);
	int miny = std::min(a.y, b.y);
	int maxy = std::max(a.y, b.y);
	for (int x = minx; x <= maxx; x++)
	{
		for (int y = miny; y <= maxy; y++)
		{
			xyLoc tmp(x, y);
			// Are we inside the shortest-path region
			if (fequal(env->HCost(a, tmp)+env->HCost(tmp, b), h))
			{
				if (tmp == a || tmp == b)
					continue;
				if (world[x][y] != 0) // == 1 (obstacles)
					return false;
			}
		}
	}
	return true;
}

void MyRandomUnitKeyHandler(unsigned long w, tKeyboardModifier , char)
{
	
}


void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
//	return false;
	static point3d startLoc;
	if (mType == kMouseDown)
	{
		switch (button)
		{
			case kRightButton: printf("Right button\n"); break;
			case kLeftButton: printf("Left button\n"); break;
			case kMiddleButton: printf("Middle button\n"); break;
		}
	}
	mouseTracking = false;
	if (button == kRightButton)
	{
		switch (mType)
		{
			case kMouseDown:
				unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px1, py1);
				startLoc = loc;
				//printf("Mouse down at (%d, %d)\n", px1, py1);
				break;
			case kMouseDrag:
				mouseTracking = true;
				unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px2, py2);
				//printf("Mouse tracking at (%d, %d)\n", px2, py2);
				break;
			case kMouseUp:
			{
				if ((px1 == -1) || (px2 == -1))
					break;
				unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px2, py2);

				printf("Searching from (%d, %d) to (%d, %d)\n", px1, py1, px2, py2);
				
				if (ma1 == 0)
				{
					ma1 = new MapEnvironment(unitSims[windowID]->GetEnvironment()->GetMap());
				}
				if (ma2 == 0)
				{
					ma2 = new CanonicalGrid::CanonicalGrid(unitSims[windowID]->GetEnvironment()->GetMap());
					ma2->SetEightConnected();
				}
				
				a1.SetStopAfterGoal(true);
				a2.SetStopAfterGoal(true);
				//a2.SetWeight(1.8);
				xyLoc s1;
				xyLoc g1;
				s1.x = px1; s1.y = py1;
				g1.x = px2; g1.y = py2;
				CanonicalGrid::xyLoc s2(px1, py1), g2(px2, py2);
				
				
				{
					a1.SetWeight(1.0);
					a2.SetWeight(1.0);

					Timer t1, t2;
					t1.StartTimer();
					a1.GetPath(ma1, s1, g1, path);
					t1.EndTimer();
					printf("Regular search: %1.4fs, path length %1.6f, %llu nodes\n", t1.GetElapsedTime(), ma1->GetPathLength(path), a1.GetNodesExpanded());
//					for (const auto &s : path)
//						std::cout << s << " ";
//					std::cout << "\n";
					t2.StartTimer();
					a2.GetPath(ma2, s2, g2, path2);
					t2.EndTimer();
					printf("Canonical search: %1.4fs, path length %1.6f, %llu nodes\n", t2.GetElapsedTime(), ma2->GetPathLength(path2), a2.GetNodesExpanded());
					printf("Speedup: %1.5f\n", t1.GetElapsedTime()/t2.GetElapsedTime());
					if (!fequal(ma2->GetPathLength(path2), ma1->GetPathLength(path)))
					{
						printf("ERROR: path lengths do not match\n");
						exit(0);
					}
					
//					for (const auto &s : path2)
//						std::cout << s << " ";
//					std::cout << "\n";
				}

				a1.SetWeight(searchWeight);
				ma1->SetEightConnected();

				a1.InitializeSearch(ma1, s1, g1, path);
				a2.InitializeSearch(ma2, s2, g2, path2);

				runningSearch1 = true;
				runningSearch2 = false;
				SetNumPorts(windowID, 1);
//				cameraMoveTo((startLoc.x+loc.x)/2, (startLoc.y+loc.y)/2, -4.0, 1.0, 0);
//				cameraMoveTo((startLoc.x+loc.x)/2, (startLoc.y+loc.y)/2, -4.0, 1.0, 1);
			}
				break;
		}
		return true;
	}
	return false;
}

