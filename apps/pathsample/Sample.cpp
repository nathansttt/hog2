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

bool mouseTracking = false;
bool runningSearch1 = false;
bool runningSearch2 = false;
int px1, py1, px2, py2;
int absType = 0;
int mazeSize = 128;

std::vector<UnitMapSimulation *> unitSims;
TemplateAStar<xyLoc, tDirection, MapEnvironment> a1;
TemplateAStar<xyLoc, tDirection, MapEnvironment> a2;
MapEnvironment *ma1 = 0;
MapEnvironment *ma2 = 0;
GraphDistanceHeuristic *gdh = 0;
std::vector<xyLoc> path;

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
		map = new Map(mazeSize/2, mazeSize/2);
		MakeMaze(map, 1);
		map->scale(mazeSize, mazeSize);
	}
	else
		map = new Map(gDefaultMap);
	map->setTileSet(kWinter);
	
	unitSims.resize(id+1);
	unitSims[id] = new UnitSimulation<xyLoc, tDirection, MapEnvironment>(new MapEnvironment(map));
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

	InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a randomly moving unit", kShiftDown, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, '1');

	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	InstallCommandLineHandler(MyCLHandler, "-size", "-batch integer", "If size is set, we create a square maze with the x and y dimensions specified.");
	
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
	if (viewport == 0)
	{
		unitSims[windowID]->StepTime(1.0/30.0);
	}
	unitSims[windowID]->OpenGLDraw();

	if (mouseTracking)
	{
		glBegin(GL_LINES);
		glColor3f(1.0f, 0.0f, 0.0f);
		Map *m = unitSims[windowID]->GetEnvironment()->GetMap();
		GLdouble x, y, z, r;
		m->getOpenGLCoord(px1, py1, x, y, z, r);
		glVertex3f(x, y, z-3*r);
		m->getOpenGLCoord(px2, py2, x, y, z, r);
		glVertex3f(x, y, z-3*r);
		glEnd();
	}

//	if (gdh)
//		gdh->OpenGLDraw();
	if ((ma1) && (viewport == 0)) // only do this once...
	{
		ma1->SetColor(0.0, 0.5, 0.0, 0.75);
		if (runningSearch1)
		{
			ma1->SetColor(0.0, 0.0, 1.0, 0.75);
			for (int x = 0; x < 50; x++)
			{
				if (a1.DoSingleSearchStep(path))
				{
					printf("Solution: moves %d, length %f\n", (int)path.size(), ma1->GetPathLength(path));
					runningSearch1 = false;
					break;
				}
			}
		}
		a1.OpenGLDraw();
	}
	if ((ma2) && (GetNumPorts(windowID) == 1 || (viewport == 1)))
	{
		ma2->SetColor(1.0, 0.0, 0.0, 0.5);
		if (runningSearch2)
		{
			ma2->SetColor(1.0, 0.0, 1.0, 0.5);
			for (int x = 0; x < 50; x++)
			{
				if (a2.DoSingleSearchStep(path))
				{
					printf("Solution: moves %d, length %f\n", (int)path.size(), ma2->GetPathLength(path));
					runningSearch2 = false;
					break;
				}
			}
		}
		a2.OpenGLDraw();
	}
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if( strcmp( argument[0], "-map" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		strncpy(gDefaultMap, argument[1], 1024);
		return 2;
	}
	else if( strcmp( argument[0], "-size" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		mazeSize = atoi(argument[1]);
		assert( mazeSize > 0 );
		return 2;
	}
	return 2; //ignore typos
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
		{
		}
			break;
		default:
			break;
	}
}

void MyRandomUnitKeyHandler(unsigned long, tKeyboardModifier , char)
{
//	if (ma == 0)
//		return;
//	if (gdh == 0)
//	{
//		gdh = new GraphDistanceHeuristic(GraphSearchConstants::GetGraph(ma->GetMap()));
//		gdh->UseSmartPlacement(true);
//		ma->SetGraphHeuristic(gdh);
//		for (int x = 0; x < 10; x++)
//			gdh->AddHeuristic();
//	}
//	if (mod == kShiftDown)
//		ma->SetGraphHeuristic(0);
//	else if (ma->GetGraphHeuristic() == 0)
//	{
//		ma->SetGraphHeuristic(gdh);
//	}
//	else {
//		gdh->AddHeuristic();
//	}
	
//	Map *m = unitSims[windowID]->GetEnvironment()->GetMap();
//	
//	int x1, y1, x2, y2;
//	x2 = random()%m->getMapWidth();
//	y2 = random()%m->getMapHeight();
//	x1 = random()%m->getMapWidth();
//	y1 = random()%m->getMapHeight();
//	SearchUnit *su1 = new SearchUnit(x1, y1, 0, 0);
//	SearchUnit *su2 = new SearchUnit(x2, y2, su1, new TemplateAStar<xyLoc, tDirection, MapEnvironment>());
//	//unitSim->AddUnit(su1);
//	unitSims[windowID]->AddUnit(su2);
//	
}

void MyPathfindingKeyHandler(unsigned long , tKeyboardModifier , char)
{
//	MapEnvironment *env = unitSims[windowID]->GetEnvironment();
//	Map *m = env->GetMap();
//
//	MapEnvironment ma(m->clone());
//	std::vector<xyLoc> path;
//	TemplateAStar<xyLoc, tDirection, MapEnvironment> a;
//	for (int x = 0; x < 10000; x++)
//	{
//		node *n1, *n2;
//		do {
//			Graph *g = mabs->GetAbstractGraph(0);
//			n1 = g->GetRandomNode();
//			n2 = g->GetRandomNode();
//		} while (!mabs->Pathable(n1, n2));
//		mabs->GetTileFromNode(n1, px1, py1);
//		mabs->GetTileFromNode(n2, px2, py2);
//		xyLoc s1, g1;
//		s1.x = px1; s1.y = py1;
//		g1.x = px2; g1.y = py2;
//		a.GetPath(&ma, s1, g1, path);
//		printf("%d\t%d\t%d\t%d\t%1.5f\n", px1, py1, px2, py2, ma.GetPathLength(path));
//	}
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	mouseTracking = false;
	if (button == kRightButton)
	{
		switch (mType)
		{
			case kMouseDown:
				unitSims[windowID]->GetEnvironment()->GetMap()->getPointFromCoordinate(loc, px1, py1);
				//printf("Mouse down at (%d, %d)\n", px1, py1);
				break;
			case kMouseDrag:
				mouseTracking = true;
				unitSims[windowID]->GetEnvironment()->GetMap()->getPointFromCoordinate(loc, px2, py2);
				//printf("Mouse tracking at (%d, %d)\n", px2, py2);
				break;
			case kMouseUp:
			{
				if ((px1 == -1) || (px2 == -1))
					break;
				unitSims[windowID]->GetEnvironment()->GetMap()->getPointFromCoordinate(loc, px2, py2);
				printf("Searching from (%d, %d) to (%d, %d)\n", px1, py1, px2, py2);
				
				if (ma1 == 0)
				{
					ma1 = new MapEnvironment(unitSims[windowID]->GetEnvironment()->GetMap());
					gdh = new GraphDistanceHeuristic(GraphSearchConstants::GetGraph(ma1->GetMap()));
					gdh->UseSmartPlacement(true);
					ma1->SetGraphHeuristic(gdh);
					for (int x = 0; x < 10; x++)
						gdh->AddHeuristic();
				}
				if (ma2 == 0)
					ma2 = new MapEnvironment(unitSims[windowID]->GetEnvironment()->GetMap());
				a1.SetStopAfterGoal(true);
				a2.SetStopAfterGoal(true);
				xyLoc s1;
				xyLoc g1;
				s1.x = px1; s1.y = py1;
				g1.x = px2; g1.y = py2;
				a1.InitializeSearch(ma1, s1, g1, path);
				a2.InitializeSearch(ma2, s1, g1, path);
				runningSearch1 = true;
				runningSearch2 = true;
			}
			break;
		}
		return true;
	}
	return false;
}
