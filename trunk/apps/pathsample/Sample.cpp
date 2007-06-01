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
//#include "AStar.h"
#include "PRAStar.h"
#include "SearchUnit.h"
//#include "SharedAMapGroup.h"
#include "MapCliqueAbstraction.h"
#include "NodeLimitAbstraction.h"
#include "MapQuadTreeAbstraction.h"
//#include "RadiusAbstraction.h"
//#include "MapFlatAbstraction.h"
//#include "ClusterAbstraction.h"
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include "Plot2D.h"
#include "Map2DEnvironment.h"
#include "RandomUnits.h"
#include "CFOptimalRefinement.h"

bool mouseTracking;
int px1, py1, px2, py2;
int absType = 0;

std::vector<UnitAbsMapSimulation *> unitSims;
CFOptimalRefinement *CFOR = 0;
//unit *cameraTarget = 0;

Plotting::Plot2D *plot = 0;
Plotting::Line *distLine = 0;

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
		map = new Map(40, 40);
		MakeMaze(map, 1);
	}
	else
		map = new Map(gDefaultMap);

	unitSims.resize(id+1);
	//unitSims[id] = new EpisodicSimulation<xyLoc, tDirection, AbsMapEnvironment>(new AbsMapEnvironment(new MapQuadTreeAbstraction(map, 2)));
	//unitSims[id] = new EpisodicSimulation<xyLoc, tDirection, AbsMapEnvironment>(new AbsMapEnvironment(new NodeLimitAbstraction(map, 8)));
	unitSims[id] = new EpisodicSimulation<xyLoc, tDirection, AbsMapEnvironment>(new AbsMapEnvironment(new MapCliqueAbstraction(map)));
	unitSims[id]->SetStepType(kMinTime);
//	unitSim = new UnitSimulation<xyLoc, tDirection, MapEnvironment>(new MapEnvironment(map),
//																																 (OccupancyInterface<xyLoc, tDirection>*)0);
//	if (absType == 0)
//		unitSim = new unitSimulation(new MapCliqueAbstraction(map));
//	else if (absType == 1)
//		unitSim = new unitSimulation(new RadiusAbstraction(map, 1));
//	else if (absType == 2)
//		unitSim = new unitSimulation(new MapQuadTreeAbstraction(map, 2));
//	else if (absType == 3)
//		unitSim = new unitSimulation(new ClusterAbstraction(map, 8));
	
	//unitSim->setCanCrossDiagonally(true);
	//unitSim = new unitSimulation(new MapFlatAbstraction(map));
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
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, 1);

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
		delete CFOR;
		CFOR = 0;
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
		if ((!unitSims[windowID]->GetPaused()) && CFOR)
			CFOR->DoOneSearchStep();

		if (CFOR)
		{
			CFOR->OpenGLDraw();
		}
	}
	unitSims[windowID]->OpenGLDraw(windowID);
	
//	if (viewport == 0)
//	{
//		if (plot)
//		{
//			if (distLine && cameraTarget)
//			{
//				MapAbstraction *ma = unitSim->GetMapAbstraction();
//				int x, y;
//				cameraTarget->getLocation(x, y);
//				node *n1 = ma->GetNodeFromMap(x, y);
//				cameraTarget->GetGoal()->getLocation(x, y);
//				node *n2 = ma->GetNodeFromMap(x, y);
//				distLine->AddPoint(ma->h(n1, n2));
//			}
//			plot->OpenGLDraw();
//			return;
//		}
//	}
//	
//	unitSim->OpenGLDraw();
//	switch (viewport%3)
//	{
//		case 0: //unitSim->GetMap()->OpenGLDraw(kLines); break;
//		case 1: //unitSim->GetMap()->OpenGLDraw(kPoints); break;
//		case 2: unitSim->GetMap()->OpenGLDraw(kPolygons); break;
//	}
//	if ((mouseTracking) && (px1 != -1) && (px2 != -1) && (py1 != -1) && (py2 != -1))
//	{
//		glColor3f(1.0, 1.0, 1.0);
//		GLdouble x1, y1, z1, rad;
//		glBegin(GL_LINES);
//		unitSim->GetMap()->getOpenGLCoord(px1, py1, x1, y1, z1, rad);
//		glVertex3f(x1, y1, z1-rad);
//		unitSim->GetMap()->getOpenGLCoord(px2, py2, x1, y1, z1, rad);
//		glVertex3f(x1, y1, z1-rad);
//		glEnd();
//	}
//	
//	if (viewport != 0)
//		return;
//	
//	char tempStr[255];
//	sprintf(tempStr, "Simulation time elapsed: %1.2f, Display Time: %1.2f",
//					unitSim->getSimulationTime(), unitSim->getDisplayTime());
//	submitTextToBuffer(tempStr);
//	
//	if (cameraTarget)
//	{
//		GLdouble x, y, z, r;
//		GLdouble xx, yy, zz;
//		cameraTarget->getOpenGLLocation(unitSim->GetMap(), x, y, z, r);
//		if (cameraTarget->GetGoal())
//			cameraTarget->GetGoal()->getOpenGLLocation(unitSim->GetMap(), xx, yy, zz, r);
//		else {
//			xx = -x; 
//			yy = -y;
//		}
//		
//		int oldPort = GetActivePort(windowID);
//		SetActivePort(windowID, 1);
//		cameraMoveTo(xx+3*(xx-x), yy+3*(yy-y), z-0.25, 0.05);
//		cameraLookAt(x, y, z, .2);
//		SetActivePort(windowID, oldPort);
//
////		SetActivePort(windowID, 0);
////		cameraMoveTo(x, y, z-250*r, 0.05);
////		cameraLookAt(x, y, z, .2);
////		SetActivePort(windowID, oldPort);
//	}
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
		{
			if (CFOR == 0)
			{
				CFOR = new CFOptimalRefinement();
				while (!CFOR->InitializeSearch(unitSims[windowID]->GetEnvironment()->GetMapAbstraction(),
																			 unitSims[windowID]->GetEnvironment()->GetMapAbstraction()->GetAbstractGraph(0)->getRandomNode(),
																			 unitSims[windowID]->GetEnvironment()->GetMapAbstraction()->GetAbstractGraph(0)->getRandomNode()))
				{}
			}
			if (CFOR->DoOneSearchStep())
				printf("DONE!!!\n");
		}
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
			if (unitSims[windowID])
				unitSims[windowID]->GetEnvironment()->GetMapAbstraction()->ToggleDrawAbstraction(((mod == kControlDown)?10:0)+(key-'0'));
			break;
	}
}

void MyRandomUnitKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	Map *m = unitSims[windowID]->GetEnvironment()->GetMap();
	
	int x1, y1, x2, y2;
	x2 = random()%m->getMapWidth();
	y2 = random()%m->getMapHeight();
	x1 = random()%m->getMapWidth();
	y1 = random()%m->getMapHeight();
	SearchUnit *su1 = new SearchUnit(x1, y1, 0, 0);
	//SearchUnit *su2 = new SearchUnit(random()%m->getMapWidth(), random()%m->getMapHeight(), su1, new praStar());
	SearchUnit *su2 = new SearchUnit(x2, y2, su1, new CFOptimalRefinement());
	//unitSim->AddUnit(su1);
	unitSims[windowID]->AddUnit(su2);
	
//	RandomerUnit *r = new RandomerUnit(random()%m->getMapWidth(), random()%m->getMapHeight());
//	int id = unitSim->AddUnit(r);
//	xyLoc loc;
//	r->GetLocation(loc);
//	printf("Added unit %d at (%d, %d)\n", id, loc.x, loc.y);

//	int x1, y1, x2, y2;
//	unit *u;
//	unitSim->getRandomLocation(x1, y1);
//	unitSim->getRandomLocation(x2, y2);
//	switch (mod)
//	{
//		case kControlDown: unitSim->addUnit(u=new rhrUnit(x1, y1)); break;
//		case kShiftDown: unitSim->addUnit(u=new randomUnit(x1, y1)); break;
//		default:
//			unit *targ;
//	unitSim->addUnit(targ = new unit(x2, y2));
//	unitSim->addUnit(u=new SearchUnit(x1, y1, targ, new praStar())); break;
//	}
//	delete plot;
//	plot = new Plotting::Plot2D();
//	delete distLine;
//	plot->AddLine(distLine = new Plotting::Line("distline"));
//	cameraTarget = u;
//	u->setSpeed(1.0/4.0);
}

void MyPathfindingKeyHandler(unsigned long , tKeyboardModifier , char)
{
//	for (int x = 0; x < ((mod==kShiftDown)?(50):(1)); x++)
//	{
//		if (unitSim->getUnitGroup(1) == 0)
//		{
//			unitSim->addUnitGroup(new SharedAMapGroup(unitSim));
//			unitSim->setmapAbstractionDisplay(2);
//		}
//		int xx1, yy1, xx2, yy2;
//		unitSim->getRandomLocation(xx1, yy1);
//		unitSim->getRandomLocation(xx2, yy2);
//		
//		unit *u, *u2 = new unit(xx2, yy2, 0);
//		
//		praStar *pra = new praStar(); pra->setPartialPathLimit(4);
//		//aStar *pra = new aStar();
//		
//		unitSim->addUnit(u2);
//		u = new SearchUnit(xx1, yy1, u2, pra);
//		// just set the group of the unit, and it will share a map with those
//		// units.
//		unitSim->getUnitGroup(1)->addUnit(u);
//		unitSim->addUnit(u);
//		u->setSpeed(0.5); // time to go 1 distance						
//	}
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	return false;
//	mouseTracking = false;
//	if (button == kRightButton)
//	{
//		switch (mType)
//		{
//			case kMouseDown:
//				unitSim->GetMap()->getPointFromCoordinate(loc, px1, py1);
//				//printf("Mouse down at (%d, %d)\n", px1, py1);
//				break;
//			case kMouseDrag:
//				mouseTracking = true;
//				unitSim->GetMap()->getPointFromCoordinate(loc, px2, py2);
//				//printf("Mouse tracking at (%d, %d)\n", px2, py2);
//				break;
//			case kMouseUp:
//			{
//				if ((px1 == -1) || (px2 == -1))
//					break;
//				unitSim->GetMap()->getPointFromCoordinate(loc, px2, py2);
//				//printf("Mouse up at (%d, %d)\n", px2, py2);
//				unit *u, *u2 = new unit(px2, py2, 0);
//				//praStar *pra = new praStar(); pra->setPartialPathLimit(4);
//				aStar *pra = new aStar();
//				unitSim->addUnit(u2);
//				u = new SearchUnit(px1, py1, u2, pra);
//				unitSim->addUnit(u);
//				u->setSpeed(0.5); // time to go 1 distance						
//			}
//			break;
//		}
//		return true;
//	}
//	return false;
}
