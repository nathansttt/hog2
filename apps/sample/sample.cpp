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

#include "common.h"
#include "sample.h"
#include "aStar.h"
#include "praStar.h"
#include "searchUnit.h"
#include "sharedAMapGroup.h"
#include "mapCliqueAbstraction.h"
#include "mapQuadTreeAbstraction.h"
#include "radiusAbstraction.h"
#include "mapFlatAbstraction.h"
#include "clusterAbstraction.h"

bool mouseTracking;
int px1, py1, px2, py2;
int absType = 0;

unitSimulation *unitSim = 0;
unit *cameraTarget = 0;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv);
}


/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
void createSimulation()
{
	Map *map;
	if (gDefaultMap[0] == 0)
		map = new Map(60, 60);
	else
		map = new Map(gDefaultMap);

	if (absType == 0)
		unitSim = new unitSimulation(new mapCliqueAbstraction(map));
	else if (absType == 1)
		unitSim = new unitSimulation(new radiusAbstraction(map, 1));
	else if (absType == 2)
		unitSim = new unitSimulation(new mapQuadTreeAbstraction(map, 2));
	else if (absType == 3)
		unitSim = new unitSimulation(new clusterAbstraction(map, 8));
	
	unitSim->setCanCrossDiagonally(true);
	//unitSim = new unitSimulation(new mapFlatAbstraction(map));
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
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		if (unitSim == 0)
			createSimulation();
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *data)
{
	if (windowID != 0)
		return;
	if (viewport == 0)
		unitSim->advanceTime(1.0/30.0);
	unitSim->openGLDraw();
	switch (viewport%3)
	{
		case 0: unitSim->getMap()->openGLDraw(kLines); break;
		case 1: unitSim->getMap()->openGLDraw(kPoints); break;
		case 2: unitSim->getMap()->openGLDraw(kPolygons); break;
	}
	
	char tempStr[255];
	sprintf(tempStr, "Simulation time elapsed: %1.2f, Display Time: %1.2f",
					unitSim->getSimulationTime(), unitSim->getDisplayTime());
	submitTextToBuffer(tempStr);
	
	if (cameraTarget)
	{
		GLdouble x, y, z, r;
		GLdouble xx, yy, zz;
		cameraTarget->getOpenGLLocation(unitSim->getMap(), x, y, z, r);
		if (cameraTarget->getTarget())
			cameraTarget->getTarget()->getOpenGLLocation(unitSim->getMap(), xx, yy, zz, r);
		else {
			xx = -x; 
			yy = -y;
		}
		
		int oldPort = GetActivePort(windowID);
		SetActivePort(windowID, 1);
		cameraMoveTo(xx+3*(xx-x), yy+3*(yy-y), z-0.25, 0.05);
		cameraLookAt(x, y, z, .2);
		SetActivePort(windowID, oldPort);

		SetActivePort(windowID, 0);
		cameraMoveTo(x, y, z-250*r, 0.05);
		cameraLookAt(x, y, z, .2);
		SetActivePort(windowID, oldPort);
	}

	if ((mouseTracking) && (px1 != -1) && (px2 != -1) && (py1 != -1) && (py2 != -1))
	{
		glColor3f(1.0, 1.0, 1.0);
		GLdouble x1, y1, z1, rad;
		glBegin(GL_LINES);
		unitSim->getMap()->getOpenGLCoord(px1, py1, x1, y1, z1, rad);
		glVertex3f(x1, y1, z1-rad);
		unitSim->getMap()->getOpenGLCoord(px2, py2, x1, y1, z1, rad);
		glVertex3f(x1, y1, z1-rad);
		glEnd();
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
		case '\t': SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID)); break;
		case 'p': unitSim->setSimulationPaused(!unitSim->getSimulationPaused()); break;
		case 'o':
			if (unitSim->getSimulationPaused())
			{
				unitSim->setSimulationPaused(false);
				unitSim->advanceTime(.1);
				unitSim->setSimulationPaused(true);
			}
			break;
		case ']': absType = (absType+1)%3; break;
		case '[': absType = (absType+4)%3; break;
		case '{': unitSim->setSimulationPaused(true); unitSim->offsetDisplayTime(-0.5); break;
		case '}': unitSim->offsetDisplayTime(0.5); break;
		default:
			if (unitSim->getMapAbstractionDisplay())
				unitSim->getMapAbstractionDisplay()->toggleDrawAbstraction(((mod == kControlDown)?10:0)+(key-'0'));
			break;
	}
}

void MyRandomUnitKeyHandler(unsigned long windowID, tKeyboardModifier mod, char)
{
	int x1, y1, x2, y2;
	unit *u;
	unitSim->getRandomLocation(x1, y1);
	unitSim->getRandomLocation(x2, y2);
	switch (mod)
	{
		case kControlDown: unitSim->addUnit(u=new rhrUnit(x1, y1)); break;
		case kShiftDown: unitSim->addUnit(u=new randomUnit(x1, y1)); break;
		default:
			unit *targ;
			unitSim->addUnit(targ = new unit(x2, y2));
			unitSim->addUnit(u=new searchUnit(x1, y1, targ, new praStar())); break;
	}
	cameraTarget = u;
	u->setSpeed(1.0/4.0);
}

void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier mod, char)
{
	for (int x = 0; x < ((mod==kShiftDown)?(50):(1)); x++)
	{
		if (unitSim->getUnitGroup(1) == 0)
		{
			unitSim->addUnitGroup(new sharedAMapGroup(unitSim));
			unitSim->setmapAbstractionDisplay(2);
		}
		int xx1, yy1, xx2, yy2;
		unitSim->getRandomLocation(xx1, yy1);
		unitSim->getRandomLocation(xx2, yy2);
		
		unit *u, *u2 = new unit(xx2, yy2, 0);
		
		praStar *pra = new praStar(); pra->setPartialPathLimit(4);
		//aStar *pra = new aStar();
		
		unitSim->addUnit(u2);
		u = new searchUnit(xx1, yy1, u2, pra);
		// just set the group of the unit, and it will share a map with those
		// units.
		unitSim->getUnitGroup(1)->addUnit(u);
		unitSim->addUnit(u);
		u->setSpeed(0.5); // time to go 1 distance						
	}
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	return false;
	mouseTracking = false;
	if (button == kRightButton)
	{
		switch (mType)
		{
			case kMouseDown:
				unitSim->getMap()->getPointFromCoordinate(loc, px1, py1);
				//printf("Mouse down at (%d, %d)\n", px1, py1);
				break;
			case kMouseDrag:
				mouseTracking = true;
				unitSim->getMap()->getPointFromCoordinate(loc, px2, py2);
				//printf("Mouse tracking at (%d, %d)\n", px2, py2);
				break;
			case kMouseUp:
			{
				if ((px1 == -1) || (px2 == -1))
					break;
				unitSim->getMap()->getPointFromCoordinate(loc, px2, py2);
				//printf("Mouse up at (%d, %d)\n", px2, py2);
				unit *u, *u2 = new unit(px2, py2, 0);
				//praStar *pra = new praStar(); pra->setPartialPathLimit(4);
				aStar *pra = new aStar();
				unitSim->addUnit(u2);
				u = new searchUnit(px1, py1, u2, pra);
				unitSim->addUnit(u);
				u->setSpeed(0.5); // time to go 1 distance						
			}
			break;
		}
		return true;
	}
	return false;
}

