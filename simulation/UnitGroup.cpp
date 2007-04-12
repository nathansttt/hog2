/*
 * $Id: unitGroup.cpp,v 1.12 2006/10/18 23:52:38 nathanst Exp $
 *
 *  unitGroup.cpp
 *  HOG
 *
 *  Created by Nathan Sturtevant on 12/9/04.
 *  Copyright 2004 Nathan Sturtevant, University of Alberta. All rights reserved.
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

#include "graph.h"
#include "unitGroup.h"
#include "unit.h"

int unitGroup::groupID = 0;

unitGroup::unitGroup(mapProvider *)
{
  id = unitGroup::groupID++;
}

/**
* This function returns the mapAbstraction built by this group; 0 if none is built.
 */
mapAbstraction *unitGroup::getMapAbstraction()
{
	return 0; //us->getMapAbstraction(kUnitSimulationMap);
}

void unitGroup::addUnit(unit *u)
{
	// Check if we already have this unit
	for (unsigned int x = 0; x < myUnits.size(); x++)
		if (myUnits[x] == u)
			return;

	// If not then add the unit to the group and set the unit's group
	myUnits.push_back(u);
	u->setUnitGroup(this);
}

void unitGroup::removeUnit(unit *u)
{
	for (unsigned int x = 0; x < myUnits.size(); x++)
	{
		if (myUnits[x] == u)
		{
			u->setUnitGroup(0);
			myUnits[x] = myUnits[myUnits.size()-1];
			myUnits.pop_back();
		}
	}
}

tDirection unitGroup::makeMove(unit *u, mapProvider *mp, reservationProvider *rp, simulationInfo *simInfo)
{
	return u->makeMove(mp, rp, simInfo);
}

void unitGroup::openGLDraw(mapProvider *mp, simulationInfo *)
{
	Map *map = mp->getMap();
	glBegin(GL_LINES);
	glColor3f(.5, .5, .5);
	for (unsigned int t = 0; t < myUnits.size(); t++)
	{
		GLdouble x, y, z, rad;
		myUnits[t]->getOpenGLLocation(map, x, y, z, rad);
		// draw a vertical line above each unit in our group
		glVertex3f(x, y, z);
		glVertex3f(x, y+2*rad, z);
	}
	glEnd();
}

/** Inform the given unit of its current/new location */
void unitGroup::updateLocation(unit *u, mapProvider *, int _x, int _y,
															 bool success, simulationInfo *simInfo)
{
	u->updateLocation(_x, _y, success, simInfo);
}


/** Is the group done with their exploration? */
bool unitGroup::done()
{
	for (unsigned int x = 0; x < myUnits.size(); x++)
		if (!myUnits[x]->done())
			return false;
	return true;
}

/** Lets the unit group do what it needs to reset a trial */
void unitGroup::startNewTrial(statCollection *)
{
}

void unitGroup::logStats(statCollection *)
{
	//u->logStats(stats);
	//u->printRoundStats(f);
}

/** allows you to iterate through units in the group */
unit *unitGroup::getUnit(unsigned int which)
{
	if (which < myUnits.size())
		return myUnits[which];
	return 0;
}
