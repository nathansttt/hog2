/*
 * $Id: unitGroup.h,v 1.11 2006/10/18 23:52:38 nathanst Exp $
 *
 *  unitGroup.h
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


#include "bitVector.h"
#include "constants.h"
#include "map.h"
#include "mapAbstraction.h"
#include "reservationProvider.h"
#include "mapProvider.h"
#include "unitSimulation.h"
#include "statCollection.h"

#ifndef UNITGROUP_H
#define UNITGROUP_H

class unit;
class simulationInfo;

/**
 * A unitGroup provides shared memory and computation for all units within the group.
 */

class unitGroup {
public:
	unitGroup(mapProvider *);
	virtual ~unitGroup() {}
	virtual tDirection makeMove(unit *u, mapProvider *, reservationProvider *, simulationInfo *simInfo);
	/** gives the unit group time to think on a regular basis. In a synchronous simluation
		this will be called once at the beginning of each timestep. */
	virtual void think(mapProvider *) { }
	virtual const char *getName() { return "UnitGroupx"; }

	virtual void openGLDraw(mapProvider *, simulationInfo *);
	virtual void addUnit(unit *);
	virtual void removeUnit(unit *);
	virtual mapAbstraction *getMapAbstraction();
	
	/** Inform the given unit of its current/new location */
	virtual void updateLocation(unit *, mapProvider *, int _x, int _y, bool, simulationInfo *);
	/** Is the group done with racing? */
	virtual bool done();
	/** Lets the unit group do what it needs to reset a trial */
	virtual void startNewTrial(statCollection *stats);
	// print stats; if we don't know any, get the unit to do it
	virtual void logStats(statCollection *stats);
	virtual void logFinalStats(statCollection *) {}
	//virtual void printRoundStats(unit *u, FILE *f);

	/** lets the user cycle through the display mode by hitting Shift-Tab */
	virtual void cycleDisplayMode(void) { }
	
	/** lets the user cycle through the display mode by hitting [ and ] */
	virtual void increaseDisplayALevel(void) {  }
	virtual void decreaseDisplayALevel(void) {  }
	
	/** allows you to iterate through units in the group */
	unit *getUnit(unsigned int which);
	int getGroupID() { return id; }

protected:
	//friend void unitSimulation::addUnitGroup(unitGroup *);
	///** This is only for the unit simulation to set when the unitGroup is added. */
	//virtual void setUnitSimulation(unitSimulation *_us, Map *m);
	//Map *map;
	std::vector<unit *> myUnits;
	//unitSimulation *us;
	int id;
	static int groupID;
};

#include "unit.h"

#endif

