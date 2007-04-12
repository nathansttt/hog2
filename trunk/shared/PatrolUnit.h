/*
 * $Id: patrolUnit.h,v 1.6 2006/09/18 06:19:31 nathanst Exp $
 *
 *  patrolUnit.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 3/23/06.
 *  Copyright 2006 Nathan Sturtevant, University of Alberta. All rights reserved.
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

#include "aStar.h"
#include "praStar.h"
#include "unit.h"

#ifndef PATROLUNIT_H
#define PATROLUNIT_H

class patrolUnit : public unit {
public:
	patrolUnit(int x, int y);
	patrolUnit(int _x, int _y, int numPLocations, unitSimulation* us);
	virtual const char *getName() { return "patrolUnit"; }
	virtual tDirection makeMove(mapProvider *, reservationProvider *, simulationInfo *simInfo);
	void openGLDraw(mapProvider *, simulationInfo *);
	void addPatrolLocation(unit *);
	unit *getTarget();
	void updateLocation(int _x, int _y, bool worked, simulationInfo *)
	{ x = _x; y = _y; if (!worked) { moves.resize(0);	if (currTarget != -1) currTarget = 0; } }
	void logStats(statCollection *stats);
		void logFinalStats(statCollection *stats);
private:
	double goToLoc(mapAbstraction *aMap, int which);
	void addPathToCache(path *p);
	std::vector<tDirection> moves;
	std::vector<unit *> Locs;
	aStar a;
//	praStar a;
	int currTarget;
	int nodesExpanded;
	int nodesTouched;
};

#endif
