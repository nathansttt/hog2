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

#include "AStar.h"
#include "PRAStar.h"
#include "Unit.h"

#ifndef PATROLUNIT_H
#define PATROLUNIT_H

class patrolUnit : public Unit {
public:
	patrolUnit(int x, int y);
//	patrolUnit(int _x, int _y, int numPLocations, unitSimulation* us);
	virtual const char *GetName() { return "patrolUnit"; }
	virtual tDirection makeMove(MapProvider *, reservationProvider *, SimulationInfo *simInfo);
	void OpenGLDraw(const MapProvider *, const SimulationInfo *) const;
	void addPatrolLocation(unit *);
	unit *GetGoal();
	void updateLocation(int _x, int _y, bool worked, SimulationInfo *)
	{ x = _x; y = _y; if (!worked) { moves.resize(0);	if (currTarget != -1) currTarget = 0; } }
	void LogStats(StatCollection *stats);
		void LogFinalStats(StatCollection *stats);
private:
	double goToLoc(MapAbstraction *aMap, int which);
	void addPathToCache(path *p);
	std::vector<tDirection> moves;
	std::vector<unit *> Locs;
	aStar a;
//	praStar a;
	int currTarget;
	uint64_t nodesExpanded;
	uint64_t nodesTouched;
};

#endif
