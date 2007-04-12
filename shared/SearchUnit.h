/*
 * $Id: searchUnit.h,v 1.13 2006/09/18 06:19:31 nathanst Exp $
 *
 *  Hierarchical Open Graph File
 *
 *  Created by Nathan Sturtevant on 10/4/04.
 *  Copyright 2004 Nathan Sturtevant. All rights reserved.
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

#include "unit.h"
#include "map.h"
#include "searchAlgorithm.h"
#include "spreadExecSearchAlgorithm.h"

#ifndef searchUnit_H
#define searchUnit_H

/**
* A general unit which collects path information from a searchAlgorithm and
* incrementally executes that path in the world
*/

class searchUnit : public unit {
public:
	searchUnit(int x, int y, unit *target, searchAlgorithm *alg);
	searchUnit(int x, int y, unit *target, spreadExecSearchAlgorithm *alg);
	searchUnit(int _x, int _y, int _r, int _g, int _b, unit *_target, searchAlgorithm *alg);
	searchUnit(int _x, int _y, float _r, float _g, float _b, unit *_target, searchAlgorithm *alg);
	virtual ~searchUnit();
	virtual const char *getName() { return algorithm->getName(); }
	virtual searchAlgorithm* getAlgorithm() { return algorithm; }
	//void setUnitSimulation(unitSimulation *_US) { US = _US; algorithm->setSimulationEnvironment(US); }
	virtual bool done() { return onTarget; }
	
	//using unit::makeMove;
	// this is where the World says you are  
	virtual tDirection makeMove(mapProvider *, reservationProvider *, simulationInfo *simInfo); 
	
	virtual void updateLocation(int _x, int _y, bool, simulationInfo *);
	virtual void openGLDraw(mapProvider *, simulationInfo *);
	//void printRoundStats(FILE *f);
	void logStats(statCollection *stats);
	void logFinalStats(statCollection *stats);
protected:
	virtual void addPathToCache(path *p);
	bool getCachedMove(tDirection &dir);
	int nodesExpanded;
	int nodesTouched;
	std::vector<tDirection> moves;
	//	path *p;
	searchAlgorithm *algorithm;
	spreadExecSearchAlgorithm *s_algorithm;
	path *spread_cache;
	double targetTime;
	bool onTarget;
};

#endif
