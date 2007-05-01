/*
 * $Id: unitRaceSimulation.cpp,v 1.29 2007/02/21 02:31:27 bulitko Exp $
 *
 *  unitRaceSimulation.cpp
 *  HOG
 *
 *  Created by Nathan Sturtevant on 12/17/04.
 *  Copyright 2004 University of Alberta. All rights reserved.
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

#include "UnitRaceSimulation.h"
#include "Unit.h"
#include "UnitGroup.h"
#include "FPUtil.h"

static const bool verbose = false;

unitRaceSimulation::unitRaceSimulation(mapAbstraction *m, bool keepStats)
:unitSimulation(m, keepStats)
{
	stopOnConvergence = true;
	currRound = 0;
	setUseBlocking(false);
	allRacesDone = false;
	sameStart = true;
	useTravelLimit = false;
	useMaxRounds = false;
	targetTolerance = 0.0;
	setAsynchronous();
	disjunctiveTrialEnd = false;
}

unitRaceSimulation::~unitRaceSimulation()
{
}

void unitRaceSimulation::doTimestepCalc()
{
	//	for (unsigned int t = 0; t < units.size(); t++)
	//	{
	//		if (isUnitRacing(units[t]))
	//			printf("%s totalTrialDistanceMoved (so far) %1.2f\n", units[t]->agent->getName(), 
	//						  (double)units[t]->moveDist);
	//	}
	
	if (!allRacesDone)
		unitSimulation::doTimestepCalc();
}

void unitRaceSimulation::doPreTimestepCalc()
{
	if (allRacesDone)
		return;
	
	if (verbose)
		printf("unitRaceSimulation::doPreTimestepCalc checking onTarget\n");
	
	bool trialEnd;
	
	// Trial end depends on the mode we are in
	if (!disjunctiveTrialEnd)
	{
		// In the default conjunctive mode a trial ends when all
		// racing units reach their targets
		trialEnd = true;
		// check to see if all units are on top of their target.
		// we need to modify this if we are ever to use blocking.
		// (or unblock units whenver they reach their target?)
		for (unsigned int t = 0; t < units.size(); t++)
		{
			if (verbose) printf("Check unit %d\n",t);
			
			if (!isUnitRacing(units[t]))
				continue;
			
			if (verbose) printf("Unit %d is racing, check if it is on target\n",t);
			
			if (!unitOnTarget(units[t]))
			{
				if (verbose) printf("Unit %d is not on target, break\n",t);
				trialEnd = false;
				break;
			}
		}
	}
	else {
		// In the disjunctive mode a trial ends when at least
		// one racing unit reaches its target
		trialEnd = false;
		// check to see if all units are on top of their target.
		// we need to modify this if we are ever to use blocking.
		// (or unblock units whenver they reach their target?)
		for (unsigned int t = 0; t < units.size(); t++)
		{
			if (verbose) printf("Check unit %d\n",t);
			
			if (!isUnitRacing(units[t]))
				continue;
			
			if (verbose) printf("Unit %d is racing, check if it is on target\n",t);
			
			if (unitOnTarget(units[t]))
			{
				if (verbose) printf("Unit %d is on target, break\n",t);
				trialEnd = true;
				break;
			}
		}
	}
	// if all units are on their target, then start the next round
	if (!trialEnd)
	{
		updateMap();
		return;
	}
	
	// we only get into this code if all units are on target...
	
	// Go through all racing units and output stats:
	// - on the distance traveled
	// - on whether the target was actually reached
	for (unsigned int t = 0; t < units.size(); t++)
	{
		if (isUnitRacing(units[t]))
		{
			stats.addStat("trialDistanceMoved", units[t]->agent->getName(), 
										(double) units[t]->moveDist);
			stats.addStat("reachedTarget", units[t]->agent->getName(), 
										(long) (unitOnTargetStatus(units[t]) == kReachedTarget));
		}
	}
	
	if (!allRacesDone)
	{
		stats.addStat("Trial End", "Race Simulation", (long)currRound);
		if (verbose)
		{
			for (unsigned int t = 0; t < units.size(); t++)
			{
				if (!isUnitRacing(units[t]))
					continue;
				printf("Round ended, moving %d back to (%d, %d)\n",
							 t, units[t]->startx, units[t]->starty);
			}
		}
	}
	
	// the unit simulation tells us if all groups are done
	if ((unitSimulation::done() && sameStart && stopOnConvergence) || (useMaxRounds && currRound >= maxRounds))
	{
		if (verbose) printf("All trials finished; last trial: %d\n", currRound);
		allRacesDone = true;
		return;
	}
	
	allRacesDone = false;
	if (verbose)
		printf("Continuing trial: %d\n", currRound);
	
	// call void startNewTrial(); on all groups
	for (unsigned int t = 0; t < unitGroups.size(); t++)
	{
		unitGroups[t]->startNewTrial(&stats);
	}
	
	if (!sameStart)
	{
		int xx1, yy1, xx2, yy2;
		do {
			do {
				xx1 = random()%map_width;
				yy1 = random()%map_height;
			}	while ((bv->get(yy1*map_width+xx1)) ||
							 (map->getTerrainType(xx1, yy1) != kGround));
			xx2 = random()%map_width;
			yy2 = random()%map_height;
		}	while ((bv->get(yy2*map_width+xx2)) ||
						 (map->getTerrainType(xx2, yy2) != kGround) ||
						 ((xx1==xx2)||(yy1==yy2)) ||
						 (!getMapAbstraction()->pathable(getMapAbstraction()->getNodeFromMap(xx1, yy1),
																						 getMapAbstraction()->getNodeFromMap(xx2, yy2))));
		for (unsigned int t = 0; t < units.size(); t++)
		{
			if (isUnitRacing(units[t]))
			{
				units[t]->startx = xx1;
				units[t]->starty = yy1;
				
				unitInfo *target = findUnit(units[t]->agent->getTarget());
				target->startx = xx2;
				target->starty = yy2;
				target->currx = xx2;
				target->curry = yy2;
				target->agent->updateLocation(xx2, yy2, false, this);
			}
			else {
				units[t]->startx = random()%map_width;
				units[t]->starty = random()%map_height;
			}
		}
		for (unsigned int t = 0; t < displayUnits.size(); t++)
		{
			displayUnits[t]->startx = xx2;
			displayUnits[t]->starty = yy2;
			displayUnits[t]->currx = xx2;
			displayUnits[t]->curry = yy2;
			displayUnits[t]->agent->updateLocation(xx2, yy2, false, this);
		}				
		if (verbose) printf("Race: (%d, %d) to (%d, %d)\n", xx1, yy1, xx2, yy2);
	}
	
	// call set location on all units back to their starting location
	// set unit times to the current time
	for (unsigned int t = 0; t < units.size(); t++)
	{
		// check if the unit is racing. If not then don't reset it
		if (!isUnitRacing(units[t]))
			continue;
		
		// stats.addStat("distanceMoved", units[t]->agent->getName(), (double)0);
		if (units[t]->blocking)
		{
			bv->set(units[t]->curry*map_width+units[t]->currx, 0);
			bv->set(units[t]->starty*map_width+units[t]->startx, 1);
		}
		units[t]->agent->updateLocation(units[t]->startx, units[t]->starty, false, this);
		units[t]->currx = units[t]->startx;
		units[t]->curry = units[t]->starty;
		units[t]->nextTime = currTime;
		units[t]->thinkTime = 0;
		//			units[t]->thinkStates = 0;
		units[t]->moveDist = 0;
	}
	currRound++;
	// randomize order -- only matters if they are blocking each other
	
	updateMap();
}


/** Checks and returns details of whether a racing unit is on target 
kNoTarget, kOutOfTravel, kReachedTarget, kNotOnTarget
*/
tUnitOnTargetStatus unitRaceSimulation::unitOnTargetStatus(unitInfo *u)
{
	int x, y;
	unit *target = u->agent->getTarget();
	if (verbose) printf("Unit %s, its target %s\n",u->agent->getName(),target->getName());
	
	if (useTravelLimit && (u->moveDist > travelLimit)) return kOutOfTravel;
	
	if (target == NULL) return kNoTarget;
	
	target->getLocation(x, y);
	if (!fgreater(aMap->octileDistance(u->currx,u->curry,x,y),targetTolerance))
		return kReachedTarget;
	
	return kNotOnTarget;
}


/** A racing unit is on-target iff:
- it has no target or
- it uses a travel limit and it used it up or
- it is within tolerance of its target
	*/
bool unitRaceSimulation::unitOnTarget(unitInfo *u)
{
	switch (unitOnTargetStatus(u)) {
		case kNoTarget: case kOutOfTravel: case kReachedTarget: return true;
		case kNotOnTarget: return false;
		default: assert(false); 
	}
	return false;
}



/** Check if the unit is racing */
bool unitRaceSimulation::isUnitRacing(unitInfo *u)
{
	return (!u->ignoreOnTarget && (u->agent->getObjectType() == kWorldObject) &&
					(u->agent->getTarget() != NULL));
}
