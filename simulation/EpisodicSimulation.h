/*
 * $Id: EpisodicSimulation.h,v 1.18 2006/11/21 02:56:47 bulitko Exp $
 *
 *  EpisodicSimulation.h
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

#include "UnitSimulation.h"

#ifndef UNITRACESIMULATION_H
#define UNITRACESIMULATION_H

/**
 * A class where units repeatedly path from a start to a goal until they have
 * stopped exploring/learning/etc.
 */


/** Possible status of checking if a unit is on target */
enum tUnitOnTargetStatus {
	kNoTarget, kOutOfTravel, kReachedTarget, kNotOnTarget
};

template<class state, class action, class environment>
class EpisodicSimulation : public UnitSimulation<state, action, environment> {
public:
	EpisodicSimulation(environment *e)
	:UnitSimulation(e)
	{
		stopOnConvergence = true;
		currRound = 0;
		setUseBlocking(false);
		allRacesDone = false;
		useTravelLimit = false;
		useMaxRounds = false;
		targetTolerance = 0.0;
		disjunctiveTrialEnd = false;
		verbose = false;
	}
	~EpisodicSimulation() {}
	void AddUnit(Unit<state, action, environment> *u) { allRacesDone = false; unitSimulation::addUnit(u); }

	virtual bool Done() { return allRacesDone; }
	void SetStopOnConvergence(bool stop) { stopOnConvergence = stop; }
	void SetTargetTolerance(double x) { targetTolerance = x; }
	double GetTargetTolerance() { return targetTolerance; }
	void SetDisjunctiveTrialEnd(bool value) { disjunctiveTrialEnd = value; }
	void SetUseSameStart(bool val) { sameStart = val; }
	int GetCurrRound() { return currRound; }
	void SetTravelLimit(double lim) { useTravelLimit = true; travelLimit = lim; }
	void SetTrialLimit(long maxTrials) { useMaxRounds = true; maxRounds = maxTrials; }
	void DisableTravelLimit() { useTravelLimit = false; }

private:
	virtual void DoPreTimestepCalc()
	{
		if (allRacesDone)
			return;
		
		if (verbose)
			printf("EpisodicSimulation::doPreTimestepCalc checking onTarget\n");
		
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
				
				if (!UnitOnTarget(units[t]))
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
				
				if (UnitOnTarget(units[t]))
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
				stats.AddStat("trialDistanceMoved", units[t]->agent->getName(), 
											(double) units[t]->moveDist);
				stats.AddStat("reachedTarget", units[t]->agent->getName(), 
											(long) (UnitOnTargetStatus(units[t]) == kReachedTarget));
			}
		}
		
		if (!allRacesDone)
		{
			stats.AddStat("Trial End", "Race Simulation", (long)currRound);
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
		
		// call set location on all units back to their starting location
		// set unit times to the current time
		for (unsigned int t = 0; t < units.size(); t++)
		{
			// check if the unit is racing. If not then don't reset it
			if (!isUnitRacing(units[t]))
				continue;
			
			// stats.AddStat("distanceMoved", units[t]->agent->getName(), (double)0);
//			if (units[t]->blocking)
//			{
//				bv->set(units[t]->curry*map_width+units[t]->currx, 0);
//				bv->set(units[t]->starty*map_width+units[t]->startx, 1);
//			}
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
	}

	virtual void DoTimestepCalc()
	{
		if (!allRacesDone)
			UnitSimulation::doTimestepCalc();
	}
	
	tUnitOnTargetStatus UnitOnTargetStatus(UnitInfo<state, action, environment> *u)
	{
		int x, y;
		unit *target = u->agent->GetGoal();
		if (verbose) printf("Unit %s, its target %s\n",u->agent->getName(),target->getName());
		
		if (useTravelLimit && (u->moveDist > travelLimit)) return kOutOfTravel;
		
		if (target == NULL) return kNoTarget;
		
		target->getLocation(x, y);
		if (!fgreater(aMap->octileDistance(u->currx,u->curry,x,y),targetTolerance))
			return kReachedTarget;
		
		return kNotOnTarget;
	}
	
	bool UnitOnTarget(UnitInfo<state, action, environment> *u)
	{
		switch (unitOnTargetStatus(u)) {
			case kNoTarget: case kOutOfTravel: case kReachedTarget: return true;
			case kNotOnTarget: return false;
			default: assert(false); 
		}
		return false;
	}

	bool IsUnitRacing(UnitInfo<state, action, environment> *u)
	{
		return (!u->ignoreOnTarget &&
						(u->agent->GetGoal() != NULL));
	}
	
	bool allRacesDone;
	int currRound;
	double travelLimit;
	bool useTravelLimit;
	bool stopOnConvergence;
	long maxRounds;
	bool useMaxRounds;
	double targetTolerance;
	bool disjunctiveTrialEnd;
	bool verbose;
};

#endif
