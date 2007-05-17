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

#ifndef UNITRACESIMULATION_H
#define UNITRACESIMULATION_H

#include "UnitSimulation.h"

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
	:UnitSimulation<state, action, environment>(e)
	{
		stopOnConvergence = true;
		currRound = 0;
		//setUseBlocking(false);
		allRacesDone = false;
		useTravelLimit = false;
		useMaxRounds = false;
		targetTolerance = 0.0;
		disjunctiveTrialEnd = false;
		verbose = false;
	}
	~EpisodicSimulation() {}
	int AddUnit(Unit<state, action, environment> *u) { allRacesDone = false; return UnitSimulation<state, action, environment>::AddUnit(u); }

	virtual bool Done() { return allRacesDone; }
	void SetStopOnConvergence(bool stop) { stopOnConvergence = stop; }
	void SetTargetTolerance(double x) { targetTolerance = x; }
	double GetTargetTolerance() { return targetTolerance; }
	void SetDisjunctiveTrialEnd(bool value) { disjunctiveTrialEnd = value; }
	//void SetUseSameStart(bool val) { sameStart = val; }
	int GetCurrRound() { return currRound; }
	void SetTravelLimit(double lim) { useTravelLimit = true; travelLimit = lim; }
	void SetTrialLimit(long maxTrials) { useMaxRounds = true; maxRounds = maxTrials; }
	void DisableTravelLimit() { useTravelLimit = false; }

protected:
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
			for (unsigned int t = 0; t < this->units.size(); t++)
			{
				if (verbose) printf("Check unit %d\n",t);
				
				if (!IsUnitRacing(this->units[t]))
					continue;
				
				if (verbose) printf("Unit %d is racing, check if it is on target\n",t);
				
				if (!UnitOnTarget(this->units[t]))
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
			for (unsigned int t = 0; t < this->units.size(); t++)
			{
				if (verbose) printf("Check unit %d\n",t);
				
				if (!IsUnitRacing(this->units[t]))
					continue;
				
				if (verbose) printf("Unit %d is racing, check if it is on target\n",t);
				
				if (UnitOnTarget(this->units[t]))
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
			return;
		}
		
		// we only get into this code if all units are on target...
		
		// Go through all racing units and output stats:
		// - on the distance traveled
		// - on whether the target was actually reached
		for (unsigned int t = 0; t < this->units.size(); t++)
		{
			if (IsUnitRacing(this->units[t]))
			{
				this->stats.AddStat("trialDistanceMoved", this->units[t]->agent->GetName(), 
														(double) this->units[t]->totalDistance);
				this->stats.AddStat("reachedTarget", this->units[t]->agent->GetName(), 
														(long) (UnitOnTargetStatus(this->units[t]) == kReachedTarget));
			}
		}
		
		if (!allRacesDone)
		{
			this->stats.AddStat("Trial End", "Race Simulation", (long)currRound);
			if (verbose)
			{
				for (unsigned int t = 0; t < this->units.size(); t++)
				{
					if (!IsUnitRacing(this->units[t]))
						continue;
					std::cout << "Round ended, moving " << t << " (" << this->units[t]->agent->GetName()
					<< ") back to " << this->units[t]->startState << std::endl;
				}
			}
		}
		
		// the unit simulation tells us if all groups are done
		if ((this->EpisodeDone() && /*sameStart &&*/ stopOnConvergence) ||
				(useMaxRounds && currRound >= maxRounds))
		{
			if (verbose) printf("All trials finished; last trial: %d\n", currRound);
			allRacesDone = true;
			return;
		}
		
		allRacesDone = false;
		if (verbose)
			printf("Continuing trial: %d\n", currRound);
		
		// call void startNewTrial(); on all groups
		for (unsigned int t = 0; t < this->unitGroups.size(); t++)
		{
			this->unitGroups[t]->StartNewTrial(&this->stats);
		}
		
		// call set location on all units back to their starting location
		// set unit times to the current time
		for (unsigned int t = 0; t < this->units.size(); t++)
		{
			// check if the unit is racing. If not then don't reset it
			if (!IsUnitRacing(this->units[t]))
				continue;
			
			// stats.AddStat("distanceMoved", units[t]->agent->GetName(), (double)0);
//			if (this->units[t]->blocking)
//			{
//				bv->set(this->units[t]->curry*map_width+units[t]->currx, 0);
//				bv->set(this->units[t]->starty*map_width+units[t]->startx, 1);
//			}
			this->units[t]->currentState = this->units[t]->startState;
			this->units[t]->agent->GetUnitGroup()->UpdateLocation(this->units[t]->agent, this->env, this->units[t]->startState, false, this);
			this->units[t]->nextTime = this->currTime;
			this->units[t]->totalThinking = 0;
			//			units[t]->thinkStates = 0;
			this->units[t]->totalDistance = 0;
		}
		currRound++;
		// randomize order -- only matters if they are blocking each other
	}

	virtual void DoTimestepCalc(double amount)
	{
		if (!allRacesDone)
			UnitSimulation<state, action, environment>::DoTimestepCalc(amount);
	}
	
	virtual bool EpisodeDone()
	{
			bool isDone = true;
			for (unsigned int t = 0; (t < this->units.size())&&(isDone); t++)
			{
				state loc, goaloc;
				this->units[t]->agent->GetLocation(loc);
				this->units[t]->agent->GetGoal(goaloc);
				isDone = (loc == goaloc);
			}
			return isDone;
	}
	
	tUnitOnTargetStatus UnitOnTargetStatus(UnitInfo<state, action, environment> *u)
	{
		state s1, s2;
		//Unit<state, action, environment> *target = ;
		//if (verbose) printf("Unit %s, its target %s\n", u->agent->GetName(), target->GetName());
		
		if (useTravelLimit && (u->totalDistance > travelLimit)) return kOutOfTravel;
		
		u->agent->GetLocation(s1);
		u->agent->GetGoal(s2);
		if (!fgreater(this->env->GCost(s1, s2),targetTolerance))
			return kReachedTarget;
		
		return kNotOnTarget;
	}
	
	bool UnitOnTarget(UnitInfo<state, action, environment> *u)
	{
		switch (UnitOnTargetStatus(u)) {
			case kNoTarget: case kOutOfTravel: case kReachedTarget: return true;
			case kNotOnTarget: return false;
			default: assert(false); 
		}
		return false;
	}

	bool IsUnitRacing(UnitInfo<state, action, environment> *u)
	{
		return true;
//		return (!u->ignoreOnTarget &&
//						(u->agent->GetGoal() != NULL));
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
