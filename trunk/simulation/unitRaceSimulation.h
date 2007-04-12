/*
 * $Id: unitRaceSimulation.h,v 1.18 2006/11/21 02:56:47 bulitko Exp $
 *
 *  unitRaceSimulation.h
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

#include "unitSimulation.h"

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

class unitRaceSimulation : public unitSimulation {
public:
	unitRaceSimulation(mapAbstraction *m, bool keepStats = false);
	~unitRaceSimulation();
	void addUnit(unit *u) { allRacesDone = false; unitSimulation::addUnit(u); }
	void addUnit(unit *u, bool block)  { allRacesDone = false; unitSimulation::addUnit(u, block); }

	virtual bool done() { return allRacesDone; }
	void setStopOnConvergence(bool stop) { stopOnConvergence = stop; }
	void setTargetTolerance(double x) { targetTolerance = x; }
	double getTargetTolerance() { return targetTolerance; }
	void setDisjunctiveTrialEnd(bool value) { disjunctiveTrialEnd = value; }
	void setUseSameStart(bool val) { sameStart = val; }
	int getCurrRound() { return currRound; }
	void setTravelLimit(double lim) { useTravelLimit = true; travelLimit = lim; }
	void setTrialLimit(long maxTrials) { useMaxRounds = true; maxRounds = maxTrials; }
	void disableTravelLimit() { useTravelLimit = false; }
private:
	virtual void doPreTimestepCalc();
	virtual void doTimestepCalc();
	tUnitOnTargetStatus unitOnTargetStatus(unitInfo *);
	bool unitOnTarget(unitInfo *);
	bool isUnitRacing(unitInfo *u);
	bool allRacesDone;
	bool sameStart;
	int currRound;
	double travelLimit;
	bool useTravelLimit;
	bool stopOnConvergence;
	long maxRounds;
	bool useMaxRounds;
	double targetTolerance;
	bool disjunctiveTrialEnd;
};

#endif
