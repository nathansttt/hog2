/*
 *  FleeUnit.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/12/11.
 *  Copyright 2011 University of Denver. All rights reserved.
 *
 */

#include "Unit.h"
#include <deque>
#include "FPUtil.h"
#include "SteeringEnvironment.h"

#ifndef FLEEAGENT_H
#define FLEEAGENT_H

class FleeUnit : public Unit<steeringState, steeringAction, SteeringEnvironment> {
	
public:
	FleeUnit(steeringState &s, float targetx, float targety)
	{
		currentLoc = s;
		goalLoc.x = targetx;
		goalLoc.y = targety;
	}
	virtual ~FleeUnit() { }
	const char *GetName() { return "FleeUnit"; }
	void SetTarget(float targetx, float targety)
	{
		goalLoc.x = targetx*worldRadius;
		goalLoc.y = targety*worldRadius;
	}
	bool Done()
	{
		return false;
	}
	void StartNewTrial(StatCollection *s) { }
	
	bool MakeMove(SteeringEnvironment *, OccupancyInterface<steeringState, steeringAction> *, SimulationInfo<steeringState, steeringAction, SteeringEnvironment> *, steeringAction& a);
	
	void OpenGLDraw(const SteeringEnvironment *e, const SimulationInfo<steeringState, steeringAction, SteeringEnvironment> *) const;
	void UpdateLocation(SteeringEnvironment *, steeringState &s, bool success, SimulationInfo<steeringState, steeringAction, SteeringEnvironment> *)
	{
		if (success) 
		{
			//std::cout << "Updating location from " << currentLoc << " to " << s << std::endl;
			currentLoc = s;
		}
		else {
			//std::cout << "Move failed, not updating location" << std::endl;
		}
	}
	void GetGoal(steeringState &s) { s = goalLoc; }
	void GetLocation(steeringState &s) { s = currentLoc; }
	virtual void LogFinalStats(StatCollection *)
	{ }
private:
	steeringState currentLoc, goalLoc;
};

#endif
