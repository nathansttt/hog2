/*
 *  SteeringUnit.h
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

#ifndef STEERAGENT_H
#define STEERAGENT_H

enum tSteeringMode {
	kAlignHeading,
	kAlignSpeed,
	kSeek,
	kFlee,
	kSeparation,
	
	kChase,
	kWander,
	
	kFlock,
};

class SteeringUnit : public Unit<steeringState, steeringAction, SteeringEnvironment> {
	
public:
	SteeringUnit(steeringState &s, float targetx, float targety)
	{
		currentLoc = s;
		goalLoc.x = targetx;
		goalLoc.y = targety;
		mode = kWander;
		target = 0;
		avoiding = false;
		turnSpeed = 16.0;
	}
	virtual ~SteeringUnit() { }
	const char *GetName() { return "SteeringUnit"; }
	void SetTarget(float targetx, float targety);
	void SetTarget(Unit<steeringState, steeringAction, SteeringEnvironment> *t)
	{target = t;}
	void SetMode(tSteeringMode m)
	{ mode = m; }
	bool Done()
	{
		return false;
	}
	void StartNewTrial(StatCollection *) { }
	
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
	steeringAction GetAlignMove();
	steeringAction GetSeekMove();
	steeringAction GetFleeMove();
	steeringAction GetMatchSpeedMove();
	steeringAction GetSeparationMove(SimulationInfo<steeringState, steeringAction, SteeringEnvironment> *);
	steeringAction GetWanderMove();
	steeringAction GetChaseMove();
	steeringAction GetFlockMove(SimulationInfo<steeringState, steeringAction, SteeringEnvironment> *);

	steeringState currentLoc, goalLoc;
	tSteeringMode mode;
	bool avoiding;
	float turnSpeed;
	Unit<steeringState, steeringAction, SteeringEnvironment> *target;
};

#endif
