/*
 *  RandomUnits.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/23/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */


#include "Unit.h"
#include "MapUnit.h"
#include "Map2DEnvironment.h"

#ifndef RANDOMUNITS_H
#define RANDOMUNITS_H

/**
* A unit that moves in random directions changing direction randomly.
 */
class RandomUnit : public MapUnit {
public:
	RandomUnit(int _x, int _y)
	:MapUnit(_x, _y) { lastIndex = 0; }
	virtual const char *GetName() { return "randomUnit"; }

//	virtual bool MakeMove(environment *, OccupancyInterface<state,action> *, MapSimulationInfo *, tDirection& a) = 0;
	bool MakeMove(MapEnvironment *, OccupancyInterface<xyLoc, tDirection> *, MapSimulationInfo *, tDirection &dir);
	void UpdateLocation(MapEnvironment *, xyLoc &, bool, MapSimulationInfo *);
private:
		int lastIndex;
};

/**
* A unit which randomly teleports around the world.
 */
class TeleportUnit : public MapUnit {
public:
	TeleportUnit(int x, int y, int _stayTime)
	:MapUnit(x, y), stayTime(_stayTime), timer(_stayTime) { }
	const char *GetName() { return "TeleportUnit"; }

	bool MakeMove(MapEnvironment *, OccupancyInterface<xyLoc, tDirection> *, MapSimulationInfo *, tDirection &dir);
private:
	int stayTime;
	int timer;
};

/**
* A unit that moves in random directions for every step.
 */
class RandomerUnit : public MapUnit {
public:
	RandomerUnit(int _x, int _y)
	:MapUnit(_x, _y) { }
	const char *GetName() { return "RandomerUnit"; }
	
	bool MakeMove(MapEnvironment *, OccupancyInterface<xyLoc, tDirection> *, MapSimulationInfo *, tDirection &dir);
};

/**
* A configurable billiard ball unit 
 */
class BilliardBallUnit : public MapUnit {
public:
	BilliardBallUnit(int _x, int _y, int _coolOffPeriod, double _probDirChange)
	:MapUnit(_x, _y) { 
		lastIndex = kStayIndex; 
		collisionStatus = 0;
		coolOffPeriod = _coolOffPeriod;
		probDirChange = _probDirChange;
	}
	virtual const char *GetName() { return "BilliardBallUnit"; }

	bool MakeMove(MapEnvironment *, OccupancyInterface<xyLoc, tDirection> *, MapSimulationInfo *, tDirection &dir);
	void UpdateLocation(MapEnvironment *, xyLoc &, bool, MapSimulationInfo *);
private:
		int lastIndex;
	int collisionStatus;
	double probDirChange;
	int coolOffPeriod;
};

#endif
