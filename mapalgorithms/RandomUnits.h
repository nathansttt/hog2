/*
 *  RandomUnits.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/23/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef RANDOMUNITS_H
#define RANDOMUNITS_H

#include "Unit.h"
#include "MapUnit.h"

/**
* A unit that moves in random directions changing direction randomly.
 */
class RandomUnit : public MapUnit {
public:
	RandomUnit(int _x, int _y)
	:MapUnit(_x, _y) { lastIndex = 0; }
	virtual const char *GetName() { return "randomUnit"; }

	tDirection MakeMove(MapEnvironment *, BaseMapOccupancyInterface *, SimulationInfo *);
	void UpdateLocation(MapEnvironment *, BaseMapOccupancyInterface *, xyLoc, bool, SimulationInfo *);
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

	tDirection MakeMove(MapEnvironment *, BaseMapOccupancyInterface *, SimulationInfo *);
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
	
	tDirection MakeMove(MapEnvironment *, BaseMapOccupancyInterface *, SimulationInfo *);
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

	tDirection MakeMove(MapEnvironment *, BaseMapOccupancyInterface *, SimulationInfo *);
	void UpdateLocation(MapEnvironment *, BaseMapOccupancyInterface *, xyLoc, bool, SimulationInfo *);
private:
		int lastIndex;
	int collisionStatus;
	double probDirChange;
	int coolOffPeriod;
};

#endif
