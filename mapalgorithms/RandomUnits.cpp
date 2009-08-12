/*
 *  RandomUnits.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/23/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "RandomUnits.h"
#include "FPUtil.h"

/**
* Make a random move.
 *
 * the random unit follows a certain direction for a random amount of time,
 * and then picks a new direction. makeMove just returns the current direction.
 */
bool RandomUnit::MakeMove(MapEnvironment *, OccupancyInterface<xyLoc, tDirection> *, MapSimulationInfo *, tDirection &dir)
{
	dir = possibleDir[lastIndex];
	return true;
}

/**
* Set location after last move.
 *
 * After moving, the unit picks a new random direction if the move wasn't a success.
 * If it was, it has a 5% chance of changing direction.
 */
void RandomUnit::UpdateLocation(MapEnvironment *me, xyLoc &l, bool success, MapSimulationInfo *si)
{
	MapUnit::UpdateLocation(me, l, success, si);
	if (success)
	{ // I moved successfully
		if (random()%20 == 7)
			lastIndex = random()%9;
	}
	else {
		lastIndex = random()%9;
	}
}

/**
* Make a random move.
 *
 * the random unit follows a certain direction for a random amount of time,
 * and then picks a new direction. makeMove just returns the current direction.
 */
bool RandomerUnit::MakeMove(MapEnvironment *, OccupancyInterface<xyLoc, tDirection> *, MapSimulationInfo *, tDirection &dir)
{
	dir = possibleDir[random()%9];
	return true;
}

/**
* Make a move.
 *
 * the billiard ball unit keeps following the same direction until either a collision
 * or instability
 */
bool BilliardBallUnit::MakeMove(MapEnvironment *, OccupancyInterface<xyLoc, tDirection> *, MapSimulationInfo *, tDirection &dir)
{
	dir = possibleDir[lastIndex];
	return true;
}

/**
* Set location after last move.
 *
 * if the billiard ball unit collides, it will cool off for a period of time
 * otherwise, it may changes its mind if it unstable
 */
void BilliardBallUnit::UpdateLocation(MapEnvironment *me, xyLoc &l, bool success, MapSimulationInfo *si)
{
	MapUnit::UpdateLocation(me, l, success, si);
	if (success)
	{ 
		// I moved successfully
		// If I was staying put then I should start moving after the coolOffPeriod expires
		if (lastIndex == kStayIndex && --collisionStatus <= 0) 
			lastIndex = random()%9;
		// If I was moving then I may change my mind with probability probDirChange
		if (lastIndex != kStayIndex && fless(random()%10000,probDirChange*10000.0))
			lastIndex = random()%9;
	}
	else {
		// I was not successful in my move (i.e., I collided)
		// start resting
		collisionStatus = coolOffPeriod;
		lastIndex = coolOffPeriod > 0 ? kStayIndex : random()%9;		
	}
}

/**
* make teleport move.
 *
 * the teleport unit stays put for some # moves, and then teleports to a new random
 * location.
 */
bool TeleportUnit::MakeMove(MapEnvironment *me, OccupancyInterface<xyLoc, tDirection> *, MapSimulationInfo *, tDirection &dir)
{
	if (timer == 0)
	{
		timer = stayTime;
		loc.x = random()%me->GetMap()->GetMapWidth();
		loc.y = random()%me->GetMap()->GetMapHeight();
		dir = kTeleport;
		return true;
	}
	timer--;
	dir = kStay;
	return true;
}

