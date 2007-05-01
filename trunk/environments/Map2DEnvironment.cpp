/*
 *  Map2DEnvironment.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/20/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "Map2DEnvironment.h"
#include "FPUtil.h"

MapEnvironment::MapEnvironment(Map *_m)
{
	map = _m;
}

MapEnvironment::~MapEnvironment()
{
	delete map;
}

void MapEnvironment::GetSuccessors(xyLoc loc, std::vector<xyLoc> &neighbors)
{
	bool up=false, down=false;
	if ((map->getTerrainType(loc.x, loc.y+1) == kGround))
	{
		down = true;
		neighbors.push_back(xyLoc(loc.x, loc.y+1));
	}
	if ((map->getTerrainType(loc.x, loc.y-1) == kGround))
	{
		up = true;
		neighbors.push_back(xyLoc(loc.x, loc.y-1));
	}
	if ((map->getTerrainType(loc.x-1, loc.y) == kGround))
	{
		if ((up && (map->getTerrainType(loc.x-1, loc.y-1) == kGround)))
			neighbors.push_back(xyLoc(loc.x-1, loc.y-1));
		if ((down && (map->getTerrainType(loc.x-1, loc.y+1) == kGround)))
			neighbors.push_back(xyLoc(loc.x-1, loc.y+1));
		neighbors.push_back(xyLoc(loc.x-1, loc.y));
	}
	if ((map->getTerrainType(loc.x+1, loc.y) == kGround))
	{
		if ((up && (map->getTerrainType(loc.x+1, loc.y-1) == kGround)))
			neighbors.push_back(xyLoc(loc.x+1, loc.y-1));
		if ((down && (map->getTerrainType(loc.x+1, loc.y+1) == kGround)))
			neighbors.push_back(xyLoc(loc.x+1, loc.y+1));
		neighbors.push_back(xyLoc(loc.x+1, loc.y));
	}
}

void MapEnvironment::GetActions(xyLoc loc, std::vector<tDirection> &actions)
{
	bool up=false, down=false;
	if ((map->getTerrainType(loc.x, loc.y+1) == kGround))
	{
		down = true;
		actions.push_back(kS);
	}
	if ((map->getTerrainType(loc.x, loc.y-1) == kGround))
	{
		up = true;
		actions.push_back(kN);
	}
	if ((map->getTerrainType(loc.x-1, loc.y) == kGround))
	{
		if ((up && (map->getTerrainType(loc.x-1, loc.y-1) == kGround)))
			actions.push_back(kNW);
		if ((down && (map->getTerrainType(loc.x-1, loc.y+1) == kGround)))
			actions.push_back(kSW);
		actions.push_back(kW);
	}
	if ((map->getTerrainType(loc.x+1, loc.y) == kGround))
	{
		if ((up && (map->getTerrainType(loc.x+1, loc.y-1) == kGround)))
			actions.push_back(kNE);
		if ((down && (map->getTerrainType(loc.x+1, loc.y+1) == kGround)))
			actions.push_back(kSE);
		actions.push_back(kE);
	}
}

tDirection MapEnvironment::GetAction(xyLoc s1, xyLoc s2)
{
	int result = kStay;
	switch (s1.x-s2.x)
	{
		case -1: result = kE; break;
		case 0: break;
		case 1: result = kW; break;
		default: return kTeleport;
	}
	
	// Tack the vertical move onto it
	// Notice the exploit of particular encoding of kStay, kE, etc. labels
	switch (s1.y-s2.y)
	{
		case -1: result = result|kS; break;
		case 0: break;
		case 1: result = result|kN; break;
		default: return kTeleport;
	}
	return (tDirection)result;
}

xyLoc MapEnvironment::ApplyAction(xyLoc s, tDirection dir)
{
	xyLoc old = s;
	switch (dir)
	{
		case kN: s.y-=1; break;
		case kS: s.y+=1; break;
		case kE: s.x+=1; break;
		case kW: s.x-=1; break;
		case kNW: s.y-=1; s.x-=1; break;
		case kSW: s.y+=1; s.x-=1; break;
		case kNE: s.y-=1; s.x+=1; break;
		case kSE: s.y+=1; s.x+=1; break;
		default: break;
	}
	if (map->canStep(s.x, s.y, old.x, old.y))
		return s;
	return old;
}

double MapEnvironment::HCost(xyLoc l1, xyLoc l2)
{
	double a = ((l1.x>l2.x)?(l1.x-l2.x):(l2.x-l1.x));
	double b = ((l1.y>l2.y)?(l1.y-l2.y):(l2.y-l1.y));
	return (a>b)?(b*ROOT_TWO+a-b):(a*ROOT_TWO+b-a);
}

double MapEnvironment::GCost(xyLoc l1, xyLoc l2)
{
	double h = HCost(l1, l2);
	if (fgreater(h, ROOT_TWO))
		return DBL_MAX;
	return h;
}

bool MapEnvironment::GoalTest(xyLoc node, xyLoc goal)
{
	return ((node.x == goal.x) && (node.y == goal.y));
}

uint32_t MapEnvironment::GetStateHash(xyLoc node)
{
	return (node.x<<16)|node.y;
}

uint32_t MapEnvironment::GetActionHash(tDirection act)
{
	return (uint32_t) act;
}


/************************************************************/

AbsMapEnvironment::AbsMapEnvironment(mapAbstraction *_ma)
:MapEnvironment(_ma->getMap())
{
	ma = _ma;
}

AbsMapEnvironment::~AbsMapEnvironment()
{
	map = 0;
	delete ma;
}
