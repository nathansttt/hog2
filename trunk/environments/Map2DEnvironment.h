/*
 *  Map2DEnvironment.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/20/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef MAP2DENVIRONMENT_H
#define MAP2DENVIRONMENT_H

#include "Constants.h"
#include <stdint.h>
#include "Map.h"
#include "MapAbstraction.h"
#include "UnitSimulation.h"

struct xyLoc {
public:
	xyLoc() {}
	xyLoc(uint16_t _x, uint16_t _y) :x(_x), y(_y) {}
	uint16_t x;
	uint16_t y;
};

class MapEnvironment : public SearchEnvironment<xyLoc, tDirection>
{
public:
	MapEnvironment(Map *m);
	virtual ~MapEnvironment();
	void GetSuccessors(xyLoc nodeID, std::vector<xyLoc> &neighbors);
	void GetActions(xyLoc nodeID, std::vector<tDirection> &actions);
	tDirection GetAction(xyLoc s1, xyLoc s2);
	double HCost(xyLoc node1, xyLoc node2);
	double GCost(xyLoc node1, xyLoc node2);
	bool GoalTest(xyLoc node, xyLoc goal);
	uint32_t GetStateHash(xyLoc node);
	uint32_t GetActionHash(tDirection act);
protected:
	Map *map;
};

class MapAbstractionEnvironment : public MapEnvironment
{
public:
	MapAbstractionEnvironment(mapAbstraction *ma);
	virtual ~MapAbstractionEnvironment();
	mapAbstraction *GetMapAbstraction() { return ma; }
private:
	mapAbstraction *ma;
};

#endif
