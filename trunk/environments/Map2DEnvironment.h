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
	xyLoc ApplyAction(xyLoc s, tDirection dir);

	double HCost(xyLoc node1, xyLoc node2);
	double GCost(xyLoc node1, xyLoc node2);
	bool GoalTest(xyLoc node, xyLoc goal);
	uint32_t GetStateHash(xyLoc node);
	uint32_t GetActionHash(tDirection act);
	void OpenGLDraw() { map->OpenGLDraw(); }
	Map* GetMap() { return map; }
protected:
	Map *map;
};

class AbsMapEnvironment : public MapEnvironment
{
public:
	AbsMapEnvironment(mapAbstraction *ma);
	virtual ~AbsMapEnvironment();
	mapAbstraction *GetMapAbstraction() { return ma; }
	void OpenGLDraw() { map->OpenGLDraw(); ma->OpenGLDraw(); }
private:
	mapAbstraction *ma;
};

typedef UnitSimulation<xyLoc, tDirection, MapEnvironment> UnitMapSimulation;
typedef UnitSimulation<xyLoc, tDirection, AbsMapEnvironment> UnitAbsMapSimulation;

#endif
