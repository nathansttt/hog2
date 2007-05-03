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

enum tDirection {
	kN=0x8, kS=0x4, kE=0x2, kW=0x1, kNW=kN|kW, kNE=kN|kE, 
	kSE=kS|kE, kSW=kS|kW, kStay=0, kTeleport=kSW|kNE
};

const int numPrimitiveActions = 8;
const int numActions = 10;
const tDirection possibleDir[numActions] = { kN, kNE, kE, kSE, kS, kSW, kW, kNW, kStay, kTeleport };
const int kStayIndex = 8; // index of kStay

typedef OccupancyInterface<xyLoc, tDirection> BaseMapOccupancyInterface;

class MapEnvironment : public SearchEnvironment<xyLoc, tDirection>
{
public:
	MapEnvironment(Map *m);
	virtual ~MapEnvironment();
	void GetSuccessors(xyLoc nodeID, std::vector<xyLoc> &neighbors);
	void GetActions(xyLoc nodeID, std::vector<tDirection> &actions);
	tDirection GetAction(xyLoc s1, xyLoc s2);
	xyLoc ApplyAction(xyLoc s, tDirection dir);
	virtual OccupancyInterface<xyLoc, tDirection> *GetOccupancyInfo() { return 0; }

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
	AbsMapEnvironment(MapAbstraction *ma);
	virtual ~AbsMapEnvironment();
	MapAbstraction *GetMapAbstraction() { return ma; }
	void OpenGLDraw() { map->OpenGLDraw(); ma->OpenGLDraw(); }
private:
	MapAbstraction *ma;
};

typedef UnitSimulation<xyLoc, tDirection, MapEnvironment> UnitMapSimulation;
typedef UnitSimulation<xyLoc, tDirection, AbsMapEnvironment> UnitAbsMapSimulation;

template<>
void UnitSimulation<xyLoc, tDirection, MapEnvironment>::OpenGLDraw()
{
	env->OpenGLDraw();
	for (unsigned int x = 0; x < units.size(); x++)
	{
		units[x]->agent->OpenGLDraw(env);
	}
}

template<>
void UnitSimulation<xyLoc, tDirection, AbsMapEnvironment>::OpenGLDraw()
{
	env->OpenGLDraw();
	for (unsigned int x = 0; x < units.size(); x++)
	{
		units[x]->agent->OpenGLDraw(env);
	}
}

#endif
