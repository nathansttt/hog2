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
#include <stdlib.h>
#include <iostream>
#include "Map.h"
#include "MapAbstraction.h"
#include "SearchEnvironment.h"
#include "UnitSimulation.h"
#include "ReservationProvider.h"
#include "BitVector.h"

#include <cassert>

//#include "BaseMapOccupancyInterface.h"

struct xyLoc {
public:
	xyLoc() {}
	xyLoc(uint16_t _x, uint16_t _y) :x(_x), y(_y) {}
	uint16_t x;
	uint16_t y;
};

static std::ostream& operator <<(std::ostream & out, const xyLoc &loc)
{
	out << "(" << loc.x << ", " << loc.y << ")";
	return out;
}

static bool operator==(const xyLoc &l1, const xyLoc &l2) {
	return (l1.x == l2.x) && (l1.y == l2.y);
}



enum tDirection {
	kN=0x8, kS=0x4, kE=0x2, kW=0x1, kNW=kN|kW, kNE=kN|kE,
	kSE=kS|kE, kSW=kS|kW, kStay=0, kTeleport=kSW|kNE
};

class BaseMapOccupancyInterface : public OccupancyInterface<xyLoc,tDirection>
{
public:
	BaseMapOccupancyInterface(Map* m);
	virtual ~BaseMapOccupancyInterface();
	virtual void SetStateOccupied(xyLoc&, bool);
	virtual bool GetStateOccupied(xyLoc&);
	virtual bool CanMove(xyLoc&, xyLoc&);
	virtual void MoveUnitOccupancy(xyLoc &, xyLoc&);

private:
	bitVector *bitvec; /// For each map position, set if occupied
	long mapWidth; /// Used to compute index into bitvector
	long mapHeight; /// used to compute index into bitvector

	long CalculateIndex(uint16_t x, uint16_t y);
};


const int numPrimitiveActions = 8;
const int numActions = 10;
const tDirection possibleDir[numActions] = { kN, kNE, kE, kSE, kS, kSW, kW, kNW, kStay, kTeleport };
const int kStayIndex = 8; // index of kStay




//typedef OccupancyInterface<xyLoc, tDirection> BaseMapOccupancyInterface;


class MapEnvironment : public SearchEnvironment<xyLoc, tDirection>
{
public:
	MapEnvironment(Map *m);
	virtual ~MapEnvironment();
	void GetSuccessors(xyLoc &nodeID, std::vector<xyLoc> &neighbors) const;
	void GetActions(xyLoc &nodeID, std::vector<tDirection> &actions) const;
	tDirection GetAction(xyLoc &s1, xyLoc &s2) const;
	virtual void ApplyAction(xyLoc &s, tDirection dir) const;
	virtual BaseMapOccupancyInterface *GetOccupancyInfo() { return oi; }

	virtual bool InvertAction(tDirection &a) const;

	virtual double HCost(xyLoc &node1) {
		fprintf(stderr, "ERROR: Single State HCost not implemented for MapEnvironment\n");
		exit(1); return -1.0;}
	virtual double HCost(xyLoc &node1, xyLoc &node2);
	virtual double GCost(xyLoc &node1, xyLoc &node2);
	virtual double GCost(xyLoc &node1, tDirection &act);
	bool GoalTest(xyLoc &node, xyLoc &goal);

	bool GoalTest(xyLoc &s){
		fprintf(stderr, "ERROR: Single State Goal Test not implemented for MapEnvironment\n");
		exit(1); return false;}

	uint64_t GetStateHash(xyLoc &node) const;
	uint64_t GetActionHash(tDirection act) const;
	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const xyLoc &l) const;
	virtual void OpenGLDraw(const xyLoc &l1, const xyLoc &l2, double v) const;
	virtual void OpenGLDraw(const xyLoc &, const tDirection &) const;
	//virtual void OpenGLDraw(const xyLoc &, const tDirection &, GLfloat r, GLfloat g, GLfloat b) const;
	//virtual void OpenGLDraw(const xyLoc &l, GLfloat r, GLfloat g, GLfloat b) const;
	Map* GetMap() const { return map; }

	virtual void GetNextState(xyLoc &currents, tDirection dir, xyLoc &news) const;

	void StoreGoal(xyLoc &g) {} // stores the locations for the given goal state
	void ClearGoal() {}
	bool IsGoalStored() {return false;}

	//virtual BaseMapOccupancyInterface* GetOccupancyInterface(){std::cout<<"Mapenv\n";return oi;}
	//virtual xyLoc GetNextState(xyLoc &s, tDirection dir);
protected:
	Map *map;
	BaseMapOccupancyInterface *oi;
};

class AbsMapEnvironment : public MapEnvironment
{
public:
	AbsMapEnvironment(MapAbstraction *ma);
	virtual ~AbsMapEnvironment();
	MapAbstraction *GetMapAbstraction() { return ma; }
	void OpenGLDraw() const { map->OpenGLDraw(); ma->OpenGLDraw(); }
	void OpenGLDraw(const xyLoc &l) const { MapEnvironment::OpenGLDraw(l); }
	void OpenGLDraw(const xyLoc &l1, const xyLoc &l2, double v) const { MapEnvironment::OpenGLDraw(l1, l2, v); }
	//void OpenGLDraw(const xyLoc &l, GLfloat r, GLfloat g, GLfloat b) const {MapEnvironment::OpenGLDraw(l,r,g,b);}
	void OpenGLDraw(const xyLoc& s, const tDirection &dir) const {MapEnvironment::OpenGLDraw(s,dir);}
	//void OpenGLDraw(const xyLoc &s, const tDirection &dir, GLfloat r, GLfloat g, GLfloat b) const {MapEnvironment::OpenGLDraw(s,dir,r,g,b);}

	//virtual BaseMapOccupancyInterface* GetOccupancyInterface(){std::cout<<"AbsMap\n";return oi;}
protected:
	MapAbstraction *ma;
};

typedef UnitSimulation<xyLoc, tDirection, MapEnvironment> UnitMapSimulation;
typedef UnitSimulation<xyLoc, tDirection, AbsMapEnvironment> UnitAbsMapSimulation;


//template<>
//void UnitSimulation<xyLoc, tDirection, MapEnvironment>::OpenGLDraw()
//{
//	env->OpenGLDraw();
//	for (unsigned int x = 0; x < units.size(); x++)
//	{
//		units[x]->agent->OpenGLDraw(env);
//	}
//}
//
//template<>
//void UnitSimulation<xyLoc, tDirection, AbsMapEnvironment>::OpenGLDraw()
//{
//	env->OpenGLDraw();
//	for (unsigned int x = 0; x < units.size(); x++)
//	{
//		units[x]->agent->OpenGLDraw(env);
//	}
//}

#endif
