/*
 *  AbsMapUnit.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/27/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef ABSMAPUNIT_H
#define ABSMAPUNIT_H

#include "Unit.h"
#include "Map2DEnvironment.h"

typedef Unit<xyLoc, tDirection, AbsMapEnvironment> BaseAbsMapUnit;

typedef SimulationInfo<xyLoc, tDirection, AbsMapEnvironment> AbsMapSimulationInfo;

/**
* A simple map-based unit
 */

class AbsMapUnit : public BaseAbsMapUnit {
public:
	AbsMapUnit(int x, int y)
	:loc(x, y) { r = 1.0; g = 0; b = 0;}
	virtual ~AbsMapUnit() {}
	
	virtual const char *GetName() = 0;
	virtual bool MakeMove(AbsMapEnvironment *, OccupancyInterface<xyLoc,tDirection> *, AbsMapSimulationInfo *, tDirection &) = 0;
	virtual void UpdateLocation(AbsMapEnvironment *, xyLoc &l, bool, AbsMapSimulationInfo *)
	{ loc = l; }
	virtual void GetLocation(xyLoc &l)
	{ l = loc; }
	virtual void GetGoal(xyLoc &s) = 0;
	virtual void OpenGLDraw(const AbsMapEnvironment *, const AbsMapSimulationInfo *) const;
	//void GetLocation(int &x, int &y) { x = loc.x; y = loc.y; }
//	void SetColor(GLfloat _r, GLfloat _g, GLfloat _b) { r=_r; g=_g; b=_b; }
//	void GetColor(GLfloat& _r, GLfloat& _g, GLfloat& _b) { _r=r; _g=g; _b=b; }
protected:
	GLfloat r, g, b;
	xyLoc loc;
};

#endif
