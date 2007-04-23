/*
 *  MapUnit.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/23/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef MAPUNIT_H
#define MAPUNIT_H

#include "Unit.h"
#include "Map2DEnvironment.h"

/**
* A simple map-based unit
 */

class MapUnit : public Unit<xyLoc, tDirection, MapEnvironment> {
public:
	MapUnit(int x, int y)
	:loc(x, y) { r = 1.0; g = 0; b = 0;}
	virtual tDirection MakeMove(MapEnvironment *) = 0;
	virtual void UpdateLocation(xyLoc l, bool)
	{ loc = l; }
	virtual void GetLocation(xyLoc &l)
	{ l = loc; }
	virtual void OpenGLDraw(MapEnvironment *);
protected:
	GLfloat r, g, b;
	xyLoc loc;
};

#endif
