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

/**
* A simple map-based unit
 */

class AbsMapUnit : public Unit<xyLoc, tDirection, AbsMapEnvironment> {
public:
	AbsMapUnit(int x, int y)
	:loc(x, y) { r = 1.0; g = 0; b = 0;}
	
	virtual void UpdateLocation(xyLoc l, bool)
{ loc = l; }
	virtual void GetLocation(xyLoc &l)
{ l = loc; }
	virtual void OpenGLDraw(AbsMapEnvironment *);
	void GetLocation(int &x, int &y) { x = loc.x; y = loc.y; }
	void SetColor(GLfloat _r, GLfloat _g, GLfloat _b) { r=_r; g=_g; b=_b; }
	void GetColor(GLfloat& _r, GLfloat& _g, GLfloat& _b) { _r=r; _g=g; _b=b; }
protected:
	GLfloat r, g, b;
	xyLoc loc;
};

#endif
