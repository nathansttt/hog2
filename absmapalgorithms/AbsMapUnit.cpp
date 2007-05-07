/*
 *  AbsMapUnit.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/27/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "AbsMapUnit.h"

void AbsMapUnit::OpenGLDraw(int window, AbsMapEnvironment *me, SimulationInfo *)
{
	Map *map = me->GetMap();
	GLdouble xx, yy, zz, rad;
	if ((loc.x < 0) || (loc.x >= map->getMapWidth()) || (loc.y < 0) || (loc.y >= map->getMapHeight()))
		return;
	map->getOpenGLCoord(loc.x, loc.y, xx, yy, zz, rad);
	glColor3f(r, g, b);
	//	if (getObjectType() == kDisplayOnly)
	//		drawTriangle(xx, yy, zz, rad);
	//	else
	DrawSphere(xx, yy, zz, rad);
}
