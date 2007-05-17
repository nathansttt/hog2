/*
 * $Id: SharedAMapGroup.cpp,v 1.14 2006/10/18 23:52:25 nathanst Exp $
 *
 *  SharedAMapGroup.cpp
 *  HOG
 *
 *  Created by Nathan Sturtevant on 12/16/04.
 *  Copyright 2004 University of Alberta. All rights reserved.
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include "SharedAMapGroup.h"
#include "Unit.h"
#include "UnitGroup.h"
#include "MapCliqueAbstraction.h"
#include "MapFlatAbstraction.h"

SharedAMapGroup::SharedAMapGroup(MapProvider *mp)
:aMap(0), seen(0)
{
	visRadius = 2;
	sawNewLand = true;
	newTileCount = 0;
	newTileCountPerTrial = 0;

	Map *m = mp->GetMap();
	map = new Map(m->getMapWidth(), m->getMapHeight());
	aMap = new MapCliqueAbstraction(map);
	//aMap = new MapFlatAbstraction(map);
	seen = new bitVector(m->getMapWidth() * m->getMapHeight());
	
}

SharedAMapGroup::~SharedAMapGroup()
{
	delete aMap;
	delete seen;
}

//void SharedAMapGroup::updateLocation(BaseAbsMapUnit *u, MapProvider *mp, int _x, int _y, bool success, SimulationInfo *simInfo)
void SharedAMapGroup::UpdateLocation(Unit<xyLoc, tDirection, AbsMapEnvironment> *u, AbsMapEnvironment *mp, xyLoc &loc, bool success, SimulationInfo *si)
{
	Map *worldMap = mp->GetMap();
	int rad = visRadius; // how far we can see -- square
	int x, y, num;
	u->UpdateLocation(mp, loc, success, si);
	xyLoc l;
	u->GetLocation(l);
	x = l.x; y = l.y;
	bool changed = false;
	for (int x1 = x-rad; x1 <= x+rad; x1++)
	{
		if ((x1 < 0) || (x1 >= map->getMapWidth()))
			continue;
		for (int y1 = y-rad; y1 <= y+rad; y1++)
		{
			if ((y1 < 0) || (y1 >= map->getMapHeight()))
				continue;
			
			// if we haven't observed a particular tile, we need to look at it
			if (!seen->get(y1*map->getMapWidth()+x1))
			{
				// count a new tile!
				newTileCount++;
				// if it's different that we think, we just remove it.
				// in the future we'll need to handle water & splits here
				num = map->getNodeNum(x1, y1);
				graph *g = aMap->GetAbstractGraph(0);
				if (!worldMap->adjacentEdges(x1, y1, kLeftEdge))
				{ edge *e = g->findEdge(num, map->getNodeNum(x1-1, y1)); aMap->RemoveEdge(e, 0); }
				if (!worldMap->adjacentEdges(x1, y1, kRightEdge))
				{ edge *e = g->findEdge(num, map->getNodeNum(x1+1, y1)); aMap->RemoveEdge(e, 0); }
				if (!worldMap->adjacentEdges(x1, y1, kTopEdge))
				{ edge *e = g->findEdge(num, map->getNodeNum(x1, y1-1)); aMap->RemoveEdge(e, 0); }
				if (!worldMap->adjacentEdges(x1, y1, kBottomEdge))
				{ edge *e = g->findEdge(num, map->getNodeNum(x1, y1+1)); aMap->RemoveEdge(e, 0); }

				if (!worldMap->adjacentCorners(x1, y1, kTopLeft))
				{ edge *e = g->findEdge(num, map->getNodeNum(x1-1, y1-1)); aMap->RemoveEdge(e, 0); }
				if (!worldMap->adjacentCorners(x1, y1, kTopRight))
				{ edge *e = g->findEdge(num, map->getNodeNum(x1+1, y1-1)); aMap->RemoveEdge(e, 0); }
				if (!worldMap->adjacentCorners(x1, y1, kBottomLeft))
				{ edge *e = g->findEdge(num, map->getNodeNum(x1-1, y1+1)); aMap->RemoveEdge(e, 0); }
				if (!worldMap->adjacentCorners(x1, y1, kBottomRight))
				{ edge *e = g->findEdge(num, map->getNodeNum(x1+1, y1+1)); aMap->RemoveEdge(e, 0); }
				
				//aMap->RemoveNode(me);
				map->setTerrainType(x1, y1, (tTerrain)worldMap->getTerrainType(x1, y1));
				changed = true;
				
				sawNewLand = true;
				seen->set(y1*map->getMapWidth()+x1, true);
			}
		}
	}
	if (changed)
		aMap->RepairAbstraction();
}

void SharedAMapGroup::OpenGLDraw(int , AbsMapEnvironment *, SimulationInfo *)
{
	glBegin(GL_QUADS);
	glColor3f(.25, .25, .25); // kOutOfBounds
	glNormal3f(0, 0, 1);
	GLdouble coverage = 0.9;
	for (int x = 0; x < map->getMapWidth(); x++)
	{
		for (int y = 0; y < map->getMapHeight(); y++)
		{
			GLdouble a, b, c, radius;
			if (!seen->get(y*map->getMapWidth()+x))
			{
				map->getOpenGLCoord(x, y, a, b, c, radius);
				glVertex3f(a+coverage*radius, b+coverage*radius, c-4*radius);
				glVertex3f(a+coverage*radius, b-coverage*radius, c-4*radius);
				glVertex3f(a-coverage*radius, b-coverage*radius, c-4*radius);
				glVertex3f(a-coverage*radius, b+coverage*radius, c-4*radius);
			}
		}
	}
	glEnd();
}

MapAbstraction *SharedAMapGroup::GetMapAbstraction()
{
	return aMap;
}

Map *SharedAMapGroup::GetMap()
{
	return map;
}

/** Is the group done with their exploration? */
bool SharedAMapGroup::done()
{
	return (!sawNewLand);
}

void SharedAMapGroup::logStats(StatCollection *stats)
//void SharedAMapGroup::printRoundStats(unit *, FILE *f)
{
	if (newTileCount != 0)
	{
		stats->SumStat("newTilesExplored", GetName(), (long)newTileCount);
		newTileCountPerTrial += newTileCount;
		newTileCount = 0;
	}
	//fprintf(f,"%8.2f", (float)newTileCount);
}

/** Lets the unit group do what it needs to reset a trial */
void SharedAMapGroup::startNewTrial(StatCollection *stats)
{
	sawNewLand = false;
	newTileCount = 0;
	newTileCountPerTrial = 0;
	stats->AddStat("newTilesExplored", GetName(), (long)newTileCount);
}

void SharedAMapGroup::setVisibilityRadius(int _visibility)
{
	visRadius = _visibility;
}

int SharedAMapGroup::getVisibilityRadius()
{
	return visRadius;
}

bool SharedAMapGroup::explored(int x, int y)
{
	return seen->get(y*map->getMapWidth()+x);
}

bool SharedAMapGroup::explored(unsigned int _node)
{
	node *loc = aMap->GetAbstractGraph(0)->getNode(_node);
	return explored((unsigned int)loc->getLabelL(kFirstData), 
									(unsigned int)loc->getLabelL(kFirstData+1));
}
