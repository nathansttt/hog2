/*
 * $Id: patrolUnit.cpp,v 1.9 2006/11/01 23:26:20 nathanst Exp $
 *
 *  patrolUnit.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 3/23/06.
 *  Copyright 2006 Nathan Sturtevant, University of Alberta. All rights reserved.
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

#include "patrolUnit.h"


patrolUnit::patrolUnit(int _x, int _y)
:unit(_x, _y)
{
	setObjectType(kWorldObject);
	currTarget = -1;
	nodesExpanded = nodesTouched = 0;
}

/** Creates a patrol unit and assigns it numPLocations random locations to patrol.
    The last location is the unit's starting position to create a loop */
patrolUnit::patrolUnit(int _x, int _y, int numPLocations, unitSimulation* us)
:unit(_x, _y)
{
	setObjectType(kWorldObject);
	currTarget = -1;
	nodesExpanded = nodesTouched = 0;
	for (int i=1; i<numPLocations; i++)
	{
		int xx, yy;
		us->getRandomLocation(_x,_y,xx,yy);
		addPatrolLocation(new unit(xx, yy));
	}
	addPatrolLocation(new unit(_x, _y));
}

tDirection patrolUnit::makeMove(mapProvider *mp, reservationProvider *, simulationInfo *)
{
	mapAbstraction *aMap = mp->getMapAbstraction();
	if (moves.size() > 0)
	{
		tDirection dir = moves.back();
		moves.pop_back();
		return dir;
	}
	
	if (currTarget != -1)
	{
		goToLoc(aMap, currTarget);
		currTarget = (currTarget+1)%Locs.size();
		if (moves.size() > 0)
		{
			tDirection dir = moves.back();
			moves.pop_back();
			return dir;
		}
	}
	return kStay;
}

double patrolUnit::goToLoc(mapAbstraction *aMap, int which)
{
	double pathCost=-1;
	path *p;
	node *from, *to;
	from = aMap->getNodeFromMap(x, y); // gets my location
	int tox, toy;
	Locs[which]->getLocation(tox, toy);
//	printf("Patrol unit going to %d, %d\n", tox, toy);
	to = aMap->getNodeFromMap(tox, toy);
	p = a.getPath(aMap, from, to);
	nodesExpanded += a.getNodesExpanded();
	nodesTouched += a.getNodesTouched();
	if (p)
	{
		pathCost = aMap->distance(p);
		addPathToCache(p);
	}
	return pathCost;
}

void patrolUnit::openGLDraw(mapProvider *mp, simulationInfo *)
{
	GLdouble xx, yy, zz, rad;
	Map *map = mp->getMap();
	int posx = x, posy = y;
	map->getOpenGLCoord(posx, posy, xx, yy, zz, rad);
	glColor3f(r, g, b);
	glBegin(GL_LINE_STRIP);
//	glVertex3f(xx, yy+rad/2, zz);
	glVertex3f(xx, yy, zz-rad/2);
	for (int t = moves.size()-1; t >= 0; t--)
	{
		posx += ((moves[t]&kE)?1:0) - ((moves[t]&kW)?1:0);
		posy += ((moves[t]&kS)?1:0) - ((moves[t]&kN)?1:0);
		
		map->getOpenGLCoord(posx, posy, xx, yy, zz, rad);
		
//		glVertex3f(xx, yy+rad/2, zz);
		glVertex3f(xx, yy, zz-rad/2);
	}
	glEnd();
	
	map->getOpenGLCoord(x, y, xx, yy, zz, rad);
	glColor3f(r, g, b);
	drawSphere(xx, yy, zz, rad);
}

void patrolUnit::addPatrolLocation(unit *ru)
{
	currTarget = 0;
	Locs.push_back(ru);
}

unit *patrolUnit::getTarget()
{
	if (currTarget == -1)
		return 0;
	return Locs[currTarget];
}

void patrolUnit::addPathToCache(path *p)
{
	// we are at the last move; abort recursion
	if ((p == NULL) || (p->next == NULL))
		return;
	// there is another move; add it first to cache
	if (p->next->next)
		addPathToCache(p->next);
	
	// ----- Ok, we have a path starting at (x,y) [the current location] and
	// having at least one more state ----------------------------------------
	
	// Take the first move off the path and execute it
	int result = kStay;
	
	// Decide on the horizontal move
	switch ((p->n->getLabelL(kFirstData)-p->next->n->getLabelL(kFirstData)))
	{
		case -1: result = kE; break;
		case 0: break;
		case 1: result = kW; break;
		default :
			printf("SU: %s : The (x) nodes in the path are not next to each other!\n",
						 this->getName());
			printf("Distance is %ld\n",
						 p->n->getLabelL(kFirstData)-p->next->n->getLabelL(kFirstData));
			std::cout << *p->n << "\n" << *p->next->n << "\n";
			exit(10); break;
	}
	
	// Tack the vertical move onto it
	// Notice the exploit of particular encoding of kStay, kE, etc. labels
	switch ((p->n->getLabelL(kFirstData+1)-p->next->n->getLabelL(kFirstData+1)))
	{
		case -1: result = result|kS; break;
		case 0: break;
		case 1: result = result|kN; break;
		default :
			printf("SU: %s : The (y) nodes in the path are not next to each other!\n",
						 this->getName());
			printf("Distance is %ld\n",
						 p->n->getLabelL(kFirstData+1)-p->next->n->getLabelL(kFirstData+1));
			std::cout << *p->n << "\n" << *p->next->n << "\n";
			exit(10); break;
	}
	moves.push_back((tDirection)result);
}

void patrolUnit::logStats(statCollection *stats)
{
	if (((nodesExpanded == 0) && (nodesTouched != 0)) ||
			((nodesExpanded != 0) && (nodesTouched == 0)))
	{
		printf("Error; somehow nodes touched/expanded are inconsistent. t:%d e:%d\n",
					 nodesTouched, nodesExpanded);
	}
	// printf("searchUnit::logStats(nodesExpanded=%d, nodesTouched=%d)\n",nodesExpanded,nodesTouched);
	if (nodesExpanded != 0)
		stats->addStat("nodesExpanded", getName(), (long)nodesExpanded);
	if (nodesTouched != 0)
		stats->addStat("nodesTouched", getName(), (long)nodesTouched);
	nodesExpanded = nodesTouched = 0;
}

void patrolUnit::logFinalStats(statCollection *stats)
{
	a.logFinalStats(stats);
}
