/*
 * $Id: rewardUnit.cpp,v 1.6 2006/11/01 23:27:16 nathanst Exp $
 *
 *  rewardUnit.cpp
 *  HOG
 *
 *  Created by Nathan Sturtevant on 2/16/05.
 *  Copyright 2005 Nathan Sturtevant, University of Alberta. All rights reserved.
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

#include "rewardUnit.h"

rewardUnit::rewardUnit(int _x, int _y)
:unit(_x, _y)
{
	setObjectType(kIncidentalUnit);
}

void rewardUnit::openGLDraw(mapProvider *mp, simulationInfo *)
{
	Map *map = mp->getMap();
	GLdouble xx, yy, zz, rad;
	map->getOpenGLCoord(x, y, xx, yy, zz, rad);
	glColor3f(r, g, b);
	drawTriangle(xx, yy, zz, rad);
}



rewardSeekingUnit::rewardSeekingUnit(int _x, int _y)
:rewardUnit(_x, _y)
{
	setObjectType(kWorldObject);
}

void rewardSeekingUnit::receiveReward(double amount)
{
	printf("%s got reward %3.2f!!\n", getName(), amount);
}

tDirection rewardSeekingUnit::makeMove(mapProvider *mp, reservationProvider *, simulationInfo *)
{
	mapAbstraction *aMap = mp->getMapAbstraction();
	if (moves.size() > 0)
	{
		tDirection dir = moves.back();
		moves.pop_back();
		//		if (verbose)
		//			printf("SU %p: returning cached move 0x%X\n", this, (int)dir);
		return dir;
	}

	// add your logic code here...
	static int nextTarget = 0;
	goToRewardLoc(aMap, nextTarget);
	nextTarget = (nextTarget+1)%rewardLocs.size();
	return kStay;
}

double rewardSeekingUnit::goToRewardLoc(mapAbstraction *aMap, int which)
{
	double pathCost=-1;
	path *p;
	node *from, *to;
	from = aMap->getNodeFromMap(x, y); // gets my location
	int tox, toy;
	rewardLocs[which]->getLocation(tox, toy);
	to = aMap->getNodeFromMap(tox, toy);
	p = a.getPath(aMap, from, to);
	if (p)
	{
		pathCost = aMap->distance(p);
		addPathToCache(p);
	}
	return pathCost;
}

void rewardSeekingUnit::openGLDraw(mapProvider *mp, simulationInfo *)
{
	GLdouble xx, yy, zz, rad;
	Map *map = mp->getMap();
	int posx = x, posy = y;
	map->getOpenGLCoord(posx, posy, xx, yy, zz, rad);
	glColor3f(r, g, b);
	glBegin(GL_LINE_STRIP);
	glVertex3f(xx, yy+rad/2, zz);
	for (int t = moves.size()-1; t >= 0; t--)
	{
		posx += ((moves[t]&kE)?1:0) - ((moves[t]&kW)?1:0);
		posy += ((moves[t]&kS)?1:0) - ((moves[t]&kN)?1:0);
		
		map->getOpenGLCoord(posx, posy, xx, yy, zz, rad);
		
		glVertex3f(xx, yy+rad/2, zz);
	}
	glEnd();
	
	map->getOpenGLCoord(x, y, xx, yy, zz, rad);
	glColor3f(r, g, b);
	drawSphere(xx, yy, zz, rad);
}

void rewardSeekingUnit::addRewardLocation(rewardUnit *ru)
{
	rewardLocs.push_back(ru);
}

void rewardSeekingUnit::addPathToCache(path *p)
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
