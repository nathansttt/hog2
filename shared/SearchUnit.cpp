/*
 * $Id: searchUnit.cpp,v 1.22 2006/10/24 18:18:45 nathanst Exp $
 *
 *  Hierarchical Open Graph File
 *
 *  Created by Nathan Sturtevant on 10/4/04.
 *  Copyright 2004 Nathan Sturtevant. All rights reserved.
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

#include <iostream>
#include "searchUnit.h"

using namespace std;

static const bool verbose = false;

searchUnit::searchUnit(int _x, int _y, unit *_target, searchAlgorithm *alg)
:unit(_x, _y, _target)
{
	unitType = kWorldObject;
	algorithm = alg;
	s_algorithm = 0;
	spread_cache = 0;
	onTarget = false;
	nodesExpanded = 0;
	nodesTouched = 0;
}

searchUnit::searchUnit(int _x, int _y, unit *_target, spreadExecSearchAlgorithm *alg)
:unit(_x, _y, _target)
{
	unitType = kWorldObject;
	algorithm = alg;
	s_algorithm = alg;
	spread_cache = 0;
	onTarget = false;
	nodesExpanded = 0;
	nodesTouched = 0;
}

searchUnit::searchUnit(int _x, int _y, int _r, int _g, int _b, unit *_target, searchAlgorithm *alg)
:unit(_x, _y, _r, _g, _b, _target)
{
	unitType = kWorldObject;
	algorithm = alg;
	s_algorithm = 0;
	spread_cache = 0;
	onTarget = false;
	nodesExpanded = 0;
	nodesTouched = 0;
}

searchUnit::searchUnit(int _x, int _y, float _r, float _g, float _b, unit *_target, searchAlgorithm *alg)
:unit(_x, _y, _r, _g, _b, _target)
{
	unitType = kWorldObject;
	algorithm = alg;
	s_algorithm = 0;
	spread_cache = 0;
	onTarget = false;
	nodesExpanded = 0;
	nodesTouched = 0;
}

//searchUnit::searchUnit(int _x, int _y, int _r, int _g, int _b, unit *_target, searchAlgorithm *alg)
//:unit(_x, _y, _r, _g, _b, _target)
//{
//	unitType = kWorldObject;
//	algorithm = alg;
//	onTarget = false;
//	nodesExpanded = 0;
//	nodesTouched = 0;
//}

searchUnit::~searchUnit()
{
	if (algorithm)
		delete algorithm;
	algorithm = 0;
	if (spread_cache)
		delete spread_cache;
	spread_cache = 0;
}

bool searchUnit::getCachedMove(tDirection &dir)
{
	if (moves.size() > 0)
	{
		if (s_algorithm && (spread_cache == 0))
		{
			spread_cache = s_algorithm->think();
			nodesExpanded+=algorithm->getNodesExpanded();
			nodesTouched+=algorithm->getNodesTouched();
		}
		dir = moves.back();
		moves.pop_back();
		//		if (verbose)
		//			printf("SU %p: returning cached move 0x%X\n", this, (int)dir);
		return true;
	}
	return false;
}

tDirection searchUnit::makeMove(mapProvider *mp, reservationProvider *rp, simulationInfo *simInfo)
{
	tDirection res;
	if (getCachedMove(res))
		return res;

	Map *map = mp->getMap();
	mapAbstraction *aMap = mp->getMapAbstraction();

	// if we have a cache to be used, use it!
	if (spread_cache)
	{
		addPathToCache(spread_cache);
		node *next_start = spread_cache->tail()->n;
		int targx, targy;
		target->getLocation(targx, targy);
		
		// Get a path by path-planning
		target->getLocation(targx, targy);
		node *to = aMap->getAbstractGraph(0)->getNode(map->getNodeNum(targx, targy));
		
		s_algorithm->setTargets(mp->getMapAbstraction(), next_start, to, rp);
		delete spread_cache;
		spread_cache = 0;
		tDirection dir = moves.back();
		moves.pop_back();
		return dir;
	}
	
	// Check if we have a target defined
	if (!target)
	{
		if (verbose)
			printf("SU %s: No target, doing nothing\n", this->getName());
		return unit::makeMove(mp, rp, simInfo);
	}
	
	// Get the position of the target
	int targx, targy;
	target->getLocation(targx, targy);
	
	// Get a path by path-planning
	graph *g0 = aMap->getAbstractGraph(0);
	// Get the start and goal nodes
	node *from = g0->getNode(map->getNodeNum(x, y));
	target->getLocation(targx, targy);
	node *to = g0->getNode(map->getNodeNum(targx, targy));
	
	if (from == to)
	{
		if (!onTarget)
		{
			if (verbose)
			{
				printf("STAY ON TARGET!\n");
				printf("%p target time %1.4f\n", (void*)this, targetTime);
			}
			targetTime = simInfo->getSimulationTime();
		}
		onTarget = true;
//		return kStay;
	}
	else
		onTarget = false;
//	if (verbose)
//		printf("SU %p: Getting new path\n", this);
	path *p;
	p = algorithm->getPath(aMap, from, to, rp);
	nodesExpanded+=algorithm->getNodesExpanded();
	nodesTouched+=algorithm->getNodesTouched();

	// returning an empty path means there is no path between the start and goal
	if (p == NULL)
	{
		if (verbose)
			printf("SU %s: Path returned NIL\n", this->getName());
		return kStay;
	}
		
	if (!(p->n && p->next && p->next->n && (x == p->n->getLabelL(kFirstData)) 
				 && (y == p->n->getLabelL(kFirstData+1))))
	{
		if (p->n)
			std::cout << *p->n << std::endl;
		if ((p->next) && (p->next->n))
			std::cout << *p->next->n << std::endl;
		std::cout << x << ", " << y << std::endl;
	}

	// a valid path must have at least 2 nodes and start where the unit is located
	assert(p->n && p->next && p->next->n && (x == p->n->getLabelL(kFirstData)) 
				 && (y == p->n->getLabelL(kFirstData+1)));
	
	addPathToCache(p);
	if (s_algorithm)
	{
		node *next_start = p->tail()->n;
		s_algorithm->setTargets(mp->getMapAbstraction(), next_start, to, rp);
	}
	delete p;

	assert(moves.size() > 0);

	tDirection dir = moves.back();
	moves.pop_back();
//	if (verbose)
//		printf("SU %p: returning move 0x%X\n", this, (int)dir);
	return dir;
}

void searchUnit::addPathToCache(path *p)
{
	// we are at the last move; abort recursion
	if (p->next == NULL)
		return;
	// there is another move; add it first to cache
	if (p->next->next)
		searchUnit::addPathToCache(p->next);

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

void searchUnit::updateLocation(int _x, int _y, bool success, simulationInfo *)
{
	if (!success)
	{
		moves.clear();
		delete spread_cache;
		spread_cache = 0;
		if (verbose)
			printf("SU %s: clearing cached moves, (%d,%d)\n", this->getName(),_x,_y);
	}
	x = _x; y = _y;
}

void searchUnit::openGLDraw(mapProvider *mp, simulationInfo *si)
{
	GLdouble xx, yy, zz, rad;
	Map *map = mp->getMap();

	int posx = x, posy = y;
	map->getOpenGLCoord(posx, posy, xx, yy, zz, rad);
	glColor3f(r, g, b);
	glBegin(GL_LINE_STRIP);
	glVertex3f(xx, yy, zz-rad/2);
	for (int t = moves.size()-1; t >= 0; t--)
	{
		posx += ((moves[t]&kE)?1:0) - ((moves[t]&kW)?1:0);
		posy += ((moves[t]&kS)?1:0) - ((moves[t]&kN)?1:0);
		
		map->getOpenGLCoord(posx, posy, xx, yy, zz, rad);

		glVertex3f(xx, yy, zz-rad/2);
	}
	glEnd();
	
	// draw object
  map->getOpenGLCoord(x, y, xx, yy, zz, rad);
	if (onTarget)
	{
		double perc = (1.0-sqrt(sqrt(abs(sin(targetTime+0.25*si->getSimulationTime())))));
		glColor3f(r*perc, g*perc, b*perc);
	}
	else
		glColor3f(r, g, b);
	
	drawSphere(xx, yy, zz, rad);
}

void searchUnit::logStats(statCollection *stats)
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

void searchUnit::logFinalStats(statCollection *stats)
{
	algorithm->logFinalStats(stats);
}

//void searchUnit::printRoundStats(FILE *f)
//{
//	fprintf(f," %d", nodesExpanded);
//	nodesExpanded = 0;
//}
