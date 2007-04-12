/*
 * $Id: praStarUnit.cpp,v 1.4 2006/09/18 06:19:31 nathanst Exp $
 *
 *  praStarUnit.cpp
 *  HOG
 *
 *  Created by Nathan Sturtevant on 1/16/05.
 *  Copyright 2005 Nathan Sturtevant. All rights reserved.
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

#include "praStarUnit.h"

praStarUnit::praStarUnit(int _x, int _y, unit *_target, praStar *_alg)
:searchUnit(_x, _y, _target, _alg), algorithm(_alg)
{ cache = 0; }

//praStarUnit::praStarUnit(int _x, int _y, int _r, int _g, int _b, unit *_target, praStar *_alg)
//:searchUnit(_x, _y, _r, _g, _b, _target, _alg), algorithm(_alg)
//{ cache = 0; }


tDirection praStarUnit::makeMove(mapProvider *mp, reservationProvider *rp, simulationInfo *simInfo)
{
	mapAbstraction *aMap = mp->getMapAbstraction();

	algorithm->setExpandSearchRadius(false);
	if (moves.size() != 0)
	{
		return searchUnit::makeMove(mp, rp, simInfo);
	}
	if (cache != 0)
	{
		node *n = aMap->getNodeFromMap(x, y);
		node *l = aMap->getNthParent(n, cache->n->getLabelL(kAbstractionLevel));
		while (cache && (l != cache->n))
		{
			path *tmp = cache;
			cache = cache->next;
			tmp->next = 0;
			delete tmp;
		}
		if (cache)
			assert(l == cache->n);
		if (cache && (cache->next == 0))
		{
			delete cache;
			cache = 0;
		}
	}
	algorithm->setCache(&cache);
	//printf("--- cached PRA --- start ---\n");
	tDirection where =  searchUnit::makeMove(mp, rp, simInfo);
	//printf("--- cached PRA --- finish ---\n");
	return where;
}
