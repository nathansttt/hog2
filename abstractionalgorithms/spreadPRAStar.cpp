/*
 * $Id: spreadPRAStar.cpp,v 1.5 2006/10/24 18:18:55 nathanst Exp $
 *
 *  spreadPRAStar.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 6/27/05.
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

#include "spreadPRAStar.h"
#include "fpUtil.h"

static const int verbose = 0;

spreadPRAStar::spreadPRAStar()
{
	setPartialPathLimit(-1);
	lastPath = 0;
	expandSearchRadius = true;
}

path *spreadPRAStar::getPath(graphAbstraction *_aMap, node *from, node *to, reservationProvider *_rp)
{
  std::vector<node *> fromChain;
  std::vector<node *> toChain;
	path *lastPth = 0;
	
  if (_aMap->getAbstractGraph(from->getLabelL(kAbstractionLevel))->findEdge(from->getNum(), to->getNum()))
    return new path(from, new path(to));

	setupSearch(_aMap, fromChain, from, toChain, to);
	if (fromChain.size() == 0)
		return 0;
  do {
		lastPth = buildNextAbstractPath(_aMap, lastPth, fromChain, toChain, _rp);
  } while (lastPth && (lastPth->n->getLabelL(kAbstractionLevel) > 0));
	return lastPth;
}

void spreadPRAStar::setTargets(graphAbstraction *_aMap, node *s, node *e, reservationProvider *_rp)
{
	rp = _rp; start = s; end = e; aMap = _aMap;
	startChain.resize(0);
	endChain.resize(0);
	setupSearch(aMap, startChain, start, endChain, end);
	delete lastPath;
	lastPath = 0;
}

/** how many times do we have to "think" to find the solution, return -1 if unknown */
int spreadPRAStar::getNumThinkSteps()
{
	return -1;
}

/** do next processing for path, returns avaliability of path moves */
path *spreadPRAStar::think()
{
	nodesExpanded = 0;
	nodesTouched = 0;
	if (startChain.size() == 0)
		return 0;
	lastPath = buildNextAbstractPath(aMap, lastPath, startChain, endChain, rp);
	if (lastPath && (lastPath->n->getLabelL(kAbstractionLevel) == 0))
	{
		path *p = lastPath;
		lastPath = 0;
		return p;
	}
	return 0;
}

void spreadPRAStar::setupSearch(graphAbstraction *_aMap,
																std::vector<node *> &fromChain, node *from,
																std::vector<node *> &toChain, node *to)
{
	nodesExpanded = 0;
	nodesTouched = 0;
	
  if ((from == 0) || (to == 0) || (!_aMap->pathable(from, to)) || (from == to))
	{
    if (verbose)
		{
      if (!from)
				printf("spreadPRAStar: from == 0\n");
      if (!to)
				printf("spreadPRAStar: to == 0\n");
      if (from == to)
				printf("spreadPRAStar: from == to\n");
      if (from && to && (!_aMap->pathable(from, to)))
				printf("spreadPRAStar: no path from %p to %p\n", (void*)from, (void*)to);
			//cout << "praStar: no path from " << *from << " to " << *to << endl;
    }
		return;
  }
	
  if (verbose)
    printf("At nodes #%d and %d\n", from->getNum(), to->getNum());
//  if (aMap->getAbstractGraph(0)->findEdge(from->getNum(), to->getNum()))
//	{ // we are 1 step away
//    return new path(from, new path(to));
//  }
	
  _aMap->getParentHierarchy(from, to, fromChain, toChain);
	//	assert(aMap->getAbstractGraph(fromChain.back()->getLabelL(kAbstractionLevel))->
	//					findEdge(fromChain.back()->getNum(), toChain.back()->getNum()));
	
	unsigned int previousSize = fromChain.size();
	int minNode = (int)(2*sqrt(_aMap->getAbstractGraph(0)->getNumNodes()));
	while ((fromChain.size() > 2) && ((fromChain.size() > (previousSize)/2) ||
																		(_aMap->getAbstractGraph(fromChain.size())->getNumNodes() < minNode)))
	{
		toChain.pop_back();
		fromChain.pop_back();
	}
}

path *spreadPRAStar::buildNextAbstractPath(graphAbstraction *_aMap, path *lastPth,
																			std::vector<node *> &fromChain,
																			std::vector<node *> &toChain,
																			reservationProvider *_rp)
{
	node *to, *from, *hTarget = 0;
	to = toChain.back();
	toChain.pop_back();
	from = fromChain.back();
	fromChain.pop_back();
	
	if (verbose)
		printf("Expanded %d nodes before doing level %d\n", nodesExpanded, (int)from->getLabelL(kAbstractionLevel));		
	
	if (verbose)
		printf("Building path from %d to %d (%ld/%ld)\n",
					 from->getNum(), to->getNum(), from->getLabelL(kParent), to->getLabelL(kParent));
	
	std::vector<node *> eligibleNodeParents;
	
	if (lastPth)
	{
		// cut path down to size of partial path limit
		if (partialLimit > 0)
		{
			path *trav = lastPth;
			
			for (int x = 0; x < partialLimit; x++)
				if (trav->next) 
					trav = trav->next;
			// we don't need to reset the target if we have a complete path
			// but we do if our complete path doesn't end in our target node
			if ((trav->next != 0) || ((trav->next == 0) && ((long)trav->n->getNum() != to->getLabelL(kParent))))
			{
				to = trav->n;
				if (trav->next)
					hTarget = trav->next->n;
				else
					hTarget = to;
				delete trav->next;
				trav->next = 0;
			}
		}
		
		graph *g = _aMap->getAbstractGraph(lastPth->n->getLabelL(kAbstractionLevel));
		// find eligible nodes for lower level expansions
		for (path *trav = lastPth; trav; trav = trav->next)
		{
			if (expandSearchRadius)
			{
				edge_iterator ei = trav->n->getEdgeIter();
				for (edge *e = trav->n->edgeIterNext(ei); e; e = trav->n->edgeIterNext(ei)) {
					if (e->getFrom() == trav->n->getNum())
						eligibleNodeParents.push_back(g->getNode(e->getTo()));
					else
						eligibleNodeParents.push_back(g->getNode(e->getFrom()));
				}
			}
			eligibleNodeParents.push_back(trav->n);
		}
	}
	cAStar.setCorridor(&eligibleNodeParents);
	path *result;
	if (hTarget != 0)
	{
		result = cAStar.getBestPath(_aMap, from, to, hTarget, _rp);
		if (result == 0)
		{
			if (verbose) printf("NULL result from getBestPath in spreadPRAStar\n");
		}
	}
	else {
		result = cAStar.getPath(_aMap, from, to, _rp);
		if (result == 0)
		{
			if (verbose) printf("NULL result from getPath in spreadPRAStar\n");
		}
	}
	nodesExpanded += cAStar.getNodesExpanded();
	nodesTouched += cAStar.getNodesTouched();
	return result;
}

path *spreadPRAStar::trimPath(path *lastPth, node *origDest)
{
	if (partialLimit != -1)
	{
		int parent = -1;
		path *change = 0, *last = 0;
		for (path *trav = lastPth; trav; trav = trav->next)
		{
			if (trav->n == origDest)
				return lastPth;
			if (trav->n->getLabelL(kParent) != parent)
			{
				parent = trav->n->getLabelL(kParent);
				change = last;
			}
			last = trav;
		}
		if (change)
		{
			delete change->next;
			change->next = 0;
		}
	}
	return lastPth;
}
