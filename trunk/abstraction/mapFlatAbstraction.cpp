/*
 * $Id: mapFlatAbstraction.cpp,v 1.6 2006/10/18 23:53:25 nathanst Exp $
 *
 *  mapFlatAbstraction.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 6/10/05.
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

#include "mapFlatAbstraction.h"

mapFlatAbstraction::mapFlatAbstraction(Map *_m)
:mapAbstraction(_m)
{
	abstractions.push_back(getMapGraph(_m));
	groupsValid = false;
}

mapFlatAbstraction::~mapFlatAbstraction()
{
}

void mapFlatAbstraction::buildConnectivityGroups()
{
	int nextNum = 0;
	graph *g = abstractions[0];
	groups.resize(g->getNumNodes());
	for (unsigned int x = 0; x < groups.size(); x++)
		groups[x] = -1;
	
	node_iterator ni = g->getNodeIter();
	for (node *iter = g->nodeIterNext(ni); iter; iter = g->nodeIterNext(ni))
	{
		std::vector<unsigned int> stack;
		if (groups[iter->getNum()] == -1)
		{
			stack.push_back(iter->getNum());
			while (stack.size() > 0)
			{
				unsigned int next = stack.back();
				stack.pop_back();
				if (groups[next] == -1)
				{
					groups[next] = nextNum;
				}
				neighbor_iterator n = g->getNode(next)->getNeighborIter();
				for (int val = g->getNode(next)->nodeNeighborNext(n); val != -1; val = g->getNode(next)->nodeNeighborNext(n))
					if (groups[val] == -1)
						stack.push_back(val);
			}
			nextNum++;
		}
	}
	groupsValid = true;
}

bool mapFlatAbstraction::pathable(node *from, node *to)
{
	if (!groupsValid)
		buildConnectivityGroups();
//	int x1, x2, y1, y2;
//	getTileFromNode(from, x1, y1);
//	getTileFromNode(to, x2, y2);
//	printf("(%d, %d) and (%d, %d) %s connected\n", x1, y1, x2, y2,
//				 ((groups[from->getNum()] == groups[to->getNum()])?"are":"are not"));
	return (groups[from->getNum()] == groups[to->getNum()]);
}

void mapFlatAbstraction::verifyHierarchy()
{
}

void mapFlatAbstraction::removeNode(node *n)
{
	unsigned int oldID;
	abstractions[0]->removeNode(n, oldID);
}

void mapFlatAbstraction::removeEdge(edge *e, unsigned int)
{
	if (e) abstractions[0]->removeEdge(e);
}

void mapFlatAbstraction::addNode(node *n)
{
//	n->setLabelL(kAbstractionLevel, 0); // level in abstraction tree
//	n->setLabelL(kNumAbstractedNodes, 1); // number of abstracted nodes
//	n->setLabelL(kParent, -1); // parent of this node in abstraction hierarchy
//	n->setLabelF(kXCoordinate, kUnknownPosition);
//	n->setLabelL(kNodeBlocked, 0);
//	n->setLabelL(kFirstData, x);
//	n->setLabelL(kFirstData+1, y);
//	n->setLabelL(kFirstData+2, kNone);
	abstractions[0]->addNode(n);
}

void mapFlatAbstraction::addEdge(edge *e, unsigned int)
{
	abstractions[0]->addEdge(e);	
}

void mapFlatAbstraction::repairAbstraction()
{
	groupsValid = false;
}
