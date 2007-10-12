/*
 * $Id: MapFlatAbstraction.cpp,v 1.6 2006/10/18 23:53:25 nathanst Exp $
 *
 *  MapFlatAbstraction.cpp
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

#include "MapFlatAbstraction.h"

MapFlatAbstraction::MapFlatAbstraction(Map *_m)
:MapAbstraction(_m)
{
	abstractions.push_back(GetMapGraph(_m));
	groupsValid = false;
}

MapFlatAbstraction::~MapFlatAbstraction()
{
}

void MapFlatAbstraction::buildConnectivityGroups()
{
	int nextNum = 0;
	Graph *g = abstractions[0];
	groups.resize(g->GetNumNodes());
	for (unsigned int x = 0; x < groups.size(); x++)
		groups[x] = -1;
	
	node_iterator ni = g->getNodeIter();
	for (node *iter = g->nodeIterNext(ni); iter; iter = g->nodeIterNext(ni))
	{
		std::vector<unsigned int> stack;
		if (groups[iter->GetNum()] == -1)
		{
			stack.push_back(iter->GetNum());
			while (stack.size() > 0)
			{
				unsigned int next = stack.back();
				stack.pop_back();
				if (groups[next] == -1)
				{
					groups[next] = nextNum;
				}
				neighbor_iterator n = g->GetNode(next)->getNeighborIter();
				for (int val = g->GetNode(next)->nodeNeighborNext(n); val != -1; val = g->GetNode(next)->nodeNeighborNext(n))
					if (groups[val] == -1)
						stack.push_back(val);
			}
			nextNum++;
		}
	}
	groupsValid = true;
}

bool MapFlatAbstraction::Pathable(node *from, node *to)
{
	if (!groupsValid)
		buildConnectivityGroups();
//	int x1, x2, y1, y2;
//	GetTileFromNode(from, x1, y1);
//	GetTileFromNode(to, x2, y2);
//	printf("(%d, %d) and (%d, %d) %s connected\n", x1, y1, x2, y2,
//				 ((groups[from->GetNum()] == groups[to->GetNum()])?"are":"are not"));
	return (groups[from->GetNum()] == groups[to->GetNum()]);
}

void MapFlatAbstraction::VerifyHierarchy()
{
}

void MapFlatAbstraction::RemoveNode(node *n)
{
	unsigned int oldID;
	abstractions[0]->RemoveNode(n, oldID);
}

void MapFlatAbstraction::RemoveEdge(edge *e, unsigned int)
{
	if (e) abstractions[0]->RemoveEdge(e);
}

void MapFlatAbstraction::AddNode(node *n)
{
//	n->SetLabelL(kAbstractionLevel, 0); // level in abstraction tree
//	n->SetLabelL(kNumAbstractedNodes, 1); // number of abstracted nodes
//	n->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
//	n->SetLabelF(kXCoordinate, kUnknownPosition);
//	n->SetLabelL(kNodeBlocked, 0);
//	n->SetLabelL(kFirstData, x);
//	n->SetLabelL(kFirstData+1, y);
//	n->SetLabelL(kFirstData+2, kNone);
	abstractions[0]->AddNode(n);
}

void MapFlatAbstraction::AddEdge(edge *e, unsigned int)
{
	abstractions[0]->AddEdge(e);	
}

void MapFlatAbstraction::RepairAbstraction()
{
	groupsValid = false;
}
