/*
 *  SearchEnvironment.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 4/5/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "SearchEnvironment.h"

void MapSearchEnvironment::getNeighbors(uint32_t nodeID, std::vector<uint32_t> &neighbors)
{
	int x1, y1;
	bool up=false, down=false;
	x1 = nodeID>>16; y1 = nodeID&0xFFFF;
	if ((map->getTerrainType(x1, y1+1) == kGround))
	{
		down = true;
		neighbors.push_back((x1<<16)|(y1+1));
		if ((map->getTerrainType(x1, y1-1) == kGround))
		{
			up = true;
			neighbors.push_back((x1<<16)|(y1-1));
		}
		if ((map->getTerrainType(x1-1, y1) == kGround))
		{
			if ((up && (map->getTerrainType(x1-1, y1-1) == kGround)))
				neighbors.push_back(((x1-1)<<16)|(y1-1));
			if ((down && (map->getTerrainType(x1-1, y1+1) == kGround)))
				neighbors.push_back(((x1-1)<<16)|(y1+1));
			neighbors.push_back(((x1-1)<<16)| y1);
		}
		if ((map->getTerrainType(x1+1, y1) == kGround))
		{
			if ((up && (map->getTerrainType(x1+1, y1-1) == kGround)))
				neighbors.push_back(((x1+1)<<16)|(y1-1));
			if ((down && (map->getTerrainType(x1+1, y1+1) == kGround)))
				neighbors.push_back(((x1+1)<<16)|(y1+1));
			neighbors.push_back(((x1+1)<<16)| y1);
		}
	}
}

double MapSearchEnvironment::gcost(uint32_t node1, uint32_t node2)
{
	return heuristic(node1, node2);
}

double MapSearchEnvironment::heuristic(uint32_t node1, uint32_t node2)
{
	int x1, x2, y1, y2;
	x1 = node1>>16; y1 = node1&0xFFFF;
	x2 = node2>>16; y2 = node2&0xFFFF;
	double a = ((x1>x2)?(x1-x2):(x2-x1));
	double b = ((y1>y2)?(y1-y2):(y2-y1));
	return (a>b)?(b*ROOT_TWO+a-b):(a*ROOT_TWO+b-a);
	//return ((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

/** GraphSearchEnvironment
*/

void GraphSearchEnvironment::getNeighbors(uint32_t nodeID, std::vector<uint32_t> &neighbors)
{
	node *n1 = g->getNode(nodeID);
	neighbor_iterator ni = n1->getNeighborIter();
	for (int next = n1->nodeNeighborNext(ni); next != -1; next = n1->nodeNeighborNext(ni))
	{
		neighbors.push_back(next);
	}
}

double GraphSearchEnvironment::heuristic(uint32_t node1, uint32_t node2)
{
	edge *e = g->findEdge(node1, node2);
	if (e)
		return e->getWeight();
	return 0;
}

double GraphSearchEnvironment::gcost(uint32_t node1, uint32_t node2)
{
	return heuristic(node1, node2);
}
