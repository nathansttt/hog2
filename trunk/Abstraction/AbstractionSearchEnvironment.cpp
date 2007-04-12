/*
 *  AbstractionSearchEnvironment.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 4/5/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "AbstractionSearchEnvironment.h"


void AbstractionSearchEnvironment::getNeighbors(uint32_t nodeID, std::vector<uint32_t> &neighbors)
{
	graph *g = ga->getAbstractGraph(level);
	node *n1 = g->getNode(nodeID);
	neighbor_iterator ni = n1->getNeighborIter();
	for (int next = n1->nodeNeighborNext(ni); next != -1; next = n1->nodeNeighborNext(ni))
	{
		neighbors.push_back(next);
	}
}

double AbstractionSearchEnvironment::heuristic(uint32_t node1, uint32_t node2)
{
	graph *g = ga->getAbstractGraph(level);
	node *n1 = g->getNode(node1);
	node *n2 = g->getNode(node2);
	edge *e = g->findEdge(node1, node2);
	if (e)
		return e->getWeight();
	return ga->h(n1, n2);
}

double AbstractionSearchEnvironment::gcost(uint32_t node1, uint32_t node2)
{
	return heuristic(node1, node2);
}
