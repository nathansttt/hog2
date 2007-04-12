/*
 * $Id: GenericAStar.cpp,v 1.12 2007/04/05 23:33:39 nathanst Exp $
 *
 *  GenericAStar.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 3/22/06.
 *  Copyright 2006 Nathan Sturtevant. All rights reserved.
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

#include "GenericAStar.h"
#include "float.h"

using namespace GenericAStarUtil;
static const bool verbose = false;

inline double max(double a, double b)
{
	if (fgreater(a, b))
		return a;
	return b;
}

const char *GenericAStar::getName()
{
	static char name[32];
	sprintf(name, "GenericAStar[]");
	return name;
}


void GenericAStar::getPath(SearchEnvironment *_env, uint32_t from, uint32_t to,
													 std::vector<uint32_t> &thePath)
{
	if (!initializeSearch(_env, from, to, thePath))
		return;
	while (!doSingleSearchStep(thePath)) {}
}

bool GenericAStar::initializeSearch(SearchEnvironment *_env, uint32_t from, uint32_t to,
																		std::vector<uint32_t> &thePath)
{
	env = _env;
	assert(openQueue.size() == 0);
	assert(closedList.size() == 0);
	nodesTouched = nodesExpanded = 0;
	start = from;
	goal = to;
	
	if ((from == UINT32_MAX) || (to == UINT32_MAX) || (from == to))
	{
		thePath.resize(0);
		return false;
	}

	SearchNode first(env->heuristic(goal, start), 0, start, start);
	openQueue.add(first);

	return true;
}

bool GenericAStar::doSingleSearchStep(std::vector<uint32_t> &thePath)
{
	uint32_t currentOpenNode = UINT32_MAX;

	if (openQueue.size() == 0)
	{
		thePath.resize(0); // no path found!
		return true;
	}
			
	// get top of queue
	currentOpenNode = getNextNode();
	
	if (env->goalTest(currentOpenNode, goal))
	{
		extractPathToStart(currentOpenNode, thePath);
		closedList.clear();
		openQueue.reset();
		env = 0;
		return true;
	}
	
	if (verbose) { printf("Opening %d\n", currentOpenNode); }
	
	if (currentOpenNode == UINT32_MAX)
		printf("Oh no! The current open node is NULL\n");
	
	neighbors.resize(0);
	env->getNeighbors(currentOpenNode, neighbors);

	// iterate over all the children
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		nodesTouched++;
		uint32_t neighbor = neighbors[x];
		assert(neighbor != UINT32_MAX);
		
		if (closedList.find(neighbor) != closedList.end())
		{
			if (verbose) { printf("skipping node %d\n", neighbor); }
			continue;
		}
		else if (openQueue.isIn(SearchNode(neighbor)))
		{
			if (verbose) { printf("updating node %d\n", neighbor); }
			updateWeight(currentOpenNode, neighbor);
		}
		else {
			if (verbose) { printf("addinging node %d\n", neighbor); }
			addToOpenList(currentOpenNode, neighbor);
		}
	}
	return false;
}

uint32_t GenericAStar::checkNextNode()
{
	return openQueue.top().currNode;
}

uint32_t GenericAStar::getNextNode()
{
	nodesExpanded++;
	uint32_t next;
	SearchNode it = openQueue.remove();
	next = it.currNode;
	closedList[next] = it;
	return next;
}

void GenericAStar::updateWeight(uint32_t currOpenNode, uint32_t neighbor)
{
	SearchNode prev = openQueue.find(SearchNode(neighbor));
	SearchNode alt = closedList[currOpenNode];
	double edgeWeight = env->gcost(currOpenNode, neighbor);
	double altCost = alt.gCost+edgeWeight+(prev.fCost-prev.gCost);
	if (fgreater(prev.fCost, altCost))
	{
		prev.fCost = altCost;
		prev.gCost = alt.gCost+edgeWeight;
		prev.prevNode = currOpenNode;
		openQueue.decreaseKey(prev);
	}
}

void GenericAStar::addToOpenList(uint32_t currOpenNode, uint32_t neighbor)
{
	double edgeWeight = env->gcost(currOpenNode, neighbor);
	SearchNode n(closedList[currOpenNode].gCost+edgeWeight+env->heuristic(neighbor, goal),
							 closedList[currOpenNode].gCost+edgeWeight,
							 neighbor, currOpenNode);
	if (verbose) 
	{ printf("Adding %u to openQueue, old size %u\n", neighbor, openQueue.size()); }
	openQueue.add(n);
}

void GenericAStar::extractPathToStart(uint32_t goalNode,
																			std::vector<uint32_t> &thePath)
{
	SearchNode n;
	if (closedList.find(goalNode) != closedList.end())
	{
		n = closedList[goalNode];
	}
	else n = openQueue.find(SearchNode(goalNode));

	do {
		thePath.push_back(n.currNode);
		n = closedList[n.prevNode];
	} while (n.currNode != n.prevNode);
	thePath.push_back(n.currNode);
}

void GenericAStar::printStats()
{
	printf("%u items in closed list\n", (unsigned int)closedList.size());
	printf("%u items in open queue\n", (unsigned int)openQueue.size());
}

int GenericAStar::getMemoryUsage()
{
	return closedList.size()+openQueue.size();
}

closedList_iterator GenericAStar::getClosedListIter() const
{
	return closedList.begin();
}

uint32_t GenericAStar::closedListIterNext(closedList_iterator& it) const
{
	if (it == closedList.end())
		return UINT32_MAX;
	uint32_t val = (*it).first;
	it++;
	return val;
}
