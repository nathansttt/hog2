/*
 *  FringeSearch.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 1/19/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */

#include "FringeSearch.h"
#include "FPUtil.h"

#ifndef MAXFLOAT
#define MAXFLOAT ((float)3.40282346638528860e+38)
#endif

using namespace GraphAbstractionConstants;
static bool verbose = false;

FringeSearch::FringeSearch()
:SearchAlgorithm()
{
	bpmx = false;
	currList = &list1;
	nextList = &list2;
	hp = 0;
}

path *FringeSearch::GetPath(GraphAbstraction *_aMap, node *from, node *to, reservationProvider *)
{
	initializeSearch(_aMap, from, to);
	
	if ((from == 0) || (to == 0) || (!aMap->Pathable(from, to)) || (from == to))
		return 0;

	currList->push_back(from);
	currFLimit = getHCost(from);
	nextFLimit = MAXFLOAT;
	node *currOpenNode;
	while (currList->size() > 0)
	{
		currOpenNode = currList->back();
		currList->pop_back();
		if (currOpenNode == 0)
		{
			checkIteration();
			continue;
		}
		if (verbose) printf("Expanding %d\n", currOpenNode->GetNum());
		
		if (fgreater(getFCost(currOpenNode), currFLimit))
		{
			if (verbose) printf("FCost %f above limit %f\n", getFCost(currOpenNode), currFLimit);
			nextFLimit = min(nextFLimit, getFCost(currOpenNode));
			nodesTouched++;
			addToOpenList2(currOpenNode);
			checkIteration();
			continue;
		}

		if (currOpenNode == to)
			break;
		
		addToClosedList(currOpenNode);
		nodesExpanded++;

		edge_iterator ei = currOpenNode->getEdgeIter();
		
		// iterate over all the children
		for (edge *e = currOpenNode->edgeIterNext(ei); e; e = currOpenNode->edgeIterNext(ei))
		{
			nodesTouched++;
			unsigned int which;
			if ((which = e->getFrom()) == currOpenNode->GetNum()) which = e->getTo();
			node *neighbor = g->GetNode(which);
			assert(neighbor != 0);

			if (onClosedList(neighbor))
			{
				if (verbose) printf("->Child %d on closed list\n", neighbor->GetNum());
				updateCosts(neighbor, currOpenNode, e);
				continue;
			}
			else if (onOpenList(neighbor))
			{
				if (verbose) printf("->Child %d on open list\n", neighbor->GetNum());
				updateCosts(neighbor, currOpenNode, e);
			}
			else // not on any list
			{
				addCosts(neighbor, currOpenNode, e);
				addToOpenList(neighbor);
				if (verbose) printf("->Child %d new to search f(%f) g(%f) h(%f)\n", neighbor->GetNum(),
							 getFCost(neighbor), getGCost(neighbor), getHCost(neighbor));
			}
		}
		checkIteration();
	}
	//printf("Fringe %d nodes expanded, %d nodes touched\n", nodesExpanded, nodesTouched);
	return extractBestPath(to);
}

void FringeSearch::initializeSearch(GraphAbstraction *aGraph, node *from, node *to)
{
	nodesTouched = 0;
	nodesExpanded = 0;
	nodesReopened = 0;
	nodesHPropagated = 0;
	list1.resize(0);
	list2.resize(0);
	closedList.resize(0);
	costTable.resize(0);
	aMap = aGraph;
	goal = to;
	g = aMap->GetAbstractGraph(from->GetLabelL(kAbstractionLevel));

	addCosts(from, 0, 0);
}

void FringeSearch::addToOpenList(node *n)
{
	n->key = currList->size();
	currList->push_back(n);
}

void FringeSearch::moveToOpenList1(node *n)
{
	if ((n->key < currList->size()) && ((*currList)[n->key] == n))
		return;
	if ((n->key < nextList->size()) && ((*nextList)[n->key] == n))
	{
		if (verbose) printf("Moved %d to current open list\n", n->GetNum());
		(*nextList)[n->key] = 0;
		n->key = currList->size();
		currList->push_back(n);
	}
}

void FringeSearch::addToOpenList2(node *n)
{
	n->key = nextList->size();
	nextList->push_back(n);
}

void FringeSearch::addToClosedList(node *n)
{
	n->key = closedList.size();
	closedList.push_back(n);
}

bool FringeSearch::onClosedList(node *n)
{
	if ((n->key < closedList.size()) && (closedList[n->key] == n))
		return true;
	return false;
}

bool FringeSearch::onOpenList(node *n)
{
	if ((n->key < list1.size()) && (list1[n->key] == n))
		return true;
	if ((n->key < list2.size()) && (list2[n->key] == n))
		return true;
	return false;
}

double FringeSearch::getFCost(node *n)
{
	if (n == 0)
		return 0;
	costs val;
	getCosts(n, val);
	return val.gCost+val.hCost;
}

double FringeSearch::getGCost(node *n)
{
	if (n == 0)
		return 0;
	costs val;
	getCosts(n, val);
	return val.gCost;
}

double FringeSearch::getHCost(node *n)
{
	if (n == 0)
		return MAXFLOAT;
	costs val;
	getCosts(n, val);
	return val.hCost;
}

void FringeSearch::setHCost(node *n, double val)
{
	unsigned long index = n->GetLabelL(kTemporaryLabel);
	if ((index < costTable.size()) && (costTable[index].n == n))
	{
		costTable[index].hCost = val;
		return;
	}
	printf("setHCost Error: node %d not found!\n", n->GetNum());
	assert(false);
}

void FringeSearch::getCosts(node *n, costs &val)
{
	unsigned long index = n->GetLabelL(kTemporaryLabel);
	if ((index < costTable.size()) && (costTable[index].n == n))
	{
		val = costTable[index];
		return;
	}
	printf("Error: node %d not found!\n", n->GetNum());
	assert(false);
}

void FringeSearch::addCosts(node *n, node *parent, edge *e)
{
	n->markEdge(e);
	costs val;
	val.n = n;
	if ((parent) && (e))
	{
		val.gCost = getGCost(parent)+e->GetWeight();
		val.hCost = h(n, goal);
		n->SetLabelL(kTemporaryLabel, costTable.size());
		costTable.push_back(val);

		if (fgreater(getHCost(parent)-e->GetWeight(), h(n, goal)))
		{ // regular path max
			if (verbose) printf("Doing regular path max!\n");
			setHCost(n, getHCost(parent)-e->GetWeight());
		}
		if (bpmx)
		{
			if (fgreater(val.hCost-e->GetWeight(), getHCost(parent)))
			{ // reverse path max!
				if (verbose) printf("-> %d h value raised from %f to %f\n", parent->GetNum(),
														getHCost(parent), getHCost(n)-e->GetWeight());
				setHCost(parent, getHCost(n)-e->GetWeight());
				if (verbose) printf("Doing reverse path max!\n");
				propagateHValues(parent, 2);
			}
		}
	}
	else {
		val.gCost = getGCost(parent);
		val.hCost = h(n, goal);
		n->SetLabelL(kTemporaryLabel, costTable.size());
		costTable.push_back(val);
	}
}

void FringeSearch::updateCosts(node *n, node *parent, edge *e)
{
	if (fgreater(getGCost(n), getGCost(parent)+e->GetWeight()))
	{
		n->markEdge(e);
		costs &val = costTable[n->GetLabelL(kTemporaryLabel)];
		if (verbose) printf("Updated g-cost of %d from %f to %f (through %d) -- (%f limit)\n", n->GetNum(),
												val.gCost, getGCost(parent)+e->GetWeight(), parent->GetNum(), currFLimit);
		val.gCost = getGCost(parent)+e->GetWeight();
		propagateGValues(n);
		// I check the nextFLimit, because we might want to update it for this node
		nextFLimit = min(nextFLimit, getFCost(n));
	}
}

path *FringeSearch::extractBestPath(node *n)
{
	path *p = 0;
	edge *e;
	// extract best path from Graph -- each node has a single parent in the Graph which is the marked edge
	// for visuallization purposes, an edge can be marked meaning it will be drawn in white
	while ((e = n->getMarkedEdge()))
	{
		if (verbose) printf("%d <- ", n->GetNum());
		
		p = new path(n, p);
		
		e->setMarked(true);
		
		if (e->getFrom() == n->GetNum())
			n = g->GetNode(e->getTo());
		else
			n = g->GetNode(e->getFrom());
	}
	p = new path(n, p);
	if (verbose) printf("%d\n", n->GetNum());
	return p;	
}

void FringeSearch::checkIteration()
{
	if (currList->size() == 0) // swap our lists!
	{
		nodeList *tmp = currList;
		currList = nextList;
		nextList = tmp;
		currFLimit = nextFLimit;
		nextFLimit = MAXFLOAT;
		if (verbose)
			printf("Beginning new iteration, flimit %f, %d items in q\n",
												currFLimit, (int)currList->size());
	}
}

// just figure out how/when to call this!
void FringeSearch::propagateHValues(node *n, int dist)
{
	if (dist == 0)
		return;
	nodesExpanded++;
	nodesHPropagated++;
	edge_iterator ei = n->getEdgeIter();
	
	// iterate over all the children
	for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
	{
		nodesTouched++;
		unsigned int which;
		if ((which = e->getFrom()) == n->GetNum()) which = e->getTo();
		node *neighbor = g->GetNode(which);
		
		if (onClosedList(neighbor) || onOpenList(neighbor))
		{
			if (fless(getHCost(neighbor), getHCost(n)-e->GetWeight())) // do update!
			{
				if (verbose) printf("%d h value raised from %f to %f\n", neighbor->GetNum(),
														getHCost(neighbor), getHCost(n)-e->GetWeight());
				setHCost(neighbor, getHCost(n)-e->GetWeight());
				propagateHValues(neighbor, dist-1);
			}
		}
	}			
}

// just figure out how/when to call this!
void FringeSearch::propagateGValues(node *n)
{
	if (onClosedList(n))
		nodesReopened++;

	nodesExpanded++;
	edge_iterator ei = n->getEdgeIter();
	if (onOpenList(n) && (!fgreater(getFCost(n), currFLimit)))
		moveToOpenList1(n);
	
	// iterate over all the children
	for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
	{
		nodesTouched++;
		unsigned int which;
		if ((which = e->getFrom()) == n->GetNum()) which = e->getTo();
		node *neighbor = g->GetNode(which);
		
		if ((onOpenList(neighbor) || onClosedList(neighbor)))// && (neighbor->getMarkedEdge() == e)
		{
			updateCosts(neighbor, n, e);
		}
	}
}


double FringeSearch::h(node *n1, node *n2)
{
	if (hp)
		return hp->h(n1->GetNum(), n2->GetNum());
	if ((n1->GetNum()+n2->GetNum())%2)
		return aMap->h(n1, n2);
	return 0;
}

