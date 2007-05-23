/*
 *  CFOptimalRefinement.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/22/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */

#include "CFOptimalRefinement.h"

CFOptimalRefinement::CFOptimalRefinement()
:SearchAlgorithm()
{
	g = 0;
}

CFOptimalRefinement::~CFOptimalRefinement()
{
}

const char *CFOptimalRefinement::GetName()
{
	return "CFOptimalRefinement";
}

path *CFOptimalRefinement::GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *)
{
	InitializeSearch(aMap, from, to);
	path *p = 0;
	while (p == 0)
		p = DoOneSearchStep();
}

path *CFOptimalRefinement::DoOneSearchStep()
{
	if ((gStart == 0) || (gStart->GetLabelL(kOptimalFlag) == 0))
	{
		node *gNext = q.top().n;
		q.pop();
		if (gNext->GetLabelL(kInOpenList) == 1)
		{
			UpdateNode(gNext);
		}
		else { // in closed list
			RefineNode(gNext);
		}
	}
}

void CFOptimalRefinement::InitializeSearch(GraphAbstraction *aMap, node *from, node *to)
{
	gStart = 0;
	absGraph = aMap;
	aStart = from;
	aGoal = to;
	delete g;
	g = new Graph();
	
	// find most abstract node in Graph
	node *top = FindTopLevelNode(from, to, aMap);
	node *newTop = new node("top");
	SetInitialValues(newTop, top, 0);
	q.Add(GNode(newTop));
	g->addNode(newTop);
}

node *CFOptimalRefinement::FindTopLevelNode(node *one, node *two, GraphAbstraction *aMap)
{
	if ((one == 0) || (two == 0))
		return 0;
	if (aMap->GetAbstractionLevel(one) >= aMap->getNumAbstractGraphs())
		return 0;
	if (aMap->GetAbstractionLevel(one) == aMap->getNumAbstractGraphs() - 1)
	{
		if (one == two)
			return one;
		return 0;
	}
	node *tmp = FindTopLevelNode(aMap->GetParent(one), aMap->GetParent(two), aMap);
	if ((tmp == 0) && (one == two))
		return one;
	return 0;
}

/*
 * aRealNode is the node inside the absGraph
 * gNewNode is the node about to be added to Graph
 * gParent is the parent inside the Graph
 */
void CFOptimalRefinement::SetInitialValues(node *gNewNode, node *aRealNode, node *gParent)
{
	gNewNode->SetLabelL(kAbstractionLevel, absGraph->getAbstractionLevel(aRealNode));
	gNewNode->SetLabelL(kCorrespondingNode, aRealNode->getNum());
	if (gParent)
	{
		gNewNode->setLabel(kGCost, gParent->GetLabelL(kGCost));
		gNewNode->setLabel(kHCost, gParent->GetLabelL(kHCost));
	}
	else {
		gNewNode->SetLabelF(kGCost, 0.0);
		gNewNode->SetLabelF(kHCost, 0.0);
	}
	gNewNode->SetLabelL(kOptimalFlag, 0);
	gNewNode->SetLabelL(kInOpenList, 1);
	if (aRealNode == aStart)
	{
		gStart = gNewNode;
	}
}

void CFOptimalRefinement::UpdateNode(node *gNode)
{
	double minH = DBL_MAX;

	neighbor_iterator ni = gNode->getNeighborIter();
	for (int next = gNode->nodeNeighborNext(ni); next != -1; next = gNode->nodeNeighborNext(ni))
	{
		node *gNeighbor = g->GetNode(next);
	}
		
}

void CFOptimalRefinement::RefineNode(node *gNode)
{
	
}