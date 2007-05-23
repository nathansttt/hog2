/*
 *  CFOptimalRefinement.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/22/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */

#include "CFOptimalRefinement.h"
#include <vector>

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
	return p;
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
	return new path(aStart, new path(aGoal));
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
	g->AddNode(newTop);
	gStart = gGoal = newTop;
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
	gNewNode->SetLabelL(kAbstractionLevel, absGraph->GetAbstractionLevel(aRealNode));
	gNewNode->SetLabelL(kCorrespondingNode, aRealNode->GetNum());
	if (gParent)
	{
		gNewNode->SetLabelF(kGCost, gParent->GetLabelF(kGCost));
		gNewNode->SetLabelF(kHCost, gParent->GetLabelF(kHCost));
	}
	else {
		gNewNode->SetLabelF(kGCost, 0.0);
		gNewNode->SetLabelF(kHCost, 0.0);
	}
	gNewNode->SetLabelL(kOptimalFlag, 0);
	gNewNode->SetLabelL(kInOpenList, 1);

	if ((gParent == gStart) && (absGraph->IsParentOf(aRealNode, aStart)))
	{
		gStart = gNewNode;
	}
	if ((gParent == gGoal) && (absGraph->IsParentOf(aRealNode, aGoal)))
	{
		gGoal = gNewNode;
	}
}

void CFOptimalRefinement::UpdateNode(node *gNode)
{
	UpdateH(gNode);
	UpdateG(gNode);
	UpdateOptH(gNode);
	gNode->SetLabelL(kInOpenList, 0);
	q.Add(GNode(gNode));
}

void CFOptimalRefinement::UpdateH(node *gNode)
{
	double minH = DBL_MAX;

	// update h
	neighbor_iterator ni = gNode->getNeighborIter();
	for (int next = gNode->nodeNeighborNext(ni); next != -1; next = gNode->nodeNeighborNext(ni))
	{
		node *gNeighbor = g->GetNode(next);
		double tmpH = gNeighbor->GetLabelF(kHCost) + g->FindEdge(next, gNode->GetNum())->getWeight();
		if (fless(tmpH, minH))
			minH = tmpH;
	}
	if (fgreater(minH, gNode->GetLabelF(kHCost)) &&
			(gNode != gGoal))
	{
		gNode->SetLabelF(kHCost, minH);
		MakeNeighborsOpen(gNode);
	}
}

void CFOptimalRefinement::UpdateG(node *gNode)
{
	double minG = DBL_MAX;

	// update g
	neighbor_iterator ni = gNode->getNeighborIter();
	for (int next = gNode->nodeNeighborNext(ni); next != -1; next = gNode->nodeNeighborNext(ni))
	{
		node *gNeighbor = g->GetNode(next);
		double tmpG = gNeighbor->GetLabelF(kGCost) + g->FindEdge(next, gNode->GetNum())->getWeight();
		if (fless(tmpG, minG))
			minG = tmpG;
	}
	if (fgreater(minG, gNode->GetLabelF(kGCost)) &&
			(gNode != gStart))
	{
		gNode->SetLabelF(kGCost, minG);
		MakeNeighborsOpen(gNode);
	}
}

void CFOptimalRefinement::UpdateOptH(node *gNode)
{
	bool optH = false;
	if ((gNode->GetLabelL(kAbstractionLevel) == 0) &&
			(gNode->GetLabelL(kOptimalFlag) == 0))
	{
		if (gNode == gGoal)
		{
			optH = true;
		}
		else {
			neighbor_iterator ni = gNode->getNeighborIter();
			for (int next = gNode->nodeNeighborNext(ni); next != -1; next = gNode->nodeNeighborNext(ni))
			{
				node *gNeighbor = g->GetNode(next);
				if ((gNeighbor->GetLabelL(kOptimalFlag) == 1) &&
						(gNeighbor->GetLabelF(kGCost) + g->FindEdge(next, gNode->GetNum())->getWeight() ==
						 gNode->GetLabelF(kHCost)))
				{
					optH = true;
				}
			}
		}
		
		if ((gNode->GetLabelL(kOptimalFlag) == 0) && (optH))
		{
			gNode->SetLabelL(kOptimalFlag, 1);
			MakeNeighborsOpen(gNode);
		}
	}
}

void CFOptimalRefinement::MakeNeighborsOpen(node *gNode)
{
	neighbor_iterator ni = gNode->getNeighborIter();
	for (int next = gNode->nodeNeighborNext(ni); next != -1; next = gNode->nodeNeighborNext(ni))
	{
		node *gNeighbor = g->GetNode(next);
		if (gNeighbor->GetLabelL(kInOpenList) == 0)
		{
			gNeighbor->SetLabelL(kInOpenList, 1);
			q.DecreaseKey(GNode(gNeighbor));
		}
	}
}

void CFOptimalRefinement::RefineNode(node *gNode)
{
	std::vector<node *> aChildren;
	std::vector<node *> gChildren;
	node *aNode = GetRealNode(gNode);
	for (int x = 0; x < absGraph->GetNumChildren(aNode); x++)
	{
		aChildren.push_back(absGraph->GetNthChild(aNode, x));
	}

	// first, add children to graph g
	for (unsigned int x = 0; x < aChildren.size(); x++)
	{
		gChildren.push_back(new node("child"));
		SetInitialValues(gChildren[x], aChildren[x], gNode);
		g->AddNode(gChildren[x]);
	}

	// first, connect children to each other
	Graph *aGraph = absGraph->GetAbstractGraph(absGraph->GetAbstractionLevel(aChildren[0]));
	for (unsigned int x = 0; x < gChildren.size(); x++)
	{
		for (unsigned int y = 0; y < gChildren.size(); y++)
		{
			if (x != y)
			{
				edge *e;
				if ((e = aGraph->findDirectedEdge(aChildren[x]->GetNum(), aChildren[y]->GetNum())) != 0)
				{
					g->AddEdge(new edge(gChildren[x]->GetNum(), gChildren[y]->GetNum(),
															(absGraph->GetAbstractionLevel(aChildren[0])==0)?e->getWeight():1.0));
				}
			}
		}
	}

	// check neighbors of original node
	neighbor_iterator ni = gNode->getNeighborIter();
	for (int next = gNode->nodeNeighborNext(ni); next != -1; next = gNode->nodeNeighborNext(ni))
	{
		node *gNeighbor = g->GetNode(next);
		if (gNeighbor->GetLabelL(kAbstractionLevel) == gNode->GetLabelL(kAbstractionLevel)-1)
		{
			// same level
			for (unsigned int x = 0; x < gChildren.size(); x++)
			{
				edge *e;
				if ((e = aGraph->findDirectedEdge(aChildren[x]->GetNum(), GetRealNode(gNeighbor)->GetNum())) != 0)
				{
					g->AddEdge(new edge(gChildren[x]->GetNum(), gNeighbor->GetNum(),
															(absGraph->GetAbstractionLevel(aChildren[0])==0)?e->getWeight():1.0));
				}
			}
		}
		else if (gNeighbor->GetLabelL(kAbstractionLevel) < gNode->GetLabelL(kAbstractionLevel)-1)
		{ // neighbor is lower
			for (unsigned int x = 0; x < aChildren.size(); x++)
			{
				if (ShouldAddEdge(GetRealNode(gNeighbor), aChildren[x]))
					g->AddEdge(new edge(gChildren[x]->GetNum(), gNeighbor->GetNum(), 1.0));
			}
		}
		else { // neighbor is higher
			for (unsigned int x = 0; x < aChildren.size(); x++)
			{
				if (ShouldAddEdge(aChildren[x], GetRealNode(gNeighbor)))
					g->AddEdge(new edge(gChildren[x]->GetNum(), gNeighbor->GetNum(), 1.0));
			}
		}
	}
	
	// last thing we do!
	g->RemoveNode(gNode);
}

node *CFOptimalRefinement::GetRealNode(node *gNode)
{
	return absGraph->GetAbstractGraph(gNode->GetLabelL(kAbstractionLevel))->GetNode(gNode->GetLabelL(kCorrespondingNode));
}

bool CFOptimalRefinement::ShouldAddEdge(node *aLowerNode, node *aHigherNode)
{
	neighbor_iterator ni = aLowerNode->getNeighborIter();
	for (int next = aLowerNode->nodeNeighborNext(ni); next != -1; next = aLowerNode->nodeNeighborNext(ni))
	{
		node *aNeighbor = absGraph->GetAbstractGraph(absGraph->GetAbstractionLevel(aLowerNode))->GetNode(next);
		if (absGraph->IsParentOf(aHigherNode, aNeighbor))
			return true;
	}
	return false;
}


