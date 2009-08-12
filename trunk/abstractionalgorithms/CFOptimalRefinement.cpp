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

using namespace CFOptimalRefinementConstants;

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
	if (!InitializeSearch(aMap, from, to))
		return 0;
	path *p = 0;
	while (p == 0)
	{
		p = DoOneSearchStep();
		//g->Print(std::cout);
	}
	return p;
}

path *CFOptimalRefinement::DoOneSearchStep()
{
	if ((gStart == 0) || (gStart->GetLabelL(kOptimalFlag) == 0))
	{
		node *gNext = q.top().n;
		q.pop();
		printf("Analyzing %d next\n", gNext->GetNum());
		std::cout << *gNext << std::endl;
		if (gNext->GetLabelL(kInOpenList) == 1)
		{
			printf("Updating node %d\n", gNext->GetNum());
			UpdateNode(gNext);
		}
		else { // in closed list
			printf("Refining node %d\n", gNext->GetNum());
			if (gNext->GetLabelL(kAbstractionLevel) > 0)
				RefineNode(gNext);
		}
	}
	if ((gStart == 0) || (gStart->GetLabelL(kOptimalFlag) == 0))
	{
		return 0;
	}
	return new path(aStart, new path(aGoal));
}

bool CFOptimalRefinement::InitializeSearch(GraphAbstraction *aMap, node *from, node *to)
{
	gStart = 0;
	absGraph = aMap;
	aStart = from;
	aGoal = to;
	delete g;
	g = new Graph();
	
	// find most abstract node in Graph
	node *top = FindTopLevelNode(from, to, aMap);
	if (top == 0)
		return false;
	node *newTop = new node("top");
	gStart = gGoal = newTop;
	g->AddNode(newTop);
	SetInitialValues(newTop, top, 0);
	q.Add(GNode(newTop));
	return true;
}

node *CFOptimalRefinement::FindTopLevelNode(node *one, node *two, GraphAbstraction *aMap)
{
	if ((one == 0) || (two == 0))
		return 0;
	if (aMap->GetAbstractionLevel(one) >= (int)aMap->getNumAbstractGraphs())
		return 0;
	if (aMap->GetAbstractionLevel(one) == (int)aMap->getNumAbstractGraphs() - 1)
	{
		if (one == two)
			return one;
		return 0;
	}
	node *tmp = FindTopLevelNode(aMap->GetParent(one), aMap->GetParent(two), aMap);
	if ((tmp == 0) && (one == two))
		return one;
	return tmp;
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
		std::cout << "Assigning start to " << *gNewNode << std::endl;
		gStart = gNewNode;
	}
	if ((gParent == gGoal) && (absGraph->IsParentOf(aRealNode, aGoal)))
	{
		std::cout << "Assigning goal to " << *gNewNode << std::endl;
		gGoal = gNewNode;
	}
}

void CFOptimalRefinement::UpdateNode(node *gNode)
{
	UpdateH(gNode);
	std::cout << "After UpdateH " << *gNode << std::endl;
	UpdateG(gNode);
	std::cout << "After UpdateG " << *gNode << std::endl;
	UpdateOptH(gNode);
	std::cout << "After UpdateOptH " << *gNode << std::endl;
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
		double tmpH = gNeighbor->GetLabelF(kHCost) + g->FindEdge(next, gNode->GetNum())->GetWeight();
		if (fless(tmpH, minH))
			minH = tmpH;
	}
	if (gNode->GetLabelL(kAbstractionLevel) == 0)
		minH = std::max(minH, absGraph->h(gNode, aGoal));
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
		double tmpG = gNeighbor->GetLabelF(kGCost) + g->FindEdge(next, gNode->GetNum())->GetWeight();
		if (fless(tmpG, minG))
			minG = tmpG;
	}
	if (gNode->GetLabelL(kAbstractionLevel) == 0)
		minG = std::max(minG, absGraph->h(gNode, aStart));
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
						(fequal(gNeighbor->GetLabelF(kHCost) + g->FindEdge(next, gNode->GetNum())->GetWeight(),
										gNode->GetLabelF(kHCost))))
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
		g->AddNode(gChildren[x]);
		SetInitialValues(gChildren[x], aChildren[x], gNode);
		q.Add(GNode(gChildren[x]));
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
					g->AddEdge(new edge(gChildren[x]->GetNum(), gChildren[y]->GetNum(), //1.0));
															(absGraph->GetAbstractionLevel(aChildren[0])==0)?e->GetWeight():1.5));
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
				if ((e = aGraph->FindEdge(aChildren[x]->GetNum(), GetRealNode(gNeighbor)->GetNum())) != 0)
				{
					g->AddEdge(new edge(gChildren[x]->GetNum(), gNeighbor->GetNum(), //1.0));
															(absGraph->GetAbstractionLevel(aChildren[0])==0)?e->GetWeight():1.5));
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

node *CFOptimalRefinement::GetRealNode(node *gNode) const
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

void CFOptimalRefinement::OpenGLDraw() const
{
	if ((g == 0) || (g->GetNumNodes() == 0))
	{
		return;
	}
	
	glLineWidth(3.0);
	glBegin(GL_LINES);
	glNormal3f(0, 1, 0);
	edge_iterator ei = g->getEdgeIter();
	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		const node *n1;
		n1 = g->GetNode(e->getFrom());
		if (q.peek().n == n1)
			glColor3f(1.0, 0.0, 1.0);
		else if (n1 == gStart)
			glColor3f(0, 0, 1);
		else if (n1 == gGoal)
			glColor3f(0, 1, 0);
		else if (n1->GetLabelL(kOptimalFlag) && n1->GetLabelL(kInOpenList))
			glColor3f(1.0, 1.0, 0);
		else if (n1->GetLabelL(kOptimalFlag))
			glColor3f(0.6, 0.6, 0);
		else if (n1->GetLabelL(kInOpenList) == 0)
			glColor3f(0.5, 0.5, 0.5);
		else
			glColor3f(1, 1, 1);
		
		node *n = 0; assert(false); // there is a bug here, because n was always uninitialized
		recVec rv = absGraph->GetNodeLoc(GetRealNode(n));
		glVertex3f(rv.x, rv.y, rv.z);
		
		n = g->GetNode(e->getTo());
		if (q.peek().n == n)
			glColor3f(1.0, 0.0, 1.0);
		else if (n == gStart)
			glColor3f(0, 0, 1);
		else if (n == gGoal)
			glColor3f(0, 1, 0);
		else if (n->GetLabelL(kOptimalFlag) && n->GetLabelL(kInOpenList))
			glColor3f(1.0, 1.0, 0);
		else if (n->GetLabelL(kOptimalFlag))
			glColor3f(0.6, 0.6, 0);
		else if (n->GetLabelL(kInOpenList) == 0)
			glColor3f(0.5, 0.5, 0.5);
		else
			glColor3f(1, 1, 1);
		rv = absGraph->GetNodeLoc(GetRealNode(n));
		
		glVertex3f(rv.x, rv.y, rv.z);
	}

	glEnd();
	glLineWidth(1.0);
	
}

