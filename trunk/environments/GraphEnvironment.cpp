/*
 *  GraphEnvironment.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/29/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "GraphEnvironment.h"

GraphEnvironment::GraphEnvironment(Graph *_g)
:g(_g)
{
}

GraphEnvironment::~GraphEnvironment()
{
	// delete g; ??
}

void GraphEnvironment::GetSuccessors(graphState &stateID, std::vector<graphState> &neighbors)
{
	neighbors.resize(0);
	node *n = g->GetNode(stateID);
	edge_iterator ei = n->getOutgoingEdgeIter();
	for (edge *e = n->edgeIterNextOutgoing(ei); e; e = n->edgeIterNextOutgoing(ei))
	{
		neighbors.push_back(e->getTo());
	}
}

void GraphEnvironment::GetActions(graphState &stateID, std::vector<graphMove> &actions)
{
	actions.resize(0);
	node *n = g->GetNode(stateID);
	edge_iterator ei = n->getOutgoingEdgeIter();
	for (edge *e = n->edgeIterNextOutgoing(ei); e; e = n->edgeIterNextOutgoing(ei))
	{
		actions.push_back(graphMove(e->getFrom(), e->getTo()));
	}
}

graphMove GraphEnvironment::GetAction(graphState &s1, graphState &s2)
{
	return graphMove(s1, s2);
}

void GraphEnvironment::ApplyAction(graphState &s, graphMove a)
{
	assert(s == a.from);
	s = a.to;
}

double GraphEnvironment::HCost(graphState &state1, graphState &state2)
{
	if (fequal(g->GetNode(state1)->GetLabelF(GraphSearchConstants::kHCost), 0))
		return g->GetNode(state2)->GetLabelF(GraphSearchConstants::kHCost);
	return g->GetNode(state1)->GetLabelF(GraphSearchConstants::kHCost);
}

double GraphEnvironment::GCost(graphState &state1, graphState &state2)
{
	edge *e = g->FindEdge(state1, state2);
	assert(e);
	return e->getWeight();
}

bool GraphEnvironment::GoalTest(graphState &state, graphState &goal)
{
	return state == goal;
}

uint64_t GraphEnvironment::GetStateHash(graphState &state)
{
	return g->GetNode(state)->getUniqueID();
}

uint64_t GraphEnvironment::GetActionHash(graphMove act)
{
	return (g->GetNode(act.from)->getUniqueID()<<16)|
	(g->GetNode(act.to)->getUniqueID());
}

void GraphEnvironment::OpenGLDraw(int window)
{
}

void GraphEnvironment::OpenGLDraw(int window, graphState &s)
{
	node *n = g->GetNode(s);
	DrawSphere(n->GetLabelF(GraphSearchConstants::kXCoordinate),
						 n->GetLabelF(GraphSearchConstants::kYCoordinate),
						 n->GetLabelF(GraphSearchConstants::kZCoordinate),
						 2.0/(g->getNumNodes()*g->getNumNodes()));
}

