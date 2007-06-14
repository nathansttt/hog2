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
	if ((g == 0) || (g->getNumNodes() == 0)) return;

	glBegin(GL_LINES);
	glNormal3f(0, 1, 0);
		
	edge_iterator ei = g->getEdgeIter();
	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		//int x, y;
		//double offsetx, offsety;
		node *n;
		n = g->GetNode(e->getFrom());
		
		glColor3f(1, 0, 0);
		if (e->getMarked())
			glColor3f(1, 1, 1);
		
		GLdouble x, y, z;
		x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
		y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
		z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
		glVertex3f(x, y, z);
		
		n = g->GetNode(e->getTo());
		x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
		y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
		z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
		
		glVertex3f(x, y, z);
	}
	glEnd();
}

void GraphEnvironment::OpenGLDraw(int window, graphState &s)
{
	node *n = g->GetNode(s);
	DrawSphere((GLdouble)n->GetLabelF(GraphSearchConstants::kXCoordinate),
						 (GLdouble)n->GetLabelF(GraphSearchConstants::kYCoordinate),
						 (GLdouble)n->GetLabelF(GraphSearchConstants::kZCoordinate),
						 (GLdouble)2.0/(g->getNumNodes()*g->getNumNodes()));
}

