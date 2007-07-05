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

bool GraphEnvironment::InvertAction(graphMove &a)
{
	uint16_t tmp = a.from;
	a.from = a.to;
	a.to = tmp;
	if (g->findDirectedEdge(a.from, a.to))
		return true;
	return false;
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

void GraphEnvironment::OpenGLDraw(int, graphState &s)
{
	node *n = g->GetNode(s);
	DrawSphere((GLdouble)n->GetLabelF(GraphSearchConstants::kXCoordinate),
						 (GLdouble)n->GetLabelF(GraphSearchConstants::kYCoordinate),
						 (GLdouble)n->GetLabelF(GraphSearchConstants::kZCoordinate),
						 (GLdouble)2.0/(g->getNumNodes()*g->getNumNodes()));
}

void GraphEnvironment::OpenGLDraw(int, graphState &, graphMove &)
{
	// if we want to draw a set of moves we use this to do so
}


int GraphEnvironment::NumNodesWithinRadius(graphState from, int depth) {
	// using BFS
	int count = 1;
	std::queue<SimpleNode> myqueue;
	__gnu_cxx::hash_map<uint64_t, SimpleNode> closedlist;

	std::vector<graphState> neighbors;

	SimpleNode n0(from, from, 0);
	myqueue.push(n0);

	while(! myqueue.empty()) 
	{
		SimpleNode frontN = myqueue.front();
		uint64_t frontID = GetStateHash(frontN.me);
		myqueue.pop();
	
		if(frontN.depth >= depth)
			continue;

		GetSuccessors(frontN.me, neighbors);

		for(unsigned int x = 0; x<neighbors.size(); x++)
		{
			graphState neighbor = neighbors[x];
			uint64_t neighborID = GetStateHash(neighbor);
			if(closedlist.find(neighborID) == closedlist.end())
			{
				count++;

				SimpleNode newNode(neighborID,frontID, frontN.depth+1);
				myqueue.push(newNode);
			}
		}

		closedlist[frontID] = frontN;
	}

	return count;
}

void GraphEnvironment::PathCountWithinRadius(graphState from, int depth, __gnu_cxx::hash_map<uint64_t, int> &counts, __gnu_cxx::hash_map<uint64_t, double> &aveCosts )
{
	// using recursive version of DFS
	std::vector<SimpleNode> thePath;

	SimpleNode n0(from,from,0);
	counts[GetStateHash(from)]++;
	thePath.push_back(n0);

	DFS_VISIT(thePath,depth,counts,aveCosts,0);

	for(__gnu_cxx::hash_map<uint64_t,int> ::iterator it = counts.begin(); it != counts.end(); it++)
	{
		if(it->second > 0)
			aveCosts[it->first] /= it->second;
	}
}

void GraphEnvironment::DFS_VISIT(std::vector<SimpleNode> &thePath, int depth, __gnu_cxx::hash_map<uint64_t, int> &counts, __gnu_cxx::hash_map<uint64_t, double> &aveCosts, double gval)
{
	std::vector<graphState> neighbors;

	SimpleNode current = thePath.back();
	if(current.depth >= depth)
		return;

	GetSuccessors(current.me, neighbors);

	for(unsigned int x = 0; x<neighbors.size(); x++)
	{
		graphState neighbor = neighbors[x];
		if(neighbor == current.parent)
			continue;

		bool flag = false;
		std::vector<SimpleNode>::iterator iter;
		for(iter = thePath.begin(); iter != thePath.end(); iter++) 
		{
			if(neighbor == iter->me) 
			{
				flag = true; // this neighbor is in the path
				break;
			}
		}

		// this check is important! 
		if(flag)
			continue;

		uint64_t uniqueID = GetStateHash(neighbor);

		counts[uniqueID]++;
		aveCosts[uniqueID] += gval + GCost(current.me,neighbor);

		SimpleNode sn(neighbor, current.me, current.depth+1);
		thePath.push_back(sn);
		DFS_VISIT(thePath, depth, counts, aveCosts, aveCosts[uniqueID]);  // recursion
		thePath.pop_back();
	}
}

