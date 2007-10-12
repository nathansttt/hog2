/*
 *  NodeLimitAbstraction.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 3/24/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
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


#include "NodeLimitAbstraction.h"
#include "Graph.h"

using namespace GraphAbstractionConstants;

NodeLimitAbstraction::NodeLimitAbstraction(Map *_m, int _NodeLimit)
:MapAbstraction(_m), nodeLimit(_NodeLimit)
{
  assert(_m!=NULL);
	
  buildAbstraction();
}

NodeLimitAbstraction::~NodeLimitAbstraction()
{ 
}

bool NodeLimitAbstraction::Pathable(node *from, node *to)
{
  while (from != to)
	{
    if ((!from) || (!to) ||
				(abstractions[from->GetLabelL(kAbstractionLevel)]->GetNumEdges() == 0))
      return false;
		
    from = abstractions[from->GetLabelL(kAbstractionLevel)+1]->
      GetNode(from->GetLabelL(kParent));
    to = abstractions[to->GetLabelL(kAbstractionLevel)+1]->
      GetNode(to->GetLabelL(kParent));
  }
	if ((from == 0) || (to == 0))
		return false;
	return true;	
}

// utility functions
/** verify that the hierarchy is consistent */
void NodeLimitAbstraction::VerifyHierarchy()
{
}

// hierarchical modifications
/** remove node from abstraction */
void NodeLimitAbstraction::RemoveNode(node *)
{
}

/** remove edge from abstraction */
void NodeLimitAbstraction::RemoveEdge(edge *, unsigned int)
{
}

/** add node to abstraction */
void NodeLimitAbstraction::AddNode(node *)
{
	// add it to the 
	assert(false);
}

/** add edge to abstraction */
void NodeLimitAbstraction::AddEdge(edge *, unsigned int)
{
	assert(false);
	// if each end is already connected who cares -- doesn't mess anything up -- just add it throughout
}

/** This must be called after any of the above add/remove operations. But the
operations can be stacked followed by a single RepairAbstraction call. */
void NodeLimitAbstraction::RepairAbstraction()
{
}

void NodeLimitAbstraction::buildAbstraction()
{
	//inefficient for the moment
	abstractions.push_back(GetMapGraph(GetMap()));
	while (abstractions.back()->GetNumEdges() > 0)
	{
		Graph *g = new Graph();
		addNodes(g);
		addEdges(g);
		abstractions.push_back(g);
	}
}

void NodeLimitAbstraction::addNodes(Graph *g)
{
	int count = abstractions.back()->GetNumNodes();
	while (count > 0)
	{
		// select a random node
		node *next = abstractions.back()->GetRandomNode();
		assert(next!=NULL);
		// if it isn't abstracted, do a bfs according to the depth and abstract these nodes together
		if (next->GetLabelL(kParent) == -1)
		{
			node *parent;
			g->AddNode(parent = new node("??"));
			assert(parent!=NULL);
			parent->SetLabelL(kAbstractionLevel, next->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
			parent->SetLabelL(kNumAbstractedNodes, 0); // number of abstracted nodes
			parent->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
			parent->SetLabelF(kXCoordinate, kUnknownPosition);
			parent->SetLabelL(kNodeBlocked, 0);
			abstractionBFS(next, parent, nodeLimit);
			count-=parent->GetLabelL(kNumAbstractedNodes);
		}
	}
}

void NodeLimitAbstraction::addEdges(Graph *aGraph)
{
	Graph *g = abstractions.back();
	edge_iterator ei = g->getEdgeIter();
	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		int from = g->GetNode(e->getFrom())->GetLabelL(kParent);
		int to = g->GetNode(e->getTo())->GetLabelL(kParent);
		edge *f=0;
		
		if ((from != to) && (!(f = aGraph->FindEdge(to, from))))
		{
			double weight = h(aGraph->GetNode(from), aGraph->GetNode(to));
			f = new edge(from, to, weight);
			f->SetLabelL(kEdgeCapacity, 1);
			aGraph->AddEdge(f);
		}
		else if (f) f->SetLabelL(kEdgeCapacity, f->GetLabelL(kEdgeCapacity)+1);
	}	
}

void NodeLimitAbstraction::abstractionBFS(node *which, node *parent, int count)
{
	std::vector<node *> q;
	Graph *g = GetAbstractGraph(which);
	
	buildNodeIntoParent(which, parent);
	count--;
	q.push_back(which);
	which->key = 0;
	
	for (unsigned int x = 0; x < q.size(); x++)
	{
		if (count <= 0)
			break;

		neighbor_iterator ni = q[x]->getNeighborIter();
		for (int next = q[x]->nodeNeighborNext(ni); next != -1; next = q[x]->nodeNeighborNext(ni))
		{
			if (count <= 0)
				break;

			node *neighbor = g->GetNode(next);
			// already in our q and handled
			if ((neighbor->key < q.size()) && (q[neighbor->key] == neighbor))
			{
				continue;
			}
			// doesn't yet have a parent
			if (neighbor->GetLabelL(kParent) == -1)
			{
				buildNodeIntoParent(neighbor, parent);
				count--;
				neighbor->key = q.size();
				q.push_back(neighbor);
			}
		}
	}
	
//	if ((which == 0) || (which->GetLabelL(kParent) != -1))
//		return;
//	buildNodeIntoParent(which, parent);
//	count--;
//	if (count < 0)
//		return;
//	neighbor_iterator ni = which->getNeighborIter();
//	for (long tmp = which->nodeNeighborNext(ni); tmp != -1; tmp = which->nodeNeighborNext(ni))
//	{
//		abstractionBFS(abstractions.back()->GetNode(tmp), parent, depth-1);
//	}
}

void NodeLimitAbstraction::buildNodeIntoParent(node *n, node *parent)
{
	assert(GetAbstractionLevel(n)+1 == GetAbstractionLevel(parent));
	n->SetLabelL(kParent, parent->GetNum());
	parent->SetLabelL(kFirstData+parent->GetLabelL(kNumAbstractedNodes), n->GetNum());
	parent->SetLabelL(kNumAbstractedNodes, parent->GetLabelL(kNumAbstractedNodes)+1);
	parent->SetLabelF(kXCoordinate, kUnknownPosition);
}

