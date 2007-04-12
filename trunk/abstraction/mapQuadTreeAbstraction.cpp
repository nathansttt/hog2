/*
 * $Id: mapQuadTreeAbstraction.cpp,v 1.8 2006/10/18 23:53:25 nathanst Exp $
 *
 *  mapQuadTreeAbstraction.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 8/8/05.
 *  Copyright 2005 Nathan Sturtevant. All rights reserved.
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

#include "mapQuadTreeAbstraction.h"
#include "graph.h"

mapQuadTreeAbstraction::mapQuadTreeAbstraction(Map *_m, int _sectorSize)
:mapAbstraction(_m), sectorSize(_sectorSize)
{
	assert(_sectorSize>1);
	buildAbstraction();
}

mapQuadTreeAbstraction::~mapQuadTreeAbstraction()
{
}

bool mapQuadTreeAbstraction::pathable(node *from, node *to)
{
  while (from != to)
	{
    if ((!from) || (!to) ||
				(abstractions[from->getLabelL(kAbstractionLevel)]->getNumEdges() == 0))
      return false;
		
    from = abstractions[from->getLabelL(kAbstractionLevel)+1]->
      getNode(from->getLabelL(kParent));
    to = abstractions[to->getLabelL(kAbstractionLevel)+1]->
      getNode(to->getLabelL(kParent));
  }
	if ((from == 0) || (to == 0))
		return false;
	return true;	
}

// utility functions
/** verify that the hierarchy is consistent */
void mapQuadTreeAbstraction::verifyHierarchy()
{
}

// hierarchical modifications
/** remove node from abstraction */
void mapQuadTreeAbstraction::removeNode(node *)
{
	assert(false);
}

/** remove edge from abstraction */
void mapQuadTreeAbstraction::removeEdge(edge *, unsigned int)
{
	assert(false);
}

/** add node to abstraction */
void mapQuadTreeAbstraction::addNode(node *)
{
	// add it to the 
	assert(false);
}

/** add edge to abstraction */
void mapQuadTreeAbstraction::addEdge(edge *, unsigned int)
{
	// if each end is already connected who cares -- doesn't mess anything up -- just add it throughout
	assert(false);
}

/** This must be called after any of the above add/remove operations. But the
operations can be stacked followed by a single repairAbstraction call. */
void mapQuadTreeAbstraction::repairAbstraction()
{
}

void mapQuadTreeAbstraction::buildAbstraction()
{
	//inefficient for the moment
	abstractions.push_back(getMapGraph(getMap()));
	while (abstractions.back()->getNumEdges() > 0)
	{
		graph *g = new graph();
		addNodes(g);
		addEdges(g);
		abstractions.push_back(g);
	}
}

void mapQuadTreeAbstraction::addNodes(graph *g)
{
	node_iterator ni = abstractions.back()->getNodeIter();
	for (node *next = abstractions.back()->nodeIterNext(ni); next;
			 next = abstractions.back()->nodeIterNext(ni))
	{
		// if it isn't abstracted, do a bfs according to the quadrant and abstract these nodes together
		if (next->getLabelL(kParent) == -1)
		{
			node *parent;
			g->addNode(parent = new node("??"));
			parent->setLabelL(kAbstractionLevel, next->getLabelL(kAbstractionLevel)+1); // level in abstraction tree
			parent->setLabelL(kNumAbstractedNodes, 0); // number of abstracted nodes
			parent->setLabelL(kParent, -1); // parent of this node in abstraction hierarchy
			parent->setLabelF(kXCoordinate, kUnknownPosition);
			parent->setLabelL(kNodeBlocked, 0);
			abstractionBFS(next, parent, getQuadrant(next));
		}
	}
}

void mapQuadTreeAbstraction::addEdges(graph *aGraph)
{
	graph *g = abstractions.back();
	edge_iterator ei = g->getEdgeIter();
	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		
		int from = g->getNode(e->getFrom())->getLabelL(kParent);
		int to = g->getNode(e->getTo())->getLabelL(kParent);
		edge *f=0;
		
		if ((from != to) && (!(f = aGraph->findEdge(to, from))))
		{
			double weight = h(aGraph->getNode(from), aGraph->getNode(to));
			f = new edge(from, to, weight);
			f->setLabelL(kEdgeCapacity, 1);
			aGraph->addEdge(f);
		}
		else if (f) f->setLabelL(kEdgeCapacity, f->getLabelL(kEdgeCapacity)+1);
	}	
}

void mapQuadTreeAbstraction::abstractionBFS(node *which, node *parent, int quadrant) // depth in edges...should we try literal distance?
{
	if ((which == 0) || (which->getLabelL(kParent) != -1) || (getQuadrant(which) != quadrant))
		return;
	buildNodeIntoParent(which, parent);

	neighbor_iterator ni = which->getNeighborIter();
	for (long tmp = which->nodeNeighborNext(ni); tmp != -1; tmp = which->nodeNeighborNext(ni))
	{
		abstractionBFS(abstractions.back()->getNode(tmp), parent, quadrant);
	}
}

void mapQuadTreeAbstraction::buildNodeIntoParent(node *n, node *parent)
{
	assert(getAbstractionLevel(n)+1 == getAbstractionLevel(parent));
	n->setLabelL(kParent, parent->getNum());
	parent->setLabelL(kFirstData+parent->getLabelL(kNumAbstractedNodes), n->getNum());
	parent->setLabelL(kNumAbstractedNodes, parent->getLabelL(kNumAbstractedNodes)+1);
	parent->setLabelF(kXCoordinate, kUnknownPosition);
}

int mapQuadTreeAbstraction::getQuadrant(node *which)
{ // int sectorSize;
	// 1. get any child
	node *child = which;
	while (child->getLabelL(kAbstractionLevel) != 0)
		child = getNthChild(child, 0);
	
	int xloc = child->getLabelL(kFirstData); // x loc in map
	int yloc = child->getLabelL(kFirstData+1); // y loc in map

	int level = which->getLabelL(kAbstractionLevel);
	int absSectorSize = (int)pow((double)sectorSize, (double)level+1);
	
	int sectorNum = (getMap()->getMapWidth()/absSectorSize)*(yloc/absSectorSize)+(xloc/absSectorSize);
	
	return sectorNum;
}

//
// 0  1   2   3    ...
// w  w+1 w+2 w+3  ...

// rowSector = (lowerSector%width)/sectorSize
// colSector = (lowerSector/width)/sectorSize

// or

// size at level = sectorSize^level
// low-level-size-x%sizeatlevel
// low-level-size-y%sizeatlevel
