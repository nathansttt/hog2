/*
 * $Id: corridorAStar.cpp,v 1.11 2006/10/18 23:52:25 nathanst Exp $
 *
 *  corridorAStar.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 6/22/05.
 *  Copyright 2005 Nathan Sturtevant, University of Alberta. All rights reserved.
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

#include "CorridorAStar.h"
#include "FPUtil.h"

const static int verbose = 0;

corridorAStar::corridorAStar()
{
	corridor = &emptyCorridor;
}

void corridorAStar::setCorridor(const std::vector<node *> *c)
{
	corridor = c;
}

path *corridorAStar::getPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp)
{
	return getBestPath(aMap, from, to, to, rp);
}

path *corridorAStar::getBestPath(GraphAbstraction *aMap, node *from, node *to, node *hGoal, reservationProvider *rp)
{
	nodesExpanded = 0;
	nodesTouched = 0;

  node *n=0;
  heap *nodeHeap = new heap(corridor->size());
  std::vector<node *> expandedNodes(100);
	neighbor_iterator ni;
	bool exactGoal = true;
	
	if (to->getLabelL(kAbstractionLevel) != from->getLabelL(kAbstractionLevel))
		exactGoal = false;
	
	// get current layer & abstract corridor layer
  int absLayer = from->getLabelL(kAbstractionLevel);
	int corridorLayer = absLayer;
	if (corridor->size() > 0)
		corridorLayer = (*corridor)[0]->getLabelL(kAbstractionLevel);
	graph *absGraph = aMap->GetAbstractGraph(absLayer);
	
  // mark location of eligible nodes
  for (unsigned int x = 0; x < corridor->size(); x++)
    ((*corridor)[x])->key = x;
	
  // label start node cost 0
  n = from;
  n->setLabelF(kTemporaryLabel, aMap->h(n, hGoal));
  n->markEdge(0);
  if (verbose) printf("Starting on %d with cost %1.2f\n", from->getNum(), n->getLabelF(kTemporaryLabel));
	nodesExpanded++;
  while (1) {
    expandedNodes.push_back(n);
    n->key = expandedNodes.size()-1;
		
    if (verbose)
      printf("really working on %d with cost %1.2f\n", n->getNum(), n->getLabelF(kTemporaryLabel));
		
		ni = n->getNeighborIter();
		
		for (node *currNode = absGraph->getNode(n->nodeNeighborNext(ni));
				 currNode; currNode = absGraph->getNode(n->nodeNeighborNext(ni)))
		{
			nodesTouched++;
      if (verbose) printf("considering neighbor %d\n", currNode->getNum());
      if (nodeHeap->isIn(currNode))
			{
				nodesExpanded++;
				relaxEdge(nodeHeap, absGraph, aMap,
									absGraph->findEdge(n->getNum(), currNode->getNum()),
									n, currNode, hGoal);
      }
      else if (rp && (absLayer==0) && (currNode != to) && rp->nodeOccupied(currNode))
			{
				if (verbose) printf("Can't path to %d, %d (occupied)\n", (unsigned int)currNode->getLabelL(kFirstData), (unsigned int)currNode->getLabelL(kFirstData+1));
				expandedNodes.push_back(currNode);
				currNode->key = expandedNodes.size()-1;
				// ignore this tile if occupied.
      }
      // this node is unexpanded if (1) it isn't on the expanded list
      else if ((currNode->key >= expandedNodes.size()) ||
							 (expandedNodes[currNode->key] != currNode))
			{
				node *par = aMap->getNthParent(currNode, corridorLayer);
				unsigned int parentKey = par->key;
				
				// this node is unexpanded if (2) it's parent is in the eligible parent list
				// (3) or having no eligible parents means we can search anywhere!
				if ((corridor->size() == 0) ||
						((parentKey < corridor->size()) && ((*corridor)[parentKey] == par)))
				{
					currNode->setLabelF(kTemporaryLabel, MAXINT);
					currNode->setKeyLabel(kTemporaryLabel);
					currNode->markEdge(0);
					nodeHeap->add(currNode);
					if (verbose) printf("Adding neighbor %d\n", currNode->getNum());
					nodesExpanded++;
					relaxEdge(nodeHeap, absGraph, aMap,
										absGraph->findEdge(n->getNum(), currNode->getNum()),
										n, currNode, hGoal);
				}
				else { if (verbose) printf("%d not eligible\n", currNode->getNum()); }
      }
      else { if (verbose) printf("%d already expanded\n", currNode->getNum()); }
    }
		
    n = (node*)nodeHeap->remove();
    if (n == 0)
		{
			if (verbose) printf("Error: We expanded every possible node!\n");
			break;
		}

		// found a node who's nth parent is inside our target
		if ((!exactGoal) &&
				(aMap->getNthParent(n, to->getLabelL(kAbstractionLevel)) == to))
			break;
		
    if (n == to) { /*printf("Found goal %d\n", dest);*/ break; }
		
    if (verbose) printf("working on %d with cost %1.2f\n", n->getNum(), n->getLabelF(kTemporaryLabel));
  }
	
	if ((n != to) && (exactGoal))
	{
		if (verbose)
			printf("Corridor A* ran and didn't find goal in corridor!\n");
	}
  delete nodeHeap;
 	corridor = &emptyCorridor;
	if (n != 0) // we found a goal
		return extractBestPath(absGraph, n->getNum());
	return 0;
}

path *corridorAStar::getBestPath(GraphAbstraction *aMap, node *afrom, node *ato, node *from, node *, reservationProvider *rp)
{
	nodesExpanded = 0;
	nodesTouched = 0;
	
  node *n=0;
  heap *nodeHeap = new heap(corridor->size());
  std::vector<node *> expandedNodes(100);
	neighbor_iterator ni;
	bool exactGoal = true;
	
	if (ato->getLabelL(kAbstractionLevel) != afrom->getLabelL(kAbstractionLevel))
		exactGoal = false;
	
	// get current layer & abstract corridor layer
  int absLayer = afrom->getLabelL(kAbstractionLevel);
	int corridorLayer = absLayer;
	if (corridor->size() > 0)
		corridorLayer = (*corridor)[0]->getLabelL(kAbstractionLevel);
	graph *absGraph = aMap->GetAbstractGraph(absLayer);
	
  // mark location of eligible nodes
  for (unsigned int x = 0; x < corridor->size(); x++)
    ((*corridor)[x])->key = x;
	
  // label start node cost 0
  n = afrom;
  n->setLabelF(kTemporaryLabel, aMap->h(n, ato));
  n->markEdge(0);
  if (verbose) printf("Starting on %d with cost %1.2f\n", afrom->getNum(), n->getLabelF(kTemporaryLabel));
	nodesExpanded++;
  while (1)
	{
    expandedNodes.push_back(n);
    n->key = expandedNodes.size()-1;
		
    if (verbose)
      printf("really working on %d with cost %1.2f\n", n->getNum(), n->getLabelF(kTemporaryLabel));
		
		ni = n->getNeighborIter();
		
		for (node *currNode = absGraph->getNode(n->nodeNeighborNext(ni));
				 currNode; currNode = absGraph->getNode(n->nodeNeighborNext(ni)))
		{
			nodesTouched++;
      if (verbose) printf("considering neighbor %d\n", currNode->getNum());
      if (nodeHeap->isIn(currNode))
			{
				nodesExpanded++;
				if (n == afrom)
					relaxFirstEdge(nodeHeap, absGraph, aMap,
										absGraph->findEdge(n->getNum(), currNode->getNum()),
										from, n, currNode, ato);
				if (currNode == ato)
					relaxFinalEdge(nodeHeap, absGraph, aMap,
												 absGraph->findEdge(n->getNum(), currNode->getNum()),
												 n, currNode, ato);
				else				
					relaxEdge(nodeHeap, absGraph, aMap,
									absGraph->findEdge(n->getNum(), currNode->getNum()),
									n, currNode, ato);
      }
      else if (rp && (absLayer==0) && (currNode != ato) && rp->nodeOccupied(currNode))
			{
				if (verbose) printf("Can't path to %d, %d (occupied)\n", (unsigned int)currNode->getLabelL(kFirstData), (unsigned int)currNode->getLabelL(kFirstData+1));
				expandedNodes.push_back(currNode);
				currNode->key = expandedNodes.size()-1;
				// ignore this tile if occupied.
      }
      // this node is unexpanded if (1) it isn't on the expanded list
      else if ((currNode->key >= expandedNodes.size()) ||
							 (expandedNodes[currNode->key] != currNode))
			{
				node *par = aMap->getNthParent(currNode, corridorLayer);
				unsigned int parentKey = par->key;
				
				// this node is unexpanded if (2) it's parent is in the eligible parent list
				// (3) or having no eligible parents means we can search anywhere!
				if ((corridor->size() == 0) ||
						((parentKey < corridor->size()) && ((*corridor)[parentKey] == par)))
				{
					currNode->setLabelF(kTemporaryLabel, MAXINT);
					currNode->setKeyLabel(kTemporaryLabel);
					currNode->markEdge(0);
					nodeHeap->add(currNode);
					if (verbose) printf("Adding neighbor %d\n", currNode->getNum());
					nodesExpanded++;

					if (n == afrom)
						relaxFirstEdge(nodeHeap, absGraph, aMap,
													 absGraph->findEdge(n->getNum(), currNode->getNum()),
													 from, n, currNode, ato);
					if (currNode == ato)
						relaxFinalEdge(nodeHeap, absGraph, aMap,
													 absGraph->findEdge(n->getNum(), currNode->getNum()),
													 n, currNode, ato);
					else				
						relaxEdge(nodeHeap, absGraph, aMap,
											absGraph->findEdge(n->getNum(), currNode->getNum()),
											n, currNode, ato);
//					relaxEdge(nodeHeap, absGraph, aMap,
//										absGraph->findEdge(n->getNum(), currNode->getNum()),
//										n, currNode, hGoal);
				}
				else { if (verbose) printf("%d not eligible\n", currNode->getNum()); }
      }
      else { if (verbose) printf("%d already expanded\n", currNode->getNum()); }
    }
		
    n = (node*)nodeHeap->remove();
    if (n == 0)
		{
			if (verbose) printf("Error: We expanded every possible node!\n");
			break;
		}
		
		// found a node who's nth parent is inside our target
		if ((!exactGoal) &&
				(aMap->getNthParent(n, ato->getLabelL(kAbstractionLevel)) == ato))
			break;
		
    if (n == ato) { /*printf("Found goal %d\n", dest);*/ break; }
		
    if (verbose) printf("working on %d with cost %1.2f\n", n->getNum(), n->getLabelF(kTemporaryLabel));
  }
	
	if ((n != ato) && (exactGoal))
	{
		if (verbose)
			printf("Corridor A* ran and didn't find goal in corridor!\n");
	}
  delete nodeHeap;
 	corridor = &emptyCorridor;
	if (n != 0) // we found a goal
		return extractBestPath(absGraph, n->getNum());
	return 0;
}

void corridorAStar::relaxEdge(heap *nodeHeap, graph *, GraphAbstraction *aMap,
															edge *e, node *from, node *to, node *dest)
{
  double weight;
  weight = from->getLabelF(kTemporaryLabel)-aMap->h(from, dest)+aMap->h(to, dest)+e->getWeight();
  if (fless(weight, to->getLabelF(kTemporaryLabel)))
	{
    if (verbose)
      printf("Updating %d to %1.2f from %1.2f\n", to->getNum(), weight, to->getLabelF(kTemporaryLabel));
		//weight -= 0.001*(weight-map->h(to, d)); // always lower g-cost slightly so that we tie break in favor of higher g cost
    to->setLabelF(kTemporaryLabel, weight);
    nodeHeap->decreaseKey(to);
    // this is the edge used to get to this node in the min. path tree
    to->markEdge(e);
  }
}

void corridorAStar::relaxFirstEdge(heap *nodeHeap, graph *, GraphAbstraction *aMap,
																	 edge *e, node *from, node *afrom, node *ato, node *dest)
{
  double weight;
  weight = afrom->getLabelF(kTemporaryLabel)-aMap->h(afrom, dest)+aMap->h(ato, dest)+aMap->h(from, ato);
  if (fless(weight, ato->getLabelF(kTemporaryLabel)))
	{
    if (verbose)
      printf("Updating %d to %1.2f from %1.2f\n", ato->getNum(), weight, ato->getLabelF(kTemporaryLabel));
		//weight -= 0.001*(weight-map->h(ato, d)); // always lower g-cost slightly so that we tie break in favor of higher g cost
    ato->setLabelF(kTemporaryLabel, weight);
    nodeHeap->decreaseKey(ato);
    // this is the edge used to get to this node in the min. path tree
    ato->markEdge(e);
  }
}

void corridorAStar::relaxFinalEdge(heap *nodeHeap, graph *, GraphAbstraction *aMap,
																	 edge *e, node *from, node *to, node *realDest)
{
  double weight;
  weight = from->getLabelF(kTemporaryLabel)-aMap->h(from, to)+aMap->h(from, realDest);
  if (fless(weight, to->getLabelF(kTemporaryLabel)))
	{
    if (verbose)
      printf("Updating %d to %1.2f from %1.2f\n", to->getNum(), weight, to->getLabelF(kTemporaryLabel));
		//weight -= 0.001*(weight-map->h(to, d)); // always lower g-cost slightly so that we tie break in favor of higher g cost
    to->setLabelF(kTemporaryLabel, weight);
    nodeHeap->decreaseKey(to);
    // this is the edge used to get to this node in the min. path tree
    to->markEdge(e);
  }
}

path *corridorAStar::extractBestPath(graph *g, unsigned int current)
{
  path *p = 0;
  edge *e;
  // extract best path from graph -- each node has a single parent in the graph which is the marked edge
  // for visuallization purposes, an edge can be marked meaning it will be drawn in white
  while ((e = g->getNode(current)->getMarkedEdge()))
	{
    if (verbose) printf("%d <- ", current);
		
    p = new path(g->getNode(current), p);
		
    e->setMarked(true);
		
    if (e->getFrom() == current)
      current = e->getTo();
    else
      current = e->getFrom();
  }
  p = new path(g->getNode(current), p);
  if (verbose) printf("%d\n", current);
  return p;	
}
