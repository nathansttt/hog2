/*
 * $Id: praStar.cpp,v 1.18 2007/03/25 23:01:14 nathanst Exp $
 *
 *  Hierarchical Open Graph File
 *
 *  Created by Nathan Sturtevant on 9/28/04.
 *  Copyright 2004 Nathan Sturtevant. All rights reserved.
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

#include "FPUtil.h"
#include "PRAStar.h"

const bool verbose = false;

praStar::praStar()
:SearchAlgorithm()
{
	partialLimit = -1;
	fixedPlanLevel = -1;
	sprintf(algName,"PRA*(%d)", partialLimit);
	expandSearchRadius = true; planFromMiddle = true;
	cache = 0;
}

void praStar::setCache(path **p)
{
	cache = p;
}

path *praStar::getPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *_rp)
{
	lengths.resize(0);
	rp = _rp;
	// Reset the number of states expanded
	nodesExpanded = 0;
	nodesTouched = 0;
	
	if ((from == 0) || (to == 0) || (!aMap->Pathable(from, to)) || (from == to))
	{
		if (verbose)
		{
			if (!from)
				printf("praStar: from == 0\n");
			if (!to)
				printf("praStar: to == 0\n");
			if (from == to)
				printf("praStar: from == to\n");
			if (from && to && (!aMap->Pathable(from, to)))
				printf("praStar: no path from %p to %p\n", (void*)from, (void*)to);
			//cout << "praStar: no path from " << *from << " to " << *to << endl;
		}
		return 0;
	}
	
	std::vector<node *> fromChain;
	std::vector<node *> toChain;
	
	map = aMap;
	
	if (verbose)
		printf("At nodes #%d and %d\n", from->getNum(), to->getNum());
	if (aMap->GetAbstractGraph(0)->findEdge(from->getNum(), to->getNum())) { // we are 1 step away
		if ((cache) && (*cache))
		{
			delete *cache;
			*cache = 0;
		}
		//printf("We're just 1 step away! (or is it 2?)\n");
		return new path(from, new path(to));
	}
	
	aMap->GetNumAbstractGraphs(from, to, fromChain, toChain);
	//	assert(aMap->GetAbstractGraph(fromChain.back()->getLabelL(kAbstractionLevel))->
	//					findEdge(fromChain.back()->getNum(), toChain.back()->getNum()));
	
	path *lastPath = 0;
	if (cache && ((*cache) != 0))
	{
		if (verbose) printf("Checking cache\n");
		// check to see if cache is consistent with the planning we're going
		// to do by finding the first node in the abstraction hierarchy
		if (fromChain.back()->getLabelL(kAbstractionLevel) < (*cache)->n->getLabelL(kAbstractionLevel))
		{
			delete (*cache);
			*cache = 0;
		}
		else {
			while ((fromChain.back() != (*cache)->n) && (fromChain.size() > 0))
			{
				toChain.pop_back();
				fromChain.pop_back();
			}
			if (fromChain.size() == 0)
			{ printf("Cache invalid; not in first node.\n"); std::cout << *(*cache)->n; exit(10); }
			// if we find it, we set it to be our lastPath
			if (verbose) printf("Setting last path to cache\n");
			lastPath = *cache;
			toChain.pop_back();
			fromChain.pop_back();
		}
		
	}
	if (fixedPlanLevel != -1)
	{
		//while (((int)fromChain.size() > 2) && ((int)toChain.size() > fixedPlanLevel + 1))
		while (((int)fromChain.size() > 1) && ((int)toChain.size() > fixedPlanLevel + 1))
		{
			toChain.pop_back();
			fromChain.pop_back();
		}
		
		//std::cout << "top praStar level: " << (toChain.size() - 1) << std::endl;
	}
	else if ((planFromMiddle) && (lastPath == 0))
	{
		unsigned int previousSize = fromChain.size();
		int minNode = (int)(2*sqrt(aMap->GetAbstractGraph(aMap->GetAbstractionLevel(fromChain[0]))->getNumNodes()));
		while ((fromChain.size() > 2) && ((fromChain.size() > (previousSize)/2) ||
																			(aMap->GetAbstractGraph(fromChain.size())->getNumNodes() < minNode)))
		{
//			printf("At size %d, %d nodes\n", fromChain.size(), aMap->GetAbstractGraph(fromChain.size())->getNumNodes());
			toChain.pop_back();
			fromChain.pop_back();
		}
//		printf("Previous size: %d, nodes: %d, limit: %d now: %d\n", previousSize,
//					 (aMap->GetAbstractGraph(aMap->GetAbstractionLevel(fromChain[0]))->getNumNodes()),
//					 minNode, toChain.size());
	}
	else if (lastPath == 0)
	{
		// there should be an edge directly from the last nodes here....
		//		assert(aMap->GetAbstractGraph(fromChain.back()->getLabelL(kAbstractionLevel))->
		//					 findEdge(fromChain.back()->getNum(), toChain.back()->getNum()));
		// it's possible this will be a path from the node back to itself
		// but if that's the case, we'll still be ok.
		lastPath = new path(fromChain.back(), new path(toChain.back()));
		//printf("%d <- %d (start)\n", toChain.back()->getNum(), fromChain.back()->getNum());
		toChain.pop_back();
		fromChain.pop_back();
	}

	lengths.resize(fromChain.size());
	do {
		unsigned int destParent = 0xFFFFFFFF;
		unsigned int dest;
		
		to = toChain.back();
		from = fromChain.back();
		dest = to->getNum();
		if (verbose)
			printf("Expanded %d nodes before doing level %d\n", nodesExpanded, (int)from->getLabelL(kAbstractionLevel));		
		toChain.pop_back();
		fromChain.pop_back();
		
		if (verbose)
			printf("Building path from %d to %d (%ld/%ld)\n",
						 from->getNum(), to->getNum(), from->getLabelL(kParent), to->getLabelL(kParent));
		
		std::vector<unsigned int> eligibleNodeParents;
		
		if (lastPath)
		{
			// cut path down to size of partial path limit
			if (partialLimit > 0)
			{
				path *trav = lastPath;
				//for (int x = 0; x < partialLimit*(from->getLabelL(kAbstractionLevel)+1); x++)
				for (int x = 0; x < partialLimit; x++)
					if (trav->next) 
						trav = trav->next;
				if ((trav->next != 0) || (trav->n->getNum() != (unsigned)to->getLabelL(kParent)))
				{
					destParent = trav->n->getNum();
					if (trav->next)
						dest = trav->next->n->getLabelL(kFirstData);
					else
						dest = trav->n->getLabelL(kFirstData);
					delete trav->next;
					trav->next = 0;
					if (verbose) printf("Setting target parent to %d\n", destParent);
				}
				// set actual target to any node in destParent
				//				if (trav->next)
				//					to = aMap->GetAbstractGraph(trav->next->n->getLabelL(kAbstractionLevel)
				//																			-1)->getNode(trav->n->getLabelL(kFirstData));
				// later try for best h value to actual target
				//for (unsigned int x = 1; x < trav->n->getLabelL(kNumAbstractedNodes); x++)
				
			}
			
			// find eligible nodes for lower level expansions
			for (path *trav = lastPath; trav; trav = trav->next)
			{
				if (expandSearchRadius)
				{
					edge_iterator ei = trav->n->getEdgeIter();
					for (edge *e = trav->n->edgeIterNext(ei); e; e = trav->n->edgeIterNext(ei))
					{
						if (e->getFrom() == trav->n->getNum())
							eligibleNodeParents.push_back(e->getTo());
						else
							eligibleNodeParents.push_back(e->getFrom());
					}
				}
				eligibleNodeParents.push_back(trav->n->getNum());
			}
		}
		if ((lastPath == 0) && (cache))
		{
			lastPath = getAbstractPath(map->GetAbstractGraph((unsigned int)from->
																											 getLabelL(kAbstractionLevel)),
																 from->getNum(), destParent, eligibleNodeParents,
																 kTemporaryLabel, dest);
			*cache = lastPath;
			lengths[fromChain.size()] = lastPath->length();
		}
		else {
			if ((cache && ((*cache) != lastPath)) || (!cache))
				delete lastPath;
			lastPath = getAbstractPath(map->GetAbstractGraph((unsigned int)from->
																											 getLabelL(kAbstractionLevel)),
																 from->getNum(), destParent, eligibleNodeParents,
																 kTemporaryLabel, dest);
			lengths[fromChain.size()] = lastPath->length();
		}
	} while (fromChain.size() > 0);
	cache = 0;
	return lastPath;
}

path *praStar::getAbstractPath(graph *g, unsigned int source, unsigned int destParent,
															 std::vector<unsigned int> &eligibleNodeParents, int LABEL,
															 unsigned int dest)
{
	path *p = 0;
	edge *e;
	// extract actual path out
	unsigned int current = astar(g, source, destParent, eligibleNodeParents, LABEL, dest);
	if (current == source) return 0;
	
	while ((e = g->getNode(current)->getMarkedEdge()))
	{
		if (verbose) printf("%d <- ", current);
		p = new path(g->getNode(current), p);
		
		e->setMarked(true);
		dest = current;
		
		if (e->getFrom() == current)
			current = e->getTo();
		else
			current = e->getFrom();
	}
	p = new path(g->getNode(current), p);
	if (verbose) printf("%d\n", current);
	return p;
}

// for each primitive action, apply it as long as we can, checking to see if
// we ever hit our path again. If so, we can use the repeated primitive operator
// as a replacement for our path

// this should move into algorithm.cpp
path *praStar::smoothPath(path *p)
{
	// 1. add path to vector so we can detect it
	// 2. go each direction and see if we hit the path
	// 3. replace segment of previous path
	return p;
}

unsigned int praStar::astar(graph *g, unsigned int source, unsigned int destParent,
														std::vector<unsigned int> &eligibleNodeParents, int LABEL,
														unsigned int dest)
{
	node *n=0;
	heap *nodeHeap = new heap(eligibleNodeParents.size());
	std::vector<node *> expandedNodes(100);
	edge_iterator ei;
	node *currBest = 0;
	bool expandedAnything = false;
	unsigned int openNode = source;
	
	int absLayer = g->getNode(source)->getLabelL(kAbstractionLevel);
	
	// mark location of eligible nodes
	for (unsigned int x = 0; x < eligibleNodeParents.size(); x++)
		map->GetAbstractGraph(absLayer+1)->getNode(eligibleNodeParents[x])->key = x;
	
	// label start node cost 0
	n = g->getNode(source);
	n->setLabelF(LABEL, map->h(n, g->getNode(dest)));
	n->markEdge(0);
	if (verbose) printf("Starting on %d with cost %1.4f\n", source, n->getLabelF(LABEL));
	while (1)
	{
		nodesExpanded++;
		expandedNodes.push_back(n);
		n->key = expandedNodes.size()-1;
		
		if (verbose)
			printf("really working on %d with cost %1.4f\n", n->getNum(), n->getLabelF(LABEL));
		
		ei = n->getEdgeIter();
		
		for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
		{
			nodesTouched++;
			unsigned int which;
			if ((which = e->getFrom()) == openNode) which = e->getTo();
			if (verbose) printf("considering neighbor %d\n", which);
			node *currNode = g->getNode(which);
			
			if (nodeHeap->isIn(currNode))
			{
				//nodesExpanded++;
				relaxEdge(nodeHeap, g, e, openNode, which, dest, LABEL);
			}
#ifdef LOCAL_PATH
			else if (rp && (absLayer==0) && (currNode->getNum() != dest) &&
							 rp->nodeOccupied(currNode))
			{
				//printf("Can't path to %d, %d\n", (unsigned int)nextChild->getLabelL(kFirstData), (unsigned int)nextChild->getLabelL(kFirstData+1));
				expandedNodes.push_back(currNode);
				currNode->key = expandedNodes.size()-1;
				// ignore this tile if occupied.
			}
#else
			//else if (currNode->getLabelL(kNodeBlocked) > 0)
			else if (((n->getNum() != source) && (e->getLabelL(kEdgeCapacity) <= 0)) ||
							 ((n->getNum() == source) && (e->getLabelL(kEdgeCapacity) < 0)))
			{
				expandedNodes.push_back(currNode);
				currNode->key = expandedNodes.size()-1;
			}
#endif
			// this node is unexpanded if (1) it isn't on the expanded list
			else if ((currNode->key >= expandedNodes.size()) ||
							 (expandedNodes[currNode->key] != currNode))
			{
				
				unsigned int whichParent = (unsigned int)currNode->getLabelL(kParent);
				unsigned int parentKey = map->GetAbstractGraph(absLayer+1)->getNode(whichParent)->key;
				
				// this node is unexpanded if (2) it's parent is in the eligible parent list
				// (3) or having no eligible parents means we can search anywhere!
				if ((eligibleNodeParents.size() == 0) ||
						((parentKey < eligibleNodeParents.size()) &&
						 (eligibleNodeParents[parentKey] == whichParent)))
				{
					currNode->setLabelF(LABEL, MAXINT);
					currNode->setKeyLabel(LABEL);
					currNode->markEdge(0);
					nodeHeap->add(currNode);
					if (verbose)
						printf("Adding neighbor %d\n", which);
					//nodesExpanded++;
					relaxEdge(nodeHeap, g, e, openNode, which, dest, LABEL);
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
		expandedAnything = true;
		
		openNode = n->getNum();
		if (openNode == dest) { /*printf("Found goal %d\n", dest);*/ break; }
		
		if (verbose) printf("working on %d with cost %1.4f\n", openNode, n->getLabelF(LABEL));
		
		if (currBest)
		{
			if (currBest->getLabelL(kParent) == n->getLabelL(kParent))
			{
				// these lines cause us to take the node with the best h() value
				// instead of the first explored node by A* in the abstraction
				if (currBest->getLabelF(LABEL) >	n->getLabelF(LABEL))
					currBest = n;
			}
			else if (n->getLabelL(kParent) == (long)destParent)
			{
				currBest = n;
				//printf("Exited 'cause we're in our parent %ld/%d (2)\n", n->getLabelL(kParent), destParent);
				break;
			}
		} else {
			currBest = n;
			if (n->getLabelL(kParent) == (long)destParent)
			{
				//printf("Exited 'cause we're in our parent %ld/%d (1)\n", n->getLabelL(kParent), destParent);
				break;
			}
		}
	}
	
	delete nodeHeap;
	
	if (!expandedAnything) return source;
	
	if ((currBest) && (openNode != dest))
	{
		dest = currBest->getNum();
		if ((partialLimit > 0) && (currBest->getLabelL(kParent) != (long)destParent))
		{
			if (verbose) printf("Error: We somehow didn't end up in our parent...\n");
		}
	}
	return dest;
}

void praStar::relaxEdge(heap *nodeHeap, graph *g, edge *e, int source, int nextNode, int dest,
												int LABEL)
{
	double weight;
	node *from = g->getNode(source);
	node *to = g->getNode(nextNode);
	node *d = g->getNode(dest);
	weight = from->getLabelF(LABEL)-map->h(from, d)+map->h(to, d)+e->getWeight();
	if (fless(weight, to->getLabelF(LABEL)))
	{
		if (verbose)
			printf("Updating %d to %1.4f from %1.4f\n", nextNode, weight, to->getLabelF(LABEL));
		//weight -= 0.001*(weight-map->h(to, d)); // always lower g-cost slightly so that we tie break in favor of higher g cost
		to->setLabelF(LABEL, weight);
		nodeHeap->decreaseKey(to);
		// this is the edge used to get to this node in the min. path tree
		to->markEdge(e);
	}
}


//void praStar::addAbstractedNodesToHeap(heap *nodeHeap, graph *g, node *p, unsigned int source,
//																			 int LABEL)
//{
//  for (int cnt = 0; cnt < p->getLabelL(kNumAbstractedNodes); cnt++) {
//    node *childNode = g->getNode((unsigned int)p->getLabelL(kFirstData+cnt));
//    if (nodeHeap->isIn(childNode)) continue;
//    if (verbose) printf("%d ", childNode->getNum());
//    childNode->setKeyLabel(LABEL);
//    childNode->setLabelF(LABEL, MAXINT);
//    childNode->markEdge(0);
//    if (childNode->getNum() != source) nodeHeap->add(childNode);
//  }
//  if (verbose) printf("\n");
//}
//
//void praStar::addAbstractedNodesAndNeighborsToHeap(heap *nodeHeap, graph *g, node *p,
//																									 unsigned int source, int LABEL)
//{
//  for (int cnt = 0; cnt < p->getLabelL(kNumAbstractedNodes); cnt++) {
//    unsigned int childNum;
//    node *childNode = g->getNode(childNum = (unsigned int)p->getLabelL(kFirstData+cnt));
//    childNode->setKeyLabel(LABEL);
//    childNode->setLabelF(LABEL, MAXINT);
//    childNode->markEdge(0);
//    if ((childNode->getNum() != source) && (!nodeHeap->isIn(childNode))) {
//      if (verbose) printf("%d ", childNode->getNum());
//      nodeHeap->add(childNode);
//    }
//		
//    edge_iterator ei = childNode->getEdgeIter(); 
//    for (edge *e = childNode->edgeIterNext(ei); e; e = childNode->edgeIterNext(ei)) {
//      node *neighbor;
//      unsigned int which;
//      if ((which = e->getFrom()) == childNum) which = e->getTo();
//      neighbor = g->getNode(which);
//      if (!nodeHeap->isIn(neighbor)) {
//				neighbor->setKeyLabel(LABEL);
//				neighbor->setLabelF(LABEL, MAXINT);
//				neighbor->markEdge(0);
//				if (neighbor->getNum() != source) {
//					if (verbose) printf("%d ", neighbor->getNum());
//					nodeHeap->add(neighbor);
//				}
//      }
//    }
//  }
//  if (verbose) printf("\n");
//}
	


//void praStar::doLibraryRandomPath(bool repeat, bool optimal)
//{
//	static double lastLength, lastTime;
//	static node *r1, *r2;
//	if (verbose)
//		cout << "Clearing marked nodes" << endl;
//	ClearMarkedNodes();
//	
//	if (!repeat)
//	{
//		do {
//			do {
//				r1 = abstractions[0]->getRandomNode();
//			} while (m->getTerrainType((long)r1->getLabelL(kFirstData), (long)r1->getLabelL(kFirstData+1)) == kOutOfBounds);
//			do {
//				r2 = abstractions[0]->getRandomNode();
//			} while (m->getTerrainType((long)r2->getLabelL(kFirstData), (long)r2->getLabelL(kFirstData+1)) == kOutOfBounds);
//		} while (!Pathable(r1, r2));
//	}
//		
//	if (verbose)
//	{
//		cout << "Attempting path between nodes:" << endl;
//		cout << (*r1) << endl << (*r2) << endl;
//	}
//	
//	//	while (r1 != r2)
//	//	{
//	//		r1 = getPathStep(r1, r2);
//	//		if (verbose)
//	//			cout << "Stepping to " << (*r1) << endl;
//	//	}
//	
//	// ignoring return value! Leaking memory!
//	
//#ifdef OS_MAC
//	AbsoluteTime startTime = UpTime();
//#else
//	clock_t startTime, endTime;
//	long double duration;
//	startTime = clock();
//	
//#endif
//	
//	
//	path *p;
//	
//	if (optimal)
//		p = getLibraryPath(r1, r2);
//	else
//		p = getApproximatePath(r1, r2);
//	//		getPathStep(r1, r2);
//	
//	
//#ifdef OS_MAC
//	
//	AbsoluteTime stopTime = UpTime();
//	Nanoseconds diff = AbsoluteDeltaToNanoseconds(stopTime, startTime);
//	uint64_t nanosecs = UnsignedWideToUInt64(diff);
//	cout << nanosecs << " ns elapsed (" << (double)nanosecs/1000000.0 << " ms)" << endl;
//#else
//	endTime = clock();
//	duration=(long double)(endTime-startTime)/CLOCKS_PER_SEC;
//	cout << duration << " seconds elapsed" << endl;
//	
//#endif
//	
//	int cnt = 0;
//	double length = 0;
//	for (path *q = p; q; q = q->next)
//	{
//		if (q && q->next)
//		{
//			double t1, t2;
//			t1 = q->n->getLabelL(kFirstData)-q->next->n->getLabelL(kFirstData);
//			t2 = q->n->getLabelL(kFirstData+1)-q->next->n->getLabelL(kFirstData+1);
//			length += sqrt(t1*t1+t2*t2);
//		}
//		cnt++;
//	}
//	
//#ifdef OS_MAC
//	cout << "Path length " << cnt << " steps, " << length << ", " << (double)nanosecs/(1000*cnt) << " µs per step, ";
//	cout << (double)nanosecs/(1000*length) << " µs per distance" << endl;
//	
//	
//	if (!repeat)
//	{
//		lastLength = length;
//		lastTime = (double)nanosecs/1000000.0;
//	}
//	else {
//		cout << "Comparison: " << lastLength/length << "x longer; but " << ((double)nanosecs/1000000.0)/lastTime << "x faster." << endl;
//	}
//#else
//	cout << "Path length " << cnt << " steps, " << length << endl;
//	
//	if (!repeat)
//	{
//		lastLength = length;
//	}
//	else {
//		cout << "Comparison: " << lastLength/length << "x longer" << endl;
//	}
//#endif	
//	
//	clearDisplayLists();
//}
//
//
//using namespace PathFind; 	
//
//typedef AStar<MarkerFastClear, AStarOpenBuckets<MarkerFastClear, AStarCompareF>, AStarClosedLookup<MarkerFastClear> > ASTAR;
//
//path* praStar::getLibraryPath(node* from, node* to)
//{
//	printf("Getting the library path\n");	
//	static ASTAR *search = new ASTAR();
//	
//	//auto_ptr<Search> search(theAStar);
//	//search.reset(theAStar);
//	
//	search->setNodesLimit(100000000L);
//	
//	int start, target;
//	//path *p = 0;
//	MapEnv me(this, m, abstractions[0]);
//	//	MapEnv me(m);
//	
//	//me.getStartTarget(&start, &target);
//	start = from->getNum();
//	target = to->getNum();
//	
//	// 	SearchUtils searchUtils;
//	// 	
//	// 	if (searchUtils.checkPathExists(me, start, target))
//	// 		printf("Search Utils -> Path exists\n");
//	// 	else
//	// 		printf("Search Utils -> Path does not exist\n");
//		
//	search->findPath(me, start, target);
//		
//	const vector<int>& resultPath = search->getPath();
//	
//	while(resultPath.size() < 1) {
//		printf("Bad library path!\n");
//		//return;
//		me.getStartTarget(&start, &target);
//		
//		search->findPath(me, start, target);
//		
//		resultPath = search->getPath();
//	}
//	
//	//Display the path
//	path *p = 0;
//	edge *e;
//	
//	for (vector<int>::const_iterator i = resultPath.begin(); i != resultPath.end()-1; ++i) {
//		e = abstractions[0]->findEdge(*(i+1), *i);
//		
//		if (!e) {
//			e = abstractions[0]->findEdge(*i, *(i+1));
//			
//			if (!e) 
//				printf("Error, invalid edge: From (*i) = %d, To: (*(i+1)) = %d\n", *i, *(i+1));
//			else {
//				e->setMarked(true);
//			}
//		}
//		else {
//			e->setMarked(true);
//		}
//		p = new path(abstractions[0]->getNode(*i), p);
//	}
//	return p;
//}
//
