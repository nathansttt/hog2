/*
 *  MapLineAbstraction.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 11/10/06.
 *  Copyright 2006 __MyCompanyName__. All rights reserved.
 *
 */

#include "MapLineAbstraction.h"

MapLineAbstraction::MapLineAbstraction(Map *_map, int dist, bool uniform)
:mapAbstraction(_map), lineDistance(dist), abstractUniformly(uniform)
{
	buildAbstraction();
}

MapLineAbstraction::~MapLineAbstraction()
{
}

bool MapLineAbstraction::MapLineAbstraction::pathable(node *from, node *to)
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
void MapLineAbstraction::verifyHierarchy()
{
	assert(false); // not implemented
}

void MapLineAbstraction::removeNode(node *)
{
	assert(false); // not implemented
}

void MapLineAbstraction::removeEdge(edge *, unsigned int)
{
	assert(false); // not implemented
}

void MapLineAbstraction::addNode(node *)
{
	assert(false); // not implemented
}

void MapLineAbstraction::addEdge(edge *, unsigned int)
{
	assert(false); // not implemented
}

void MapLineAbstraction::repairAbstraction()
{
	assert(false); // not implemented
}

void MapLineAbstraction::buildAbstraction()
{
	abstractions.push_back(getMapGraph(getMap()));
	while (abstractions.back()->getNumEdges() > 0)
	{
//		printf("%d nodes and %d edges in graph %d\n",
//					 abstractions.back()->getNumNodes(),
//					 abstractions.back()->getNumEdges(),
//					 abstractions.size()-1);
		fflush(stdout);
		graph *g = new graph();
		addNodes(g);
		addEdges(g);
		abstractions.push_back(g);
	}	
//	printf("%d nodes and %d edges in graph %d\n",
//				 abstractions.back()->getNumNodes(),
//				 abstractions.back()->getNumEdges(),
//				 abstractions.size()-1);
}

void MapLineAbstraction::addNodes(graph *g)
{
	int count = abstractions.back()->getNumNodes();
	//printf("Initial count: %d\n", count);
	int xstep = pow(lineDistance,((abstractions.size()+1)/2));
	int ystep = pow(lineDistance,((abstractions.size())/2));
	int zstep = pow(lineDistance,((abstractions.size()-1)/2));
	graph *toAbstract = abstractions.back();
	//printf("Size: %d.[%d] XStep: %d, YStep: %d, ZStep %d\n", abstractions.size(), lineDistance, xstep, ystep, zstep);

	if (abstractUniformly)
	{
		for (int x = 0; x < getMap()->getMapWidth(); x += xstep)
		{
			for (int y = 0; y < getMap()->getMapHeight(); y+= ystep)
			{
				//printf("Next check starts from (%d, %d)\n", x, y);
				std::vector<node *> nodes;
				std::vector<node *> parents;
				for (int z = 0; z < lineDistance; z++)
				{
					if ((abstractions.size()%2) != 0) // vertical abstraction
					{
						//printf("->(%d, %d)\n", x+z*zstep, y);
						nodes.push_back(getNodeFromMap(x+z*zstep, y));
					}
					else {
						//printf("->(%d, %d)\n", x, y+z*zstep);
						nodes.push_back(getNodeFromMap(x, y+z*zstep));
					}
					
					if (nodes.back() == 0)
					{
						//printf("(null)\n");
						nodes.resize(0);
						break;
					}
					parents.push_back(getNthParent(nodes.back(), abstractions.size()-1));
					if ((parents.back() == 0) ||
							(parents.back()->getLabelL(kParent) != -1) ||
							((z > 0) && (!toAbstract->findEdge(parents[z]->getNum(), parents[z-1]->getNum()))))
					{
						//if (parents.back() == 0) printf("(null)\n");
						//else if (parents.back()->getLabelL(kParent) != -1) printf("(parent)\n");
						//else printf("(disconnected)\n");
						parents.resize(0);
						break;
					}
				}
				if ((nodes.size() == 0) || (parents.size() == 0))
					continue;
				//printf("Match!\n");
				node *parent = createParent(g, parents[0]);
				//printf("-->Creating parent (%d)\n", parent->getNum());
				for (int z = 0; z < parents.size(); z++)
				{
					if (parents[z]->getLabelL(kParent) != -1)
						break;
					buildNodeIntoParent(parents[z], parent);
					count--;
					//printf("Abstracting %d, count now %d\n", parents[z]->getNum(), count);
				}
			}
		}
	}
	
//	node_iterator ni = toAbstract->getNodeIter();
//	for (node *n = toAbstract->nodeIterNext(ni); n; n = toAbstract->nodeIterNext(ni))
	std::vector<node *> stopList; // nodes with no parents aren't abstracted
	while (count > 0)
	{
		// select a random node
		node *n = abstractions.back()->getRandomNode();
		assert(n!=NULL);
		if ((n->getLabelL(kParent) == -1) && (n->getNumEdges() != 0))
		{
			node *parent = createParent(g, n);
			//printf("==>Creating parent (%d)\n", parent->getNum());
			buildNodeIntoParent(n, parent);
			count -= 1;
			//printf("Abstracting %d, count now %d\n", n->getNum(), count);

			for (int x = 0; x < lineDistance - 1; x++)
			{
				node *choice = 0;
				double prob = 0;
				neighbor_iterator nbi = n->getNeighborIter();
				for (long tmp = n->nodeNeighborNext(nbi); tmp != -1; tmp = n->nodeNeighborNext(nbi))
				{
					node *neighbor = toAbstract->getNode(tmp);
					if (neighbor->getLabelL(kParent) == -1)
					{
						prob++;
						if ((random()%100) < 100.0/prob)
						{
							choice = neighbor;
						}
					}
				}
				if (choice)
				{
					buildNodeIntoParent(choice, parent);
					count -= 1;
					n = choice;
					//printf("Abstracting %d, count now %d\n", n->getNum(), count);
				}
				else {
					break;
				}
			}

		}
		else if (n->getNumEdges() == 0)
		{
			if ((n->key < stopList.size()) && (stopList[n->key] == n))
			{
				//printf("Already Removed %d from consideration\n", n->getNum());
			}
			else {
				//printf("Removing %d from consideration\n", n->getNum());
				count--;
				n->key = stopList.size();
				stopList.push_back(n);
				//printf("Abstracting [%d], count now %d\n", n->getNum(), count);
			}
		}
	}
}

void MapLineAbstraction::addEdges(graph *aGraph)
{
	graph *g = abstractions.back();
	edge_iterator ei = g->getEdgeIter();
	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		int from = g->getNode(e->getFrom())->getLabelL(kParent);
		int to = g->getNode(e->getTo())->getLabelL(kParent);
		edge *f=0;
		
		if ((from == -1) || (to == -1))
		{
			printf("Error, (%d parent %d) or (%d parent %d) didn't get abstracted!\n",
						 e->getFrom(), from, e->getTo(), to);
			assert(false);
		}
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

void MapLineAbstraction::buildNodeIntoParent(node *n, node *parent)
{
	assert(getAbstractionLevel(n)+1 == getAbstractionLevel(parent));
	n->setLabelL(kParent, parent->getNum());
	parent->setLabelL(kFirstData+parent->getLabelL(kNumAbstractedNodes), n->getNum());
	parent->setLabelL(kNumAbstractedNodes, parent->getLabelL(kNumAbstractedNodes)+1);
	parent->setLabelF(kXCoordinate, kUnknownPosition);
}

node *MapLineAbstraction::createParent(graph *g, node *n)
{
	node *parent;
	g->addNode(parent = new node("??"));
	assert(parent!=NULL);
	parent->setLabelL(kAbstractionLevel, n->getLabelL(kAbstractionLevel)+1); // level in abstraction tree
	parent->setLabelL(kNumAbstractedNodes, 0); // number of abstracted nodes
	parent->setLabelL(kParent, -1); // parent of this node in abstraction hierarchy
	parent->setLabelF(kXCoordinate, kUnknownPosition);
	parent->setLabelL(kNodeBlocked, 0);
	return parent;
}
