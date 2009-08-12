/*
 *  MapLineAbstraction.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 11/10/06.
 *  Copyright 2006 __MyCompanyName__. All rights reserved.
 *
 */

#include "MapLineAbstraction.h"

using namespace GraphAbstractionConstants;

MapLineAbstraction::MapLineAbstraction(Map *_map, int dist, bool uniform)
:MapAbstraction(_map), lineDistance(dist), abstractUniformly(uniform)
{
	buildAbstraction();
}

MapLineAbstraction::~MapLineAbstraction()
{
}

bool MapLineAbstraction::MapLineAbstraction::Pathable(node *from, node *to)
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
void MapLineAbstraction::VerifyHierarchy()
{
	assert(false); // not implemented
}

void MapLineAbstraction::RemoveNode(node *)
{
	assert(false); // not implemented
}

void MapLineAbstraction::RemoveEdge(edge *, unsigned int)
{
	assert(false); // not implemented
}

void MapLineAbstraction::AddNode(node *)
{
	assert(false); // not implemented
}

void MapLineAbstraction::AddEdge(edge *, unsigned int)
{
	assert(false); // not implemented
}

void MapLineAbstraction::RepairAbstraction()
{
	assert(false); // not implemented
}

void MapLineAbstraction::buildAbstraction()
{
	abstractions.push_back(GetMapGraph(GetMap()));
	while (abstractions.back()->GetNumEdges() > 0)
	{
//		printf("%d nodes and %d edges in Graph %d\n",
//					 abstractions.back()->GetNumNodes(),
//					 abstractions.back()->GetNumEdges(),
//					 abstractions.size()-1);
		fflush(stdout);
		Graph *g = new Graph();
		addNodes(g);
		addEdges(g);
		abstractions.push_back(g);
	}	
//	printf("%d nodes and %d edges in Graph %d\n",
//				 abstractions.back()->GetNumNodes(),
//				 abstractions.back()->GetNumEdges(),
//				 abstractions.size()-1);
}

void MapLineAbstraction::addNodes(Graph *g)
{
	int count = abstractions.back()->GetNumNodes();
	//printf("Initial count: %d\n", count);
	int xstep = pow(lineDistance,((abstractions.size()+1)/2));
	int ystep = pow(lineDistance,((abstractions.size())/2));
	int zstep = pow(lineDistance,((abstractions.size()-1)/2));
	Graph *toAbstract = abstractions.back();
	//printf("Size: %d.[%d] XStep: %d, YStep: %d, ZStep %d\n", abstractions.size(), lineDistance, xstep, ystep, zstep);

	if (abstractUniformly)
	{
		for (int x = 0; x < GetMap()->GetMapWidth(); x += xstep)
		{
			for (int y = 0; y < GetMap()->GetMapHeight(); y+= ystep)
			{
				//printf("Next check starts from (%d, %d)\n", x, y);
				std::vector<node *> nodes;
				std::vector<node *> parents;
				for (int z = 0; z < lineDistance; z++)
				{
					if ((abstractions.size()%2) != 0) // vertical abstraction
					{
						//printf("->(%d, %d)\n", x+z*zstep, y);
						nodes.push_back(GetNodeFromMap(x+z*zstep, y));
					}
					else {
						//printf("->(%d, %d)\n", x, y+z*zstep);
						nodes.push_back(GetNodeFromMap(x, y+z*zstep));
					}
					
					if (nodes.back() == 0)
					{
						//printf("(null)\n");
						nodes.resize(0);
						break;
					}
					parents.push_back(GetNthParent(nodes.back(), abstractions.size()-1));
					if ((parents.back() == 0) ||
							(parents.back()->GetLabelL(kParent) != -1) ||
							((z > 0) && (!toAbstract->FindEdge(parents[z]->GetNum(), parents[z-1]->GetNum()))))
					{
						//if (parents.back() == 0) printf("(null)\n");
						//else if (parents.back()->GetLabelL(kParent) != -1) printf("(parent)\n");
						//else printf("(disconnected)\n");
						parents.resize(0);
						break;
					}
				}
				if ((nodes.size() == 0) || (parents.size() == 0))
					continue;
				//printf("Match!\n");
				node *parent = createParent(g, parents[0]);
				//printf("-->Creating parent (%d)\n", parent->GetNum());
				for (unsigned int z = 0; z < parents.size(); z++)
				{
					if (parents[z]->GetLabelL(kParent) != -1)
						break;
					buildNodeIntoParent(parents[z], parent);
					count--;
					//printf("Abstracting %d, count now %d\n", parents[z]->GetNum(), count);
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
		node *n = abstractions.back()->GetRandomNode();
		assert(n!=NULL);
		if ((n->GetLabelL(kParent) == -1) && (n->GetNumEdges() != 0))
		{
			node *parent = createParent(g, n);
			//printf("==>Creating parent (%d)\n", parent->GetNum());
			buildNodeIntoParent(n, parent);
			count -= 1;
			//printf("Abstracting %d, count now %d\n", n->GetNum(), count);

			for (int x = 0; x < lineDistance - 1; x++)
			{
				node *choice = 0;
				double prob = 0;
				neighbor_iterator nbi = n->getNeighborIter();
				for (long tmp = n->nodeNeighborNext(nbi); tmp != -1; tmp = n->nodeNeighborNext(nbi))
				{
					node *neighbor = toAbstract->GetNode(tmp);
					if (neighbor->GetLabelL(kParent) == -1)
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
					//printf("Abstracting %d, count now %d\n", n->GetNum(), count);
				}
				else {
					break;
				}
			}

		}
		else if (n->GetNumEdges() == 0)
		{
			if ((n->key < stopList.size()) && (stopList[n->key] == n))
			{
				//printf("Already Removed %d from consideration\n", n->GetNum());
			}
			else {
				//printf("Removing %d from consideration\n", n->GetNum());
				count--;
				n->key = stopList.size();
				stopList.push_back(n);
				//printf("Abstracting [%d], count now %d\n", n->GetNum(), count);
			}
		}
	}
}

void MapLineAbstraction::addEdges(Graph *aGraph)
{
	Graph *g = abstractions.back();
	edge_iterator ei = g->getEdgeIter();
	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		int from = g->GetNode(e->getFrom())->GetLabelL(kParent);
		int to = g->GetNode(e->getTo())->GetLabelL(kParent);
		edge *f=0;
		
		if ((from == -1) || (to == -1))
		{
			printf("Error, (%d parent %d) or (%d parent %d) didn't get abstracted!\n",
						 e->getFrom(), from, e->getTo(), to);
			assert(false);
		}
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

void MapLineAbstraction::buildNodeIntoParent(node *n, node *parent)
{
	assert(GetAbstractionLevel(n)+1 == GetAbstractionLevel(parent));
	n->SetLabelL(kParent, parent->GetNum());
	parent->SetLabelL(kFirstData+parent->GetLabelL(kNumAbstractedNodes), n->GetNum());
	parent->SetLabelL(kNumAbstractedNodes, parent->GetLabelL(kNumAbstractedNodes)+1);
	parent->SetLabelF(kXCoordinate, kUnknownPosition);
}

node *MapLineAbstraction::createParent(Graph *g, node *n)
{
	node *parent;
	g->AddNode(parent = new node("??"));
	assert(parent!=NULL);
	parent->SetLabelL(kAbstractionLevel, n->GetLabelL(kAbstractionLevel)+1); // level in abstraction tree
	parent->SetLabelL(kNumAbstractedNodes, 0); // number of abstracted nodes
	parent->SetLabelL(kParent, -1); // parent of this node in abstraction hierarchy
	parent->SetLabelF(kXCoordinate, kUnknownPosition);
	parent->SetLabelL(kNodeBlocked, 0);
	return parent;
}
