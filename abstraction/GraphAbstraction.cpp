/*
 * $Id: graphAbstraction.cpp,v 1.10 2007/03/25 00:02:43 nathanst Exp $
 *
 *  graphAbstraction.cpp
 *  HOG
 *
 *  Created by Nathan Sturtevant on 1/11/05.
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

using namespace std;

#include "GraphAbstraction.h"
#include <math.h>

enum {
	kQuiet = 0x00,
	kBuildGraph = 0x01,
	kRepairGraph = 0x02,
	kMiscMessages = 0x04
};

const int verbose = kQuiet;//kRepairGraph;

graphAbstraction::~graphAbstraction()
{ 

  while (abstractions.size() > 0)
	{
    delete abstractions.back();
    abstractions.pop_back();
  }
}

void graphAbstraction::getParentHierarchy(node *from, node *to,
																					std::vector<node *> &fromChain,
																					std::vector<node *> &toChain)
{
  //	while (from->getLabelL(kParent) != to->getLabelL(kParent))
  while (!abstractions[from->getLabelL(kAbstractionLevel)]->
				 findEdge(from->getNum(), to->getNum()) &&
				 //(from->getLabelL(kParent) != to->getLabelL(kParent))
				 (from != to))
	{
    fromChain.push_back(from);
    toChain.push_back(to);
		
    from = abstractions[from->getLabelL(kAbstractionLevel)+1]->
      getNode(from->getLabelL(kParent));
		
    to = abstractions[to->getLabelL(kAbstractionLevel)+1]->
      getNode(to->getLabelL(kParent));
		
    if (verbose&kBuildGraph)
			printf("getParentHierarchy: Moving to nodes #%d and %d\n",
						 from->getNum(), to->getNum());
  }
  fromChain.push_back(from);
  toChain.push_back(to);
}

// get nth-level parent of which node
node *graphAbstraction::getNthParent(node *which, int n)
{
	while ((which != NULL) && (which->getLabelL(kAbstractionLevel) < n))
	{
		int nextLevel = which->getLabelL(kAbstractionLevel)+1;
		graph *g = abstractions[nextLevel];
		which = g->getNode(which->getLabelL(kParent));
	}
	return which;
}

void graphAbstraction::clearMarkedNodes()
{
	for (unsigned int x = 0; x < abstractions.size(); x++)
	{
		graph *g = abstractions[x];
	 	edge_iterator ei = g->getEdgeIter();
		for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
		{
			e->setMarked(false);
		}
	}
}

double graphAbstraction::distance(path *p)
{
	if ((p == 0) || (p->next == 0) || (p->n == 0) || (p->next->n == 0))
		return 0.0;
	return h(p->n, p->next->n)+distance(p->next);
}

void graphAbstraction::measureAbstractionValues(int level, double &n, double &n_dev, double &c, double &c_dev)
{
	assert((level > 0) && (level < (int)abstractions.size()));
	graph *g = abstractions[level];
	
	std::vector<int> ns;
	std::vector<int> cs;
	n = 0; c = 0;
	node_iterator ni = g->getNodeIter();
	for (node *nd = g->nodeIterNext(ni); nd; nd = g->nodeIterNext(ni))
	{
		ns.push_back(nd->getLabelL(kNumAbstractedNodes));
		n += ns.back();
		cs.push_back(computeWidth(nd));
		c += cs.back();
		//printf("node %d: #%d width:%d\n", nd->getNum(), ns.back(), cs.back());
	}
	n/= ns.size();
	c/= cs.size();
	n_dev = 0;
	c_dev = 0;
	for (unsigned int x = 0; x < ns.size(); x++)
	{
		n_dev += (ns[x]-n)*(ns[x]-n);
		c_dev += (cs[x]-c)*(cs[x]-c);
	}
	n_dev /= ns.size();
	c_dev /= cs.size();
	n_dev = sqrtf(n_dev);
	c_dev = sqrtf(c_dev);
}

int graphAbstraction::computeWidth(node *n)
{
//	printf("New WIDTH (%d)\n", n->getNum());
	int width = 0;
	for (int child = 0; child < n->getLabelL(kNumAbstractedNodes); child++)
	{
		width = std::max(width, widthBFS(getNthChild(n, child), n));
	}
	return width;
}

int graphAbstraction::widthBFS(node *child, node *parent)
{
	std::vector<int> depth;
	std::vector<node *> q;
	graph *g = getAbstractGraph(child);

	q.push_back(child);
	child->key = 0;
	depth.push_back(0);

	for (unsigned int x = 0; x < q.size(); x++)
	{
//		printf("Handling node: %d\n", q[x]->getNum());
		neighbor_iterator ni = q[x]->getNeighborIter();
		for (int next = q[x]->nodeNeighborNext(ni); next != -1; next = q[x]->nodeNeighborNext(ni))
		{
			node *neighbor = g->getNode(next);
			if ((neighbor->key < q.size()) && (q[neighbor->key] == neighbor))
			{
//				printf("         neighbor %d in Q\n", neighbor->getNum());
				continue;
			}
			if (neighbor->getLabelL(kParent) == parent->getNum())
			{
//				printf("         neighbor %d onto Q(%d) depth (%d)\n", neighbor->getNum(), q.size(), depth[x]+1);
				neighbor->key = q.size();
				q.push_back(neighbor);
				depth.push_back(depth[x]+1);
			}
		}
	}
	return depth.back();
}

double graphAbstraction::measureAverageNodeWidth(int level)
{
	if ((level > abstractions.size()) || (level <= 0))
		return 0.0f;
	graph *g = abstractions[level];

	std::vector<double> widths;
	
	double sum = 0;
	node_iterator ni = g->getNodeIter();
	for (node *nd = g->nodeIterNext(ni); nd; nd = g->nodeIterNext(ni))
	{
		double val = measureExpectedNodeWidth(nd);
		// ignore dead-end nodes
		if (val != 0)
		{
			widths.push_back(val);
			sum += val;
			//printf("node %d: expected width:%f\n", nd->getNum(), val);
		}
	}
	if (widths.size() > 0)
		return sum/widths.size();
	return 0;
}

double graphAbstraction::measureExpectedNodeWidth(node *n)
{
	// for each edge, the number of edges at that depth
	std::vector<std::vector<int> > edgeInfo;
	// the total number of edges for this child
	std::vector<int> sums;
	std::vector<double> value;

	edgeInfo.resize(n->getLabelL(kNumAbstractedNodes));
	value.resize(n->getLabelL(kNumAbstractedNodes));
	
	double expectedWidth = 0;
	int totalExternalEdges = 0;
	for (int child = 0; child < n->getLabelL(kNumAbstractedNodes); child++)
	{
		node *c = getNthChild(n, child);
		countEdgesAtDistance(c, n, edgeInfo[child]);
		sums.push_back(0);
		for (unsigned int x = 0; x < edgeInfo[child].size(); x++)
		{
			sums.back() += edgeInfo[child][x];
//			printf("Node %d child %d dist %d has %d edges\n",
//						 n->getNum(), child, x, edgeInfo[child][x]);
		}
//		printf("%d total edges [%d]\n", sums[child], child);
	}
	
	for (int child = 0; child < n->getLabelL(kNumAbstractedNodes); child++)
	{
		double dist = 0;
		// this node has at least one external edge
		// and two external edges total
		if ((edgeInfo[child][0] != 0) && sums[child] > 1)
		{
			totalExternalEdges += edgeInfo[child][0];
			for (unsigned int x = 0; x < edgeInfo[child].size(); x++)
			{
				if (edgeInfo[child][x] > 0)
				{
					if (x == 0)
						dist += (edgeInfo[child][x]-1.0)*(x+1);
					else
						dist += (edgeInfo[child][x])*(x+1);
				}
			}
			dist /= (sums[child]-1);
		}
//		printf("Average width = %f\n", dist);
		value[child] = dist;
	}
	
	if (totalExternalEdges != 0)
	{
		for (int child = 0; child < n->getLabelL(kNumAbstractedNodes); child++)
		{
			expectedWidth += (value[child]*edgeInfo[child][0])/(double)totalExternalEdges;
		}
	}
//	printf("Expected width = %f\n", expectedWidth);
	return expectedWidth;
}

// get the number of edges that n has outside of p
int graphAbstraction::getNumExternalEdges(node *n, node *p)
{
	int count = 0;
	neighbor_iterator ni = n->getNeighborIter();
	graph *g = getAbstractGraph(n);
	for (int next = n->nodeNeighborNext(ni); next != -1; next = n->nodeNeighborNext(ni))
	{
		node *nb = g->getNode(next);
		if (getParent(nb) != p)
			count++;
	}
	return count;
}

// count the number of external edges the child has from the parent at
// each distance
int graphAbstraction::countEdgesAtDistance(node *child, node *parent, std::vector<int> &dists)
{
	dists.resize(0);
	
	std::vector<int> depth;
	std::vector<node *> q;
	graph *g = getAbstractGraph(child);
	
	q.push_back(child);
	child->key = 0;
	depth.push_back(0);
	
	for (int x = 0; x < q.size(); x++)
	{
		//		printf("Handling node: %d\n", q[x]->getNum());
		neighbor_iterator ni = q[x]->getNeighborIter();
		if (depth[x]+1 > dists.size())
			dists.resize(depth[x]+1);
		dists[depth[x]] += getNumExternalEdges(q[x], parent);

		for (int next = q[x]->nodeNeighborNext(ni); next != -1; next = q[x]->nodeNeighborNext(ni))
		{
			node *neighbor = g->getNode(next);
			if ((neighbor->key < q.size()) && (q[neighbor->key] == neighbor))
			{
				//				printf("         neighbor %d in Q\n", neighbor->getNum());
				continue;
			}
			if (neighbor->getLabelL(kParent) == parent->getNum())
			{
				//				printf("         neighbor %d onto Q(%d) depth (%d)\n", neighbor->getNum(), q.size(), depth[x]+1);
				neighbor->key = q.size();
				q.push_back(neighbor);
				depth.push_back(depth[x]+1);
			}
		}
	}
	return depth.back();
}
