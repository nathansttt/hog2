/*
 * $Id: GraphAbstraction.h,v 1.14 2007/03/25 00:02:43 nathanst Exp $
 *
 *  GraphAbstraction.h
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

#include "Graph.h"
#include "Path.h"

#ifndef GRAPHABSTRACTION_H
#define GRAPHABSTRACTION_H
#include <vector>

/** Definitions for node labels */
enum {
  kAbstractionLevel = 0, // this is a LONG label
  kNumAbstractedNodes = 1, // nodes that we abstract; this is a LONG label
  kParent = 2, // node that abstracts us; this is a LONG label
  kNodeWidth = 3, // the maximum size object that can completely traverse this node; this is a LONG label
  kTemporaryLabel = 4, // for any temporary usage; this label can be LONG or FLOATING point
  kXCoordinate = 5, // cache for opengl drawing; this is a FLOATING POINT label
  kYCoordinate = 6,	// this is a FLOATING POINT label
  kZCoordinate = 7,	// this is a FLOATING POINT label
  kNodeBlocked = 8, // this is a LONG label
  kFirstData = 9
};

/** kFirstData & beyond:
* 
* in abstract graph these are the node numbers of the abstraction (LONG labels)
*
* in 0th level abstraction they are the x, y location of the tile
* along with which side (type tCorner)  (all LONG labels)
*/

/** Definitions for edge labels */
enum {
  kEdgeCapacity=2
};

const double kUnknownPosition = -50.0;

/**
 * A generic class for basic operations on graph abstractions.
 */

class GraphAbstraction {
public:
	GraphAbstraction() :abstractions() {}
	virtual ~GraphAbstraction();

	// basic graph functions
	/** is there a legal path between these 2 nodes? */
  virtual bool Pathable(node *from, node *to) = 0;
	/** given 2 nodes, find as much of their hierarchy that exists in the graph */
  void getParentHierarchy(node *from, node *to, std::vector<node *> &fromChain, std::vector<node *> &toChain);
	/** return the abstract graph at the given level */
  graph* GetAbstractGraph(int level) { return abstractions[level]; }
	/** return the total number of graphs in the hierarchy */
  unsigned int getNumAbstractGraphs() { return abstractions.size(); }
	/** heuristic cost between any two nodes */
  virtual double h(node *a, node *b) = 0;
	/** length in distance of a path */
	double distance(path *p);
	/** return nth level parent of which or null if it doesn't exist */
	node *getNthParent(node *which, int n);

	inline node *getParent(node *which) { return abstractions[getAbstractionLevel(which)+1]->getNode(which->getLabelL(kParent)); }
	inline long getNumChildren(node *which) { return which->getLabelL(kNumAbstractedNodes); }
	inline node *getNthChild(node *which, int n) { return abstractions[getAbstractionLevel(which)-1]->getNode(which->getLabelL(kFirstData+n)); }
	inline long getAbstractionLevel(node *which) { return which->getLabelL(kAbstractionLevel); }
	inline graph* GetAbstractGraph(node *which) { return abstractions[which->getLabelL(kAbstractionLevel)]; }
	// utility functions
	/** verify that the hierarchy is consistent */
	virtual void verifyHierarchy() = 0;
	/// rebuild hierarchy from original domain */
  // virtual void rebuild() = 0;
	/// get current revision of hierarchy -- indicates if changes have been made */
	//virtual int getRevision() = 0;
	void clearMarkedNodes();
	
	// hierarchical modifications
	/** remove node from abstraction */
	virtual void removeNode(node *n) = 0;
	/** remove edge from abstraction */
  virtual void removeEdge(edge *e, unsigned int absLevel) = 0;
	/** add node to abstraction */
	virtual void addNode(node *n) = 0;
	/** add edge to abstraction */
	virtual void addEdge(edge *e, unsigned int absLevel) = 0;
	/** This must be called after any of the above add/remove operations. But the
		operations can be stacked followed by a single repairAbstraction call. */
  virtual void repairAbstraction() = 0;
	virtual int measureRepairHits() { return 0; }
	void measureAbstractionValues(int level, double &n, double &n_dev, double &c, double &c_dev);
	double measureAverageNodeWidth(int level);
protected:
		std::vector<graph *> abstractions;
private:
		int computeWidth(node *n);
		int widthBFS(node *child, node *parent);
		double measureExpectedNodeWidth(node *n);
		int getNumExternalEdges(node *n, node *p);
		int countEdgesAtDistance(node *child, node *parent, std::vector<int> &dists);
};

// this class can be defined if you want to solve multiple domains...
// 
// class domainAbstraction : public GraphAbstraction {
// 	void domainAbstraction(abstractableDomain *) = 0;
// };

#endif
