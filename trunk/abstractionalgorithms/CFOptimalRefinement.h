/*
 *  CFOptimalRefinement.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/22/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef CFOPTIMALREFINEMENT_H
#define CFOPTIMALREFINEMENT_H

#include <ext/hash_map>
#include "OpenClosedList.h"
#include "FPUtil.h"
#include "graph.h"

/** Definitions for node labels */
enum {
  kAbstractionLevel = 0, // this is a LONG label
	kCorrespondingNode = 1, // this is a LONG label
	kGCost = 2, // this is a DOUBLE label
	kHCost = 3, // this is a DOUBLE label
	kOptimalFlag = 4, // this is a LONG label
	kInOpenList = 5, // this is a LONG label
	kFirstNeighbor
};

struct GNode {
	GNode(node *nn) :n(nn) {}
	node *n;
};

struct NodeEqual {
	bool operator()(const GNode &i1, const GNode &i2)
	{ return (i1.n->getUniqueID() == i2.n->getUniqueID()); }
};

struct NodeCompare {
	bool operator()(const GNode &i1, const GNode &i2)
	{
		// if f-cost is tied
		if (fequal(i1.n->getLabelF(kGCost)+i1.n->getLabelF(kHCost),
							 i2.n->getLabelF(kGCost)+i2.n->getLabelF(kHCost)))
		{
			// if both in/not in open list
			if (i1.n->getLabelL(kInOpenList) == i2.n->getLabelL(kInOpenList))
			{
				// return true if node1 has lower g-cost
				return (fless(i1.n->getLabelF(kGCost), i2.n->getLabelF(kGCost)));
			}
			// return true if node1 isn't in open list
			return (i2.n->getLabelL(kInOpenList));
		}
		// return true if node1 has higher f-cost
		return (fgreater(i1.n->getLabelF(kGCost)+i1.n->getLabelF(kHCost),
										 i2.n->getLabelF(kGCost)+i2.n->getLabelF(kHCost)));
	}
};

struct NodeHash {
		size_t operator()(const GNode &x) const
		{ return (size_t)(x.n->getUniqueID()); }
};

typedef OpenClosedList<GNode, NodeHash, NodeEqual, NodeCompare> PQueue;
typedef __gnu_cxx::hash_map<uint32_t, GNode> NodeLookupTable;

#endif
