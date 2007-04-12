/*
 * $Id: praStar.h,v 1.8 2007/03/21 22:01:11 nathanst Exp $
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

#ifndef PRASTAR_H
#define PRASTAR_H

#include <iostream>
#include "searchAlgorithm.h"
#include "heap.h"

/**
 * The pra* search algorithm which does partial pathfinding using abstraction.
 */

class praStar : public searchAlgorithm {

public:
	praStar();
	virtual ~praStar() {}
	virtual path *getPath(graphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);
	virtual const char *getName() { return algName; } 
	void setPartialPathLimit(int limit) { partialLimit = limit; 
		sprintf(algName,"PRA*(%d)",partialLimit); }
	int getPartialPathLimit() { return partialLimit; }
	void setPlanFromMiddle(bool _planFromMiddle) { planFromMiddle = _planFromMiddle; }
	void setExpandSearchRadius(bool _expandSearchRadius) { expandSearchRadius = _expandSearchRadius; }
	void setUseSmoothing(bool _smoothing) { smoothing = _smoothing; }
	void setCache(path **p);
	/** Set a fixed level for abstraction in planning. -1 to return to dynamic level selection */
	void setFixedPlanLevel(int p) { fixedPlanLevel = p; }
	void getAbstractPathLengths(std::vector<int> &len) { len = lengths; }
protected:

  path *getAbstractPath(graph *g, unsigned int source, unsigned int destParent,
			std::vector<unsigned int> &eligibleNodeParents, int LABEL,
			unsigned int dest);
  
  unsigned int astar(graph *g, unsigned int source, unsigned int destParent,
		     std::vector<unsigned int> &eligibleNodeParents, int LABEL,
		     unsigned int dest);
  
  void relaxEdge(heap *nodeHeap, graph *g, edge *e, int source, int nextNode, int dest,
		 int LABEL);
	path *smoothPath(path *p);
	
	path **cache;
  int partialLimit;
	int fixedPlanLevel;
  char algName[30];
  graphAbstraction *map;
	bool expandSearchRadius;
	bool planFromMiddle;
	bool smoothing;
	reservationProvider *rp;
	std::vector<int> lengths;
};


#endif
