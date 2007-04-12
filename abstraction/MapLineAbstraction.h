/*
 *  $Id: MapLineAbstraction.h,v 1.3 2007/04/02 23:39:13 nathanst Exp $
 *  MapLineAbstraction.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 11/10/06.
 *  Copyright 2006 __MyCompanyName__. All rights reserved.
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

#include "mapAbstraction.h"

#ifndef MAPLINEABSTRACTION_H
#define MAPLINEABSTRACTION_H

class MapLineAbstraction : public mapAbstraction {
public:
	MapLineAbstraction(Map *, int dist = 2, bool uniform = true);
	~MapLineAbstraction();
	mapAbstraction *clone(Map *_m) { return new MapLineAbstraction(_m); }
	
	virtual bool pathable(node *from, node *to);
	
	// utility functions
	/** verify that the hierarchy is consistent */
	virtual void verifyHierarchy();
	
	// hierarchical modifications
	/** remove node from abstraction */
	virtual void removeNode(node *n);
	/** remove edge from abstraction */
  virtual void removeEdge(edge *e, unsigned int absLevel);
	/** add node to abstraction */
	virtual void addNode(node *n);
	/** add edge to abstraction */
	virtual void addEdge(edge *e, unsigned int absLevel);
	/** This must be called after any of the above add/remove operations. But the
		operations can be stacked followed by a single repairAbstraction call. */
  virtual void repairAbstraction();	
private:
	void buildAbstraction();
	void buildNodeIntoParent(node *n, node *parent);
	node *createParent(graph *g, node *n);
	
	void addEdges(graph *g);
	void addNodes(graph *g);
	int lineDistance;
	bool abstractUniformly;
};

#endif
