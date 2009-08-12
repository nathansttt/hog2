/*
 * $Id: MapFlatAbstraction.h,v 1.5 2006/10/18 23:53:25 nathanst Exp $
 *
 *  MapFlatAbstraction.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 6/10/05.
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

#include "MapAbstraction.h"

#ifndef MAPFLATABSTRACTION_H
#define MAPFLATABSTRACTION_H

class MapFlatAbstraction : public MapAbstraction {
public:
	MapFlatAbstraction(Map *_m);
	~MapFlatAbstraction();
	/** return a new abstraction map of the same type as this map abstraction */
	virtual MapAbstraction *Clone(Map *_m) { return new MapFlatAbstraction(_m); }
	
	virtual bool Pathable(node *from, node *to);
	
	// utility functions
	/** verify that the hierarchy is consistent */
	virtual void VerifyHierarchy();
	
	// hierarchical modifications
	/** remove node from abstraction */
	virtual void RemoveNode(node *n);
	/** remove edge from abstraction */
  virtual void RemoveEdge(edge *e, unsigned int absLevel);
	/** add node to abstraction */
	virtual void AddNode(node *n);
	/** add edge to abstraction */
	virtual void AddEdge(edge *e, unsigned int absLevel);
	/** This must be called after any of the above add/remove operations. But the
		operations can be stacked followed by a single RepairAbstraction call. */
  virtual void RepairAbstraction();	
private:
		void buildConnectivityGroups();
		bool groupsValid;
		std::vector<int> groups;
};

#endif //  MAPFLATABSTRACTION_H
