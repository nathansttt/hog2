/*
 * $Id: path.h,v 1.2 2006/11/01 19:00:06 nathanst Exp $
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

#ifndef PATH_H
#define PATH_H

#include "Graph.h"

/**
 * A linked list of nodes which form a continuous path.
 */
class path {
public:
  node *n;
  path *next;
	
  path(node *_n, path *_next=0) : n(_n), next(_next) {}
  ~path() { if (next != NULL) delete next; }
	path *Clone() { return next?(new path(n, next->Clone())):new path(n, next);}
	path *tail() { if (next) return next->tail(); return this; }
	/** reverses path in place, and returns pointer to new head of path */
	path *reverse();
  /** returns the number of steps along the path */
  unsigned length(void); 
//  /** returns the distance covered by the path */
//  double distance(GraphAbstraction* aMap);
  unsigned degree();
  void Print(bool beginning=true);
};

#endif 
