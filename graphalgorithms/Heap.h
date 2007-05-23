/*
 * $Id: Heap.h,v 1.4 2006/11/29 17:40:14 nathanst Exp $
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
*/ 

// HOG File

#ifndef HEAP_H
#define HEAP_H

#include <vector>
#include <list>

#define DEFAULT_SIZE 10
#include "Graph.h"

/**
 * A simple & efficient Heap class which uses Graph objects.
 */

class Heap {
public:
  Heap(int s = DEFAULT_SIZE);
  ~Heap();
	unsigned int size();
  void Add(graph_object *val);
  void DecreaseKey(graph_object *val);
  bool IsIn(graph_object *val);
  graph_object *Remove();
  bool Empty();
private:
  void HeapifyUp(int index);
  void HeapifyDown(int index);
  std::vector<graph_object *> _elts;
  int count;
};

#endif
