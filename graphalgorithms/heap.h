/*
 * $Id: heap.h,v 1.4 2006/11/29 17:40:14 nathanst Exp $
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
#include "graph.h"

/**
 * A simple & efficient heap class which uses graph objects.
 */

class heap {
public:
  heap(int s = DEFAULT_SIZE);
  ~heap();
	unsigned int size();
  void add(graph_object *val);
  void decreaseKey(graph_object *val);
  bool isIn(graph_object *val);
  graph_object *remove();
  bool empty();
private:
  void heapifyUp(int index);
  void heapifyDown(int index);
  std::vector<graph_object *> _elts;
  int count;
};

#endif
