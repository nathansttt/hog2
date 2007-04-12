/*
 * $Id: heap.cpp,v 1.6 2006/11/29 17:40:14 nathanst Exp $
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

#include <iostream>
#include "fpUtil.h"
#include "heap.h"

heap::heap(int s)
{
  count = 0;
  _elts.reserve(s);
}

heap::~heap()
{
}

unsigned int heap::size()
{
	return _elts.size();
}

/**
 * Add object into heap.
 */
void heap::add(graph_object *val)
{
  val->key = count;
  _elts.push_back(val);
  count++;
  heapifyUp(val->key);
}

/**
 * Indicate that the key for a particular object has decreased.
 */
void heap::decreaseKey(graph_object *val)
{
  heapifyUp(val->key);
}

/**
 * Returns true if the object is in the heap.
 */
bool heap::isIn(graph_object *val)
{
  if (val->key < _elts.size() &&
			(_elts[val->key] == val))
		return true;
  return false;
}

/**
 * Remove the item with the lowest key from the heap & re-heapify.
 */
graph_object *heap::remove()
{
  if (empty())
		return 0;
  count--;
  graph_object *ans = _elts[0];
  _elts[0] = _elts[count];
  _elts[0]->key = 0;
  _elts.pop_back();
  heapifyDown(0);

  return ans;
}

/**
 * Returns true if no items are in the heap.
 */
bool heap::empty()
{
  return count == 0;
}

void heap::heapifyUp(int index)
{
  if (index == 0) return;
  int parent = (index-1)/2;

  if (fgreater(_elts[parent]->getKey(), _elts[index]->getKey()))
	{
    graph_object *tmp = _elts[parent];
    _elts[parent] = _elts[index];
    _elts[index] = tmp;
    _elts[parent]->key = parent;
    _elts[index]->key = index;
    heapifyUp(parent);
  }
}

void heap::heapifyDown(int index)
{
  int child1 = index*2+1;
  int child2 = index*2+2;
  int which;
	
  // find smallest child
  if (child1 >= count)
    return;
  else if (child2 >= count)
    which = child1;
  else if (fless(_elts[child1]->getKey(), _elts[child2]->getKey()))
    which = child1;
  else
    which = child2;

  if (fless(_elts[which]->getKey(), _elts[index]->getKey()))
	{
    graph_object *tmp = _elts[which];
    _elts[which] = _elts[index];
    _elts[index] = tmp;
    _elts[which]->key = which;
    _elts[index]->key = index;
    heapifyDown(which);
  }
}
