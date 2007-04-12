/*
 * $Id: heap2.h,v 1.9 2006/11/27 23:21:37 nathanst Exp $
 *
 *  hog
 *
 *  Created by Nathan Sturtevant on 1/14/06.
 *  Copyright 2006 Nathan Sturtevant. All rights reserved.
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

#ifndef HEAP2_H
#define HEAP2_H

#include <cassert>
#include <vector>
#include <ext/hash_map>

/**
* A simple & efficient heap class.
 */


template<typename OBJ, class HashKey, class EqKey, class CmpKey>
class heap2 {
public:
  heap2();
  ~heap2();
	void reset();
  void add(OBJ val);
  void decreaseKey(OBJ val);
  bool isIn(OBJ val);
  OBJ remove();
	void pop() { remove(); }
	OBJ top() { return _elts[0]; }
	OBJ find(OBJ val);
  bool empty();
	unsigned size() { return _elts.size(); }
//	void verifyData();
private:
  std::vector<OBJ> _elts;
	void heapifyUp(unsigned int index);
  void heapifyDown(unsigned int index);
	typedef __gnu_cxx::hash_map<OBJ, unsigned int, HashKey, EqKey > IndexTable;
	IndexTable table;
};


template<typename OBJ, class HashKey, class EqKey, class CmpKey>
heap2<OBJ, HashKey, EqKey, CmpKey>::heap2()
{
}

template<typename OBJ, class HashKey, class EqKey, class CmpKey>
heap2<OBJ, HashKey, EqKey, CmpKey>::~heap2()
{
}

/**
* Remove all objects from queue.
 */
template<typename OBJ, class HashKey, class EqKey, class CmpKey>
void heap2<OBJ, HashKey, EqKey, CmpKey>::reset()
{
	table.clear();
	_elts.resize(0);
}

/**
* Add object into heap2.
 */
template<typename OBJ, class HashKey, class EqKey, class CmpKey>
void heap2<OBJ, HashKey, EqKey, CmpKey>::add(OBJ val)
{
    assert(table.find(val) == table.end());
	table[val] = _elts.size();
	//  val->key = count;
  _elts.push_back(val);
  //count++;
	//  heapifyUp(val->key);
  heapifyUp(table[val]);
}

/**
* Indicate that the key for a particular object has decreased.
 */
template<typename OBJ, class HashKey, class EqKey, class CmpKey>
void heap2<OBJ, HashKey, EqKey, CmpKey>::decreaseKey(OBJ val)
{
	EqKey eq;
	assert(eq(_elts[table[val]], val));
  //heapifyUp(val->key);
	_elts[table[val]] = val;
  heapifyUp(table[val]);
}

/**
* Returns true if the object is in the heap2.
 */
template<typename OBJ, class HashKey, class EqKey, class CmpKey>
bool heap2<OBJ, HashKey, EqKey, CmpKey>::isIn(OBJ val)
{
	EqKey eq;
  if ((table.find(val) != table.end()) && (table[val] < _elts.size()) &&
			(eq(_elts[table[val]], val)))
		return true;
  return false;
}

/**
* Remove the item with the lowest key from the heap2 & re-heapify.
 */
template<typename OBJ, class HashKey, class EqKey, class CmpKey>
OBJ heap2<OBJ, HashKey, EqKey, CmpKey>::remove()
{
  if (empty())
		return OBJ();
  // count--;
  OBJ ans = _elts[0];
  _elts[0] = _elts[_elts.size()-1];
	table[_elts[0]] = 0;
  //_elts[0]->key = 0;
  _elts.pop_back();
	table.erase(ans);
  heapifyDown(0);
	
  return ans;
}

/**
* find this object in the heap and return
 */
template<typename OBJ, class HashKey, class EqKey, class CmpKey>
OBJ heap2<OBJ, HashKey, EqKey, CmpKey>::find(OBJ val)
{
	if (!isIn(val))
		return OBJ();
	return table.find(val)->first;
}

/**
* Returns true if no items are in the heap2.
 */
template<typename OBJ, class HashKey, class EqKey, class CmpKey>
bool heap2<OBJ, HashKey, EqKey, CmpKey>::empty()
{
  return _elts.size() == 0;
}

///**
//* Verify that the heap is internally consistent. Fails assertion if not.
// */
//template<typename OBJ, class HashKey, class EqKey, class CmpKey>
//void heap2<OBJ, HashKey, EqKey, CmpKey>::verifyData()
//{
//	assert(_elts.size() == table.size());
//	heap2::IndexTable::iterator iter;
//	for (iter = table.begin(); iter != table.end(); iter++)
//	{
//		EqKey eq;
//		assert(eq(iter->first, _elts[iter->second])); 
//	}
//}

template<typename OBJ, class HashKey, class EqKey, class CmpKey>
void heap2<OBJ, HashKey, EqKey, CmpKey>::heapifyUp(unsigned int index)
{
  if (index == 0) return;
  int parent = (index-1)/2;
	CmpKey compare;

	if (compare(_elts[parent], _elts[index]))
	{
    OBJ tmp = _elts[parent];
    _elts[parent] = _elts[index];
    _elts[index] = tmp;
		table[_elts[parent]] = parent;
		table[_elts[index]] = index;
		EqKey eq;
		assert(!eq(_elts[parent], _elts[index]));
    heapifyUp(parent);
  }
}

template<typename OBJ, class HashKey, class EqKey, class CmpKey>
void heap2<OBJ, HashKey, EqKey, CmpKey>::heapifyDown(unsigned int index)
{
	CmpKey compare;
  unsigned int child1 = index*2+1;
  unsigned int child2 = index*2+2;
  int which;
	unsigned int count = _elts.size();
  // find smallest child
  if (child1 >= count)
    return;
  else if (child2 >= count)
    which = child1;
  //else if (fless(_elts[child1]->getKey(), _elts[child2]->getKey()))
	else if (!(compare(_elts[child1], _elts[child2])))
		which = child1;
  else
    which = child2;
	
  //if (fless(_elts[which]->getKey(), _elts[index]->getKey()))
	if (!(compare(_elts[which], _elts[index])))
	{
    OBJ tmp = _elts[which];
    _elts[which] = _elts[index];
		table[_elts[which]] = which;
    _elts[index] = tmp;
		table[_elts[index]] = index;
//    _elts[which]->key = which;
//    _elts[index]->key = index;
    heapifyDown(which);
  }
}

#endif
