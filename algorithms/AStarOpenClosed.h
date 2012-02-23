/*
 * $Id: AStarOpenClosed.h,v 1.9 2006/11/27 23:21:37 nathanst Exp $
 *
 *  AStarOpenClosed.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/25/09.
 *  Copyright 2009 NS Software. All rights reserved.
 *
 * This file is part of HOG2.
 *
 * HOG2 is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifndef ASTAROPENCLOSED_H
#define ASTAROPENCLOSED_H

#include <cassert>
#include <vector>
#include <ext/hash_map>
#include <stdint.h>

struct AHash64 {
	size_t operator()(const uint64_t &x) const
	{ return (size_t)(x); }
};

enum dataLocation {
	kOpenList,
	kClosedList,
	kNotFound
};

const uint64_t kTAStarNoNode = 0xFFFFFFFFFFFFFFFFull;

template<typename state>
class AStarOpenClosedData {
public:
	AStarOpenClosedData() {}
	AStarOpenClosedData(const state &theData, double gCost, double hCost, uint64_t parent, uint64_t openLoc, dataLocation location)
	:data(theData), g(gCost), h(hCost), parentID(parent), openLocation(openLoc), where(location) { reopened = false; }
	state data;
	double g;
	double h;
	uint64_t parentID;
	uint64_t openLocation;
	bool reopened;
	dataLocation where;
};

template<typename state, typename CmpKey, class dataStructure = AStarOpenClosedData<state> >
class AStarOpenClosed {
public:
	AStarOpenClosed();
	~AStarOpenClosed();
	void Reset();
	uint64_t AddOpenNode(const state &val, uint64_t hash, double g, double h, uint64_t parent=kTAStarNoNode);
	uint64_t AddClosedNode(state &val, uint64_t hash, double g, double h, uint64_t parent=kTAStarNoNode);
	void KeyChanged(uint64_t objKey);
	//void IncreaseKey(uint64_t objKey);
	dataLocation Lookup(uint64_t hashKey, uint64_t &objKey) const;
	inline dataStructure &Lookup(uint64_t objKey) { return elements[objKey]; }
	inline const dataStructure &Lookat(uint64_t objKey) const { return elements[objKey]; }
	uint64_t Peek() const;
	uint64_t Close();
	void Reopen(uint64_t objKey);

	uint64_t GetOpenItem(unsigned int which) { return theHeap[which]; }
	size_t OpenSize() const { return theHeap.size(); }
	size_t ClosedSize() const { return size()-OpenSize(); }
	size_t size() const { return elements.size(); }
	//	void verifyData();
private:
	bool HeapifyUp(unsigned int index);
	void HeapifyDown(unsigned int index);

	std::vector<uint64_t> theHeap;
	// storing the element id; looking up with...hash?
	typedef __gnu_cxx::hash_map<uint64_t, uint64_t, AHash64> IndexTable;
	IndexTable table;
	std::vector<dataStructure > elements;
};


template<typename state, typename CmpKey, class dataStructure>
AStarOpenClosed<state, CmpKey, dataStructure>::AStarOpenClosed()
{
}

template<typename state, typename CmpKey, class dataStructure>
AStarOpenClosed<state, CmpKey, dataStructure>::~AStarOpenClosed()
{
}

/**
 * Remove all objects from queue.
 */
template<typename state, typename CmpKey, class dataStructure>
void AStarOpenClosed<state, CmpKey, dataStructure>::Reset()
{
	table.clear();
	elements.clear();
	theHeap.resize(0);
}

/**
 * Add object into open list.
 */
template<typename state, typename CmpKey, class dataStructure>
uint64_t AStarOpenClosed<state, CmpKey, dataStructure>::AddOpenNode(const state &val, uint64_t hash, double g, double h, uint64_t parent)
{
	// should do lookup here...
	if (table.find(hash) != table.end())
	{
		//return -1; // TODO: find correct id and return
		assert(false);
	}
	elements.push_back(dataStructure(val, g, h, parent, theHeap.size(), kOpenList));
	if (parent == kTAStarNoNode)
		elements.back().parentID = elements.size()-1;
	table[hash] = elements.size()-1; // hashing to element list location
	theHeap.push_back(elements.size()-1); // adding element id to back of heap
	HeapifyUp(theHeap.size()-1); // heapify from back of the heap
	return elements.size()-1;
}

/**
 * Add object into closed list.
 */
template<typename state, typename CmpKey, class dataStructure>
uint64_t AStarOpenClosed<state, CmpKey, dataStructure>::AddClosedNode(state &val, uint64_t hash, double g, double h, uint64_t parent)
{
	// should do lookup here...
	assert(table.find(hash) == table.end());
	elements.push_back(dataStructure(val, g, h, parent, 0, kClosedList));
	if (parent == kTAStarNoNode)
		elements.back().parentID = elements.size()-1;
	table[hash] = elements.size(); // hashing to element list location
	return elements.size()-1;
}

/**
 * Indicate that the key for a particular object has changed.
 */
template<typename state, typename CmpKey, class dataStructure>
void AStarOpenClosed<state, CmpKey, dataStructure>::KeyChanged(uint64_t val)
{
//	EqKey eq;
//	assert(eq(theHeap[table[val]], val));
//	//HeapifyUp(val->key);
//	theHeap[table[val]] = val;
	if (!HeapifyUp(elements[val].openLocation))
		HeapifyDown(elements[val].openLocation);
}

///**
// * Indicate that the key for a particular object has increased.
// */
//template<typename state, typename CmpKey, class dataStructure>
//void AStarOpenClosed<state, CmpKey, dataStructure>::IncreaseKey(uint64_t val)
//{
////	EqKey eq;
////	assert(eq(theHeap[table[val]], val));
////	theHeap[table[val]] = val;
//	HeapifyDown(val);
//}

/**
 * Returns location of object as well as object key.
 */
template<typename state, typename CmpKey, class dataStructure>
dataLocation AStarOpenClosed<state, CmpKey, dataStructure>::Lookup(uint64_t hashKey, uint64_t &objKey) const
{
	typename IndexTable::const_iterator it;
	it = table.find(hashKey);
	if (it != table.end())
	{
		objKey = (*it).second;
		return elements[objKey].where;
	}
	return kNotFound;
}


/**
 * Peek at the next item to be expanded.
 */
template<typename state, typename CmpKey, class dataStructure>
uint64_t AStarOpenClosed<state, CmpKey, dataStructure>::Peek() const
{
	assert(OpenSize() != 0);
	
	return theHeap[0];
}

/**
 * Move the best item to the closed list and return key.
 */
template<typename state, typename CmpKey, class dataStructure>
uint64_t AStarOpenClosed<state, CmpKey, dataStructure>::Close()
{
	assert(OpenSize() != 0);

	uint64_t ans = theHeap[0];
	elements[ans].where = kClosedList;
	theHeap[0] = theHeap[theHeap.size()-1];
	theHeap.pop_back();
	HeapifyDown(0);
	
	return ans;
}

/**
 * Move item off the closed list and back onto the open list.
 */
template<typename state, typename CmpKey, class dataStructure>
void AStarOpenClosed<state, CmpKey, dataStructure>::Reopen(uint64_t objKey)
{
	assert(elements[objKey].where == kClosedList);
	elements[objKey].reopened = true;
	elements[objKey].where = kOpenList;
	elements[objKey].openLocation = theHeap.size();
	theHeap.push_back(objKey);
	HeapifyUp(theHeap.size()-1);
}


///**
// * find this object in the Heap and return
// */
//template<typename state, typename CmpKey, class dataStructure>
//OBJ AStarOpenClosed<state, CmpKey, dataStructure>::find(OBJ val)
//{
//	if (!IsIn(val))
//		return OBJ();
//    return theHeap[table[val]];
//}

///**
// * Returns true if no items are in the AStarOpenClosed.
// */
//template<typename state, typename CmpKey, class dataStructure>
//bool AStarOpenClosed<state, CmpKey, dataStructure>::Empty()
//{
//	return theHeap.size() == 0;
//}

///**
//* Verify that the Heap is internally consistent. Fails assertion if not.
// */
//template<typename state, typename CmpKey, class dataStructure>
//void AStarOpenClosed<state, CmpKey, dataStructure>::verifyData()
//{
//	assert(theHeap.size() == table.size());
//	AStarOpenClosed::IndexTable::iterator iter;
//	for (iter = table.begin(); iter != table.end(); iter++)
//	{
//		EqKey eq;
//		assert(eq(iter->first, theHeap[iter->second])); 
//	}
//}

/**
 * Moves a node up the heap. Returns true if the node was moved, false otherwise.
 */
template<typename state, typename CmpKey, class dataStructure>
bool AStarOpenClosed<state, CmpKey, dataStructure>::HeapifyUp(unsigned int index)
{
	if (index == 0) return false;
	int parent = (index-1)/2;
	CmpKey compare;
	
	if (compare(elements[theHeap[parent]], elements[theHeap[index]]))
	{
		unsigned int tmp = theHeap[parent];
		theHeap[parent] = theHeap[index];
		theHeap[index] = tmp;
		elements[theHeap[parent]].openLocation = parent;
		elements[theHeap[index]].openLocation = index;
		HeapifyUp(parent);
		return true;
	}
	return false;
}

template<typename state, typename CmpKey, class dataStructure>
void AStarOpenClosed<state, CmpKey, dataStructure>::HeapifyDown(unsigned int index)
{
	CmpKey compare;
	unsigned int child1 = index*2+1;
	unsigned int child2 = index*2+2;
	int which;
	unsigned int count = theHeap.size();
	// find smallest child
	if (child1 >= count)
		return;
	else if (child2 >= count)
		which = child1;
	else if (!(compare(elements[theHeap[child1]], elements[theHeap[child2]])))
		which = child1;
	else
		which = child2;
	
	//if (fless(theHeap[which]->GetKey(), theHeap[index]->GetKey()))
	if (!(compare(elements[theHeap[which]], elements[theHeap[index]])))
	{
		unsigned int tmp = theHeap[which];
		theHeap[which] = theHeap[index];
//		table[theHeap[which]] = which;
		theHeap[index] = tmp;
		elements[theHeap[which]].openLocation = which;
		elements[theHeap[index]].openLocation = index;
		//		table[theHeap[index]] = index;
		//    theHeap[which]->key = which;
		//    theHeap[index]->key = index;
		HeapifyDown(which);
	}
}

#endif
