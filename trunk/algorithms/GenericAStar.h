/*
 * $Id: GenericAStar.h,v 1.11 2007/04/05 23:35:42 yngvi Exp $
 *
 *  GenericAStar.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 10/30/06.
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


#ifndef GENERICSTAR_H
#define GENERICSTAR_H

#define __STDC_CONSTANT_MACROS
#include <stdint.h>
// this is defined in stdint.h, but it doesn't always get defined correctly
// even when __STDC_CONSTANT_MACROS is defined before including stdint.h
// because stdint might be included elsewhere first...
#ifndef UINT32_MAX
#define UINT32_MAX        4294967295U
#endif

#include "FPUtil.h"
#include <ext/hash_map>
#include "OpenClosedList.h"
#include "SearchEnvironment.h" // for the SearchEnvironment class


namespace GenericAStarUtil
{
	class SearchNode {
public:
		SearchNode(double _fCost=0, double _gCost=0, uint32_t curr=0, uint32_t prev=0)
		:fCost(_fCost), gCost(_gCost), currNode(curr), prevNode(prev) {}

		SearchNode(uint32_t curr)
		:fCost(0), gCost(0), currNode(curr), prevNode(0) {}

		double fCost;
		double gCost;
		uint32_t currNode;
		uint32_t prevNode;
	};
		
	struct SearchNodeEqual {
		bool operator()(const SearchNode &i1, const SearchNode &i2)
		{ return (i1.currNode == i2.currNode); } };
	
	struct SearchNodeCompare {
		bool operator()(const SearchNode &i1, const SearchNode &i2)
		{
			if (fequal(i1.fCost, i2.fCost))
			{
				return (fless(i1.gCost, i2.gCost));
			}
			return (fgreater(i1.fCost, i2.fCost));
		} };
		
	struct SearchNodeHash {
		size_t operator()(const SearchNode &x) const
		{ return (size_t)(x.currNode); }
	};
	
	typedef OpenClosedList<GenericAStarUtil::SearchNode, GenericAStarUtil::SearchNodeHash,
		GenericAStarUtil::SearchNodeEqual, GenericAStarUtil::SearchNodeCompare> PQueue;
	
	typedef __gnu_cxx::hash_map<uint32_t, GenericAStarUtil::SearchNode > NodeLookupTable;
	
	typedef __gnu_cxx::hash_map<uint32_t, bool > Corridor;
}


typedef GenericAStarUtil::NodeLookupTable::const_iterator closedList_iterator;

class GenericAStar {
public:
	GenericAStar() {}
	virtual ~GenericAStar() {}
	void getPath(SearchEnvironment *env, uint32_t from, uint32_t to,
							 std::vector<uint32_t> &thePath);

	bool initializeSearch(SearchEnvironment *env, uint32_t from, uint32_t to,
												std::vector<uint32_t> &thePath);
	bool doSingleSearchStep(std::vector<uint32_t> &thePath);
	uint32_t checkNextNode();
	void extractPathToStart(uint32_t n, std::vector<uint32_t> &thePath);

	virtual const char *getName();
	
	void printStats();
	long getNodesExpanded() { return nodesExpanded; }
	long getNodesTouched() { return nodesTouched; }
	void resetNodeCount() { nodesExpanded = nodesTouched = 0; }
	int getMemoryUsage();

	closedList_iterator getClosedListIter() const;
  uint32_t closedListIterNext(closedList_iterator&) const;

private:
		long nodesTouched, nodesExpanded;

	uint32_t getNextNode();
	void updateWeight(uint32_t currOpenNode, uint32_t neighbor);
	void addToOpenList(uint32_t currOpenNode, uint32_t neighbor);
	GenericAStarUtil::PQueue openQueue;
	GenericAStarUtil::NodeLookupTable closedList; //openList
	uint32_t goal, start;

	std::vector<uint32_t> neighbors;
	SearchEnvironment *env;
	GenericAStarUtil::Corridor eligibleNodes;
};

#endif
