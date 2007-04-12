/*
 *  GenericIDAStar.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 1/3/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */


#ifndef GENERICIDASTAR_H
#define GENERICIDASTAR_H

#include <ext/hash_map>
#include "SearchEnvironment.h" // for the SearchEnvironment class

typedef __gnu_cxx::hash_map<uint32_t, double> NodeHashTable;

class GenericIDAStar {
public:
	GenericIDAStar() {}
	virtual ~GenericIDAStar() {}
	void getPath(SearchEnvironment *env, uint32_t from, uint32_t to,
							 std::vector<uint32_t> &thePath);

	long getNodesExpanded() { return nodesExpanded; }
	long getNodesTouched() { return nodesTouched; }
	void resetNodeCount() { nodesExpanded = nodesTouched = 0; }
	void setUseBDPathMax(bool val) { usePathMax = val; }
private:
	unsigned long nodesExpanded, nodesTouched;

	double doIteration(SearchEnvironment *env,
										 uint32_t parent, uint32_t currState, uint32_t goal,
										 std::vector<uint32_t> &thePath, double bound, double g,
										 double maxH);
	void updateNextBound(double currBound, double fCost);
	double nextBound;
	NodeHashTable nodeTable;
	bool usePathMax;
};	

#endif
