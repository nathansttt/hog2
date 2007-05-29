/*
 *  MeroB.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/29/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef MEROB_H
#define MEROB_H

#include "SearchEnvironment.h"
#include "GraphEnvironment.h"
#include <ext/hash_map>
#include "FPUtil.h"

typedef __gnu_cxx::hash_map<uint64_t, double> NodeHashTable;

class MeroB {
public:
	MeroB() { }
	virtual ~MeroB() {}
	void GetPath(GraphEnvironment *env, graphState from, graphState to, std::vector<graphState> &thePath);
	
	long GetNodesExpanded() { return nodesExpanded; }
	long GetNodesTouched() { return nodesTouched; }
private:
	long nodesExpanded, nodesTouched;
};	

#endif
