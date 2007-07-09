/*
 *  GraphCheck.h
 *  hog2
 *
 *  Created by Zhifu Zhang on 7/7/07.
 *
 */

#ifndef GRAPHCHECK_H
#define GRAPHCHECK_H

#include <stdint.h>
#include <ext/hash_map>
#include "SearchEnvironment.h"
#include "UnitSimulation.h"
#include "Graph.h"

class SimpleNode<state> {
public:
	SimpleNode<state>() 
	{
		depth = 0;
		me = 0;
		parent = 0; 
	}
	SimpleNode<state>(state m, state p, int d) 
	{
		depth = d;
		me = m;
		parent = p;
	}

	state parent;
	state me;
	int depth;
};

struct Hash64 {
		size_t operator()(const uint64_t &x) const
		{ return (size_t)(x); }
};

class GraphCheck<state, action> {
public:
	static void NumNodesWithinRadius(SearchEnvironment<state, action> &env, graphState from, int depth, int &inner_count, int &leaf_count);
	static void PathCountWithinRadius(SearchEnvironment<state, action> &env, graphState from, int depth, __gnu_cxx::hash_map<uint64_t, int, Hash64> &counts, __gnu_cxx::hash_map<uint64_t, double, Hash64> &aveCosts );

private:
	static void DFSVisit(SearchEnvironment<state, action> &env, std::vector<SimpleNode> &thePath, int depth, __gnu_cxx::hash_map<uint64_t, int, Hash64> &counts, __gnu_cxx::hash_map<uint64_t, double, Hash64> &aveCosts, double gval);

};

#endif
