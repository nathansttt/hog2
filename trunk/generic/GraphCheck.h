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
#include <vector>
#include <ext/hash_map>
#include "SearchEnvironment.h"
#include "UnitSimulation.h"
#include "Graph.h"

template <typename T>
class SimpleNode {
public:
	SimpleNode() 
	{
		depth = (T)0;
		me = (T)0;
		parent = (T)0; 
	}
	SimpleNode(T m, T p, int d) 
	{
		depth = d;
		me = m;
		parent = p;
	}

	T parent;
	T me;
	int depth;
};



struct Hash64 {
		size_t operator()(const uint64_t &x) const
		{ return (size_t)(x); }
};

template <typename state, typename action>
class GraphCheck {
public:
	static void NumNodesWithinRadius(SearchEnvironment<state, action> &env, state from, int depth, int &inner_count, int &leaf_count);
	static void PathCountWithinRadius(SearchEnvironment<state, action> &env, state from, int depth, __gnu_cxx::hash_map<uint64_t, int, Hash64> &counts, __gnu_cxx::hash_map<uint64_t, double, Hash64> &aveCosts );

private:
	static void DFSVisit(SearchEnvironment<state, action> &env, std::vector<SimpleNode<state> > &thePath, int depth, __gnu_cxx::hash_map<uint64_t, int, Hash64> &counts, __gnu_cxx::hash_map<uint64_t, double, Hash64> &aveCosts, double gval);

};

#endif
