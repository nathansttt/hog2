/*
 *  IDAStar.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/22/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef IDASTAR_H
#define IDASTAR_H

#include "SearchEnvironment.h"
#include <ext/hash_map>
#include "FPUtil.h"

typedef __gnu_cxx::hash_map<uint64_t, double> NodeHashTable;

template <class state, class action>
class IDAStar {
public:
	IDAStar() { useHashTable = usePathMax = false; }
	virtual ~IDAStar() {}
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
							 std::vector<state> &thePath);
	
	long GetNodesExpanded() { return nodesExpanded; }
	long GetNodesTouched() { return nodesTouched; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; }
	void SetUseBDPathMax(bool val) { usePathMax = val; }
private:
	unsigned long nodesExpanded, nodesTouched;
	
	double DoIteration(SearchEnvironment<state, action> *env,
										 state parent, state currState, state goal,
										 std::vector<state> &thePath, double bound, double g,
										 double maxH);
	void UpdateNextBound(double currBound, double fCost);
	double nextBound;
	NodeHashTable nodeTable;
	bool usePathMax;
	bool useHashTable;
};	

template <class state, class action>
void IDAStar<state, action>::GetPath(SearchEnvironment<state, action> *env,
																		 state from, state to,
																		 std::vector<state> &thePath)
{
	nextBound = 0;
	nodesExpanded = nodesTouched = 0;
	thePath.resize(0);
	UpdateNextBound(0, env->HCost(from, to));
	while (thePath.size() == 0)
	{
		nodeTable.clear();
		printf("Starting iteration with bound %f\n", nextBound);
		DoIteration(env, from, from, to, thePath, nextBound, 0, 0);
	}
}

template <class state, class action>
double IDAStar<state, action>::DoIteration(SearchEnvironment<state, action> *env,
																					 state parent, state currState, state goal,
																					 std::vector<state> &thePath, double bound, double g,
																					 double maxH)
{
	nodesExpanded++;
	double h = env->HCost(currState, goal);
	
	// path max
	if (usePathMax && fless(h, maxH))
		h = maxH;
	if (fgreater(g+h, bound))
	{
		UpdateNextBound(bound, g+h);
		//printf("Stopping at (%d, %d). g=%f h=%f\n", currState>>16, currState&0xFFFF, g, h);
		return h;
	}
// FIXME -- hash table
//	if (nodeTable.find(currState) != nodeTable.end()) // already seen
//	{
//		if (fless(g/*+h*/, nodeTable[currState])) // with lower g-cost
//		{
//		}
//		else {
//			return h;
//		}
//	}
//	nodeTable[currState] = g;//+h;
		
	std::vector<state> neighbors;
	env->GetSuccessors(currState, neighbors);
	nodesTouched += neighbors.size();
	
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		if (neighbors[x] == parent)
			continue;
		thePath.push_back(neighbors[x]);
		double edgeCost = env->GCost(currState, neighbors[x]);
		double childH = DoIteration(env, currState, neighbors[x], goal, thePath, bound,
																g+edgeCost, maxH - edgeCost);
		if (thePath.back() == goal)
			return 0;
		thePath.pop_back();
		// pathmax
		if (usePathMax && fgreater(childH-edgeCost, h))
		{
//			nodeTable[currState] = g;//+h
			h = childH-edgeCost;
			if (fgreater(g+h, bound))
			{
				UpdateNextBound(bound, g+h);
				return h;
			}
		}
	}
	return h;
}

template <class state, class action>
void IDAStar<state, action>::UpdateNextBound(double currBound, double fCost)
{
	if (!fgreater(nextBound, currBound))
		nextBound = fCost;
	else if (fgreater(fCost, currBound) && fless(fCost, nextBound))
		nextBound = fCost;
}


#endif

//template <class state, class action>
//class SearchEnvironment {
//public:
//	virtual ~SearchEnvironment() {}
//	virtual void GetSuccessors(state &nodeID, std::vector<state> &neighbors) = 0;
//	virtual void GetActions(state &nodeID, std::vector<action> &actions) = 0;
//	virtual action GetAction(state &s1, state &s2) = 0;
//	virtual void ApplyAction(state &s, action a) = 0;
//	
//	virtual double HCost(state &node1, state &node2) = 0;
//	virtual double GCost(state &node1, state &node2) = 0;
//	virtual bool GoalTest(state &node, state &goal) = 0;
//	
//	virtual uint64_t GetStateHash(state &node) = 0;
//	virtual uint64_t GetActionHash(action act) = 0;
//};
