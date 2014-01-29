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

#include <iostream>
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
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<action> &thePath);

	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; }
	void SetUseBDPathMax(bool val) { usePathMax = val; }
private:
	unsigned long long nodesExpanded, nodesTouched;
	
	double DoIteration(SearchEnvironment<state, action> *env,
					   state parent, state currState,
					   std::vector<state> &thePath, double bound, double g,
					   double maxH);
	double DoIteration(SearchEnvironment<state, action> *env,
					   action forbiddenAction, state &currState,
					   std::vector<action> &thePath, double bound, double g,
					   double maxH, double parentH);
	
	void UpdateNextBound(double currBound, double fCost);
	state goal;
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
	goal = to;
	thePath.push_back(from);
	while (true) //thePath.size() == 0)
	{
		nodeTable.clear();
		printf("Starting iteration with bound %f\n", nextBound);
		if (DoIteration(env, from, from, thePath, nextBound, 0, 0) == 0)
			break;
	}
}

template <class state, class action>
void IDAStar<state, action>::GetPath(SearchEnvironment<state, action> *env,
									 state from, state to,
									 std::vector<action> &thePath)
{
	nextBound = 0;
	nodesExpanded = nodesTouched = 0;
	thePath.resize(0);

	if (env->GoalTest(from, to))
		return;

	double rootH = env->HCost(from, to);
	UpdateNextBound(0, rootH);
	goal = to;
	std::vector<action> act;
	env->GetActions(from, act);
	while (thePath.size() == 0)
	{
		nodeTable.clear();
		printf("Starting iteration with bound %f; %llu expanded\n", nextBound, nodesExpanded);
		fflush(stdout);
		DoIteration(env, act[0], from, thePath, nextBound, 0, 0, rootH);
	}
}

template <class state, class action>
double IDAStar<state, action>::DoIteration(SearchEnvironment<state, action> *env,
										   state parent, state currState,
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
	if (env->GoalTest(currState, goal))
		return 0;
		
	std::vector<state> neighbors;
	env->GetSuccessors(currState, neighbors);
	nodesTouched += neighbors.size();
	
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		if (neighbors[x] == parent)
			continue;
		thePath.push_back(neighbors[x]);
		double edgeCost = env->GCost(currState, neighbors[x]);
		double childH = DoIteration(env, currState, neighbors[x], thePath, bound,
																g+edgeCost, maxH - edgeCost);
		if (env->GoalTest(thePath.back(), goal))
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
double IDAStar<state, action>::DoIteration(SearchEnvironment<state, action> *env,
										   action forbiddenAction, state &currState,
										   std::vector<action> &thePath, double bound, double g,
										   double maxH, double parentH)
{
	nodesExpanded++;
	double h = env->HCost(currState, goal, parentH);
	parentH = h;
	// path max
	if (usePathMax && fless(h, maxH))
		h = maxH;
	if (fgreater(g+h, bound))
	{
		UpdateNextBound(bound, g+h);
		//printf("Stopping at (%d, %d). g=%f h=%f\n", currState>>16, currState&0xFFFF, g, h);
		return h;
	}
	// must do this after we check the f-cost bound
	if (env->GoalTest(currState, goal))
		return -1; // found goal
	
	std::vector<action> actions;
	env->GetActions(currState, actions);
	nodesTouched += actions.size();
	int depth = thePath.size();
	
	for (unsigned int x = 0; x < actions.size(); x++)
	{
		if ((depth != 0) && (actions[x] == forbiddenAction))
			continue;

		thePath.push_back(actions[x]);

		double edgeCost = env->GCost(currState, actions[x]);
		env->ApplyAction(currState, actions[x]);
		action a = actions[x];
		env->InvertAction(a);
		double childH = DoIteration(env, a, currState, thePath, bound,
									g+edgeCost, maxH - edgeCost, parentH);
		env->UndoAction(currState, actions[x]);
		if (fequal(childH, -1)) // found goal
			return -1;

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
	{
		nextBound = fCost;
		//printf("Updating next bound to %f\n", nextBound);
	}
	else if (fgreater(fCost, currBound) && fless(fCost, nextBound))
	{
		nextBound = fCost;
		//printf("Updating next bound to %f\n", nextBound);
	}
}


#endif

//template <class state, class action>
//class SearchEnvironment {
//public:
//	virtual ~SearchEnvironment() {}
//	virtual void GetSuccessors(const state &nodeID, std::vector<state> &neighbors) = 0;
//	virtual void GetActions(const state &nodeID, std::vector<action> &actions) = 0;
//	virtual action GetAction(state &s1, state &s2) = 0;
//	virtual void ApplyAction(state &s, action a) = 0;
//	
//	virtual double HCost(state &node1, state &node2) = 0;
//	virtual double GCost(state &node1, state &node2) = 0;
//	virtual bool GoalTest(state &node, state &goal) = 0;
//	
//	virtual uint64_t GetStateHash(const state &node) = 0;
//	virtual uint64_t GetActionHash(action act) = 0;
//};
