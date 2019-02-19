//
//  BID.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/28/18.
//  Copyright © 2018 University of Denver. All rights reserved.
//

#ifndef BID_h
#define BID_h

#include "vectorCache.h"

template <class state, class action>
class BID {
public:
	BID(double minGrow, double maxGrow, double expEpsilon) :c1(minGrow), c2(maxGrow), initialGap(expEpsilon) {}
//	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
//				 std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<action> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, Heuristic<state> *heuristic, state from, state to,
				 std::vector<action> &thePath);
	void RedoMinWork();
	
	uint64_t GetNodesExpanded() { return totalNodesExpanded; }
	uint64_t GetNodesTouched() { return totalNodesTouched; }
	void ResetNodeCount() { totalNodesExpanded = totalNodesTouched = 0; }
private:
	struct searchData {
		double f;
		double nextF;
		double failedF;
		uint64_t nodes;
	};
	BID<state, action>::searchData BinarySearch(searchData d, uint64_t nodeLimit);
	BID<state, action>::searchData ExponentialSearch(const searchData &d, uint64_t nodeLimit);
	BID<state, action>::searchData DFBNB(double costLimit, uint64_t nodeLimit);
	BID<state, action>::searchData DFBNBHelper(state &currState, double pathCost, double costLimit, searchData &sd,
						 uint64_t nodeLimit, action forbidden);

	uint64_t totalNodesExpanded, totalNodesTouched;
	Heuristic<state> *h;
	SearchEnvironment<state, action> *env;
	std::vector<action> solutionPath, currPath;
	double solutionCost;
	vectorCache<action> actCache;
	state start, goal;
	double c1, c2;
	double initialGap;
};

template <class state, class action>
void BID<state, action>::GetPath(SearchEnvironment<state, action> *env, state from, state to,
								 std::vector<action> &thePath)
{
	GetPath(env, env, from, to, thePath);
}

template <class state, class action>
void BID<state, action>::GetPath(SearchEnvironment<state, action> *env, Heuristic<state> *heuristic, state from, state to,
								 std::vector<action> &thePath)
{
	this->env = env;
	h = heuristic;
	start = from;
	goal = to;
	solutionPath.clear();
	solutionCost = DBL_MAX;
	ResetNodeCount();
	
	printf("EBIS: Baseline search: f: %1.2f\n", h->HCost(from, to));
	searchData base = DFBNB(h->HCost(from, to), -1);
	while (solutionCost > base.f)
	{
		printf("EBIS: First search: f: %1.2f target: (%llu, %llu]\n",
			   base.nextF, uint64_t(c1*base.nodes), uint64_t(c2*base.nodes));
		searchData curr = DFBNB(base.nextF, -1);
		if (solutionPath.size() != 0)
		{ thePath = solutionPath; return; }

		// Didn't reach the node expansions bound
		if (curr.nodes < uint64_t(c1*base.nodes))
		{
			printf("EBIS: %llu under target %llu, initiate EXP search\n", curr.nodes, uint64_t(c1*base.nodes));
			curr = ExponentialSearch(curr, base.nodes);

			// Overshot the node expansions bound
			if (curr.nodes >= uint64_t(c2*base.nodes))
			{
				printf("EBIS: %llu over target %llu, initiate BIN search\n", curr.nodes, uint64_t(c2*base.nodes));
				curr = BinarySearch(curr, base.nodes);
			}
		}
		if (solutionPath.size() != 0)
		{ thePath = solutionPath; return; }
		base = curr;
	}
}

template <class state, class action>
typename BID<state, action>::searchData BID<state, action>::ExponentialSearch(const searchData &d, uint64_t nodeLimit)
{
	uint64_t lowNodes = c1*nodeLimit;
	uint64_t highNodes = c2*nodeLimit;
	double delta = std::max(d.nextF - d.f, initialGap);
	int i = 0;
	searchData lastSuccess = d;
	while (true)
	{
		double bound = d.f + delta*pow(2, i);
		searchData curr = DFBNB(bound, c2*nodeLimit);
		
		if (lowNodes <= curr.nodes && curr.nodes < highNodes)
		{
			printf("EBIS:->EXP: in node window [%llu <= %llu < %llu]\n", lowNodes, curr.nodes, highNodes);
			return curr;
		}
		else if (curr.nodes >= highNodes)
		{
			printf("EBIS:->EXP: over node window\n");
			lastSuccess.nodes = curr.nodes;
			lastSuccess.failedF = bound;
			return lastSuccess;
		}
		else {
			if (solutionPath.size() != 0)
			{
				printf("EBIS:->EXP: Solution proven below window; done\n");
				return curr;
			}
			printf("EBIS:->EXP: below node window\n");
			lastSuccess = curr;
		}
		i++;
		if (d.f + delta*pow(2, i) < curr.nextF)
			printf("EBIS:->EXP: delta too small, increasing growth past nextf\n");
		while (d.f + delta*pow(2, i) < curr.nextF)
		{
			i++;
		}
	}
}

template <class state, class action>
typename BID<state, action>::searchData BID<state, action>::BinarySearch(searchData d, uint64_t nodeLimit)
{
	uint64_t lowNodes = c1*nodeLimit;
	uint64_t highNodes = c2*nodeLimit;
	double middlef = (d.failedF + d.f)/2.0;
	searchData curr;
	if (middlef <= d.nextF)
	{
		curr = DFBNB(d.nextF, -1);
		if (curr.nodes >= lowNodes)
			return curr;
	}
	else {
		curr = DFBNB(middlef, highNodes);
	}
	// found and proved solution
	if (solutionCost <= middlef && curr.nodes < highNodes)
		return curr;
	
	if (lowNodes <= curr.nodes && curr.nodes < highNodes)
	{
		printf("EBIS:->BIN: in node window [%llu <= %llu < %llu]\n", lowNodes, curr.nodes, highNodes);
		return curr;
	}
	else if (curr.nodes >= highNodes)
	{
		printf("EBIS:->BIN: over node window\n");
		d.failedF = std::min(middlef, solutionCost);
		return BinarySearch(d, nodeLimit);
	}
	else {
		printf("EBIS:->BIN: below node window\n");
		curr.failedF = d.failedF;
		return BinarySearch(curr, nodeLimit);
	}
}


template <class state, class action>
typename BID<state, action>::searchData BID<state, action>::DFBNB(double costLimit, uint64_t nodeLimit)
{
	state currState = start;
	if (nodeLimit != -1)
		printf("    --+DFBnB f: %1.3f; nodes: %llu; ", costLimit, nodeLimit);
	else
		printf("    --+DFBnB f: %1.3f; nodes: ∞; ", costLimit);
	currPath.clear();
	searchData sd;
	sd.nextF = DBL_MAX;
	sd.f = 0;
	sd.nodes = 0;
	action a;
	sd = DFBNBHelper(currState, 0, costLimit, sd, nodeLimit, a);
	totalNodesExpanded += sd.nodes;
	if (sd.nextF == DBL_MAX) // so few nodes expanded we didn't find the next bound
	{
		printf(" (oops) ");
		sd = DFBNBHelper(currState, 0, costLimit, sd, -1, a);
		totalNodesExpanded += sd.nodes;
	}
	printf("%llu (new) %llu (total), maxf: %f, nextf: %f\n", sd.nodes, totalNodesExpanded, sd.f, sd.nextF);
	return sd;
}

template <class state, class action>
typename BID<state, action>::searchData BID<state, action>::DFBNBHelper(state &currState, double pathCost, double costLimit,
										 searchData &sd, uint64_t nodeLimit, action forbidden)
{
	double currF = pathCost+env->HCost(currState, goal);
//	printf("-------->%f [%f]\n", currF, pathCost);
	if (fgreater(currF, costLimit) || fgreatereq(currF, solutionCost))
	{
		sd.nextF = std::min(sd.nextF, currF);
		return sd;
	}
	else {
		sd.f = std::max(currF, sd.f);
	}
	
	if (sd.nodes >= nodeLimit)
		return sd;
	if (env->GoalTest(currState, goal))
	{
		if (fless(currF, solutionCost))
		{
			solutionPath = currPath;
			solutionCost = currF;
		}
		return sd;
	}
	
	// TODO: cache these for later use
	std::vector<action> &acts = *actCache.getItem();
	env->GetActions(currState, acts);
	sd.nodes++;
	totalNodesTouched+=acts.size();

	for (int x = 0; x < acts.size(); x++)
	{
		if (acts[x] == forbidden && currPath.size() > 0)
			continue;
		double edgeCost = env->GCost(currState, acts[x]);
		env->ApplyAction(currState, acts[x]);
		currPath.push_back(acts[x]);
		action rev = acts[x];
		env->InvertAction(rev);
		sd = DFBNBHelper(currState, pathCost+edgeCost, costLimit, sd, nodeLimit, rev);
		currPath.pop_back();
		env->UndoAction(currState, acts[x]);
	}
	actCache.returnItem(&acts);
	return sd;
}

template <class state, class action>
void BID<state, action>::RedoMinWork()
{
	ResetNodeCount();
	DFBNB(solutionCost, -1);
}

#endif /* BID_h */

