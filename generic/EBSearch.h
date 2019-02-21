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
#include "AStarOpenClosed.h"

template <class state>
struct BFHSCompare {
	bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
	{
		return (fgreater(i1.g, i2.g));
	}
};


template <class state, class action, class environment, bool DFS = true, uint64_t costScale = 1>
class EBSearch {
public:
	EBSearch(double minGrow, double maxGrow, double expEpsilon) :c1(minGrow), c2(maxGrow), initialGap(expEpsilon) {}
	void GetPath(environment *env, state from, state to,
				 std::vector<action> &thePath);
	void GetPath(environment *env, Heuristic<state> *heuristic, state from, state to,
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
	EBSearch<state, action, environment, DFS, costScale>::searchData LowLevelSearch(double costLimit, uint64_t nodeLimit);
	EBSearch<state, action, environment, DFS, costScale>::searchData BinarySearch(searchData d, uint64_t nodeLimit);
	EBSearch<state, action, environment, DFS, costScale>::searchData ExponentialSearch(const searchData &d, uint64_t nodeLimit);

	// Functions for DF Search
	EBSearch<state, action, environment, DFS, costScale>::searchData DFBNB(double costLimit, uint64_t nodeLimit);
	EBSearch<state, action, environment, DFS, costScale>::searchData DFBNBHelper(state &currState, double pathCost, double costLimit,
													searchData &sd, uint64_t nodeLimit, action forbidden);

	// Functions for BFHS
	EBSearch<state, action, environment, DFS, costScale>::searchData BFHS(double costLimit, uint64_t nodeLimit);
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath);

	
	
	uint64_t totalNodesExpanded, totalNodesTouched;
	Heuristic<state> *h;
	environment *env;
	std::vector<action> solutionPath, currPath;
	double solutionCost;
	vectorCache<action> actCache;
	state start, goal;
	double c1, c2;
	double initialGap;

	// Data for BFHS
	AStarOpenClosed<state, BFHSCompare<state>> q;
	std::vector<state> neighbors;
	std::vector<uint64_t> neighborID;
	std::vector<double> edgeCosts;
	std::vector<dataLocation> neighborLoc;
	std::vector<state> solutionStates;
};

template <class state, class action, class environment, bool DFS, uint64_t costScale>
void EBSearch<state, action, environment, DFS, costScale>::GetPath(environment *env, state from, state to,
								 std::vector<action> &thePath)
{
	GetPath(env, env, from, to, thePath);
}

template <class state, class action, class environment, bool DFS, uint64_t costScale>
void EBSearch<state, action, environment, DFS, costScale>::GetPath(environment *env, Heuristic<state> *heuristic, state from, state to,
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
	searchData base = LowLevelSearch(h->HCost(from, to), -1);
	while (solutionCost > base.f)
	{
		printf("EBIS: First search: f: %1.2f target: (%llu, %llu]\n",
			   base.nextF, uint64_t(c1*base.nodes), uint64_t(c2*base.nodes));
		searchData curr = LowLevelSearch(base.nextF, -1);
		if (solutionCost <= base.nextF)
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
		if (solutionCost <= curr.f)
		{ thePath = solutionPath; return; }
		base = curr;
	}
}

template <class state, class action, class environment, bool DFS, uint64_t costScale>
typename EBSearch<state, action, environment, DFS, costScale>::searchData EBSearch<state, action, environment, DFS, costScale>::ExponentialSearch(const searchData &d, uint64_t nodeLimit)
{
	uint64_t lowNodes = c1*nodeLimit;
	uint64_t highNodes = c2*nodeLimit;
	double delta = std::max(d.nextF - d.f, initialGap);
	int i = 0;
	searchData lastSuccess = d;
	while (true)
	{
		double bound = d.f + delta*pow(2, i);
		searchData curr = LowLevelSearch(bound, highNodes);
		if (solutionCost <= bound && curr.nodes < highNodes)
		{
			printf("***EBIS:->EXP: Solution proven below window; done\n");
			curr.f = solutionCost;
			return curr;
		}

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

template <class state, class action, class environment, bool DFS, uint64_t costScale>
typename EBSearch<state, action, environment, DFS, costScale>::searchData EBSearch<state, action, environment, DFS, costScale>::BinarySearch(searchData d, uint64_t nodeLimit)
{
	uint64_t lowNodes = c1*nodeLimit;
	uint64_t highNodes = c2*nodeLimit;
	double middlef = (d.failedF + d.f)/2.0;
	searchData curr;
	if (middlef <= d.nextF)
	{
		middlef = d.nextF;
		curr = LowLevelSearch(middlef, -1);
		if (solutionCost <= middlef)
			curr.f = middlef;
		if (curr.nodes >= lowNodes)
			return curr;
	}
	else {
		curr = LowLevelSearch(middlef, highNodes);
	}
	// found and proved solution
	if (solutionCost <= middlef && curr.nodes < highNodes)
	{
		curr.f = middlef;
		return curr;
	}
	
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

template <class state, class action, class environment, bool DFS, uint64_t costScale>
typename EBSearch<state, action, environment, DFS, costScale>::searchData EBSearch<state, action, environment, DFS, costScale>::DFBNB(double costLimit, uint64_t nodeLimit)
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

template <class state, class action, class environment, bool DFS, uint64_t costScale>
typename EBSearch<state, action, environment, DFS, costScale>::searchData EBSearch<state, action, environment, DFS, costScale>::DFBNBHelper(state &currState, double pathCost, double costLimit,
										 searchData &sd, uint64_t nodeLimit, action forbidden)
{
	double currF = pathCost+h->HCost(currState, goal);
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

template <class state, class action, class environment, bool DFS, uint64_t costScale>
void EBSearch<state, action, environment, DFS, costScale>::RedoMinWork()
{
	ResetNodeCount();
	printf("EBIS Validation:\n");
	LowLevelSearch(solutionCost, -1);
}

template <class state, class action, class environment, bool DFS, uint64_t costScale>
typename EBSearch<state, action, environment, DFS, costScale>::searchData EBSearch<state, action, environment, DFS, costScale>::BFHS(double costLimit, uint64_t nodeLimit)
{
	if (nodeLimit == -1 && costLimit == DBL_MAX)
		printf("    --+BFHS f: ∞; nodes: ∞; ", costLimit, nodeLimit);
	else if (nodeLimit == -1)
		printf("    --+BFHS f: %1.3f; nodes: ∞; ", costLimit);
	else
		printf("    --+BFHS f: %1.3f; nodes: %llu; ", costLimit, nodeLimit);

	q.Reset(env->GetMaxHash());
	// put start in open
	q.AddOpenNode(start, env->GetStateHash(start), 0, 0);
	double nextCost = DBL_MAX, maxFExpanded = 0;
	uint64_t nodesExpanded = 0;
	while (nodesExpanded < nodeLimit && q.OpenSize() > 0)
	{
		// expand by low g until
		// (1) we find the goal
		// (2) we hit the node limit
		// (3) we exhaust all states
		uint64_t nodeid = q.Close();
		nodesExpanded++;
		totalNodesExpanded++;
	
		maxFExpanded = std::max(maxFExpanded, q.Lookup(nodeid).g+h->HCost(q.Lookup(nodeid).data, goal));
		
		if (env->GoalTest(q.Lookup(nodeid).data, goal))
		{
			solutionCost = q.Lookup(nodeid).g;
			ExtractPathToStartFromID(nodeid, solutionStates);
			// Path is backwards - reverse
			reverse(solutionStates.begin(), solutionStates.end());
			// f, nextF, failedF, nodes
			for (int x = 0; x < solutionStates.size()-1; x++)
			{
				solutionPath.push_back(env->GetAction(solutionStates[x], solutionStates[x+1]));
			}
			return {q.Lookup(nodeid).g, q.Lookup(nodeid).g, -1, nodesExpanded};
		}
		
		neighbors.resize(0);
		edgeCosts.resize(0);
		neighborID.resize(0);
		neighborLoc.resize(0);
		
		//	std::cout << "Expanding: " << q.Lookup(nodeid).data << " with f:";
		//	std::cout << q.Lookup(nodeid).g << std::endl;
		
		env->GetSuccessors(q.Lookup(nodeid).data, neighbors);

		// 1. load all the children
		for (unsigned int x = 0; x < neighbors.size(); x++)
		{
			uint64_t theID;
			neighborLoc.push_back(q.Lookup(env->GetStateHash(neighbors[x]), theID));
			neighborID.push_back(theID);
			edgeCosts.push_back(env->GCost(q.Lookup(nodeid).data, neighbors[x]));
		}
		
		// iterate again updating costs and writing out to memory
		for (int x = 0; x < neighbors.size(); x++)
		{
			totalNodesTouched++;
			
			switch (neighborLoc[x])
			{
				case kClosedList:
					break;
				case kOpenList:
					if (fless(q.Lookup(nodeid).g+edgeCosts[x], q.Lookup(neighborID[x]).g))
					{
						q.Lookup(neighborID[x]).parentID = nodeid;
						q.Lookup(neighborID[x]).g = q.Lookup(nodeid).g+edgeCosts[x];
						q.KeyChanged(neighborID[x]);
					}
					break;
				case kNotFound:
				{
					double fcost = q.Lookup(nodeid).g+edgeCosts[x]+h->HCost(neighbors[x], goal);
					// TODO: don't add states with g == best solution when one exists
					if (fcost > costLimit)
					{
						nextCost = std::min(nextCost, fcost);
					}
					else {
						q.AddOpenNode(neighbors[x],
								  env->GetStateHash(neighbors[x]),
								  q.Lookup(nodeid).g+edgeCosts[x],
								  0,
								  nodeid);
					}
				}
			}
		}
	}
	// f, nextF, failedF, nodes

	if (nextCost == DBL_MAX)
		printf("%llu (new) %llu (total), maxf: %f, nextf: ∞\n", nodesExpanded, totalNodesExpanded, maxFExpanded);
	else
		printf("%llu (new) %llu (total), maxf: %f, nextf: %f\n", nodesExpanded, totalNodesExpanded, maxFExpanded, nextCost);

	// Exceeded node limit
	if (nodesExpanded == nodeLimit)
	{
		return {maxFExpanded, nextCost, maxFExpanded, nodesExpanded};
	}
	// expanded all nodes in limit
	if (q.OpenSize()==0)
	{
		return {maxFExpanded, nextCost, -1, nodesExpanded};
	}
	assert(!"Shouldn't get here");
}

template <class state, class action,class environment,bool DFS, uint64_t costScale>
void EBSearch<state, action, environment, DFS, costScale>::ExtractPathToStartFromID(uint64_t node,
																	   std::vector<state> &thePath)
{
	thePath.clear();
	do {
		thePath.push_back(q.Lookup(node).data);
		node = q.Lookup(node).parentID;
	} while (q.Lookup(node).parentID != node);
	thePath.push_back(q.Lookup(node).data);
}


template <class state, class action, class environment, bool DFS, uint64_t costScale>
typename EBSearch<state, action, environment, DFS, costScale>::searchData EBSearch<state, action, environment, DFS, costScale>::LowLevelSearch(double costLimit, uint64_t nodeLimit)
{
	if (DFS)
		return DFBNB(costLimit, nodeLimit);
	else
		return BFHS(costLimit, nodeLimit);
}
#endif /* BID_h */

