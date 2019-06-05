//
//  IBEX.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 6/3/19.
//  Copyright © 2019 University of Denver. All rights reserved.
//

#ifndef IBEX_h
#define IBEX_h

#include "vectorCache.h"
#include "AStarOpenClosed.h"

namespace IBEX {
	
	template <class state>
	struct BFHSCompare {
		bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
		{
			return (fgreater(i1.g, i2.g));
		}
	};
	
	const int infiniteWorkBound = -1;
	
	template <class state, class action, class environment, bool DFS = true>
	class IBEX {
	public:
		IBEX(uint64_t minGrow, uint64_t maxGrow, double growthRate)
		:c1(minGrow), c2(maxGrow), gamma(growthRate)  {}
		void GetPath(environment *env, state from, state to,
					 std::vector<action> &thePath);
		void GetPath(environment *env, Heuristic<state> *heuristic, state from, state to,
					 std::vector<action> &thePath);
		double RedoMinWork();
		
		uint64_t GetNodesExpanded() { return totalNodesExpanded; }
		uint64_t GetNodesTouched() { return totalNodesTouched; }
		void ResetNodeCount() { totalNodesExpanded = totalNodesTouched = 0; }
	private:
		struct searchBounds {
			double f_below; // value below the actual limit
			double f_above;  // value above the actual limit
			uint64_t nodes; // nodes used to search
		};
		struct costInterval {
			double lowerBound;
			double upperBound;
			uint64_t nodes; // nodes used to search
			costInterval &operator&=(const costInterval &i)
			{
				lowerBound = std::max(lowerBound, i.lowerBound);
				upperBound = std::min(upperBound, i.upperBound);
				nodes = i.nodes;
				return *this;
			}
		};
		IBEX<state, action, environment, DFS>::costInterval LowLevelSearch(double costLimit, uint64_t nodeLimit);
//		IBEX<state, action, environment, DFS>::solutionInterval BinarySearch(searchBounds d, uint64_t nodeLimit);
//		IBEX<state, action, environment, DFS>::solutionInterval ExponentialSearch(const searchBounds &d, uint64_t nodeLimit);
		
		// Functions for DF Search
		double GCost(const state &s1, const state &s2)
		{ return env->GCost(s1, s2); }
		double GCost(const state &s, const action &a)
		{ return env->GCost(s, a); }
		double HCost(const state &s)
		{ return h->HCost(s, goal); }
		IBEX<state, action, environment, DFS>::costInterval DFBNB(double costLimit, uint64_t nodeLimit);
		IBEX<state, action, environment, DFS>::searchBounds DFBNBHelper(state &currState, double pathCost, double costLimit,
																	  searchBounds &sd, uint64_t nodeLimit, action forbidden);
		
		// Functions for BFHS
		IBEX<state, action, environment, DFS>::costInterval BFHS(double costLimit, uint64_t nodeLimit);
		void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath);
		
		
		uint64_t totalNodesExpanded, totalNodesTouched;
		Heuristic<state> *h;
		environment *env;
		std::vector<action> solutionPath, currPath;
		double solutionCost;
		vectorCache<action> actCache;
		state start, goal;
		uint64_t c1, c2;
		double gamma;
		
		// Data for BFHS
		AStarOpenClosed<state, BFHSCompare<state>> q;
		std::vector<state> neighbors;
		std::vector<uint64_t> neighborID;
		std::vector<double> edgeCosts;
		std::vector<dataLocation> neighborLoc;
		std::vector<state> solutionStates;
	};
	
	template <class state, class action, class environment, bool DFS>
	void IBEX<state, action, environment, DFS>::GetPath(environment *env, state from, state to,
														std::vector<action> &thePath)
	{
		GetPath(env, env, from, to, thePath);
	}
	
	template <class state, class action, class environment, bool DFS>
	void IBEX<state, action, environment, DFS>::GetPath(environment *env, Heuristic<state> *heuristic, state from, state to,
														std::vector<action> &thePath)
	{		
		this->env = env;
		h = heuristic;
		start = from;
		goal = to;
		solutionPath.clear();
		solutionCost = DBL_MAX;
		ResetNodeCount();
		
		uint64_t nodeLB = 1;
		costInterval solutionInterval;
//		nodeLB = 83886080/2;
//		solutionInterval.lowerBound = 63.01167;//HCost(from);
		solutionInterval.lowerBound = HCost(from);
		solutionInterval.upperBound = DBL_MAX;
		while (fgreater(solutionCost, solutionInterval.lowerBound))
		{
			printf("IBEX: Base search: f: %1.5f, cost limit ∞, target [%llu, %llu]\n", solutionInterval.lowerBound, c1*nodeLB, c2*nodeLB);
			solutionInterval &= LowLevelSearch(solutionInterval.lowerBound, infiniteWorkBound);

			// Move to next iteration
			if (solutionInterval.nodes >= c1*nodeLB)
			{
				nodeLB = solutionInterval.nodes;
				solutionInterval.upperBound = DBL_MAX;
				continue;
			}
			
			while (!(fequal(solutionInterval.upperBound, solutionInterval.lowerBound) ||
					 (solutionInterval.nodes >= c1*nodeLB && solutionInterval.nodes < c2*nodeLB)))
			{
				double nextCost;
				if (solutionInterval.upperBound == DBL_MAX)
					nextCost = solutionInterval.lowerBound * gamma;
				else
					nextCost = (solutionInterval.lowerBound+solutionInterval.upperBound)/2.0;

				solutionInterval &= LowLevelSearch(nextCost, c2*nodeLB);
			}
			
			nodeLB = std::max(solutionInterval.nodes, c1*nodeLB);
			solutionInterval.upperBound = DBL_MAX;

			if (fequal(solutionInterval.lowerBound, solutionCost))
				break;
		}
		thePath = solutionPath;
		printf("Found solution cost %1.5f\n", solutionCost);
	}
	
//	template <class state, class action, class environment, bool DFS>
//	typename IBEX<state, action, environment, DFS>::searchData IBEX<state, action, environment, DFS>::ExponentialSearch(const searchData &d, uint64_t nodeLimit)
//	{
//		uint64_t lowNodes = c1*nodeLimit;
//		uint64_t highNodes = c2*nodeLimit;
//		uint64_t delta = std::max(d.nextF - d.f, initialGap);
//		int i = 0;
//		searchData lastSuccess = d;
//		while (true)
//		{
//			uint64_t bound = d.f + delta*pow(2, i);
//			searchData curr = LowLevelSearch(bound, highNodes);
//			if (solutionCost <= bound && curr.nodes < highNodes)
//			{
//				printf("***EBIS:->EXP: Solution proven below window; done\n");
//				curr.f = solutionCost;
//				return curr;
//			}
//
//			if (lowNodes <= curr.nodes && curr.nodes < highNodes)
//			{
//				printf("EBIS:->EXP: in node window [%llu <= %llu < %llu]\n", lowNodes, curr.nodes, highNodes);
//				return curr;
//			}
//			else if (curr.nodes >= highNodes)
//			{
//				printf("EBIS:->EXP: over node window\n");
//				lastSuccess.nodes = curr.nodes;
//				lastSuccess.failedF = bound;
//				return lastSuccess;
//			}
//			else {
//				printf("EBIS:->EXP: below node window\n");
//				lastSuccess = curr;
//			}
//			i++;
//			if (d.f + delta*pow(2, i) < curr.nextF)
//				printf("EBIS:->EXP: delta too small, increasing growth past nextf\n");
//			while (d.f + delta*pow(2, i) < curr.nextF)
//			{
//				i++;
//			}
//		}
//	}
//
//	template <class state, class action, class environment, bool DFS>
//	typename IBEX<state, action, environment, DFS>::searchData IBEX<state, action, environment, DFS>::BinarySearch(searchData d, uint64_t nodeLimit)
//	{
//		uint64_t lowNodes = c1*nodeLimit;
//		uint64_t highNodes = c2*nodeLimit;
//		uint64_t middlef = (d.failedF + d.f)/2.0;
//		searchData curr;
//		if (middlef <= d.nextF)
//		{
//			middlef = d.nextF;
//			curr = LowLevelSearch(middlef, infiniteWorkBound);
//			if (solutionCost <= middlef)
//				curr.f = middlef;
//			if (curr.nodes >= lowNodes)
//				return curr;
//		}
//		else {
//			curr = LowLevelSearch(middlef, highNodes);
//		}
//		// found and proved solution
//		if (solutionCost <= middlef && curr.nodes < highNodes)
//		{
//			printf("***EBIS:->BIN: Solution proven below window; done\n");
//			curr.f = middlef;
//			return curr;
//		}
//
//		if (lowNodes <= curr.nodes && curr.nodes < highNodes)
//		{
//			printf("EBIS:->BIN: in node window [%llu <= %llu < %llu]\n", lowNodes, curr.nodes, highNodes);
//			return curr;
//		}
//		else if (curr.nodes >= highNodes)
//		{
//			printf("EBIS:->BIN: over node window\n");
//			d.failedF = std::min(middlef, solutionCost);
//			return BinarySearch(d, nodeLimit);
//		}
//		else {
//			printf("EBIS:->BIN: below node window\n");
//			curr.failedF = d.failedF;
//			return BinarySearch(curr, nodeLimit);
//		}
//	}
	
	template <class state, class action, class environment, bool DFS>
	typename IBEX<state, action, environment, DFS>::costInterval IBEX<state, action, environment, DFS>::DFBNB(double costLimit, uint64_t nodeLimit)
	{
		state currState = start;
		if (nodeLimit == infiniteWorkBound)
			printf("    --+DFBnB f: %1.5f; nodes: ∞; ", costLimit);
		else
			printf("    --+DFBnB f: %1.5f; nodes: %llu; ", costLimit, nodeLimit);
		currPath.clear();
		searchBounds sd;
		sd.f_below = 0;
		sd.f_above = DBL_MAX;
		sd.nodes = 0;
		action a;
		sd = DFBNBHelper(currState, 0, costLimit, sd, nodeLimit, a);
		totalNodesExpanded += sd.nodes;
		// TODO: In practice we should set the nextF to be sd.f
		// Need to test this.
//		if (sd.nextF == -1ull) // so few nodes expanded we didn't find the next bound
//		{
//			printf(" (oops) ");
//			sd = DFBNBHelper(currState, 0, costLimit, sd, -1ull, a);
//			totalNodesExpanded += sd.nodes;
//		}
		//if (nodesExpanded)
		// TODO: return ranges here
		costInterval v;
		if (sd.nodes >= nodeLimit)
		{
			v.lowerBound = 0;
			v.upperBound = sd.f_below;
		}
		else if (solutionCost != DBL_MAX && fgreatereq(sd.f_below, solutionCost))
		{
			v.lowerBound = solutionCost;
			v.upperBound = solutionCost;
		}
		else {
			v.lowerBound = sd.f_above;
			v.upperBound = DBL_MAX;
			assert(fgreater(sd.f_above, costLimit));
		}
		v.nodes = sd.nodes;
		if (v.upperBound == DBL_MAX)
			printf("%llu (new) %llu (total), solution range: [%1.5f, ∞] ", v.nodes, totalNodesExpanded, v.lowerBound);
		else
			printf("%llu (new) %llu (total), solution range: [%1.5f, %1.5f] ", v.nodes, totalNodesExpanded, v.lowerBound, v.upperBound);
		if (solutionCost != DBL_MAX)
			printf("sol: %1.5f\n", solutionCost);
		else
			printf("\n");
		return v;
		//return sd;
	}
	
	template <class state, class action, class environment, bool DFS>
	typename IBEX<state, action, environment, DFS>::searchBounds IBEX<state, action, environment, DFS>::DFBNBHelper(state &currState, double pathCost, double costLimit,
																												  searchBounds &sd, uint64_t nodeLimit, action forbidden)
	{
		double currF = pathCost+HCost(currState);
		//	printf("-------->%f [%f]\n", currF, pathCost);
		if (fgreater(currF, costLimit))
		{
			sd.f_above = std::min(sd.f_above, currF);
			return sd;
		}
		else if (fgreatereq(currF, solutionCost))
		{
			sd.f_below = solutionCost;
			return sd;
		}
		else {
			sd.f_below = std::max(currF, sd.f_below);
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
		
		std::vector<action> &acts = *actCache.getItem();
		env->GetActions(currState, acts);
		sd.nodes++;
		totalNodesTouched+=acts.size();
		
		for (int x = 0; x < acts.size(); x++)
		{
			if (acts[x] == forbidden && currPath.size() > 0)
				continue;
			double edgeCost = GCost(currState, acts[x]);
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
	
	template <class state, class action, class environment, bool DFS>
	double IBEX<state, action, environment, DFS>::RedoMinWork()
	{
		ResetNodeCount();
		printf("IBEX Validation:\n");
		LowLevelSearch(solutionCost, -1);
		return solutionCost;
	}
	
	template <class state, class action, class environment, bool DFS>
	typename IBEX<state, action, environment, DFS>::costInterval IBEX<state, action, environment, DFS>::BFHS(double costLimit, uint64_t nodeLimit)
	{
		if (nodeLimit == -1ull && costLimit == DBL_MAX)
			printf("    --+BFHS f: ∞; nodes: ∞; ");
		else if (nodeLimit == -1)
			printf("    --+BFHS f: %1.5f; nodes: ∞; ", costLimit);
		else
			printf("    --+BFHS f: %1.5f; nodes: %llu; ", costLimit, nodeLimit);
		
		searchBounds sd;
		sd.f_below = 0;
		sd.f_above = DBL_MAX;
		sd.nodes = 0;
		
		q.Reset(env->GetMaxHash());

		// put start in open
		q.AddOpenNode(start, env->GetStateHash(start), 0, 0);
		
		while (sd.nodes < nodeLimit && q.OpenSize() > 0)
		{
			// expand by low g until
			// (1) we find the goal
			// (2) we hit the node limit
			// (3) we exhaust all states
			uint64_t nodeid = q.Close();
			sd.nodes++;
			totalNodesExpanded++;
			
			double nextf = q.Lookup(nodeid).g + HCost(q.Lookup(nodeid).data);
			if (fgreater(nextf, costLimit))
			{
				// case shouldn't happen - pruned elsewhere
				assert(false);
			} else {
				sd.f_below = std::max(sd.f_below, nextf);
			}

			
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
				// TODO: return range here
				return {q.Lookup(nodeid).g, q.Lookup(nodeid).g, sd.nodes};
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
				edgeCosts.push_back(GCost(q.Lookup(nodeid).data, neighbors[x]));
			}
			
			// iterate again updating costs and writing out to memory
			for (int x = 0; x < neighbors.size(); x++)
			{
				totalNodesTouched++;
			
				double childGCost = q.Lookup(nodeid).g+edgeCosts[x];
				double childFCost = childGCost+HCost(neighbors[x]);
				if (fgreater(childFCost, costLimit) || fgreatereq(childFCost, solutionCost))
				{
					sd.f_above = std::min(sd.f_above, childFCost);
					continue;
				}

				switch (neighborLoc[x])
				{
					case kClosedList:
						break;
					case kOpenList:
						if (fless(childGCost, q.Lookup(neighborID[x]).g))
						{
							q.Lookup(neighborID[x]).parentID = nodeid;
							q.Lookup(neighborID[x]).g = childGCost;
							q.KeyChanged(neighborID[x]);
						}
						break;
					case kNotFound:
					{
						q.AddOpenNode(neighbors[x],
									  env->GetStateHash(neighbors[x]),
									  childGCost,
									  0,
									  nodeid);
					}
				}
			}
		}
		// f, nextF, failedF, nodes
		
		// TODO: return range here
		costInterval v;
		if (sd.nodes >= nodeLimit)
		{
			v.lowerBound = 0;
			v.upperBound = sd.f_below;
		}
		else {
			v.lowerBound = sd.f_above;
			v.upperBound = DBL_MAX;
		}
		v.nodes = sd.nodes;

		if (v.upperBound == DBL_MAX)
			printf("%llu (new) %llu (total), solution range: [%1.5f, ∞]\n", sd.nodes, totalNodesExpanded, v.lowerBound);
		else
			printf("%llu (new) %llu (total), solution range: [%1.5f, %1.5f]\n", sd.nodes, totalNodesExpanded, v.lowerBound, v.upperBound);

		return v;
	}
	
	template <class state, class action,class environment,bool DFS>
	void IBEX<state, action, environment, DFS>::ExtractPathToStartFromID(uint64_t node,
																		 std::vector<state> &thePath)
	{
		thePath.clear();
		do {
			thePath.push_back(q.Lookup(node).data);
			node = q.Lookup(node).parentID;
		} while (q.Lookup(node).parentID != node);
		thePath.push_back(q.Lookup(node).data);
	}
	
	
	template <class state, class action, class environment, bool DFS>
	typename IBEX<state, action, environment, DFS>::costInterval IBEX<state, action, environment, DFS>::LowLevelSearch(double costLimit, uint64_t nodeLimit)
	{
		if (DFS)
			return DFBNB(costLimit, nodeLimit);
		else
			return BFHS(costLimit, nodeLimit);
	}
	
}

#endif /* IBEX_h */
