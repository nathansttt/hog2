/*
 *  FLRTAStar.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/15/10.
 *  Copyright 2010 University of Denver. All rights reserved.
 *
 */

#ifndef FLRTASTAR_H
#define FLRTASTAR_H

#include "GenericSearchAlgorithm.h"
#include "SearchEnvironment.h"
#include "FPUtil.h"
#include <deque>
#include <vector>
#include <ext/hash_map>
#include "TemplateAStar.h"
#include <queue>
#include <iostream>

namespace FLRTA {
	static bool verbose = false;
	
	template <class state>
	class borderData {
	public:
		borderData(const state &s, const double val) :theState(s), value(val) {}
		state theState;
		double value;
	};
	
	template <class state>
	class compareBorderData
	{
	public:
		bool operator() (const borderData<state> &lhs, const borderData<state> &rhs) const
		{
			return (lhs.value > rhs.value);
		}
	};
	
	template <class state>
	class learnedStateData {
	public:
		learnedStateData() :theState(), gCost(DBL_MAX), hCost(0), dead(false), redundant(false) {}
		state theState;
		double gCost;
		double hCost;
		bool dead;
		bool redundant;
//		std::vector<state> parents;
	};

	template <class state>
	struct dblCmp {
		bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
		{
			return (fgreater(i1.g, i2.g));
		}
	};
	
	template <class state, class action, class environment>
	class FLRTAStar : public GenericSearchAlgorithm<state,action,environment>, public Heuristic<state> {
	public:
		FLRTAStar(int nodeLimit = 8, double weight = 1.5)
		{ fAmountLearned = 0.0f; nodeExpansionLimit = nodeLimit; /*pe = 0;*/ nodeLearningLimit = 1;
			fWeight = weight; orderRedundant = false;
		}
		virtual ~FLRTAStar(void) { /*delete pe;*/ }
		
		void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
		void GetPath(environment *, const state& , const state& , std::vector<action> & ) { assert(false); };
		virtual const char *GetName() { static char name[255]; sprintf(name, "FLRTAStar(%d,%1.1f,%c)", nodeExpansionLimit, fWeight, orderRedundant?'o':'u'); return name; }
		void SetOrderRedundant(bool val) { orderRedundant = val; }
		
		void KillState(const state &where)
		{
//			if (stateData.find(m_pEnv->GetStateHash(where)) != stateData.end())
//				stateData[m_pEnv->GetStateHash(where)].dead = true;
//			else {
			//std::cout << "Hashing state:1 " << std::endl << where << std::endl;
			stateData[m_pEnv->GetStateHash(where)].theState = where;
			stateData[m_pEnv->GetStateHash(where)].dead = true;
//			}
		}
		
		bool IsDead(const state &where)
		{
			// the default constructor makes dead == false, so we only have to initalize the state
			//std::cout << "Hashing state:2 " << std::endl << where << std::endl;
			if (stateData.find(m_pEnv->GetStateHash(where)) == stateData.end())
				return false;
			stateData[m_pEnv->GetStateHash(where)].theState = where;
			return stateData[m_pEnv->GetStateHash(where)].dead;
		}
		
		void SetGCost(environment *env, const state &where, double val)
		{
			if (verbose) std::cout << "-->GCost of " << where << " setting to " << val << std::endl;
			//std::cout << "Hashing state:3 " << std::endl << where << std::endl;
			fAmountLearned -= stateData[env->GetStateHash(where)].hCost;
			stateData[env->GetStateHash(where)].hCost = 0;
			stateData[env->GetStateHash(where)].gCost = val;
			stateData[env->GetStateHash(where)].theState = where;
			stateData[env->GetStateHash(where)].dead = false; // updated g-cost, make it alive again
		}
		
		double FCost(const state &where, const state &goal)
		{
			// could be more efficient
			return GCost(where)+HCost(where, goal);
		}
		
		double GCost(environment *env, const state &where)
		{
			//std::cout << "Hashing state:4 " << std::endl << where << std::endl;
			if (stateData.find(env->GetStateHash(where)) != stateData.end())
			{
				stateData[env->GetStateHash(where)].theState = where;
				return stateData[env->GetStateHash(where)].gCost;
			}
			return DBL_MAX; // g-costs can only be lowered, hence the high start
		}
		double GCost(const state &where)
		{ return GCost(m_pEnv, where); }
		
		
		void SetHCost(environment *env, const state &where, const state &to, double val)
		{
			double tmp = val-env->HCost(where, to);
			if (tmp < 0) tmp = 0;
			//std::cout << "Hashing state:5 " << std::endl << where << std::endl;
			stateData[env->GetStateHash(where)].hCost = tmp;
			stateData[env->GetStateHash(where)].theState = where;
		}
		double HCost(environment *env, const state &from, const state &to)
		{
			//std::cout << "Hashing state:6 " << std::endl << from << std::endl;
			if (stateData.find(env->GetStateHash(from)) != stateData.end())
			{
//				if (IsDead(from))
//					return DBL_MAX;
				return stateData[env->GetStateHash(from)].hCost+env->HCost(from, to);
			}
			return env->HCost(from, to);
		}
		double HCost(const state &from, const state &to)
		{ return HCost(m_pEnv, from, to); }
		
		bool IsRedundant(const state &where)
		{
//			return false;
			std::vector<state> succ;
			m_pEnv->GetSuccessors(where, succ);
			for (unsigned int x = 0; x < succ.size(); x++)
			{
				if (IsDead(succ[x]))
				{
					if (verbose) std::cout << succ[x] << " is dead" << std::endl;
					continue;
				}
				if (fequal(GCost(succ[x]), DBL_MAX))
				{
					if (verbose) std::cout << where << " has child " << succ[x] << " with infinite g-cost. Not redundant!" << std::endl;
					return false;
				}
				//std::cout << "GCost state:1 " << std::endl << succ[x] << where << std::endl;
				if (fequal(GCost(succ[x]),
						   GCost(where)+m_pEnv->GCost(where, succ[x])) &&
					(NumParents(succ[x]) <= 1))
				{
					if (verbose)
						std::cout << succ[x] << " relies on " << where << " as its only parent, not redundant" << std::endl;
					return false;
				}
				if (verbose) std::cout << DBL_MAX << succ[x] << " has no status -- gcost: " << GCost(succ[x]) << std::endl;
				if (orderRedundant && IsBestParent(succ[x], where))
					return false;
			}
			return true;
		}

		int NumParents(const state &where)
		{
			int count = 0;
			std::vector<state> succ;
			m_pEnv->GetSuccessors(where, succ);
			for (unsigned int x = 0; x < succ.size(); x++)
			{
//				// unexplored parent/child, counts as a parent for now
//				if (fequal(GCost(succ[x]), DBL_MAX))
//					count++;
				// parent has less or equal path cost
				//std::cout << "GCost state:2 " << std::endl << succ[x] << where << std::endl;
				if (fequal(GCost(succ[x])+m_pEnv->GCost(succ[x], where),
							  GCost(where)) &&
					!IsDead(succ[x]))
					count++;
			}
			if (verbose) std::cout << where << " has " << count << " successors" << std::endl;
			return count;
		}

		bool IsBestParent(const state &child, const state &parent)
		{
			std::vector<state> succ;
			m_pEnv->GetSuccessors(child, succ);
			for (unsigned int x = 0; x < succ.size(); x++)
			{
				if (succ[x] == parent) continue;

				//std::cout << "GCost state:3 " << std::endl << succ[x] << parent << child << std::endl;
				if (fequal(GCost(succ[x])+m_pEnv->GCost(succ[x], child),
						   GCost(child)) &&
					!IsDead(succ[x]) &&
					GCost(succ[x]) >= GCost(parent))
					return true;
			}
			return false;
		}
		
		virtual uint64_t GetNodesExpanded() const { return nodesExpanded; }
		virtual uint64_t GetNodesTouched() const { return nodesTouched; }
		virtual void LogFinalStats(StatCollection *s) { s->AddStat("TotalLearning", GetName(),fAmountLearned); }
		
		double GetAmountLearned() { return fAmountLearned; }
		void OpenGLDraw() const {}
		void OpenGLDraw(const environment *env) const;
	private:
		typedef __gnu_cxx::hash_map<uint64_t, learnedStateData<state>, Hash64 > LearnedStateData;
		typedef __gnu_cxx::hash_map<uint64_t, bool, Hash64 > ClosedList;
		void ExtractBestPath(environment *env, const state &from, const state &to, std::vector<state> &thePath);
		void MakeTrappedMove(environment *env, const state &from, std::vector<state> &thePath);
		
		void ExpandLSS(const state &from, const state &to, std::vector<state> &thePath);
		void MarkDeadRedundant(environment *env, const state &goal);
		void DoHCostLearning(environment *env, const state &from, const state &to);
		void PropagateGCosts(const state &next, const state &to, bool alsoExpand);
		
		environment *m_pEnv;
		LearnedStateData stateData;
		double fAmountLearned, fWeight;
		uint64_t nodesExpanded, nodesTouched;
		int nodeExpansionLimit, nodeLearningLimit;
		state theEnd;
		bool orderRedundant;
		
		typedef std::priority_queue<borderData<state>,std::vector<borderData<state> >,compareBorderData<state> > pQueue;	
		pQueue gCostQueue;

		AStarOpenClosed<state, dblCmp<state> > aoc;
	};
	
	/** The core routine of FLRTAStar -- computes at most one-move path */
	template <class state, class action, class environment>
	void FLRTAStar<state, action, environment>::GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath)
	{
		nodesExpanded = nodesTouched = 0;
		m_pEnv = env;
		thePath.resize(0);
		theEnd = to;
		aoc.Reset();
		assert(gCostQueue.empty());
		if (from==to)
			return;
		
		if (GCost(from) == DBL_MAX)
			SetGCost(env, from, 0);

		ExpandLSS(from, to, thePath);
//		if (nodesExpanded >= nodeExpansionLimit)
//			std::cout << GetName() << " Lookahead " << nodeExpansionLimit << ", " << nodesExpanded-nodeExpansionLimit << " expanded during g-cost propagation" << std::endl;
//		else
//			std::cout << GetName() << " Lookahead " << nodeExpansionLimit << ", " << 0 << " expanded during g-cost propagation" << std::endl;
		// in case goal was found in LSS
		if (thePath.size() != 0)
			return;
		uint64_t old = nodesExpanded;
		DoHCostLearning(env, from, to);
//		std::cout << GetName() << " " << nodesExpanded-old << " learning expansions before dead/redundant " << std::endl;
		MarkDeadRedundant(env, to);
//		std::cout << GetName() << nodesExpanded << " total nodes this step" << std::endl;
		ExtractBestPath(env, from, to, thePath);
		
		
		if (verbose) std::cout << "FLRTA* heading towards " << thePath[0] << " with h-value " << HCost(env, thePath[0], to) << std::endl;
	}

	template <class state, class action, class environment>
	void FLRTAStar<state, action, environment>::ExtractBestPath(environment *env, const state &from, const state &to, std::vector<state> &thePath)
	{
		state best;
		unsigned int cnt = 0;
		for (; cnt < aoc.OpenSize(); cnt++)
		{
			if (!IsDead(aoc.Lookat(aoc.GetOpenItem(cnt)).data) &&
				!(aoc.Lookat(aoc.GetOpenItem(cnt)).data == from))
			{
				best = aoc.Lookat(aoc.GetOpenItem(cnt)).data;
				break;
			}
		}
		// no path found, go backwards in g-cost
		if (cnt == aoc.OpenSize())
		{
			MakeTrappedMove(env, from, thePath);
			return;
		}
		// 3. Find node with highest f-cost / g-cost
		for (; cnt < aoc.OpenSize(); cnt++)
		{
			const AStarOpenClosedData<state> data = aoc.Lookat(aoc.GetOpenItem(cnt));
			if (IsDead(data.data))
				continue;
			if (fgreater(GCost(best)+fWeight*HCost(best, to),
						 GCost(data.data)+fWeight*HCost(data.data, to)) &&
				!(data.data == from))
			{
				best = data.data;
			}
			else if (fequal(GCost(best)+fWeight*HCost(best, to), GCost(data.data)+fWeight*HCost(data.data, to)) &&
					 fgreater(GCost(data.data), GCost(best)) &&
					 !(data.data == from))
			{
				best = data.data;
			}
		}
		// 4. construct best path
		if (verbose) std::cout << "Moving towards " << best << " cost " << GCost(best) << std::endl;
			uint64_t node;
			aoc.Lookup(env->GetStateHash(best), node);
			do {
				thePath.push_back(aoc.Lookup(node).data);
				node = aoc.Lookup(node).parentID;
			} while (aoc.Lookup(node).parentID != node);
			thePath.push_back(aoc.Lookup(node).data);
	}

	template <class state, class action, class environment>
	void FLRTAStar<state, action, environment>::MakeTrappedMove(environment *env, const state &from, std::vector<state> &thePath)
	{
		std::vector<state> succ;
		env->GetSuccessors(from, succ);
		nodesExpanded++;
		int back = -1;
		bool found = false;
		for (unsigned int x = 0; x < succ.size(); x++)
		{
			if ((back == -1) && (GCost(succ[x]) < GCost(from)))
			{
				back = x;
				found = true;
			}
			else if ((back != -1) && (GCost(succ[x]) < GCost(succ[back])))
			{
				back = x;
				found = true;
			}
		}
		if (!found)
		{
			std::cout << "No successors of " << from << " with smaller g-cost than " << GCost(from) << std::endl;
			for (unsigned int x = 0; x < succ.size(); x++)
				std::cout << succ[x] << " has cost " << GCost(succ[x]) << std::endl;
			assert(!"Invalid environment; apparently cannot reach all predecessors of a state");
			exit(0);
		}
		thePath.push_back(succ[back]);
		thePath.push_back(from);
		//if (verbose) std::cout << "FLRTA* heading towards " << thePath[0] << " with h-value " << HCost(env, thePath[0], to) << std::endl;
		return;
	}

	template <class state, class action, class environment>
	void FLRTAStar<state, action, environment>::ExpandLSS(const state &from, const state &to, std::vector<state> &thePath)
	{
		if (verbose) std::cout << "Expanding LSS" << std::endl;
		// 0. put from on OPEN
		//std::cout << "Hashing state:8 " << std::endl << from << std::endl;
		aoc.AddOpenNode(from, m_pEnv->GetStateHash(from), 0.0, 0.0);
		std::vector<state> tempDead;

		for (int x = 0; x < nodeExpansionLimit; x++)//nodeLearningLimit
		{
			if (aoc.OpenSize() == 0)
				break;
			// 1. choose next node to expand
			uint64_t next = aoc.Close();
			if (verbose) std::cout << "Next in LSS: " << aoc.Lookat(next).data << std::endl;

			// 2. propagate g-costs to neighbors and beyond if this node isn't dead
			if (IsDead(aoc.Lookat(next).data) && x != 0)
			{
				// dead; ignore(?)
				x--;
				//tempDead.push_back(aoc.Lookat(next).data)
				if (verbose) std::cout << aoc.Lookat(next).data << " is dead; left on closed (LSS)" << std::endl;
				continue;
			}
			else {
				// cannot pass data from AOC by reference, because when data structures are
				// resized it can become invalidated
				state val = aoc.Lookat(next).data;
				PropagateGCosts(val, to, true);
			}

			// 3. reached goal, stop
			if (aoc.Lookat(next).data == to)
			{
				uint64_t node = next;
				do {
//					std::cout << aoc.Lookup(node).data << " ";
					thePath.push_back(aoc.Lookup(node).data);
					node = aoc.Lookup(node).parentID;
				} while (aoc.Lookup(node).parentID != node);
				thePath.push_back(aoc.Lookup(node).data);
				return;
			}
		}
		
		// 4. finish propagation from edge of open list
		for (unsigned int x = 0; x < aoc.OpenSize(); x++)
		{
			state s = aoc.Lookat(aoc.GetOpenItem(x)).data;
			gCostQueue.push(borderData<state>(s, GCost(s)));
		}
		while (!gCostQueue.empty())
		{
			state s = gCostQueue.top().theState;
			gCostQueue.pop();
			std::vector<state> neighbors;
			m_pEnv->GetSuccessors(s, neighbors);
			nodesExpanded++;
			for (unsigned int x = 0; x < neighbors.size(); x++)
			{
				uint64_t childKey;
				double edgeCost = m_pEnv->GCost(s, neighbors[x]);
				if ((GCost(s) != DBL_MAX) && (fless(GCost(s)+edgeCost, GCost(neighbors[x]))))
				{
					SetGCost(m_pEnv, neighbors[x], GCost(s)+edgeCost);
					if (aoc.Lookup(m_pEnv->GetStateHash(neighbors[x]), childKey) != kNotFound)
						gCostQueue.push(borderData<state>(neighbors[x], GCost(neighbors[x])));
				}
			}
		}
	}


	template <class state, class action, class environment>
	void FLRTAStar<state, action, environment>::PropagateGCosts(const state &next, const state &to, bool alsoExpand)
	{
		if (verbose) std::cout << "=Propagating from: " << next << (alsoExpand?" and expanding":" not expanding") << std::endl;
		// decrease g-cost as long as somewhere on aoc / expand if requested
		std::vector<state> neighbors;
		m_pEnv->GetSuccessors(next, neighbors);
		nodesExpanded++;
		uint64_t parentKey;
		//std::cout << "Hashing state:9 " << std::endl << next << std::endl;
		dataLocation pLoc = aoc.Lookup(m_pEnv->GetStateHash(next), parentKey);

		if (verbose) std::cout << GCost(next) << " gcost in " <<
			((pLoc==kOpenList)?("open"):((pLoc==kClosedList)?"closed":"none")) << std::endl;
		for (unsigned int x = 0; x < neighbors.size(); x++)
		{
			nodesTouched++;
			double edgeCost = m_pEnv->GCost(next, neighbors[x]);
			uint64_t childKey;
			//std::cout << "Hashing state:10 " << std::endl << neighbors[x] << std::endl;
			dataLocation cLoc = aoc.Lookup(m_pEnv->GetStateHash(neighbors[x]), childKey);

			if (alsoExpand && pLoc != kNotFound)
			{
				if (cLoc == kNotFound) // add node even if it is dead!
				{
					if (verbose) std::cout << "Adding " << neighbors[x] << " to open" << std::endl;
					aoc.AddOpenNode(neighbors[x], m_pEnv->GetStateHash(neighbors[x]),
									aoc.Lookat(parentKey).g+edgeCost, HCost(neighbors[x], to), parentKey);
				}
				else if (cLoc == kOpenList)
				{   // these are local g costs
					if (fless(aoc.Lookup(parentKey).g+edgeCost, aoc.Lookup(childKey).g))
					{
						if (verbose) std::cout << "Updating " << neighbors[x] << " on open" << std::endl;
						aoc.Lookup(childKey).parentID = parentKey;
						aoc.Lookup(childKey).g = aoc.Lookup(parentKey).g+edgeCost;
						aoc.KeyChanged(childKey);
					}
				}
				else if (cLoc == kClosedList)
				{
					if (fless(aoc.Lookup(parentKey).g+edgeCost, aoc.Lookup(childKey).g))
					{
						if (verbose) std::cout << "Reopening " << neighbors[x] << std::endl;
						aoc.Lookup(childKey).parentID = parentKey;
						aoc.Lookup(childKey).g = aoc.Lookup(parentKey).g+edgeCost;
						aoc.Reopen(childKey);
					}
				}
			}

			// shorter g-cost to neighbors[x] from global search perspective
			if ((GCost(next) != DBL_MAX) && (fless(GCost(next)+edgeCost, GCost(neighbors[x]))))
			{
				bool propagate = false;
				if (IsDead(neighbors[x]) && (cLoc == kClosedList)) // important step in proof!
				{
					aoc.Reopen(childKey);
					propagate = true;
					SetGCost(m_pEnv, neighbors[x], GCost(next)+edgeCost);
					if (verbose) std::cout << "Updating " << neighbors[x] << " from " << GCost(neighbors[x]) <<
						" to " << GCost(next) << "(" << next << ") + " << edgeCost << " = " << GCost(next)+edgeCost << std::endl;
					PropagateGCosts(neighbors[x], to, propagate);
				}
				else {
					SetGCost(m_pEnv, neighbors[x], GCost(next)+edgeCost);
//					if (cLoc != kNotFound)
//						PropagateGCosts(neighbors[x], to, propagate);
				}
//				else {
//					if (verbose) std::cout << "Updating " << neighbors[x] << " from " << GCost(neighbors[x]) <<
//						" to " << GCost(next) << "(" << next << ") + " << edgeCost << " = " << GCost(next)+edgeCost << std::endl;
//					SetGCost(m_pEnv, neighbors[x], GCost(next)+edgeCost);
//
//					// if it's on the open we'll check later
//					if (cLoc == kClosedList)
//						gCostQueue.push(borderData<state>(neighbors[x], -GCost(data.data)));
//
//					if (cLoc != kNotFound)
//					{
//						// 3. if neighbor is on CLOSED/OPEN and has g-cost reduced, continue the propagation
//						PropagateGCosts(neighbors[x], to, propagate);
//					}
//				}
			}
			// shorter g-cost from neighbor to here, reverse search
			//std::cout << "GCost state:9 " << std::endl << next << neighbors[x] << std::endl;
//			if (fless(edgeCost+GCost(neighbors[x]), GCost(next)))
//			{
//				PropagateGCosts(neighbors[x], to, false);
//			}
		}
		if (verbose) std::cout << "=Done Propagating from: " << next << std::endl;
	}
	
	template <class state, class action, class environment>
	void FLRTAStar<state, action, environment>::MarkDeadRedundant(environment *env, const state &goal)
	{
		// 1. put all open nodes in pqueue
		pQueue q;
		
		unsigned int openSize = aoc.size();
		for (unsigned int x = 0; x < openSize; x++)
		{
			const AStarOpenClosedData<state> data = aoc.Lookat(x);//astar.GetItem(x);
			//if ((data.where == kClosedList) || (nodeExpansionLimit == 1))
			{
				q.push(borderData<state>(data.data, -GCost(data.data)));
				if (verbose) std::cout <<std::endl<< ">>>Preparing state: " << data.data << " g: " << GCost(data.data) << std::endl;
			}
		}
		
		std::vector<state> succ;
		state first = q.top().theState;
		while (q.size() > 0)
		{
			//nodesExpanded++;
			//nodesTouched++;
			state s = q.top().theState;
			if (verbose) std::cout << "Doing update from " << s << " g: " << q.top().value << "/" << GCost(s) << std::endl;
			q.pop();

			if (IsDead(s) || (s == goal))
				continue;
			//std::cout << "GCost state:13 " << std::endl << s << goal << std::endl;
			if (fgreater(GCost(s)+HCost(s, goal), GCost(goal)))
			{
				if (verbose) std::cout<<"Marking " << GCost(s) << ":" << HCost(s, goal) << " " << s << " as dead -- too far from goal: " << GCost(goal) << goal << std::endl;
				KillState(s);
			}
			else if (IsRedundant(s)) // catches the dead case too
			{
				if (verbose) 
					std::cout<<"Marking " << s << " as redundant" << std::endl;
				KillState(s);
				//put successors back on queue in case they are dead too
//				nodesExpanded++;
//				env->GetSuccessors(s, succ);
//				for (unsigned int x = 0; x < succ.size(); x++)
//				{
//					uint64_t tmpKey;
					//std::cout << "Hashing state:14 " << std::endl << succ[x] << std::endl;
//					if ((kNotFound != aoc.Lookup(env->GetStateHash(succ[x]), tmpKey)) && (!IsDead(succ[x])))
//						//if (astar.HaveExpandedState(succ[x]) && !IsDead(succ[x]))
//						q.push(borderData<state>(succ[x], DBL_MAX));
//				}
			}
		}
	}
	
	
	template <class state, class action, class environment>
	void FLRTAStar<state, action, environment>::DoHCostLearning(environment *env, const state &from, const state &to)
	{
		// 1. put all open nodes in pqueue
		pQueue q;

		// for successors
		std::vector<state> succ;
		
		unsigned int openSize = aoc.OpenSize();
		for (unsigned int x = 0; x < openSize; x++)
		{
			const AStarOpenClosedData<state> data = aoc.Lookat(aoc.GetOpenItem(x));
			if (!IsDead(data.data))
			{
				q.push(borderData<state>(data.data, HCost(data.data, to)));
				if (verbose) std::cout << "Preparing border state: " << data.data << " h: " << data.h << std::endl;
			}
		}
		if (q.size() == 0)
		{
			return;
		}
		
		state first = q.top().theState;
		ClosedList c;
		while (q.size() > 0)
		{
			state s = q.top().theState;
			if (verbose) std::cout << "Propagating from " << s << " h: " << q.top().value << "/" << HCost(s, to) << std::endl;
			q.pop();
			//			std::cout << s << " " << learnData[env->GetStateHash(s)].learnedHeuristic << std::endl;
			env->GetSuccessors(s, succ);
			nodesExpanded++;
			nodesTouched++;
			double hCost = HCost(s, to);
			for (unsigned int x = 0; x < succ.size(); x++)
			{
				nodesTouched++;
				double succHCost;

				uint64_t succKey;
				//std::cout << "Hashing state:15 " << std::endl << succ[x] << std::endl;
				dataLocation pLoc = aoc.Lookup(env->GetStateHash(succ[x]), succKey);
				if (pLoc != kClosedList)
				{
					//if (verbose) std::cout << succ[x] << " not in closed\n";
					continue;
				}
				double edgeCost = env->GCost(s, succ[x]);
				succHCost = HCost(env, succ[x], to);
				if (c[env->GetStateHash(succ[x])]) // in closed list, but seen before, update if smaller
				{
					if (verbose) std::cout << succ[x] << " updated before " << std::endl;
					if (fless(hCost + edgeCost, succHCost))
					{
						fAmountLearned -= succHCost-hCost-edgeCost;
						if (verbose) std::cout << "lowering cost to " << hCost + edgeCost;
						//if (verbose) std::cout << " learning now " << fAmountLearned;
						SetHCost(env, succ[x], to, hCost + edgeCost);
						q.push(borderData<state>(succ[x], hCost + edgeCost));
					}
					//if (verbose) std::cout << std::endl;
				}
				else { // not expanded before and in closed list, always update
					// 2. expand successors, checking if they are in closed list and adding to queue
					//				if (fless(succHCost, hCost - edgeCost))
					if (verbose) std::cout << succ[x] << " NOT updated before ";
					//if (fgreater(hCost + edgeCost, succHCost))
					{
						if (verbose) std::cout << "setting cost to " << hCost + edgeCost << " over " << succHCost << std::endl;
						fAmountLearned += (edgeCost + hCost) - succHCost;
						//if (verbose) std::cout << " learning now " << fAmountLearned;
						SetHCost(env, succ[x], to, hCost + edgeCost);
						q.push(borderData<state>(succ[x], hCost + edgeCost));
						c[env->GetStateHash(succ[x])] = true;
					}
					//if (verbose) std::cout << std::endl;
				}
			}
		}
//		return first;
	}
	
	template <class state, class action, class environment>
	void FLRTAStar<state, action, environment>::OpenGLDraw(const environment *e) const
	{
//		if (astar.GetNodesExpanded() > 0)
//		astar.OpenGLDraw();
		char str[32];
		
		double learned = 0;
		for (typename LearnedStateData::const_iterator it = stateData.begin(); it != stateData.end(); it++)
		{
			double thisState = (*it).second.hCost;
			if (learned < thisState)
				learned = thisState;
		}
		for (typename LearnedStateData::const_iterator it = stateData.begin(); it != stateData.end(); it++)
		{
			uint64_t node;
			dataLocation loc = (aoc.Lookup(m_pEnv->GetStateHash((*it).second.theState), node));
			//if (loc != kOpenList) continue;

			//if (loc == kOpenList)
			{
				if ((*it).second.dead)
					sprintf(str, " %1.1f", (*it).second.gCost);
				else
					sprintf(str, "%1.1f %1.1f", (*it).second.gCost, (*it).second.hCost+m_pEnv->HCost((*it).second.theState, theEnd));
				e->SetColor(0.9, 0.9, 0.9, 1);
				e->GLLabelState((*it).second.theState, str);
			}
			if ((*it).second.dead)
			{
				e->SetColor(0.0, 0.0+((loc==kOpenList)?0.5:0.0), 0.0, 1);
				e->OpenGLDraw((*it).second.theState);
			}
			else {
				double r = (*it).second.hCost;
				if (r > 0)
				{
					e->SetColor(0.5+0.5*r/learned, ((loc==kOpenList)?0.5:0.0), 0, 0.1+0.8*r/learned);
					e->OpenGLDraw((*it).second.theState);
				}
				else if (loc == kOpenList)
				{
					e->SetColor(0.0, 0.5, 0.0, 1);
					e->OpenGLDraw((*it).second.theState);
				}
			}
		}
	}
	
}
							
#endif
