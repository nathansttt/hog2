/*
 *  LSSLRTAStar.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/6/09.
 *  Copyright 2009 NS Software. All rights reserved.
 *
 */

#ifndef LSSLRTASTAR_H
#define LSSLRTASTAR_H

#include "GenericSearchAlgorithm.h"
#include "FPUtil.h"
#include <deque>
#include <vector>
#include <ext/hash_map>
#include "TemplateAStar.h"
#include <queue>
#include <iostream>

static bool verbose = false;

template <class state>
class borderData {
public:
	borderData(const state &s, const double h) :theState(s), heuristic(h) {}
	state theState;
	double heuristic;
};

template <class state>
class compareBorderData
{
public:
	bool operator() (const borderData<state> &lhs, const borderData<state> &rhs) const
	{
		return (lhs.heuristic > rhs.heuristic);
	}
};


template <class state, class action, class environment>
class LSSLRTAStar : public GenericSearchAlgorithm<state,action,environment>, public Heuristic<state> {
public:
	LSSLRTAStar(int nodeLimit = 8)
	{ fAmountLearned = 0.0f; nodeExpansionLimit = nodeLimit; }
	virtual ~LSSLRTAStar(void) { }
	
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	void GetPath(environment *, const state& , const state& , std::vector<action> & ) { assert(false); };
	virtual const char *GetName() { static char name[255]; sprintf(name, "LSSLRTAStar(%d)", nodeExpansionLimit); return name; }
	void SetHCost(environment *env, state &where, double val) { heur[env->GetStateHash(where)] = val; }
	double HCost(environment *env, const state &from, const state &to) { return std::max(heur[env->GetStateHash(from)], env->HCost(from, to)); }
	double HCost(const state &from, const state &to) { return std::max(heur[m_pEnv->GetStateHash(from)], m_pEnv->HCost(from, to)); }
	
	virtual uint64_t GetNodesExpanded() { return nodesExpanded; }
	virtual uint64_t GetNodesTouched() { return nodesTouched; }
	virtual void LogFinalStats(StatCollection *s) { s->AddStat("TotalLearning", GetName(),fAmountLearned); }
	
	double GetAmountLearned() { return fAmountLearned; }
	void OpenGLDraw() const {}
	void OpenGLDraw(const environment *env) const;
private:
	typedef __gnu_cxx::hash_map<uint64_t, double, Hash64 > LearnedHeuristic;
	typedef __gnu_cxx::hash_map<uint64_t, bool, Hash64 > ClosedList;
	
	environment *m_pEnv;
	LearnedHeuristic heur;
	double fAmountLearned;
	uint64_t nodesExpanded, nodesTouched;
	int nodeExpansionLimit;
	TemplateAStar<state, action, environment> astar;
};

/** The core routine of LSSLRTAStar -- computes at most one-move path */
template <class state, class action, class environment>
void LSSLRTAStar<state, action, environment>::LSSLRTAStar<state, action, environment>::GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath)
{
	m_pEnv = env;
	thePath.resize(0);
	if (from==to)
		return;
	
	astar.InitializeSearch(env, from, to, thePath);
	astar.SetHeuristic(this);
	for (int x = 0; x < nodeExpansionLimit; x++)
		if (astar.DoSingleSearchStep(thePath))
		{
			// return reversed path
			std::reverse(thePath.begin(), thePath.end());
			break;
		}
	nodesExpanded = astar.GetNodesExpanded();
	nodesTouched = astar.GetNodesTouched();
	
	if (thePath.size() != 0)
		return;

	// 1. put all open nodes in pqueue
	typedef std::priority_queue<borderData<state>,std::vector<borderData<state> >,compareBorderData<state> > pQueue;	
	pQueue q;
	
	unsigned int openSize = astar.GetNumOpenItems();
	for (unsigned int x = 0; x < openSize; x++)
	{
		const AStarOpenClosedData<state> data = astar.GetOpenItem(x);
		q.push(borderData<state>(data.data, data.h));
		if (verbose) std::cout << "Preparing border state: " << data.data << " h: " << data.h << std::endl;
	}
	
	std::vector<state> succ;
	state first = q.top().theState;
	ClosedList c;
	while (q.size() > 0)
	{
		nodesExpanded++;
		nodesTouched++;
		state s = q.top().theState;
		if (verbose) std::cout << "Starting with " << s << " h: " << q.top().heuristic << "/" << HCost(s, to) << std::endl;
		q.pop();
		//			std::cout << s << " " << learnData[env->GetStateHash(s)].learnedHeuristic << std::endl;
		env->GetSuccessors(s, succ);
		double hCost = HCost(s, to);
		for (unsigned int x = 0; x < succ.size(); x++)
		{
			nodesTouched++;
			double succHCost;
			if (!astar.GetClosedListGCost(succ[x], succHCost))
			{
				if (verbose) std::cout << succ[x] << " not in closed\n";
				continue;
			}
			double edgeCost = env->GCost(s, succ[x]);
			succHCost = HCost(env, succ[x], to);
			if (c[env->GetStateHash(succ[x])]) // in closed list, but seen before, update if smaller
			{
				if (verbose) std::cout << succ[x] << " updated before ";
				if (fless(hCost + edgeCost, succHCost))
				{
					fAmountLearned -= succHCost-hCost-edgeCost;
					if (verbose) std::cout << "lowering cost to " << hCost + edgeCost;
					if (verbose) std::cout << " learning now " << fAmountLearned;
					SetHCost(env, succ[x], hCost + edgeCost);
					q.push(borderData<state>(succ[x], hCost + edgeCost));
				}
				if (verbose) std::cout << std::endl;
			}
			else { // not expanded before and in closed list, always update
				// 2. expand successors, checking if they are in closed list and adding to queue
//				if (fless(succHCost, hCost - edgeCost))
				if (verbose) std::cout << succ[x] << " NOT updated before ";
				//if (fgreater(hCost + edgeCost, succHCost))
				{
					if (verbose) std::cout << "setting cost to " << hCost + edgeCost << " over " << succHCost;
					fAmountLearned += (edgeCost + hCost) - succHCost;
					if (verbose) std::cout << " learning now " << fAmountLearned;
					SetHCost(env, succ[x], hCost + edgeCost);
					q.push(borderData<state>(succ[x], hCost + edgeCost));
					c[env->GetStateHash(succ[x])] = true;
				}
				if (verbose) std::cout << std::endl;
			}
		}
	}
	
	// 3. construct best path
	astar.ExtractPathToStart(first, thePath);
	if (verbose) std::cout << "LSS-LRTA* heading towards " << thePath[0] << "with h-value " << HCost(env, thePath[0], to) << std::endl;
}

template <class state, class action, class environment>
void LSSLRTAStar<state, action, environment>::LSSLRTAStar<state, action, environment>::OpenGLDraw(const environment *) const
{
	astar.OpenGLDraw();
	//	for (typename LearnedHeuristic::const_iterator it = hashTable.begin(); it != hashTable.end(); it++)
	//	{
	//		if ((*it).second.theState == currentLoc)
	//		{
	//		}
	//		else {
	//			e->OpenGLDraw((*it).second.theState);
	//		}
	//	}
}

#endif
