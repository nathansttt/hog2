//
//  DynamicPotentialSearch.h
//  HOG2 Demos
//
//  Created by Nathan Sturtevant on 5/24/16.
//  Copyright © 2016 NS Software. All rights reserved.
//

#ifndef DynamicPotentialSearch_h
#define DynamicPotentialSearch_h


#include <iostream>
#include "FPUtil.h"
#include <unordered_map>
#include "AStarOpenClosed.h"
#include "BucketOpenClosed.h"
#include "TemplateAStar.h"
//#include "SearchEnvironment.h" // for the SearchEnvironment class
#include "float.h"
#include <algorithm> // for vector reverse
#include "GenericSearchAlgorithm.h"
#include <unordered_map>

/**
 * A templated version of A*, based on HOG genericAStar
 */
//template <class state>
//struct DPSCompare {
//	bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
//	{
//		if (fequal(i1.g+i1.h, i2.g+i2.h))
//		{
//			return (fless(i1.g, i2.g));
//		}
//		return (fgreater(i1.g+i1.h, i2.g+i2.h));
//	}
//};
template<typename state>
class DPSData {
public:
	DPSData() {}
	DPSData(const state &theData, double gCost, double hCost, const state &par)
	:data(theData), g(gCost), h(hCost), parent(par), open(true), reopened(false) {}
	state data;
	double g;
	double h;
	state parent;
	bool open;
	bool reopened;
};



template <class state, class action, class environment>
class DynamicPotentialSearch {
public:
	DynamicPotentialSearch() { ResetNodeCount(); env = 0; weight=3; bound = 1.5; theHeuristic = 0;}
	virtual ~DynamicPotentialSearch() {}
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	void GetPath(environment *, const state& , const state& , std::vector<action> & );
	
//	AStarOpenClosed<state, AStarCompare<state>> open;
	std::unordered_map<uint64_t, DPSData<state>> openClosed;
	typename std::unordered_map<uint64_t, DPSData<state>>::const_iterator iter;
	state goal, start;
	
	bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
//	void AddAdditionalStartState(state& newState);
//	void AddAdditionalStartState(state& newState, double cost);
	
//	state CheckNextNode();
	void ExtractPathToStart(const state &node, std::vector<state> &thePath)
	{
		thePath.push_back(node);
		const auto &i = openClosed[env->GetStateHash(node)];
		if (i.parent == node)
			return;
		ExtractPathToStart(i.parent, thePath);
	}
	const state &GetParent(const state &s);
	virtual const char *GetName();
	
	void PrintStats();
//	uint64_t GetUniqueNodesExpanded() { return uniqueNodesExpanded; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; uniqueNodesExpanded = 0; }
	int GetMemoryUsage();

	double GetBestFMin();
	void ResetIterator();
	bool GetNext(double &g, double &h);

//	bool GetClosedListGCost(const state &val, double &gCost) const;
//	bool GetOpenListGCost(const state &val, double &gCost) const;
//	bool GetClosedItem(const state &s, AStarOpenClosedData<state> &);
//	unsigned int GetNumOpenItems() { return fhat.OpenSize(); }
//	inline const AStarOpenClosedData<state> &GetOpenItem(unsigned int which) { return fhat.Lookat(fhat.GetOpenItem(which)); }
//	inline const int GetNumItems() { return fhat.size(); }
//	inline const AStarOpenClosedData<state> &GetItem(unsigned int which) { return fhat.Lookat(which); }
//	bool HaveExpandedState(const state &val)
//	{ uint64_t key; return fhat.Lookup(env->GetStateHash(val), key) != kNotFound; }
//	dataLocation GetStateLocation(const state &val)
//	{ uint64_t key; return fhat.Lookup(env->GetStateHash(val), key); }
	
	void SetReopenNodes(bool re) { reopenNodes = re; }
	bool GetReopenNodes() { return reopenNodes; }
	
	void SetHeuristic(Heuristic<state> *h) { theHeuristic = h; }
	
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	
//	void LogFinalStats(StatCollection *) {}
	
	void OpenGLDraw() const;
	void Draw(Graphics::Display &d) const;
	
//	void SetWeight(double w) {weight = w;}
	void SetOptimalityBound(double w) {bound = w;}
	double GetOptimalityBound() {return bound;}
private:
	uint64_t nodesTouched, nodesExpanded;
	
	std::vector<state> neighbors;
//	std::vector<uint64_t> neighborID;
//	std::vector<double> edgeCosts;
//	std::vector<dataLocation> neighborLoc;
	environment *env;
	double bestSolution;
	double weight;
	double bound;
	bool reopenNodes;
	uint64_t uniqueNodesExpanded;
	Heuristic<state> *theHeuristic;

};

//static const bool verbose = false;

/**
 * Return the name of the algorithm.
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The name of the algorithm
 */

template <class state, class action, class environment>
const char *DynamicPotentialSearch<state,action,environment>::GetName()
{
	static char name[32];
	sprintf(name, "DynamicPotentialSearch[%1.2f, %1.2f]", bound, weight);
	return name;
}

/**
 * Perform an A* search between two states.
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @param _env The search environment
 * @param from The start state
 * @param to The goal state
 * @param thePath A vector of states which will contain an optimal path
 * between from and to when the function returns, if one exists.
 */
template <class state, class action, class environment>
void DynamicPotentialSearch<state,action,environment>::GetPath(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	//discardcount=0;
	if (!InitializeSearch(_env, from, to, thePath))
	{
		return;
	}
	while (!DoSingleSearchStep(thePath))
	{ }
}

template <class state, class action, class environment>
void DynamicPotentialSearch<state,action,environment>::GetPath(environment *_env, const state& from, const state& to, std::vector<action> &path)
{
	std::vector<state> thePath;
	if (!InitializeSearch(_env, from, to, thePath))
	{
		return;
	}
	path.resize(0);
	while (!DoSingleSearchStep(thePath))
	{	}
	for (int x = 0; x < thePath.size()-1; x++)
	{
		path.push_back(_env->GetAction(thePath[x], thePath[x+1]));
	}
}


/**
 * Initialize the A* search
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @param _env The search environment
 * @param from The start state
 * @param to The goal state
 * @return TRUE if initialization was successful, FALSE otherwise
 */
template <class state, class action, class environment>
bool DynamicPotentialSearch<state,action,environment>::InitializeSearch(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	bestSolution = DBL_MAX;
	
	if (theHeuristic == 0)
		theHeuristic = _env;
	thePath.resize(0);
	env = _env;
	openClosed.clear();
	ResetNodeCount();
	start = from;
	goal = to;
	
	if (env->GoalTest(from, to)) //assumes that from and to are valid states
	{
		return false;
	}
	openClosed[env->GetStateHash(start)] = {start, 0, theHeuristic->HCost(start, goal), start};
	
	return true;
}

///**
// * Add additional start state to the search. This should only be called after Initialize Search and before DoSingleSearchStep.
// * @author Nathan Sturtevant
// * @date 01/06/08
// */
//template <class state, class action, class environment>
//void DynamicPotentialSearch<state,action,environment>::AddAdditionalStartState(state& newState)
//{
//	fhat.AddOpenNode(newState, env->GetStateHash(newState), 0, weight*theHeuristic->HCost(start, goal));
//	f.AddOpenNode(newState, env->GetStateHash(newState), 0, theHeuristic->HCost(start, goal));
//}
//
///**
// * Add additional start state to the search. This should only be called after Initialize Search
// * @author Nathan Sturtevant
// * @date 09/25/10
// */
//template <class state, class action, class environment>
//void DynamicPotentialSearch<state,action,environment>::AddAdditionalStartState(state& newState, double cost)
//{
//	fhat.AddOpenNode(newState, env->GetStateHash(newState), cost, weight*theHeuristic->HCost(start, goal));
//	f.AddOpenNode(newState, env->GetStateHash(newState), cost, theHeuristic->HCost(start, goal));
//}

/**
 * Expand a single node.
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @param thePath will contain an optimal path from start to goal if the
 * function returns TRUE
 * @return TRUE if there is no path or if we have found the goal, FALSE
 * otherwise
 */
template <class state, class action, class environment>
bool DynamicPotentialSearch<state,action,environment>::DoSingleSearchStep(std::vector<state> &thePath)
{
	
	double fmin = DBL_MAX;
	// get best f on open
	for (const auto &item : openClosed)
	{
		auto &i = item.second;
		if (i.open == true && fless(i.g+i.h, fmin))
			fmin = i.g+i.h;
	}
	
	// get best priority
	double bestP = 0;
	DPSData<state> *next = 0;
	for (auto &item : openClosed)
	{
		auto &i = item.second;
		// //	 (B × fmin − g(n))/h(n)
		if (i.open)
		{
			double pr = DBL_MAX;
			if (i.h != 0)
				pr = (bound * fmin - i.g)/i.h;
			if (fgreater(pr, bestP))
			{
				bestP = pr;
				next = &i;
			}
		}
	}
	if (next == 0)
	{
		// no path found
		return true;
	}
//	std::cout << "Expanding " << next->data << " with priority " << bestP << "\n";
	next->open = false;
	
	nodesExpanded++;
	if (env->GoalTest(next->data, goal))
	{
		ExtractPathToStart(next->data, thePath);
		return true;
	}
	
	env->GetSuccessors(next->data, neighbors);

	// iterate again updating costs and writing out to memory
	for (int x = 0; x < neighbors.size(); x++)
	{
		uint64_t hash = env->GetStateHash(neighbors[x]);
		double edgeCost = env->GCost(next->data, neighbors[x]);
		nodesTouched++;
		
		auto item = openClosed.find(hash);
		
		if (item == openClosed.end()) // not found
		{
			openClosed[hash] = {neighbors[x], next->g+edgeCost, theHeuristic->HCost(neighbors[x], goal), next->data};
			continue;
		}
		auto &i = item->second;
		if (fless(next->g+edgeCost+i.h, i.g+i.h)) // found shorter path
		{
			i.parent = next->data;
			i.g = next->g+edgeCost;
			if (i.open == false)
			{
				i.open = true;
				i.reopened = true;
			}
		}
	}
	return false;
}

template <class state, class action, class environment>
double DynamicPotentialSearch<state, action,environment>::GetBestFMin()
{
	double fmin = DBL_MAX;
	// get best f on open
	for (const auto &item : openClosed)
	{
		auto &i = item.second;
		if (i.open == true && fless(i.g+i.h, fmin))
			fmin = i.g+i.h;
	}
	//std::cout << "-->Best fmnin " << fmin << "\n";
	return fmin;
}

template <class state, class action, class environment>
void DynamicPotentialSearch<state, action,environment>::ResetIterator()
{
	iter = openClosed.begin();
	while ((iter != openClosed.end()) && iter->second.open == false)
	{
		iter++;
	}
}

template <class state, class action, class environment>
bool DynamicPotentialSearch<state, action,environment>::GetNext(double &g, double &h)
{
	if (iter == openClosed.end())
		return false;
	g = iter->second.g;
	h = iter->second.h;
	//std::cout << " " << iter->second.data;
	iter++;
	while ((iter != openClosed.end()) && iter->second.open == false)
	{
		iter++;
	}
	return true;
}

///**
// * Returns the next state on the open list (but doesn't pop it off the queue).
// * @author Nathan Sturtevant
// * @date 03/22/06
// *
// * @return The first state in the open list.
// */
//template <class state, class action, class environment>
//state DynamicPotentialSearch<state, action,environment>::CheckNextNode()
//{
//	uint64_t key = fhat.Peek();
//	return fhat.Lookup(key).data;
//	//assert(false);
//	//return openQueue.top().currNode;
//}


///**
// * Get the path from a goal state to the start state
// * @author Nathan Sturtevant
// * @date 03/22/06
// *
// * @param goalNode the goal state
// * @param thePath will contain the path from goalNode to the start state
// */
//template <class state, class action,class environment,class openList>
//void DynamicPotentialSearch<state, action,environment>::ExtractPathToStartFromID(uint64_t node,
//																				 std::vector<state> &thePath)
//{
//	do {
//		thePath.push_back(fhat.Lookup(node).data);
//		node = fhat.Lookup(node).parentID;
//	} while (fhat.Lookup(node).parentID != node);
//	thePath.push_back(fhat.Lookup(node).data);
//}

//template <class state, class action,class environment,class openList>
//const state &DynamicPotentialSearch<state, action,environment>::GetParent(const state &s)
//{
//	uint64_t theID;
//	fhat.Lookup(env->GetStateHash(s), theID);
//	theID = fhat.Lookup(theID).parentID;
//	return fhat.Lookup(theID).data;
//}

///**
// * A function that prints the number of states in the closed list and open
// * queue.
// * @author Nathan Sturtevant
// * @date 03/22/06
// */
//template <class state, class action, class environment>
//void DynamicPotentialSearch<state, action,environment>::PrintStats()
//{
//	printf("%u items in closed list\n", (unsigned int)fhat.ClosedSize());
//	printf("%u items in open queue\n", (unsigned int)fhat.OpenSize());
//}
//
///**
// * Return the amount of memory used by DynamicPotentialSearch
// * @author Nathan Sturtevant
// * @date 03/22/06
// *
// * @return The combined number of elements in the closed list and open queue
// */
//template <class state, class action, class environment>
//int DynamicPotentialSearch<state, action,environment>::GetMemoryUsage()
//{
//	return fhat.size();
//}

///**
// * Get state from the closed list
// * @author Nathan Sturtevant
// * @date 10/09/07
// *
// * @param val The state to lookup in the closed list
// * @gCost The g-cost of the node in the closed list
// * @return success Whether we found the value or not
// * the states
// */
//template <class state, class action, class environment>
//bool DynamicPotentialSearch<state, action,environment>::GetClosedListGCost(const state &val, double &gCost) const
//{
//	uint64_t theID;
//	dataLocation loc = fhat.Lookup(env->GetStateHash(val), theID);
//	if (loc == kClosedList)
//	{
//		gCost = fhat.Lookat(theID).g;
//		return true;
//	}
//	return false;
//}

//template <class state, class action, class environment>
//bool DynamicPotentialSearch<state, action,environment>::GetOpenListGCost(const state &val, double &gCost) const
//{
//	uint64_t theID;
//	dataLocation loc = fhat.Lookup(env->GetStateHash(val), theID);
//	if (loc == kOpenList)
//	{
//		gCost = fhat.Lookat(theID).g;
//		return true;
//	}
//	return false;
//}

//template <class state, class action, class environment>
//bool DynamicPotentialSearch<state, action,environment>::GetClosedItem(const state &s, AStarOpenClosedData<state> &result)
//{
//	uint64_t theID;
//	dataLocation loc = fhat.Lookup(env->GetStateHash(s), theID);
//	if (loc == kClosedList)
//	{
//		result = fhat.Lookat(theID);
//		return true;
//	}
//	return false;
//
//}


/**
 * Draw the open/closed list
 * @author Nathan Sturtevant
 * @date 03/12/09
 *
 */
template <class state, class action, class environment>
void DynamicPotentialSearch<state, action,environment>::OpenGLDraw() const
{

}

template <class state, class action, class environment>
void DynamicPotentialSearch<state, action,environment>::Draw(Graphics::Display &d) const
{
	double transparency = 1.0;
	
	for (const auto &item : openClosed)
	{
		const auto &i = item.second;
		if (i.open && i.reopened)
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->Draw(d, i.data);
		}
		else if (i.open)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->Draw(d, i.data);
		}
		else if (!i.open && i.reopened)
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			env->Draw(d, i.data);
		}
		else if (!i.open)
		{
			env->SetColor(1.0, 0.0, 0.0, transparency);
			env->Draw(d, i.data);
		}
	}
	env->SetColor(1.0, 0.5, 1.0, 0.5);
	env->Draw(d, goal);
}

#endif /* DynamicPotentialSearch_h */
