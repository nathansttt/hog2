//
//  OptimisitcSearch.h
//  HOG2 Demos
//
//  Created by Nathan Sturtevant on 5/24/16.
//  Copyright © 2016 NS Software. All rights reserved.
//

#ifndef OptimisitcSearch_h
#define OptimisitcSearch_h


#include <iostream>
#include "FPUtil.h"
#include <ext/hash_map>
#include "AStarOpenClosed.h"
#include "BucketOpenClosed.h"
#include "TemplateAStar.h"
//#include "SearchEnvironment.h" // for the SearchEnvironment class
#include "float.h"
#include <algorithm> // for vector reverse
#include "GenericSearchAlgorithm.h"

/**
 * A templated version of A*, based on HOG genericAStar
 */
template <class state, class action, class environment, class openList = AStarOpenClosed<state, AStarCompare<state>> >
class OptimisticSearch : public GenericSearchAlgorithm<state,action,environment> {
public:
	OptimisticSearch() { ResetNodeCount(); env = 0; weight=3; bound = 1.5; theHeuristic = 0;}
	virtual ~OptimisticSearch() {}
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	void GetPath(environment *, const state& , const state& , std::vector<action> & );
	
	openList f;
	openList fhat;
	//AStarOpenClosed<state, AStarCompare<state> > openClosedList;
	//BucketOpenClosed<state, AStarCompare<state> > openClosedList;
	state goal, start;
	
	bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	void AddAdditionalStartState(state& newState);
	void AddAdditionalStartState(state& newState, double cost);
	
	state CheckNextNode();
	void ExtractPathToStart(state &node, std::vector<state> &thePath)
	{ uint64_t theID; fhat.Lookup(env->GetStateHash(node), theID); ExtractPathToStartFromID(theID, thePath); }
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath);
	const state &GetParent(const state &s);
	virtual const char *GetName();
	
	void PrintStats();
	uint64_t GetUniqueNodesExpanded() { return uniqueNodesExpanded; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; uniqueNodesExpanded = 0; }
	int GetMemoryUsage();
	
	bool GetClosedListGCost(const state &val, double &gCost) const;
	bool GetOpenListGCost(const state &val, double &gCost) const;
	bool GetClosedItem(const state &s, AStarOpenClosedData<state> &);
	unsigned int GetNumOpenItems() { return fhat.OpenSize(); }
	inline const AStarOpenClosedData<state> &GetOpenItem(unsigned int which) { return fhat.Lookat(fhat.GetOpenItem(which)); }
	inline const int GetNumItems() { return fhat.size(); }
	inline const AStarOpenClosedData<state> &GetItem(unsigned int which) { return fhat.Lookat(which); }
	bool HaveExpandedState(const state &val)
	{ uint64_t key; return fhat.Lookup(env->GetStateHash(val), key) != kNotFound; }
	dataLocation GetStateLocation(const state &val)
	{ uint64_t key; return fhat.Lookup(env->GetStateHash(val), key); }
	
	void SetReopenNodes(bool re) { reopenNodes = re; }
	bool GetReopenNodes() { return reopenNodes; }
	
	void SetHeuristic(Heuristic<state> *h) { theHeuristic = h; }
	
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	
	void LogFinalStats(StatCollection *) {}
	
	void OpenGLDraw() const;
	
	void SetWeight(double w) {weight = w;}
	void SetOptimalityBound(double w) {bound = w;}
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

template <class state, class action, class environment, class openList>
const char *OptimisticSearch<state,action,environment,openList>::GetName()
{
	static char name[32];
	sprintf(name, "OptimisticSearch[%1.2f, %1.2f]", bound, weight);
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
template <class state, class action, class environment, class openList>
void OptimisticSearch<state,action,environment,openList>::GetPath(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	//discardcount=0;
	if (!InitializeSearch(_env, from, to, thePath))
	{
		return;
	}
	while (!DoSingleSearchStep(thePath))
	{
		//		if (0 == nodesExpanded%100000)
		//			printf("%llu nodes expanded\n", nodesExpanded);
	}
}

template <class state, class action, class environment, class openList>
void OptimisticSearch<state,action,environment,openList>::GetPath(environment *_env, const state& from, const state& to, std::vector<action> &path)
{
	std::vector<state> thePath;
	if (!InitializeSearch(_env, from, to, thePath))
	{
		return;
	}
	path.resize(0);
	while (!DoSingleSearchStep(thePath))
	{
	}
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
template <class state, class action, class environment, class openList>
bool OptimisticSearch<state,action,environment,openList>::InitializeSearch(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	bestSolution = DBL_MAX;
	
	if (theHeuristic == 0)
		theHeuristic = _env;
	thePath.resize(0);
	env = _env;
	//	closedList.clear();
	//	openQueue.reset();
	//	assert(openQueue.size() == 0);
	//	assert(closedList.size() == 0);
	fhat.Reset(env->GetMaxHash());
	f.Reset(env->GetMaxHash());
	ResetNodeCount();
	start = from;
	goal = to;
	
	if (env->GoalTest(from, to)) //assumes that from and to are valid states
	{
		return false;
	}
	
	fhat.AddOpenNode(start, env->GetStateHash(start), 0, weight*theHeuristic->HCost(start, goal));
	f.AddOpenNode(start, env->GetStateHash(start), 0, theHeuristic->HCost(start, goal));
	
	return true;
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search and before DoSingleSearchStep.
 * @author Nathan Sturtevant
 * @date 01/06/08
 */
template <class state, class action, class environment, class openList>
void OptimisticSearch<state,action,environment,openList>::AddAdditionalStartState(state& newState)
{
	fhat.AddOpenNode(newState, env->GetStateHash(newState), 0, weight*theHeuristic->HCost(start, goal));
	f.AddOpenNode(newState, env->GetStateHash(newState), 0, theHeuristic->HCost(start, goal));
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search
 * @author Nathan Sturtevant
 * @date 09/25/10
 */
template <class state, class action, class environment, class openList>
void OptimisticSearch<state,action,environment,openList>::AddAdditionalStartState(state& newState, double cost)
{
	fhat.AddOpenNode(newState, env->GetStateHash(newState), cost, weight*theHeuristic->HCost(start, goal));
	f.AddOpenNode(newState, env->GetStateHash(newState), cost, theHeuristic->HCost(start, goal));
}

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
template <class state, class action, class environment, class openList>
bool OptimisticSearch<state,action,environment,openList>::DoSingleSearchStep(std::vector<state> &thePath)
{
	if (fhat.OpenSize() == 0)
	{
		thePath.resize(0); // no path found!
		//closedList.clear();
		return true;
	}

	// proven within bound
	if (bound*(f.Lookat(f.Peek()).g+f.Lookat(f.Peek()).h) >= bestSolution)
	{
		ExtractPathToStart(goal, thePath);
		// Path is backwards - reverse
		reverse(thePath.begin(), thePath.end());
		return true;
	}
	
//	openf ← {initial}
//	openfhat  ← {initial}
//	incumbent ← ∞
//	repeat until bound · f (first on openf ) ≥ f (incumbent):
//	
//	if f (first on open fhat) < f (incumbent) then
//		n ← remove first on openfhat
//		remove n from openf
//	else n ← remove first on openf
//		remove n from openfhat
//	add n to closed
	
	uint64_t nodeid;
	uint64_t f_id;
	bool reopen = true;

	if (fless(fhat.Lookat(fhat.Peek()).g+fhat.Lookat(fhat.Peek()).h, bestSolution))
	{
		nodeid = fhat.Close();
		
		dataLocation d = f.Lookup(env->GetStateHash(fhat.Lookup(nodeid).data), f_id);
		assert(d != kNotFound);
		f.Close(f_id);
	}
	else {
		f_id = f.Close();
		reopen = true;
		dataLocation d = fhat.Lookup(env->GetStateHash(fhat.Lookup(f_id).data), nodeid);
		assert(d != kNotFound);
		fhat.Close(nodeid);
	}
	
	if (!fhat.Lookup(nodeid).reopened)
		uniqueNodesExpanded++;
	nodesExpanded++;
	
	//	if n is a goal then
	//		incumbent ← n
	if (env->GoalTest(fhat.Lookup(nodeid).data, goal))
	{
		// Actually extract the path, since the cost may not actually be the g-cost
		ExtractPathToStartFromID(nodeid, thePath);
		bestSolution = env->GetPathLength(thePath);
		return false; // check on next iteration through the loop
	}

	
	//	std::cout << "Expanding: " << fhat.Lookup(nodeid).data << " with f:";
	//	std::cout << fhat.Lookup(nodeid).g+fhat.Lookup(nodeid).h << std::endl;
	
	env->GetSuccessors(fhat.Lookup(nodeid).data, neighbors);

	// iterate again updating costs and writing out to memory
	for (int x = 0; x < neighbors.size(); x++)
	{
		uint64_t childID_fhat, childID_f;
		dataLocation d_fhat, d_f;

		d_fhat = fhat.Lookup(env->GetStateHash(neighbors[x]), childID_fhat);
		d_f = fhat.Lookup(env->GetStateHash(neighbors[x]), childID_f);
		double edgeCost = env->GCost(fhat.Lookup(nodeid).data, neighbors[x]);
		
		nodesTouched++;
		
		//	else for each child c of n
		//		if c is duplicated in openf then
		//			if c is better than the duplicate then
		//				replace copies in openf and openfhat
		//		else if c is duplicated in closed then
		//			if c is better than the duplicate then
		//				add c to openf and openfhat
		//          else add c to openf and openfhat

		if (d_f == kOpenList) // update cost if on open in
		{
			assert(d_fhat == kOpenList);
			
			if (fless(f.Lookup(f_id).g+edgeCost, f.Lookup(childID_f).g))
			{
				// update in open
				f.Lookup(childID_f).parentID = f_id;
				f.Lookup(childID_f).g = f.Lookup(f_id).g+edgeCost;
				f.Lookup(childID_f).data = neighbors[x];
				f.KeyChanged(childID_f);

				// update in open_hat
				fhat.Lookup(childID_fhat).parentID = nodeid;
				fhat.Lookup(childID_fhat).g = fhat.Lookup(nodeid).g+edgeCost;
				fhat.Lookup(childID_fhat).data = neighbors[x];
				fhat.KeyChanged(childID_fhat);
			}
		}
		else if (d_f == kClosedList)
		{
			assert(d_fhat == kClosedList);

			if (fless(f.Lookup(f_id).g+edgeCost, f.Lookup(childID_f).g) && reopen)
			{
				f.Lookup(childID_f).parentID = f_id;
				f.Lookup(childID_f).g = f.Lookup(f_id).g+edgeCost;
				f.Reopen(childID_f);
				f.Lookup(childID_f).data = neighbors[x];

				fhat.Lookup(childID_fhat).parentID = nodeid;
				fhat.Lookup(childID_fhat).g = fhat.Lookup(nodeid).g+edgeCost;
				fhat.Reopen(childID_fhat);
				fhat.Lookup(childID_fhat).data = neighbors[x];
			}
		}
		else {
			assert(d_fhat == kNotFound);
			assert(d_f == kNotFound);
			fhat.AddOpenNode(neighbors[x],
							 env->GetStateHash(neighbors[x]),
							 fhat.Lookup(nodeid).g+edgeCost,
							 weight*theHeuristic->HCost(neighbors[x], goal),
							 nodeid);

			f.AddOpenNode(neighbors[x],
						  env->GetStateHash(neighbors[x]),
						  f.Lookup(f_id).g+edgeCost,
						  theHeuristic->HCost(neighbors[x], goal),
						  nodeid);
		}
	}
	return false;
}

/**
 * Returns the next state on the open list (but doesn't pop it off the queue).
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The first state in the open list.
 */
template <class state, class action, class environment, class openList>
state OptimisticSearch<state, action,environment,openList>::CheckNextNode()
{
	uint64_t key = fhat.Peek();
	return fhat.Lookup(key).data;
	//assert(false);
	//return openQueue.top().currNode;
}


/**
 * Get the path from a goal state to the start state
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @param goalNode the goal state
 * @param thePath will contain the path from goalNode to the start state
 */
template <class state, class action,class environment,class openList>
void OptimisticSearch<state, action,environment,openList>::ExtractPathToStartFromID(uint64_t node,
																				 std::vector<state> &thePath)
{
	do {
		thePath.push_back(fhat.Lookup(node).data);
		node = fhat.Lookup(node).parentID;
	} while (fhat.Lookup(node).parentID != node);
	thePath.push_back(fhat.Lookup(node).data);
}

template <class state, class action,class environment,class openList>
const state &OptimisticSearch<state, action,environment,openList>::GetParent(const state &s)
{
	uint64_t theID;
	fhat.Lookup(env->GetStateHash(s), theID);
	theID = fhat.Lookup(theID).parentID;
	return fhat.Lookup(theID).data;
}

/**
 * A function that prints the number of states in the closed list and open
 * queue.
 * @author Nathan Sturtevant
 * @date 03/22/06
 */
template <class state, class action, class environment, class openList>
void OptimisticSearch<state, action,environment,openList>::PrintStats()
{
	printf("%u items in closed list\n", (unsigned int)fhat.ClosedSize());
	printf("%u items in open queue\n", (unsigned int)fhat.OpenSize());
}

/**
 * Return the amount of memory used by OptimisticSearch
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The combined number of elements in the closed list and open queue
 */
template <class state, class action, class environment, class openList>
int OptimisticSearch<state, action,environment,openList>::GetMemoryUsage()
{
	return fhat.size();
}

/**
 * Get state from the closed list
 * @author Nathan Sturtevant
 * @date 10/09/07
 *
 * @param val The state to lookup in the closed list
 * @gCost The g-cost of the node in the closed list
 * @return success Whether we found the value or not
 * the states
 */
template <class state, class action, class environment, class openList>
bool OptimisticSearch<state, action,environment,openList>::GetClosedListGCost(const state &val, double &gCost) const
{
	uint64_t theID;
	dataLocation loc = fhat.Lookup(env->GetStateHash(val), theID);
	if (loc == kClosedList)
	{
		gCost = fhat.Lookat(theID).g;
		return true;
	}
	return false;
}

template <class state, class action, class environment, class openList>
bool OptimisticSearch<state, action,environment,openList>::GetOpenListGCost(const state &val, double &gCost) const
{
	uint64_t theID;
	dataLocation loc = fhat.Lookup(env->GetStateHash(val), theID);
	if (loc == kOpenList)
	{
		gCost = fhat.Lookat(theID).g;
		return true;
	}
	return false;
}

template <class state, class action, class environment, class openList>
bool OptimisticSearch<state, action,environment,openList>::GetClosedItem(const state &s, AStarOpenClosedData<state> &result)
{
	uint64_t theID;
	dataLocation loc = fhat.Lookup(env->GetStateHash(s), theID);
	if (loc == kClosedList)
	{
		result = fhat.Lookat(theID);
		return true;
	}
	return false;
	
}


/**
 * Draw the open/closed list
 * @author Nathan Sturtevant
 * @date 03/12/09
 *
 */
template <class state, class action, class environment, class openList>
void OptimisticSearch<state, action,environment,openList>::OpenGLDraw() const
{
	double transparency = 1.0;
	if (fhat.size() == 0)
		return;
	uint64_t top = -1;
	double bound = DBL_MAX;

	//	double minf = 1e9, maxf = 0;
	if (fhat.OpenSize() > 0)
	{
		top = fhat.Peek();
		const auto &i = f.Lookat(f.Peek());
		bound = i.g+i.h;
		printf("Lowest f on open: %f\n", bound);
	}
	for (unsigned int x = 0; x < fhat.size(); x++)
	{
		const auto &data = fhat.Lookat(x);
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
		if ((data.where == kClosedList && !fgreater(data.g+data.h/weight, bound)))
		{
			env->SetColor(0.0, 0.0, 1.0, transparency);
			env->OpenGLDraw(data.data);
		}
		else if ((data.where == kOpenList) && (data.reopened))
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->OpenGLDraw(data.data);
		}
		else if (data.where == kOpenList)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
		else if ((data.where == kClosedList) && (data.reopened))
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			env->OpenGLDraw(data.data);
		}
		else if (data.where == kClosedList)
		{
			//			if (top != -1)
			//			{
			//				env->SetColor((data.g+data.h-minf)/(maxf-minf), 0.0, 0.0, transparency);
			//			}
			//			else {
			if (data.parentID == x)
				env->SetColor(1.0, 0.5, 0.5, transparency);
			else
				env->SetColor(1.0, 0.0, 0.0, transparency);
			//			}
			env->OpenGLDraw(data.data);
		}
	}
	env->SetColor(1.0, 0.5, 1.0, 0.5);
	env->OpenGLDraw(goal);
}


#endif /* OptimisticSearch_h */
