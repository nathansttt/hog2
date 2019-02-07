//
//  AStarEpsilon.h
//  HOG2 Demos
//
//  Created by Nathan Sturtevant on 5/24/16.
//  Copyright © 2016 NS Software. All rights reserved.
//

#ifndef AStarEpsilon_h
#define AStarEpsilon_h


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

template <class state>
struct GBFSCompare {
	bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
	{
		if (fequal(i1.h, i2.h))
		{
			return (fless(i1.g, i2.g));
		}
		return (fgreater(i1.h, i2.h));
	}
};


/**
 * A templated version of A*, based on HOG genericAStar
 */
template <class state, class action, class environment>
class AStarEpsilon : public GenericSearchAlgorithm<state,action,environment> {
public:
	AStarEpsilon(double optimalBound = 2) { ResetNodeCount(); env = 0; bound = optimalBound; theHeuristic = 0;}
	virtual ~AStarEpsilon() {}
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	void GetPath(environment *, const state& , const state& , std::vector<action> & );
	
	// uses admissible heuristic (regular A* search)
	AStarOpenClosed<state, AStarCompare<state>> f;
	// uses inadmissible heuristic
	AStarOpenClosed<state, GBFSCompare<state>> focal;
	state goal, start;
	
	bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	
	void ExtractPathToStart(state &node, std::vector<state> &thePath)
	{ uint64_t theID; focal.Lookup(env->GetStateHash(node), theID); ExtractPathToStartFromID(theID, thePath); }
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath);
	const state &GetParent(const state &s);
	virtual const char *GetName();
	
	void PrintStats();
	uint64_t GetUniqueNodesExpanded() { return uniqueNodesExpanded; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; uniqueNodesExpanded = 0; }
	int GetMemoryUsage();
	
	state CheckNextOpenNode();
	state CheckNextFocalNode();
	bool GetOpenListGCost(const state &val, double &gCost) const;
	bool GetFocalListGCost(const state &val, double &gCost) const;

	bool GetClosedListGCost(const state &val, double &gCost) const;
	bool GetClosedItem(const state &s, AStarOpenClosedData<state> &);
	unsigned int GetNumOpenItems() { return f.OpenSize(); }
	unsigned int GetNumFocalItems() { return focal.OpenSize(); }
	inline const AStarOpenClosedData<state> &GetOpenItem(unsigned int which) { return f.Lookat(f.GetOpenItem(which)); }
	inline const AStarOpenClosedData<state> &GetFocalItem(unsigned int which) { return focal.Lookat(focal.GetOpenItem(which)); }
	inline const int GetNumItems() { return focal.size(); }
	inline const AStarOpenClosedData<state> &GetItem(unsigned int which) { return focal.Lookat(which); }
	bool HaveExpandedState(const state &val)
	{ uint64_t key; return focal.Lookup(env->GetStateHash(val), key) != kNotFound; }
	dataLocation GetStateLocation(const state &val)
	{ uint64_t key; return focal.Lookup(env->GetStateHash(val), key); }
	
	void SetHeuristic(Heuristic<state> *h) { theHeuristic = h; }
	
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	
	void LogFinalStats(StatCollection *) {}
	
	void OpenGLDraw() const;
	void Draw(Graphics::Display &d) const;
	
	void SetOptimalityBound(double w) {bound = w;}
	double GetOptimalityBound() { return bound; }
private:
	void DrawOpen(Graphics::Display &d) const;
	void DrawFocal(Graphics::Display &d) const;
	void ExpandOpen();
	void ExpandFocal();
	uint64_t nodesTouched, nodesExpanded;
	
	std::vector<state> neighbors;
	std::vector<uint64_t> neighborID;
	std::vector<double> edgeCosts;
	std::vector<dataLocation> neighborLoc;

	std::vector<state> solution;
	environment *env;
	double bestSolution;
	double bound;
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
const char *AStarEpsilon<state,action,environment>::GetName()
{
	static char name[32];
	sprintf(name, "AStarEpsilon[%1.2f]", bound);
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
void AStarEpsilon<state,action,environment>::GetPath(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
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

template <class state, class action, class environment>
void AStarEpsilon<state,action,environment>::GetPath(environment *_env, const state& from, const state& to, std::vector<action> &path)
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
template <class state, class action, class environment>
bool AStarEpsilon<state,action,environment>::InitializeSearch(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	bestSolution = DBL_MAX;
	
	if (theHeuristic == 0)
		theHeuristic = _env;
	thePath.resize(0);
	env = _env;
	focal.Reset(env->GetMaxHash());
	f.Reset(env->GetMaxHash());
	solution.clear();
	ResetNodeCount();
	start = from;
	goal = to;
	
	if (env->GoalTest(from, to)) //assumes that from and to are valid states
	{
		return false;
	}
	
	focal.AddOpenNode(start, env->GetStateHash(start), 0, theHeuristic->HCost(start, goal));
	f.AddOpenNode(start, env->GetStateHash(start), 0, theHeuristic->HCost(start, goal));
	
	return true;
}

///**
// * Add additional start state to the search. This should only be called after Initialize Search and before DoSingleSearchStep.
// * @author Nathan Sturtevant
// * @date 01/06/08
// */
//template <class state, class action, class environment>
//void AStarEpsilon<state,action,environment>::AddAdditionalStartState(state& newState)
//{
//	focal.AddOpenNode(newState, env->GetStateHash(newState), 0, weight*theHeuristic->HCost(start, goal));
//	f.AddOpenNode(newState, env->GetStateHash(newState), 0, theHeuristic->HCost(start, goal));
//}
//
///**
// * Add additional start state to the search. This should only be called after Initialize Search
// * @author Nathan Sturtevant
// * @date 09/25/10
// */
//template <class state, class action, class environment>
//void AStarEpsilon<state,action,environment>::AddAdditionalStartState(state& newState, double cost)
//{
//	focal.AddOpenNode(newState, env->GetStateHash(newState), cost, weight*theHeuristic->HCost(start, goal));
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
bool AStarEpsilon<state,action,environment>::DoSingleSearchStep(std::vector<state> &thePath)
{
	if (solution.size() > 0)
	{
		thePath = solution;
		return true;
	}
	if (focal.OpenSize() == 0)
	{
		printf("No path\n");
		thePath.resize(0); // no path found!
		return true;
	}
	
	uint64_t nodeOnfocal;
	uint64_t nodeOnF;
	// only reopen states taken from f, not fhat
	bool reopen = true;

	double regularF = f.Lookat(f.Peek()).g+f.Lookat(f.Peek()).h;
	double focalF = focal.Lookat(focal.Peek()).g+focal.Lookat(focal.Peek()).h;
	printf("Min f: %1.2f; next focal f: %1.2f. Max allowable: %1.2f.", regularF, focalF, regularF*bound);
	if (flesseq(focalF, bound*regularF))
	{
		//printf(" Expanding focal\n");
		ExpandFocal();
	}
	else {
		//printf(" Expanding open\n");
		ExpandOpen();
	}
	
	if (solution.size() > 0)
	{
		thePath = solution;
		return true;
	}

//	if (fless(focal.Lookat(focal.Peek()).g+focal.Lookat(focal.Peek()).h, bestSolution))
//	{
//		nodeOnfocal = focal.Close();
//		reopen = false;
//		dataLocation d = f.Lookup(env->GetStateHash(focal.Lookup(nodeOnfocal).data), nodeOnF);
//		assert(d != kNotFound);
////		f.Close(nodeOnF);
//	}
//	else {
//		nodeOnF = f.Close();
//		reopen = true;
//		dataLocation d = focal.Lookup(env->GetStateHash(f.Lookup(nodeOnF).data), nodeOnfocal);
//		assert(d != kNotFound);
//		if (d == kOpenList)
//		{
//			focal.Close(nodeOnfocal);
//			d = focal.Lookup(env->GetStateHash(f.Lookup(nodeOnF).data), nodeOnfocal);
//			assert(d == kClosedList);
//		}
//	}
//
//	if (!fequal(oldF, f.Lookat(f.Peek()).g+f.Lookat(f.Peek()).h))
//	{
//		printf("Best solution %1.2f\n", bestSolution);
//		printf("Best on open %1.2f - lower bound is %1.2f\n", f.Lookat(f.Peek()).g+f.Lookat(f.Peek()).h,
//			   (f.Lookat(f.Peek()).g+f.Lookat(f.Peek()).h)*bound);
//	}
//	if (!focal.Lookup(nodeOnfocal).reopened)
//		uniqueNodesExpanded++;
//	nodesExpanded++;
//
//	//	if n is a goal then
//	//		incumbent ← n
//	if (env->GoalTest(focal.Lookup(nodeOnfocal).data, goal))
//	{
//		// Actually extract the path, since the cost may not actually be the g-cost
//		ExtractPathToStartFromID(nodeOnfocal, thePath);
//		bestSolution = env->GetPathLength(thePath);
//		thePath.resize(0);
//		printf("Best solution %1.2f\n", bestSolution);
//		printf("Best on open %1.2f - lower bound is %1.2f\n", f.Lookat(f.Peek()).g+f.Lookat(f.Peek()).h,
//			   (f.Lookat(f.Peek()).g+f.Lookat(f.Peek()).h)*bound);
//
//		return false; // check on next iteration through the loop
//	}

	
	//	std::cout << "Expanding: " << focal.Lookup(nodeOnfocal).data << " with f:";
	//	std::cout << focal.Lookup(nodeid).g+focal.Lookup(nodeOnfocal).h << std::endl;
	
	return false;
}

/**
 * Expands a single state from the given open list
 * @author Nathan Sturtevant
 * @date 01/27/19
 */
template <class state, class action, class environment>
void AStarEpsilon<state,action,environment>::ExpandOpen()
{
	uint64_t nodeid = f.Close();
	if (!f.Lookup(nodeid).reopened)
		uniqueNodesExpanded++;
	nodesExpanded++;
	
	if (env->GoalTest(f.Lookup(nodeid).data, goal))
	{
		ExtractPathToStartFromID(nodeid, solution);
		// Path is backwards - reverse
		reverse(solution.begin(), solution.end());
		return;
	}
	
	neighbors.resize(0);
	edgeCosts.resize(0);
	neighborID.resize(0);
	neighborLoc.resize(0);
	
	//	std::cout << "Expanding: " << f.Lookup(nodeid).data << " with f:";
	//	std::cout << f.Lookup(nodeid).g+f.Lookup(nodeid).h << std::endl;
	
	env->GetSuccessors(f.Lookup(nodeid).data, neighbors);
	double bestH = f.Lookup(nodeid).h;
	double lowHC = DBL_MAX;
	// 1. load all the children
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		uint64_t theID;
		neighborLoc.push_back(f.Lookup(env->GetStateHash(neighbors[x]), theID));
		neighborID.push_back(theID);
		edgeCosts.push_back(env->GCost(f.Lookup(nodeid).data, neighbors[x]));
	}
	
	// iterate again updating costs and writing out to memory
	for (int x = 0; x < neighbors.size(); x++)
	{
		nodesTouched++;
		
		switch (neighborLoc[x])
		{
			case kClosedList:
//				if (reopenNodes)
//				{
//					if (fless(f.Lookup(nodeid).g+edgeCosts[x], f.Lookup(neighborID[x]).g))
//					{
//						f.Lookup(neighborID[x]).parentID = nodeid;
//						f.Lookup(neighborID[x]).g = f.Lookup(nodeid).g+edgeCosts[x];
//						f.Reopen(neighborID[x]);
//						// This line isn't normally needed, but in some state spaces we might have
//						// equality but different meta information, so we need to make sure that the
//						// meta information is also copied, since this is the most generic A* implementation
//						f.Lookup(neighborID[x]).data = neighbors[x];
//					}
//				}
				break;
			case kOpenList:
				//edgeCost = env->GCost(f.Lookup(nodeid).data, neighbors[x]);
				if (fless(f.Lookup(nodeid).g+edgeCosts[x], f.Lookup(neighborID[x]).g))
				{
					f.Lookup(neighborID[x]).parentID = nodeid;
					f.Lookup(neighborID[x]).g = f.Lookup(nodeid).g+edgeCosts[x];
					// This line isn't normally needed, but in some state spaces we might have
					// equality but different meta information, so we need to make sure that the
					// meta information is also copied, since this is the most generic A* implementation
					f.Lookup(neighborID[x]).data = neighbors[x];
					f.KeyChanged(neighborID[x]);
					//					std::cout << " Reducing cost to " << f.Lookup(nodeid).g+edgeCosts[x] << "\n";
					// TODO: unify the KeyChanged calls.
				}
				else {
					//					std::cout << " no cheaper \n";
				}
				break;
			case kNotFound:
			{
				f.AddOpenNode(neighbors[x],
							  env->GetStateHash(neighbors[x]),
							  f.Lookup(nodeid).g+edgeCosts[x],
							  theHeuristic->HCost(neighbors[x], goal),
							  nodeid);
			}
		}
	}
}

template <class state, class action, class environment>
void AStarEpsilon<state,action,environment>::ExpandFocal()
{
	uint64_t nodeid = focal.Close();
	if (!focal.Lookup(nodeid).reopened)
		uniqueNodesExpanded++;
	nodesExpanded++;
	
	if (env->GoalTest(focal.Lookup(nodeid).data, goal))
	{
		ExtractPathToStartFromID(nodeid, solution);
		// Path is backwards - reverse
		reverse(solution.begin(), solution.end());
		return;
	}
	
	neighbors.resize(0);
	edgeCosts.resize(0);
	neighborID.resize(0);
	neighborLoc.resize(0);
	
	std::cout << "Expanding: " << focal.Lookup(nodeid).data << " with f:";
	std::cout << focal.Lookup(nodeid).g+focal.Lookup(nodeid).h << std::endl;
	
	env->GetSuccessors(focal.Lookup(nodeid).data, neighbors);
	double bestH = focal.Lookup(nodeid).h;
	double lowHC = DBL_MAX;
	// 1. load all the children
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		uint64_t theID;
		neighborLoc.push_back(focal.Lookup(env->GetStateHash(neighbors[x]), theID));
		neighborID.push_back(theID);
		edgeCosts.push_back(env->GCost(focal.Lookup(nodeid).data, neighbors[x]));
	}
	
	// iterate again updating costs and writing out to memory
	for (int x = 0; x < neighbors.size(); x++)
	{
		nodesTouched++;
		
		switch (neighborLoc[x])
		{
			case kClosedList:
				break;
			case kOpenList:
				//edgeCost = env->GCost(focal.Lookup(nodeid).data, neighbors[x]);
				if (fless(focal.Lookup(nodeid).g+edgeCosts[x], focal.Lookup(neighborID[x]).g))
				{
					focal.Lookup(neighborID[x]).parentID = nodeid;
					focal.Lookup(neighborID[x]).g = focal.Lookup(nodeid).g+edgeCosts[x];
					// This line isn't normally needed, but in some state spaces we might have
					// equality but different meta information, so we need to make sure that the
					// meta information is also copied, since this is the most generic A* implementation
					focal.Lookup(neighborID[x]).data = neighbors[x];
					focal.KeyChanged(neighborID[x]);
					//					std::cout << " Reducing cost to " << focal.Lookup(nodeid).g+edgeCosts[x] << "\n";
					// TODO: unify the KeyChanged calls.
				}
				else {
					//					std::cout << " no cheaper \n";
				}
				break;
			case kNotFound:
			{
				focal.AddOpenNode(neighbors[x],
							  env->GetStateHash(neighbors[x]),
							  focal.Lookup(nodeid).g+edgeCosts[x],
							  theHeuristic->HCost(neighbors[x], goal),
							  nodeid);
			}
		}
	}
}

/**
 * Returns the next state on the open list (but doesn't pop it off the queue).
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The first state in the open list.
 */
//template <class state, class action, class environment>
//state AStarEpsilon<state, action,environment>::CheckNextNode()
//{
//	uint64_t key = focal.Peek();
//	return focal.Lookup(key).data;
//	//assert(false);
//	//return openQueue.top().currNode;
//}


/**
 * Get the path from a goal state to the start state
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @param goalNode the goal state
 * @param thePath will contain the path from goalNode to the start state
 */
template <class state, class action,class environment>
void AStarEpsilon<state, action,environment>::ExtractPathToStartFromID(uint64_t node,
																				 std::vector<state> &thePath)
{
	do {
		thePath.push_back(focal.Lookup(node).data);
		node = focal.Lookup(node).parentID;
	} while (focal.Lookup(node).parentID != node);
	thePath.push_back(focal.Lookup(node).data);
}

template <class state, class action,class environment>
const state &AStarEpsilon<state, action,environment>::GetParent(const state &s)
{
	uint64_t theID;
	focal.Lookup(env->GetStateHash(s), theID);
	theID = focal.Lookup(theID).parentID;
	return focal.Lookup(theID).data;
}

/**
 * A function that prints the number of states in the closed list and open
 * queue.
 * @author Nathan Sturtevant
 * @date 03/22/06
 */
template <class state, class action, class environment>
void AStarEpsilon<state, action,environment>::PrintStats()
{
	printf("%u items in closed list\n", (unsigned int)focal.ClosedSize());
	printf("%u items in open queue\n", (unsigned int)focal.OpenSize());
}

/**
 * Return the amount of memory used by AStarEpsilon
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The combined number of elements in the closed list and open queue
 */
template <class state, class action, class environment>
int AStarEpsilon<state, action,environment>::GetMemoryUsage()
{
	return focal.size();
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
template <class state, class action, class environment>
bool AStarEpsilon<state, action,environment>::GetClosedListGCost(const state &val, double &gCost) const
{
	uint64_t theID;
	dataLocation loc = focal.Lookup(env->GetStateHash(val), theID);
	if (loc == kClosedList)
	{
		gCost = focal.Lookat(theID).g;
		return true;
	}
	return false;
}

template <class state, class action, class environment>
state AStarEpsilon<state, action,environment>::CheckNextOpenNode()
{
	uint64_t key = f.Peek();
	return f.Lookup(key).data;
}

template <class state, class action, class environment>
state AStarEpsilon<state, action,environment>::CheckNextFocalNode()
{
	uint64_t key = focal.Peek();
	return focal.Lookup(key).data;
}


template <class state, class action, class environment>
bool AStarEpsilon<state, action,environment>::GetOpenListGCost(const state &val, double &gCost) const
{
	uint64_t theID;
	dataLocation loc = f.Lookup(env->GetStateHash(val), theID);
	if (loc == kOpenList)
	{
		gCost = f.Lookat(theID).g;
		return true;
	}
	return false;
}

template <class state, class action, class environment>
bool AStarEpsilon<state, action,environment>::GetFocalListGCost(const state &val, double &gCost) const
{
	uint64_t theID;
	dataLocation loc = focal.Lookup(env->GetStateHash(val), theID);
	if (loc == kOpenList)
	{
		gCost = focal.Lookat(theID).g;
		return true;
	}
	return false;
}


template <class state, class action, class environment>
bool AStarEpsilon<state, action,environment>::GetClosedItem(const state &s, AStarOpenClosedData<state> &result)
{
	uint64_t theID;
	dataLocation loc = focal.Lookup(env->GetStateHash(s), theID);
	if (loc == kClosedList)
	{
		result = focal.Lookat(theID);
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
template <class state, class action, class environment>
void AStarEpsilon<state, action,environment>::OpenGLDraw() const
{
	double transparency = 1.0;
	if (focal.size() == 0)
		return;
	uint64_t top = -1;
	double bound = DBL_MAX;

	//	double minf = 1e9, maxf = 0;
	if (focal.OpenSize() > 0)
	{
		top = focal.Peek();
		const auto &i = f.Lookat(f.Peek());
		bound = i.g+i.h;
		printf("Lowest f on open: %f\n", bound);
	}
	for (unsigned int x = 0; x < focal.size(); x++)
	{
		const auto &data = focal.Lookat(x);
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
//		if ((data.where == kClosedList && !fgreater(data.g+data.h/weight, bound)))
//		{
//			env->SetColor(0.0, 0.0, 1.0, transparency);
//			env->OpenGLDraw(data.data);
//		}
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

template <class state, class action, class environment>
void AStarEpsilon<state, action,environment>::Draw(Graphics::Display &d) const
{
	DrawFocal(d);
	DrawOpen(d);
	env->SetColor(1.0, 0.5, 1.0, 0.5);
	env->Draw(d, goal);
}

template <class state, class action, class environment>
void AStarEpsilon<state, action,environment>::DrawOpen(Graphics::Display &d) const
{
	double transparency = 1.0;
	if (f.size() == 0)
		return;
	uint64_t top = -1;
	double bound = DBL_MAX;
	
	//	double minf = 1e9, maxf = 0;
	if (f.OpenSize() > 0)
	{
		top = f.Peek();
		const auto &i = f.Lookat(f.Peek());
		bound = i.g+i.h;
//		printf("Lowest f on open: %f\n", bound);
	}
	for (unsigned int x = 0; x < f.size(); x++)
	{
		const auto &data = f.Lookat(x);
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->Draw(d, data.data);
		}
//		if ((data.where == kClosedList && !fgreater(data.g+data.h/weight, bound)))
//		{
//			env->SetColor(0.0, 0.0, 1.0, transparency);
//			env->Draw(d, data.data);
//		}
		if ((data.where == kOpenList) && (data.reopened))
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->Draw(d, data.data);
		}
		else if (data.where == kOpenList)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->Draw(d, data.data);
		}
		else if ((data.where == kClosedList) && (data.reopened))
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			env->Draw(d, data.data);
		}
		else if (data.where == kClosedList)
		{
			if (data.parentID == x)
				env->SetColor(1.0, 0.5, 0.5, transparency);
			else
				env->SetColor(0.0, 0.0, 1.0, transparency);
			env->Draw(d, data.data);
		}
	}
}

template <class state, class action, class environment>
void AStarEpsilon<state, action,environment>::DrawFocal(Graphics::Display &d) const
{
	double transparency = 1.0;
	if (focal.size() == 0)
		return;
	uint64_t top = -1;
	double bound = DBL_MAX;
	
	//	double minf = 1e9, maxf = 0;
	if (focal.OpenSize() > 0)
	{
		top = focal.Peek();
		const auto &i = focal.Lookat(focal.Peek());
		bound = i.g+i.h;
		//		printf("Lowest f on open: %f\n", bound);
	}
	for (unsigned int x = 0; x < focal.size(); x++)
	{
		const auto &data = focal.Lookat(x);
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->Draw(d, data.data);
		}
		//		if ((data.where == kClosedList && !fgreater(data.g+data.h/weight, bound)))
		//		{
		//			env->SetColor(0.0, 0.0, 1.0, transparency);
		//			env->Draw(d, data.data);
		//		}
		if ((data.where == kOpenList) && (data.reopened))
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->Draw(d, data.data);
		}
		else if (data.where == kOpenList)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->Draw(d, data.data);
		}
		else if ((data.where == kClosedList) && (data.reopened))
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			env->Draw(d, data.data);
		}
		else if (data.where == kClosedList)
		{
			if (data.parentID == x)
				env->SetColor(1.0, 0.5, 0.5, transparency);
			else
				env->SetColor(1.0, 0.0, 0.0, transparency);
			env->Draw(d, data.data);
		}
	}
}
#endif /* AStarEpsilon_h */
