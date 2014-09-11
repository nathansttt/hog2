/**
 * @file TemplateAStar.h
 * @package hog2
 * @brief A templated version of the original HOG's genericAstar.h
 * @author Nathan Sturtevant
 * SearchEnvironment
 * @date 3/22/06, modified 06/13/2007
 *
 * This file is part of HOG2.
 * HOG : http://www.cs.ualberta.ca/~nathanst/hog.html
 * HOG2: http://code.google.com/p/hog2/
 *
 * HOG2 is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG2; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef TemplateAStar_H
#define TemplateAStar_H

#define __STDC_CONSTANT_MACROS
#include <stdint.h>
// this is defined in stdint.h, but it doesn't always get defined correctly
// even when __STDC_CONSTANT_MACROS is defined before including stdint.h
// because stdint might be included elsewhere first...
#ifndef UINT32_MAX
#define UINT32_MAX        4294967295U
#endif

#include "FPUtil.h"
#include <ext/hash_map>
#include "AStarOpenClosed.h"
#include "BucketOpenClosed.h"
//#include "SearchEnvironment.h" // for the SearchEnvironment class
#include "float.h"

#include <algorithm> // for vector reverse

#include "GenericSearchAlgorithm.h"
static double lastF = 0;

template <class state>
struct AStarCompare {
	bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
	{
		if (fequal(i1.g+i1.h, i2.g+i2.h))
		{
			return (fless(i1.g, i2.g));
		}
		return (fgreater(i1.g+i1.h, i2.g+i2.h));
	}
};

/**
 * A templated version of A*, based on HOG genericAStar
 */
template <class state, class action, class environment>
class TemplateAStar : public GenericSearchAlgorithm<state,action,environment> {
public:
	TemplateAStar() { ResetNodeCount(); env = 0; useBPMX = 0; radius = 4.0; stopAfterGoal = true; weight=1; useRadius=false; useOccupancyInfo=false; radEnv = 0; reopenNodes = false; }
	virtual ~TemplateAStar() {}
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	
	void GetPath(environment *, const state& , const state& , std::vector<action> & ) { assert(false); };
	
	AStarOpenClosed<state, AStarCompare<state> > openClosedList;
	//BucketOpenClosed<state, AStarCompare<state> > openClosedList;
	state goal, start;
	
	bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	void AddAdditionalStartState(state& newState);
	void AddAdditionalStartState(state& newState, double cost);
	
	state CheckNextNode();
	void ExtractPathToStart(state &node, std::vector<state> &thePath)
	{ uint64_t theID; openClosedList.Lookup(env->GetStateHash(node), theID); ExtractPathToStartFromID(theID, thePath); }
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath);
	void DoAbstractSearch(){useOccupancyInfo = false; useRadius = false;}
	virtual const char *GetName();
	
	void PrintStats();
	uint64_t GetUniqueNodesExpanded() { return uniqueNodesExpanded; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; uniqueNodesExpanded = 0; }
	int GetMemoryUsage();
	
	bool GetClosedListGCost(const state &val, double &gCost) const;
	unsigned int GetNumOpenItems() { return openClosedList.OpenSize(); }
	inline const AStarOpenClosedData<state> &GetOpenItem(unsigned int which) { return openClosedList.Lookat(openClosedList.GetOpenItem(which)); }
	inline const int GetNumItems() { return openClosedList.size(); }
	inline const AStarOpenClosedData<state> &GetItem(unsigned int which) { return openClosedList.Lookat(which); }
	bool HaveExpandedState(const state &val)
	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key) != kNotFound; }
	
	void SetUseBPMX(int depth) { useBPMX = depth; if (depth) reopenNodes = true; }
	int GetUsingBPMX() { return useBPMX; }

	void SetReopenNodes(bool re) { reopenNodes = re; }
	bool GetReopenNodes() { return reopenNodes; }
	
	void SetHeuristic(Heuristic<state> *h) { theHeuristic = h; }
	
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	
	void LogFinalStats(StatCollection *) {}
	
	void SetRadius(double rad) {radius = rad;}
	double GetRadius() { return radius; }
	
	void SetRadiusEnvironment(environment *e) {radEnv = e;}
	
	void SetStopAfterGoal(bool val) { stopAfterGoal = val; }
	bool GetStopAfterGoal() { return stopAfterGoal; }
	
	void FullBPMX(uint64_t nodeID, int distance);
	
	void OpenGLDraw() const;
	
	void SetWeight(double w) {weight = w;}
private:
	uint64_t nodesTouched, nodesExpanded;
//	bool GetNextNode(state &next);
//	//state Node();
//	void UpdateClosedNode(environment *env, state& currOpenNode, state& neighbor);
//	void UpdateWeight(environment *env, state& currOpenNode, state& neighbor);
//	void AddToOpenList(environment *env, state& currOpenNode, state& neighbor);
	
	std::vector<state> neighbors;
	std::vector<uint64_t> neighborID;
	std::vector<double> edgeCosts;
	std::vector<dataLocation> neighborLoc;
	environment *env;
	bool stopAfterGoal;
	
	double radius; // how far around do we consider other agents?
	double weight; 
	
	bool useOccupancyInfo;// = false;
	bool useRadius;// = false;
	int useBPMX;
	bool reopenNodes;
	uint64_t uniqueNodesExpanded;
	environment *radEnv;
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
const char *TemplateAStar<state,action,environment>::GetName()
{
	static char name[32];
	sprintf(name, "TemplateAStar[]");
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
void TemplateAStar<state,action,environment>::GetPath(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
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
bool TemplateAStar<state,action,environment>::InitializeSearch(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	lastF = 0;
	
	theHeuristic = _env;
	thePath.resize(0);
	//if(useRadius)
	//std::cout<<"Using radius\n";
	env = _env;
	if(!radEnv)
		radEnv = _env;
	//	closedList.clear();
	//	openQueue.reset();
	//	assert(openQueue.size() == 0);
	//	assert(closedList.size() == 0);
	openClosedList.Reset();
	ResetNodeCount();
	start = from;
	goal = to;
	
	if (env->GoalTest(from, to) && (stopAfterGoal)) //assumes that from and to are valid states
	{
		return false;
	}
	
	openClosedList.AddOpenNode(start, env->GetStateHash(start), 0, weight*theHeuristic->HCost(start, goal));
	
	return true;
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search and before DoSingleSearchStep.
 * @author Nathan Sturtevant
 * @date 01/06/08
 */
template <class state, class action, class environment>
void TemplateAStar<state,action,environment>::AddAdditionalStartState(state& newState)
{
	openClosedList.AddOpenNode(newState, env->GetStateHash(newState), 0, weight*theHeuristic->HCost(start, goal));
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search
 * @author Nathan Sturtevant
 * @date 09/25/10
 */
template <class state, class action, class environment>
void TemplateAStar<state,action,environment>::AddAdditionalStartState(state& newState, double cost)
{
	openClosedList.AddOpenNode(newState, env->GetStateHash(newState), cost, weight*theHeuristic->HCost(start, goal));
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
template <class state, class action, class environment>
bool TemplateAStar<state,action,environment>::DoSingleSearchStep(std::vector<state> &thePath)
{
	if (openClosedList.OpenSize() == 0)
	{
		thePath.resize(0); // no path found!
		//closedList.clear();
		return true;
	}
	uint64_t nodeid = openClosedList.Close();
	if (openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h > lastF)
	{ lastF = openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h;
		//printf("Updated limit to %f\n", lastF);
	}
	if (!openClosedList.Lookup(nodeid).reopened)
		uniqueNodesExpanded++;
	nodesExpanded++;

	if ((stopAfterGoal) && (env->GoalTest(openClosedList.Lookup(nodeid).data, goal)))
	{
		ExtractPathToStartFromID(nodeid, thePath);
		// Path is backwards - reverse
		reverse(thePath.begin(), thePath.end()); 
		return true;
	}
	
 	neighbors.resize(0);
	edgeCosts.resize(0);
	neighborID.resize(0);
	neighborLoc.resize(0);
	
	//std::cout << "Expanding: " << openClosedList.Lookup(nodeid).data << " with f:";
	//std::cout << openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h << std::endl;
	
 	env->GetSuccessors(openClosedList.Lookup(nodeid).data, neighbors);
	double bestH = 0;
	// 1. load all the children
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		uint64_t theID;
		neighborLoc.push_back(openClosedList.Lookup(env->GetStateHash(neighbors[x]), theID));
		neighborID.push_back(theID);
		edgeCosts.push_back(env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]));
		if (useBPMX && (neighborLoc.back() != kNotFound))
		{
			// get best child h-cost
			bestH = std::max(bestH, openClosedList.Lookup(theID).h-edgeCosts.back());
		}
	}
	
	if (useBPMX) // propagate best child to parent
	{
		openClosedList.Lookup(nodeid).h = std::max(openClosedList.Lookup(nodeid).h, bestH); 
	}
	
	// iterate again updating costs and writing out to memory
	for (int x = 0; x < neighbors.size(); x++)
	{
		nodesTouched++;
		//double edgeCost;
		
		switch (neighborLoc[x])
		{
			case kClosedList:
				//edgeCost = env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]);
				
				if (useBPMX) // propagate best child to parent - do this before potentially re-opening
				{
					if (fless(openClosedList.Lookup(neighborID[x]).h, bestH-edgeCosts[x]))
					{
						openClosedList.Lookup(neighborID[x]).h = bestH-edgeCosts[x]; 
						if (useBPMX > 1) FullBPMX(neighborID[x], useBPMX-1);
					}
				}
				if (reopenNodes)
				{
					if (fless(openClosedList.Lookup(nodeid).g+edgeCosts[x], openClosedList.Lookup(neighborID[x]).g))
					{
						openClosedList.Lookup(neighborID[x]).parentID = nodeid;
						openClosedList.Lookup(neighborID[x]).g = openClosedList.Lookup(nodeid).g+edgeCosts[x];
						openClosedList.Reopen(neighborID[x]);
					}
				}
				break;
			case kOpenList:
				//edgeCost = env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]);
				if (fless(openClosedList.Lookup(nodeid).g+edgeCosts[x], openClosedList.Lookup(neighborID[x]).g))
				{
					openClosedList.Lookup(neighborID[x]).parentID = nodeid;
					openClosedList.Lookup(neighborID[x]).g = openClosedList.Lookup(nodeid).g+edgeCosts[x];
					openClosedList.KeyChanged(neighborID[x]);
					// TODO: unify the KeyChanged calls.
				}
				if (useBPMX) // propagate best child to parent
				{
					if (fgreater(bestH-edgeCosts[x], openClosedList.Lookup(neighborID[x]).h))
					{
						openClosedList.Lookup(neighborID[x]).h = std::max(openClosedList.Lookup(neighborID[x]).h, bestH-edgeCosts[x]); 
						openClosedList.KeyChanged(neighborID[x]);
					}
				}
				break;
			case kNotFound:
				// node is occupied; just mark it closed
				if (useRadius && useOccupancyInfo && env->GetOccupancyInfo() && radEnv && (radEnv->HCost(start, neighbors[x]) < radius) &&(env->GetOccupancyInfo()->GetStateOccupied(neighbors[x])) && ((!(radEnv->GoalTest(neighbors[x], goal)))))
				{
					//double edgeCost = env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]);
					openClosedList.AddClosedNode(neighbors[x],
												 env->GetStateHash(neighbors[x]),
												 openClosedList.Lookup(nodeid).g+edgeCosts[x],
												 std::max(theHeuristic->HCost(neighbors[x], goal), openClosedList.Lookup(nodeid).h-edgeCosts[x]),
												 nodeid);
				}
				else { // add node to open list
					//double edgeCost = env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]);
					if (useBPMX)
					{
						openClosedList.AddOpenNode(neighbors[x],
												   env->GetStateHash(neighbors[x]),
												   openClosedList.Lookup(nodeid).g+edgeCosts[x],
												   std::max(weight*theHeuristic->HCost(neighbors[x], goal), openClosedList.Lookup(nodeid).h-edgeCosts[x]),
												   nodeid);
					}
					else {
						openClosedList.AddOpenNode(neighbors[x],
												   env->GetStateHash(neighbors[x]),
												   openClosedList.Lookup(nodeid).g+edgeCosts[x],
												   weight*theHeuristic->HCost(neighbors[x], goal),
												   nodeid);
					}
//					if (loc == -1)
//					{ // duplicate edges
//						neighborLoc[x] = kOpenList;
//						x--;
//					}
				}
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
template <class state, class action, class environment>
state TemplateAStar<state, action,environment>::CheckNextNode()
{
	uint64_t key = openClosedList.Peek();
	return openClosedList.Lookup(key).data;
	//assert(false);
	//return openQueue.top().currNode;
}

/**
 * Perform a full bpmx propagation
 * @author Nathan Sturtevant
 * @date 6/9/9
 * 
 * @return The first state in the open list. 
 */
template <class state, class action, class environment>
void TemplateAStar<state, action,environment>::FullBPMX(uint64_t nodeID, int distance)
{
	if (distance <= 0)
		return;
	
	nodesExpanded++;
	std::vector<state> succ;
 	env->GetSuccessors(openClosedList.Lookup(nodeID).data, succ);
	double parentH = openClosedList.Lookup(nodeID).h;
	
	// load all the children and push parent heuristic value to children
	for (unsigned int x = 0; x < succ.size(); x++)
	{
		uint64_t theID;
		dataLocation loc = openClosedList.Lookup(env->GetStateHash(succ[x]), theID);
		double edgeCost = env->GCost(openClosedList.Lookup(nodeID).data, succ[x]);
		double newHCost = parentH-edgeCost;
		
		switch (loc)
		{
			case kClosedList:
			{
				if (fgreater(newHCost, openClosedList.Lookup(theID).h))
				{
					openClosedList.Lookup(theID).h = newHCost;
					FullBPMX(theID, distance-1);
				}
			}
			case kOpenList:
			{
				if (fgreater(newHCost, openClosedList.Lookup(theID).h))
				{
					openClosedList.Lookup(theID).h = newHCost;
					openClosedList.KeyChanged(theID);
				}
			}
			case kNotFound: break;
		}
	}
}


/**
 * Get the path from a goal state to the start state 
 * @author Nathan Sturtevant
 * @date 03/22/06
 * 
 * @param goalNode the goal state
 * @param thePath will contain the path from goalNode to the start state
 */
template <class state, class action,class environment>
void TemplateAStar<state, action,environment>::ExtractPathToStartFromID(uint64_t node,
																	 std::vector<state> &thePath)
{
	do {
		thePath.push_back(openClosedList.Lookup(node).data);
		node = openClosedList.Lookup(node).parentID;
	} while (openClosedList.Lookup(node).parentID != node);
	thePath.push_back(openClosedList.Lookup(node).data);
}

/**
 * A function that prints the number of states in the closed list and open
 * queue. 
 * @author Nathan Sturtevant
 * @date 03/22/06
 */
template <class state, class action, class environment>
void TemplateAStar<state, action,environment>::PrintStats()
{
	printf("%u items in closed list\n", (unsigned int)openClosedList.ClosedSize());
	printf("%u items in open queue\n", (unsigned int)openClosedList.OpenSize());
}

/**
 * Return the amount of memory used by TemplateAStar
 * @author Nathan Sturtevant
 * @date 03/22/06
 * 
 * @return The combined number of elements in the closed list and open queue
 */
template <class state, class action, class environment>
int TemplateAStar<state, action,environment>::GetMemoryUsage()
{
	return openClosedList.size();
}

/**
 * Get state from the closed list
 * @author Nathan Sturtevant
 * @date 10/09/07
 * 
 * @param val The state to lookup in the closed list
 * @gCost The g-cost of the node in the closed list
 * @return success Whether we found the value or not
 * more states
 */
template <class state, class action, class environment>
bool TemplateAStar<state, action,environment>::GetClosedListGCost(const state &val, double &gCost) const
{
	uint64_t theID;
	dataLocation loc = openClosedList.Lookup(env->GetStateHash(val), theID);
	if (loc == kClosedList)
	{
		gCost = openClosedList.Lookat(theID).g;
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
void TemplateAStar<state, action,environment>::OpenGLDraw() const
{
	double transparency = 1.0;
	if (openClosedList.size() == 0)
		return;
	uint64_t top = -1;
//	double minf = 1e9, maxf = 0;
	if (openClosedList.OpenSize() > 0)
	{
		top = openClosedList.Peek();
	}
//	for (unsigned int x = 0; x < openClosedList.size(); x++)
//	{
//		const AStarOpenClosedData<state> &data = openClosedList.Lookat(x);
//		double f = data.g+data.h;
//		if (f > maxf)
//			maxf = f;
//		if (f < minf)
//			minf = f;
//	}
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const AStarOpenClosedData<state> &data = openClosedList.Lookat(x);
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
		if ((data.where == kOpenList) && (data.reopened))
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
			env->SetColor(1.0, 0.0, 0.0, transparency);
//			}
			env->OpenGLDraw(data.data);
		}
	}
}

#endif
