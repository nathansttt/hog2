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
	TemplateAStar() { ResetNodeCount(); env = 0; useBPMX = 0; radius = 4.0; stopAfterGoal = true; weight=1; useRadius=false; useOccupancyInfo=false; radEnv = 0;}
	virtual ~TemplateAStar() {}
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	
	void GetPath(environment *, const state& , const state& , std::vector<action> & ) { assert(false); };
	
	AStarOpenClosed<state, AStarCompare<state> > openClosedList;
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
	
	//closedList_iterator GetClosedListIter() const;
	//	void GetClosedListIter(closedList_iterator);
	//	bool ClosedListIterNext(closedList_iterator& it, state& next) const;
	//bool GetClosedListGCost(state &val, double &gCost) const;
	bool GetClosedListGCost(const state &val, double &gCost) const;
	unsigned int GetNumOpenItems() { return openClosedList.OpenSize(); }
	inline const AStarOpenClosedData<state> &GetOpenItem(unsigned int which) { return openClosedList.Lookat(openClosedList.GetOpenItem(which)); }
	inline const int GetNumItems() { return openClosedList.size(); }
	inline const AStarOpenClosedData<state> &GetItem(unsigned int which) { return openClosedList.Lookat(which); }
	bool HaveExpandedState(const state &val)
	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key) != kNotFound; }
	
	void SetUseBPMX(int depth) { useBPMX = depth; }
	int GetUsingBPMX() { return useBPMX; }
	
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
	bool GetNextNode(state &next);
	//state Node();
	void UpdateClosedNode(environment *env, state& currOpenNode, state& neighbor);
	void UpdateWeight(environment *env, state& currOpenNode, state& neighbor);
	void AddToOpenList(environment *env, state& currOpenNode, state& neighbor);
	
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
  	while (!DoSingleSearchStep(thePath)) {}
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
	
	//SearchNode<state> first(env->heuristic(goal, start), 0, start, start);
	//SearchNode<state> first(start, start, weight*env->HCost(start, goal), 0, env->GetStateHash(start));
	//openQueue.Add(first);
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
	//	SearchNode<state> first(newState, newState, weight*env->HCost(goal, newState), 0,env->GetStateHash(newState));
	//	openQueue.Add(first);
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
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		nodesTouched++;
		//double edgeCost;
		
		switch (neighborLoc[x])
		{
			case kClosedList:
				break;
				//edgeCost = env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]);
				
				if (useBPMX) // propagate best child to parent - do this before potentially re-opening
				{
					if (fless(openClosedList.Lookup(neighborID[x]).h, bestH-edgeCosts[x]))
					{
						openClosedList.Lookup(neighborID[x]).h = bestH-edgeCosts[x]; 
						if (useBPMX > 1) FullBPMX(neighborID[x], useBPMX-1);
					}
				}
				if (fless(openClosedList.Lookup(nodeid).g+edgeCosts[x], openClosedList.Lookup(neighborID[x]).g))
				{
					openClosedList.Lookup(neighborID[x]).parentID = nodeid;
					openClosedList.Lookup(neighborID[x]).g = openClosedList.Lookup(nodeid).g+edgeCosts[x];
					openClosedList.Reopen(neighborID[x]);
				}
				break;
			case kOpenList:
				//edgeCost = env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]);
				if (fless(openClosedList.Lookup(nodeid).g+edgeCosts[x], openClosedList.Lookup(neighborID[x]).g))
				{
					openClosedList.Lookup(neighborID[x]).parentID = nodeid;
					openClosedList.Lookup(neighborID[x]).g = openClosedList.Lookup(nodeid).g+edgeCosts[x];
					openClosedList.KeyChanged(neighborID[x]);
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
					openClosedList.AddOpenNode(neighbors[x],
											   env->GetStateHash(neighbors[x]),
											   openClosedList.Lookup(nodeid).g+edgeCosts[x],
											   std::max(theHeuristic->HCost(neighbors[x], goal), openClosedList.Lookup(nodeid).h-edgeCosts[x]),
											   nodeid);
				}
		}
	}
	
	// should only do this if a "find best path" option is set
	//	if ((openQueue.size() == 0) && (stopAfterGoal == false))
	//	{
	//		ExtractPathToStart(currentOpenNode, thePath);
	//		// Path is backwards - reverse
	//		reverse(thePath.begin(), thePath.end()); 
	//		return true;
	//	}
	
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


///**
// * Removes the top node from the open list  
// * @author Nathan Sturtevant
// * @date 03/22/06
// *
// * @param state will contain the next state in the open list
// * @return TRUE if next contains a valid state, FALSE if there is no  more states in the
// * open queue 
// */
//template <class state, class action, class environment>
//bool TemplateAStar<state,action,environment>::GetNextNode(state &next)
//{
//	nodesExpanded++;
//	if(openQueue.Empty())
//		return false;
//	SearchNode<state> it = openQueue.Remove();
//	//if(it == openQueue.end())
//	//	return false;
//	next = it.currNode;
//	//printf("h-cost\t%f\n", it.fCost-it.gCost);
//	
//	//	std::cout << "Current open node " << next << " f-cost: " << it.fCost << " g-cost: " << it.gCost <<
//	//	" h-value: " << env->HCost(next, goal) << "/" << it.fCost-it.gCost << std::endl;
//	
//	closedList[env->GetStateHash(next)] = it;
//	//	std::cout<<"Getting "<<it.gCost<<" ";
//	return true;
//}

///**
// * Check and update the weight of a closed node. 
// * @author Nathan Sturtevant
// * @date 11/10/08
// * 
// * @param currOpenNode The node that's currently being expanded
// * @param neighbor The node whose weight will be updated
// */
//template <class state, class action, class environment>
//void TemplateAStar<state,action,environment>::UpdateClosedNode(environment *e, state &currOpenNode, state &neighbor)
//{
//	//printf("Found in closed list!\n");
//	SearchNode<state> prev = closedList[e->GetStateHash(neighbor)];
//	//openQueue.find(SearchNode<state>(neighbor, e->GetStateHash(neighbor)));
//	SearchNode<state> alt = closedList[e->GetStateHash(currOpenNode)];
//	double edgeWeight = e->GCost(currOpenNode, neighbor);
//	double altCost = alt.gCost+edgeWeight+(prev.fCost-prev.gCost);
//	if (fgreater(prev.fCost, altCost))
//	{
//		//std::cout << "Reopening node " << neighbor << " setting parent to " << currOpenNode << std::endl;
//		//printf("Reopening node %d setting parent to %d - %f vs. %f\n", neighbor, currOpenNode, prev.fCost, altCost);
//		prev.fCost = altCost;
//		prev.gCost = alt.gCost+edgeWeight;
//	 	prev.prevNode = currOpenNode;
//		// this is generally unneeded. But, in a search space where two
//		// nodes may be "equal" but not identical, it is important.
//		prev.currNode = neighbor;
//		closedList.erase(e->GetStateHash(neighbor));
//		assert(closedList.find(e->GetStateHash(neighbor)) == closedList.end());
//		openQueue.Add(prev);
//	}
//}

///**
// * Update the weight of a node. 
// * @author Nathan Sturtevant
// * @date 03/22/06
// * 
// * @param currOpenNode The node that's currently being expanded
// * @param neighbor The node whose weight will be updated
// */
//template <class state, class action, class environment>
//void TemplateAStar<state,action,environment>::UpdateWeight(environment *e, state &currOpenNode, state &neighbor)
//{
//	//printf("Found in open list!\n");
//	SearchNode<state> prev = openQueue.find(SearchNode<state>(neighbor, e->GetStateHash(neighbor)));
//	SearchNode<state> alt = closedList[e->GetStateHash(currOpenNode)];
//	double edgeWeight = e->GCost(currOpenNode, neighbor);
//	double altCost = alt.gCost+edgeWeight+(prev.fCost-prev.gCost);
//	if (fgreater(prev.fCost, altCost))
//	{
//		//		std::cout << "Updating node " << neighbor << " setting parent to " << currOpenNode << std::endl;
//		//printf("Resetting node %d setting parent to %d\n", neighbor, currOpenNode);
//		prev.fCost = altCost;
//		prev.gCost = alt.gCost+edgeWeight;
//		prev.prevNode = currOpenNode;
//		// this is generally unneeded. But, in a search space where two
//		// nodes may be "equal" but not identical, it is important.
//		prev.currNode = neighbor;
//		openQueue.DecreaseKey(prev);
//	}
//}

///**
// * Add a node to the open list
// * @author Nathan Sturtevant
// * @date 03/22/06
// * 
// * @param currOpenNode the state that's currently being expanded
// * @param neighbor the state to be added to the open list
// */
//template <class state, class action,class environment>
//void TemplateAStar<state, action,environment>::AddToOpenList(environment *e, state &currOpenNode, state &neighbor)
//{
//	//printf("Adding node %d setting parent to %d\n", neighbor, currOpenNode);
//	//	std::cout << "Adding node " << neighbor << " setting parent to " << currOpenNode << std::endl;
//	double edgeWeight = e->GCost(currOpenNode, neighbor);
//	
//	double oldfCost = closedList[e->GetStateHash(currOpenNode)].fCost;
//	double oldgCost = closedList[e->GetStateHash(currOpenNode)].gCost;
//	double gCost = oldgCost+edgeWeight;
//	double fCost = gCost+weight*e->HCost(neighbor, goal);
//	if (/*(useBPMX) && */(fgreater(oldfCost, fCost))) // pathmax rule
//	{
//		//		std::cout << "Adding node: (" << neighbor << ") f-cost from " << fCost << " to " << oldfCost
//		//		<< " h-cost: " << fCost-gCost << " g-cost: " << gCost << " hash: " << e->GetStateHash(neighbor) << std::endl;
//		fCost = oldfCost;
//	}
//	else {
//		//		std::cout << "Adding node: (" << neighbor << ") f-cost " << fCost 
//		//		<< " h-cost: " << fCost-gCost << " g-cost: " << gCost << " hash: " << e->GetStateHash(neighbor) << std::endl;
//	}
//	SearchNode<state> n(neighbor, currOpenNode, fCost, gCost, e->GetStateHash(neighbor));
//	
//	openQueue.Add(n);	
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
void TemplateAStar<state, action,environment>::ExtractPathToStartFromID(uint64_t node,
																	 std::vector<state> &thePath)
{
	do {
		thePath.push_back(openClosedList.Lookup(node).data);
		node = openClosedList.Lookup(node).parentID;
	} while (openClosedList.Lookup(node).parentID != node);
	thePath.push_back(openClosedList.Lookup(node).data);
	//	SearchNode<state> n;
	//	if (closedList.find(env->GetStateHash(goalNode)) != closedList.end())
	//	{
	//		n = closedList[env->GetStateHash(goalNode)];
	//	}
	//	else n = openQueue.find(SearchNode<state>(goalNode, env->GetStateHash(goalNode)));
	//	
	//	do {
	//		//std::cout << "Extracting " << n.currNode << " with parent " << n.prevNode << std::endl;
	//		//printf("Extracting %d with parent %d\n", n.currNode, n.prevNode);
	//		thePath.push_back(n.currNode);
	//		if (closedList.find(env->GetStateHash(n.prevNode)) != closedList.end())
	//		{
	//			n = closedList[env->GetStateHash(n.prevNode)];
	//		}
	//		else {
	//			printf("No backward path found!\n");
	//			break;
	//		}
	//	} while (!(n.currNode == n.prevNode));
	//	thePath.push_back(n.currNode);
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

///**
// * Get an iterator for the closed list
// * @author Nathan Sturtevant
// * @date 06/13/07
// * 
// * @return An iterator pointing to the first node in the closed list
// */
//template <class state, class action,class environment>
////__gnu_cxx::hash_map<state, TemplateAStarUtil::SearchNode<state> >::const_iterator
//void TemplateAStar<state, action,environment>::GetClosedListIter(closedList_iterator) //const
//{
//	return closedList.begin();
//}
//
///**
// * Get the next state in the closed list
// * @author Nathan Sturtevant
// * @date 06/13/07
// * 
// * @param it A closedList_iterator pointing at the current state in the closed 
// * list
// * @return The next state in the closed list. Returns UINT_MAX if there's no 
// * more states
// */
//template <class state, class action, class environment>
//bool TemplateAStar<state, action,environment>::ClosedListIterNext(closedList_iterator& it, state& next) const
//{
//	if (it == closedList.end())
//		return false;
//	next = (*it).first;
//	it++;
//	return true;
//}
//

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
//	if (closedList.find(env->GetStateHash(val)) != closedList.end())
//	{
//		gCost = closedList.find(env->GetStateHash(val))->second.gCost;
//		return true;
//	}
//	return false;
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
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const AStarOpenClosedData<state> &data = openClosedList.Lookat(x);
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
			env->SetColor(1.0, 0.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
	}
//	int x = 0;
//	if (env == 0)
//		return;
//	//env->SetColor(0.0, 0.0, 0.0, 0.15);
//	for (closedList_iterator it = closedList.begin(); it != closedList.end(); it++)
//	{
//		env->OpenGLDraw((*it).second.currNode);
//	}
}

#endif
