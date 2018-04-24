//
//  BID.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/28/18.
//  Copyright © 2018 University of Denver. All rights reserved.
//

#ifndef BID_h
#define BID_h

template <class state, class action>
class BID {
public:
//	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
//				 std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<action> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, Heuristic<state> *heuristic, state from, state to,
				 std::vector<action> &thePath);

	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; }
private:
	std::pair<uint32_t, uint32_t> ExponentialSearch();
	uint32_t BinarySearch(uint32_t b1, uint32_t b2);
	uint64_t DFBNB(state currState, uint32_t costLimit, uint64_t nodeLimit);
	uint64_t DFBNBHelper(state &currState, uint32_t pathCost, uint32_t costLimit, uint64_t nodesExpanded, uint64_t nodeLimit);

	uint64_t nodesExpanded, nodesTouched;
	Heuristic<state> *h;
	SearchEnvironment<state, action> *env;

	state start, goal;
	uint32_t CUpperBound, CLowerBound;
	uint64_t nodeLowerBound, nodeUpperBound;

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
	CLowerBound = h->(start, goal);
	CUpperBound = 0xFFFFFFFF;
	nodeLowerBound = 1;// Do B&B search to establish the number by initial bound
	nodeUpperBound = 2;
	
	while (true)
	{
		// Searches strictly within the node bound until it finds:
		// 1) A new lower-bound on the solution cost which results in node expansions in the node bounds
		// 2) A range of solution costs where the first value is below and the last value is outside the node bound range
		// Additionally, it may reduce CUpperBound as solutions are found
		// Returns the range of possible costs for this node bound
		auto p = ExponentialSearch();
		
		// we hit the range
		if (p.first == p.second)
		{
			continue;
		}

		// Searches between the cost values in the pair to find somewhere where node expansions fall in the bound.
		// 1) If we don't find a cost between the solution bounds then we search the full iteration that goes over
		//    the bound, assuming we haven't yet found a solution.
		// We return the cost bound that we've exhaustively searched
		CLowerBound = BinarySearch(p.first, p.second, NodeBound);
	}
}

// Searches for next c-limit
template <class state, class action>
std::pair<uint32_t, uint32_t> BID<state, action>::ExponentialSearch()
{
	int k = 1;
	uint32_t lb;
	
	while (true)
	{
		lb = CLowerBound+k;
		uint64_t actual = DFBNB(start, lb); // TODO: Use next limit
		if (actual >= nodeLowerBound && actual <= nodeUpperBound) // success
		{
			// Set next node limits to actual*2 ... actual*10
			nodeLowerBound = actual*2;
			nodeUpperBound = actual*10;
			// Set next cost limit lower bound to be lb
			return {lb, lb};
		}
		if (actual > nodeUpperBound) // done
			return {CLowerBound+k/2, ClowerBound+k};
		k = k*2;
	}
}

// Search between b1 and b2 exclusive
// We know b1 is under the range and b2 is over the range
// Return the cost bound we've searched exhaustively
template <class state, class action>
uint32_t BID<state, action>::BinarySearch(uint32_t b1, uint32_t b2, int32_t nodeLimit)
{
	// base cases (failure)
	if (b1+1 >= b2)
	{
		// exhaustively search b2 and continue from there
		uint64_t actual = DFBNB(start, b2, nodeLimit); // TODO: run infinitely
		// TODO: keep track of next bound in DFBNB
		nodeLowerBound = actual*2;
		nodeUpperBound = actual*10;
		return 0;
	}
	// recursive cases
	// search (b1+b2)/2
	uint64_t actual = DFBNB(start, (b1+b2)/2, nodeLimit);
	
	if (actual < nodeLimit) // found in bound - (need to check off-by-one computation)
	{
		return BinarySearch((b1+b2)/2, b2, nodeLimit);
	}
	else { // over bound
		return BinarySearch(b1, (b1+b2)/2, nodeLimit);
	}
}

template <class state, class action>
uint64_t BID<state, action>::DFBNB(state currState, uint32_t costLimit, uint64_t nodeLimit)
{
	return DFBNBHelper(currState, 0, costLimit, 0, nodeLimit);
}

template <class state, class action>
uint64_t BID<state, action>::DFBNBHelper(state &currState, uint32_t pathCost, uint32_t costLimit, uint64_t nodesExpanded, uint64_t nodeLimit)
{
	if (pathCost+env->HCost(currState, goal) > costLimit)
		return nodesExpanded;
	if (nodesExpanded >= nodeLimit)
		return nodesExpanded;
	
	// TODO: cache these for later use
	std::vector<action> acts;
	env->GetActions(currState, acts);
	nodesExpanded++;
	for (int x = 0; x < acts.size(); x++)
	{
		uint32_t edgeCost = env->GCost(currState, acts[x]);
		env->ApplyAction(currState, acts[x]);
		nodesExpanded = DFBNBHelper(currState, pathCost+edgeCost, costLimit, nodesExpanded, nodeLimit);
		env->UndoAction(currState, acts[x]);
	}
	return nodesExpanded;
}


#endif /* BID_h */
