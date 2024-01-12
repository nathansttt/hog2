//
//  DSDWAStar.h
//  HOG2 ObjC
//
//  Created by Nathan Sturtevant on 12/26/22.
//  Copyright © 2022 MovingAI. All rights reserved.
//

#ifndef DSDWAStar_h
#define DSDWAStar_h

#include "TemplateAStar.h" // to get state definitions

enum tExpansionPriority {
	kWA=0,
	kXDP=1,
	kXUP=2,
	kHalfEdgeDrop=3,
	kGreedy=4,
	kFullEdgeDrop=5,
	kPathSuboptDouble=6,
	kDSDPolicyCount=7,
};

template <class state, class action, class environment, class openList = AStarOpenClosed<state, AStarCompareWithF<state>, AStarOpenClosedDataWithF<state>> >
class DSDWAStar : public GenericSearchAlgorithm<state,action,environment> {
public:
	DSDWAStar() {
		ResetNodeCount(); env = 0; stopAfterGoal = true; weight=1; reopenNodes = false; theHeuristic = 0;
		theConstraint = 0;
	}
	virtual ~DSDWAStar() {}
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	void GetPath(environment *, const state&, const state&, std::vector<action> & );
	
	openList openClosedList;
	state goal, start;
	
	bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	void AddAdditionalStartState(state& newState);
	void AddAdditionalStartState(state& newState, double cost);
	
	state CheckNextNode();
	void ExtractPathToStart(state &node, std::vector<state> &thePath)
	{
		thePath.clear();
		uint64_t theID;
		if (openClosedList.Lookup(env->GetStateHash(node), theID) != kNotFound)
			ExtractPathToStartFromID(theID, thePath);
	}
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath);
	const state &GetParent(const state &s);
	virtual const char *GetName();
	
	void PrintStats();
	uint64_t GetUniqueNodesExpanded() { return uniqueNodesExpanded; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; uniqueNodesExpanded = 0; }
	int GetMemoryUsage();
	
	bool GetClosedListGCost(const state &val, double &gCost) const;
	bool GetOpenListGCost(const state &val, double &gCost) const;
	bool GetHCost(const state &val, double &hCost) const;
	bool GetClosedItem(const state &s, AStarOpenClosedDataWithF<state> &);
	unsigned int GetNumOpenItems() { return openClosedList.OpenSize(); }
	inline const AStarOpenClosedDataWithF<state> &GetOpenItem(unsigned int which) { return openClosedList.Lookat(openClosedList.GetOpenItem(which)); }
	inline const int GetNumItems() { return openClosedList.size(); }
	inline const AStarOpenClosedDataWithF<state> &GetItem(unsigned int which) { return openClosedList.Lookat(which); }
	bool HaveExpandedState(const state &val)
	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key) != kNotFound; }
	dataLocation GetStateLocation(const state &val)
	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key); }
	
	void SetReopenNodes(bool re) { reopenNodes = re; }
	bool GetReopenNodes() { return reopenNodes; }

	void SetHeuristic(Heuristic<state> *h) { theHeuristic = h; }
	void SetConstraint(Constraint<state> *c) { theConstraint = c; }

	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	uint64_t GetNecessaryExpansions() const;
	void LogFinalStats(StatCollection *) {}
	
	void SetStopAfterGoal(bool val) { stopAfterGoal = val; }
	bool GetStopAfterGoal() { return stopAfterGoal; }
		
	void OpenGLDraw() const;
	void Draw(Graphics::Display &disp) const;
	void DrawPriorityGraph(Graphics::Display &disp) const;
	
	double Phi(double h, double g)
	{
		return GetPriority(h, g);
	}
	void SetWeight(double w)
	{
		weight = w;
	}
	double GetWeight() { return weight; }
	float GetPriority(float h, float g)
	{
		if (data.size() == 0)
		{
			if (fequal(g, 0))
				return h;
			printf("WARNING: Invalid priority lookups %s line %d\n", __FILE__, __LINE__);
			return INFINITY;
		}
		float slope = g/h;
		if (fgreater(slope, data.back().slope))
		{
			printf("WARNING: Invalid priority lookups %s line %d\n", __FILE__, __LINE__);
			return INFINITY;
		}
		// dumb/slow but correct
		for (int x = 0; x < data.size(); x++)
			if (flesseq(slope, data[x].slope))
				return data[x].K*(g + data[x].weight * h);
		return INFINITY;
	}
	
	// directly target next suboptimality
	void SetNextWeight(float h, float g, float targetWeight) // const Graphics::point &loc
	{
		if (fequal(h, 0))
		{
			float w;
			point3d last;
			if (data.size() > 0)
				 last = data.back().crossPoint;
			else
				last = {1, 0};
			w = (weight-last.y)/last.x;
			// connect to y axis at bound*(1)
			float K = 1/(last.y+w*last.x);
			data.push_back({INFINITY, w, K, {static_cast<float>(weight), 0.0f}});
		}
		if (fgreater(h, 0) && fgreater(g, 0))
		{
			float slope = g/h;
			if (data.size() == 0 || data.back().slope < slope)
			{
				//std::cout << "Virtual hit of " << loc << " slope " << slope << "\n";
				float minWeight, maxWeight;
				if (data.size() > 0)
					GetNextWeightRange(minWeight, maxWeight, data.back().crossPoint, slope);
				else
					GetNextWeightRange(minWeight, maxWeight, {1, 0}, slope);

				float K;
				float nextWeight = std::min(maxWeight, std::max(minWeight, targetWeight));
				point3d last;
				if (data.size() == 0)
				{
					last = point3d(1, 0);
				}
				else {
					last = data.back().crossPoint;
				}
				K = 1/(last.y+nextWeight*last.x);

				// our cross point of next slope
				point3d crossPoint1;
				crossPoint1.x = 1.0f/(K*(slope+nextWeight));
				crossPoint1.y = crossPoint1.x*slope;
				
				data.push_back({slope, nextWeight, K, crossPoint1});
			}
		}
	}
	
	void SetNextPriority(float h, float g, float target) // const Graphics::point &loc
	{
		if (fequal(h, 0))
		{
			float w;
			point3d last;
			if (data.size() > 0)
				 last = data.back().crossPoint;
			else
				last = {1, 0};
			w = (weight-last.y)/last.x;
			// connect to y axis at bound*(1)
			float K = 1/(last.y+w*last.x);
			data.push_back({INFINITY, w, K, {static_cast<float>(weight), 0.0f}});
		}
		if (fgreater(h, 0) && fgreater(g, 0))
		{
			float slope = g/h;
			if (data.size() == 0 || data.back().slope < slope)
			{
				//std::cout << "Virtual hit of " << loc << " slope " << slope << "\n";
				float minWeight, maxWeight;
				if (data.size() > 0)
					GetNextWeightRange(minWeight, maxWeight, data.back().crossPoint, slope);
				else
					GetNextWeightRange(minWeight, maxWeight, {1, 0}, slope);

				float K;
				float nextWeight;
				point3d last;
				if (data.size() == 0)
				{
					last = point3d(1, 0);
				}
				else {
					last = data.back().crossPoint;
				}

				
				switch (policy)
				{
					case kWA: // Weighted A*
					{
						K = 1/weight;
						nextWeight = weight;
						break;
					}
					case kXUP: // PWXUP
					{
						nextWeight = maxWeight;
						K = 1/(last.y+nextWeight*last.x);
						break;
					}
					case kXDP: // PWXDP
					{
						nextWeight = minWeight;
						K = 1/(last.y+nextWeight*last.x);
						break;
					}
					default:
					case kGreedy:
					{
						// K (g + [w] * h) = 1 at previous point
						// returns nextWeight and K
						nextWeight = ChooseWeightForTargetPriority({h, g}, target, minWeight, maxWeight, last, K);
					}
				}
				

				// our cross point of next slope
				point3d crossPoint1;
				crossPoint1.x = 1.0f/(K*(slope+nextWeight));
				crossPoint1.y = crossPoint1.x*slope;
				
				data.push_back({slope, nextWeight, K, crossPoint1});
				
	//			std::cout << "Cross Priorities: ";
	//			for (const auto &i : data)
	//			{
	//				std::cout << i.crossPoint << ": ";
	//				std::cout << GetPriority(i.crossPoint.x, i.crossPoint.y) << " ";
	//			}
	//			std::cout << "\n";
			}
		}
	}
	
	/**
	 * Given the slope of the next bounding line, give the possbile range of weights that can be used in the priority function
	 *
	 * \param minWeight (returned) The minimum weight that can be used without going under the lower limit
	 * \param maxWeight (returned) The maximum weight that can be used without going over the upper limit
	 **/
	void GetNextWeightRange(float &minWeight, float &maxWeight, float nextSlope)
	{
		if (data.size() > 0)
			GetNextWeightRange(minWeight, maxWeight, data.back().crossPoint, nextSlope);
		else
			GetNextWeightRange(minWeight, maxWeight, {1, 0}, nextSlope);
	}

	/**
	 * Given the slope of the next bounding line, give the possbile range of weights that can be used in the priority function
	 *
	 * \param minWeight (returned) The minimum weight that can be used without going under the lower limit
	 * \param maxWeight (returned) The maximum weight that can be used without going over the upper limit
	 * \param currPoint The point on the previous bounding line with priorirty 1.0
	 * \param nextSlope The slope of the next bounding line
	 **/
	void GetNextWeightRange(float &minWeight, float &maxWeight, point3d currPoint, float nextSlope)
	{
		// defaults
		minWeight = 1;
		maxWeight = 2*weight-1;

		// 0. next slope line is y = slope*x
		// 1. cannot go over the [y = -x + w] line
		// slope*x = -x + w; slope*x + x = w; x = w/(slope+1)
		// y/slope = w-y; y = slope * w - slope *y; y*(1+slope) = slope*w; y = slope*w/(1+slope)
		point3d upperPoint(weight/(nextSlope+1),
						   nextSlope*weight/(1+nextSlope));
	//	std::cout << "Upper point: " << upperPoint << "\n";
		// 2. cannot go under the [y = -(2w-1)x + w] line
		// slope*x = -(2w-1)x + w; x*(slope + (2w-1)) = w
		// y/slope = (w-y)/(2w-1)
		// y (2w-1) = slope * w - slope*y
		// y (slope + 2w-1) = slope*w
		point3d lowerPoint(weight/(nextSlope+2*weight-1),
						   nextSlope*weight / (nextSlope + 2*weight-1));
		// get (negative) slopes to upper and lower points
		minWeight = std::max(minWeight, (lowerPoint.y-currPoint.y)/(currPoint.x-lowerPoint.x));
		if (upperPoint.x < currPoint.x)
			maxWeight = std::min(maxWeight, (upperPoint.y-currPoint.y)/(currPoint.x-upperPoint.x));
	//	printf("Weight needs to be [%f, %f]\n", minWeight, maxWeight);
	}
	
	/*
	 * Given location, and a range of priorities, try to give the point the exact desired priority
	 * Returned priority will always be in the minWeight/maxWeight range
	 */
	float ChooseWeightForTargetPriority(point3d loc, float priority, float minWeight, float maxWeight, point3d last, float &K)
	{
		float weight;
		// New: Last point is the priority 1 crossing point.
		// 1. Find the point on the previous line with priority {priority}
		point3d projectedPoint = last*priority;
		// 2. FYI: loc is the point on the new line that we want to have the desired priority
		// 3. Find the slope between the last point and our new point.
		//    Which is delta y / delta x
		if (flesseq(loc.y, projectedPoint.y))
		{
			//printf("Ill defined case (new y < old y); defaulting to min\n");

			// Explainer: State can be immediately expanded, just use min
			K = 1/(last.y+minWeight*last.x);
			return minWeight;
		}
		if (fgreatereq(loc.x, projectedPoint.x))
		{
//			printf("Ill defined case (new x > old x); defaulting to max\n");
			
			K = 1/(last.y+maxWeight*last.x);
			return maxWeight;
		}
		
		// Then try to extend that point to the new point giving it the desired priority
		weight = (loc.y-projectedPoint.y)/(projectedPoint.x-loc.x);
		// and bound by min/max weight
		weight = std::max(std::min(weight, maxWeight), minWeight);
		K = 1/(last.y+weight*last.x);
		return weight;
	}
	tExpansionPriority policy = kFullEdgeDrop;//kHalfEdgeDrop;
private:
	point3d HOGToLocal(point3d p) const
	{
		return point3d((p.x+1)*weight/2.0f , (p.y-1)*weight/-2.0);
	}

	point3d LocalToHOG(point3d p) const
	{
		return point3d(2.0f*(p.x/weight-0.5), -2.0*(p.y/weight-0.5));
	}

	uint64_t nodesTouched, nodesExpanded;

	std::vector<state> neighbors;
	std::vector<uint64_t> neighborID;
	std::vector<double> edgeCosts;
	std::vector<double> heuristicCosts;
	std::vector<dataLocation> neighborLoc;
	environment *env;
	bool stopAfterGoal;
	
	double goalFCost;
	double weight;
	bool reopenNodes;
	uint64_t uniqueNodesExpanded;
	environment *radEnv;
	Heuristic<state> *theHeuristic;
	Constraint<state> *theConstraint;

	struct DSDdata {
		float slope;
		float weight;
		float K;
		point3d crossPoint; // cached for simplicity
	};
	std::vector<DSDdata> data;
};

/**
 * Return the name of the algorithm.
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The name of the algorithm
 */

template <class state, class action, class environment, class openList>
const char *DSDWAStar<state,action,environment,openList>::GetName()
{
	static char name[32];
	sprintf(name, "DSDWAStar[]");
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
void DSDWAStar<state,action,environment,openList>::GetPath(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	if (!InitializeSearch(_env, from, to, thePath))
	{
		return;
	}
	while (!DoSingleSearchStep(thePath))
	{
//		if (0 == nodesExpanded%100000)
//			printf("%" PRId64 " nodes expanded, %" PRId64 " generated\n", nodesExpanded, nodesTouched);
	}
}

template <class state, class action, class environment, class openList>
void DSDWAStar<state,action,environment,openList>::GetPath(environment *_env, const state& from, const state& to, std::vector<action> &path)
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
	for (size_t x = 0; x < thePath.size()-1; x++)
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
bool DSDWAStar<state,action,environment,openList>::InitializeSearch(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	if (theHeuristic == 0)
		theHeuristic = _env;
	thePath.resize(0);
	env = _env;
	openClosedList.Reset(env->GetMaxHash());
	ResetNodeCount();
	start = from;
	goal = to;
	if (env->GoalTest(from, to) && (stopAfterGoal)) //assumes that from and to are valid states
	{
		return false;
	}
	data.resize(0);

	double h = theHeuristic->HCost(start, goal);
	openClosedList.AddOpenNode(start, env->GetStateHash(start), Phi(h, 0), 0, h);
	
	return true;
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search and before DoSingleSearchStep.
 * @author Nathan Sturtevant
 * @date 01/06/08
 */
template <class state, class action, class environment, class openList>
void DSDWAStar<state,action,environment,openList>::AddAdditionalStartState(state& newState)
{
	double h = theHeuristic->HCost(newState, goal);
	openClosedList.AddOpenNode(newState, env->GetStateHash(newState), Phi(h, 0), 0, h);
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search
 * @author Nathan Sturtevant
 * @date 09/25/10
 */
template <class state, class action, class environment, class openList>
void DSDWAStar<state,action,environment,openList>::AddAdditionalStartState(state& newState, double cost)
{
	double h = theHeuristic->HCost(newState, goal);
	openClosedList.AddOpenNode(newState, env->GetStateHash(newState), Phi(h, cost), cost, h);
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
bool DSDWAStar<state,action,environment,openList>::DoSingleSearchStep(std::vector<state> &thePath)
{
	if (openClosedList.OpenSize() == 0)
	{
		thePath.resize(0); // no path found!
		//closedList.clear();
		return true;
	}
	uint64_t nodeid = openClosedList.Close();
//	if (openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h > lastF)
//	{ lastF = openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h;
//		//printf("Updated limit to %f\n", lastF);
//	}
	if (!openClosedList.Lookup(nodeid).reopened)
		uniqueNodesExpanded++;
	nodesExpanded++;

	if ((stopAfterGoal) && (env->GoalTest(openClosedList.Lookup(nodeid).data, goal)))
	{
		ExtractPathToStartFromID(nodeid, thePath);
		// Path is backwards - reverse
		reverse(thePath.begin(), thePath.end());
		goalFCost = openClosedList.Lookup(nodeid).f;// + openClosedList.Lookup(nodeid).h;
		return true;
	}
	
	neighbors.resize(0);
	edgeCosts.resize(0);
	neighborID.resize(0);
	neighborLoc.resize(0);
	heuristicCosts.resize(0);
	
	//std::cout << "Expanding: " << env->GetStateHash(openClosedList.Lookup(nodeid).data) << " with f:";
	//std::cout << openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h << std::endl;
	
	env->GetSuccessors(openClosedList.Lookup(nodeid).data, neighbors);
	float maxSlope = 0;
	float maxSlopeG=-1, maxSlopeH=-1;
	int which = -1;
	// 1. load all the children
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		uint64_t theID;
		neighborLoc.push_back(openClosedList.Lookup(env->GetStateHash(neighbors[x]), theID));
		neighborID.push_back(theID);
		edgeCosts.push_back(env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]));
		heuristicCosts.push_back(theHeuristic->HCost(neighbors[x], goal));

		// open list can only be reduced in cost, thus not needing to extend the priority function
		if (neighborLoc[x] == kOpenList)
			continue;
		float slope = (openClosedList.Lookup(nodeid).g+edgeCosts[x])/heuristicCosts[x];
		if (fgreater(slope, maxSlope))
		{
			maxSlope = slope;
			maxSlopeG = openClosedList.Lookup(nodeid).g+edgeCosts[x];
			maxSlopeH = heuristicCosts[x];
			which = x;
		}
	}
	if (fgreater(maxSlope, 0))
	{
		// TODO: handle edge cases
		if (openClosedList.OpenSize() != 0)
		{
			if (policy == kGreedy)
			{
				SetNextPriority(maxSlopeH, maxSlopeG, openClosedList.Lookup(openClosedList.Peek()).f);
			}
			else if (policy == kHalfEdgeDrop) {
				SetNextPriority(maxSlopeH, maxSlopeG, openClosedList.Lookup(openClosedList.Peek()).f-edgeCosts[which]*(1-weight)/2.0);
			}
			else if (policy == kFullEdgeDrop) {
				SetNextPriority(maxSlopeH, maxSlopeG, openClosedList.Lookup(openClosedList.Peek()).f-edgeCosts[which]*(1-weight));
			}
			else if (policy == kPathSuboptDouble)
			{
				// Estimated cost to here: theHeuristic->HCost(start, neighbors[which]);
				// Actual cost to here: maxSlopeG
				float soFar = maxSlopeG/theHeuristic->HCost(start, neighbors[which]);
				SetNextWeight(maxSlopeH, maxSlopeG, (soFar+weight)-1); // const Graphics::point &loc
			}
			else {
				// last argument will be ignored
				SetNextPriority(maxSlopeH, maxSlopeG, 0.01);
			}
		}
		else {
			// Unsure if this is the right setting - can use lowest possible weight
			// or perhaps use same weight as parent?
			SetNextPriority(maxSlopeH, maxSlopeG, 0.01);
		}
	}
	
	// iterate again updating costs and writing out to memory
	for (size_t x = 0; x < neighbors.size(); x++)
	{
		nodesTouched++;
		//std::cout << "looking at child with hash : " << env->GetStateHash(neighbors[x]) << "and g-cost"<<openClosedList.Lookup(nodeid).g+edgeCosts[x]<<std::endl;
		if (theConstraint &&
			theConstraint->ShouldNotGenerate(start, openClosedList.Lookup(nodeid).data, neighbors[x],
											 openClosedList.Lookup(nodeid).g+edgeCosts[x], goal))
			continue;

		switch (neighborLoc[x])
		{
			case kClosedList:
				// TODO: Can update parent pointers when shorter paths are found to improve solution quality
				if (reopenNodes)
				{
					if (fless(openClosedList.Lookup(nodeid).g+edgeCosts[x], openClosedList.Lookup(neighborID[x]).g))
					{
						auto &i = openClosedList.Lookup(neighborID[x]);
						i.parentID = nodeid;
						i.g = openClosedList.Lookup(nodeid).g+edgeCosts[x];
						i.f = Phi(i.h, i.g);
						openClosedList.Reopen(neighborID[x]);
						// This line isn't normally needed, but in some state spaces we might have
						// equality but different meta information, so we need to make sure that the
						// meta information is also copied, since this is the most generic A* implementation
						i.data = neighbors[x];
					}
				}
				break;
			case kOpenList:
				if (fless(openClosedList.Lookup(nodeid).g+edgeCosts[x], openClosedList.Lookup(neighborID[x]).g))
				{
					auto &i = openClosedList.Lookup(neighborID[x]);
					i.parentID = nodeid;
					i.g = openClosedList.Lookup(nodeid).g+edgeCosts[x];
					i.f = Phi(i.h, i.g);
					// This line isn't normally needed, but in some state spaces we might have
					// equality but different meta information, so we need to make sure that the
					// meta information is also copied, since this is the most generic A* implementation
					i.data = neighbors[x];
					openClosedList.KeyChanged(neighborID[x]);
//					std::cout << " Reducing cost to " << openClosedList.Lookup(nodeid).g+edgeCosts[x] << "\n";
					// TODO: unify the KeyChanged calls.
				}
				else {
//					std::cout << " no cheaper \n";
				}
				break;
			case kNotFound:
				{
					double h = heuristicCosts[x];
					openClosedList.AddOpenNode(neighbors[x],
											   env->GetStateHash(neighbors[x]),
											   Phi(h, openClosedList.Lookup(nodeid).g+edgeCosts[x]),
											   openClosedList.Lookup(nodeid).g+edgeCosts[x],
											   h,
											   nodeid);
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
template <class state, class action, class environment, class openList>
state DSDWAStar<state, action,environment,openList>::CheckNextNode()
{
	uint64_t key = openClosedList.Peek();
	return openClosedList.Lookup(key).data;
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
void DSDWAStar<state, action,environment,openList>::ExtractPathToStartFromID(uint64_t node,
																	 std::vector<state> &thePath)
{
	do {
		thePath.push_back(openClosedList.Lookup(node).data);
		node = openClosedList.Lookup(node).parentID;
	} while (openClosedList.Lookup(node).parentID != node);
	thePath.push_back(openClosedList.Lookup(node).data);
}

template <class state, class action,class environment,class openList>
const state &DSDWAStar<state, action,environment,openList>::GetParent(const state &s)
{
	uint64_t theID;
	openClosedList.Lookup(env->GetStateHash(s), theID);
	theID = openClosedList.Lookup(theID).parentID;
	return openClosedList.Lookup(theID).data;
}

template <class state, class action, class environment, class openList>
uint64_t DSDWAStar<state, action,environment,openList>::GetNecessaryExpansions() const
{
	uint64_t n = 0;
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const auto &data = openClosedList.Lookat(x);
		if (fless(data.g + data.h, goalFCost))
			n++;
	}
	return n;
}


/**
 * A function that prints the number of states in the closed list and open
 * queue.
 * @author Nathan Sturtevant
 * @date 03/22/06
 */
template <class state, class action, class environment, class openList>
void DSDWAStar<state, action,environment,openList>::PrintStats()
{
	printf("%u items in closed list\n", (unsigned int)openClosedList.ClosedSize());
	printf("%u items in open queue\n", (unsigned int)openClosedList.OpenSize());
}

/**
 * Return the amount of memory used by DSDWAStar
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The combined number of elements in the closed list and open queue
 */
template <class state, class action, class environment, class openList>
int DSDWAStar<state, action,environment,openList>::GetMemoryUsage()
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
 * the states
 */
template <class state, class action, class environment, class openList>
bool DSDWAStar<state, action,environment,openList>::GetClosedListGCost(const state &val, double &gCost) const
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

template <class state, class action, class environment, class openList>
bool DSDWAStar<state, action,environment,openList>::GetOpenListGCost(const state &val, double &gCost) const
{
	uint64_t theID;
	dataLocation loc = openClosedList.Lookup(env->GetStateHash(val), theID);
	if (loc == kOpenList)
	{
		gCost = openClosedList.Lookat(theID).g;
		return true;
	}
	return false;
}

template <class state, class action, class environment, class openList>
bool DSDWAStar<state, action,environment,openList>::GetHCost(const state &val, double &hCost) const
{
	uint64_t theID;
	dataLocation loc = openClosedList.Lookup(env->GetStateHash(val), theID);
	if (loc != kNotFound)
	{
		hCost = openClosedList.Lookat(theID).h;
		return true;
	}
	return false;
}

template <class state, class action, class environment, class openList>
bool DSDWAStar<state, action,environment,openList>::GetClosedItem(const state &s, AStarOpenClosedDataWithF<state> &result)
{
	uint64_t theID;
	dataLocation loc = openClosedList.Lookup(env->GetStateHash(s), theID);
	if (loc == kClosedList)
	{
		result = openClosedList.Lookat(theID);
		return true;
	}
	return false;

}


/**
 * Draw the open/closed list
 * @author Nathan Sturtevant
 * @date 03/12/09
 * Deprecated
 */
template <class state, class action, class environment, class openList>
void DSDWAStar<state, action,environment,openList>::OpenGLDraw() const
{} // deprecated

/**
 * Draw the open/closed list
 * @author Nathan Sturtevant
 * @date 7/12/16
 *
 */
template <class state, class action, class environment, class openList>
void DSDWAStar<state, action,environment,openList>::Draw(Graphics::Display &disp) const
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
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const auto &data = openClosedList.Lookat(x);
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->Draw(disp, data.data);
		}
		else if ((data.where == kOpenList) && (data.reopened))
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->Draw(disp, data.data);
		}
		else if (data.where == kOpenList)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->Draw(disp, data.data);
		}
		else if ((data.where == kClosedList) && (data.reopened))
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			env->Draw(disp, data.data);
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
			env->Draw(disp, data.data);
		}
	}
	env->SetColor(1.0, 0.5, 1.0, 0.5);
	env->Draw(disp, goal);
}


template <class state, class action, class environment, class openList>
void DSDWAStar<state, action,environment,openList>::DrawPriorityGraph(Graphics::Display &display) const
{
	point3d origin(-1, 1);
	float priority = 1;
	float bound = weight;
	
	// draw bounding line
	point3d bl1(priority, 0), bl2(0, priority*bound); // main suboptimality line
	point3d bl3(priority*bound, 0);//(0+2, priority*bound-2); //
	point3d bl4(priority*bound/(2*bound-1), 0);//priority*bound-(2*bound-1));
	point3d bl2a(0, priority);
	point3d bl2c(priority-priority*bound/(2*bound-1), priority*bound);
	// WA* priority line
	display.DrawLine(LocalToHOG(bl1), LocalToHOG(bl2), 1/100.0f, Colors::yellow);
	// 45° upper bound line
	display.DrawLine(LocalToHOG(bl2), LocalToHOG(bl3), 1/100.0f, Colors::darkgray);
	// 2w-1 upper bound line
	display.DrawLine(LocalToHOG(bl1), LocalToHOG(bl2c), 1/100.0f, Colors::darkgray);

	// 45° lower bound line
	display.DrawLine(LocalToHOG(bl2a), LocalToHOG(bl1), 1/100.0f, Colors::lightgray);
	// 2w-1 lower bound line
	display.DrawLine(LocalToHOG(bl2), LocalToHOG(bl4), 1/100.0f, Colors::lightgray);
	
	
	// Draw actual priority line across
	for (int x = 0; x < data.size(); x++)
	{
		point3d value = origin;
		
		// y = slope * x // x=1 -> y = slope; y=1 -> x = 1/slope;
		if (isinf(data[x].slope))
		{
			value.x = -1;
			value.y = -1;
		}
		else if (data[x].slope < 1)
		{
			value.x += 2;
			value.y -= 2*data[x].slope;
		}
		else {
			value.x += 2/data[x].slope;
			value.y -= 2;
		}
		display.DrawLine(origin, value, 1/200.0f, Colors::blue);
		
		if (isinf(data[x].slope))
		{
			point3d crossPoint2;
			float lastSlope = ((x==0)?(0):(data[x-1].slope));
			crossPoint2.x = priority/(data[x].K*(lastSlope+data[x].weight));
			crossPoint2.y = crossPoint2.x*lastSlope;
			display.DrawLine(LocalToHOG({0, static_cast<float>(weight)}), LocalToHOG(crossPoint2), 1/100.0f, Colors::red);
		}
		else {
			point3d crossPoint1, crossPoint2;
			crossPoint1.x = priority/(data[x].K*(data[x].slope+data[x].weight));
			crossPoint1.y = crossPoint1.x*data[x].slope;
			float lastSlope = ((x==0)?(0):(data[x-1].slope));
			crossPoint2.x = priority/(data[x].K*(lastSlope+data[x].weight));
			crossPoint2.y = crossPoint2.x*lastSlope;
			display.DrawLine(LocalToHOG(crossPoint1), LocalToHOG(crossPoint2), 1/100.0f, Colors::red);
		}
	}
	for (int x = 0; x < data.size(); x++)
		display.FillCircle(LocalToHOG(data[x].crossPoint), 0.01, Colors::darkgreen);

	display.DrawLine(origin, {1, 1}, 1./100.0f, Colors::white);
	display.DrawLine(origin, {-1, -1}, 1./100.0f, Colors::white);
}

#endif /* DSDWAStar_h */
