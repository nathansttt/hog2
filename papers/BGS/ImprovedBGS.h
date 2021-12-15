//
//  IncrementalBGS.h
//  hog2
//
//  This is an incremental version of Budgeted Graph Search - IBEX on trees with Bounded Dijkstra at the low level.
//  This code isn't meant for production - just to animate how the algorithm works as a comparison to IDA*.
//  See (Helmert et al, IJCAI 2019) for details of the algorithm
//
// This implementation contains working of Algo2 for the efficient BGS. 
// The search can start from the current f-cost without discarding any nodes in the queue.
// There is only need to update the budget values correctly.
#ifndef IncrementalBGS_h
#define IncrementalBGS_h

#include "Heuristic.h"
#include "AStarOpenClosed.h"
bool mode = 0;
int flag = 0;
template <class state, class action>
class IncrementalBGS {
public:
	IncrementalBGS(double initialBound = 0) :bound(initialBound), initialBound(initialBound), nextBound(initialBound)
	{ ResetNodeCount(); previousBound = 0; }
	bool InitializeSearch(SearchEnvironment<state, action> *env, state from, state to, Heuristic<state> *h,
						  std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to, Heuristic<state> *h,
				 std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<action> &thePath);
				 
	bool DoSingleSearchStep();
	bool DoExponentialSingleSearchStep(double cost);
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesReExpanded() { return nodesReexpanded; }
	uint64_t GetIterationNodesExpanded() { return MInodesexpanded + MInodesreexpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	void ResetNodeCount()
	{
		nodesExpanded = nodesTouched = 0;
		newNodeCount = newNodesLastIteration = 0;
		nodesExpanded = 0;
		nodesReexpanded = 0;
	}
	void Reset() { bound = initialBound; nextBound = initialBound; solutionPath.clear();
		history.clear(); ResetNodeCount(); previousBound = 0; }
	void OpenGLDraw();
	void Draw(Graphics::Display &display) const;
	state GetCurrentState() 
	{
		if (q_g.size() > 0)
			return q_g.Lookup(q_g.Peek()).data;
//			return search.back().currState;
//		if (flesseq(solutionCost, data.solutionInterval.lowerBound))
//			return goal;
//		return start;
		
	}
	void GetCurrentPath(std::vector<state> &p) const
	{ p = solutionPath; }
	double GetCurrentFLimit() { return std::min(bound, solutionCost); }
	double GetSolutionCost() { return solutionCost;}
	double GetNextFLimit() { return nextBound; }
	uint64_t GetNewNodesLastIteration() { return newNodesLastIteration; }
	const uint64_t c1 = 2;
	const uint64_t c2 = 8;
	const uint64_t gamma = 2;
	const double k = 1;
	const int infiniteWorkBound = -1;
	void GetGlobalCostInterval(double &lower, double &upper)
	{ lower = data.solutionInterval.lowerBound; upper = data.solutionInterval.upperBound; }
	void GetNodeInterval(uint64_t &lower, uint64_t &upper)
	{ lower = data.nodeLB*c1; upper = data.workBound; }
	std::string stage;
	std::string fEquation;
private:
	struct costInterval {
		double lowerBound;
		double upperBound;
		costInterval &operator&=(const costInterval &i)
		{
			lowerBound = std::max(lowerBound, i.lowerBound);
			upperBound = std::min(upperBound, i.upperBound);
			return *this;
		}
	};
	struct IBEXData {
		uint64_t nodesExpanded;
		uint64_t workBound;
		uint64_t nodeLB;
		costInterval solutionInterval;
		double delta;
	};
	IBEXData data;
	struct searchData {
		double f_below;
		double f_above;
	};
	searchData sd;
	double solutionCost;
	bool IterationComplete() { 
		return q_f.OpenSize() == 0; }
	bool IterationCompleteUsingG(){
		return q_g.OpenSize() == 0;
	}
	bool MainIterationComplete(){ 
		if(fc_next == DBL_MAX){
			return false;
		}
		else{
			return true;
		}
	}
	unsigned long nodesExpanded, nodesTouched;
	unsigned long nodesReexpanded;
	void SetupIteration(double cost);
	void SetupExponentialBinaryIteration(double cost);
	bool StepIteration();
	bool StepIterationUsingG(double cost);
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath);
	void GetAllNodesfromQwithReopening();
	void GetAllNodesfromQ();
	void GetClosedNodesfromQ();
	void GetOpenNodesfromQ();
	std::vector<std::pair<state, double>> history;
	std::vector<state> solutionPath;
	state start, goal;
	double previousBound;
	double bound;
	double initialBound;
	double nextBound;
	double fc;
	double fc_next;
	SearchEnvironment<state, action> *env;
	Heuristic<state> *h;
	std::vector<state> succ;
	uint64_t newNodeCount, newNodesLastIteration;
	int MInodesexpanded;
	int MInodesreexpanded;
	int nodeLB;
	int MODE=0;
	struct BFHSCompare {
		bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
		{
			return (fgreater(i1.g, i2.g));
		}
	};

	struct BFHSCompare_f {
		bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
		{
			return (fgreater(i1.g+i1.h, i2.g+i2.h));
		}
	};

	// Data for BFHS
	AStarOpenClosed<state, BFHSCompare> q_g;
	AStarOpenClosed<state, BFHSCompare_f> q_f;
	std::vector<state> neighbors;
	std::vector<uint64_t> neighborID;
	std::vector<double> edgeCosts;
	std::vector<dataLocation> neighborLoc;
	std::vector<state> solutionStates;
};

template <class state, class action>
void IncrementalBGS<state, action>::GetPath(SearchEnvironment<state, action> *e, state from, state to,
											Heuristic<state> *h, std::vector<state> &thePath)
{
	if (InitializeSearch(e, from, to, h, thePath))
		return;
	while (!DoSingleSearchStep())
	{}
}

template <class state, class action>
void IncrementalBGS<state, action>::GetPath(SearchEnvironment<state, action> *e, state from, state to,
											std::vector<action> &thePath)
{
	
}

template <class state, class action>
bool IncrementalBGS<state, action>::InitializeSearch(SearchEnvironment<state, action> *e, state from, state to, Heuristic<state> *h,
													 std::vector<state> &thePath)
{
	printf("Search Initialization\n");
	Reset();
	if (from==to)
	{
		thePath.clear();
		thePath.push_back(from);
		return true;
	}
	start = from;
	goal = to;
	this->env = e;
	start = from;
	this->h = h;

	solutionPath.clear();


	// Setup BTS control data
/*	data.nodesExpanded = 0;
	data.workBound = infiniteWorkBound;
	data.nodeLB = 0;
	data.solutionInterval.lowerBound = h->HCost(start, goal);
	data.solutionInterval.upperBound = DBL_MAX;
	data.delta = 0;
	fEquation = to_string_with_precision(h->HCost(start, goal), 0);
	solutionCost = DBL_MAX;
	SetupIteration(h->HCost(start, goal));
	stage = "NEW ITERATION";
*/
    q_g.Reset(env->GetMaxHash());
	q_f.Reset(env->GetMaxHash());
	fc = DBL_MAX;
	fc_next = DBL_MAX;
	MInodesexpanded = 0;
	MInodesreexpanded = 0;
	MODE = 0;
	nodeLB = 0;
	SetupIteration(h->HCost(start, goal));
	solutionCost = DBL_MAX;
	stage = "INITIALIZE SEARCH";
	return false;
}

template <class state, class action>
void IncrementalBGS<state, action>::SetupIteration(double cost)	
{


	if(q_f.OpenSize() == 0){
        q_f.AddOpenNode(start,env->GetStateHash(start), 0.0,0/*h->HCost(start, goal)*/);
	}
	previousBound = bound;
	bound = cost;
	nextBound = -1;
	printf("Setup iteration bound %1.1f complete\n", bound);
	
}


template <class state, class action>
void IncrementalBGS<state, action>::SetupExponentialBinaryIteration(double cost)
{
	assert(MODE==1);
	data.nodesExpanded = nodeLB;
	data.workBound = infiniteWorkBound;
	data.nodeLB = nodeLB;
	data.solutionInterval.lowerBound = cost;
	data.solutionInterval.upperBound = DBL_MAX;
	data.delta = 0;
	printf("Setup Exponentail Binary Phase with %1.1 complete\n",cost);
}


template <class state, class action>
void IncrementalBGS<state, action>::ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath)
{
	thePath.clear();
	thePath.push_back(q_f.Lookup(node).data);
	return;
	do {
		thePath.push_back(q_f.Lookup(node).data);
		node = q_f.Lookup(node).parentID;
	} while (q_f.Lookup(node).parentID != node);
	thePath.push_back(q_f.Lookup(node).data);
}

template <class state, class action>
bool IncrementalBGS<state, action>::StepIteration()
{
	uint64_t nodeid = q_f.Close();
	if(q_f.Lookup(nodeid).reopened == true){
		nodesReexpanded++;
	
	}
	else{
	    nodesExpanded++;
	}
	// Check if we are done
	if (env->GoalTest(q_f.Lookup(nodeid).data, goal))
	{
		printf("Goal has been found\n");
		ExtractPathToStartFromID(nodeid, solutionPath);
		std::reverse(solutionPath.begin(), solutionPath.end());
		solutionCost = env->GetPathLength(solutionPath);
		return true;
	}

	//check if you need to adjust the bounds
	double currentFCost = q_f.Lookup(nodeid).g + h->HCost(q_f.Lookup(nodeid).data, goal);
	std::cout << "Checking currentf-cost " <<  q_f.Lookup(nodeid).g +  h->HCost(q_f.Lookup(nodeid).data, goal)<<std::endl;
	if(fgreater(currentFCost,bound)){
		previousBound = bound;
		bound = nextBound;
		nextBound = -1;
	}
	neighbors.resize(0);
	edgeCosts.resize(0);
	neighborID.resize(0);
	neighborLoc.resize(0);
	//std::vector<uint64_t> nodes = q_f.ClosedNodesTillNow();
	std::cout << "Expanding: " << env->GetStateHash(q_f.Lookup(nodeid).data) <<std::endl;
	std::cout << "Expanding f-cost " <<  q_f.Lookup(nodeid).g + h->HCost(q_f.Lookup(nodeid).data, goal)<<std::endl;
	env->GetSuccessors(q_f.Lookup(nodeid).data, neighbors);
	
	// 1. load all the children
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		uint64_t theID;
		neighborLoc.push_back(q_f.Lookup(env->GetStateHash(neighbors[x]), theID));
		neighborID.push_back(theID);
		edgeCosts.push_back(env->GCost(q_f.Lookup(nodeid).data, neighbors[x]));
	}
	
	// iterate again updating costs and writing out to memory
	for (int x = 0; x < neighbors.size(); x++)
	{
		double childGCost = q_f.Lookup(nodeid).g+edgeCosts[x];
		double childFCost = childGCost+h->HCost(neighbors[x], goal);
		printf("ChildFcost is %1.5f, current bound is %1.5f and next bound is %1.5f\n",childFCost,bound,nextBound);
		switch (neighborLoc[x])
		{	case kClosedList:
				printf("kclosedlist child\n");
				if (fless(childGCost, q_f.Lookup(neighborID[x]).g))
				{
					q_f.Lookup(neighborID[x]).parentID = nodeid;
					q_f.Lookup(neighborID[x]).g = childGCost;
					q_f.Reopen(neighborID[x]);
				}
				break;
			case kOpenList:
			    printf("kOpenlist child\n");
				if (fless(childGCost, q_f.Lookup(neighborID[x]).g))
				{
					q_f.Lookup(neighborID[x]).parentID = nodeid;
					q_f.Lookup(neighborID[x]).g = childGCost;
					q_f.KeyChanged(neighborID[x]);
				}
				break;
			case kNotFound:
			{   printf("knotFound child\n");
				q_f.AddOpenNode(neighbors[x],
							  env->GetStateHash(neighbors[x]),
							  childGCost,
							  h->HCost(neighbors[x], goal),
							  nodeid);
			}
		}
		if(fgreater(childFCost,bound)){
			if(nextBound == -1){
			  nextBound = childFCost;
		    }
		    else if (fless(childFCost,nextBound)){
			  nextBound = childFCost;
		    }
		}

	}
	printf("Step iteration over\n");
	return false;
}

template <class state, class action>
bool IncrementalBGS<state, action>::StepIterationUsingG(double cost)
{
    uint64_t nodeid = q_g.Peek();
	double nodeGCost = q_g.Lookup(nodeid).g;
	double nodeFCost = nodeGCost + h->HCost(q_g.Lookup(nodeid).data, goal);
	if(!flesseq(nodeFCost,cost)){
		state s = q_g.Lookup(nodeid).data;
		uint64_t parentId = q_g.Lookup(nodeid).parentID;
		q_f.AddOpenNode(s,
						env->GetStateHash(s),
						nodeGCost,
						h->HCost(q_g.Lookup(nodeid).data, goal),
						parentId);
		q_g.Remove(env->GetStateHash(q_g.Lookup(nodeid).data));
		
		printf("node with Fcost %1.5f is pruned \n",nodeFCost);
		printf("Step iteration over, node was beyond the limit\n");
	    return false;
	}

	nodeid = q_g.Close();
	nodesExpanded++;
	// Check if we are done
	if (env->GoalTest(q_g.Lookup(nodeid).data, goal))
	{
		printf("Goal has been found\n");
		ExtractPathToStartFromID(nodeid, solutionPath);
		std::reverse(solutionPath.begin(), solutionPath.end());
		solutionCost = env->GetPathLength(solutionPath);
		return true;
		
	}
	
	neighbors.resize(0);
	edgeCosts.resize(0);
	neighborID.resize(0);
	neighborLoc.resize(0);
	std::cout << "Expanding: " << env->GetStateHash(q_g.Lookup(nodeid).data) << " with nodeid " << nodeid<<std::endl;
	std::cout << "Expanding g-cost " <<  q_g.Lookup(nodeid).g  <<std::endl;
	env->GetSuccessors(q_g.Lookup(nodeid).data, neighbors);
	
	// 1. load all the children
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		uint64_t theID;
		neighborLoc.push_back(q_g.Lookup(env->GetStateHash(neighbors[x]), theID));
		neighborID.push_back(theID);
		edgeCosts.push_back(env->GCost(q_g.Lookup(nodeid).data, neighbors[x]));
	}
	
	// iterate again updating costs and writing out to memory
	for (int x = 0; x < neighbors.size(); x++)
	{
		double childGCost = q_g.Lookup(nodeid).g+edgeCosts[x];
		double childFCost = childGCost+h->HCost(neighbors[x], goal);
		printf("ChildFcost is %1.5f, current bound is %1.5f and next bound is %1.5f\n",childFCost,bound,nextBound);
		dataLocation d_1 ;
		uint64_t id;
		switch (neighborLoc[x])
		{
			case kClosedList:
				printf("kclosedlist child\n");
				if (fless(childGCost, q_g.Lookup(neighborID[x]).g))
				{
					q_g.Lookup(neighborID[x]).parentID = nodeid;
					q_g.Lookup(neighborID[x]).g = childGCost;
					q_g.Reopen(neighborID[x]);
				}
				break;
			case kOpenList:
				printf("kOpenlist child\n");
				if (fless(childGCost, q_g.Lookup(neighborID[x]).g))
				{
					q_g.Lookup(neighborID[x]).parentID = nodeid;
					q_g.Lookup(neighborID[x]).g = childGCost;
					q_g.KeyChanged(neighborID[x]);
				}
				break;
			case kNotFound:
			{
				printf("knotFound child\n");
				q_g.AddOpenNode(neighbors[x],
							  env->GetStateHash(neighbors[x]),
							  childGCost,
							  h->HCost(neighbors[x], goal),
							  nodeid);
			}
		}
		if(fgreater(childFCost,bound)){
			if(nextBound == -1){
			  nextBound = childFCost;
		    }
		    else if (fless(childFCost,nextBound)){
			  nextBound = childFCost;
		    }
		}
		
	}
	printf("Step iteration over\n");
	printf("current bound is %1.5f and next bound is %1.5f\n",bound,nextBound);
	return false;
}
template <class state, class action>
bool IncrementalBGS<state, action>::DoSingleSearchStep()
{
	printf("Single Search Step\n");
	bool s;
	if(!MainIterationComplete()){
		printf("MInodesExpanded is %d \n",MInodesexpanded);
        printf("MInodesReexpanded is %d \n",MInodesreexpanded);
		if(MODE == 0){
			int b = 10000;
			//b = -1;
			b = k*nodeLB;
			//b = int(sqrt(double(nodeLB)));
			//b = int(cbrt(double(nodeLB)));
			printf("b is --- %d\n",b);
			if(MInodesreexpanded <= b && (MInodesexpanded) <= c1*nodeLB){
					int temp1 = nodesExpanded;
					int temp2 = nodesReexpanded;
					s = StepIteration();
					printf("bound is %1.5f\n",bound);
					MInodesexpanded += (nodesExpanded - temp1);
					MInodesreexpanded += (nodesReexpanded -temp2);
					return s;
			}
			else if(MInodesreexpanded <= b && (MInodesexpanded) > c1*nodeLB) {
					printf("expand the remaining nodes with current bound from low to high g\n");
				    GetAllNodesfromQ();
					while (q_g.OpenSize() != 0){
						int temp1 = nodesExpanded;
						s = StepIterationUsingG(bound);
						if(s){
							break;
						}
						MInodesexpanded += (nodesExpanded - temp1);
					}
					printf("next bound is %1.5f\n",nextBound);
					fc_next = bound;
					return s;
			}
			else if(MInodesreexpanded > b && (MInodesexpanded) > c1*nodeLB) {
					printf("expand the remaining nodes with current bound from low to high g\n");
				    GetAllNodesfromQ();
					while (q_g.OpenSize() != 0){
						int temp1 = nodesExpanded;
						s = StepIterationUsingG(bound);
						if(s){
							break;
						}
						MInodesexpanded += (nodesExpanded - temp1);
					}
					printf("next bound is %1.5f\n",nextBound);
					fc_next = bound;
					return s;
			}
			else {
					printf("nodesExpanded is %d \n",nodesExpanded);
					printf("nodesReexpanded is %d \n",nodesReexpanded);
					GetAllNodesfromQ();
				    MODE = 1;
					MInodesexpanded = 0;
					MInodesreexpanded = 0;
					//nodeLB = nodesExpanded + nodesReexpanded;
					nodeLB = nodesExpanded ;
					nodesReexpanded = 0;
					nodesExpanded = nodeLB;
					SetupExponentialBinaryIteration(bound);
				    return false;
				}
		}
		else{
			return DoExponentialSingleSearchStep(bound);
		}
	}
	else{
		printf("Main iteration is complete\n");
		printf("Total nodes expanded  %d \n",nodesExpanded);
		printf("Total nodes reexpanded  %d \n",nodesReexpanded);
		//nodeLB = nodesExpanded + nodesReexpanded;
		nodeLB = nodesExpanded ;
		printf("Total nodes lower bound is %d\n",(c1)*nodeLB);
		MInodesexpanded = 0;
		MInodesreexpanded = 0;
		if(fc == DBL_MAX){
			printf("Grew Exponentially between 0 and %1.5f\n",fc_next);
		}
		else{
			printf("Grew Exponentially between %1.5f and %1.5f\n",fc,fc_next);
		}
		fc = fc_next;
		fc_next = DBL_MAX;
		MODE = 0;
		GetClosedNodesfromQ();  //set up f-queue by getting all the nodes closed from q_g and open where there already : so only the nodes closed
		//every time you finish an iteration you look into the q_g and see if there are any nodes closed and move them as closed in q_f
		if(nextBound != -1){
			bound = nextBound;
		    SetupIteration(bound);
		}
		else{
			uint64_t nodeid = q_f.Peek();
	        double nodeGCost = q_f.Lookup(nodeid).g;
	        double nodeFCost = nodeGCost + h->HCost(q_f.Lookup(nodeid).data, goal);
			bound = nodeFCost;
			SetupIteration(bound);
		}
		return false;
	}
	printf("Single Search Step over\n");
}


template <class state, class action>
bool IncrementalBGS<state, action>::DoExponentialSingleSearchStep(double cost)
{
	printf("Exponential Binary Single Search Step\n");
	bool s;
	if (!IterationCompleteUsingG() && data.nodesExpanded < data.workBound)
	{
		uint64_t tmp = nodesExpanded;
		s = StepIterationUsingG(cost);
		data.nodesExpanded += nodesExpanded-tmp;
		printf("data.nodesExpanded is %d\n",data.nodesExpanded);
		return s;
	}
	
	// Invariant - iteration complete *or* exceeded work limit
	// last search completed - set the interval from this search
	costInterval v;
	if (data.nodesExpanded >= data.workBound)
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
	}
	data.solutionInterval &= v;
	
	printf("--> Iteration complete - %llu expanded; target [%llu, %llu)\n", data.nodesExpanded, c1*data.nodeLB, c2*data.nodeLB);
	// Move to next iteration
	if (data.nodesExpanded >= c1*data.nodeLB && data.workBound == infiniteWorkBound)
	{
		printf("Expanded %llu - needed at least %llu\n", data.nodesExpanded, c1*data.nodeLB);
		if (data.solutionInterval.lowerBound == DBL_MAX) // No new nodes in iteration
			printf("[HIT]--Critical f in [%1.5f, %1.5f]\n", solutionCost, solutionCost);
		else
			printf("[HIT]--Critical f in [%1.5f, ∞]\n", data.solutionInterval.lowerBound);
		//get all the nodes from g-queue that are closed to f-queue as closed
		//the nodes discarded are there as open in the f-queue
		fc_next = bound;
		MODE = 0;
		data.solutionInterval.upperBound = DBL_MAX;
		nodesExpanded = data.nodesExpanded;
		return false;
	}
		
	// Check if bounds are equal or whether we fell within the node bounds
	if (!
		(fequal(data.solutionInterval.upperBound, data.solutionInterval.lowerBound) ||
		 (data.nodesExpanded >= c1*data.nodeLB && data.nodesExpanded < c2*data.nodeLB)
		 ))
	{
		if (data.solutionInterval.upperBound == DBL_MAX)
			printf("    ]--Critical f in [%1.5f, ∞]\n", data.solutionInterval.lowerBound);
		else
			printf("    ]--Critical f in [%1.5f, %1.5f]\n", data.solutionInterval.lowerBound, data.solutionInterval.upperBound);
			
		double nextCost;
		if (data.solutionInterval.upperBound == DBL_MAX)
		{
			nextCost = data.solutionInterval.lowerBound+pow(gamma, data.delta);
			fEquation = to_string_with_precision(data.solutionInterval.lowerBound, 0)+"+"+to_string_with_precision(gamma, 0)+"^"+to_string_with_precision(data.delta, 0)+"="+to_string_with_precision(nextCost, 0);
			stage = "EXP";
		}
		else {
			nextCost = (data.solutionInterval.lowerBound+data.solutionInterval.upperBound)/2.0;
			fEquation = "("+to_string_with_precision(data.solutionInterval.lowerBound, 0)+"+"+ to_string_with_precision(data.solutionInterval.upperBound, 0)+")/2"+"="+to_string_with_precision(nextCost, 0);
			stage = "BIN";
		}
		data.delta += 1;
		data.workBound = c2*data.nodeLB;
		//data.nodesExpanded = 0;  //since i am never starting the search from scratch, this should not be changed
		previousBound = bound;
		bound = nextCost;
		nextBound = -1;
		GetOpenNodesfromQ();
        printf("-> Starting iteration with f: %f; node limit: %llu\n", nextCost, data.workBound);
        return false;
	}

	if (data.solutionInterval.lowerBound == DBL_MAX)
		printf("[HIT]---Critical f in [∞, ∞]\n");
	else
		printf("[HIT]---Critical f in [%1.5f, ∞]\n", data.solutionInterval.lowerBound);
	// finished iteration with current bound - ready to start next IBEX/BTS iteration
	fc_next = bound;
	MODE = 0;
	nodesExpanded = data.nodesExpanded;
	return false;
}

template <class state, class action>
void IncrementalBGS<state, action>::Draw(Graphics::Display &disp) const
{
	double transparency = 1.0;
	if (q_f.size() == 0)
		return;
	uint64_t top = -1;
	//	double minf = 1e9, maxf = 0;
	if (q_f.OpenSize() > 0)
	{
		top = q_f.Peek();
	}
	for (unsigned int x = 0; x < q_f.size(); x++)
	{
		const auto &data = q_f.Lookat(x);
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
	env->Draw(disp, goal);}

template <class state, class action>
void IncrementalBGS<state, action>::OpenGLDraw()
{
}

template <class state, class action>
void IncrementalBGS<state, action>::GetAllNodesfromQwithReopening(){
	int n_size = q_f.size();
	//there are nodes in g-queue that are closed
	//put them into f-queue as closed
	//now take everything from f-queue and put it in q-queue as open
	printf("size of f-queue is %d\n",n_size);
	for (uint64_t i = 0 ; i < q_g.size();i++){
		state s = q_g.Lookup(i).data;
		double g_value = q_g.Lookup(i).g;
		double h_value = q_g.Lookup(i).h;
		uint64_t parentId = q_g.Lookup(i).parentID;
		printf("%d : g value is %1.5f, h value is %1.5f, total is %1.5f\n",i,g_value,h_value,g_value+h_value);
		printf("state hash %llu is added to q_g from q_f with parent id %llu\n",env->GetStateHash(s),parentId);
		if(q_g.Lookup(env->GetStateHash(s), i) == kClosedList){
			printf("adding a closed node\n");
			q_f.AddClosedNode(s,
						env->GetStateHash(s),
						g_value,
						h_value,
						parentId);
		}
		
	}
	q_g.Reset(env->GetMaxHash());
	//f-queue should have nodes as closed 
	//g-queue should have nodes as open
	for (uint64_t i = 0 ; i < q_f.size();i++){
		state s = q_f.Lookup(i).data;
		double g_value = q_f.Lookup(i).g;
		double h_value = q_f.Lookup(i).h;
		uint64_t parentId = q_f.Lookup(i).parentID;
		printf("%d : g value is %1.5f, h value is %1.5f, total is %1.5f\n",i,g_value,h_value,g_value+h_value);
		printf("state hash %llu is added to q_g from q_f with parent id %llu\n",env->GetStateHash(s),parentId);
		
			q_g.AddOpenNode(s,
						env->GetStateHash(s),
						g_value,
						h_value,
						parentId);
	}
		
	q_f.Reset(env->GetMaxHash());

	//f-queue has nothing
	//g-queue has all
	printf("sizes are %d and %d \n",q_f.size(),q_g.OpenSize());
}


template <class state, class action>
void IncrementalBGS<state, action>::GetClosedNodesfromQ(){
	int n_size = q_g.size();
	printf("size of g-queue is %d and closedsize is %d\n",n_size,q_g.ClosedSize());
	for (uint64_t i = 0 ; i < q_g.size();i++){
		state s = q_g.Lookup(i).data;
		double g_value = q_g.Lookup(i).g;
		double h_value = q_g.Lookup(i).h;
		uint64_t parentId = q_g.Lookup(i).parentID;
		printf("%d : g value is %1.5f, h value is %1.5f, total is %1.5f\n",i,g_value,h_value,g_value+h_value);
		printf("state hash %llu is added to q_g from q_f with parent id %llu\n",env->GetStateHash(s),parentId);
		if(q_g.Lookup(env->GetStateHash(s), i) == kClosedList){
			q_f.AddClosedNode(s,
						env->GetStateHash(s),
						g_value,
						h_value,
						parentId);
		}
		
		}
	q_g.Reset(env->GetMaxHash());
//	assert(n_size == q_f.size());
}
template <class state, class action>
void IncrementalBGS<state, action>::GetOpenNodesfromQ(){
	for (uint64_t i = 0 ; i < q_g.size();i++){
		state s = q_g.Lookup(i).data;
		double g_value = q_g.Lookup(i).g;
		double h_value = q_g.Lookup(i).h;
		uint64_t parentId = q_g.Lookup(i).parentID;
		printf("%d : g value is %1.5f, h value is %1.5f, total is %1.5f\n",i,g_value,h_value,g_value+h_value);
		printf("state hash %llu is added to q_g from q_f with parent id %llu\n",env->GetStateHash(s),parentId);
		if(q_g.Lookup(env->GetStateHash(s), i) == kClosedList){
		//	printf("adding a closed node\n");
			q_f.AddClosedNode(s,
						env->GetStateHash(s),
						g_value,
						h_value,
						parentId);
		}
		
		}
	q_g.Reset(env->GetMaxHash());
	for (uint64_t i = 0 ; i < q_f.size();i++){
		state s = q_f.Lookup(i).data;
		double g_value = q_f.Lookup(i).g;
		double h_value = q_f.Lookup(i).h;
		uint64_t parentId = q_f.Lookup(i).parentID;
		printf("%d : g value is %1.5f, h value is %1.5f, total is %1.5f\n",i,g_value,h_value,g_value+h_value);
		printf("state hash %llu is added to q_g from q_f with parent id %llu\n",env->GetStateHash(s),parentId);
		if(q_f.Lookup(env->GetStateHash(s), i) == kOpenList){
		q_g.AddOpenNode(s,
						env->GetStateHash(s),
						g_value,
						h_value,
						parentId);
		}
		else if(q_f.Lookup(env->GetStateHash(s), i) == kClosedList){
		q_g.AddClosedNode(s,
						env->GetStateHash(s),
						g_value,
						h_value,
						parentId);
		}
		}
	q_f.Reset(env->GetMaxHash());
	printf("sizes are %d and %d \n",q_g.size(),q_g.ClosedSize());
}

template <class state, class action>
void IncrementalBGS<state, action>::GetAllNodesfromQ(){
	int n_size = q_f.size();
	//all nodes in f-queue(open+closed)
	q_g.Reset(env->GetMaxHash());
    for (uint64_t i = 0 ; i < q_f.size();i++){
		state s = q_f.Lookup(i).data;
		double g_value = q_f.Lookup(i).g;
		double h_value = q_f.Lookup(i).h;
		uint64_t parentId = q_f.Lookup(i).parentID;
		printf("%d : g value is %1.5f, h value is %1.5f, total is %1.5f\n",i,g_value,h_value,g_value+h_value);
		printf("state hash %llu is added to q_g from q_f with parent id %llu\n",env->GetStateHash(s),parentId);
		if(q_f.Lookup(env->GetStateHash(s), i) == kOpenList){
		q_g.AddOpenNode(s,
						env->GetStateHash(s),
						g_value,
						h_value,
						parentId);
		}
		else if(q_f.Lookup(env->GetStateHash(s), i) == kClosedList){
		q_g.AddClosedNode(s,
						env->GetStateHash(s),
						g_value,
						h_value,
						parentId);
		}
		}
	
		
	q_f.Reset(env->GetMaxHash());

	//all nodes in q-queue(open+closed)
	printf("sizes are %d and %d \n",q_f.size(),q_g.OpenSize());
}



#endif /* IncrementalBGS_h */
