//
//  MM.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/27/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#ifndef MM_h
#define MM_h

#include "AStarOpenClosed.h"
#include "FPUtil.h"

template <class state>
struct MMCompare {
	bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
	{
		double p1 = std::max(i1.g+i1.h, i1.g*2);
		double p2 = std::max(i2.g+i2.h, i2.g*2);
//		double p1 = i1.g+i1.h;
//		double p2 = i2.g+i2.h;
		if (fequal(p1, p2))
		{
			//return (fgreater(i1.g, i2.g)); // low g-cost over high
			return (fless(i1.g, i2.g)); // high g-cost over low
		}
		return (fgreater(p1, p2)); // low priority over high
	}
};


template <class state, class action, class environment, class priorityQueue = AStarOpenClosed<state, MMCompare<state>> >
class MM {
public:
	MM() { forwardHeuristic = 0; backwardHeuristic = 0; env = 0; ResetNodeCount(); }
	virtual ~MM() {}
	void GetPath(environment *env, const state& from, const state& to,
				 Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);
	bool InitializeSearch(environment *env, const state& from, const state& to,
						  Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	
	priorityQueue forwardQueue, backwardQueue;
	state goal, start;
	
	
	virtual const char *GetName() { return "MM"; }
	
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; }
	
//	bool GetClosedListGCost(const state &val, double &gCost) const;
//	unsigned int GetNumOpenItems() { return openClosedList.OpenSize(); }
//	inline const AStarOpenClosedData<state> &GetOpenItem(unsigned int which) { return openClosedList.Lookat(openClosedList.GetOpenItem(which)); }
	inline const int GetNumForwardItems() { return forwardQueue.size(); }
	inline const AStarOpenClosedData<state> &GetForwardItem(unsigned int which) { return forwardQueue.Lookat(which); }
	inline const int GetNumBackwardItems() { return backwardQueue.size(); }
	inline const AStarOpenClosedData<state> &GetBackwardItem(unsigned int which) { return backwardQueue.Lookat(which); }
//	bool HaveExpandedState(const state &val)
//	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key) != kNotFound; }
//	
//	void SetForwardHeuristic(Heuristic<state> *h) { forwardHeuristic = h; }
//	void SetBackwardHeuristic(Heuristic<state> *h) { backwardHeuristic = h; }
	
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	
	//void FullBPMX(uint64_t nodeID, int distance);
	
	void OpenGLDraw() const;
	
//	void SetWeight(double w) {weight = w;}
private:
	void OpenGLDraw(const priorityQueue &queue) const;
	
	void Expand(priorityQueue &current,
				priorityQueue &opposite,
				Heuristic<state> *heuristic, const state &target);
	uint64_t nodesTouched, nodesExpanded;
	state middleNode;
	double currentCost;
	std::vector<state> neighbors;
	environment *env;
	
	Heuristic<state> *forwardHeuristic;
	Heuristic<state> *backwardHeuristic;
};

template <class state, class action, class environment, class priorityQueue>
void MM<state, action, environment, priorityQueue>::GetPath(environment *env, const state& from, const state& to,
			 Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath)
{
	if (InitializeSearch(env, from, to, forward, backward, thePath) == false)
		return;
	
	while (!DoSingleSearchStep(thePath))
	{ }
}

template <class state, class action, class environment, class priorityQueue>
bool MM<state, action, environment, priorityQueue>::InitializeSearch(environment *env, const state& from, const state& to,
																	 Heuristic<state> *forward, Heuristic<state> *backward,
																	 std::vector<state> &thePath)
{
	this->env = env;
	forwardHeuristic = forward;
	backwardHeuristic = backward;
	currentCost = DBL_MAX;
	forwardQueue.Reset();
	backwardQueue.Reset();
	ResetNodeCount();
	thePath.resize(0);
	start = from;
	goal = to;
	if (start == goal)
		return false;

	forwardQueue.AddOpenNode(start, env->GetStateHash(start), 0, forwardHeuristic->HCost(start, goal));
	backwardQueue.AddOpenNode(goal, env->GetStateHash(goal), 0, forwardHeuristic->HCost(goal, start));
	
	return true;
}

template <class state, class action, class environment, class priorityQueue>
bool MM<state, action, environment, priorityQueue>::DoSingleSearchStep(std::vector<state> &thePath)
{
	if (forwardQueue.OpenSize() == 0 && backwardQueue.OpenSize() == 0)
	{
		return true;
	}
	
	if (forwardQueue.OpenSize() == 0)
		Expand(backwardQueue, forwardQueue, backwardHeuristic, start);

	if (backwardQueue.OpenSize() == 0)
		Expand(forwardQueue, backwardQueue, forwardHeuristic, goal);

	uint64_t forward = forwardQueue.Peek();
	uint64_t backward = backwardQueue.Peek();
	
	const AStarOpenClosedData<state> &nextForward = forwardQueue.Lookat(forward);
	const AStarOpenClosedData<state> &nextBackward = backwardQueue.Lookat(backward);

	double p1 = std::max(nextForward.g+nextForward.h, nextForward.g*2);
	double p2 = std::max(nextBackward.g+nextBackward.h, nextBackward.g*2);
	
	if (fless(p1, p2))
	{
		Expand(forwardQueue, backwardQueue, forwardHeuristic, goal);
	}
	else if (fless(p2, p1))
	{
		Expand(backwardQueue, forwardQueue, backwardHeuristic, start);
	}
	else { // equal priority
		if (fless(nextForward.g, nextBackward.g))
		{
			Expand(forwardQueue, backwardQueue, forwardHeuristic, goal);
		}
		else if (fless(nextBackward.g, nextForward.g))
		{
			Expand(backwardQueue, forwardQueue, backwardHeuristic, start);
		}
		else {
			Expand(forwardQueue, backwardQueue, forwardHeuristic, goal);
		}
	}
	// check if we can terminate
	if (fless(currentCost, DBL_MAX))
	{
		// TODO: make this more efficient
		double minForwardG = DBL_MAX;
		double minBackwardG = DBL_MAX;
		double minForwardF = DBL_MAX;
		double minBackwardF =  DBL_MAX;
		double forwardP;
		double backwardP;
		for (int x = 0; x < backwardQueue.OpenSize(); x++)
		{
			auto item = backwardQueue.Lookat(backwardQueue.GetOpenItem(x));
			if (fless(item.g, minBackwardG))
				minBackwardG = item.g;
			if (fless(item.g+item.h, minBackwardF))
				minBackwardF = item.g+item.h;
		}
		for (int x = 0; x < forwardQueue.OpenSize(); x++)
		{
			auto item = forwardQueue.Lookat(forwardQueue.GetOpenItem(x));
			if (fless(item.g, minForwardG))
				minForwardG = item.g;
			if (fless(item.g+item.h, minForwardF))
				minForwardF = item.g+item.h;
		}
		{
			auto iB = backwardQueue.Lookat(backwardQueue.Peek());
			backwardP = std::max(iB.g+iB.h, iB.g*2);
			auto iF = forwardQueue.Lookat(forwardQueue.Peek());
			forwardP = std::max(iF.g+iF.h, iF.g*2);
		}
		bool done = false;
		if (!fgreater(currentCost, minForwardF))
		{
			printf("Terminated on forwardf\n");
			done = true;
		}
		if (!fgreater(currentCost, minBackwardF))
		{
			printf("Terminated on backwardf\n");
			done = true;
		}
		if (!fgreater(currentCost, minForwardG+minBackwardG+1)) // TODO: epsilon
		{
			printf("Terminated on g+g+epsilon\n");
			done = true;
		}
		if (!fgreater(currentCost, forwardP))
		{
			printf("Terminated on forwardP\n");
			done = true;
		}
		if (!fgreater(currentCost, backwardP))
		{
			printf("Terminated on backwardP\n");
			done = true;
		}
		// for now, always terminate
		if (done)
			return true;
	}
	return false;
}

template <class state, class action, class environment, class priorityQueue>
void MM<state, action, environment, priorityQueue>::Expand(priorityQueue &current,
														   priorityQueue &opposite,
														   Heuristic<state> *heuristic, const state &target)
{
	uint64_t nextID = current.Close();
	nodesExpanded++;

	env->GetSuccessors(current.Lookup(nextID).data, neighbors);
	for (auto &succ : neighbors)
	{
		nodesTouched++;
		uint64_t childID;
		auto loc = current.Lookup(env->GetStateHash(succ), childID);
		switch (loc)
		{
			case kClosedList: // ignore
				break;
			case kOpenList: // update cost if needed
			{
				double edgeCost = env->GCost(current.Lookup(nextID).data, succ);
				if (fless(current.Lookup(nextID).g+edgeCost, current.Lookup(childID).g))
				{
					current.Lookup(childID).parentID = nextID;
					current.Lookup(childID).g = current.Lookup(nextID).g+edgeCost;
					current.KeyChanged(childID);
					
					// TODO: check if we improved the current solution?
					uint64_t reverseLoc;
					auto loc = opposite.Lookup(env->GetStateHash(succ), reverseLoc);
					if (loc == kOpenList)
					{
						if (fless(current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g, currentCost))
						{
							// TODO: store current solution
							printf("Potential updated solution found, cost: %1.2f + %1.2f = %1.2f\n",
								   current.Lookup(nextID).g+edgeCost,
								   opposite.Lookup(reverseLoc).g,
								   current.Lookup(nextID).g+edgeCost+opposite.Lookup(reverseLoc).g);
							currentCost = current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g;
							middleNode = succ;
						}
					}
				}
			}
				break;
			case kNotFound:
			{
				double edgeCost = env->GCost(current.Lookup(nextID).data, succ);
				current.AddOpenNode(succ,
									env->GetStateHash(succ),
									current.Lookup(nextID).g+edgeCost,
									heuristic->HCost(succ, target),
									nextID);
				// check for solution
				uint64_t reverseLoc;
				auto loc = opposite.Lookup(env->GetStateHash(succ), reverseLoc);
				if (loc == kOpenList)
				{
					if (fless(current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g, currentCost))
					{
						// TODO: store current solution
						printf("Potential solution found, cost: %1.2f + %1.2f = %1.2f\n",
							   current.Lookup(nextID).g+edgeCost,
							   opposite.Lookup(reverseLoc).g,
							   current.Lookup(nextID).g+edgeCost+opposite.Lookup(reverseLoc).g);
						currentCost = current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g;
						middleNode = succ;
					}
				}
			}
		}
	}
}

template <class state, class action, class environment, class priorityQueue>
void MM<state, action, environment, priorityQueue>::OpenGLDraw() const
{
	OpenGLDraw(forwardQueue);
	OpenGLDraw(backwardQueue);
}

template <class state, class action, class environment, class priorityQueue>
void MM<state, action, environment, priorityQueue>::OpenGLDraw(const priorityQueue &queue) const
{
	double transparency = 0.9;
	if (queue.size() == 0)
		return;
	uint64_t top = -1;
	//	double minf = 1e9, maxf = 0;
	if (queue.OpenSize() > 0)
	{
		top = queue.Peek();
	}
	for (unsigned int x = 0; x < queue.size(); x++)
	{
		const AStarOpenClosedData<state> &data = queue.Lookat(x);
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
			env->SetColor(1.0, 0.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
	}
}

#endif /* MM_h */
