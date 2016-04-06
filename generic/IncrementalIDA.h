//
//  IncrementalDFID.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 3/24/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef IncrementalDFID_h
#define IncrementalDFID_h

#include "Heuristic.h"

template <class state, class action>
class IncrementalIDA {
public:
	IncrementalIDA(int initialBound = 0) :bound(initialBound), initialBound(initialBound), nextBound(initialBound) {}
	void GetPath(SearchEnvironment<state, action> *env, state from, state to, Heuristic<state> *h,
				 std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<action> &thePath);
	bool DoSingleSearchStep(SearchEnvironment<state, action> *env, state from, state to,
							 Heuristic<state> *h, std::vector<state> &thePath);
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; }
	void Reset() { bound = initialBound; nextBound = initialBound; path.clear(); history.clear(); }
	void OpenGLDraw();
	state GetCurrentState() const { if (path.size() > 0) return path.back(); return start; }
	void GetCurrentPath(std::vector<state> &p) const { p = path; }
	int GetCurrentFLimit() { return bound; }
	int GetNextFLimit() { return nextBound; }
private:
	unsigned long nodesExpanded, nodesTouched;
	
	bool DoIteration(SearchEnvironment<state, action> *env,
					 state parent, state currState,
					 std::vector<state> &thePath, double bound, double g);
	bool DoIteration(SearchEnvironment<state, action> *env,
					 action forbiddenAction, state &currState,
					 std::vector<action> &thePath, double bound, double g);
	
	std::vector<std::pair<state, int>> history;
	std::vector<state> path;
	state start;
	int bound;
	int initialBound;
	int nextBound;
	SearchEnvironment<state, action> *env;
	std::vector<state> succ;
};

template <class state, class action>
void IncrementalIDA<state, action>::GetPath(SearchEnvironment<state, action> *env, state from, state to,
											Heuristic<state> *h, std::vector<state> &thePath)
{
	while (!DoSingleSearchStep(env, from, to, thePath))
	{}
}

template <class state, class action>
void IncrementalIDA<state, action>::GetPath(SearchEnvironment<state, action> *env, state from, state to,
			 std::vector<action> &thePath)
{
	
}

template <class state, class action>
bool IncrementalIDA<state, action>::DoSingleSearchStep(SearchEnvironment<state, action> *env, state from, state to,
													   Heuristic<state> *h, std::vector<state> &thePath)
{
	if (history.size() == 0)
	{
		start = from;
		history.push_back({from, 0});
		bound = nextBound;
		nextBound = -1;
		printf("Starting iteration bound %d\n", bound);
	}

	this->env = env;
	int depth = history.back().second;
	state currState = history.back().first;
	env->GetSuccessors(currState, succ);
	
	if (path.size() > depth)
	{
		path.pop_back();

		// terminating on next step - for visualization only! (avoids flicker)
		if (path.size() <= depth && env->GoalTest(currState, to))
			path.push_back(currState);
		return false;
	}
	path.push_back(currState);

	// Later than normal - for the purposes of drawing nicely
	if (env->GoalTest(currState, to))
	{
		printf("Done!");
		return true;
	}
	
	history.pop_back();
	double f = depth+h->HCost(currState, to);
	if (f <= bound)
	{
		for (int x = succ.size()-1; x >= 0; x--)
		{
			if (path.size() < 2 || path[path.size()-2] != succ[x])
				history.push_back({succ[x], depth+1});
		}
	}
	else {
		if (nextBound == -1)
			nextBound = depth+h->HCost(currState, to);
		else if (f < nextBound)
			nextBound = f;
	}
	return false;
}

template <class state, class action>
void IncrementalIDA<state, action>::OpenGLDraw()
{
//	for (auto x : history)
//		env->OpenGLDraw(x.first);
	for (int x = 1; x < path.size(); x++)
		env->GLDrawLine(path[x-1], path[x]);
}


#endif /* IncrementalDFID_h */
