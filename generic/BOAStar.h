//
//  BOAStar.h
//  HOG2 ObjC
//
//  Created by Nathan Sturtevant on 2/16/23.
//  Copyright © 2023 MovingAI. All rights reserved.
//
// Bi-objective A* By Ulloa et, 2020

#include "TemplateAStar.h"

#ifndef BOAStar_h
#define BOAStar_h

template <class state, class action, class environment>
class BOAStar //: public GenericSearchAlgorithm<state,action,environment>
{
public:
	BOAStar() { ResetNodeCount(); };
	virtual ~BOAStar() {}
	void GetPath(environment *env1, environment *env2, Heuristic<state> *h1, Heuristic<state> *h2,
				 const state& from, const state& to, std::vector<state> &thePath);
//	void GetPath(environment *, const state& , const state& , std::vector<action> & );
	
	state goal, start;
	
	bool InitializeSearch(environment *env1, environment *env2, Heuristic<state> *h1, Heuristic<state> *h2,
						  const state& from, const state& to, std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	
	void ExtractPathToStart(state &node, std::vector<state> &thePath);
	void ExtractPathToStartFromID(size_t node, std::vector<state> &thePath);
	virtual const char *GetName() { return "BOAStar"; }
	
	uint64_t GetUniqueNodesExpanded() { return uniqueNodesExpanded; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; uniqueNodesExpanded = 0; }
//	int GetMemoryUsage();
//
	size_t GetNumItems() const { return allStates.size(); }
	bool IsOpen(size_t item) const { return allStates[item].open; }
	state GetItem(size_t item) const { return allStates[item].s; }
	float GetItemGCost(size_t item) const { return allStates[item].gCost; }
	float GetItemHCost(size_t item) const { return allStates[item].hCost; }
	
//	void SetHeuristic(Heuristic<state> *h) { theHeuristic = h; }
	
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	
	void LogFinalStats(StatCollection *) {}
//
	void Draw(Graphics::Display &d) const;
	void DrawFrontier(Graphics::Display &d) const;

	void SetPhi(std::function<double(double, double)> p)
	{
		phi = p;
	}
	double Phi(double h, double g) const
	{
		return phi(h, g);
	}
	void SetOptimalityBound(double w)
	{
		phi = [w](double h, double g){ return g/w+h; };
		bound = w;
	}
	double GetOptimalityBound() { return bound; }
private:
	void GetMinFOnOpen() const;
	size_t GetBestStateOnOpen() const;

	void DrawOpen(Graphics::Display &d) const;
	void Expand(size_t which);
	void UpdateCost(size_t next, size_t parent);
	void AddState(const state &s, size_t parent);
	size_t Lookup(const state &s);
	uint64_t nodesTouched, nodesExpanded;
	
	struct stateInfo {
		stateInfo() {gmin = std::numeric_limits<float>::max();}
		bool Update(float v) {if (fless(v, gmin)) {gmin = std::min(gmin, v); return true; } return false; }
		float gmin;
	};
	struct item {
		state s;
		float gCost1, gCost2;
		float hCost1, hCost2;
		float fCost1() const { return gCost1+hCost1; }
		float fCost2() const { return gCost2+hCost2; }
		bool open;
		size_t parent;
	};
	// stores gmin for all states seen thus far
	std::unordered_map<uint64_t, stateInfo> hashTable;
	std::vector<item> allStates;
	std::vector<state> neighbors;
//	std::vector<item> closed;
//	std::vector<uint64_t> neighborID;
//	std::vector<double> edgeCosts;
//	std::vector<dataLocation> neighborLoc;

	std::vector<item> goals;
	std::vector<state> solution;
	environment *e1, *e2;
	Heuristic<state> *h1, *h2;
	//	double bestSolution;
	double bound;
	uint64_t uniqueNodesExpanded;
	//Heuristic<state> *theHeuristic;

	std::function<double(double, double)> phi;
};

template <class state, class action, class environment>
void BOAStar<state, action, environment>::GetPath(environment *env1, environment *env2, Heuristic<state> *h1, Heuristic<state> *h2,
												 const state& from, const state& to, std::vector<state> &thePath)
{
	if (!InitializeSearch(env1, env2, h1, h2, from, to, thePath))
	{
		return;
	}
	while (!DoSingleSearchStep(thePath))
	{
	}
}

template <class state, class action, class environment>
bool BOAStar<state, action, environment>::InitializeSearch(environment *env1, environment *env2,
														  Heuristic<state> *h1, Heuristic<state> *h2,
														  const state& from, const state& to, std::vector<state> &thePath)
{
	thePath.resize(0);
	e1 = env1;
	e2 = env2;
	allStates.resize(0);
	solution.clear();
	ResetNodeCount();
	start = from;
	goal = to;
	goals.resize(0);
	this->h1 = h1;
	this->h2 = h2;
	allStates.push_back({start, 0, 0,
		(float)h1->HCost(start, goal),
		(float)h2->HCost(start, goal),
		true, 0});
	
	return true; // success
}

template <class state, class action, class environment>
bool BOAStar<state, action, environment>::DoSingleSearchStep(std::vector<state> &thePath)
{
	// 1. Find min lexicographical state on open
	size_t which = GetBestStateOnOpen();

	if (which == allStates.size()) // search is done
	{
		std::cout << "Search done; " << GetNodesExpanded() << " expanded\n";
		return true;
	}
	
	// 2. Prune this state if it is dominated
	allStates[which].open = false;

//	printf("Goal gmin: %f\n", hashTable[e1->GetStateHash(goal)].gmin);
	
	// Didn't decrease g2; since f1 is increasing, node is dominated
	//if (i.gCost2 >= hashTable[e1->GetStateHash(neighbors[x])].gmin)
	if (allStates[which].gCost2 >= hashTable[e1->GetStateHash(allStates[which].s)].gmin)
	{
//		std::cout << "dominated1\n";
		return false;
	}

//	printf("Goal gmin: %f\n", hashTable[e1->GetStateHash(goal)].gmin);
	// Can't reach the goal on a non-dominated path
	if (allStates[which].fCost2() >= hashTable[e1->GetStateHash(goal)].gmin)
	{
//		std::cout << "dominated2\n";
		return false;
	}

	hashTable[e1->GetStateHash(allStates[which].s)].gmin = allStates[which].gCost2;

	
	// 3. Check for goal
	if (e1->GoalTest(allStates[which].s, goal))
	{
		std::cout << "Adding goal " << allStates[which].s << " with costs {" << allStates[which].gCost1 << ", " << allStates[which].gCost2 << "}\n";
		goals.push_back(allStates[which]);
		allStates[which].open = false;
	}
	
	// 4. Expand state
	Expand(which);

	return false;
}

template <class state, class action, class environment>
size_t BOAStar<state, action, environment>::GetBestStateOnOpen() const
{
	bool found = false;
	size_t which = allStates.size();
	for (size_t x = 0; x < allStates.size(); x++)
	{
		if (allStates[x].open &&
			(!found ||
			 (fless(allStates[x].fCost1(), allStates[which].fCost1()) ||
			  (fequal(allStates[x].fCost1(), allStates[which].fCost1()) &&
			   fless(allStates[x].fCost2(), allStates[which].fCost2()))
			  )
			 ))
		{
			found = true;
			which = x;
		}
	}
//	std::cout << "x" << which << "-" << allStates[which].s << " [" << goal << "]";
//	printf(" Next expansions: f1: %1.2f, f2: %1.2f\n", allStates[which].fCost1(), allStates[which].fCost2());
	return which;
}

template <class state, class action, class environment>
void BOAStar<state, action, environment>::Expand(size_t which)
{
	nodesExpanded++;
	e1->GetSuccessors(allStates[which].s, neighbors);
	for (int x = 0; x < neighbors.size(); x++)
	{
//		std::cout << "g" << neighbors[x];
		item i =
		{neighbors[x],
			static_cast<float>(allStates[which].gCost1+e1->GCost(allStates[which].s, neighbors[x])),
			static_cast<float>(allStates[which].gCost2+e2->GCost(allStates[which].s, neighbors[x])),
			static_cast<float>(h1->HCost(neighbors[x], goal)),
			static_cast<float>(h2->HCost(neighbors[x], goal)),
			true,
			which
		};
		// Check if state is dominated
		// Already have shorter gmin for neighbor
		if (i.gCost2 >= hashTable[e1->GetStateHash(neighbors[x])].gmin)
		{
//			std::cout << "prune\n";
			continue;
		}

		// Can't reach the goal on a non-dominated path
		if (i.fCost2() >= hashTable[e1->GetStateHash(goal)].gmin)
		{
//			std::cout << "prune\n";
			continue;
		}

		allStates.push_back(i);
//		std::cout << "add " << allStates.size()-1 << "\n";
	}
}

template <class state, class action, class environment>
void BOAStar<state, action, environment>::Draw(Graphics::Display &d) const
{
	for (size_t x = 0; x < allStates.size(); x++)
	{
		if (allStates[x].open)
		{
			e1->SetColor(Colors::green);
			e1->Draw(d, allStates[x].s);
		}
		else {
			e1->SetColor(Colors::red);
			e1->Draw(d, allStates[x].s);
		}
	}
	e1->SetColor(Colors::blue);
	e1->Draw(d, goal);
}

template <class state, class action, class environment>
void BOAStar<state, action, environment>::DrawFrontier(Graphics::Display &d) const
{
	float maxf = 0;
	for (int x = 0; x < goals.size(); x++)
	{
		maxf = std::max(maxf, goals[x].fCost1());
		maxf = std::max(maxf, goals[x].fCost2());
	}
	// axes
	d.DrawLine({-1, 1}, {-1, -1}, 0.025, Colors::white);
	d.DrawLine({-1, 1}, {1, 1}, 0.025, Colors::white);
	for (int x = 0; x < goals.size(); x++)
	{
		Graphics::point p(-1+2*goals[x].fCost1()/maxf, 1-2*(goals[x].fCost2()/maxf));
		d.FillCircle(p, 0.025, Colors::lightblue);
		maxf = std::max(maxf, goals[x].fCost1());
		maxf = std::max(maxf, goals[x].fCost2());
	}

	//	for (size_t x = 0; x < allStates.size(); x++)
//	{
//		if (allStates[x].open)
//		{
//			e1->SetColor(Colors::green);
//			e1->Draw(d, allStates[x].s);
//		}
//		else {
//			e1->SetColor(Colors::red);
//			e1->Draw(d, allStates[x].s);
//		}
//	}
//	e1->SetColor(Colors::blue);
//	e1->Draw(d, goal);
}


#endif /* BOAStar_h */
