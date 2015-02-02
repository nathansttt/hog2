/*
 *  TopSpin.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/4/14.
 *  Copyright 2014 Nathan Sturtevant, University of Denver. All rights reserved.
 *
 */

#ifndef TopSpin_H
#define TopSpin_H

#include <stdint.h>
#include <iostream>
#include "SearchEnvironment.h"
#include "PermutationPuzzleEnvironment.h"
#include "UnitSimulation.h"
#include "GraphEnvironment.h"
#include "Graph.h"
#include <sstream>
#include <unordered_map>

class TopSpinState {
public:
	TopSpinState(int N = 20, int k = 4)
	{
		puzzle.resize(N);
		for (unsigned int x = 0; x < puzzle.size(); x++)
			puzzle[x] = x;
	}
	void Reset()
	{
		for (unsigned int x = 0; x < puzzle.size(); x++)
			puzzle[x] = x;
	}
	std::vector<int> puzzle;
};

/**
 * Actions are the first tile that gets swapped.
 */
typedef int TopSpinAction;

static std::ostream& operator <<(std::ostream & out, const TopSpinState &loc)
{
	for (unsigned int x = 0; x < loc.puzzle.size(); x++)
		out << loc.puzzle[x] << " ";
	return out;
}

static bool operator==(const TopSpinState &l1, const TopSpinState &l2)
{
	if (l1.puzzle.size() != l2.puzzle.size())
		return false;
	for (unsigned int x = 0; x < l1.puzzle.size(); x++)
	{
		if (l1.puzzle[x] != l2.puzzle[x])
			return false;
	}
	return true;
}

class TopSpin : public PermutationPuzzleEnvironment<TopSpinState, TopSpinAction> {
public:
	TopSpin(unsigned int N = 20, unsigned int k = 4);
	~TopSpin();
	void SetWeighted(bool w) { weighted = w; }
	bool GetWeighted() { return weighted; }
	void SetPruneSuccessors(bool val)
	{ if (val) ComputeMovePruning(); pruneSuccessors = val; history.resize(0); }
	void GetSuccessors(const TopSpinState &stateID, std::vector<TopSpinState> &neighbors) const;
	void GetActions(const TopSpinState &stateID, std::vector<TopSpinAction> &actions) const;
	void ApplyAction(TopSpinState &s, TopSpinAction a) const;
	void UndoAction(TopSpinState &s, TopSpinAction a) const;
	bool InvertAction(TopSpinAction &a) const;
	static unsigned GetParity(TopSpinState &state);

	OccupancyInterface<TopSpinState, TopSpinAction> *GetOccupancyInfo() { return 0; }
	double HCost(const TopSpinState &state1, const TopSpinState &state2);
//	double HCost(const TopSpinState &state1);

	double GCost(const TopSpinState &state1, const TopSpinState &state2);
	double GCost(const TopSpinState &, const TopSpinAction &);
	bool GoalTest(const TopSpinState &state, const TopSpinState &goal);

	bool GoalTest(const TopSpinState &s);

	//void LoadPDB(char *fname, const std::vector<int> &tiles, bool additive);

	uint64_t GetActionHash(TopSpinAction act) const;
	void OpenGLDraw() const;
	void OpenGLDraw(const TopSpinState &s) const;
	void OpenGLDraw(const TopSpinState &l1, const TopSpinState &l2, float v) const;
	void OpenGLDraw(const TopSpinState &, const TopSpinAction &) const { /* currently not drawing moves */ }
	void StoreGoal(TopSpinState &); // stores the locations for the given goal state

	/** Returns stored goal state if it is stored.**/
	TopSpinState Get_Goal(){
		return goal;
	}

	virtual const std::string GetName();

	void ClearGoal() { } // clears the current stored information of the goal

	bool IsGoalStored(){return true;} // returns if a goal is stored or not

	bool State_Check(const TopSpinState &to_check) { return true; }

	void PrintHStats()
	{
		printf("-\t");
		for (int x = 0; x < hDist.size(); x++)
			printf("%d\t", x);
		printf("\n");
		for (int y = 0; y <= 12; y++)
		{
			printf("%d\t", y);
			for (int x = 0; x < hDist.size(); x++)
			{
				if (y >= hDist[x].size())
					printf("0\t");
				else
					printf("%d\t", hDist[x][y]);
			}
			printf("\n");
		}
	}
private:
	void ComputeMovePruning();
	void RecursiveMovePruning(int depth, TopSpinState &state);

	std::unordered_map<uint64_t,bool> pruningMap;
	std::unordered_map<uint64_t,int> pruningCostMap;
	
	unsigned int numTiles, swapDiameter;
	std::vector<TopSpinAction> operators;
	std::vector<bool> movePrune;
	
	// stores the heuristic value of each tile-position pair indexed by the tile value (0th index is empty)
	unsigned **h_increment;
	TopSpinState goal;
	std::vector<std::vector<int> > hDist;

	mutable std::vector<TopSpinAction> history;
	bool pruneSuccessors;
	bool weighted;
};

typedef UnitSimulation<TopSpinState, TopSpinAction, TopSpin> TopSpinSimulation;

#endif
