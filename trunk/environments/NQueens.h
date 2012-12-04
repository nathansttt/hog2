//
//  NQueens.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 9/18/11.
//  Copyright 2011 University of Denver. All rights reserved.
//

#ifndef NQUEENS_H
#define NQUEENS_H

#include "SearchEnvironment.h"
#include "FPUtil.h"
#include <vector>

class NQueenState
{
public:
	NQueenState(int numQueens = 8)
	{
		locs.resize(numQueens);
		for (unsigned int x = 0; x < numQueens; x++)
		{
			locs[x] = -1;
		}
	}
	std::vector<int> locs;
};

static bool operator==(const NQueenState &l1, const NQueenState &l2)
{
	if (l1.locs.size() != l2.locs.size())
		return false;
	
	for (unsigned int x = 0; x < l1.locs.size(); x++)
		if (l1.locs[x] != l2.locs[x])
			return false;
	return true;
}


class NQueenAction
{
public:
	int loc, value;
};

class NQueens : public SearchEnvironment<NQueenState, NQueenAction>
{
public:
	NQueens() { }
	~NQueens() {}
	virtual void GetSuccessors(const NQueenState &nodeID, std::vector<NQueenState> &neighbors) const;
	virtual void GetActions(const NQueenState &nodeID, std::vector<NQueenAction> &actions) const;
	//virtual int GetNumSuccessors(const NQueenState &stateID) const;
	virtual NQueenAction GetAction(const NQueenState &s1, const NQueenState &s2) const;
	virtual void ApplyAction(NQueenState &s, NQueenAction a) const;
	
	virtual void GetNextState(const NQueenState &, NQueenAction , NQueenState &) const;
	
	virtual bool InvertAction(NQueenAction &a) const { return false; }
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const NQueenState &node1, const NQueenState &node2) { return 0; }
	
	/** Heuristic value between node and the stored goal. Asserts that the
	 goal is stored **/
	virtual double HCost(const NQueenState &node)
	{ return 0; }
	
	virtual double GCost(const NQueenState &node1, const NQueenState &node2) { return 1.0; }
	virtual double GCost(const NQueenState &node, const NQueenAction &act) { return 1.0; }
	virtual bool GoalTest(const NQueenState &node, const NQueenState &goal) { return GoalTest(node); }
	
	/** Goal Test if the goal is stored **/
	virtual bool GoalTest(const NQueenState &node);
	
	virtual uint64_t GetStateHash(const NQueenState &node) const { return 0; }
	virtual uint64_t GetActionHash(NQueenAction act) const { return 0; }
	
	int NumCollisions(const NQueenState &node) const;

	//virtual double GetPathLength(std::vector<NQueenState> &neighbors);
	
	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const NQueenState&) const;
	/** Draw the transition at some percentage 0...1 between two states */
	virtual void OpenGLDraw(const NQueenState&, const NQueenState&, float) const;
	virtual void OpenGLDraw(const NQueenState&, const NQueenAction&) const;
private:
};

#endif