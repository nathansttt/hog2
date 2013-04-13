//
//  RubiksCube.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 4/6/13.
//  Copyright (c) 2013 University of Denver. All rights reserved.
//

#include "RubiksCube.h"
#include <cassert>

void RubiksCube::GetSuccessors(const RubiksState &nodeID, std::vector<RubiksState> &neighbors) const
{
	assert(false);
}

void RubiksCube::GetActions(const RubiksState &nodeID, std::vector<RubiksAction> &actions) const
{
	actions.resize(0);
	if (!pruneSuccessors || history.size() == 0)
	{
		for (int x = 0; x < 18; x++)
			actions.push_back(x);
	}
	else {
		// 0, 5, 2, 4, 1, 3

		for (int x = 0; x < 18; x++)
		{
			// 1. after any face you can't turn the same face again
			if (x/3 == history.back()/3)
				continue;

			// 2. after faces 5, 4, 3 you can't turn 0, 2, 1 respectively
			if ((1 == (history.back()/3)%2) &&
				(x/3+1 == history.back()/3))
				continue;
			
			actions.push_back(x);
		}
	}
}

RubiksAction RubiksCube::GetAction(const RubiksState &s1, const RubiksState &s2) const
{
	assert(false);
	return 0;
}

void RubiksCube::ApplyAction(RubiksState &s, RubiksAction a) const
{
	c.ApplyAction(s.corner, a);
	e.ApplyAction(s.edge, a);
	if (pruneSuccessors)
		history.push_back(a);
}

void RubiksCube::UndoAction(RubiksState &s, RubiksAction a) const
{
	if (pruneSuccessors && history.size() > 0)
	{
		assert(history.back() == a);
		history.pop_back();
	}
	InvertAction(a);
	c.ApplyAction(s.corner, a);
	e.ApplyAction(s.edge, a);

}

//void ApplyMove(RubiksState &s, RubikCornerMove *a)
//{
//	
//}
//
//void UndoMove(RubiksState &s, RubikCornerMove *a)
//{
//	
//}

void RubiksCube::GetNextState(const RubiksState &s1, RubiksAction a, RubiksState &s2) const
{
	s2 = s1;
	ApplyAction(s2, a);
}

bool RubiksCube::InvertAction(RubiksAction &a) const
{
	if (2 == a%3)
		return true;
	if (1 == a%3)
	{
		a -= 1;
		return true;
	}
	a += 1;
	return true;
}

/** Heuristic value between two arbitrary nodes. **/
double RubiksCube::HCost(const RubiksState &node1, const RubiksState &node2)
{
	double val = 0;
	if (cornerPDB.Size() > 0)
	{
		val = cornerPDB.Get(c.GetStateHash(node1.corner));
	}
	if (edge7PDB.Size() > 0)
	{
		uint64_t index = e7.GetStateHash(node1.edge7);
		val = max(val, edge7PDB.Get(index));
	}
//	if (edgePDB.Size() > 0)
//	{
//		uint64_t index = e.GetStateHash(node1.edge);
//		if (index < edgePDB.Size())
//			val = max(val, edgePDB.Get(index));
//	}
	return val;
}

/** Heuristic value between node and the stored goal. Asserts that the
 goal is stored **/
double RubiksCube::HCost(const RubiksState &node)
{
	return 0;
}

bool RubiksCube::GoalTest(const RubiksState &node, const RubiksState &goal)
{
	return (node.corner.state == goal.corner.state &&
			node.edge.state == goal.edge.state);
}

/** Goal Test if the goal is stored **/
bool RubiksCube::GoalTest(const RubiksState &node)
{
	assert(false);
	return false;
}

uint64_t RubiksCube::GetStateHash(const RubiksState &node) const
{
	uint64_t hash = c.GetStateHash(node.corner);
	hash *= e.getMaxSinglePlayerRank();
	hash += e.GetStateHash(node.edge);
	return hash;
}

void RubiksCube::GetStateFromHash(uint64_t hash, RubiksState &node) const
{
	e.GetStateFromHash(hash%e.getMaxSinglePlayerRank(), node.edge);
	c.GetStateFromHash(hash/e.getMaxSinglePlayerRank(), node.corner);
}

void RubiksCube::OpenGLDraw() const
{
}

void RubiksCube::OpenGLDraw(const RubiksState&s) const
{
	e.OpenGLDraw(s.edge);
	c.OpenGLDraw(s.corner);
}

/** Draw the transition at some percentage 0...1 between two states */
void RubiksCube::OpenGLDraw(const RubiksState&, const RubiksState&, float) const
{
	
}

void RubiksCube::OpenGLDraw(const RubiksState&, const RubiksAction&) const
{
	
}
