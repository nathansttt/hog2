/*
 *  MNAgentPuzzle.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/25/10.
 *  Copyright 2010 University of Denver. All rights reserved.
 *
 */

#include "MNAgentPuzzle.h"
#include <assert.h>

void MNAgentEnvironment::GetSuccessors(const MNAgentPuzzleState &nodeID, std::vector<MNAgentPuzzleState> &neighbors) const
{
	neighbors.resize(0);
	std::vector<MNAgentPuzzleAction> actions;
	GetActions(nodeID, actions);
	for (unsigned int x = 0; x < actions.size(); x++)
	{
		MNAgentPuzzleState s = nodeID;
		ApplyAction(s, actions[x]);
		neighbors.push_back(s);
	}
}

void MNAgentEnvironment::GetActions(const MNAgentPuzzleState &s, std::vector<MNAgentPuzzleAction> &actions) const
{
	MNAgentPuzzleAction c;
	MNAgentPuzzleState tmp = s;
	std::vector<bool> moved(s.width*s.height);
	actions.resize(0);
	FindLegalMoves(tmp, c, actions, 0, 0, moved);
}

void MNAgentEnvironment::FindLegalMoves(MNAgentPuzzleState &s,
										MNAgentPuzzleAction &curr,
										std::vector<MNAgentPuzzleAction> &actions,
										int realMoves,
										int depth,
										std::vector<bool> &moved) const
{
	if (depth == s.blanks.size())
	{
		if (realMoves > 0) // don't have every blank stay
			actions.push_back(curr);
		return;
	}
	
	
	curr.SetBlankAction(depth, kBlankStay);
	FindLegalMoves(s, curr, actions, realMoves, depth+1, moved);

	if (((s.blanks[depth]%s.width) != 0) && (s.agents[s.blanks[depth]-1] != 0) && !moved[s.agents[s.blanks[depth]-1]])
	{
		curr.SetBlankAction(depth, kBlankLeft);
		
		moved[s.agents[s.blanks[depth]-1]] = true;
		
		s.agents[s.blanks[depth]] = s.agents[s.blanks[depth]-1];
		s.blanks[depth]--;
		s.agents[s.blanks[depth]] = 0;
		
		FindLegalMoves(s, curr, actions, realMoves+1, depth+1, moved);

		s.agents[s.blanks[depth]] = s.agents[s.blanks[depth]+1];
		s.blanks[depth]++;
		s.agents[s.blanks[depth]] = 0;
		
		moved[s.agents[s.blanks[depth]-1]] = false;
	}
	if (((s.blanks[depth]%s.width) != s.width-1) && (s.agents[s.blanks[depth]+1] != 0) && !moved[s.agents[s.blanks[depth]+1]])
	{
		curr.SetBlankAction(depth, kBlankRight);
		
		moved[s.agents[s.blanks[depth]+1]] = true;
		
		s.agents[s.blanks[depth]] = s.agents[s.blanks[depth]+1];
		s.blanks[depth]++;
		s.agents[s.blanks[depth]] = 0;
		
		FindLegalMoves(s, curr, actions, realMoves+1, depth+1, moved);
		
		s.agents[s.blanks[depth]] = s.agents[s.blanks[depth]-1];
		s.blanks[depth]--;
		s.agents[s.blanks[depth]] = 0;
		
		moved[s.agents[s.blanks[depth]+1]] = false;
	}
	if (((s.blanks[depth]/s.width) != 0) && (s.agents[s.blanks[depth]-s.width] != 0) && !moved[s.agents[s.blanks[depth]-s.width]])
	{
		curr.SetBlankAction(depth, kBlankUp);
		
		moved[s.agents[s.blanks[depth]-s.width]] = true;
		
		s.agents[s.blanks[depth]] = s.agents[s.blanks[depth]-s.width];
		s.blanks[depth]-=s.width;
		s.agents[s.blanks[depth]] = 0;

		FindLegalMoves(s, curr, actions, realMoves+1, depth+1, moved);

		s.agents[s.blanks[depth]] = s.agents[s.blanks[depth]+s.width];
		s.blanks[depth]+=s.width;
		s.agents[s.blanks[depth]] = 0;
		
		moved[s.agents[s.blanks[depth]-s.width]] = false;
	}
	if (((s.blanks[depth]/s.width) != s.height-1) && (s.agents[s.blanks[depth]+s.width] != 0) && !moved[s.agents[s.blanks[depth]+s.width]])
	{
		curr.SetBlankAction(depth, kBlankDown);

		moved[s.agents[s.blanks[depth]+s.width]] = true;
		
		s.agents[s.blanks[depth]] = s.agents[s.blanks[depth]+s.width];
		s.blanks[depth]+=s.width;
		s.agents[s.blanks[depth]] = 0;

		FindLegalMoves(s, curr, actions, realMoves+1, depth+1, moved);
		
		s.agents[s.blanks[depth]] = s.agents[s.blanks[depth]-s.width];
		s.blanks[depth]-=s.width;
		s.agents[s.blanks[depth]] = 0;
		
		moved[s.agents[s.blanks[depth]+s.width]] = false;
	}
}


MNAgentPuzzleAction MNAgentEnvironment::GetAction(const MNAgentPuzzleState &s1, const MNAgentPuzzleState &s2) const
{
	assert(false);
}

void MNAgentEnvironment::ApplyAction(MNAgentPuzzleState &s, MNAgentPuzzleAction a) const
{
	for (int depth = 0; depth < s.blanks.size(); depth++)
	{
		if (a.GetBlankAction(depth) == kBlankLeft)
		{
			s.agents[s.blanks[depth]] = s.agents[s.blanks[depth]-1];
			s.blanks[depth]--;
			s.agents[s.blanks[depth]] = 0;
		}
		if (a.GetBlankAction(depth) == kBlankRight)
		{
			s.agents[s.blanks[depth]] = s.agents[s.blanks[depth]+1];
			s.blanks[depth]++;
			s.agents[s.blanks[depth]] = 0;
		}
		if (a.GetBlankAction(depth) == kBlankUp)
		{
			s.agents[s.blanks[depth]] = s.agents[s.blanks[depth]-s.width];
			s.blanks[depth]-=s.width;
			s.agents[s.blanks[depth]] = 0;
		}
		if (a.GetBlankAction(depth) == kBlankDown)
		{
			s.agents[s.blanks[depth]] = s.agents[s.blanks[depth]+s.width];
			s.blanks[depth]+=s.width;
			s.agents[s.blanks[depth]] = 0;
		}
	}
}

void MNAgentEnvironment::GetNextState(MNAgentPuzzleState &s1, MNAgentPuzzleAction a, MNAgentPuzzleState &s2) const
{
	s2 = s1;
	ApplyAction(s2, a);
}

bool MNAgentEnvironment::InvertAction(MNAgentPuzzleAction &a) const
{
	assert(false);
}

/** Heuristic value between two arbitrary nodes. **/
double MNAgentEnvironment::HCost(const MNAgentPuzzleState &node1, const MNAgentPuzzleState &node2)
{
	// for now we don't need this
	return 1;
}

double MNAgentEnvironment::GCost(const MNAgentPuzzleState &node1, const MNAgentPuzzleState &node2)
{ return 1; }
double MNAgentEnvironment::GCost(const MNAgentPuzzleState &node, const MNAgentPuzzleAction &act)
{ return 1; }

bool MNAgentEnvironment::GoalTest(const MNAgentPuzzleState &node, const MNAgentPuzzleState &goal)
{
	for (unsigned int x = 0; x < node.agents.size(); x++)
		if (node.agents[x] != goal.agents[x])
			return false;
	return true;	
}


uint64_t MNAgentEnvironment::GetStateHash(const MNAgentPuzzleState &node) const
{
	int base = node.width*node.height;
	uint64_t hash = 0;
	for (unsigned int x = 0; x < node.agents.size(); x++)
	{
		uint64_t next = node.agents[x];
		if ((next >= 1) && (next <= 1+domainAbstractionSize)) next = 1;
		hash = hash*base+next;
	}

	return hash;
}

uint64_t MNAgentEnvironment::GetActionHash(MNAgentPuzzleAction act) const
{
	return act.theAct;
}

void MNAgentEnvironment::OpenGLDraw() const
{
}

void MNAgentEnvironment::OpenGLDraw(const MNAgentPuzzleState&) const
{
}

/** Draw the transition at some percentage 0...1 between two MNAgentPuzzleStates */
void MNAgentEnvironment::OpenGLDraw(const MNAgentPuzzleState&, const MNAgentPuzzleState&, float) const
{
}

void MNAgentEnvironment::OpenGLDraw(const MNAgentPuzzleState&, const MNAgentPuzzleAction&) const
{
}
