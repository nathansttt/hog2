/*
 *  MNAgentPuzzle.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/25/10.
 *  Copyright 2010 University of Denver. All rights reserved.
 *
 */

#include "SearchEnvironment.h"
#include <vector>

#ifndef MNAGENTPUZZLE_H
#define MNAGENTPUZZLE_H

enum {
	kBlankStay = 0,
	kBlankLeft = 1,
	kBlankRight = 2,
	kBlankUp = 3,
	kBlankDown = 4
};

class MNAgentPuzzleState {
public:
	MNAgentPuzzleState() { width = height = -1; }
	MNAgentPuzzleState(unsigned int _width, unsigned int _height, unsigned int numBlanks)
	:width(_width), height(_height)
	{
//		agents.resize(width*height);
		for (unsigned int x = 0; x < width*height-numBlanks; x++)
		{
			agents.push_back(x+1);
		}
		for (unsigned int x = 0; x < numBlanks; x++)
		{
			agents.push_back(0);
			blanks.push_back(width*height-numBlanks+x);
		}
	}
	unsigned int width, height;
	std::vector<int> agents;
	std::vector<int> blanks;
};

static std::ostream& operator <<(std::ostream & out, const MNAgentPuzzleState &loc)
{
	out << "(" << loc.width << "x" << loc.height << ")";
	for (unsigned int x = 0; x < loc.agents.size(); x++)
		out << loc.agents[x] << " ";
	out << " blanks: ";
	for (unsigned int x = 0; x < loc.blanks.size(); x++)
		out << loc.blanks[x] << " ";
	return out;
}

static bool operator==(const MNAgentPuzzleState &l1, const MNAgentPuzzleState &l2) {
	for (unsigned int x = 0; x < l1.agents.size(); x++)
		if (l1.agents[x] != l2.agents[x])
			return false;
	return true;
}


class MNAgentPuzzleAction {
public:
	int GetBlankAction(int which)
	{
		return (theAct>>(which*3))&0x7;
	}
	void SetBlankAction(int which, int act)
	{
		theAct = (theAct&(~ (uint64_t)(0x7<<(which*3)))) | (uint64_t)((act&0x7) << which*3);
	}
//private:
	uint64_t theAct;
};

class MNAgentEnvironment : public SearchEnvironment<MNAgentPuzzleState, MNAgentPuzzleAction>
{
public:
	MNAgentEnvironment() { domainAbstractionSize = -1; }
	virtual void GetSuccessors(const MNAgentPuzzleState &nodeID, std::vector<MNAgentPuzzleState> &neighbors) const;
	virtual void GetActions(const MNAgentPuzzleState &nodeID, std::vector<MNAgentPuzzleAction> &actions) const;
	//virtual int GetNumSuccessors(const MNAgentPuzzleState &stateID) const;
	virtual MNAgentPuzzleAction GetAction(const MNAgentPuzzleState &s1, const MNAgentPuzzleState &s2) const;
	virtual void ApplyAction(MNAgentPuzzleState &s, MNAgentPuzzleAction a) const;
	
	virtual void GetNextState(MNAgentPuzzleState &, MNAgentPuzzleAction , MNAgentPuzzleState &) const;
	
	virtual bool InvertAction(MNAgentPuzzleAction &a) const;	
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const MNAgentPuzzleState &node1, const MNAgentPuzzleState &node2);
	
	/** Heuristic value between node and the stored goal. Asserts that the
	 goal is stored **/
	virtual double HCost(const MNAgentPuzzleState &node)
	{ assert(bValidSearchGoal); return HCost(node, searchGoal); }
	
	virtual double GCost(const MNAgentPuzzleState &node1, const MNAgentPuzzleState &node2);
	virtual double GCost(const MNAgentPuzzleState &node, const MNAgentPuzzleAction &act);
	virtual bool GoalTest(const MNAgentPuzzleState &node, const MNAgentPuzzleState &goal);
	
	/** Goal Test if the goal is stored **/
	virtual bool GoalTest(const MNAgentPuzzleState &node)
	{ return bValidSearchGoal&&(node == searchGoal); }
	
	void SetDomainAbstractionSize(int val) { domainAbstractionSize = val; }
	virtual uint64_t GetStateHash(const MNAgentPuzzleState &node) const;
	virtual uint64_t GetActionHash(MNAgentPuzzleAction act) const;
	
	//virtual double GetPathLength(std::vector<MNAgentPuzzleState> &neighbors);
		
	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const MNAgentPuzzleState&) const;
	/** Draw the transition at some percentage 0...1 between two states */
	virtual void OpenGLDraw(const MNAgentPuzzleState&, const MNAgentPuzzleState&, float) const;
	virtual void OpenGLDraw(const MNAgentPuzzleState&, const MNAgentPuzzleAction&) const;
private:
	void FindLegalMoves(MNAgentPuzzleState &s,
						MNAgentPuzzleAction &curr,
						std::vector<MNAgentPuzzleAction> &actions,
						int realMoves,
						int depth,
						std::vector<bool> &moved) const;
	int domainAbstractionSize;
};

#endif
