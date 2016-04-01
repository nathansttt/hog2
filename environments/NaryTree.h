/*
 *  NaryTree.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/20/10.
 *  Copyright 2010 University of Denver. All rights reserved.
 *
 */


#include "SearchEnvironment.h"
#include <vector>

#ifndef NARYTREE_H
#define NARTTREE_H

typedef uint64_t NaryState;
typedef int NaryAction;

class NaryTree : public SearchEnvironment<NaryState, NaryAction>
{
public:
	NaryTree(int branchingFactor, int depth);
	int GetBranchingFactor() { return b; }
	NaryState GetLastNode() { return totalNodesAtDepth.back()-1; }
	NaryState GetParent(NaryState s) const;
	virtual void GetSuccessors(const NaryState &nodeID, std::vector<NaryState> &neighbors) const;
	virtual void GetActions(const NaryState &nodeID, std::vector<NaryAction> &actions) const;
	//virtual int GetNumSuccessors(const NaryState &stateID) const;
	virtual NaryAction GetAction(const NaryState &s1, const NaryState &s2) const;
	virtual void ApplyAction(NaryState &s, NaryAction a) const;
	
	virtual void GetNextState(const NaryState &, NaryAction , NaryState &) const;
	
	virtual bool InvertAction(NaryAction &a) const;
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const NaryState &node1, const NaryState &node2) const;
	
	/** Heuristic value between node and the stored goal. Asserts that the
	 goal is stored **/
	virtual double HCost(const NaryState &node) const
	{ assert(bValidSearchGoal); return HCost(node, searchGoal); }
	
	virtual double GCost(const NaryState &node1, const NaryState &node2) const;
	virtual double GCost(const NaryState &node, const NaryAction &act) const;
	virtual bool GoalTest(const NaryState &node, const NaryState &goal) const;
	
	/** Goal Test if the goal is stored **/
	virtual bool GoalTest(const NaryState &node) const
	{ return bValidSearchGoal&&(node == searchGoal); }
	
	virtual uint64_t GetStateHash(const NaryState &node) const;
	virtual uint64_t GetActionHash(NaryAction act) const;
	
	//virtual double GetPathLength(std::vector<NaryState> &neighbors);
	void SetWidthScale(double v) { scaleWidth = v;}
	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const NaryState&) const;
	/** Draw the transition at some percentage 0...1 between two states */
	virtual void OpenGLDraw(const NaryState&, const NaryState&, float) const;
	virtual void OpenGLDraw(const NaryState&, const NaryAction&) const;
	//virtual void GLLabelState(const state&, const char *) const {} // draw label over state
	void GLDrawLine(const NaryState &x, const NaryState &y) const;

private:
	int GetDepth(const NaryState s) const;
	uint64_t GetOffset(const NaryState s) const;
	void GetLocation(const NaryState &s, double &x, double &y) const;
	int b, d;
	std::vector<uint64_t> nodesAtDepth;
	std::vector<uint64_t> totalNodesAtDepth;
	double scaleWidth;
};

#endif
