//
//  RC.h (Alternate Rubik's Cube implementation)
//
//  Created by Nathan Sturtevant on 7/9/21.
//

#ifndef __RC__
#define __RC__

#include <iostream>
#include <stdint.h>
#include <unordered_map>
#include <vector>
#include "SearchEnvironment.h"
#include "PDBHeuristic.h"

class RCState
{
public:
	RCState()
	{
		Reset();
	}
	void Reset()
	{
		// Set all values to default
	}
	// Put data structures here
};

typedef int RCAction;

// Can't write these until data structures are defined
static bool operator==(const RCState &l1, const RCState &l2)
{
	return false;
	//return (l1.corner == l2.corner) && (l1.edge == l2.edge);
}
//
//static bool operator!=(const RCState &l1, const RCState &l2)
//{
//	return !(l1 == l2);
//}
//
//static std::ostream &operator<<(std::ostream &out, const RCState &s)
//{
//	out << "{" << s.edge << ", " << s.corner << "}";
//	return out;
//}

class RC : public SearchEnvironment<RCState, RCAction>
{
public:
	std::string GetName() { return "RC"; }
//	void SetPruneSuccessors(bool val) { pruneSuccessors = val; history.resize(0); }
	virtual void GetSuccessors(const RCState &nodeID, std::vector<RCState> &neighbors) const;
	virtual void GetActions(const RCState &nodeID, std::vector<RCAction> &actions) const;
	virtual void GetPrunedActions(const RCState &nodeID, RCAction lastAction, std::vector<RCAction> &actions) const;
	virtual RCAction GetAction(const RCState &s1, const RCState &s2) const;
	virtual void ApplyAction(RCState &s, RCAction a) const;
	virtual void UndoAction(RCState &s, RCAction a) const;

	virtual void GetNextState(const RCState &, RCAction , RCState &) const;
	
	virtual bool InvertAction(RCAction &a) const;
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const RCState &node1, const RCState &node2) const;
	virtual double HCost(const RCState &node1, const RCState &node2, double parentHCost) const;
	int Edge12PDBDist(const RCState &s);
	
	/** Heuristic value between node and the stored goal. Asserts that the
	 goal is stored **/
	virtual double HCost(const RCState &node) const;
	
	virtual double GCost(const RCState &node1, const RCState &node2) const { return 1.0; }
	virtual double GCost(const RCState &node, const RCAction &act) const { return 1.0; }
	virtual bool GoalTest(const RCState &node, const RCState &goal) const;
	
	/** Goal Test if the goal is stored **/
	virtual bool GoalTest(const RCState &node) const;
	
	virtual uint64_t GetStateHash(const RCState &node) const;

	virtual uint64_t GetActionHash(RCAction act) const { return act; }
	virtual void GetStateFromHash(uint64_t hash, RCState &node) const;
	
	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const RCState&) const;
	virtual void OpenGLDrawCorners(const RCState&) const;
	virtual void OpenGLDrawEdges(const RCState&) const;
	virtual void OpenGLDrawEdgeDual(const RCState&) const;
	virtual void OpenGLDrawCenters() const;
	virtual void OpenGLDrawCubeBackground() const;
	/** Draw the transition at some percentage 0...1 between two states */
	virtual void OpenGLDraw(const RCState&, const RCState&, float) const;
	virtual void OpenGLDraw(const RCState&, const RCAction&) const;
	
	void OpenGLDrawCube(int cube) const;
	void SetFaceColor(int face) const;
	mutable std::vector<RCAction> history;
	
	
	bool pruneSuccessors;
};

#endif /* defined(__hog2_glut__RubiksCube__) */
