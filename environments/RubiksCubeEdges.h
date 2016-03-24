//
//  RubikEdge.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/4/13.
//  Copyright (c) 2013 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__RubikEdge__
#define __hog2_glut__RubikEdge__

#include <iostream>
#include <stdint.h>
#include <vector>
#include "SearchEnvironment.h"
#include "PDBHeuristic.h"

class RubikEdgeState
{
public:
	RubikEdgeState()
	{
		Reset();
	}
	void Reset()
	{
		state = 0;
		for (int x = 0; x < 12; x++)
			SetCubeInLoc(x, x);
	}
	void GetDual(RubikEdgeState &s) const;
	int GetCubeInLoc(int whichLoc) const
	{
		return (state>>(12+4*whichLoc))&0xF;
	}
	void SetCubeInLoc(int whichLoc, int cube)
	{
		if (whichLoc >= 12)
			return;
		uint64_t blank = 0xF;
		uint64_t value = cube&0xF;
		state = state&(~(blank<<(12+4*whichLoc)));
		state |= (value<<(12+4*whichLoc));
	}
	bool GetCubeOrientation(int whichLoc) const
	{
		return state&(0x1<<whichLoc);
	}
	void SetCubeOrientation(int whichLoc, bool flip)
	{
		if (whichLoc >= 12)
			return;
		uint64_t blank = 0x1;
		if (flip)
			state |= (0x1<<whichLoc);
		else
			state = state&(~(blank<<whichLoc));
	}
	void FlipCubeOrientation(int whichLoc)
	{
		if (whichLoc >= 12)
			return;
		//		printf("Was: 0x%X [flip %d] -- ", state, whichLoc);
		state = state^(0x1<<whichLoc);
		//		printf("Now: 0x%X \n", state);
	}
	// 60 bits
	// 12 bits of flips
	// 48 bits of pieces
	uint64_t state;
};

static bool operator==(const RubikEdgeState &l1, const RubikEdgeState &l2)
{
	return l1.state == l2.state;
}

static std::ostream& operator <<(std::ostream & out, const RubikEdgeState &s)
{
	for (int x = 0; x < 12; x++)
	{
		out << s.GetCubeInLoc(x) << " [" << s.GetCubeOrientation(x) << "] ";
		//out << s.GetCubeInLoc(x) << "-" << (s.GetCubeOrientation(s.GetCubeInLoc(x))?1:0) << " ";
	}
	return out;
}


typedef int RubikEdgeAction;

class RubikEdgeMove {
public:
	RubikEdgeAction act;
	RubikEdgeMove *next;
	int length() { if (next == 0) return 1; return 1+next->length(); }
};

class RubikEdge  : public SearchEnvironment<RubikEdgeState, RubikEdgeAction>
{
public:
	RubikEdge()
	{
		piecesToRank = 12;
		for (int x = 0; x < 18; x++)
		{
			moves[x].act = x;
			if (x != 17)
				moves[x].next = &moves[x+1];
		} moves[17].next = 0;
	}
	~RubikEdge() {}
	virtual void GetSuccessors(const RubikEdgeState &nodeID, std::vector<RubikEdgeState> &neighbors) const;
	virtual void GetActions(const RubikEdgeState &nodeID, std::vector<RubikEdgeAction> &actions) const;
	virtual RubikEdgeAction GetAction(const RubikEdgeState &s1, const RubikEdgeState &s2) const;
	virtual void ApplyAction(RubikEdgeState &s, RubikEdgeAction a) const;
	virtual void UndoAction(RubikEdgeState &s, RubikEdgeAction a) const;

	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const RubikEdgeState &node1, const RubikEdgeState &node2) const { return 1; }
	virtual double GCost(const RubikEdgeState &node1, const RubikEdgeState &node2) const { return 1; }
	virtual double GCost(const RubikEdgeState &node, const RubikEdgeAction &act) const { return 1; }
	virtual bool GoalTest(const RubikEdgeState &node, const RubikEdgeState &goal) const { return (node.state == goal.state); }

	void ApplyMove(RubikEdgeState &s, RubikEdgeMove *a);
	void UndoMove(RubikEdgeState &s, RubikEdgeMove *a);
	void unrankPlayer(uint64_t d, RubikEdgeState & s, int who)
	{
		GetStateFromHash(d, s);
	}
	//
	int64_t getMaxSinglePlayerRank() const;
	int64_t getMaxSinglePlayerRank2();
	int64_t getMaxSinglePlayerRank2(int64_t firstIndex);
	
	RubikEdgeMove *getMoves(RubikEdgeState &) { return &moves[0]; }
	void freeMove(RubikEdgeMove *m) {}
	virtual void GetNextState(const RubikEdgeState &, RubikEdgeAction , RubikEdgeState &) const;
	
	virtual bool InvertAction(RubikEdgeAction &a) const;
	
	int64_t rankPlayer(RubikEdgeState &s, int who)
	{ return GetStateHash(s); }
	void rankPlayerFirstTwo(const RubikEdgeState &s, int who, int64_t &rank);
	void rankPlayerRemaining(const RubikEdgeState &s, int who, int64_t &rank);
	void rankPlayer(const RubikEdgeState &s, int who, int64_t &index1, int64_t &index2)
	{ rankPlayerFirstTwo(s, 0, index1); rankPlayerRemaining(s, 0, index2); }
	
	virtual uint64_t GetStateHash(const RubikEdgeState &node) const;
	virtual uint64_t GetActionHash(RubikEdgeAction act) const { return 0; }
	virtual void GetStateFromHash(uint64_t hash, RubikEdgeState &node) const;
	
	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const RubikEdgeState&) const;
	/** Draw the transition at some percentage 0...1 between two states */
	virtual void OpenGLDraw(const RubikEdgeState&, const RubikEdgeState&, float) const;
	virtual void OpenGLDraw(const RubikEdgeState&, const RubikEdgeAction&) const;
	void OpenGLDrawCube(const RubikEdgeState &s, int cube) const;
	static void MRUnrank(int n, uint64_t r, uint64_t &perm);
	static void MRUnrank2(int n, uint64_t r, uint64_t &perm);
	static uint64_t MRRank(int n, uint64_t perm, uint64_t dual);
	static uint64_t MRRank2(int n, uint64_t perm, uint64_t dual);

private:
	int piecesToRank;
	
	void SetCubeColor(int which, bool face, const RubikEdgeState&) const;
	RubikEdgeMove moves[18];
};

class RubikEdgePDB : public PDBHeuristic<RubikEdgeState, RubikEdgeAction, RubikEdge, RubikEdgeState, 4> {
public:
	RubikEdgePDB(RubikEdge *e, const RubikEdgeState &s, std::vector<int> &distinctEdges);
	static uint64_t GetStateSpaceSize();
	static uint64_t GetStateHash(const RubikEdgeState &s);
	static void GetStateFromHash(RubikEdgeState &s, uint64_t hash);
	uint64_t GetPDBSize() const;
	uint64_t GetPDBHash(const RubikEdgeState &s, int threadID = 0) const;
	virtual uint64_t GetAbstractHash(const RubikEdgeState &s, int threadID = 0) const { return GetPDBHash(s); }
	void GetStateFromPDBHash(uint64_t hash, RubikEdgeState &s, int threadID = 0) const;
	RubikEdgeState GetStateFromAbstractState(RubikEdgeState &s) const { return s; }

	bool Load(const char *prefix);
	void Save(const char *prefix);
	bool Load(FILE *f);
	void Save(FILE *f);
	std::string GetFileName(const char *prefix);
private:
	static uint64_t Factorial(int val);
	static uint64_t FactorialUpperK(int n, int k);
	std::vector<int> edges;
	size_t puzzleSize;
	uint64_t pdbSize;
	
	// cache for computing ranking/unranking
	mutable std::vector<std::vector<int> > dual;
	mutable std::vector<std::vector<int> > locs;
};


#endif /* defined(__hog2_glut__RubikEdge__) */
