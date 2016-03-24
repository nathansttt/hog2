//
//  RubiksCorner.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/11/13.
//  Copyright (c) 2013 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__RubiksCorner__
#define __hog2_glut__RubiksCorner__

#include <iostream>
#include <stdint.h>
#include <vector>
#include "PDBHeuristic.h"

class RubiksCornerState
{
public:
	RubiksCornerState()
	{
		Reset();
	}
	void Reset()
	{
		state = 0;
		for (int x = 0; x < 8; x++)
		{
			SetCubeInLoc(x, x);
		}
	}
	int GetCubeInLoc(int whichLoc) const
	{
		return (state>>(16+4*whichLoc))&0xF;
	}
	void SetCubeInLoc(int whichLoc, int cube)
	{
		uint64_t blank = 0xF;
		uint64_t value = cube&0xF;
		state = state&(~(blank<<(16+4*whichLoc)));
		state |= (value<<(16+4*whichLoc));
	}
	uint64_t GetCubeOrientation(int whichLoc) const
	{
		return (state>>(2*whichLoc))&0x3;
	}
	void SetCubeOrientation(int whichLoc, int orient) // orientation is the offset of the 0(low) side
	{
		if (whichLoc > 7)
			return;
		uint64_t blank = 0x3;
		uint64_t value = orient&0x3;
		state = state&(~(blank<<(2*whichLoc)));
		state |= (value<<(2*whichLoc));
	}
	int GetFaceInLoc(int whichLoc) const // loc 0...24
	{
		int cube = GetCubeInLoc(whichLoc/3);
		int rot = (int)GetCubeOrientation(cube);
		return cube*3+(3+(whichLoc%3)-rot)%3;
	}
	void Rotate(uint64_t a, uint64_t b, uint64_t c, uint64_t d);
	void Swap(uint64_t a, uint64_t b, uint64_t c, uint64_t d);
	// 48 bits
	// 16 bits of rotations
	// 32 bits of pieces
	uint64_t state;
};

static bool operator==(const RubiksCornerState &l1, const RubiksCornerState &l2)
{
	return l1.state == l2.state;
}

static std::ostream &operator<<(std::ostream &out, const RubiksCornerState &s)
{
	for (int x = 0; x < 8; x++)
		out << s.GetCubeInLoc(x) << " [" << s.GetCubeOrientation(x) << "] ";
	return out;
}


typedef int RubiksCornersAction;

class RubikCornerMove {
public:
	RubiksCornersAction act;
	RubikCornerMove *next;
	int length() { if (next == 0) return 1; return 1+next->length(); }
};

class RubiksCorner
{
public:
	RubiksCorner()
	{
		for (int x = 0; x < 18; x++)
		{
			moves[x].act = x;
			if (x != 17)
				moves[x].next = &moves[x+1];
		} moves[17].next = 0;
	}
	
	~RubiksCorner() {}
	virtual void GetSuccessors(const RubiksCornerState &nodeID, std::vector<RubiksCornerState> &neighbors) const;
	virtual void GetActions(const RubiksCornerState &nodeID, std::vector<RubiksCornersAction> &actions) const;
	virtual RubiksCornersAction GetAction(const RubiksCornerState &s1, const RubiksCornerState &s2) const;
	virtual void ApplyAction(RubiksCornerState &s, RubiksCornersAction a) const;
	
	void ApplyMove(RubiksCornerState &s, RubikCornerMove *a);
	void UndoMove(RubiksCornerState &s, RubikCornerMove *a);
	void unrankPlayer(uint64_t d, RubiksCornerState & s, int who)
	{
		GetStateFromHash(d, s);
	}
	//
	int64_t getMaxSinglePlayerRank();
	int64_t getMaxSinglePlayerRank2();
	int64_t getMaxSinglePlayerRank2(int64_t firstIndex);
	
	RubikCornerMove *getMoves(RubiksCornerState &) { return &moves[0]; }
	void freeMove(RubikCornerMove *m) {}
	
	int64_t rankPlayer(RubiksCornerState &s, int who)
	{ return GetStateHash(s); }
	void rankPlayerFirstTwo(const RubiksCornerState &s, int who, int64_t &rank);
	void rankPlayerRemaining(const RubiksCornerState &s, int who, int64_t &rank);
	void rankPlayer(RubiksCornerState &s, int who, int64_t &index1, int64_t &index2)
	{ rankPlayerFirstTwo(s, 0, index1); rankPlayerRemaining(s, 0, index2); }
	
	
	virtual void GetNextState(const RubiksCornerState &, RubiksCornersAction , RubiksCornerState &) const;
	
	virtual bool InvertAction(RubiksCornersAction &a) const;
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const RubiksCornerState &node1, const RubiksCornerState &node2) const { return 0; }
	
	/** Heuristic value between node and the stored goal. Asserts that the
	 goal is stored **/
	virtual double HCost(const RubiksCornerState &node) const
	{ return 0; }
	
	virtual double GCost(const RubiksCornerState &node1, const RubiksCornerState &node2) const { return 1.0; }
	virtual double GCost(const RubiksCornerState &node, const RubiksCornersAction &act) const { return 1.0; }
	virtual bool GoalTest(const RubiksCornerState &node, const RubiksCornerState &goal) const { return GoalTest(node); }
	
	/** Goal Test if the goal is stored **/
	virtual bool GoalTest(const RubiksCornerState &node) const;
	
	static uint64_t GetStateHash(const RubiksCornerState &node);
	static void GetStateFromHash(uint64_t hash, RubiksCornerState &node);
	virtual uint64_t GetActionHash(RubiksCornersAction act) const { return 0; }
	
	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const RubiksCornerState&) const;
	/** Draw the transition at some percentage 0...1 between two states */
	virtual void OpenGLDraw(const RubiksCornerState&, const RubiksCornerState&, float) const;
	virtual void OpenGLDraw(const RubiksCornerState&, const RubiksCornersAction&) const;
	void OpenGLDrawCube(const RubiksCornerState &s, int cube) const;
private:
	void SetFaceColor(int face, const RubiksCornerState&) const;
	//	void SetFaceColor(int face, const RubiksCornerState&) const;
	static uint64_t MRRank(int n, uint64_t perm, uint64_t dual);
	static void MRUnrank2(int n, uint64_t r, uint64_t &perm);
	RubikCornerMove moves[18];
};

class RubikCornerPDB : public PDBHeuristic<RubiksCornerState, RubiksCornersAction, RubiksCorner, RubiksCornerState, 4> {
public:
	RubikCornerPDB(RubiksCorner *e, const RubiksCornerState &s, std::vector<int> &distinctCorners);
	static uint64_t GetStateSpaceSize();
	static uint64_t GetStateHash(const RubiksCornerState &s);
	static void GetStateFromHash(RubiksCornerState &s, uint64_t hash);
	uint64_t GetPDBSize() const;
	uint64_t GetPDBHash(const RubiksCornerState &s, int threadID = 0) const;
	virtual uint64_t GetAbstractHash(const RubiksCornerState &s, int threadID = 0) const { return GetPDBHash(s); }
	void GetStateFromPDBHash(uint64_t hash, RubiksCornerState &s, int threadID = 0) const;
	RubiksCornerState GetStateFromAbstractState(RubiksCornerState &s) const { return s; }

	virtual bool Load(const char *prefix);
	virtual void Save(const char *prefix);
	virtual bool Load(FILE *f);
	virtual void Save(FILE *f);
	virtual std::string GetFileName(const char *prefix);
private:
	uint64_t Factorial(int val) const;
	uint64_t FactorialUpperK(int n, int k) const;
	std::vector<int> corners;
	size_t puzzleSize;
	uint64_t pdbSize;
	
	// cache for computing ranking/unranking
	mutable std::vector<std::vector<int> > dual;
	mutable std::vector<std::vector<int> > locs;
};


#define Rubik RubiksCorner
#define RubikState RubiksCornerState
#define RubikMove RubikCornerMove

#endif /* defined(__hog2_glut__RubiksCorner__) */

	
