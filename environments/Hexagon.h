//
//  Hexagon.h
//  Hexagon
//
//  Created by Nathan Sturtevant on 11/13/21.
//  Copyright © 2021 MovingAI. All rights reserved.
//

#ifndef Hexagon_h
#define Hexagon_h

#include <cstdio>
#include <vector>
#include "SearchEnvironment.h"
#include "FPUtil.h"
#include "NBitArray.h"

class HexagonAction
{
public:
	int piece : 4; // 10 pieces
	int location : 6; // 54 possible locations
	int rotation : 3; // 3 rotations x 2 flips = 6
};

class HexagonState
{
public:
	HexagonState()
	:state(66) {
	}
	NBitArray<4> state;
};

static bool operator==(const HexagonState &l1, const HexagonState &l2)
{
	return l1.state == l2.state;
}

static std::ostream &operator<<(std::ostream &out, const HexagonAction &a)
{
	return out;
}

class Hexagon : public SearchEnvironment<HexagonState, HexagonAction>
{
public:
	Hexagon();
	~Hexagon();
	void LoadPuzzle(const char *, HexagonState &s);
	void LoadSolution(const char *, HexagonState &s);
	void GetSuccessors(const HexagonState &nodeID, std::vector<HexagonState> &neighbors) const;
	void GetActions(const HexagonState &nodeID, std::vector<HexagonAction> &actions) const;

	HexagonAction GetAction(const HexagonState &s1, const HexagonState &s2) const;
	void ApplyAction(HexagonState &s, HexagonAction a) const;
	
	void GetNextState(const HexagonState &, HexagonAction , HexagonState &) const;
	bool InvertAction(HexagonAction &a) const;
	
	void RotateCW(HexagonState &s) const;
	
	/** Goal Test if the goal is stored **/
	bool GoalTest(const HexagonState &node) const;
	
	double HCost(const HexagonState &node1, const HexagonState &node2) const { return 0; }
	double GCost(const HexagonState &node1, const HexagonState &node2) const { return 1; }
	double GCost(const HexagonState &node, const HexagonAction &act) const { return 1; }
	bool GoalTest(const HexagonState &node, const HexagonState &goal) const
	{ return GoalTest(node); }

	uint64_t GetStateHash(const HexagonState &node) const;
	uint64_t GetActionHash(HexagonAction act) const;
	
	void Draw(Graphics::Display &display) const;
	/** Draws available pieces and constraints */
	void DrawSetup(Graphics::Display &display) const;
	void Draw(Graphics::Display &display, const HexagonState&) const;
private:
	void Load(const char *, HexagonState &s, bool solution);
	void GetCorners(int x, int y, Graphics::point &p1, Graphics::point &p2, Graphics::point &p3) const;
	bool GetBorder(int x, int y, int xoff, int yoff, Graphics::point &p1, Graphics::point &p2) const;
	bool Valid(int x, int y) const;
	HexagonState solution;
	std::vector<rgbColor> pieceColors;
	std::vector<int> diagPieces;
	std::vector<int> noFlipPieces;
	std::vector<int> notTouchPieces;
	std::vector<int> touchPieces;
};


#endif /* Hexagon_h */
