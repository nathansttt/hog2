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
#include <array>
#include "SearchEnvironment.h"
#include "FPUtil.h"
#include "NBitArray.h"

enum tPieceName {
	kHexagon = 0,
	kButterfly = 1,
	kElbow = 2,
	kLine = 3,
	kMountains = 4,
	kWrench = 5,
	kTriangle = 6,
	kHook = 7,
	kTrapezoid = 8,
	kSnake = 9
};

enum tFlipType {
	kCanFlip = 0,
	kSide1 = 1,
	kSide2 = 2
};

const int numPieces = 10;
const std::string pieceNames[numPieces] =
{
	"Hexagon",
	"Butterfly",
	"Elbow",
	"Line",
	"Mountains",
	"Wrench",
	"Triangle",
	"Hook",
	"Trapezoid",
	"Snake"
};

struct HexagonAction
{
	unsigned int piece : 8; // Support up to 256 pieces for analysis
	unsigned int location : 8; // Each piece can be in up to 256(?) different locations
	// Up to 324 (54x6) locations
//	int location : 6; // 54 possible locations
//	int rotation : 3; // 3 rotations x 2 flips = 6
};

class HexagonState
{
public:
	HexagonState()
	:state(60) {}
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


class HexagonEnvironment;

// (Old) code for loading and displaying files
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
	friend HexagonEnvironment;
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

class HexagonSearchState
{
public:
	HexagonSearchState()
	:bits(0), cnt(0)
	{
	}
	void Reset()
	{ bits = 0; cnt = 0; }
	uint64_t bits;
	int cnt;
	std::array<HexagonAction, 12> state;
//	NBitArray<4> state;
};

static bool operator==(const HexagonAction &l1, const HexagonAction &l2)
{
	return l1.piece == l2.piece && l1.location == l2.location;
}

static bool operator==(const HexagonSearchState &l1, const HexagonSearchState &l2)
{
	if (l1.bits != l2.bits || l1.cnt != l2.cnt)
		return false;
	for (int x = 0; x < l1.cnt; x++)
	{
		bool found = false;
		for (int y = 0; y < l1.cnt; y++) // needed if we have multiples of the same object
		{
			if (l1.state[x] == l2.state[y])
			{
				found = true;
				break;
			}
		}
		if (!found)
			return false;
	}
	return true;
}


// efficient search implementation
class HexagonEnvironment : public SearchEnvironment<HexagonSearchState, HexagonAction>
{
public:
	HexagonEnvironment();
	~HexagonEnvironment();
	void SetPieces(const std::vector<tPieceName> &pieces);
	void SetFlippable(const std::array<tFlipType, numPieces> &flips);
	
	void GetSuccessors(const HexagonSearchState &nodeID, std::vector<HexagonSearchState> &neighbors) const;
	void GetActions(const HexagonSearchState &nodeID, std::vector<HexagonAction> &actions) const;

	HexagonAction GetAction(const HexagonSearchState &s1, const HexagonSearchState &s2) const;
	void ApplyAction(HexagonSearchState &s, HexagonAction a) const;
	void UndoAction(HexagonSearchState &s, HexagonAction a) const;

	void GetNextState(const HexagonSearchState &, HexagonAction , HexagonSearchState &) const;
	bool InvertAction(HexagonAction &a) const;
	
	void RotateCW(HexagonSearchState &s) const;
	HexagonAction RotateCW(HexagonAction a) const;
	void Flip(HexagonSearchState &s) const;
	HexagonAction Flip(HexagonAction a) const;

	/** Goal Test if the goal is stored **/
	bool GoalTest(const HexagonSearchState &node) const;
	
	double HCost(const HexagonSearchState &node1, const HexagonSearchState &node2) const { return 0; }
	double GCost(const HexagonSearchState &node1, const HexagonSearchState &node2) const { return 1; }
	double GCost(const HexagonSearchState &node, const HexagonAction &act) const { return 1; }
	bool GoalTest(const HexagonSearchState &node, const HexagonSearchState &goal) const
	{ return GoalTest(node); }

	uint64_t GetStateHash(const HexagonSearchState &node) const;
	uint64_t GetActionHash(HexagonAction act) const;
	
	/** Prints out the triangles used for this piece in HOG2 coordinates */
	void GeneratePieceCoordinates(tPieceName p);
	/** Prints out the outer coorsinates of the board*/
	void GenerateBoardBorder();
	
	void Draw(Graphics::Display &display) const;
	/** Draws available pieces and constraints */
	void DrawSetup(Graphics::Display &display) const;
	void Draw(Graphics::Display &display, const HexagonSearchState&) const;
private:
	void BuildRotationTable();
	void BuildFlipTable();
	void IndexToXY(int index, int &x, int &y) const;
	void GetCorners(int x, int y, Graphics::point &p1, Graphics::point &p2, Graphics::point &p3) const;
	bool GetBorder(int x, int y, int xoff, int yoff, Graphics::point &p1, Graphics::point &p2) const;
	bool Valid(int x, int y) const;
	std::vector<rgbColor> pieceColors;
	std::vector<int> pieces;
	std::array<tFlipType, numPieces> flippable;
	Hexagon hex;
	int rotate30Map[numPieces][14*6*2+1];
	int flipMap[numPieces][14*6*2+1];
};

#endif /* Hexagon_h */
