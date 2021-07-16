//
//  Witness.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/20/18.
//  Copyright © 2018 University of Denver. All rights reserved.
//

#ifndef Witness_h
#define Witness_h

#include <stdio.h>
#include "SearchEnvironment.h"
#include <bitset>
#include <iostream>
#include <cassert>
#include <array>
#include "vectorCache.h"

template <int width, int height>
int GetEdgeHash(bool horiz, int x, int y)
{
	if (horiz)
	{
		return y*(width)+x;
	}
	return width*(height+1)+y*(width+1)+x;
}

template <int width, int height>
int GetEdgeHash(int x1, int y1, int x2, int y2)
{
	if (y1 == y2)
		return GetEdgeHash<width, height>(true, std::min(x1, x2), y1);
	else if (x1 == x2)
		return GetEdgeHash<width, height>(false, x1, std::min(y1, y2));
	assert(false);
}

template <int width, int height>
class WitnessState {
public:
	WitnessState() { Reset(); }
	void Reset() { path.resize(0); occupiedCorners.reset(); occupiedEdges.reset(); }
	bool Occupied(int x, int y) const { return occupiedCorners[y*(width+1)+x]; }
	void Occupy(int x, int y) { occupiedCorners.set(y*(width+1)+x, true); }
	void Unoccupy(int x, int y) { if (x <= width && y <= height) occupiedCorners.set(y*(width+1)+x, false); }

	bool OccupiedEdge(int x1, int y1, int x2, int y2) const { return occupiedEdges[GetEdgeHash<width, height>(x1, y1, x2, y2)]; }
	void OccupyEdge(int x1, int y1, int x2, int y2) { occupiedEdges.set(GetEdgeHash<width, height>(x1, y1, x2, y2)); }
	void UnoccupyEdge(int x1, int y1, int x2, int y2) { occupiedEdges.set(GetEdgeHash<width, height>(x1, y1, x2, y2), false); }
	std::vector< std::pair<int, int> > path;
	std::bitset<(width+1)*(height+1)> occupiedCorners;
	std::bitset<(width+1)*(height)+(width)*(height+1)> occupiedEdges;
};

enum WitnessAction {
	kUp, kDown, kLeft, kRight, kStart, kEnd
};

template <int width, int height>
class InteractiveWitnessState {
public:
	void Reset() { ws.Reset(); frac = 0; currState = kWaitingStart; }
	void IncrementTime() { if (currState != kWaitingStart) return; frac += 0.04; if (frac > 3) frac = 0; }
	WitnessState<width, height> ws;
	// for drawing
	float frac;
	std::pair<int, int> target; // where we are heading next
	WitnessAction targetAct;
	
	enum controlState {
		kWaitingStart,
		kInPoint,
		kBetweenPoints,
		kWaitingRestart
	};
	controlState currState;
};

template <int width, int height>
static bool operator==(const WitnessState<width, height> &a, const WitnessState<width, height> &b)
{ return a == b; }


template <int width, int height>
class Witness : public SearchEnvironment<WitnessState<width, height>, WitnessAction> {
public:
	enum constraintType {
		kNone=0,
		kSeparation=1,
		kStar=2,
		kTetris=3,
		kNegativeTetris=4,
		kTriangle=5,
		kEraser=6,
		kConstraintCount = 7
	};
	
	Witness()// :separationConstraints(width*height), separationCount(0), tetrisConstraints(width*height), tetrisCount(0)
	{
		static_assert((width<=8)&&(height<=8)&&(width>0)&&(height>0), "Error: width/height must be between 1...8");
		Reset();
	}

	Witness(const Witness<width, height> &w)
	{
		mustCrossEdgeConstraints = w.mustCrossEdgeConstraints;
		mustCrossConstraints = w.mustCrossConstraints;
		constraints = w.constraints;
		constraintCount = w.constraintCount;
		start = w.start;
	}

	Witness<width, height> &operator=(const Witness<width, height> &w)
	{
		mustCrossEdgeConstraints = w.mustCrossEdgeConstraints;
		mustCrossConstraints = w.mustCrossConstraints;
		constraints = w.constraints;
		constraintCount = w.constraintCount;
		start = w.start;
		return *this;
	}
	
	void Reset()
	{
		for (int x = 0; x < width; x++)
		{
			for (int y = 0; y < height; y++)
			{
				constraints[x][y].t = kNone;
			}
		}
		for (int c = 0; c < kConstraintCount; c++)
			constraintCount[c] = 0;
		constraintCount[kNone] = width*height;
		mustCrossConstraints.clear();
		mustCrossEdgeConstraints.clear();
		start.clear();
		start.push_back({0,0});
	}

	void GetSuccessors(const WitnessState<width, height> &nodeID, std::vector<WitnessState<width, height>> &neighbors) const;
	void GetActions(const WitnessState<width, height> &nodeID, std::vector<WitnessAction> &actions) const;
	
	void ApplyAction(WitnessState<width, height> &s, WitnessAction a) const;
	void ApplyAction(std::pair<int, int> &s, WitnessAction a) const;
	bool InvertAction(WitnessAction &a) const;
	void UndoAction(WitnessState<width, height> &s, WitnessAction a) const;
	bool Legal(WitnessState<width, height> &s, WitnessAction a) const;

	/** Heuristic value between two arbitrary nodes. **/
	double HCost(const WitnessState<width, height> &node1, const WitnessState<width, height> &node2) const;
	
	double GCost(const WitnessState<width, height> &node1, const WitnessState<width, height> &node2) const;
	double GCost(const WitnessState<width, height> &node, const WitnessAction &act) const;
	bool GoalTest(const WitnessState<width, height> &node, const WitnessState<width, height> &goal) const;
	bool GoalTest(const WitnessState<width, height> &node) const;

	uint64_t GetMaxHash() const;
	uint64_t GetStateHash(const WitnessState<width, height> &node) const;
	void GetStateFromHash(uint64_t parent, WitnessState<width, height> &s) const;
	uint64_t GetActionHash(WitnessAction act) const;
	
	void OpenGLDraw() const {};
	void OpenGLDraw(const WitnessState<width, height>&) const {};
	/** Draw the transition at some percentage 0...1 between two states */
	void OpenGLDraw(const WitnessState<width, height>&, const WitnessState<width, height>&, float) const {};
	void OpenGLDraw(const WitnessState<width, height>&, const WitnessAction&) const {};
	void GLLabelState(const WitnessState<width, height>&, const char *) const {} ;// draw label over state
	void GLDrawLine(const WitnessState<width, height> &x, const WitnessState<width, height> &y) const {};
	
	void Draw(Graphics::Display &display);
	void Draw(Graphics::Display &display, const WitnessState<width, height>&) const;
	void Draw(Graphics::Display &display, const InteractiveWitnessState<width, height>&) const;

	// returns true if the goal was reached
	bool Click(Graphics::point, InteractiveWitnessState<width, height> &ws);
	void Move(Graphics::point, InteractiveWitnessState<width, height> &ws);

	
	/* Hexagon constraints - path must cross this point */
	constexpr int GetNumMustCrossConstraints() const;
	void ClearMustCrossConstraints() { mustCrossConstraints.clear(); mustCrossEdgeConstraints.clear(); }
	void SetMustCrossConstraint(int);
	bool GetMustCrossConstraint(int) const;
	void ClearMustCrossConstraint(int);
	void AddMustCrossConstraint(bool horiz, int x, int y) { mustCrossEdgeConstraints.push_back({horiz, {x, y}});}
	void AddMustCrossConstraint(int x, int y) { mustCrossConstraints.push_back({x, y});}
	void RemoveMustCrossConstraint(bool horiz, int x, int y) { mustCrossEdgeConstraints.pop_back();}
	void RemoveMustCrossConstraint(int x, int y) { mustCrossConstraints.pop_back();}

	void ClearInnerConstraints()
	{
		for (int x = 0; x < width; x++)
		{
			for (int y = 0; y < height; y++)
			{
				constraints[x][y].t = kNone;
			}
		}
		for (int c = 0; c < kConstraintCount; c++)
			constraintCount[c] = 0;
		constraintCount[kNone] = width*height;
	}

	void ClearConstraint(constraintType t)
	{
		for (int x = 0; x < width; x++)
		{
			for (int y = 0; y < height; y++)
			{
				if (t == constraints[x][y].t)
				{
					constraintCount[t]--;
					constraints[x][y].t = kNone;
					constraintCount[kNone]++;
				}
			}
		}
	}

	void ClearConstraint(int x, int y)
	{
		constraintCount[constraints[x][y].t]--;
		constraintCount[kNone]++;
		constraints[x][y].t = kNone;
	}

	
	/* Triangles - must cross as many edges as triangles */
	constexpr int GetNumTriangleConstraints() const { return width*height; }
	void ClearTriangleConstraints()
	{
		ClearConstraint(kTriangle);
		//triangleConstraints.clear(); triangleConstraints.resize(width*height); triangleCount = 0;
	}
	void AddTriangleConstraint(int x, int y, int count)
	{
		assert(count >= 1 && count <= 3);
		constraintCount[constraints[x][y].t]--;
		constraintCount[kTriangle]++;
		constraints[x][y].t = kTriangle;
		constraints[x][y].parameter = count;
		constraints[x][y].c = triangleColor;
	}
//	{ triangleCount++; triangleConstraints[y*width+x] = count; }
	void AddTriangleConstraint(int which, int count) //{ triangleCount++; triangleConstraints[which] = count; }
	{
		AddTriangleConstraint(which%width, which/width, count);
	}

	/* Color (rounded) square - must separate these */
	/* TODO: Star constraints are a special case */
	void ClearSeparationConstraints()
	{
		ClearConstraint(kSeparation);
	}
	//{ separationConstraints.clear(); separationConstraints.resize(width*height), separationCount = 0; }
	constexpr int GetNumSeparationConstraints() const { return width*height; }
	void AddSeparationConstraint(int x, int y, rgbColor c)
	{
		constraintCount[constraints[x][y].t]--;
		constraintCount[kSeparation]++;
		constraints[x][y].t = kSeparation;
		constraints[x][y].c = c;
	}
//	{ auto &i = separationConstraints[y*width+x]; i.color = c; i.valid = true; separationCount++;}
	void AddSeparationConstraint(int which, rgbColor c)
	//{ auto &i = separationConstraints[which]; i.color = c; i.valid = true; separationCount++;}
	{
		AddSeparationConstraint(GetX(which), GetY(which), c);
	}
//	void RemoveSeparationConstraint(int x, int y) { separationConstraints[y*width+x].valid = false; separationCount--;}
//	void RemoveSeparationConstraint(int which) { separationConstraints[which].valid = false; separationCount--;}

	// TODO: Not yet complete
	/* Tetris constraints - must solve packing problem to validate these */
	/* We allow most constraints with 1...4 blocks -- 14 total */
	/*  1  *
	 *  2  **
	 *  3  *
	 *     *
	 *  4  **
	 *     *
	 *  5  **
	 *      *
	 *  6  *
	 *     **
	 *  7   *
	 *     **
	 *  8  ***
	 *  9  *
	 *     *
	 *     *
	 *  10 **
	 *     **
	 *  11 ***
	 *     *
	 *  12 *
	 *     ***
	 *  13 ***
	 *       *
	 *  14   *
	 *     ***
	 *  15 **
	 *     *
	 *     *
	 *  16 **
	 *      *
	 *      *
	 *  17 *
	 *     *
	 *     **
	 *  18  *
	 *      *
	 *     **
	 *  19 ****
	 *  20 *
	 *     *
	 *     *
	 *     *
	 *  21 ***
	 *      *
	 *  22  *
	 *     ***
	 *  23 *
	 *     **
	 *     *
	 *  24  *
	 *     **
	 *      *
	 */
	void ClearTetrisConstraints()
	//{ tetrisConstraints.resize(0); tetrisConstraints.resize(width*height); tetrisCount = 0; }
	{
		ClearConstraint(kTetris);
		ClearConstraint(kNegativeTetris);
	}
	void AddNegativeTetrisConstraint(int x, int y, int which)
	{
		assert(which >= 1);
		constraintCount[constraints[x][y].t]--;
		constraintCount[kNegativeTetris]++;
		constraints[x][y].t = kNegativeTetris;
		constraints[x][y].c = tetrisBlue;
		constraints[x][y].parameter = which;
	}
	void AddTetrisConstraint(int x, int y, int which)
	{
		assert(which >= 1);
		constraintCount[constraints[x][y].t]--;
		constraintCount[kTetris]++;
		constraints[x][y].t = kTetris;
		constraints[x][y].c = tetrisYellow;
		constraints[x][y].parameter = which;
	}
	
	void AddNegativeTetrisConstraint(int loc, int which)
	{
		AddNegativeTetrisConstraint(GetX(loc), GetY(loc), which);
	}
	void AddTetrisConstraint(int loc, int which)// { tetrisConstraints[loc] = which; tetrisCount++; }
	{
		AddTetrisConstraint(GetX(loc), GetY(loc), which);
	}
	constexpr int GetNumTetrisConstraints() const { return width*height; }
	// TODO: don't increase count if piece is already there
	const uint16_t tetrisBits[25] =
	{ 0, 0x8000, 0xC000, 0x8800, 0xC800, 0xC400, 0x8C00, 0x4C00, 0xE000, 0x8880, 0xCC00, 0xE800, 0x8E00, 0xE200, 0x2E00, 0xC880, 0xC440, 0x88C0, 0x44C0, 0xF000, 0x8888,
		0xE400, 0x4E00, 0x8C80, 0x4C40
	};
	const uint64_t tetrisBits64[25] =
	{0,
		0x8000000000000000ull, 0xC000000000000000ull, 0x8080000000000000ull, 0x80C0000000000000ull,
		0x40C0000000000000ull, 0xC080000000000000ull, 0xC040000000000000ull,
		0xE000000000000000ull, 0x8080800000000000ull, 0xC0C0000000000000ull,
		0x80E0000000000000ull, 0xE080000000000000ull, 0x20E0000000000000ull, 0xE020000000000000ull,
		0x8080C00000000000ull, 0x4040C00000000000ull, 0xC080800000000000ull, 0xC040400000000000ull,
		0xF000000000000000ull, 0x8080808000000000ull,
		0x40E0000000000000ull, 0xE040000000000000ull, 0x80C0800000000000ull, 0x40C0400000000000ull
	};
	const uint16_t tetrisSize[25] =
	{ 0, 1, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4 };
	const uint16_t tetrisWH[25][2] =
	{ {0,0}, {1,1}, {2,1}, {1,2}, {2,2}, {2,2}, {2,2}, {2,2}, {3,1}, {1,3}, {2,2}, {3,2}, {3,2}, {3,2}, {3,2}, {2,3}, {2,3}, {2,3}, {2,3}, {4,1}, {1,4}, {3, 2}, {3, 2}, {2, 3}, {2, 3} };

	
	void ClearStarConstraints()
	{
		ClearConstraint(kStar);
	}
	constexpr int GetNumStarConstraints() const { return width*height; }
	void AddStarConstraint(int x, int y, rgbColor c)
	{
		constraintCount[constraints[x][y].t]--;
		constraintCount[kStar]++;
		constraints[x][y].t = kStar;
		constraints[x][y].c = c;
	}
	void AddStarConstraint(int which, rgbColor c)
	{
		AddStarConstraint(GetX(which), GetY(which), c);
	}
	
	
	std::vector<std::pair<int, int>> start;
//	const int kStartX = 0, kStartY = 0;

	std::string SaveToHashString() const
	{
		// JSON string { "name":"value" }
		// JSON number { "name":number }
		// JSON array { "name":{ "object", "object", "object"} }
		std::string quote = "\"";
		std::string hash = "{";
		
		{
			hash += quote + "dim" + quote + ":" + quote + std::to_string(width)+"x"+std::to_string(height) + quote;
			hash += ",";
		}

		{
			hash += quote + "cc" + quote + ":{";
			for (int x = 0; x < width; x++)
			{
				for (int y = 0; y < height; y++)
				{
					if (x != 0 || y != 0)
						hash += ",";
					hash += quote;
					hash += std::to_string(constraints[x][y].t)+";";
					// no point writing garbage
					if (constraints[x][y].t == kNone)
						hash += "0;";
					else
						hash += std::to_string(constraints[x][y].parameter)+";";
					hash += constraints[x][y].c.hex();
					hash += quote;
				}
			}
			hash += "},";
		}
		
		{
			hash += quote + "mc" + quote + ":" + quote;
			// add must-cross constraints
			for (int x = 0; x < GetNumMustCrossConstraints(); x++)
				hash += GetMustCrossConstraint(x)?"1":"0";
			hash += quote;
		}
		hash += "}";
		return hash;
	}
	void GetDimensionsFromHashString(const std::string &s, int &w, int &h) const
	{
		const char *loc = strstr(s.c_str(), "dim");
		if (loc == NULL)
		{
			w = -1;
			h = -1;
			return;
		}
		char a, b;
		sscanf(loc, "%*[^0-9]%cx%c", &a, &b);
		w = a-'0';
		h = b-'0';
		printf("Width: %d; height: %d\n", w, h);
	}

	// TODO: Code assumes input is well-formed! Fix this.
	void LoadFromHashString(std::string s)
	{
		Reset();
		// looking for dim, cc & mc
		int w, h;
		GetDimensionsFromHashString(s, w, h);
		if (w != width || h != height)
		{
			printf("Dimensions do not match, aborting load!\n");
			return;
		}
		
		// get crossing constraints
		{
			const char *loc = strstr(s.c_str(), "mc");
			if (loc == NULL)
			{
				printf("Failed loading MC constraints\n");
				return;
			}
			// get end of mc variable
			while (loc[0] != ':')
				loc++;
			// get start of mc constraints
			while (loc[0] != '"')
				loc++;
			loc++;
			for (int x = 0; x < GetNumMustCrossConstraints(); x++)
			{
				if (loc[0] == '1')
					SetMustCrossConstraint(x);
				loc++;
			}
		}
		
		// get regular constraints
		{
			const char *loc = strstr(s.c_str(), "cc");
			if (loc == NULL)
			{
				printf("Failed loading CC constraints\n");
				return;
			}
			// get end of mc variable
			while (loc[0] != '{')
				loc++;
			
			for (int x = 0; x < width; x++)
			{
				for (int y = 0; y < height; y++)
				{
					if (loc[0] == '}')
					{
						printf("Unexpected '}', aborting\n");
						return;
					}
					while (loc[0] != '"')
						loc++;
					
					loc++; // skip open quote
					int cType = atoi(loc);
					while (loc[0] != ';')
						loc++;
					loc++;
					int param = atoi(loc);
					while (loc[0] != ';')
						loc++;
					loc++;
					rgbColor c;
					c.hex(loc);
					
					constraintCount[constraints[x][y].t]--;
					constraints[x][y].t = static_cast<Witness<width, height>::constraintType>(cType);
					constraintCount[constraints[x][y].t]++;
					constraints[x][y].c = c;
					constraints[x][y].parameter = param;

					while (loc[0] != ',')
						loc++;
				}
			}
		}
	}

	const rgbColor tetrisYellow = {0.862745098f, 0.6549019608f, 0.0f};
	const rgbColor tetrisBlue = {0.2196078431f, 0.3607843137f, 0.8705882353f};
	const rgbColor drawColor = Colors::darkbluegray;//Colors::lightblue;
	const rgbColor lineColor = Colors::lightgray;
	const rgbColor backColor = Colors::gray;
	const rgbColor outerBackColor = Colors::white;
	const rgbColor triangleColor = Colors::orange;
private:
	const float scale = 0.75f;
	const float gapOffset = scale*2.0f/(width>height?width:height); // size of each square
	const float lineWidth = gapOffset*0.1f;
	const float xGap = (((width>height)?(width):(height))-width)*gapOffset/2.0f;
	const float yGap = -(((width>height)?(width):(height))-height)*gapOffset/2.0f;


	int GetIndex(int x, int y) const { return y*width+x; }
	int GetX(int index) const { return index%width; }
	int GetY(int index) const { return index/width; }
	struct constraint {
		constraintType t;
		int parameter;
		rgbColor c;
	};
	
	// Must cross edge/node
	struct mustCrossEdgeConstraint {
		bool horiz;
		std::pair <int, int> location;
		bool operator==(const mustCrossEdgeConstraint &a)
		{ return a.horiz == this->horiz && a.location == this->location; }
	};
	std::vector<mustCrossEdgeConstraint> mustCrossEdgeConstraints;
	std::vector<std::pair<int, int>> mustCrossConstraints;

	struct separationObject {
		separationObject() :valid(false), color(Colors::pink) {}
		bool valid;
		rgbColor color;
	};
	std::array<std::array<constraint, height>, width> constraints;
//	constraint constraints[width][height];
	std::array<int, (int)kConstraintCount> constraintCount;
//	int constraintCount[kConstraintCount];

	// TODO: merge these
//	std::vector<separationObject> separationConstraints;
//	int separationCount;
//
//	std::vector<int> tetrisConstraints;
//	int tetrisCount;
//
//	std::vector<int> triangleConstraints;
//	int triangleCount;

	
	void LabelRegions(const WitnessState<width, height> &s) const;
	// this is mapped to a 2d map which has the region number for any x/y location
	//mutable std::vector<int> regions;
	mutable std::array<int, width*height> regions;
	// for each region, this is a list of the cells in the region
	// the size of the vector tells us the size of the region
	mutable std::vector<std::vector<int>*> regionList;
	mutable vectorCache<int> regionCache;

	mutable std::vector<int> tetrisBlockCount;
	mutable std::vector<int> tetrisBlocksInRegion;

	bool RecursivelyPlacePieces(int curr, uint64_t board, uint64_t oob, uint64_t posFootprint, uint64_t negFootprint) const;

	void GetMouseActions(const WitnessState<width, height> &nodeID, std::vector<WitnessAction> &actions) const;

	void DoLine(Graphics::Display &display, const Graphics::point &p1, const Graphics::point &p2, const rgbColor &c) const
	{
		if (p1.x == p2.x) // vertical line
		{
			display.FillRect({(p1.x-lineWidth), std::min(p1.y, p2.y), (p1.x+lineWidth), std::max(p1.y, p2.y)}, c);
		}
		else {
			display.FillRect({std::min(p1.x, p2.x), (p1.y-lineWidth), std::max(p1.x, p2.x), (p1.y+lineWidth)}, c);
		}
	}
	Graphics::point GetScreenCoord(int x, int y) const
	{
		if (x > width || y > height)
			return {(-scale+width*gapOffset+xGap), (scale-(height+4*lineWidth)*gapOffset+yGap)};

		return {(-scale+x*gapOffset+xGap), (scale-y*gapOffset+yGap)};
	}
	
	void DebugPrint(uint64_t val, int offset=0) const
	{
		for (int y = 7; y >= 0; y--)
		{
			for (int x = 0; x < offset; x++)
				printf(" ");
			for (int x = 0; x < 8; x++)
			{
				printf("%llu", (val>>(7-x))>>((7-y)*8)&0x1);
			}
			printf("\n");
		}
	}
};

template <int width, int height>
void Witness<width, height>::GetSuccessors(const WitnessState<width, height> &nodeID, std::vector<WitnessState<width, height>> &neighbors) const
{
	
}
template <int width, int height>
void Witness<width, height>::GetActions(const WitnessState<width, height> &nodeID, std::vector<WitnessAction> &actions) const
{
	actions.resize(0);
	if (nodeID.path.size() == 0)
	{
		actions.push_back(kStart);
		return;
	}

	int currX = nodeID.path.back().first;
	int currY = nodeID.path.back().second;

	// TODO: Only works with one exit going from lower left to upper right
	if (currX == width && currY == height)
	{
		actions.push_back(kEnd);
		return;
	}

	if (currX > width || currY > height)
		return;

	if (currX > 0 && !nodeID.Occupied(currX-1, currY))
		actions.push_back(kLeft);
	if (currX < width && !nodeID.Occupied(currX+1, currY))
		actions.push_back(kRight);
	if (currY > 0 && !nodeID.Occupied(currX, currY-1))
		actions.push_back(kDown);
	if (currY < height && !nodeID.Occupied(currX, currY+1))
		actions.push_back(kUp);
}

template <int width, int height>
void Witness<width, height>::GetMouseActions(const WitnessState<width, height> &nodeID, std::vector<WitnessAction> &actions) const
{
	actions.resize(0);
	if (nodeID.path.size() == 0)
	{
		actions.push_back(kStart);
		return;
	}
	int currX = nodeID.path.back().first;
	int currY = nodeID.path.back().second;
	
	if (currX == width && currY == height)
		actions.push_back(kEnd);
	if (currX > width || currY > height)
	{
		actions.push_back(kDown);
		actions.push_back(kEnd);
		return;
	}

	if (currX > 0)
		actions.push_back(kLeft);
	if (currX < width)
		actions.push_back(kRight);
	if (currY > 0)
		actions.push_back(kDown);
	if (currY < height)
		actions.push_back(kUp);
}

template <int width, int height>
void Witness<width, height>::ApplyAction(std::pair<int, int> &s, WitnessAction a) const
{
	switch (a)
	{
		case kEnd:
			//s.first++;
			s.second++;
			break;
		case kStart:
			break;
		case kLeft:
			s.first--;
			break;
		case kRight:
			s.first++;
			break;
		case kUp:
			s.second++;
			break;
		case kDown:
			s.second--;
			break;
	}
}

template <int width, int height>
void Witness<width, height>::ApplyAction(WitnessState<width, height> &s, WitnessAction a) const
{
	auto len = s.path.size()+1;
	switch (a)
	{
		case kEnd:
			s.path.push_back({width, height+1});
			return; // don't occupe on end action
		case kStart:
			s.Reset();
			s.path.push_back(start[0]);
			s.Occupy(s.path.back().first, s.path.back().second);
			break;
		case kLeft:
			s.path.push_back(s.path.back());
			s.path.back().first--;
			s.Occupy(s.path.back().first, s.path.back().second);
			s.OccupyEdge(s.path[len-1].first, s.path[len-1].second, s.path[len-2].first, s.path[len-2].second);
			break;
		case kRight:
			s.path.push_back(s.path.back());
			s.path.back().first++;
			s.Occupy(s.path.back().first, s.path.back().second);
			s.OccupyEdge(s.path[len-1].first, s.path[len-1].second, s.path[len-2].first, s.path[len-2].second);
			break;
		case kUp:
			s.path.push_back(s.path.back());
			s.path.back().second++;
			s.Occupy(s.path.back().first, s.path.back().second);
			s.OccupyEdge(s.path[len-1].first, s.path[len-1].second, s.path[len-2].first, s.path[len-2].second);
			break;
		case kDown:
			s.path.push_back(s.path.back());
			s.path.back().second--;
			s.Occupy(s.path.back().first, s.path.back().second);
			s.OccupyEdge(s.path[len-1].first, s.path[len-1].second, s.path[len-2].first, s.path[len-2].second);
			break;
	}

	//s.occupiedCorners.set(s.path.back().second*width+s.path.back().first); // y*width+x;
}

template <int width, int height>
void Witness<width, height>::UndoAction(WitnessState<width, height> &s, WitnessAction a) const
{
	auto len = s.path.size();
	if (s.path[len-1].first <= width && s.path[len-1].second <= height)
	{
		if (len > 1)
			s.UnoccupyEdge(s.path[len-1].first, s.path[len-1].second, s.path[len-2].first, s.path[len-2].second);
		s.Unoccupy(s.path.back().first, s.path.back().second);
	}
	s.path.pop_back();
}

template <int width, int height>
bool Witness<width, height>::Legal(WitnessState<width, height> &s, WitnessAction a) const
{
	int currX = s.path.back().first;
	int currY = s.path.back().second;
	switch (a)
	{
		case kEnd:
			return (currX == width && currY == height);
		case kStart:
			return s.path.size() == 0;
		case kLeft:
			return (currX > 0 && !s.Occupied(currX-1, currY));
		case kRight:
			return (currX < width && !s.Occupied(currX+1, currY));
		case kDown:
			return (currY > 0 && !s.Occupied(currX, currY-1));
		case kUp:
			return (currY < height && !s.Occupied(currX, currY+1));
	}
	return false;
}

template <int width, int height>bool Witness<width, height>::InvertAction(WitnessAction &a) const
{
	switch (a)
	{
		case kStart: return false;
		case kEnd: return false;
		case kUp: a = kDown; break;
		case kDown: a = kUp; break;
		case kLeft: a = kRight; break;
		case kRight: a = kLeft; break;
	}
	return true;
}

/** Heuristic value between two arbitrary nodes. **/
template <int width, int height>
double Witness<width, height>::HCost(const WitnessState<width, height> &node1, const WitnessState<width, height> &node2) const
{
	return 0;
}

template <int width, int height>
double Witness<width, height>::GCost(const WitnessState<width, height> &node1, const WitnessState<width, height> &node2) const
{
	return 1;
}
template <int width, int height>
double Witness<width, height>::GCost(const WitnessState<width, height> &node, const WitnessAction &act) const
{
	return 1;
}
template <int width, int height>
bool Witness<width, height>::GoalTest(const WitnessState<width, height> &node, const WitnessState<width, height> &goal) const
{
	// Check constraints
	return GoalTest(node);
}

template <int width, int height>
bool Witness<width, height>::GoalTest(const WitnessState<width, height> &node) const
{
	for (auto &c : mustCrossConstraints)
	{
		if (!node.Occupied(c.first, c.second))
			return false;
	}
	// First pass - mustCross
	for (auto &c : mustCrossEdgeConstraints)
	{
//		printf("Checking (%d, %d) to (%d, %d) - ", c.location.first, c.location.second, c.location.first+(c.horiz?1:0), c.location.second+(c.horiz?0:1));
		if (!node.OccupiedEdge(c.location.first, c.location.second, c.location.first+(c.horiz?1:0), c.location.second+(c.horiz?0:1)))
		{
//			printf("Failure\n");
			return false;
		}
//		printf("Success\n");
	}

	// Didn't hit end of puzzle
	if (node.path.size() == 0 || node.path.back().second <= height)
		return false;
	
	if (constraintCount[kTriangle] > 0)
		//if (triangleCount > 0)
	{
		for (int x = 0; x < width; x++)
		{
			for (int y = 0; y < height; y++)
			{
				if (constraints[x][y].t == kTriangle)
//				if (triangleConstraints[y*width+x] > 0)
				{
					int count = node.OccupiedEdge(x, y, x, y+1);
					count += node.OccupiedEdge(x, y, x+1, y);
					count += node.OccupiedEdge(x+1, y, x+1, y+1);
					count += node.OccupiedEdge(x, y+1, x+1, y+1);
//					if (count != triangleConstraints[y*width+x])
					if (count != constraints[x][y].parameter)
						return false;
				}
			}
		}
	}
	
	if (constraintCount[kSeparation] == 0 && constraintCount[kTetris] == 0 &&
		constraintCount[kStar] == 0 && constraintCount[kEraser] == 0)
//	if (separationCount == 0 && tetrisCount == 0)
		return true;

	LabelRegions(node);

	if (constraintCount[kSeparation] > 0)
	{
		for (auto &v : regionList) // vector of locations
		{
			bool found = false;
			rgbColor c;
			for (auto &i : *v)
			{
				int x = GetX(i);//l%width;
				int y = GetY(i);//l/width;

				//if (separationConstraints[i].valid)
				if (constraints[x][y].t == kSeparation)
				{
					if (!found)
					{
						c = constraints[x][y].c;//separationConstraints[i].color;
						found = true;
					}
					else if (c != constraints[x][y].c)//separationConstraints[i].color)
					{
						return false;
					}
				}
			}
		}
	}

	// TODO: After this is working, see if we can merge the tetris constraints into the separation code
	if (constraintCount[kTetris] == 0 && constraintCount[kNegativeTetris] > 0)
		return false;

	if (constraintCount[kStar] > 0)
	{
		for (auto &v : regionList) // vector of locations
		{
			for (auto &i : *v)
			{
				int x = GetX(i);//l%width;
				int y = GetY(i);//l/width;
				rgbColor finishedColor(1.0/512.0, 1.0/512.0,1.0/512.0);
				//if (separationConstraints[i].valid)
				if (constraints[x][y].t == kStar)
				{
					if (constraints[x][y].c == finishedColor)
						continue;
					
					finishedColor = constraints[x][y].c;//separationConstraints[i].color;
					int count = 0;
					for (auto &r : *v)
					{
						int xx = GetX(r);//l%width;
						int yy = GetY(r);//l/width;
						
						if (constraints[xx][yy].t != kNone &&
							constraints[x][y].c == constraints[xx][yy].c)
						{
							count++;
							if (count > 2)
								return false;
						}
					}
					if (count != 2)
						return false;
				}
			}
		}
	}
	
	if (constraintCount[kTetris] > 0)
//	if (tetrisCount > 0)
	{
		tetrisBlockCount.resize(regionList.size());
		
		// 1. Collect the tetris constraints for each region
		for (int x = 0; x < regionList.size(); x++)
		{
			const auto &v = *regionList[x]; // v is vector of locations in this region
			tetrisBlockCount[x] = 0;
			for (auto l : v) // individual location
			{
				if (constraints[GetX(l)][GetY(l)].t == kTetris)
				{
					int whichConstraint = constraints[GetX(l)][GetY(l)].parameter;
					int numPieces = tetrisSize[whichConstraint];
					tetrisBlockCount[x] += numPieces;
				}
				else if (constraints[GetX(l)][GetY(l)].t == kNegativeTetris)
				{
					int whichConstraint = constraints[GetX(l)][GetY(l)].parameter;
					int numPieces = tetrisSize[whichConstraint];
					tetrisBlockCount[x] -= numPieces;
				}
			}
			// 2. Make sure the counts of tetris blocks matches the region size
			if (tetrisBlockCount[x] > 0 && tetrisBlockCount[x] != regionList[x]->size())
			{
				//printf("Region %d has %d tetris blocks and size %lu\n", x, tetrisBlockCount[x], regionList[x].size());
				return false;
			}
		}
		
		
		// 3. Do the full layout
		for (int x = 0; x < regionList.size(); x++)
		{
			bool hasNegations = false;
			const auto &v = *regionList[x]; // v is vector of locations in this region
			if (v.size() == 0)
				continue;
			
			tetrisBlocksInRegion.resize(0);
			// Get bit map of board
			uint64_t board = 0;

			for (auto l : v) // individual location
			{
				uint64_t xx = GetX(l);//l%width;
				uint64_t yy = GetY(l);//l/width;
				// x and y are offset from bottom left (screen space)
				// need to convert to 8x8 bitmaps space
				board |= ((1ull<<(7-xx))<<((7-yy)*8));

				if (constraints[GetX(l)][GetY(l)].t == kTetris)
				{
					int whichConstraint = constraints[xx][yy].parameter;//tetrisConstraints[l];
					tetrisBlocksInRegion.push_back(whichConstraint);
				}
				if (constraints[GetX(l)][GetY(l)].t == kNegativeTetris)
				{
					int whichConstraint = constraints[xx][yy].parameter;//tetrisConstraints[l];
					tetrisBlocksInRegion.push_back(-whichConstraint);
					hasNegations = true;

				}

//				int whichConstraint = constraints[xx][yy].parameter;//tetrisConstraints[l];
//				if (whichConstraint > 0)
//				{
//					tetrisBlocksInRegion.push_back(whichConstraint);
//				}
//				else if (whichConstraint < 0)
//				{ // negative constraints are just another xor
//					tetrisBlocksInRegion.push_back(whichConstraint);
//					hasNegations = true;
//				}
			}
			
			if (tetrisBlocksInRegion.size() == 0)
				continue;
			
			// Get out of bounds map -- places pieces can't go
			uint64_t oob = ~board;
			if (hasNegations)
				oob = 0;
//			printf("Region %d\n", x);
//			DebugPrint(board);
//			DebugPrint(oob);

			// 4. Now we have all pieces, recursively try to place them
			if (!RecursivelyPlacePieces(0, board, oob, 0, 0))
				return false; // No way to place them
//			printf("-%d-\n", 0);
//			DebugPrint(board, 0);
//			printf("Region %d successful\n", x);
		}
		return true; // didn't find a way to place them
	}
	
	return true;
}

template <int width, int height>
bool Witness<width, height>::RecursivelyPlacePieces(int curr, uint64_t board, uint64_t oob, uint64_t posFootprint, uint64_t negFootprint) const
{
//	DebugPrint(board, curr*2);
	if (curr == tetrisBlocksInRegion.size())
		return (board == 0) && ((negFootprint&posFootprint)==negFootprint);
	
	bool neg = false;
	int whichBlock = tetrisBlocksInRegion[curr];
	if (whichBlock < 0)
	{
		neg = true;
		whichBlock = -whichBlock;
	}
	
	// In theory we should just check where pieces are placed, instead of the whole grid.
	// But, this optimization currently requires that the upper/left corner of the piece bitmap always
	// contains a piece, which it doesn't. Adding a reference point offset for each piece type would
	// solve this, but we are first worried about writing correct code
	for (int x = 0; x < width-tetrisWH[whichBlock][0]+1; x++)
	{
		for (int y = 0; y < height-tetrisWH[whichBlock][1]+1; y++)
		{
			//printf("%d in %d,%d\n", whichBlock, x, y);
			uint64_t block = ((tetrisBits64[whichBlock]>>x)>>(8*y));
			board ^= block;
			if ((board&oob) == 0) // piece not out of bounds
			{
				if (RecursivelyPlacePieces(curr+1, board, oob, neg?posFootprint:(posFootprint|block), neg?(negFootprint|block):negFootprint))
				{
//					printf("-%d-\n", curr+1);
//					DebugPrint(board, 0);
					return true;
				}
			}
			else {
//				DebugPrint(board, curr*2+4);
			}
			board ^= block;//(tetrisBits64[whichBlock]>>x)>>(8*y);
		}
	}
	return false;
}


template <int width, int height>
uint64_t Witness<width, height>::GetMaxHash() const
{
	assert(false);
}
template <int width, int height>
uint64_t Witness<width, height>::GetStateHash(const WitnessState<width, height> &node) const
{
	assert(false);
}
template <int width, int height>
void Witness<width, height>::GetStateFromHash(uint64_t parent, WitnessState<width, height> &s) const
{
	assert(false);
}
template <int width, int height>
uint64_t Witness<width, height>::GetActionHash(WitnessAction act) const
{
	return act;
}


template <int width, int height>
constexpr int Witness<width, height>::GetNumMustCrossConstraints() const
{
	return width*(height+1)+(width+1)*height+(width+1)*(height+1);
}

template <int width, int height>
void Witness<width, height>::SetMustCrossConstraint(int which)
{
	if (which < width*(height+1))
	{
		int x = which%(width);
		int y = which/(width);
		AddMustCrossConstraint(true, x, y);
		return;
	}
	which -= width*(height+1);
	if (which < (width+1)*height)
	{
		int x = which%(width+1);
		int y = which/(width+1);
		AddMustCrossConstraint(false, x, y);
		return;
	}
	which -= (width+1)*height;
	// constraint on corner
	int x = which%(width+1);
	int y = which/(width+1);
	AddMustCrossConstraint(x, y);
}

template <int width, int height>
bool Witness<width, height>::GetMustCrossConstraint(int which) const
{
	if (which < width*(height+1))
	{
		int x = which%(width);
		int y = which/(width);
		mustCrossEdgeConstraint e = {true, {x, y}};
		for (const auto &c : mustCrossEdgeConstraints)
			if (e == c)
				return true;
		return false;
	}
	which -= width*(height+1);
	if (which < (width+1)*height)
	{
		int x = which%(width+1);
		int y = which/(width+1);
		mustCrossEdgeConstraint e = {false, {x, y}};
		for (const auto &c : mustCrossEdgeConstraints)
			if (e == c)
				return true;
		return false;
	}
	which -= (width+1)*height;
	// constraint on corner
	int x = which%(width+1);
	int y = which/(width+1);
	std::pair<int, int> e = {x, y};
	for (const auto &c : mustCrossConstraints)
		if (e == c)
			return true;
	return false;
}

template <int width, int height>
void Witness<width, height>::ClearMustCrossConstraint(int which)
{
	if (which < width*(height+1))
	{
		int x = which%(width);
		int y = which/(width);
		RemoveMustCrossConstraint(true, x, y);
		return;
	}
	which -= width*(height+1);
	if (which < (width+1)*height)
	{
		int x = which%(width+1);
		int y = which/(width+1);
		RemoveMustCrossConstraint(false, x, y);
		return;
	}
	which -= (width+1)*height;
	// constraint on corner
	int x = which%(width+1);
	int y = which/(width+1);
	RemoveMustCrossConstraint(x, y);
}



template <int width, int height>
void Witness<width, height>::LabelRegions(const WitnessState<width, height> &s) const
{
	regions.fill(0);
//	regions.clear();
//	regions.resize(width*height);
	for (std::vector<int> *i : regionList)
	{
		//printf("Returned %p; ", i);
		regionCache.returnItem(i);
	}
	regionList.resize(0);
	//printf("Size to %lu\n", regionList.size());

	static std::vector<int> queue;
	queue.clear();
	
	int index = 0;
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			if (regions[GetIndex(x, y)/*y*width+x*/] == 0)
			{
				queue.push_back(GetIndex(x, y));
				index++;
				while (regionList.size() < index)
				{
					regionList.push_back(regionCache.getItem());
//					printf("Got %p; ", regionList.back());
//					printf("size to %lu\n", regionList.size());
				}
				//regionList.resize(index);
			}
			while (queue.size() > 0)
			{
				int next = queue.back();
				queue.pop_back();
				
				if (regions[next] == 0)
				{
					regions[next] = index;
					regionList[index-1]->push_back(next);
					// check 4 directions
					int xx = GetX(next);
					int yy = GetY(next);
					if (xx > 0 && !s.OccupiedEdge(xx, yy, xx, yy+1) && regions[GetIndex(xx-1, yy)] == 0)
						queue.push_back(GetIndex(xx-1, yy));
					if (xx < width-1 && !s.OccupiedEdge(xx+1, yy, xx+1, yy+1) && regions[GetIndex(xx+1, yy)] == 0)
						queue.push_back(GetIndex(xx+1, yy));
					if (yy > 0 && !s.OccupiedEdge(xx, yy, xx+1, yy) && regions[GetIndex(xx, yy-1)] == 0)
						queue.push_back(GetIndex(xx, yy-1));
					if (yy < height-1 && !s.OccupiedEdge(xx, yy+1, xx+1, yy+1) && regions[GetIndex(xx, yy+1)] == 0)
						queue.push_back(GetIndex(xx, yy+1));
				}
			}
			
		}
	}

//	for (auto i : s.path)
//		std::cout << "(" << i.first << ", " << i.second << ") ";
//	std::cout << "\n";
//
//	printf(" ");
//	for (int x = 0; x < width; x++)
//	{
//		if (s.OccupiedEdge(x, 0, x+1, 0))
//			printf("- ");
//		else
//			printf("  ");
//	}
//	printf("\n");
//
//	for (int y = 0; y < height; y++)
//	{
//		if (s.OccupiedEdge(0, y, 0, y+1))
//			printf("|");
//		else
//			printf(" ");
//		for (int x = 0; x < width; x++)
//		{
//			printf("%d", regions[GetIndex(x, y)]);
//			if (s.OccupiedEdge(x+1, y, x+1, y+1))
//				printf("|");
//			else
//				printf(" ");
//		}
//		printf("\n");
//		printf(" ");
//		for (int x = 0; x < width; x++)
//		{
//			if (s.OccupiedEdge(x, y+1, x+1, y+1))
//				printf("- ");
//			else
//				printf("  ");
//		}
//		printf("\n");
//	}
//	printf("\n\n");
}





template <int width, int height>
bool Witness<width, height>::Click(Graphics::point mouseLoc, InteractiveWitnessState<width, height> &iws)
{
	// in start area
	//std::cout << mouseLoc-GetScreenCoord(0, 0) << " size " << (mouseLoc-GetScreenCoord(0, 0)).length() << " vs " << 3*lineWidth << "\n";
	if ((iws.currState == InteractiveWitnessState<width, height>::kWaitingStart ||
		 iws.currState == InteractiveWitnessState<width, height>::kWaitingRestart) &&
		(mouseLoc-GetScreenCoord(start[0].first, start[0].second)).length() < 3*lineWidth)
	{
		//printf("Switched from watiting to start to kInPoint\n");
		ApplyAction(iws.ws, kStart);
		iws.currState = InteractiveWitnessState<width, height>::kInPoint;
		return false;
	}

	if (iws.ws.path.size() > 0 && iws.ws.path.back().second > height) // at goal, may not be solution
	{
		iws.currState = InteractiveWitnessState<width, height>::kWaitingRestart;
		return true;
	}
	
	// quit path
	iws.Reset();
	return false;
	
	

}

template <int width, int height>
void Witness<width, height>::Move(Graphics::point mouseLoc, InteractiveWitnessState<width, height> &iws)
{
	if (iws.currState == InteractiveWitnessState<width, height>::kWaitingStart ||
		iws.currState == InteractiveWitnessState<width, height>::kWaitingRestart)
	{
		return;
	}

	if (iws.currState == InteractiveWitnessState<width, height>::kInPoint)
	{
		Graphics::point end = GetScreenCoord(iws.ws.path.back().first, iws.ws.path.back().second);

		float factor = 3;
		if (iws.ws.path.size() > 1)
			factor = 1;

		// check if we left the point
		if (!Graphics::PointInRect(mouseLoc, Graphics::rect(end, factor*lineWidth)))
		{
			
			// 1. Find closest successor to mouse
			// find where to add to path
			std::vector<WitnessAction> moves;
			GetMouseActions(iws.ws, moves);
			iws.target = iws.ws.path.back();
			WitnessAction a = moves[0];
			float dist = 1000;
			for (auto m : moves)
			{
				iws.target = iws.ws.path.back();
				ApplyAction(iws.target, m);
				Graphics::point p = GetScreenCoord(iws.target.first, iws.target.second);
				//std::cout << "Dist from " << mouseLoc << " to " << p << " is " << ((mouseLoc-p).length()) << " act: " << m << "\n";
				if ((mouseLoc-p).length() < dist)
				{
					dist = (mouseLoc-p).length();
					a = m;
				}
			}
			
			// 2. Add to target location
			iws.target = iws.ws.path.back();
			ApplyAction(iws.target, a);
			iws.targetAct = a;
			
			// 3. Change state
			//printf("Switched from kInPoint to kBetweenPoints\n");
			iws.currState = InteractiveWitnessState<width, height>::kBetweenPoints;
			iws.frac = 0;
			
			int len = static_cast<int>(iws.ws.path.size());

			if (len >= 2 && iws.target == iws.ws.path[len-2]) // going backwards
			{
				iws.target = iws.ws.path.back();
				InvertAction(iws.targetAct);
				iws.frac = 1.0;
				UndoAction(iws.ws, a);
				//printf("Switched from kInPoint to kBetweenPoints [backwards]\n");
				iws.currState = InteractiveWitnessState<width, height>::kBetweenPoints;
			}
		}
	
		return;
	}

	if (iws.currState == InteractiveWitnessState<width, height>::kBetweenPoints)
	{
		Graphics::point from = GetScreenCoord(iws.ws.path.back().first, iws.ws.path.back().second);
		Graphics::point to = GetScreenCoord(iws.target.first, iws.target.second);

		// check if we entered the end (legally)
		if (Graphics::PointInRect(mouseLoc, Graphics::rect(to, lineWidth)) && Legal(iws.ws, iws.targetAct))
		{
			iws.currState = InteractiveWitnessState<width, height>::kInPoint;
			ApplyAction(iws.ws, iws.targetAct);
			//printf("Switched from kBetweenPoints to kInPoint [1]\n");
		}
		else if (from.x == to.x && (mouseLoc.y-from.y)/(to.y-from.y) > 0.9 && Legal(iws.ws, iws.targetAct)) // tracking in y
		{
			iws.currState = InteractiveWitnessState<width, height>::kInPoint;
			ApplyAction(iws.ws, iws.targetAct);
			//printf("Switched from kBetweenPoints to kInPoint [2]\n");
		}
		else if (from.y == to.y && (mouseLoc.x-from.x)/(to.x-from.x) > 0.9 && Legal(iws.ws, iws.targetAct)) // tracking in y
		{
			iws.currState = InteractiveWitnessState<width, height>::kInPoint;
			ApplyAction(iws.ws, iws.targetAct);
			//printf("Switched from kBetweenPoints to kInPoint [3]\n");
		}
		// entered start
		else if (Graphics::PointInRect(mouseLoc, Graphics::rect(from, lineWidth)))
		{
			iws.currState = InteractiveWitnessState<width, height>::kInPoint;
			//printf("Switched from kBetweenPoints to kInPoint [backtrack:1]\n");
		}
		else if (from.x == to.x && (mouseLoc.y-from.y)/(to.y-from.y) < 0.1) // tracking in y
		{
			iws.currState = InteractiveWitnessState<width, height>::kInPoint;
			//printf("Switched from kBetweenPoints to kInPoint [backtrack:2]\n");
		}
		else if (from.y == to.y && (mouseLoc.x-from.x)/(to.x-from.x) < 0.1) // tracking in y
		{
			iws.currState = InteractiveWitnessState<width, height>::kInPoint;
			//printf("Switched from kBetweenPoints to kInPoint [backtrack:3]\n");
		}
		
		// If still tracking, update track distance
		if (iws.currState == InteractiveWitnessState<width, height>::kBetweenPoints)
		{
			if (from.x == to.x) // tracking in y
				iws.frac = (mouseLoc.y-from.y)/(to.y-from.y);
			else
				iws.frac = (mouseLoc.x-from.x)/(to.x-from.x);

			if (iws.frac > 1)
				iws.frac = 1;
			if (!Legal(iws.ws, iws.targetAct))
			{
				if ((iws.target == std::pair<int, int>(0,0)) && iws.frac > 0.6f)
					iws.frac = 0.6f;
				else if (iws.frac > 0.8f)
					iws.frac = 0.8f;
			}
			//				if (!Legal(iws.ws, iws.targetAct) && iws.frac > 0.8)
			//					iws.frac = 0.8;
			if (iws.frac < 0)
				iws.frac = 0;
		}
	}
}


template <int width, int height>
void Witness<width, height>::Draw(Graphics::Display &display)
{
	// Background drawing
	{
		Graphics::point p1 = GetScreenCoord(0, 0);
		Graphics::point p2 = GetScreenCoord(width, height);
		
		display.FillRect({-1, -1, 1, 1}, outerBackColor);
		display.FillRect({p1.x, p2.y, p2.x, p1.y}, backColor);
	}

	for (int x = 0; x <= width; x++)
	{
		DoLine(display, GetScreenCoord(x, 0), GetScreenCoord(x, height), lineColor);
		//display.FillRect({(-1+gapOffset*x-lineWidth)*scale, -1*scale, (-1+gapOffset*x+lineWidth)*scale, 1*scale}, Colors::lightgray);
	}
	for (int y = 0; y <= height; y++)
	{
		DoLine(display, GetScreenCoord(0, y), GetScreenCoord(width, y), lineColor);
		//display.FillRect({-1*scale, (-1+gapOffset*y-lineWidth)*scale, 1*scale, (-1+gapOffset*y+lineWidth)*scale}, Colors::lightgray);
	}
	
	// corners -- all in case we move the start/goal
	display.FillCircle(GetScreenCoord(0, height), lineWidth, lineColor);
	display.FillCircle(GetScreenCoord(width, 0), lineWidth, lineColor);
	display.FillCircle(GetScreenCoord(0, 0), lineWidth, lineColor);
	display.FillCircle(GetScreenCoord(width, height), lineWidth, lineColor);

	// start
	display.FillCircle(GetScreenCoord(start[0].first, start[0].second), lineWidth*3.f, lineColor);

	// end
	DoLine(display, GetScreenCoord(width, height), GetScreenCoord(width+1, height+1), lineColor);
	display.FillCircle(GetScreenCoord(width+1, height+1), lineWidth, lineColor);

	// must-cross constraints (polygons)
	for (auto &c : mustCrossEdgeConstraints)
	{
		Graphics::point p1 = GetScreenCoord(c.location.first, c.location.second);
		Graphics::point p2 = GetScreenCoord(c.location.first+(c.horiz?1:0), c.location.second+(c.horiz?0:1));
		Graphics::point pt = (p1+p2)*0.5;
		display.FillNGon(pt,  lineWidth*0.9f, 6, 30, Colors::black);
		//display.FillRect(Graphics::rect(pt, lineWidth*0.5), Colors::black);
	}
	for (auto &c : mustCrossConstraints)
	{
		Graphics::point pt = GetScreenCoord(c.first, c.second);
		display.FillNGon(pt,  lineWidth*0.9f, 6, 30, Colors::black);
		//display.FillRect(Graphics::rect(pt, lineWidth*0.5), Colors::black);
	}

	for (int x = 0; x < width; x++)
	{
		for (int y = 0; y < height; y++)
		{
			switch(constraints[x][y].t)
			{
				case kNone: break;
				case kSeparation:
				{
					Graphics::point p1 = GetScreenCoord(x, y);
					Graphics::point p2 = GetScreenCoord(x+1, y+1);
					Graphics::point p3 = (p1+p2)*0.5;
					Graphics::point delta = {lineWidth, lineWidth};
					display.FillCircle(p3+delta, lineWidth, constraints[x][y].c);
					display.FillCircle(p3-delta, lineWidth, constraints[x][y].c);
					delta.x = -delta.x;
					display.FillCircle(p3+delta, lineWidth, constraints[x][y].c);
					display.FillCircle(p3-delta, lineWidth, constraints[x][y].c);
					display.FillRect({p3.x-1.0f*lineWidth, p3.y-2.0f*lineWidth, p3.x+1.0f*lineWidth, p3.y+2.0f*lineWidth},
									 constraints[x][y].c);
					display.FillRect({p3.x-2.0f*lineWidth, p3.y-1.0f*lineWidth, p3.x+2.0f*lineWidth, p3.y+1.0f*lineWidth},
									 constraints[x][y].c);
				}
					break;
				case kStar:
				{
					Graphics::point p1 = GetScreenCoord(x, y);
					Graphics::point p2 = GetScreenCoord(x+1, y+1);
					Graphics::point p3 = (p1+p2)*0.5;
					
					display.FillNGon(p3, gapOffset/5.0f, 4, 0, constraints[x][y].c);
					display.FillNGon(p3, gapOffset/5.0f, 4, 45, constraints[x][y].c);

				}
					break;
				case kTetris:
				case kNegativeTetris:
				{
					int whichPiece = abs(constraints[x][y].parameter);
					bool negative = constraints[x][y].t == kNegativeTetris;//constraints[x][y].parameter<0;
					if (whichPiece != 0)
					{
						float xOff = 0;
						float yOff = 0;
						switch (whichPiece)
						{
							case 1: xOff = 1.5; yOff = 1.5; break;
							case 2: xOff = 1; yOff = 1.5; break;
							case 3: xOff = 1.5; yOff = 1; break;
							case 4:
							case 5:
							case 6:
							case 7:
							case 8: xOff = 0.5; yOff = 1.5; break;
							case 9: xOff = 1.5; yOff = 0.5; break;
							case 10:
								xOff = 1; yOff = 1; break;
							case 11:
							case 12:
							case 13:
							case 14:
								xOff = 0.5; yOff = 1; break;
							case 15:
							case 16:
							case 17:
							case 18:
								xOff = 1; yOff = 0.5; break;
							case 19: xOff = 0; yOff = 1.5; break;
							case 20: xOff = 1.5; yOff = 0; break;
							case 21:
							case 22:
								xOff = 0.5; yOff = 1; break;
							case 23:
							case 24:
								xOff = 1.0; yOff = 0.5; break;
						}
						Graphics::point p1 = GetScreenCoord(x, y);
						Graphics::point p2 = GetScreenCoord(x+1, y+1);
						Graphics::point p3 = (p1+p2)*0.5;
						float blockSize = gapOffset/8.0f;
						for (int yy = 0; yy < 4; yy++)
						{
							for (int xx = 0; xx < 4; xx++)
							{
								if ((tetrisBits[whichPiece]>>(yy*4+xx))&1)
								{
									
									Graphics::point p4(-2*blockSize+xx*blockSize+0.5f*blockSize-xOff*blockSize,
													   -2*blockSize+yy*blockSize+0.5f*blockSize-yOff*blockSize);
									Graphics::rect r((p3-p4), blockSize*0.35f);
									if (negative)
									{
										display.FillRect(r, tetrisBlue);
										Graphics::rect inner((p3-p4), blockSize*0.35f*0.55f);
										display.FillRect(inner, backColor);
									}
									else
										display.FillRect(r, tetrisYellow);
								}
							}
						}
					}
				}
					break;
				case kTriangle:
				{
					Graphics::point p1 = GetScreenCoord(x, y);
					Graphics::point p2 = GetScreenCoord(x+1, y+1);
					Graphics::point p3 = (p1+p2)*0.5;
					
					switch (constraints[x][y].parameter)
					{
						case 1:
						{
							display.FillNGon(p3,  lineWidth*0.9f, 3, 60, Colors::orange);
							break;
						}
						case 2:
						{
							p3.x -= lineWidth;
							display.FillNGon(p3,  lineWidth*0.9f, 3, 60, Colors::orange);
							p3.x += 2*lineWidth;
							display.FillNGon(p3,  lineWidth*0.9f, 3, 60, Colors::orange);
							break;
						}
						case 3:
						{
							display.FillNGon(p3,  lineWidth*0.9f, 3, 60, Colors::orange);
							p3.x -= 2*lineWidth;
							display.FillNGon(p3,  lineWidth*0.9f, 3, 60, Colors::orange);
							p3.x += 4*lineWidth;
							display.FillNGon(p3,  lineWidth*0.9f, 3, 60, Colors::orange);
							
							break;
						}
					}
				}
					break;
				case kEraser: break;
				case kConstraintCount: break;

			}
		}
	}
	
	
//	// triangle constraints
//	for (int y = 0; y < height; y++)
//	{
//		for (int x = 0; x < width; x++)
//		{
//			if (triangleConstraints[y*width+x] > 0)
//			{
//				Graphics::point p1 = GetScreenCoord(x, y);
//				Graphics::point p2 = GetScreenCoord(x+1, y+1);
//				Graphics::point p3 = (p1+p2)*0.5;
//
//				switch (triangleConstraints[y*width+x])
//				{
//					case 1:
//					{
//						display.FillNGon(p3,  lineWidth*0.9f, 3, 60, Colors::orange);
//						break;
//					}
//					case 2:
//					{
//						p3.x -= lineWidth;
//						display.FillNGon(p3,  lineWidth*0.9f, 3, 60, Colors::orange);
//						p3.x += 2*lineWidth;
//						display.FillNGon(p3,  lineWidth*0.9f, 3, 60, Colors::orange);
//						break;
//					}
//					case 3:
//					{
//						display.FillNGon(p3,  lineWidth*0.9f, 3, 60, Colors::orange);
//						p3.x -= 2*lineWidth;
//						display.FillNGon(p3,  lineWidth*0.9f, 3, 60, Colors::orange);
//						p3.x += 4*lineWidth;
//						display.FillNGon(p3,  lineWidth*0.9f, 3, 60, Colors::orange);
//
//						break;
//					}
//				}
//			}
//		}
//	}
	
//	// separation constraints (rounded squares - maybe stars eventually)
//	for (int y = 0; y < height; y++)
//	{
//		for (int x = 0; x < width; x++)
//		{
//			if (separationConstraints[y*width+x].valid)
//			{
//				Graphics::point p1 = GetScreenCoord(x, y);
//				Graphics::point p2 = GetScreenCoord(x+1, y+1);
//				Graphics::point p3 = (p1+p2)*0.5;
//				Graphics::point delta = {lineWidth, lineWidth};
//				display.FillCircle(p3+delta, lineWidth, separationConstraints[y*width+x].color);
//				display.FillCircle(p3-delta, lineWidth, separationConstraints[y*width+x].color);
//				delta.x = -delta.x;
//				display.FillCircle(p3+delta, lineWidth, separationConstraints[y*width+x].color);
//				display.FillCircle(p3-delta, lineWidth, separationConstraints[y*width+x].color);
//				display.FillRect({p3.x-1.0f*lineWidth, p3.y-2.0f*lineWidth, p3.x+1.0f*lineWidth, p3.y+2.0f*lineWidth},
//								 separationConstraints[y*width+x].color);
//				display.FillRect({p3.x-2.0f*lineWidth, p3.y-1.0f*lineWidth, p3.x+2.0f*lineWidth, p3.y+1.0f*lineWidth},
//								 separationConstraints[y*width+x].color);
//			}
//		}
//	}

	// tetris constraints
//	for (int y = 0; y < height; y++)
//	{
//		for (int x = 0; x < width; x++)
//		{
//			int whichPiece = abs(tetrisConstraints[y*width+x]);
//			bool negative = tetrisConstraints[y*width+x]<0;
//			if (whichPiece != 0)
//			{
//				float xOff = 0;
//				float yOff = 0;
//				switch (whichPiece)
//				{
//					case 1: xOff = 1.5; yOff = 1.5; break;
//					case 2: xOff = 1; yOff = 1.5; break;
//					case 3: xOff = 1.5; yOff = 1; break;
//					case 4:
//					case 5:
//					case 6:
//					case 7:
//					case 8: xOff = 0.5; yOff = 1.5; break;
//					case 9: xOff = 1.5; yOff = 0.5; break;
//					case 10:
//						xOff = 1; yOff = 1; break;
//					case 11:
//					case 12:
//					case 13:
//					case 14:
//						xOff = 0.5; yOff = 1; break;
//					case 15:
//					case 16:
//					case 17:
//					case 18:
//						xOff = 1; yOff = 0.5; break;
//					case 19: xOff = 0; yOff = 1.5; break;
//					case 20: xOff = 1.5; yOff = 0; break;
//					case 21:
//					case 22:
//						xOff = 0.5; yOff = 1; break;
//					case 23:
//					case 24:
//						xOff = 1.0; yOff = 0.5; break;
//				}
//				Graphics::point p1 = GetScreenCoord(x, y);
//				Graphics::point p2 = GetScreenCoord(x+1, y+1);
//				Graphics::point p3 = (p1+p2)*0.5;
//				float blockSize = gapOffset/8.0f;
//				for (int yy = 0; yy < 4; yy++)
//				{
//					for (int xx = 0; xx < 4; xx++)
//					{
//						if ((tetrisBits[whichPiece]>>(yy*4+xx))&1)
//						{
//
//							Graphics::point p4(-2*blockSize+xx*blockSize+0.5f*blockSize-xOff*blockSize,
//											   -2*blockSize+yy*blockSize+0.5f*blockSize-yOff*blockSize);
//							Graphics::rect r((p3-p4), blockSize*0.35f);
//							if (negative)
//							{
//								display.FillRect(r, tetrisBlue);
//								Graphics::rect inner((p3-p4), blockSize*0.35f*0.55f);
//								display.FillRect(inner, backColor);
//							}
//							else
//								display.FillRect(r, tetrisYellow);
//						}
////						else {
////							Graphics::point p4(-2*blockSize+xx*blockSize+0.5*blockSize-xOff*blockSize,
////											   -2*blockSize+yy*blockSize+0.5*blockSize-yOff*blockSize);
////							Graphics::rect r((p3-p4), blockSize*0.45f);
////							display.FillRect(r, Colors::blue);
////						}
//					}
//				}
//			}
//		}
//	}
}

template <int width, int height>
void Witness<width, height>::Draw(Graphics::Display &display, const WitnessState<width, height>&s) const
{
	if (s.path.size() == 0)
	{
		return;
	}
	display.FillCircle(GetScreenCoord(start[0].first, start[0].second), lineWidth*3.f, drawColor);
	for (int x = 1; x < s.path.size(); x++)
	{
		Graphics::point p1 = GetScreenCoord(s.path[x-1].first, s.path[x-1].second);
		Graphics::point p2 = GetScreenCoord(s.path[x].first, s.path[x].second);

		DoLine(display, p1, p2, drawColor);

		if (x > 1)
		{
			display.FillCircle(p1, lineWidth, drawColor);
		}
		display.FillCircle(p2, lineWidth, drawColor);
	}
}


template <int width, int height>
void Witness<width, height>::Draw(Graphics::Display &display, const InteractiveWitnessState<width, height>&iws) const
{
	//	float scale = 0.8f;
	
	// draw animated start marker
	if (iws.currState == InteractiveWitnessState<width, height>::kWaitingStart)
	{
		assert((iws.ws.path.size() == 0));
		if (iws.frac <= 1)
			display.FrameCircle(GetScreenCoord(start[0].first, start[0].second), lineWidth*3.f*iws.frac, drawColor, lineWidth*0.5f);
		return;
	}

	// draw main state
	Draw(display, iws.ws);
	
	if (iws.currState == InteractiveWitnessState<width, height>::kBetweenPoints)
	{
		// draw last fraction
		Graphics::point p1 = GetScreenCoord(iws.ws.path.back().first, iws.ws.path.back().second);
		Graphics::point p2 = GetScreenCoord(iws.target.first, iws.target.second);
		p2 = p1+(p2-p1)*iws.frac;
		DoLine(display, p1, p2, drawColor);
		display.FillCircle(p2, lineWidth, drawColor);
	}
}

#endif /* Witness_hpp */
