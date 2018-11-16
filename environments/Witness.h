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
	Witness() :separationConstraints(width*height), separationCount(0) {}
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
	void ClearMustCrossConstraint(int);
	void AddMustCrossConstraint(bool horiz, int x, int y) { mustCrossEdgeConstraints.push_back({horiz, {x, y}});}
	void AddMustCrossConstraint(int x, int y) { mustCrossConstraints.push_back({x, y});}
	void RemoveMustCrossConstraint(bool horiz, int x, int y) { mustCrossEdgeConstraints.pop_back();}
	void RemoveMustCrossConstraint(int x, int y) { mustCrossConstraints.pop_back();}
	
	/* Color (rounded) square - must separate these */
	/* TODO: Star constraints are a special case */
	void ClearSeparationConstraints() { separationConstraints.clear(); separationConstraints.resize(width*height), separationCount = 0; }
	constexpr int GetNumSeparationConstraints() const { return width*height; }
	void AddSeparationConstraint(int x, int y, rgbColor c) { auto &i = separationConstraints[y*width+x]; i.color = c; i.valid = true; separationCount++;}
	void AddSeparationConstraint(int which, rgbColor c) { auto &i = separationConstraints[which]; i.color = c; i.valid = true; separationCount++;}
	void RemoveSeparationConstraint(int x, int y) { separationConstraints[y*width+x].valid = false; separationCount--;}
	void RemoveSeparationConstraint(int which) { separationConstraints[which].valid = false; separationCount--;}

	// TODO: Not yet complete
	/* Tetris constraints - must solve packing problem to validate these */
	/* We allow most constraints with 1...4 blocks -- 14 total */
	/*  0  *
	 *  1  **
	 *  2  *
	 *     *
	 *  3  **
	 *     *
	 *  4  **
	 *      *
	 *  5  *
	 *     **
	 *  6   *
	 *     **
	 *  7  **
	 *     **
	 *  8  ***
	 *     *
	 *  9  *
	 *     ***
	 *  10 **
	 *     *
	 *     *
	 *  11 **
	 *      *
	 *      *
	 *  12 ****
	 *  13 *
	 *     *
	 *     *
	 *     *
	 */
	void ClearTetrisConstraints() {}
	constexpr int GetNumTetrisConstraints() const { return width*height; }
	void AddSeparationConstraint(int x, int y, int which) {}
	const uint16_t tetris[14] =
	{ 0x8000, 0xC000, 0x8800, 0xC800, 0xC400, 0x8C00, 0x4C00, 0xCC00, 0xE800, 0x8E00, 0xC880, 0xC440, 0xF000, 0x8888};
private:
	const float scale = 0.8f;
	const float gapOffset = scale*2.0f/(std::max(width, height));
	const float lineWidth = gapOffset*0.1f;
	const float xGap = (std::max(width, height)-width)*gapOffset/2.0;
	const float yGap = (std::max(width, height)-height)*gapOffset/2.0;
	const rgbColor offWhite = {0.95, 0.95, 0.95};
	
	// Must cross edge/node
	struct mustCrossEdgeConstraint {
		bool horiz;
		std::pair <int, int> location;
	};
	std::vector<mustCrossEdgeConstraint> mustCrossEdgeConstraints;
	std::vector<std::pair<int, int>> mustCrossConstraints;

	struct separationObject {
		separationObject() :valid(false), color(Colors::pink) {}
		bool valid;
		rgbColor color;
	};
	std::vector<separationObject> separationConstraints;
	int separationCount;
	
	void LabelRegions(const WitnessState<width, height> &s) const;
	mutable std::vector<int> regions;
	mutable std::vector<std::vector<int>> regionList;

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
			//s.path.push_back({0, 0});
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
			s.path.push_back({0, 0});
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

	LabelRegions(node);
	for (auto &v : regionList) // vector of locations
	{
		bool found = false;
		rgbColor c;
		for (auto &i : v)
		{
			if (separationConstraints[i].valid)
			{
				if (!found)
				{
					c = separationConstraints[i].color;
					found = true;
				}
				else if (c != separationConstraints[i].color)
				{
					return false;
				}
			}
		}
	}

	return true;
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
	if (separationCount == 0)
		return;
//	mutable std::vector<int> regions;
	regions.clear();
	regions.resize(width*height);
	regionList.clear();
	
	static std::vector<int> queue;
	queue.clear();
	
	int index = 0;
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			if (regions[y*width+x] == 0)
			{
				queue.push_back(y*width+x);
				index++;
				regionList.resize(index);
			}
			while (queue.size() > 0)
			{
				int next = queue.back();
				queue.pop_back();
				
				if (regions[next] == 0)
				{
					regions[next] = index;
					regionList[index-1].push_back(next);
					// check 4 directions
					int xx = next%width;
					int yy = next/width;
					if (xx > 0 && !s.OccupiedEdge(xx, yy, xx, yy+1))
						queue.push_back(yy*width+xx-1);
					if (xx < width-1 && !s.OccupiedEdge(xx+1, yy, xx+1, yy+1))
						queue.push_back(yy*width+xx+1);
					if (yy > 0 && !s.OccupiedEdge(xx, yy, xx+1, yy))
						queue.push_back((yy-1)*width+xx);
					if (yy < height-1 && !s.OccupiedEdge(xx, yy+1, xx+1, yy+1))
						queue.push_back((yy+1)*width+xx);
				}
			}
			
		}
	}

//	for (int y = 0; y < height; y++)
//	{
//		for (int x = 0; x < width; x++)
//		{
//			printf("%d", regions[y*width+x]);
//		}
//		printf("\n");
//	}
//	printf("\n\n");
}





template <int width, int height>
bool Witness<width, height>::Click(Graphics::point mouseLoc, InteractiveWitnessState<width, height> &iws)
{
	// in start area
	if ((iws.currState == InteractiveWitnessState<width, height>::kWaitingStart ||
		 iws.currState == InteractiveWitnessState<width, height>::kWaitingRestart) &&
		(mouseLoc-GetScreenCoord(0, 0)).length() < 3*lineWidth)
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
			WitnessAction a;
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
			
			int len = iws.ws.path.size();

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
				if ((iws.target == std::pair<int, int>(0,0)) && iws.frac > 0.6)
					iws.frac = 0.6;
				else if (iws.frac > 0.8)
					iws.frac = 0.8;
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
//	display.FillRect({-1, -1, 1, 1}, Colors::darkgray);

	for (int x = 0; x <= width; x++)
	{
		DoLine(display, GetScreenCoord(x, 0), GetScreenCoord(x, height), Colors::lightgray);
		//display.FillRect({(-1+gapOffset*x-lineWidth)*scale, -1*scale, (-1+gapOffset*x+lineWidth)*scale, 1*scale}, Colors::lightgray);
	}
	for (int y = 0; y <= height; y++)
	{
		DoLine(display, GetScreenCoord(0, y), GetScreenCoord(width, y), Colors::lightgray);
		//display.FillRect({-1*scale, (-1+gapOffset*y-lineWidth)*scale, 1*scale, (-1+gapOffset*y+lineWidth)*scale}, Colors::lightgray);
	}
	
	// corners
	display.FillCircle(GetScreenCoord(0, height), lineWidth, Colors::lightgray);
	display.FillCircle(GetScreenCoord(width, 0), lineWidth, Colors::lightgray);

	// start
	display.FillCircle(GetScreenCoord(0, 0), lineWidth*3.f, Colors::lightgray);

	// end
	DoLine(display, GetScreenCoord(width, height), GetScreenCoord(width+1, height+1), Colors::lightgray);
	display.FillCircle(GetScreenCoord(width+1, height+1), lineWidth, Colors::lightgray);

	for (auto &c : mustCrossEdgeConstraints)
	{
		Graphics::point p1 = GetScreenCoord(c.location.first, c.location.second);
		Graphics::point p2 = GetScreenCoord(c.location.first+(c.horiz?1:0), c.location.second+(c.horiz?0:1));
		Graphics::point pt = (p1+p2)*0.5;
		display.FillNGon(pt,  lineWidth*0.75, 6, 30, Colors::black);
		//display.FillRect(Graphics::rect(pt, lineWidth*0.5), Colors::black);
	}
	for (auto &c : mustCrossConstraints)
	{
		Graphics::point pt = GetScreenCoord(c.first, c.second);
		display.FillNGon(pt,  lineWidth*0.75, 6, 30, Colors::black);
		//display.FillRect(Graphics::rect(pt, lineWidth*0.5), Colors::black);
	}

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			if (separationConstraints[y*width+x].valid)
			{
				Graphics::point p1 = GetScreenCoord(x, y);
				Graphics::point p2 = GetScreenCoord(x+1, y+1);
				Graphics::point p3 = (p1+p2)*0.5;
				Graphics::point delta = {lineWidth, lineWidth};
				display.FillCircle(p3+delta, lineWidth, separationConstraints[y*width+x].color);
				display.FillCircle(p3-delta, lineWidth, separationConstraints[y*width+x].color);
				delta.x = -delta.x;
				display.FillCircle(p3+delta, lineWidth, separationConstraints[y*width+x].color);
				display.FillCircle(p3-delta, lineWidth, separationConstraints[y*width+x].color);
				display.FillRect({p3.x-1.0f*lineWidth, p3.y-2.0f*lineWidth, p3.x+1.0f*lineWidth, p3.y+2.0f*lineWidth},
								 separationConstraints[y*width+x].color);
				display.FillRect({p3.x-2.0f*lineWidth, p3.y-1.0f*lineWidth, p3.x+2.0f*lineWidth, p3.y+1.0f*lineWidth},
								 separationConstraints[y*width+x].color);
			}
		}
	}
}

template <int width, int height>
void Witness<width, height>::Draw(Graphics::Display &display, const WitnessState<width, height>&s) const
{
	if (s.path.size() == 0)
	{
		return;
	}
	auto color = offWhite;
//	if (GoalTest(s))
	color = Colors::white;
	
	display.FillCircle(GetScreenCoord(0, 0), lineWidth*3.f, color);
	for (int x = 1; x < s.path.size(); x++)
	{
		
		Graphics::point p1 = GetScreenCoord(s.path[x-1].first, s.path[x-1].second);
		Graphics::point p2 = GetScreenCoord(s.path[x].first, s.path[x].second);

		DoLine(display, p1, p2, color);

		if (x > 1)
		{
			display.FillCircle(p1, lineWidth, color);
		}
		display.FillCircle(p2, lineWidth, color);
	}

//	for (int x = 0; x <= width; x++)
//	{
//		for (int y = 0; y <= height; y++)
//		{
//			if (s.Occupied(x, y))
//				display.FillCircle(GetScreenCoord(x, y), lineWidth, Colors::red);
//		}
//	}
//	for (int x = 0; x < width; x++)
//	{
//		for (int y = 0; y < height; y++)
//		{
//			if (s.OccupiedEdge(x, y, x+1, y))
//			{
//				display.FillCircle((GetScreenCoord(x, y)+GetScreenCoord(x+1, y))*0.5f, lineWidth, Colors::green);
//			}
//			if (s.OccupiedEdge(x, y, x, y+1))
//			{
//				display.FillCircle((GetScreenCoord(x, y)+GetScreenCoord(x, y+1))*0.5f, lineWidth, Colors::green);
//			}
//		}
//	}
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
			display.FrameCircle(GetScreenCoord(0, 0), lineWidth*3.f*iws.frac, offWhite, lineWidth*0.5f);
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
		DoLine(display, p1, p2, offWhite);
		display.FillCircle(p2, lineWidth, offWhite);
	}
}

#endif /* Witness_hpp */
