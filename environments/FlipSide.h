/*
 *  FlipSide.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/22/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef FLIPSIDE_H
#define FLIPSIDE_H

#include <stdint.h>
#include <iostream>
#include "SearchEnvironment.h"
#include "UnitSimulation.h"

class FlipSideState {
public:
	FlipSideState()
{ width = 5;
		puzzle.resize(width*2);
		for (unsigned int x = 0; x < puzzle.size(); x++)
			puzzle[x] = x;
}
	FlipSideState(int _width)
	:width(_width)
{
	puzzle.resize(width*2);
	for (unsigned int x = 0; x < puzzle.size(); x++)
		puzzle[x] = x;
}
unsigned int width;
std::vector<int> puzzle;
};

class flipMove {
public:
	flipMove() { top = 0; bottom = 0; }
	flipMove(int t, int b) :top(t), bottom(b) {}
	uint16_t top, bottom;
};

static std::ostream& operator <<(std::ostream & out, const FlipSideState &loc)
{
	for (unsigned int x = 0; x < loc.puzzle.size(); x++)
	{
		if (x == loc.puzzle.size()/2)
			out << std::endl;
		out << loc.puzzle[x] << " ";
	}
	return out;
}

static bool operator==(const FlipSideState &l1, const FlipSideState &l2)
{
	if (l1.width != l2.width)
		return false;
	for (unsigned int x = 0; x < l1.puzzle.size(); x++)
		if (l1.puzzle[x] != l2.puzzle[x])
			return false;
	return true;
}

class FlipSide : SearchEnvironment<FlipSideState, flipMove> {
public:
	FlipSide(int width = 5);
	~FlipSide();
	void GetSuccessors(FlipSideState &stateID, std::vector<FlipSideState> &neighbors);
	void GetActions(FlipSideState &stateID, std::vector<flipMove> &actions);
	flipMove GetAction(FlipSideState &s1, FlipSideState &s2);
	void ApplyAction(FlipSideState &s, flipMove a);
	OccupancyInterface<FlipSideState, flipMove> *GetOccupancyInfo() { return 0; }
	double HCost(FlipSideState &state1, FlipSideState &state2);
	double GCost(FlipSideState &state1, FlipSideState &state2);
	bool GoalTest(FlipSideState &state, FlipSideState &goal);
	uint64_t GetStateHash(FlipSideState &state);
	uint64_t GetActionHash(flipMove act);
	void OpenGLDraw(int window);
	void OpenGLDraw(int window, FlipSideState &s);
private:
		int width;
};

typedef UnitSimulation<FlipSideState, flipMove, FlipSide> FlipPuzzleSimulation;

#endif
