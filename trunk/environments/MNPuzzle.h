/*
 *  MNPuzzle.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/9/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef MNPUZZLE_H
#define MNPUZZLE_H

#include <stdint.h>
#include <iostream>
#include "SearchEnvironment.h"
#include "UnitSimulation.h"

class MNPuzzleState {
public:
	MNPuzzleState() { width = height = -1; }
	MNPuzzleState(int _width, int _height)
	:width(_width), height(_height)
  {
		puzzle.resize(width*height);
		for (unsigned int x = 0; x < puzzle.size(); x++)
			puzzle[x] = x;
		blank = 0;
	}
	int width, height;
	int blank;
	std::vector<int> puzzle;
};

enum slideDir {
	kLeft, kUp, kDown, kRight
};

static std::ostream& operator <<(std::ostream & out, const MNPuzzleState &loc)
{
	out << "(" << loc.width << "x" << loc.height << ")";
	for (int x = 0; x < loc.puzzle.size(); x++)
		out << loc.puzzle[x] << " ";
	return out;
}

static bool operator==(const MNPuzzleState &l1, const MNPuzzleState &l2)
{
	if (l1.width != l2.width)
		return false;
	if (l1.height != l2.height)
		return false;
	for (int x = 0; x < l1.puzzle.size(); x++)
		if (l1.puzzle[x] != l2.puzzle[x])
			return false;
	return true;
}

class MNPuzzle : SearchEnvironment<MNPuzzleState, slideDir> {
public:
	MNPuzzle(int width, int height);
	~MNPuzzle();
	void GetSuccessors(MNPuzzleState &stateID, std::vector<MNPuzzleState> &neighbors);
	void GetActions(MNPuzzleState &stateID, std::vector<slideDir> &actions);
	slideDir GetAction(MNPuzzleState &s1, MNPuzzleState &s2);
	void ApplyAction(MNPuzzleState &s, slideDir a);
	OccupancyInterface<MNPuzzleState, slideDir> *GetOccupancyInfo() { return 0; }
	double HCost(MNPuzzleState &state1, MNPuzzleState &state2);
	double GCost(MNPuzzleState &state1, MNPuzzleState &state2);
	bool GoalTest(MNPuzzleState &state, MNPuzzleState &goal);
	uint64_t GetStateHash(MNPuzzleState &state);
	uint64_t GetActionHash(slideDir act);
	void OpenGLDraw(int window);
	void OpenGLDraw(int window, MNPuzzleState &s);
private:
		int width, height;
};

typedef UnitSimulation<MNPuzzleState, slideDir, MNPuzzle> PuzzleSimulation;

#endif
