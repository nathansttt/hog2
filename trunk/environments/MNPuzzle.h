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
#include "UnitSimulation.h"


class MNPuzzleState {
	MNPuzzleState(int _width, int _height)
	:width(_width), height(_height)
  {
		puzzle.resize(width*height);
		for (unsigned int x = 0; x < puzzle.size(); x++)
			puzzle[x] = x;
	}
	int width, height;
	std::vector<int> puzzle;
};

enum slideDir {
	kLeft, kUp, kDown, kRight
};


class MNPuzzle : SearchEnvironment<MNPuzzleState, slideDir> {
public:
	MNPuzzle(int width, int height);
	~MNPuzzle();
	void GetSuccessors(MNPuzzleState stateID, std::vector<MNPuzzleState> &neighbors);
	void GetActions(MNPuzzleState stateID, std::vector<slideDir> &actions);
	slideDir GetAction(MNPuzzleState s1, MNPuzzleState s2);
	void ApplyAction(MNPuzzleState &s, slideDir a);
	OccupancyInterface<MNPuzzleState, slideDir> *GetOccupancyInfo() { return 0; }
	double HCost(MNPuzzleState state1, MNPuzzleState state2);
	double GCost(MNPuzzleState state1, MNPuzzleState state2);
	bool GoalTest(MNPuzzleState state, MNPuzzleState goal);
	uint64_t GetStateHash(MNPuzzleState state);
	uint64_t GetActionHash(slideDir act);
	void OpenGLDraw(int window);
private:
		int width, height;
};


#endif
