/*
 *  GraphEnvironment.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/29/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef GRAPHENVIRONMENT_H
#define GRAPHENVIRONMENT_H

#include <stdint.h>
#include <iostream>
#include "SearchEnvironment.h"
#include "UnitSimulation.h"
#include "Graph.h"

typedef unsigned long graphState;

class graphMove {
public:
	graphMove() :from(-1), to(-1) {}
	graphMove(uint16_t f, uint16_t t) :from(f), to(t) {}
	uint16_t from, to;
};

namespace GraphSearchConstants
{
	enum {
		kHCost = 0,
		kXCoordinate = 1,
		kYCoordinate = 2,
		kZCoordinate = 3
	};
}

class GraphEnvironment : public SearchEnvironment<graphState, graphMove> {
public:
	GraphEnvironment(Graph *g);
	~GraphEnvironment();
	void GetSuccessors(graphState &stateID, std::vector<graphState> &neighbors);
	void GetActions(graphState &stateID, std::vector<graphMove> &actions);
	graphMove GetAction(graphState &s1, graphState &s2);
	void ApplyAction(graphState &s, graphMove a);
	bool InvertAction(graphMove &a);

	OccupancyInterface<graphState, graphMove> *GetOccupancyInfo() { return 0; }
	double HCost(graphState &state1, graphState &state2);
	double GCost(graphState &state1, graphState &state2);
	bool GoalTest(graphState &state, graphState &goal);
	uint64_t GetStateHash(graphState &state);
	uint64_t GetActionHash(graphMove act);
	void OpenGLDraw(int window);
	void OpenGLDraw(int window, graphState &s);
private:
	Graph *g;
};

typedef UnitSimulation<graphState, graphMove, GraphEnvironment> GraphSimulation;

#endif
