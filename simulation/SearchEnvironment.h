/*
 *  SearchEnvironment.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/15/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef SEARCHENVIRONMENT_H
#define SEARCHENVIRONMENT_H

#include <stdint.h>
#include <vector>
//#include "ReservationProvider.h"
#include "OccupancyInterface.h"



struct Hash64 {
		size_t operator()(const uint64_t &x) const
		{ return (size_t)(x); }
};

template <class state, class action>
class SearchEnvironment {
public:
	virtual ~SearchEnvironment() {}
	virtual void GetSuccessors(state &nodeID, std::vector<state> &neighbors) = 0;
	virtual void GetActions(state &nodeID, std::vector<action> &actions) = 0;

	virtual action GetAction(state &s1, state &s2) = 0;
	virtual void ApplyAction(state &s, action a) = 0;

	virtual void GetNextState(state &currents, action dir, state &news){};

	virtual bool InvertAction(action &a) = 0;

	/** Stores the goal for use by single-state HCost. **/
	virtual void StoreGoal(state &s) = 0;

	/** Clears the goal from memory. **/
	virtual void ClearGoal() = 0;

	/** Returns true if the goal is stored and false otherwise. **/
	virtual bool IsGoalStored() = 0;

	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(state &node1, state &node2) = 0;

	/** Heuristic value between node and the stored goal. Asserts that the
	 goal is stored **/
	virtual double HCost(state &node) = 0;

	virtual double GCost(state &node1, state &node2) = 0;
	virtual double GCost(state &node, action &act) = 0;
	virtual bool GoalTest(state &node, state &goal) = 0;

	/** Goal Test if the goal is stored **/
	virtual bool GoalTest(state &node) = 0;

	virtual uint64_t GetStateHash(state &node) = 0;
	virtual uint64_t GetActionHash(action act) = 0;

	virtual double GetPathLength(std::vector<state> &neighbors);

	virtual OccupancyInterface<state,action> *GetOccupancyInfo() { return 0; }

	virtual void OpenGLDraw(int window) = 0;
	virtual void OpenGLDraw(int window, state&) = 0;
	virtual void OpenGLDraw(int window, state&, action&) = 0;

};

template <class state, class action>
double SearchEnvironment<state,action>::GetPathLength(std::vector<state> &neighbors)
{
	double length = 0;
	for (unsigned int x = 1; x < neighbors.size(); x++)
	{
		length += GCost(neighbors[x-1], neighbors[x]);
	}
	return length;
}

#endif
