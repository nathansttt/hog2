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
#include <assert.h>
#include "OccupancyInterface.h"
#include "GLUtil.h"


struct Hash64 {
		size_t operator()(const uint64_t &x) const
		{ return (size_t)(x); }
};

template <class state, class action>
class SearchEnvironment {
public:
	virtual ~SearchEnvironment() {}
	virtual void GetSuccessors(state &nodeID, std::vector<state> &neighbors) const = 0;
	virtual void GetActions(state &nodeID, std::vector<action> &actions) const = 0;

	virtual action GetAction(state &s1, state &s2) const = 0;
	virtual void ApplyAction(state &s, action a) const = 0;
	virtual void UndoAction(state &s, action a) const
	{ assert(InvertAction(a)); ApplyAction(s, a); }

	virtual void GetNextState(state &, action , state &) const { assert(false); };

	virtual bool InvertAction(action &a) const = 0;

	/** Stores the goal for use by single-state HCost. **/
	virtual void StoreGoal(state &s)
	{ bValidSearchGoal = true; searchGoal = s; }

	/** Clears the goal from memory. **/
	virtual void ClearGoal()
	{ bValidSearchGoal = false; }

	/** Returns true if the goal is stored and false otherwise. **/
	virtual bool IsGoalStored()
	{ return bValidSearchGoal; }

	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(state &node1, state &node2) = 0;

	/** Heuristic value between node and the stored goal. Asserts that the
	 goal is stored **/
	virtual double HCost(state &node)
	{ assert(bValidSearchGoal); return HCost(node, searchGoal); }

	virtual double GCost(state &node1, state &node2) = 0;
	virtual double GCost(state &node, action &act) = 0;
	virtual bool GoalTest(state &node, state &goal) = 0;

	/** Goal Test if the goal is stored **/
	virtual bool GoalTest(state &node)
	{ return bValidSearchGoal&&(node == searchGoal); }

	virtual uint64_t GetStateHash(state &node) const = 0;
	virtual uint64_t GetActionHash(action act) const = 0;

	virtual double GetPathLength(std::vector<state> &neighbors);

	virtual OccupancyInterface<state,action> *GetOccupancyInfo()
	{ return 0; }

	virtual void OpenGLDraw() const = 0;
	virtual void OpenGLDraw(const state&) const = 0;
	/** Draw the transition at some percentage 0...1 between two states */
	virtual void OpenGLDraw(const state&, const state&, float) const {}
	virtual void OpenGLDraw(const state&, const action&) const = 0;

	virtual void SetColor(GLfloat rr, GLfloat g, GLfloat b, GLfloat t) const { color.r = rr; color.g = g; color.b = b; transparency = t; }
	virtual void GetColor(GLfloat& rr, GLfloat& g, GLfloat& b, GLfloat &t) const { rr=color.r; g=color.g; b=color.b; t = transparency;}
private:
	bool bValidSearchGoal;
	state searchGoal;
	mutable recColor color;
	mutable GLfloat transparency;
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
