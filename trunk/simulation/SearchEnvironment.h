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

template <class state, class action>
class SearchEnvironment {
public:
	virtual ~SearchEnvironment() {}
	virtual void GetSuccessors(state &nodeID, std::vector<state> &neighbors) = 0;
	virtual void GetActions(state &nodeID, std::vector<action> &actions) = 0;
	virtual action GetAction(state &s1, state &s2) = 0;
	virtual void ApplyAction(state &s, action a) = 0;

	virtual double HCost(state &node1, state &node2) = 0;
	virtual double GCost(state &node1, state &node2) = 0;
	virtual bool GoalTest(state &node, state &goal) = 0;

	virtual uint64_t GetStateHash(state &node) = 0;
	virtual uint64_t GetActionHash(action act) = 0;

	virtual OccupancyInterface<state, action> *GetOccupancyInfo() = 0;

	virtual void OpenGLDraw(int window) = 0;
	virtual void OpenGLDraw(int window, state&) = 0;
};

#endif
