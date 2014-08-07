/*
 *  ConfigEnvironment.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 1/9/09.
 *  Copyright 2009 NS Software. All rights reserved.
 *
 */

#include "GLUtil.h"
#include "SearchEnvironment.h"
#include <stdlib.h>
#include <stdio.h>

class ConfigEnvironment : public SearchEnvironment<recVec, line2d>
{
public:
	ConfigEnvironment();
	virtual ~ConfigEnvironment();

	void AddObstacle(line2d obs) { obstacles.push_back(obs); }
	void PopObstacle() { obstacles.pop_back(); }
	void GetSuccessors(const recVec &nodeID, std::vector<recVec> &neighbors) const;
	void GetActions(const recVec &nodeID, std::vector<line2d> &actions) const;
	line2d GetAction(const recVec &s1, const recVec &s2) const;
	virtual void ApplyAction(recVec &s, line2d dir) const;

	virtual bool InvertAction(line2d &a) const;

	virtual double HCost(const recVec &){
		printf("Single State HCost Failure: method not implemented for ConfigEnvironment\n");
		exit(0); return -1.0;}

	virtual double HCost(const recVec &node1, const recVec &node2);
	virtual double GCost(const recVec &node1, const recVec &node2);
	virtual double GCost(const recVec &node1, const line2d &act);
	bool GoalTest(const recVec &node, const recVec &goal);

	bool GoalTest(const recVec &){
		printf("Single State Goal Test Failure: method not implemented for ConfigEnvironment\n");
		exit(0); return false;}

	uint64_t GetStateHash(const recVec &node) const;
	uint64_t GetActionHash(line2d act) const;

	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const recVec &l) const;
	virtual void OpenGLDraw(const recVec &, const line2d &) const;
	virtual void OpenGLDraw(const recVec&, const recVec&, float) const {}

	virtual void GetNextState(const recVec &currents, line2d dir, recVec &news) const;

	void StoreGoal(recVec &g) { goal = g; goal_stored = true;} // stores the locations for the given goal state
	void ClearGoal() {goal_stored = false;}
	bool IsGoalStored() {return false;}


//	bool LegalState(line2d &a);
//	bool LegalArmConfig(line2d &a);
private:
	bool Legal(const recVec &a, const recVec &b) const;
	recVec goal;
	void DrawLine(line2d l) const;
	std::vector<line2d> obstacles;

	bool goal_stored;
};
