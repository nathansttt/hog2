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

class ConfigEnvironment : public SearchEnvironment<recVec, line2d>
{
public:
	ConfigEnvironment();
	virtual ~ConfigEnvironment();
	
	void AddObstacle(line2d obs) { obstacles.push_back(obs); }
	void GetSuccessors(recVec &nodeID, std::vector<recVec> &neighbors);
	void GetActions(recVec &nodeID, std::vector<line2d> &actions);
	line2d GetAction(recVec &s1, recVec &s2);
	virtual void ApplyAction(recVec &s, line2d dir);
	
	virtual bool InvertAction(line2d &a);
	
	virtual double HCost(recVec &node1, recVec &node2);
	virtual double GCost(recVec &node1, recVec &node2);
	virtual double GCost(recVec &node1, line2d &act);
	bool GoalTest(recVec &node, recVec &goal);
	uint64_t GetStateHash(recVec &node);
	uint64_t GetActionHash(line2d act);
	
	virtual void OpenGLDraw(int window);
	virtual void OpenGLDraw(int window, recVec &l);
	virtual void OpenGLDraw(int, recVec &, line2d &);
	virtual void OpenGLDraw(int, recVec &, line2d &, GLfloat r, GLfloat g, GLfloat b);
	virtual void OpenGLDraw(int, recVec &l, GLfloat r, GLfloat g, GLfloat b);
	
	virtual void GetNextState(recVec &currents, line2d dir, recVec &news);
	
	void StoreGoal(recVec &g) { goal = g; } // stores the locations for the given goal state
//	bool LegalState(line2d &a);
//	bool LegalArmConfig(line2d &a);
private:
	bool Legal(recVec &a, recVec &b);
	recVec goal;
	void DrawLine(line2d l);
	std::vector<line2d> obstacles;
};
