/*
 *  SimpleHierarchicalAStar.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/9/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef SIMPLEHIERARCHICALASTAR_H
#define SIMPLEHIERARCHICALASTAR_H

#include "TemplateAStar.h"
#include "SearchEnvironment.h"

class HierarchicalHeuristicEnvironment : public SearchEnvironment<unsigned long, unsigned long>
{
	virtual ~HierarchicalHeuristicEnvironment() {}
	virtual void GetSuccessors(unsigned long &nodeID, std::vector<unsigned long> &neighbors) = 0;
	virtual void GetActions(unsigned long &nodeID, std::vector<unsigned long> &actions) = 0;
	virtual unsigned long GetAction(unsigned long &s1, unsigned long &s2) = 0;
	virtual void ApplyAction(unsigned long &s, unsigned long a) = 0;

	virtual void GetNextState(unsigned long &currents, unsigned long dir, unsigned long &news){};

	virtual bool InvertAction(unsigned long &a) = 0;

	virtual double HCost(unsigned long &node1, unsigned long &node2) = 0;

	virtual double HCost(unsigned long &state1){
		printf("Single State HCost Failure: method not implemented for HierarchicalHeuristicEnvironment\n");
		exit(0); return -1.0;}

	virtual double GCost(unsigned long &node1, unsigned long &node2) = 0;
	virtual bool GoalTest(unsigned long &node, unsigned long &goal) = 0;

	virtual bool GoalTest(unsigned long &s){
		printf("Single State Goal Test Failure: method not implemented for HierarchicalHeuristicEnvironment\n");
		exit(0); return false;}

	virtual uint64_t GetStateHash(unsigned long &node) = 0;
	virtual uint64_t GetActionHash(unsigned long act) = 0;

	virtual OccupancyInterface<unsigned long,unsigned long> *GetOccupancyInfo() = 0;

	virtual void OpenGLDraw(int window) = 0;
	virtual void OpenGLDraw(int window, unsigned long&) = 0;
	virtual void OpenGLDraw(int window, unsigned long&, unsigned long&) = 0;

	void StoreGoal(unsigned long &s){}
	void ClearGoal(){}
	bool IsGoalStored(){return false;}
};

class SimpleHierarchicalAStar : public SearchAlgorithm {
public:
	SimpleHierarchicalAStar();
	virtual ~SimpleHierarchicalAStar();
	virtual const char *GetName() { return "SimpleHierarchicalAStar"; }
	virtual path *GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);
	path *DoOneSearchStep();
	bool InitializeSearch(GraphAbstraction *aMap, node *from, node *to);
	void OpenGLDraw();
private:
TemplateAStar<unsigned long, unsigned long,
};

#endif