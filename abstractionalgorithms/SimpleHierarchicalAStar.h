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
//
//class HierarchicalHeuristicEnvironment : public SearchEnvironment<unsigned long, unsigned long>
//{
//	virtual ~HierarchicalHeuristicEnvironment() {}
//	virtual void GetSuccessors(const unsigned long &nodeID, std::vector<unsigned long> &neighbors) const = 0;
//	virtual void GetActions(const unsigned long &nodeID, std::vector<unsigned long> &actions) const = 0;
//	virtual unsigned long GetAction(unsigned long &s1, unsigned long &s2) const = 0;
//	virtual void ApplyAction(unsigned long &s, unsigned long a) const = 0;
//
//	virtual void GetNextState(unsigned long &currents, unsigned long dir, unsigned long &news) const{};
//
//	virtual bool InvertAction(unsigned long &a) const = 0;
//
//	virtual double HCost(const unsigned long &node1, const unsigned long &node2) = 0;
//
//	virtual double HCost(const unsigned long &state1){
//		fprintf(stderr, "ERROR: Single State HCost not implemented for HierarchicalHeuristicEnvironment\n");
//		exit(1); return -1.0;}
//
//	virtual double GCost(const unsigned long &node1, const unsigned long &node2) = 0;
//	virtual bool GoalTest(const  unsigned long &node, const  unsigned long &goal) = 0;
//
//	virtual bool GoalTest(const  unsigned long &s){
//		fprintf(stderr, "ERROR: Single State Goal Test not implemented for HierarchicalHeuristicEnvironment\n");
//		exit(1); return false;}
//
//	virtual uint64_t GetStateHash(const unsigned long &node) const = 0;
//	virtual uint64_t GetActionHash(unsigned long act) const = 0;
//
//	virtual OccupancyInterface<unsigned long,unsigned long> *GetOccupancyInfo() = 0;
//
//	virtual void OpenGLDraw() const = 0;
//	virtual void OpenGLDraw(unsigned long&) const = 0;
//	virtual void OpenGLDraw(unsigned long&, unsigned long&) const = 0;
//
//	void StoreGoal(unsigned long &s){}
//	void ClearGoal(){}
//	bool IsGoalStored(){return false;}
//};
//
//class SimpleHierarchicalAStar : public SearchAlgorithm {
//public:
//	SimpleHierarchicalAStar();
//	virtual ~SimpleHierarchicalAStar();
//	virtual const char *GetName() { return "SimpleHierarchicalAStar"; }
//	virtual path *GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);
//	path *DoOneSearchStep();
//	bool InitializeSearch(GraphAbstraction *aMap, node *from, node *to);
//	void OpenGLDraw() const;
//private:
//TemplateAStar<unsigned long, unsigned long,
//};

#endif