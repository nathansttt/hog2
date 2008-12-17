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
#include <ext/hash_map>
#include <iostream>
#include "SearchEnvironment.h"
#include "UnitSimulation.h"
#include "Graph.h"
#include "GLUtil.h"

#ifndef UINT32_MAX
#define UINT32_MAX        4294967295U
#endif

typedef unsigned long graphState;

class graphMove {
public:
	graphMove() :from(UINT32_MAX), to(UINT32_MAX) {}
	graphMove(uint16_t f, uint16_t t) :from(f), to(t) {}
	uint16_t from, to;
};




namespace GraphSearchConstants
{
	enum {
		kXCoordinate = 0,
		kYCoordinate = 1,
		kZCoordinate = 2,
		kHCost = 3, // this is relative to a single goal
		kMapX = 4,
		kMapY = 5,
		kTemporaryLabel = 6
	};

	const double kStraightEdgeCost = 1.0;
	const double kDiagonalEdgeCost = ROOT_TWO;
	
	Graph *GetGraph(Map *m);
	void AddEdges(Map *m, Graph *g, int x, int y,
				  double straigtEdgeCost = 1.0,
				  double diagEdgeCost = ROOT_TWO,
				  int straightEdgeProb = 100,
				  int diagEdgeProb = 100);
}

// pure virtual class
class GraphHeuristic {
public:
	virtual ~GraphHeuristic() { }
	virtual double HCost(graphState &state1, graphState &state2) = 0;
private:
};

// this class uses the label on a graph for a heuristic
// but the heuristic is only available for a single goal node
class GraphLabelHeuristic : public GraphHeuristic {
public:
	GraphLabelHeuristic(Graph *graph, graphState target)
	{ g = graph; goal = target; }
	double HCost(graphState &state1, graphState &state2)
	{
		if (state2 == goal)
			return g->GetNode(state1)->GetLabelF(GraphSearchConstants::kHCost);
		return 0;
	}
private:
	graphState goal;
	Graph *g;
};

class GraphMapHeuristic : public GraphHeuristic {
public:
	GraphMapHeuristic(Map *map, Graph *graph)
	:m(map), g(graph) {}
	double HCost(graphState &state1, graphState &state2)
	{
		int x1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapX);
		int y1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapY);
		int x2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapX);
		int y2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapY);

		double a = ((x1>x2)?(x1-x2):(x2-x1));
		double b = ((y1>y2)?(y1-y2):(y2-y1));
		return (a>b)?(b*ROOT_TWO+a-b):(a*ROOT_TWO+b-a);
	}
private:
	Map *m;
	Graph *g;
};

class GraphMapInconsistentHeuristic : public GraphHeuristic {
public:
	GraphMapInconsistentHeuristic(Map *map, Graph *graph);
	double HCost(graphState &state1, graphState &state2);
	static int hmode;
	static int HN;
private:
	void GetOptimalDistances(node *n, std::vector<double> &values);
	void AddHeuristic(std::vector<double> &values, graphState location);
	Map *m;
	Graph *g;
	std::vector<std::vector<double> > heuristics;
	std::vector<graphState> locations;
};

class GraphMapPerfectHeuristic : public GraphHeuristic {
public:
	GraphMapPerfectHeuristic(Map *map, Graph *graph):m(map), g(graph)
	{
		fillProbTable();
	}
	double HCost(graphState &state1, graphState &state2)
	{ // warning: in this implementation HCost(s1,s2) != HCost(s2,s1)

		if(probTable[int(state1)]) {
			int x1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapX);
			int y1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapY);
			int x2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapX);
			int y2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapY);
			return GetOctileDistance(x1-x2, y1-y2);
		}
		else
			return 0;
	}
	~GraphMapPerfectHeuristic() 
	{
		delete probTable;
	}
	static double prob;
private:
	double GetOctileDistance(double dx, double dy) 
	{
		dx = fabs(dx);
		dy = fabs(dy);

		if(dx > dy)
			return dx-dy + sqrt(2)*dy;
		else
			return dy-dx + sqrt(2)*dx;
	}
	void fillProbTable() 
	{
		int size = m->getMapWidth() * m->getMapHeight();
		probTable = (bool*)malloc( size );
		for(int i=0;i<size;i++) {
			if(drand48() < prob)
				probTable[i] = 1;
			else
				probTable[i] = 0;
		}
	}
	Map *m;
	Graph *g;
	bool* probTable;
};

class GraphEnvironment : public SearchEnvironment<graphState, graphMove> {
public:
	GraphEnvironment(Graph *g, GraphHeuristic *gh);
	~GraphEnvironment();
	virtual void GetSuccessors(graphState &stateID, std::vector<graphState> &neighbors);
	virtual void GetActions(graphState &stateID, std::vector<graphMove> &actions);
	virtual graphMove GetAction(graphState &s1, graphState &s2);
	virtual void ApplyAction(graphState &s, graphMove a);
	virtual bool InvertAction(graphMove &a);

	void SetDirected(bool b) {directed = b;}

	OccupancyInterface<graphState, graphMove> *GetOccupancyInfo() { return 0; }
	virtual double HCost(graphState &state1, graphState &state2);
	virtual double GCost(graphState &state1, graphState &state2);
	virtual double GCost(graphState &state1, graphMove &state2);
	virtual bool GoalTest(graphState &state, graphState &goal);
	virtual uint64_t GetStateHash(graphState &state);
	virtual uint64_t GetActionHash(graphMove act);
	virtual void OpenGLDraw(int window);
	virtual void OpenGLDraw(int window, graphState &s);
	virtual void OpenGLDraw(int window, graphState &s, graphMove &gm);
protected:
	bool directed;
	Graph *g;
	GraphHeuristic *h;

};

typedef UnitSimulation<graphState, graphMove, GraphEnvironment> GraphSimulation;

#endif
