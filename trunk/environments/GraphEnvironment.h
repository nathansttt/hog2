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
#include "GraphAbstraction.h"
#include "GLUtil.h"

#ifndef UINT32_MAX
#define UINT32_MAX        4294967295U
#endif
#ifndef UINT16_MAX
#define UINT16_MAX 65535
#endif

typedef unsigned long graphState;

class graphMove {
public:
	graphMove() :from(UINT16_MAX), to(UINT16_MAX) {}
	graphMove(uint16_t f, uint16_t t) :from(f), to(t) {}
	uint16_t from, to;
};




namespace GraphSearchConstants
{
	enum {
		kHCost = 0, // this is relative to a single goal
		kXCoordinate = 5,
		kYCoordinate = 6,
		kZCoordinate = 7,
		kTemporaryLabel = 8,
		kMapX = 9,
		kMapY = 10,
		kFirstData = 11
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
	virtual Graph *GetGraph() = 0;
	virtual double HCost(graphState &state1, graphState &state2) = 0;
	// if one is better as the start or goal state, this can swap for you.
	virtual void ChooseStartGoal(graphState &start, graphState &goal) {}
	virtual void OpenGLDraw() const {}
private:
};

// this class uses the label on a graph for a heuristic
// but the heuristic is only available for a single goal node
class GraphLabelHeuristic : public GraphHeuristic {
public:
	GraphLabelHeuristic(Graph *graph, graphState target)
	{ g = graph; goal = target; }
	Graph *GetGraph() { return g; }
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
	Graph *GetGraph() { return g; }
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

class GraphMapPerfectHeuristic : public GraphHeuristic {
public:
	GraphMapPerfectHeuristic(Map *map, Graph *graph):m(map), g(graph)
	{
		prob = 0.5;
		fillProbTable();
	}
	Graph *GetGraph() { return g; }
	void SetProbability(double p) { prob = p; }
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
	double prob;
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

class GraphDistanceHeuristic : public GraphHeuristic {
public:
	GraphDistanceHeuristic(Graph *graph) :g(graph) { smartPlacement = false; }
	~GraphDistanceHeuristic() {}
	virtual double HCost(graphState &state1, graphState &state2) = 0;
	void AddHeuristic(node *n = 0);
	int GetNumHeuristics() { return heuristics.size(); }
	void UseSmartPlacement(bool use) { smartPlacement = use; }
	Graph *GetGraph() { return g; }
	void ChooseStartGoal(graphState &start, graphState &goal);
	void OpenGLDraw() const;
protected:
	void GetOptimalDistances(node *n, std::vector<double> &values);
	void AddHeuristic(std::vector<double> &values, graphState location);
	node *FindFarNode(node *n);

	bool smartPlacement;
	Graph *g;
	std::vector<std::vector<double> > heuristics;
	std::vector<graphState> locations;
};

enum tHeuristicCombination
{
	kIgnore = 0, // don't combine with database
	kRandom = 1, // combine random selection of databases
	kMax = 2,    // take max of all heuristics
	kGridMax = 3 // 0 on non-grid points
};

class GraphMapInconsistentHeuristic : public GraphDistanceHeuristic {
public:
	GraphMapInconsistentHeuristic(Map *map, Graph *graph);
	double HCost(graphState &state1, graphState &state2);
	void SetMode(tHeuristicCombination mode) { hmode = mode; }
	void SetNumUsedHeuristics(int count) { numHeuristics = count; }
private:
	tHeuristicCombination hmode;
	int numHeuristics;
	Map *m;
};

class GraphEnvironment : public SearchEnvironment<graphState, graphMove> {
public:
	GraphEnvironment(Graph *g, GraphHeuristic *gh);
	virtual ~GraphEnvironment();
	virtual void GetSuccessors(graphState &stateID, std::vector<graphState> &neighbors) const;
	virtual void GetActions(graphState &stateID, std::vector<graphMove> &actions) const;
	virtual graphMove GetAction(graphState &s1, graphState &s2) const;
	virtual void ApplyAction(graphState &s, graphMove a) const;
	virtual bool InvertAction(graphMove &a) const;

	void SetDirected(bool b) {directed = b;}

	OccupancyInterface<graphState, graphMove> *GetOccupancyInfo() { return 0; }
	virtual double HCost(graphState &state1, graphState &state2);
	virtual double GCost(graphState &state1, graphState &state2);
	virtual double GCost(graphState &state1, graphMove &state2);
	virtual bool GoalTest(graphState &state, graphState &goal);
	virtual uint64_t GetStateHash(graphState &state) const;
	virtual uint64_t GetActionHash(graphMove act) const;
	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const graphState &s) const;
	virtual void OpenGLDraw(const graphState &s, const graphMove &gm) const;
	Graph *GetGraph() { return g; };

	virtual void StoreGoal(graphState &state1) {}
	virtual void ClearGoal() {}
	virtual bool IsGoalStored() {return false;}

	virtual double HCost(graphState &state1) {
		fprintf(stderr, "ERROR: Single State HCost not implemented for RoboticArm\n");
		exit(1); return -1.0;}

	virtual bool GoalTest(graphState &s){
		fprintf(stderr, "ERROR: Single State Goal Test not implemented for GraphEnvironment\n");
		exit(1); return false;}

protected:
	bool directed;
	Graph *g;
	GraphHeuristic *h;

};

class AbstractionGraphEnvironment: public GraphEnvironment {
	public:
	AbstractionGraphEnvironment( GraphAbstraction *gabs, unsigned int level, GraphHeuristic *gh );
	~AbstractionGraphEnvironment();

	virtual void OpenGLDraw() const;
	double scale() { return graphscale; };

	protected:
	GraphAbstraction *gabs;
	double graphscale;
};

typedef UnitSimulation<graphState, graphMove, GraphEnvironment> GraphSimulation;

#endif
