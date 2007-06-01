/*
 *  FringeSearch.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 1/19/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */

#include "SearchAlgorithm.h"

#ifndef FRINGESEARCH_H
#define FRINGESEARCH_H

struct costs
{
	node *n;
	double gCost, hCost;
};

typedef std::vector<node *> nodeList;
typedef std::vector<costs> costList;

class heuristicProvider {
public:
	virtual ~heuristicProvider() {}
	virtual double h(uint32_t node1, uint32_t node2) = 0;
};

class FringeSearch : public SearchAlgorithm {
public:
	FringeSearch();
	const char *GetName() { return bpmx?"BPMXFringeSearch":"FringeSearch"; }
	virtual path *GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);
	void setUseBPMX(bool use) { bpmx = use; }
	void setHeuristicProvider(heuristicProvider *hh) { hp = hh; }
	unsigned int getNodesReopened() { return nodesReopened; }
	unsigned int getHValuesPropagated() { return nodesHPropagated; }
private:
	void initializeSearch(GraphAbstraction *aGraph, node *from, node *to);
	bool onClosedList(node *n);
	bool onOpenList(node *n);
	void addToClosedList(node *n);
	void addToOpenList(node *n);
	void addToOpenList2(node *n);
	void moveToOpenList1(node *n);
	double getFCost(node *n);
	double getGCost(node *n);
	double getHCost(node *n);
	double h(node *n1, node *n2);
	void setHCost(node *n, double val);
	void addCosts(node *n, node *parent, edge *e);
	void getCosts(node *n, costs &val);
	void updateCosts(node *n, node *parent, edge *e);
	path *extractBestPath(node *n);
	void checkIteration();
	void propagateHValues(node *n, int dist = 10000);
	void propagateGValues(node *n);
	node *goal;
	Graph *g;
	GraphAbstraction *aMap;
	heuristicProvider *hp;
	nodeList list1, list2;
	nodeList *currList, *nextList;
	nodeList closedList;
	costList costTable;
	double currFLimit, nextFLimit;
	unsigned int nodesReopened, nodesHPropagated;
	bool bpmx;
};

#endif
