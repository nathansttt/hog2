/*
 *  GraphCanonicalHeuristic.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/9/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef GRAPHCANONICALHEURISTIC_H
#define GRAPHCANONICALHEURISTIC_H

#include "GraphEnvironment.h"
#include "MapQuadTreeAbstraction.h"
#include "Map2DEnvironment.h"
#include <vector>

// pure virtual class
class GraphCanonicalHeuristic : public GraphHeuristic {
public:
	GraphCanonicalHeuristic(Map *m, int DBSize = 16);
	~GraphCanonicalHeuristic();
	Graph *GetGraph();
	Map *GetMap();
	double HCost(graphState &state1, graphState &state2);
private:
	void BuildPDB(Map *map);
	void GetCenters();
	void GetPDBValues();
	void ComputerCenterDistances();
	
	std::vector<std::vector<double> > lengths;
	std::vector<int> centers;
	std::vector<double> centerDist;
	MapQuadTreeAbstraction *msa;
	MapEnvironment *ma;
	int sectorSize;
};

#endif
