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
#include "MapOverlay.h"
#include <vector>

// pure virtual class
class GraphCanonicalHeuristic : public GraphHeuristic {
public:
	GraphCanonicalHeuristic(Map *m, int numClosest, int sizeFactor = 10);
	GraphCanonicalHeuristic(Graph *g, GraphHeuristic *gh, int numClosest, int sizeFactor = 10);
	GraphCanonicalHeuristic(char *file);
	~GraphCanonicalHeuristic();
	Graph *GetGraph();
	Map *GetMap();
	void load(char *file);
	void save(char *file);
	double HCost(graphState &state1, graphState &state2);
	void ChooseStartGoal(graphState &start, graphState &goal);

	void OpenGLDraw() const;
	int GetNumEntries() {return centerDist.size() + lengths.size() * (lengths.size()); }
private:
	void BuildPDB();
	void GetCenters();
	void GetPDBValues();
	void ComputerCenterDistances();
	
	std::vector<std::vector<float> > lengths;
	std::vector<int> centers;
	std::vector<std::vector<float> > centerDist;
	std::vector<std::vector<int> > whichPDB;
	//MapQuadTreeAbstraction *msa;
	Map *map;
	Graph *g;
	GraphEnvironment *ga;
	MapOverlay *mo;
	int m_SizeFactor;
	int m_NumClosest;
	int m_NumCanonicals;
};

#endif
