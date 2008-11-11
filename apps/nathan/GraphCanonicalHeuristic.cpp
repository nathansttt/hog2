/*
 *  GraphCanonicalHeuristic.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/9/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "GraphCanonicalHeuristic.h"
#include "TemplateAStar.h"

GraphCanonicalHeuristic::GraphCanonicalHeuristic(Map *m, int DBSize)
:lengths(), centers(), centerDist(), msa(0)
{
	sectorSize = DBSize;
	ma = new MapEnvironment(m->clone());
	BuildPDB(m);
}

GraphCanonicalHeuristic::~GraphCanonicalHeuristic()
{
	delete msa;
}

Graph *GraphCanonicalHeuristic::GetGraph()
{
	if (msa == 0)
		return 0;
	return msa->GetAbstractGraph(0);
}

Map *GraphCanonicalHeuristic::GetMap()
{
	if (msa == 0)
		return 0;
	return msa->GetMap();
}

double GraphCanonicalHeuristic::HCost(graphState &state1, graphState &state2)
{
	Graph *g = msa->GetAbstractGraph(0);
	node *n1 = g->GetNode(state1);
	node *n2 = g->GetNode(state2);
	node *p1 = msa->GetParent(n1);
	node *p2 = msa->GetParent(n2);
	
	double val = lengths[p1->GetNum()][p2->GetNum()] - centerDist[n1->GetNum()] - centerDist[n2->GetNum()];
	int x1, x2, y1, y2;
	msa->GetTileFromNode(n1, x1, y1);
	msa->GetTileFromNode(n2, x2, y2);
	xyLoc one(x1, y1);
	xyLoc two(x2, y2);
	val = max(val, ma->HCost(one, two));
	// compare with h-function...
	return val;
}


void GraphCanonicalHeuristic::BuildPDB(Map *map)
{
	// get map size
	msa = new MapQuadTreeAbstraction(map, sectorSize);
	msa->ToggleDrawAbstraction(0);
	msa->ToggleDrawAbstraction(1);
	
	printf("Getting centers\n");
	GetCenters();
	//TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
	
	// for each center, do a BFS and find the length to all other centers
	printf("Solving for values\n");
	GetPDBValues();
	
	ComputerCenterDistances();
}

void GraphCanonicalHeuristic::ComputerCenterDistances()
{
	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
	//MapEnvironment ma(msa->GetMap()->clone());
	
	Graph *g = msa->GetAbstractGraph(0);
	centerDist.resize(g->GetNumNodes());
	for (int count = 0; count < g->GetNumNodes(); count++)
	{
		int locx, locy;
		node *n = g->GetNode(count);
		
		msa->GetTileFromNode(msa->GetParent(n), locx, locy);
		xyLoc loc1(locx, locy);
		msa->GetTileFromNode(n, locx, locy);
		xyLoc loc2(locx, locy);
		
		std::vector<xyLoc> path;
		astar.GetPath(ma, loc1, loc2, path);
		double len = 0.0;
		for (unsigned int z = 1; z < path.size(); z++)
			len += ma->GCost(path[z-1], path[z]);
		printf("Local Results = %f (length %d) [h=%f]\n", len, (int)path.size(), ma->HCost(loc1, loc2));
		
		centerDist[n->GetNum()] = len;
	}
}

void GraphCanonicalHeuristic::GetCenters()
{
	Graph *g = msa->GetAbstractGraph(1);
	// for every high-level node
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		// find closest low-level node
		int locx, locy;
		msa->GetTileFromNode(g->GetNode(x), locx, locy);
		centers.push_back(msa->GetNodeFromMap(locx, locy)->GetNum());
	}
}

void GraphCanonicalHeuristic::GetPDBValues()
{
	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
	astar.SetStopAfterGoal(false);

	Graph *g = msa->GetAbstractGraph(0);
	lengths.resize(centers.size());
	for (unsigned int x = 0; x < centers.size(); x++)
	{
		lengths[x].resize(centers.size());
		lengths[x][x] = 0;
		for (unsigned int y = x+1; y < centers.size(); y++)
			lengths[y].resize(centers.size());
	}
	for (unsigned int x = 0; x < centers.size(); x++)
	{
		printf("Solving from [%d] (%d)\n", x, centers[x]);
		int locx, locy;
		msa->GetTileFromNode(g->GetNode(centers[x]), locx, locy);
		xyLoc loc1(locx, locy);
		xyLoc locNone(0xFFFF, 0xFFFF);

		std::vector<xyLoc> path;
		astar.GetPath(ma, loc1, locNone, path);

		for (unsigned int y = x+1; y < centers.size(); y++)
		{
			msa->GetTileFromNode(g->GetNode(centers[y]), locx, locy);
			xyLoc loc2(locx, locy);

			double len = -1;
			astar.GetClosedListGCost(loc2, len);
			printf("(%d, %d)<->(%d, %d) = %f\n", loc1.x, loc1.y, loc2.x, loc2.y, len);
			lengths[x][y] = len;
			lengths[y][x] = len;
		}
	}
}


