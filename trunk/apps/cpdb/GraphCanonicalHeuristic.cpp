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
:lengths(), centers(), centerDist(), map(m), g(0), mo(0)
{
//	if (DBSize == -1)
//	{
//		DBSize = (int)pow(m->getMapWidth()*m->getMapHeight(), 0.25)/2;
//		printf("Dynamically setting size to %d\n", DBSize);
//	}
	sectorSize = DBSize;
	ma = new MapEnvironment(m->clone());
	BuildPDB();
}

GraphCanonicalHeuristic::GraphCanonicalHeuristic(char *file)
{
	FILE *f = fopen(file, "r");
	if (!f)
		return;

	unsigned centersSize, centerDistSize, lengthsSize;
	fscanf(f, "%u %u %u %u\n", &sectorSize, &centersSize, &centerDistSize, &lengthsSize);
	map = new Map(f);
	ma = new MapEnvironment(map->clone());
	g = GraphSearchConstants::GetGraph(map);
//	msa = new MapQuadTreeAbstraction(m, sectorSize);
//	msa->ToggleDrawAbstraction(0);
//	msa->ToggleDrawAbstraction(1);
	
	//std::vector<int> centers;
	centers.resize(0);
	for (unsigned int x = 0; x < centersSize; x++)
	{
		int value;
		fscanf(f, "%d ", &value);
		centers.push_back(value);
	}
	
	//std::vector<double> centerDist;
	centerDist.resize(0);
	for (unsigned int x = 0; x < centerDistSize; x++)
	{
		float value;
		fscanf(f, "%f", &value);
		centerDist.push_back(value);
	}

	//std::vector<double> centerDist;
	whichPDB.resize(0);
	for (unsigned int x = 0; x < centerDistSize; x++)
	{
		int value;
		fscanf(f, "%d", &value);
		whichPDB.push_back(value);
	}
	
	lengths.resize(lengthsSize);
	//std::vector<std::vector<double> > lengths;
	for (unsigned int x = 0; x < lengthsSize; x++)
	{
		lengths[x].resize(lengthsSize);
		for (unsigned int y = 0; y < lengthsSize; y++)
		{
			float value;
			fscanf(f, "%f", &value);
			lengths[x][y] = value;
		}
	}
	fclose(f);

	mo  = new MapOverlay(map);
	mo->setTransparentValue(0);
	for (int x = 0; x < map->getMapWidth(); x++)
	{
		for (int y = 0; y < map->getMapHeight(); y++)
		{
			node *n = g->GetNode(map->getNodeNum(x, y));
			if (n)
			{
				mo->setOverlayValue(x, y, centerDist[n->GetNum()]);
				//printf("(%d, %d) center distances: %f\n", x, y, centerDist[n->GetNum()]);
			}
			else
				mo->setOverlayValue(x, y, 0);
		}
	}
}

void GraphCanonicalHeuristic::save(char *file)
{
	FILE *f = fopen(file, "w+");
	if (!f)
		return;
	fprintf(f, "%d %d %d %d\n", sectorSize, (int)centers.size(), (int)centerDist.size(), (int)lengths.size());
	ma->GetMap()->save(f);

	//std::vector<int> centers;
	for (unsigned int x = 0; x < centers.size(); x++)
		fprintf(f, "%d ", centers[x]);
	fprintf(f, "\n");

	//std::vector<double> centerDist;
	for (unsigned int x = 0; x < centerDist.size(); x++)
		fprintf(f, "%f ", centerDist[x]);
	fprintf(f, "\n");

	for (unsigned int x = 0; x < whichPDB.size(); x++)
		fprintf(f, "%d ", whichPDB[x]);
	
	//std::vector<std::vector<double> > lengths;
	for (unsigned int x = 0; x < lengths.size(); x++)
	{
		for (unsigned int y = 0; y < lengths[x].size(); y++)
		{
			fprintf(f, "%f ", lengths[x][y]);
		}
		fprintf(f, "\n");
	}
	fprintf(f, "\n");
	fclose(f);
}

GraphCanonicalHeuristic::~GraphCanonicalHeuristic()
{
	delete map;
	delete g;
	delete mo;
	//delete msa;
}

Graph *GraphCanonicalHeuristic::GetGraph()
{
	return g;
}

Map *GraphCanonicalHeuristic::GetMap()
{
	return map;
}

double GraphCanonicalHeuristic::HCost(graphState &state1, graphState &state2)
{
	node *n1 = g->GetNode(state1);
	node *n2 = g->GetNode(state2);
	int p1 = whichPDB[n1->GetNum()];
	int p2 = whichPDB[n2->GetNum()];
	
	double val = lengths[p1][p2] - centerDist[p1] - centerDist[p2];
	int x1, x2, y1, y2;
	x1 = n1->GetLabelL(GraphSearchConstants::kMapX);
	y1 = n1->GetLabelL(GraphSearchConstants::kMapY);
	x2 = n2->GetLabelL(GraphSearchConstants::kMapX);
	y2 = n2->GetLabelL(GraphSearchConstants::kMapY);
//	msa->GetTileFromNode(n1, x1, y1);
//	msa->GetTileFromNode(n2, x2, y2);
	xyLoc one(x1, y1);
	xyLoc two(x2, y2);
	val = max(val, ma->HCost(one, two));
	// compare with h-function...
	return val;
}


void GraphCanonicalHeuristic::BuildPDB()
{
//	if (sectorSize == -1)
//	{
		g = GraphSearchConstants::GetGraph(map);
		//DBSize = pow(g->GetNumNodes(), 0.25);
	//sectorSize = (int)pow(g->GetNumNodes(), 0.25)/2;
	//delete g;
//	}
	
	// get map size
//	msa = new MapQuadTreeAbstraction(map, sectorSize);
//
//	int cnt1 = msa->GetAbstractGraph(0)->GetNumNodes();
//	int cnt2 = msa->GetAbstractGraph(1)->GetNumNodes();
//	// we want cnt2^2 = 2*cnt1
//	
//	msa->ToggleDrawAbstraction(0);
//	msa->ToggleDrawAbstraction(1);
//	printf("Sector size %d, map nodes %d, abstract nodes %d\n",
//		   sectorSize, msa->GetAbstractGraph(0)->GetNumNodes(),
//		   msa->GetAbstractGraph(1)->GetNumNodes());
	
	printf("Getting centers\n");
	GetCenters();
	//TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
	
	// for each center, do a BFS and find the length to all other centers
	printf("Solving for values\n");
	GetPDBValues();
	
	ComputerCenterDistances();
	mo  = new MapOverlay(map);
	mo->setTransparentValue(0);
	for (int x = 0; x < map->getMapWidth(); x++)
	{
		for (int y = 0; y < map->getMapHeight(); y++)
		{
			node *n = g->GetNode(map->getNodeNum(x, y));
			if (n)
			{
				mo->setOverlayValue(x, y, centerDist[n->GetNum()]);
				//printf("(%d, %d) center distances: %f\n", x, y, centerDist[n->GetNum()]);
			}
			else
				mo->setOverlayValue(x, y, 0);
		}
	}
}

void GraphCanonicalHeuristic::ComputerCenterDistances()
{
//	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
//	//MapEnvironment ma(msa->GetMap()->clone());
//	
//	//Graph *g = msa->GetAbstractGraph(0);
//	centerDist.resize(g->GetNumNodes());
//	for (int count = 0; count < g->GetNumNodes(); count++)
//	{
//		int locx, locy;
//		node *n = g->GetNode(count);
//		
//		msa->GetTileFromNode(msa->GetParent(n), locx, locy);
//		xyLoc loc1(locx, locy);
//		msa->GetTileFromNode(n, locx, locy);
//		xyLoc loc2(locx, locy);
//		
//		std::vector<xyLoc> path;
//		astar.GetPath(ma, loc1, loc2, path);
//		double len = 0.0;
//		for (unsigned int z = 1; z < path.size(); z++)
//			len += ma->GCost(path[z-1], path[z]);
//		//printf("Local Results = %f (length %d) [h=%f]\n", len, (int)path.size(), ma->HCost(loc1, loc2));
//		
//		centerDist[n->GetNum()] = len;
//	}
}

void GraphCanonicalHeuristic::GetCenters()
{
	int limit = sectorSize;
	if (limit == -1)
		limit = 2*sqrt(g->GetNumNodes());
	while (centers.size() < limit)
	{
		int val = g->GetRandomNode()->GetNum();
		for (unsigned int x = 0; x < centers.size(); x++)
			if ((centers[x] == val) || (g->FindEdge(val, centers[x])))
				val = -1;
		if (val != -1)
			centers.push_back(val);
	}
	//	Graph *g = msa->GetAbstractGraph(1);
//	// for every high-level node
//	for (int x = 0; x < g->GetNumNodes(); x++)
//	{
//		// find closest low-level node
//		int locx, locy;
//		msa->GetTileFromNode(g->GetNode(x), locx, locy);
//		centers.push_back(msa->GetNodeFromMap(locx, locy)->GetNum());
//	}
}

void GraphCanonicalHeuristic::GetPDBValues()
{
	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
	astar.SetStopAfterGoal(false);

	centerDist.resize(g->GetNumNodes());
	whichPDB.resize(g->GetNumNodes());
	for (unsigned int x = 0; x < whichPDB.size(); x++)
		centerDist[x] = 0x7FFFFFFF;
	
	//Graph *g = msa->GetAbstractGraph(0);
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
		printf("Solving from [%d of %d] (%d)\n", x, (int)centers.size(), centers[x]);
		int locx, locy;
		locx = g->GetNode(centers[x])->GetLabelL(GraphSearchConstants::kMapX);
		locy = g->GetNode(centers[x])->GetLabelL(GraphSearchConstants::kMapY);
//		msa->GetTileFromNode(g->GetNode(centers[x]), locx, locy);
		xyLoc loc1(locx, locy);
		xyLoc locNone(0xFFFF, 0xFFFF);

		std::vector<xyLoc> path;
		astar.GetPath(ma, loc1, locNone, path);

		// mark closest points
		double len;
		for (int y = 0; y < g->GetNumNodes(); y++)
		{
			locx = g->GetNode(y)->GetLabelL(GraphSearchConstants::kMapX);
			locy = g->GetNode(y)->GetLabelL(GraphSearchConstants::kMapY);
			xyLoc loc2(locx, locy);
			if (astar.GetClosedListGCost(loc2, len))
			{
				if (len < centerDist[y])
				{
					centerDist[y] = len;
					whichPDB[y] = x;
				}
			}
		}
		for (unsigned int y = x+1; y < centers.size(); y++)
		{
			locx = g->GetNode(centers[y])->GetLabelL(GraphSearchConstants::kMapX);
			locy = g->GetNode(centers[y])->GetLabelL(GraphSearchConstants::kMapY);
			//msa->GetTileFromNode(g->GetNode(centers[y]), locx, locy);
			xyLoc loc2(locx, locy);

			len = -1;
			if (astar.GetClosedListGCost(loc2, len))
			{ }
			//printf("(%d, %d)<->(%d, %d) = %f\n", loc1.x, loc1.y, loc2.x, loc2.y, len);
			lengths[x][y] = len;
			lengths[y][x] = len;
		}
	}
	for (unsigned int x = 0; x < whichPDB.size(); x++)
		if (centerDist[x] == 0x7FFFFFFF)
			centerDist[x] = 0;

}


void GraphCanonicalHeuristic::OpenGLDraw()
{
	map->OpenGLDraw(0, kPolygons);
	mo->OpenGLDraw(0);
}
