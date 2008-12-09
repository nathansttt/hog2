/*
 *  FloydWarshall.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 12/7/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "FloydWarshall.h"

//1 /* Assume a function edgeCost(i,j) which returns the cost of the edge from i to j
// 2    (infinity if there is none).
// 3    Also assume that n is the number of vertices and edgeCost(i,i)=0
// 4 */
//5
//6 int path[][];
//7 /* A 2-dimensional matrix. At each step in the algorithm, path[i][j] is the shortest path
// 8    from i to j using intermediate vertices (1..k-1).  Each path[i][j] is initialized to
// 9    edgeCost(i,j).
// 10 */
//11
void FloydWarshall(Graph *g, std::vector<std::vector<double> > &lengths)
{
	lengths.resize(0);
	lengths.resize(g->GetNumNodes());
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		lengths[x].resize(g->GetNumNodes());
		for (int i = 0; i < g->GetNumNodes(); i++)
			lengths[x][i] = 1e10;
	}

	for (int x = 0; x < g->GetNumEdges(); x++)
	{
		edge *e = g->GetEdge(x);
		lengths[e->getFrom()][e->getTo()] = e->GetWeight();
		lengths[e->getTo()][e->getFrom()] = e->GetWeight();
	}
	
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		for (int i = 0; i < g->GetNumNodes(); i++)
		{
			for (int j = 0; j < g->GetNumNodes(); j++)
			{
				lengths[i][j] = std::min(lengths[i][j], lengths[i][x]+lengths[x][j]);
			}
		}
	}
}

