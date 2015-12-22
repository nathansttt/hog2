/*
 * $Id: sample.cpp,v 1.23 2006/11/01 23:33:56 nathanst Exp $
 *
 *  sample.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Copyright 2005 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include "Common.h"
#include "Sample.h"
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include "Map2DEnvironment.h"
#include "RandomUnits.h"
#include "AStar.h"
#include "TemplateAStar.h"
#include "GraphEnvironment.h"
#include "MapSectorAbstraction.h"
#include "GraphRefinementEnvironment.h"
#include "ScenarioLoader.h"
#include "BFS.h"
#include "PEAStar.h"
#include "EPEAStar.h"
#include "MapGenerators.h"
#include "FPUtil.h"
#include "CanonicalGrid.h"

bool screenShot = false;
bool recording = false;
bool running = false;
std::vector<graphState> thePath;

void LoadGraph();

GraphDistanceHeuristic *gdh = 0;
GraphEnvironment *ge = 0;

TemplateAStar<graphState, graphMove, GraphEnvironment> astar;                                                                                                                                                                                                               

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1000, 1000);
}


/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
void CreateSimulation(int id)
{
	LoadGraph();
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record the screen.", kNoModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Reset Rotations", "Reset the current rotation/translation of the map.", kAnyModifier, '|');

	InstallKeyboardHandler(MyPathfindingKeyHandler, "", "", kNoModifier, 'd');

	
	
	InstallCommandLineHandler(MyCLHandler, "-makeRoom", "-makeRoom x-dim y-dim roomSie filename", "Resizes map to specified dimensions and saves");

	InstallWindowHandler(MyWindowHandler);
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
		
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		LoadGraph();
//		CreateSimulation(windowID);
		SetNumPorts(windowID, 1);
		//SetZoom(windowID, 10);
	}

}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (ge)
	{
		ge->OpenGLDraw();

		if (running)
		{
			running = !astar.DoSingleSearchStep(thePath);
			astar.OpenGLDraw();
		}
	}

	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		static int cnt = 0;
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/%d%d%d%d", (cnt/1000)%10, (cnt/100)%10, (cnt/10)%10, cnt%10);
		SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
		cnt++;
	}
}


int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp( argument[0], "-makeRoom" ) == 0 )
	{
		if (maxNumArgs <= 4)
			return 0;
		return maxNumArgs;
	}
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '|': resetCamera(); break;
		case 'r': recording = !recording; break;
		default:
			break;
	}
}

void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	printf("Starting Search\n");
	running = true;
	node *n1 = ge->GetGraph()->GetRandomNode();
	node *n2 = ge->GetGraph()->GetRandomNode();
	astar.InitializeSearch(ge, n1->GetNum(), n2->GetNum(), thePath);
	
}

void LoadGraph()
{
	Graph *g = new Graph();
	node *n = new node("");
	g->AddNode(n);
	FILE *f = fopen("/Users/nathanst/Downloads/USA-road-d.COL.co", "r");
	std::vector<double> xloc, yloc;
	double minx = DBL_MAX, maxx=-DBL_MAX, miny=DBL_MAX, maxy=-DBL_MAX;
	while (!feof(f))
	{
		char line[255];
		fgets(line, 255, f);
		if (line[0] == 'v')
		{
			float x1, y1;
			int id;
			if (3 != sscanf(line, "v %d %f %f", &id, &x1, &y1))
				continue;
			//assert(id == xloc.size()+1);
			xloc.push_back(x1);
			yloc.push_back(y1);
			//printf("%d: (%f, %f) [%f, %f]\n", xloc.size(), x1, y1, minx, maxx);
			if (x1 > maxx) maxx = x1;
			if (x1 < minx) minx = x1;
			if (y1 > maxy) maxy = y1;
			if (y1 < miny) miny = y1;
			//if (maxx > -1)
		}
	}
	fclose(f); 
	printf("x between (%f, %f), y between (%f, %f)\n",
		   minx, maxx, miny, maxy);
	double scale = std::max(maxx-minx,maxy-miny);
	for (unsigned int x = 0; x < xloc.size(); x++)
	{
		//printf("(%f, %f) -> ", xloc[x], yloc[x]);
		xloc[x] -= (minx);
		xloc[x] /= scale;
		xloc[x] = xloc[x]*2-1;

		yloc[x] -= (miny);
		yloc[x] /= scale;
		yloc[x] = yloc[x]*2-1;
		yloc[x] = -yloc[x];
		
		node *n = new node("");
		g->AddNode(n);
		n->SetLabelF(GraphSearchConstants::kXCoordinate, xloc[x]);
		n->SetLabelF(GraphSearchConstants::kYCoordinate, yloc[x]);
		n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
		
		//printf("(%f, %f)\n", xloc[x], yloc[x]);
	}
	//a 1 2 1988
	f = fopen("/Users/nathanst/Downloads/USA-road-d.COL.gr", "r");
	while (!feof(f))
	{
		char line[255];
		fgets(line, 255, f);
		if (line[0] == 'a')
		{
			int x1, y1;
			sscanf(line, "a %d %d %*d", &x1, &y1);
			g->AddEdge(new edge(x1, y1, 1.0));
			//printf("%d to %d\n", x1, y1);
		}
	}
	fclose(f); 
	ge = new GraphEnvironment(g);
	ge->SetDirected(true);
}


//void runProblemSet3(char *scenario)
//{
//
//	TemplateAStar<graphState, graphMove, GraphEnvironment> astar;
//  astar.SetWeight(0); // Dijkstra's algorithm
//  astar.SetWeight(1); // A*
//  astar.SetHeuristic(??); // use a diff heuristic than the environment
//	std::vector<xyLoc> thePath;
//	MapEnvironment ma(map);
//	ma.SetFourConnected();
//	
//	for (int x = 0; x < sl.GetNumExperiments(); x++)
//	{
//		Timer t;
//		t.StartTimer();
//		astar.GetPath(&ma, from, to, thePath);
//		t.EndTimer();
//		printf("\tastar\t%ld\t%1.6f\t%llu\t%u\n", thePath.size(), t.GetElapsedTime(), astar.GetNodesExpanded(), astar.GetNumOpenItems());
//	}
//	
//	exit(0);
//}
