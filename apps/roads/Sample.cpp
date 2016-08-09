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
#include "MM.h"

bool screenShot = false;
bool recording = false;
bool running = false;
bool showSearch = false;

bool runningBidirectional = false;
bool showSearchBidirectional = false;

bool drawLine = false;
std::vector<graphState> thePath;
std::vector<graphState> thePath2;

point3d lineStart, lineEnd;

std::string graphFile, coordinatesFile;

void LoadGraph();

int search = 0;

GraphDistanceHeuristic *gdh = 0;
GraphEnvironment *ge = 0;

TemplateAStar<graphState, graphMove, GraphEnvironment> astar;
MM<graphState, graphMove, GraphEnvironment> mm;

uint32_t gStepsPerFrame = 1;
double distance(graphState n1, graphState n2);

class GraphDistHeuristic : public Heuristic<graphState> {
public:
	double HCost(const graphState &a, const graphState &b) const
	{
		return distance(a, b);
	}
};

GraphDistHeuristic h;
ZeroHeuristic<graphState> z;

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
	InstallKeyboardHandler(MyDisplayHandler, "Faster", "Simulate search faster.", kNoModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Slower", "Simulate search slower.", kNoModifier, ']');

	InstallKeyboardHandler(MyPathfindingKeyHandler, "", "", kNoModifier, 'd');

	
	
	InstallCommandLineHandler(MyCLHandler, "-graph", "-graph <filename>", "Specifies file name for graph. Both graph and coordinates must be supplied.");
	InstallCommandLineHandler(MyCLHandler, "-coord", "-coord <filename>", "Specifies file name for coordinates. Both graph and coordinates must be supplied.");

	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler);
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
		SetNumPorts(windowID, 1);
	}

}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (ge)
	{
		ge->SetColor(1.0, 1.0, 0.5);
		ge->OpenGLDraw();

		if (running)
		{
			//std::cout << "Expanding next: " << astar.CheckNextNode() << "\n";
			for (int x = 0; x < gStepsPerFrame && running; x++)
			{
				running = !astar.DoSingleSearchStep(thePath);
			}
			if (!running)
			{
				for (int x = 1; x < thePath.size(); x++)
				{
					ge->GetGraph()->findDirectedEdge(thePath[x-1], thePath[x])->setMarked(true);
				}
			}
		}
		if (showSearch)
		{
			astar.OpenGLDraw();
		}

		if (runningBidirectional)
		{
			//std::cout << "Expanding next: " << astar.CheckNextNode() << "\n";
			for (int x = 0; x < gStepsPerFrame && runningBidirectional; x++)
			{
				runningBidirectional = !mm.DoSingleSearchStep(thePath2);
			}
			if (!runningBidirectional)
			{
				for (int x = 1; x < thePath.size(); x++)
				{
					ge->GetGraph()->findDirectedEdge(thePath[x-1], thePath2[x])->setMarked(true);
				}
			}
		}
		if (showSearchBidirectional)
		{
			mm.OpenGLDraw();
		}

		if (drawLine)
		{
			glLineWidth(5);
			glColor4f(1, 1, 1, 0.9);
			glBegin(GL_LINES);
			glVertex3f(lineStart.x, lineStart.y, -0.01);
			glVertex3f(lineEnd.x, lineEnd.y, -0.01);
			glEnd();
			glLineWidth(1);
		}
	}

	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		static int cnt = 0;
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/graph-%d%d%d%d", (cnt/1000)%10, (cnt/100)%10, (cnt/10)%10, cnt%10);
		SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
		cnt++;
	}
}


int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp( argument[0], "-graph" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		graphFile = argument[1];
		return 2;
	}
	if (strcmp( argument[0], "-coord" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		coordinatesFile = argument[1];
		return 2;
	}
	return 0;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '|': resetCamera(); break;
		case 'r': recording = !recording; break;
		case '[': if (gStepsPerFrame > 1) gStepsPerFrame /= 2; break;
		case ']': gStepsPerFrame *= 2; break;
		default:
			break;
	}
}

void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	printf("Starting Search\n");
	running = true;
	showSearch = true;
	for (int x = 1; x < thePath.size(); x++)
	{
		ge->GetGraph()->findDirectedEdge(thePath[x-1], thePath[x])->setMarked(false);
	}
	node *n1 = ge->GetGraph()->GetRandomNode();
	node *n2 = ge->GetGraph()->GetRandomNode();
	astar.InitializeSearch(ge, n1->GetNum(), n2->GetNum(), thePath);
	
}

double distance(graphState n1, graphState n2)
{
	Graph *g = ge->GetGraph();
	double dx1 = g->GetNode(n1)->GetLabelF(GraphSearchConstants::kXCoordinate);
	double dy1 = g->GetNode(n1)->GetLabelF(GraphSearchConstants::kYCoordinate);
	
	double dx2 = g->GetNode(n2)->GetLabelF(GraphSearchConstants::kXCoordinate);
	double dy2 = g->GetNode(n2)->GetLabelF(GraphSearchConstants::kYCoordinate);
	
	return sqrt((dx1-dx2)*(dx1-dx2)+(dy1-dy2)*(dy1-dy2));
}

void LoadGraph()
{
	Graph *g = new Graph();
	node *n = new node("");
	g->AddNode(n);
	ge = new GraphEnvironment(g);
	ge->SetDirected(true);

	FILE *f = fopen(coordinatesFile.c_str(), "r");
	//FILE *f = fopen("/Users/nathanst/Downloads/USA-road-d.COL.co", "r");
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
	double xoff = (maxx-minx)-scale;
	double yoff = (maxy-miny)-scale;
	for (unsigned int x = 0; x < xloc.size(); x++)
	{
		//printf("(%f, %f) -> ", xloc[x], yloc[x]);
		xloc[x] -= (minx);
		xloc[x] /= scale;
		xloc[x] = xloc[x]*2-1+xoff/scale;

		yloc[x] -= (miny);
		yloc[x] /= scale;
		yloc[x] = yloc[x]*2-1;
		yloc[x] = -yloc[x]+yoff/scale;
		
		node *n = new node("");
		g->AddNode(n);
		n->SetLabelF(GraphSearchConstants::kXCoordinate, xloc[x]);
		n->SetLabelF(GraphSearchConstants::kYCoordinate, yloc[x]);
		n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
		
		//printf("(%f, %f)\n", xloc[x], yloc[x]);
	}
	//a 1 2 1988
	
	f = fopen(graphFile.c_str(), "r");
	//f = fopen("/Users/nathanst/Downloads/USA-road-d.COL.gr", "r");
	int dups = 0;
	while (!feof(f))
	{
		char line[255];
		fgets(line, 255, f);
		if (line[0] == 'a')
		{
			int x1, y1;
			sscanf(line, "a %d %d %*d", &x1, &y1);
			if (g->findDirectedEdge(x1, y1) == 0)
			{
				g->AddEdge(new edge(x1, y1, distance(x1, y1)));
			}
			else {
				dups++;
				//printf("Not adding duplicate directed edge between %d and %d\n", x1, y1);
			}
			//printf("%d to %d\n", x1, y1);
		}
	}
	printf("%d dups ignored\n", dups);
	fclose(f); 
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	//	return false;
	static point3d startLoc;
	if (mType == kMouseDown)
	{
		switch (button)
		{
			case kRightButton: printf("Right button\n"); break;
			case kLeftButton: printf("Left button\n"); break;
			case kMiddleButton: printf("Middle button\n"); break;
		}
	}
	if (button != kLeftButton)
		return false;
	switch (mType)
	{
		case kMouseDown:
		{
			printf("Hit (%f, %f, %f)\n", loc.x, loc.y, loc.z);
			lineStart = loc;
			lineEnd = loc;
			drawLine = true;
			running = false;
			showSearch = false;
			for (int x = 1; x < thePath.size(); x++)
			{
				ge->GetGraph()->findDirectedEdge(thePath[x-1], thePath[x])->setMarked(false);
			}

			return true;
		}
		case kMouseDrag:
		{
			lineEnd = loc;
			return true;
		}
		case kMouseUp:
		{
			printf("UnHit at (%f, %f, %f)\n", loc.x, loc.y, loc.z);
			lineEnd = loc;

			printf("Starting Search\n");
			if (search == 0 || search == 1)
			{
				running = true;
				showSearch = true;
			}
			else if (search == 2)
			{
				showSearchBidirectional = true;
				runningBidirectional = true;
			}
			// find closest point to start/goal loc and run from there.

			node *start = ge->GetGraph()->GetRandomNode(), *goal = ge->GetGraph()->GetRandomNode();
			double startDist = 20, goalDist = 20; // maximum actual distance is 4^2 = 16
			node_iterator ni = ge->GetGraph()->getNodeIter();
			for (node *next = ge->GetGraph()->nodeIterNext(ni); next;
				 next = ge->GetGraph()->nodeIterNext(ni))
			{
				double xdist = next->GetLabelF(GraphSearchConstants::kXCoordinate)-lineStart.x;
				double ydist = next->GetLabelF(GraphSearchConstants::kYCoordinate)-lineStart.y;
				if (xdist*xdist+ydist*ydist < startDist)
				{
					startDist = xdist*xdist+ydist*ydist;
					start = next;
				}
				xdist = next->GetLabelF(GraphSearchConstants::kXCoordinate)-lineEnd.x;
				ydist = next->GetLabelF(GraphSearchConstants::kYCoordinate)-lineEnd.y;
				if (xdist*xdist+ydist*ydist < goalDist)
				{
					goalDist = xdist*xdist+ydist*ydist;
					goal = next;
				}
			}
			drawLine = false;
			start = ge->GetGraph()->GetNode(216446);
			goal = ge->GetGraph()->GetNode(108951);
			if (start == goal)
			{
				printf("Same start and goal; no search\n");
				running = false;
				showSearch = false;
			}
			else {
				printf("Searching from (%1.2f, %1.2f) to (%1.2f, %1.2f) [%d to %d]\n",
					   start->GetLabelF(GraphSearchConstants::kXCoordinate),
					   start->GetLabelF(GraphSearchConstants::kYCoordinate),
					   goal->GetLabelF(GraphSearchConstants::kXCoordinate),
					   goal->GetLabelF(GraphSearchConstants::kYCoordinate),
					   start->GetNum(), goal->GetNum());
				astar.InitializeSearch(ge, start->GetNum(), goal->GetNum(), thePath);
				astar.SetHeuristic(&h);
				if (search == 0)
					astar.SetWeight(0);
				else if (search == 1)
					astar.SetWeight(1);
				else if (search == 2)
					mm.InitializeSearch(ge, start->GetNum(), goal->GetNum(), &z, &z, thePath2);
				else if (search == 3)
					mm.InitializeSearch(ge, start->GetNum(), goal->GetNum(), &h, &h, thePath2);
				search++;
				search = search%4;
			}
			return true;
		}
	}
	return false;
}
