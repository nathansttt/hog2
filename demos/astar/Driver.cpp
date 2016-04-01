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
#include "Driver.h"
#include "GraphEnvironment.h"
#include "TemplateAStar.h"
#include "TextOverlay.h"
#include <string>

enum mode {
	kAddNodes,
	kAddEdges,
	kMoveNodes,
	kFindPath
};

TemplateAStar<graphState, graphMove, GraphEnvironment> astar;
std::vector<graphState> path;

mode m = kAddNodes;

bool recording = false;
bool running = false;

double edgeCost = 1.0;
double weight = 1.0;

Graph *g = 0;
GraphEnvironment *ge;
graphState from=-1, to=-1;

TextOverlay te(35);

void SaveGraph(const char *file);
void LoadGraph(const char *file);
void ShowSearchInfo();
static char name[2] = "a";

double distance(unsigned long n1, unsigned long n2);

class GraphDistHeuristic : public Heuristic<graphState> {
public:
	double HCost(const graphState &a, const graphState &b) const
	{
		return distance(a, b);
	}
};

GraphDistHeuristic h;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1600, 800);
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Clear", "Clear graph", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Help", "Draw help", kAnyModifier, '?');
	InstallKeyboardHandler(MyDisplayHandler, "Weight", "Toggle Dijkstra & A*", kAnyModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Save", "Save current graph", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Load", "Load last saved graph", kAnyModifier, 'l');
	InstallKeyboardHandler(DefaultGraph, "Default", "Build Deafult Graph", kAnyModifier, 'a', 'd');

	//InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	
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
		glClearColor(0.99, 0.99, 0.99, 1.0);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		SetNumPorts(windowID, 2);
		g = new Graph();
		ge = new GraphEnvironment(g);
		ge->SetDrawEdgeCosts(true);
		ge->SetDrawNodeLabels(true);
		astar.SetWeight(1.0);
		astar.SetHeuristic(&h);
		te.AddLine("A* algorithm sample code");
		te.AddLine("Current mode: add nodes (click to add node)");
		te.AddLine("Press [ or ] to change modes. '?' for help.");
	}
}

int frameCnt = 0;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (viewport == 0)
	{
		if (ge == 0 || g == 0)
			return;
		ge->SetColor(0.5, 0.5, 1.0);
		ge->OpenGLDraw();
		
		if (from != -1 && to != -1)
		{
			glLineWidth(4.);
			ge->SetColor(1, 0, 0);
			ge->GLDrawLine(from, to);
		}
		
		if (running)
		{
			//astar.DoSingleSearchStep(path);
			astar.OpenGLDraw();
		}
		else {
			ge->SetColor(0.75, 0.75, 1.0);
			for (int x = 0; x < g->GetNumNodes(); x++)
				ge->OpenGLDraw(x);
		}
		
		if (path.size() > 0)
		{
			ge->SetColor(0, 1, 0);
			glLineWidth(10);
			for (int x = 1; x < path.size(); x++)
			{
				ge->GLDrawLine(path[x-1], path[x]);
			}
			glLineWidth(1);
		}
	}
	if (viewport == 1)
	{
		te.OpenGLDraw(windowID);
	}
	
	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/astar-%d%d%d%d",
				(frameCnt/1000)%10, (frameCnt/100)%10, (frameCnt/10)%10, frameCnt%10);
		SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
		frameCnt++;
		if (path.size() == 0)
		{
			MyDisplayHandler(windowID, kNoModifier, 'o');
		}
		else {
			recording = false;
		}
	}
	return;
	
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	strncpy(gDefaultMap, argument[1], 1024);
	return 2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '{':
		{
			break;
		}
		case ']':
		{
			switch (m)
			{
				case kAddNodes: m = kAddEdges; te.AddLine("Current mode: add edges"); break;
				case kAddEdges: m = kMoveNodes; te.AddLine("Current mode: moves nodes"); break;
				case kMoveNodes: m = kFindPath; te.AddLine("Current mode: find path"); break;
				case kFindPath: m = kAddNodes; te.AddLine("Current mode: add nodes"); break;
			}

		}
			break;
		case '[':
		{
			switch (m)
			{
				case kMoveNodes: m = kAddEdges; te.AddLine("Current mode: add edges"); break;
				case kFindPath: m = kMoveNodes; te.AddLine("Current mode: moves nodes"); break;
				case kAddNodes: m = kFindPath; te.AddLine("Current mode: find path"); break;
				case kAddEdges: m = kAddNodes; te.AddLine("Current mode: add nodes"); break;
			}
		}
			break;
		case '|':
		{
			name[0] = 'a';
			g->Reset();
			te.AddLine("Current mode: add nodes");
			m = kAddNodes;
			path.resize(0);
			running = false;
		}
			break;
		case 'w':
			if (weight > 0.5)
				weight = 0.0;
			else
				weight = 1.0;
			astar.SetWeight(weight);
			astar.InitializeSearch(ge, astar.start, astar.goal, path);
			ShowSearchInfo();
			
			running = true;
			break;
		case 'r':
			recording = !recording;
			break;
		case '0':
//		case '1': edgeCost = 1.0; te.AddLine("Adding edges; New edges cost 1"); m = kAddEdges; break;
//		case '2': edgeCost = 2.0; te.AddLine("Adding edges; New edges cost 2"); m = kAddEdges; break;
//		case '3': edgeCost = 3.0; te.AddLine("Adding edges; New edges cost 3"); m = kAddEdges; break;
//		case '4': edgeCost = 4.0; te.AddLine("Adding edges; New edges cost 4"); m = kAddEdges; break;
//		case '5': edgeCost = 5.0; te.AddLine("Adding edges; New edges cost 5"); m = kAddEdges; break;
//		case '6': edgeCost = 6.0; te.AddLine("Adding edges; New edges cost 6"); m = kAddEdges; break;
//		case '7': edgeCost = 7.0; te.AddLine("Adding edges; New edges cost 7"); m = kAddEdges; break;
//		case '8': edgeCost = 8.0; te.AddLine("Adding edges; New edges cost 8"); m = kAddEdges; break;
//		case '9': edgeCost = 9.0; te.AddLine("Adding edges; New edges cost 9"); m = kAddEdges; break;
		case '\t':
			if (mod != kShiftDown)
				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
			else
			{
				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
			}
			break;
		case 'p':
			//running = !running;
			break;
		case 'o':
		{
			if (running)
			{
				astar.DoSingleSearchStep(path);
				ShowSearchInfo();
			}
		}
			break;
		case '?':
		{
			te.AddLine("Help:");
			te.AddLine("-----");
			te.AddLine("Press '[' and ']' to switch between modes.");
			te.AddLine("Add nodes: click to add a node to the graph");
			te.AddLine("Add edges: drag between nodes to add an edge");
			te.AddLine("           Press 1-9 to change edge weight");
			te.AddLine("Move nodes: drag to move node locations");
			te.AddLine("Find Path: Drag to find path between nodes");
			te.AddLine("           'o' to step pathfinding forward");
			te.AddLine("           Red nodes: closed list");
			te.AddLine("           Green nodes: open list");
			te.AddLine("           Yellow node: next on open list");
			te.AddLine("           Pink node: goal state");
		}
			break;
		case 's':
			SaveGraph("save.graph");
			break;
		case 'l':
			LoadGraph("save.graph");
			break;
		default:
			break;
	}
	
}

void DefaultGraph(unsigned long windowID, tKeyboardModifier mod, char key)
{
	if (key == 'a')
	{
		MyDisplayHandler(windowID, kNoModifier, '|'); // clear current graph
		
		g->AddNode(new node("a"));
		g->AddNode(new node("b"));
		g->AddNode(new node("c"));
		g->AddNode(new node("d"));
		g->AddNode(new node("e"));

		g->AddEdge(new edge(0, 1, 1));
		g->AddEdge(new edge(0, 2, 2));
		g->AddEdge(new edge(1, 3, 3));
		g->AddEdge(new edge(2, 3, 1));
		g->AddEdge(new edge(2, 4, 9));
		g->AddEdge(new edge(3, 4, 5));

		
		g->GetNode(0)->SetLabelF(GraphSearchConstants::kXCoordinate, -0.5);
		g->GetNode(0)->SetLabelF(GraphSearchConstants::kYCoordinate, -0.65);
		g->GetNode(0)->SetLabelF(GraphSearchConstants::kZCoordinate, 0);

		g->GetNode(1)->SetLabelF(GraphSearchConstants::kXCoordinate, 0.5);
		g->GetNode(1)->SetLabelF(GraphSearchConstants::kYCoordinate, -0.65);
		g->GetNode(1)->SetLabelF(GraphSearchConstants::kZCoordinate, 0);

		g->GetNode(2)->SetLabelF(GraphSearchConstants::kXCoordinate, -0.5);
		g->GetNode(2)->SetLabelF(GraphSearchConstants::kYCoordinate, 0.15);
		g->GetNode(2)->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
		
		g->GetNode(3)->SetLabelF(GraphSearchConstants::kXCoordinate, 0.5);
		g->GetNode(3)->SetLabelF(GraphSearchConstants::kYCoordinate, 0.15);
		g->GetNode(3)->SetLabelF(GraphSearchConstants::kZCoordinate, 0);

		g->GetNode(4)->SetLabelF(GraphSearchConstants::kXCoordinate, 0.0);
		g->GetNode(4)->SetLabelF(GraphSearchConstants::kYCoordinate, 0.75);
		g->GetNode(4)->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
	
	}
}
#include <iomanip>
#include <sstream>

std::string MyToString(double val)
{
	std::stringstream ss;
	ss << std::fixed << std::setprecision(2) << val;
	return ss.str();
}

void ShowSearchInfo()
{
	const int colWidth = 24;
	std::string s;
	te.Clear();
	s = "-----> Searching from ";
	s +=g->GetNode(astar.start)->GetName();
	s +=" to ";
	s += g->GetNode(astar.goal)->GetName();
	s += " <-----";
	te.AddLine(s.c_str());
	te.AddLine("Press 'o' to advance search.");
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		double gcost;
		s = g->GetNode(x)->GetName();
		switch (astar.GetStateLocation(x))
		{
			case kClosedList:
			{
				s += ": Closed  (g: ";
				astar.GetClosedListGCost(x, gcost);
				s += MyToString(gcost);
				s += ", h: ";
				s += MyToString(h.HCost(x, astar.goal));
				s += ")";
			}
			break;
			case kOpenList:
			{
				s += ": Open    (g: ";
				astar.GetOpenListGCost(x, gcost);
				s += MyToString(gcost);
				s += ", h: ";
				s += MyToString(h.HCost(x, astar.goal));
				s += ")";
			}
				break;

			case kNotFound:
				s += ": Ungenerated (h: ";
				s += MyToString(h.HCost(x, astar.goal));
				s += ")";
				break;
		}
		
		te.AddLine(s.c_str());
	}

	te.AddLine("");
	te.AddLine("Open List:");
	size_t length = strlen(te.GetLastLine());
	for (int x = length; x < colWidth; x++)
		te.AppendToLine(" ");
	te.AppendToLine("Closed List:");

	std::vector<std::string> open, closed;
	
	for (int x = 0; x < astar.GetNumOpenItems(); x++)
	{
		auto item = astar.GetOpenItem(x);
		s = g->GetNode(item.data)->GetName();
		s += ": ";
		s += MyToString(item.g+item.h);
		s += "=";
		s += MyToString(item.g);
		s += "+";
		s += MyToString(item.h);
		s += " p: ";
		s += g->GetNode(astar.GetItem(item.parentID).data)->GetName();
		//te.AddLine(s.c_str());
		open.push_back(s);
	}
	
	for (int x = 0; x < astar.GetNumItems(); x++)
	{
		auto item = astar.GetItem(x);
		if (item.where == kClosedList)
		{
			s = g->GetNode(item.data)->GetName();
			s += ": ";
			s += MyToString(item.g+item.h);
			s += "=";
			s += MyToString(item.g);
			s += "+";
			s += MyToString(item.h);
			s += " p: ";
			s += g->GetNode(astar.GetItem(item.parentID).data)->GetName();
			//te.AddLine(s.c_str());
			closed.push_back(s);
		}
	}
	for (size_t x = 0; x < open.size(); x++)
	{
		te.AddLine(open[x].c_str());
		for (size_t t = open[x].length(); t < colWidth; t++)
			te.AppendToLine(" ");
		if (x < closed.size())
			te.AppendToLine(closed[x].c_str());
	}
	for (size_t x = open.size(); x < closed.size(); x++)
	{
		te.AddLine("                        ");
		te.AppendToLine(closed[x].c_str());
	}
}

double distsquared(unsigned long node, point3d loc)
{
	double dx = g->GetNode(node)->GetLabelF(GraphSearchConstants::kXCoordinate);
	double dy = g->GetNode(node)->GetLabelF(GraphSearchConstants::kYCoordinate);

	return (dx-loc.x)*(dx-loc.x) + (dy-loc.y)*(dy-loc.y);
}

double distance(unsigned long n1, unsigned long n2)
{
	double dx1 = g->GetNode(n1)->GetLabelF(GraphSearchConstants::kXCoordinate);
	double dy1 = g->GetNode(n1)->GetLabelF(GraphSearchConstants::kYCoordinate);

	double dx2 = g->GetNode(n2)->GetLabelF(GraphSearchConstants::kXCoordinate);
	double dy2 = g->GetNode(n2)->GetLabelF(GraphSearchConstants::kYCoordinate);

	return sqrt((dx1-dx2)*(dx1-dx2)+(dy1-dy2)*(dy1-dy2));
}

node *FindClosestNode(Graph *gr, point3d loc)
{
	if (gr->GetNumNodes() == 0)
		return 0;
	unsigned long best = 0;
	double dist = distsquared(0, loc);
	for (unsigned long x = 1; x < gr->GetNumNodes(); x++)
	{
		if (fless(distsquared(x, loc), dist))
		{
			dist = distsquared(x, loc);
			best = x;
		}
	}
	return gr->GetNode(best);
}


bool MyClickHandler(unsigned long , int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
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
			if (m == kAddNodes)
			{
				if (loc.x > 1 || loc.x < -1 || loc.y > 1 || loc.y < -1)
					return false;
				node *n = new node(name);
				name[0]++;
				g->AddNode(n);
				n->SetLabelF(GraphSearchConstants::kXCoordinate, loc.x);
				n->SetLabelF(GraphSearchConstants::kYCoordinate, loc.y);
				n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
				printf("Added node %d to graph\n", g->GetNumNodes());
			}
			if (m == kAddEdges || m == kFindPath)
			{
				from = to = FindClosestNode(g, loc)->GetNum();
			}
			if (m == kMoveNodes)
			{
				from = to = FindClosestNode(g, loc)->GetNum();

				if (loc.x > 1) loc.x = 1;
				if (loc.x < -1) loc.x = -1;
				if (loc.y > 1) loc.y = 1;
				if (loc.y < -1) loc.y = -1;
				g->GetNode(from)->SetLabelF(GraphSearchConstants::kXCoordinate, loc.x);
				g->GetNode(from)->SetLabelF(GraphSearchConstants::kYCoordinate, loc.y);
				g->GetNode(from)->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
				edge_iterator i = g->GetNode(from)->getEdgeIter();
				for (edge *e = g->GetNode(from)->edgeIterNext(i); e; e = g->GetNode(from)->edgeIterNext(i))
					e->setWeight(distance(e->getFrom(), e->getTo()));
			}
			return true;
		}
		case kMouseDrag:
		{
			if (m == kAddEdges || m == kFindPath)
			{
				to = FindClosestNode(g, loc)->GetNum();
			}
			if (m == kMoveNodes)
			{
				if (loc.x > 1) loc.x = 1;
				if (loc.x < -1) loc.x = -1;
				if (loc.y > 1) loc.y = 1;
				if (loc.y < -1) loc.y = -1;
				g->GetNode(from)->SetLabelF(GraphSearchConstants::kXCoordinate, loc.x);
				g->GetNode(from)->SetLabelF(GraphSearchConstants::kYCoordinate, loc.y);
				g->GetNode(from)->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
				edge_iterator i = g->GetNode(from)->getEdgeIter();
				for (edge *e = g->GetNode(from)->edgeIterNext(i); e; e = g->GetNode(from)->edgeIterNext(i))
					e->setWeight(distance(e->getFrom(), e->getTo()));
			}
			return true;
		}
		case kMouseUp:
		{
			printf("UnHit at (%f, %f, %f)\n", loc.x, loc.y, loc.z);
			if (m == kAddEdges)
			{
				to = FindClosestNode(g, loc)->GetNum();
				if (from != to)
				{
					edge *e;
					if ((e = g->FindEdge(from, to)) != 0)
					{
						//e->setWeight(distance(from, to));
					}
					else {
						g->AddEdge(new edge(from, to, distance(from, to)));
					}
				}
			}
			if (m == kFindPath)
			{
				to = FindClosestNode(g, loc)->GetNum();
				if (from != to)
				{
					weight = 1.0;
					astar.SetWeight(weight);
					astar.InitializeSearch(ge, from, to, path);
					ShowSearchInfo();
					
					running = true;
				}
			}
			if (m == kMoveNodes)
			{
				if (loc.x > 1) loc.x = 1;
				if (loc.x < -1) loc.x = -1;
				if (loc.y > 1) loc.y = 1;
				if (loc.y < -1) loc.y = -1;

				g->GetNode(from)->SetLabelF(GraphSearchConstants::kXCoordinate, loc.x);
				g->GetNode(from)->SetLabelF(GraphSearchConstants::kYCoordinate, loc.y);
				g->GetNode(from)->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
				edge_iterator i = g->GetNode(from)->getEdgeIter();
				for (edge *e = g->GetNode(from)->edgeIterNext(i); e; e = g->GetNode(from)->edgeIterNext(i))
					e->setWeight(distance(e->getFrom(), e->getTo()));
			}
			from = to = -1;
			return true;
		}
	}
	return false;
}

void SaveGraph(const char *file)
{
	FILE *f = fopen(file, "w+");
	if (f)
	{
		fprintf(f, "%d %d\n", g->GetNumNodes(), g->GetNumEdges());
		for (int x = 0; x < g->GetNumNodes(); x++)
		{
			fprintf(f, "%d %f %f %f %s\n", x,
					g->GetNode(x)->GetLabelF(GraphSearchConstants::kXCoordinate),
					g->GetNode(x)->GetLabelF(GraphSearchConstants::kYCoordinate),
					g->GetNode(x)->GetLabelF(GraphSearchConstants::kZCoordinate),
					g->GetNode(x)->GetName()
					);
		}
		edge_iterator ei = g->getEdgeIter();
		while (1)
		{
			edge *e = (edge *)g->edgeIterNext(ei);
			if (e)
				fprintf(f, "%d %d %f\n", e->getFrom(), e->getTo(), e->GetWeight());
			else
				break;
		}
		fclose(f);
	}
}

void LoadGraph(const char *file)
{
	g->Reset();
	char nodeName[255];
	FILE *f = fopen(file, "r");
	if (f)
	{
		int numNodes, numEdges;
		fscanf(f, "%d %d\n", &numNodes, &numEdges);
		for (int n = 0; n < numNodes; n++)
		{
			int which;
			float x, y, z;
			fscanf(f, "%d %f %f %f %s\n", &which, &x, &y, &z, nodeName);
			assert(which == n);
			node *next = new node(nodeName);
			next->SetLabelF(GraphSearchConstants::kXCoordinate, x);
			next->SetLabelF(GraphSearchConstants::kYCoordinate, y);
			next->SetLabelF(GraphSearchConstants::kZCoordinate, z);
			g->AddNode(next);
		}
		for (int e = 0; e < numEdges; e++)
		{
			int from, to;
			float weight;
			fscanf(f, "%d %d %f", &from, &to, &weight);
			g->AddEdge(new edge(from, to, weight));
		}
		fclose(f);
	}
}
