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
#include "TextOverlay.h"
#include <string>
#include "MNPuzzle.h"
#include "IncrementalIDA.h"
#include "PermutationPDB.h"

IncrementalIDA<graphState, graphMove> ida(0);

enum mode {
	kAddNodes,
	kAddEdges,
	kMoveNodes,
	kFindPath
};

bool recording = false;
bool running = false;
double rate = 1.0/32.0;

MNPuzzle mnp(3, 2);
MNPuzzleState start(3, 2);
MNPuzzleState goal(3, 2);
MNPuzzleState t1(3, 2);
MNPuzzleState t2(3, 2);
MNPuzzleState t3(3, 2);
MNPuzzleState t4(3, 2);
std::vector<slideDir> acts;

Graph *g = 0;
GraphEnvironment *ge;
graphState from=mnp.GetMaxStateHash()-1;
graphState hit = -1;
std::vector<graphState> path;
std::vector<graphState> currentPath;
std::vector<graphState> lastPath;

int colorScheme = 10;

TextOverlay te(5);
double timer = 0;

double distance(unsigned long n1, unsigned long n2);

std::vector<int> p1 = {0, 1, 2};
std::vector<int> p2 = {0, 3, 4};
std::vector<int> p3 = {0, 2, 5};

PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> *pdb1 = 0;
PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> *pdb2 = 0;
PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> *pdb3 = 0;

int whichHeuristic = 1;

class GraphDistHeuristic : public Heuristic<graphState> {
public:
	double HCost(const graphState &a, const graphState &b) const
	{
		MNPuzzleState t1(3, 2), t2(3, 2);
		mnp.GetStateFromHash(t1, g->GetNode(a)->GetNum());
		mnp.GetStateFromHash(t2, g->GetNode(b)->GetNum());
		//return int(mnp.HCost(t1, t2));
		int md = int(mnp.HCost(t1, t2));
		int p1=0, p2=0, p3=0;
		if (pdb1)
			p1 = int(pdb1->HCost(t1, t2));
		if (pdb2)
			p2 = int(pdb2->HCost(t1, t2));
		if (pdb3)
			p3 = int(pdb3->HCost(t1, t2));

		if (((whichHeuristic>>0)&1) == 0) // MD
			md = 0;
		if (((whichHeuristic>>1)&1) == 0) // P1
			p1 = 0;
		if (((whichHeuristic>>2)&1) == 0) // P2
			p2 = 0;
		if (((whichHeuristic>>3)&1) == 0) // P3
			p3 = 0;
		return std::max(std::max(std::max(md, p1), p2), p3);
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
	InstallKeyboardHandler(MyDisplayHandler, "Choose Heuristic", "Choose which heuristic is used", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Color", "Increase color schema", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Color", "Decrease color schema", kAnyModifier, '{');

	InstallKeyboardHandler(MyDisplayHandler, "Faster", "Speed up simulation", kAnyModifier, '+');
	InstallKeyboardHandler(MyDisplayHandler, "Slower", "Slow down simulation", kAnyModifier, '-');

	
	InstallKeyboardHandler(MyDisplayHandler, "Increase rank", "Increase start rank and reset graph", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Decrease rank", "Decrease start rank and reset graph", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Clear", "Clear graph", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Help", "Draw help", kAnyModifier, '?');
	InstallKeyboardHandler(MyDisplayHandler, "Heuristic", "Toggle Heuristic", kAnyModifier, 'h');
	//InstallKeyboardHandler(MyDisplayHandler, "Weight", "Toggle Dijkstra & A*", kAnyModifier, 'w');
	//InstallKeyboardHandler(DefaultGraph, "Default", "Build Deafult Graph", kAnyModifier, 'a', 'd');

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

		goal.Reset();
		mnp.StoreGoal(goal);
		pdb1 = new PermutationPDB<MNPuzzleState, slideDir, MNPuzzle>(&mnp, goal, p1);
		pdb2 = new PermutationPDB<MNPuzzleState, slideDir, MNPuzzle>(&mnp, goal, p2);
		pdb3 = new PermutationPDB<MNPuzzleState, slideDir, MNPuzzle>(&mnp, goal, p3);
		
		pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
		pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
		pdb3->BuildPDB(goal, std::thread::hardware_concurrency());

		
		glClearColor(0.99, 0.99, 0.99, 1.0);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		SetNumPorts(windowID, 2);
		g = new Graph();
		ge = new GraphEnvironment(g);
		DefaultGraph(0, kNoModifier, 'a');
		//ida.DoSingleSearchStep(ge, mnp.GetMaxStateHash()-1, 0, &h, path);
		MyDisplayHandler(windowID, kNoModifier, 'o');
		//ge->SetDrawEdgeCosts(true);
		//ge->SetDrawNodeLabels(true);
		//te.AddLine("Hello");
		ge->SetNodeScale(20);

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
		
		ge->SetColor(0.75, 0.75, 1.0);
		double startf = g->GetNode(from)->GetLabelL(GraphSearchConstants::kTemporaryLabel)+h.HCost(from, 0);
		double goalf = g->GetNode(0)->GetLabelL(GraphSearchConstants::kTemporaryLabel);
		for (int x = 0; x < g->GetNumNodes(); x++)
		{
			double cost = g->GetNode(x)->GetLabelL(GraphSearchConstants::kTemporaryLabel)+h.HCost(x, 0);
			recColor c = getColor(cost, startf, goalf, colorScheme);
			if (hit == x)
			{
				ge->SetColor(0, 0, 0);
				ge->SetNodeScale(25);
				ge->OpenGLDraw(x);
				ge->SetNodeScale(20);
			}
			if (!fgreater(cost, goalf))
			{
				ge->SetColor(c.r, c.g, c.b);
				ge->OpenGLDraw(x);
			}
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

		ge->SetColor(1.0, 0, 0);
		glLineWidth(10);
		ida.OpenGLDraw();
		glLineWidth(1);
	}
	if (viewport == 1)
	{
		te.OpenGLDraw(windowID);
		glScalef(0.7*1.0, 0.7*0.6666, 0.7*1.0);
		glTranslatef(0, 0.7, 0);
		if (currentPath.size() > 0 && lastPath.size() > 0)
		{
			if (timer < 1)
			{
				mnp.GetStateFromHash(t1, currentPath.back());
				mnp.GetStateFromHash(t2, lastPath.back());
				mnp.OpenGLDraw(t1, t2, timer);

				glTranslatef(0, -1.5, 0);
				glScalef(0.45*0.75, 0.45*0.75, 0.45*0.75);
				glTranslatef(-2.25, 0, 0);
				if (pdb1 && whichHeuristic&0x2)
				{
					uint64_t h1 = pdb1->GetPDBHash(t1);
					uint64_t h2 = pdb1->GetPDBHash(t2);
					pdb1->GetStateFromPDBHash(h1, t3);
					pdb1->GetStateFromPDBHash(h2, t4);
					mnp.OpenGLDraw(t3, t4, timer);
				}
				glTranslatef(2.25, 0, 0);
				if (pdb2 && whichHeuristic&0x4)
				{
					uint64_t h1 = pdb2->GetPDBHash(t1);
					uint64_t h2 = pdb2->GetPDBHash(t2);
					pdb2->GetStateFromPDBHash(h1, t3);
					pdb2->GetStateFromPDBHash(h2, t4);
					mnp.OpenGLDraw(t3, t4, timer);
				}
				glTranslatef(2.25, 0, 0);
				if (pdb3 && whichHeuristic&0x8)
				{
					uint64_t h1 = pdb3->GetPDBHash(t1);
					uint64_t h2 = pdb3->GetPDBHash(t2);
					pdb3->GetStateFromPDBHash(h1, t3);
					pdb3->GetStateFromPDBHash(h2, t4);
					mnp.OpenGLDraw(t3, t4, timer);
				}
			}
			else {
				mnp.GetStateFromHash(t1, ida.GetCurrentState());
				mnp.OpenGLDraw(t1);

				glTranslatef(0, -1.5, 0);
				glScalef(0.45*0.75, 0.45*0.75, 0.45*0.75);
				glTranslatef(-2.25, 0, 0);
				if (pdb1 && whichHeuristic&0x2)
				{
					uint64_t h1 = pdb1->GetPDBHash(t1);
					pdb1->GetStateFromPDBHash(h1, t2);
					mnp.OpenGLDraw(t2);
				}
				glTranslatef(2.25, 0, 0);
				if (pdb2 && whichHeuristic&0x4)
				{
					uint64_t h1 = pdb2->GetPDBHash(t1);
					pdb2->GetStateFromPDBHash(h1, t2);
					mnp.OpenGLDraw(t2);
				}
				glTranslatef(2.25, 0, 0);
				if (pdb3 && whichHeuristic&0x8)
				{
					uint64_t h1 = pdb3->GetPDBHash(t1);
					pdb3->GetStateFromPDBHash(h1, t2);
					mnp.OpenGLDraw(t2);
				}
			}
		}
		else {
			timer += 1;
			if (hit != -1)
				mnp.GetStateFromHash(t1, hit);
			else
				mnp.GetStateFromHash(t1, ida.GetCurrentState());
			mnp.OpenGLDraw(t1);

			glTranslatef(0, -1.5, 0);
			glScalef(0.45*0.75, 0.45*0.75, 0.45*0.75);
			glTranslatef(-2.25, 0, 0);
			if (pdb1 && whichHeuristic&0x2)
			{
				uint64_t h1 = pdb1->GetPDBHash(t1);
				pdb1->GetStateFromPDBHash(h1, t2);
				mnp.OpenGLDraw(t2);
			}
			glTranslatef(2.25, 0, 0);
			if (pdb2 && whichHeuristic&0x4)
			{
				uint64_t h1 = pdb2->GetPDBHash(t1);
				pdb2->GetStateFromPDBHash(h1, t2);
				mnp.OpenGLDraw(t2);
			}
			glTranslatef(2.25, 0, 0);
			if (pdb3 && whichHeuristic&0x8)
			{
				uint64_t h1 = pdb3->GetPDBHash(t1);
				pdb3->GetStateFromPDBHash(h1, t2);
				mnp.OpenGLDraw(t2);
			}
		}
		timer += rate;
	}

	
	
	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/idastar-%d%d%d%d",
				(frameCnt/1000)%10, (frameCnt/100)%10, (frameCnt/10)%10, frameCnt%10);
		SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
		frameCnt++;
		if (path.size() == 0)
		{
		}
		else {
			recording = false;
		}
	}
	
	if (timer > 1 && running && viewport == GetNumPorts(windowID)-1)
		MyDisplayHandler(windowID, kNoModifier, 'o');
	return;
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	strncpy(gDefaultMap, argument[1], 1024);
	return 2;
}

void RedrawTextDisplay()
{
	te.Clear();
	std::string s = "Current Limit: ";
	s += std::to_string(ida.GetCurrentFLimit());
	te.AddLine(s.c_str());
	s = "Next Limit: ";
	s += std::to_string(ida.GetNextFLimit());
	te.AddLine(s.c_str());
	te.AddLine("");
	s = "Current f-cost: ";
	if (currentPath.size() > 0)
	{
		s += std::to_string((int)(h.HCost(currentPath.back(), 0)+
								  currentPath.size()-1));
		//g->GetNode(currentPath.back())->GetLabelL(GraphSearchConstants::kTemporaryLabel)));
		s += " (g: "+std::to_string(currentPath.size()-1);
		//std::to_string(g->GetNode(currentPath.back())->GetLabelL(GraphSearchConstants::kTemporaryLabel));
		s += " h: "+std::to_string((int)h.HCost(currentPath.back(), 0))+")";
	}
	else if (hit != -1)
	{
		s += std::to_string((int)(h.HCost(hit, 0))+g->GetNode(hit)->GetLabelL(GraphSearchConstants::kTemporaryLabel));
		s += " (g: ";//std::to_string(currentPath.size()-1);
		s += std::to_string(g->GetNode(hit)->GetLabelL(GraphSearchConstants::kTemporaryLabel));
		s += " h: "+std::to_string((int)h.HCost(hit, 0))+")";
	}
	else {
		s += "-";
	}
	te.AddLine(s.c_str());
	//te.AddLine("");
	s = "Heuristic: ";
	if (whichHeuristic&1)
		s += "MD ";
	if (whichHeuristic&2)
		s += "P1 ";
	if (whichHeuristic&4)
		s += "P2 ";
	if (whichHeuristic&8)
		s += "P3 ";
	te.AddLine(s.c_str());
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 'h': whichHeuristic = (whichHeuristic+1)%16; break;
		case '+': rate *= 2; if (rate > 1) rate = 1; break;
		case '-': rate /= 2; break;
		case '{':
		{
			colorScheme = (colorScheme+20)%21;
			printf("Color scheme %d\n", colorScheme);
			break;
		}
		case '}':
		{
			colorScheme = (colorScheme+1)%21;
			printf("Color scheme %d\n", colorScheme);
			break;
		}
		case ']':
		{
			from = (from+1)%mnp.GetMaxStateHash();
			printf("Root: %lu\n", from);
			DefaultGraph(windowID, mod, key);
			ida.Reset();
			currentPath.clear();
			lastPath.clear();
			MyDisplayHandler(windowID, kNoModifier, 'o');
		}
			break;
		case '[':
		{
			from = (from+mnp.GetMaxStateHash()-1)%mnp.GetMaxStateHash();
			printf("Root: %lu\n", from);
			DefaultGraph(windowID, mod, key);
			ida.Reset();
			currentPath.clear();
			lastPath.clear();
			MyDisplayHandler(windowID, kNoModifier, 'o');
		}
			break;
		case '|':
		{
			g->Reset();
			path.resize(0);
			running = false;
		}
			break;
		case 'r':
			recording = !recording;
			running = true;
			break;
		case '0':
		{
			whichHeuristic = 0;
			ida.Reset();
			currentPath.clear();
			lastPath.clear();
			RedrawTextDisplay();
		}
			break;
		case '1':
		{
			whichHeuristic ^= 0x1;
			ida.Reset();
			currentPath.clear();
			lastPath.clear();
			RedrawTextDisplay();
		}
			break;
		case '2':
		{
			whichHeuristic ^= 0x2;
			ida.Reset();
			currentPath.clear();
			lastPath.clear();
			RedrawTextDisplay();
		}
			break;

		case '3':
		{
			whichHeuristic ^= 0x4;
			ida.Reset();
			currentPath.clear();
			lastPath.clear();
			RedrawTextDisplay();
		}
			break;

		case '4':
		{
			whichHeuristic ^= 0x8;
			ida.Reset();
			currentPath.clear();
			lastPath.clear();
			RedrawTextDisplay();
		}
			break;

		case '\t':
			if (mod != kShiftDown)
				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
			else
			{
				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
			}
			break;
		case 'p':
			running = !running;
			break;
		case 'o':
		{
			ida.GetCurrentPath(lastPath);
			ida.DoSingleSearchStep(ge, from, 0, &h, path);
			ida.GetCurrentPath(currentPath);
			timer = 0;
			RedrawTextDisplay();
		}
			break;
		case '?':
		{
		}
			break;
		default:
			break;
	}
	
}

void DefaultGraph(unsigned long windowID, tKeyboardModifier mod, char key)
{
	uint64_t maxVal = from;
	g->Reset();
	for (uint64_t x = 0; x < mnp.GetMaxStateHash(); x++)
	{
		node *n;
		g->AddNode(n = new node(""));
		n->SetLabelL(GraphSearchConstants::kTemporaryLabel, -1);
	}
	for (uint64_t x = 0; x < mnp.GetMaxStateHash(); x++)
	{
		mnp.GetStateFromHash(start, x);
		mnp.GetActions(start, acts);
		for (auto a : acts)
		{
			mnp.ApplyAction(start, a);
			uint64_t child = mnp.GetStateHash(start);
			if (g->FindEdge(x, child) == 0)
				g->AddEdge(new edge(x, child, 1));
			mnp.UndoAction(start, a);
		}
	}
	std::vector<int> distribution;
	g->GetNode(maxVal)->SetLabelL(GraphSearchConstants::kTemporaryLabel, 0);
	std::deque<int> bfsQueue;
	bfsQueue.push_front(maxVal);
	while (bfsQueue.size() > 0)
	{
		node *n = g->GetNode(bfsQueue.back());
		bfsQueue.pop_back();
		int depth = n->GetLabelL(GraphSearchConstants::kTemporaryLabel);
		if (depth >= distribution.size())
			distribution.resize(depth+1);
		n->SetLabelL(GraphSearchConstants::kFirstData, distribution[depth]);
		distribution[depth]++;
		//printf("%d at depth %d\n", n->GetNum(), depth);
		
		edge_iterator ei = n->getEdgeIter();
		for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
		{
			unsigned int neighbor = e->getFrom();
			if (neighbor == n->GetNum())
				neighbor = e->getTo();
			
			if (g->GetNode(neighbor)->GetLabelL(GraphSearchConstants::kTemporaryLabel) == -1)
			{
				g->GetNode(neighbor)->SetLabelL(GraphSearchConstants::kTemporaryLabel, depth+1);
				bfsQueue.push_front(neighbor);
			}
		}
	}
	
	for (uint64_t x = 0; x < mnp.GetMaxStateHash(); x++)
	{
		node *n = g->GetNode(x);
		long depth = n->GetLabelL(GraphSearchConstants::kTemporaryLabel);
		long location = n->GetLabelL(GraphSearchConstants::kFirstData);
		//printf("%d at depth %ld loc %ld\n", n->GetNum(), depth, location);
		n->SetLabelF(GraphSearchConstants::kXCoordinate, (2.0*(location+1.0))/(distribution[depth]+1.0)-1.0);
		n->SetLabelF(GraphSearchConstants::kYCoordinate, -1.0+2.0*depth/distribution.size());
		n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
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
	//if (button != kLeftButton)

	// no mouse right now
	//return false;
	
	switch (mType)
	{
		case kMouseDown:
		{
			printf("Hit (%f, %f, %f)\n", loc.x, loc.y, loc.z);
			running = false;
			ida.Reset();
			currentPath.clear();
			lastPath.clear();
			hit = FindClosestNode(g, loc)->GetNum();
			RedrawTextDisplay();
			//mnp.GetStateFromHash(start, from);
			return true;
		}
		case kMouseDrag:
		{
			hit = FindClosestNode(g, loc)->GetNum();
			RedrawTextDisplay();
			//mnp.GetStateFromHash(start, from);
			return true;
		}
		case kMouseUp:
		{
			printf("UnHit at (%f, %f, %f)\n", loc.x, loc.y, loc.z);
			hit = -1;
			RedrawTextDisplay();
//			from = FindClosestNode(g, loc)->GetNum();
//			mnp.GetStateFromHash(start, from);
			return true;
		}
	}
	return false;
}

