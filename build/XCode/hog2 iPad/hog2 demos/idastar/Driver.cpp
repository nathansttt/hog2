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
#include "IncrementalBTS.h"
#include "PermutationPDB.h"
#include "LexPermutationPDB.h"

//IncrementalIDA<graphState, graphMove> ida;
IncrementalBTS<graphState, graphMove> ida;

enum mode {
	kAddNodes,
	kAddEdges,
	kMoveNodes,
	kFindPath
};

enum view {
	kTreeView = 0,
	kTextView = 1,
	kStateView = 2,
	kPDB1View = 3,
	kPDB2View = 4,
	kPDB3View = 5
};

bool recording = false;
bool running = false;
double rate = 1.0/32.0;
int displayPrecision = 0;

MNPuzzle<3, 2> mnp;
MNPuzzleState<3, 2> start;
MNPuzzleState<3, 2> goal;
MNPuzzleState<3, 2> t1;
MNPuzzleState<3, 2> t2;
MNPuzzleState<3, 2> t3;
MNPuzzleState<3, 2> t4;
std::vector<slideDir> acts;

Graph *g = 0;
GraphEnvironment *ge;
graphState from=mnp.GetMaxStateHash()-1;
graphState hit = -1;
std::vector<graphState> path;
std::vector<graphState> currentPath;
std::vector<graphState> lastPath;

int colorScheme = 4;// 10, 7 & 4 are reasonable

TextOverlay te(6);
double timer = 0;

double distance(unsigned long n1, unsigned long n2);

std::vector<int> p1 = {0, 1, 2};
std::vector<int> p2 = {0, 3, 4};
std::vector<int> p3 = {0, 2, 5};

PermutationPDB<MNPuzzleState<3, 2>, slideDir, MNPuzzle<3, 2>> *pdb1 = 0;
PermutationPDB<MNPuzzleState<3, 2>, slideDir, MNPuzzle<3, 2>> *pdb2 = 0;
PermutationPDB<MNPuzzleState<3, 2>, slideDir, MNPuzzle<3, 2>> *pdb3 = 0;

int whichHeuristic = 1;
bool compressedPDBs = false;

class GraphDistHeuristic : public Heuristic<graphState> {
public:
	GraphDistHeuristic(double w = 1.0)
	:weight(w) {}
	double HCost(const graphState &a, const graphState &b) const
	{
		MNPuzzleState<3, 2> t1, t2;
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
		return weight*std::max(std::max(std::max(md, p1), p2), p3);
	}
private:
	double weight;
};

GraphDistHeuristic h;
//GraphDistHeuristic wh(5);

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1600, 800);
	return 0;
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

	InstallKeyboardHandler(MyDisplayHandler, "Compress", "Compress PDBs", kAnyModifier, 'c');

	
	InstallKeyboardHandler(MyDisplayHandler, "Increase rank", "Increase start rank and reset graph", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Decrease rank", "Decrease start rank and reset graph", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Clear", "Clear graph", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Help", "Draw help", kAnyModifier, '?');
	InstallKeyboardHandler(MyDisplayHandler, "Heuristic", "Toggle Heuristic", kAnyModifier, 'h');
	InstallKeyboardHandler(MyDisplayHandler, "Unit Cost", "Toggle Unit Costs", kAnyModifier, 'u');
	//InstallKeyboardHandler(MyDisplayHandler, "Weight", "Toggle Dijkstra & A*", kAnyModifier, 'w');
	//InstallKeyboardHandler(DefaultGraph, "Default", "Build Deafult Graph", kAnyModifier, 'a', 'd');

	//InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	
	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler);
}

void BuildPDBs()
{
	goal.Reset();
	mnp.StoreGoal(goal);

	delete pdb1;
	delete pdb2;
	delete pdb3;
	
	pdb1 = new LexPermutationPDB<MNPuzzleState<3, 2>, slideDir, MNPuzzle<3, 2>>(&mnp, goal, p1);
	pdb2 = new LexPermutationPDB<MNPuzzleState<3, 2>, slideDir, MNPuzzle<3, 2>>(&mnp, goal, p2);
	pdb3 = new LexPermutationPDB<MNPuzzleState<3, 2>, slideDir, MNPuzzle<3, 2>>(&mnp, goal, p3);
	
	// non-threaded BuildPDB code
	pdb1->BuildPDB(goal);
	pdb2->BuildPDB(goal);
	pdb3->BuildPDB(goal);

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

		BuildPDBs();
		
		//glClearColor(0.99, 0.99, 0.99, 1.0);
		InstallFrameHandler(MyFrameHandler, windowID, 0);

		//	kTreeView = 0,
		//	kTextView = 1,
		//	kStateView = 2,
		//	kPDB1View = 3,
		//	kPDB2View = 4,
		//	kPDB3View = 5

		ReinitViewports(windowID, {-1, -1, 0, 1}, kScaleToSquare); // kTreeView
		AddViewport(windowID, {0, -1, 1, 1}, kScaleToSquare); // kTextView
		AddViewport(windowID, {0.25, 0, 0.75, 1}, kScaleToSquare); // kStateView
		AddViewport(windowID, {0, -0.666f+0.2f, 0.333f, 0.2f}, kScaleToSquare); // kPDB1View
		AddViewport(windowID, {0.333f, -0.666f+0.2f, 0.666f, 0.2f}, kScaleToSquare); // kPDB2View
		AddViewport(windowID, {0.666f, -0.666f+0.2f, 1, 0.2f}, kScaleToSquare); // kPDB3View

		g = new Graph();
		ge = new GraphEnvironment(g);
		ge->SetDrawNodeLabels(true);
		
		//mnp.SetWeighted(kUnitPlusFrac);
		BuildGraphFromPuzzle();
		ida.InitializeSearch(ge, from, 0, &h, path);
		RedrawTextDisplay();
		//MyDisplayHandler(windowID, kNoModifier, 'o');
		ge->SetNodeScale(20);

	}
}

int frameCnt = 0;

void DrawPDB(Graphics::Display &display, PermutationPDB<MNPuzzleState<3, 2>, slideDir, MNPuzzle<3, 2>> *pdb)
{
	display.FillRect({-1, -1, 1, 1}, Colors::black);
	if (hit != -1) // node selected
	{
		mnp.GetStateFromHash(t1, hit);
		uint64_t h1 = pdb->GetPDBHash(t1);
		pdb->GetStateFromPDBHash(h1, t1);
		mnp.Draw(display, t1);
	}
	else { // no node selected
		if (timer < 1)
			timer += rate;
		if (timer < 1 && currentPath.size() > 0 && lastPath.size() > 0)
		{
			mnp.GetStateFromHash(t1, currentPath.back());
			mnp.GetStateFromHash(t2, lastPath.back());
			uint64_t h1 = pdb->GetPDBHash(t1);
			uint64_t h2 = pdb->GetPDBHash(t2);
			pdb->GetStateFromPDBHash(h1, t1);
			pdb->GetStateFromPDBHash(h2, t2);
			mnp.Draw(display, t1, t2, timer);
		}
		else // no animation in progres
		{
			mnp.GetStateFromHash(t1, ida.GetCurrentState());
			uint64_t h1 = pdb->GetPDBHash(t1);
			pdb->GetStateFromPDBHash(h1, t1);
			mnp.Draw(display, t1);
		}
	}

}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
//	display.FrameRect({-1, -1, 1, 1}, Colors::white, 1.0);


	if (viewport == kTreeView)
	{
		if (timer >= 1 && running)
			MyDisplayHandler(windowID, kNoModifier, 'o');

		display.FillRect({-1, -1, 1, 1}, Colors::white);
		if (ge == 0 || g == 0)
			return;
		ge->SetColor(0.5, 0.5, 1.0);
		// draws edges but not states
		ge->Draw(display);
		
		// draw in bold the search path
		if (currentPath.size() > 0)
		{
			ge->SetColor(Colors::black);
			for (int x = 1; x < currentPath.size(); x++)
			{
				ge->DrawLine(display, currentPath[x-1], currentPath[x], 5);
				ge->Draw(display, currentPath[x-1]);
				//ge->GLDrawLine(path[x-1], path[x]);
			}
		}

		
		ge->SetColor(0.75, 0.75, 1.0);
		double startf = g->GetNode(from)->GetLabelF(GraphSearchConstants::kTemporaryLabel)+h.HCost(from, 0);
		double goalf = g->GetNode(0)->GetLabelF(GraphSearchConstants::kTemporaryLabel);
		for (int x = 0; x < g->GetNumNodes(); x++)
		{
			double cost = g->GetNode(x)->GetLabelF(GraphSearchConstants::kTemporaryLabel)+h.HCost(x, 0);
			rgbColor c = Colors::GetColor(cost, startf, goalf, colorScheme);
			if (hit == x)
			{
				ge->SetColor(c.r, c.g, c.b);
				//ge->SetColor(1, 1, 1);
				ge->SetNodeScale(30);
				ge->Draw(display, x);
				ge->SetNodeScale(20);
			}
			else if (!fgreater(cost, goalf))
			{
				ge->SetColor(c.r, c.g, c.b);
				ge->Draw(display, x);
			}
		}
		
		// draw search states in bold too
		if (currentPath.size() > 0)
		{
			ge->SetColor(Colors::black);
			for (int x = 0; x < currentPath.size(); x++)
			{
				ge->Draw(display, currentPath[x]);
			}
		}

		// draw solution
		if (path.size() > 0)
		{
			//printf("Drawing edge on solution path\n");
			ge->SetColor(0, 1, 0);
			for (int x = 1; x < path.size(); x++)
			{
				ge->DrawLine(display, path[x-1], path[x], 10);
				//ge->GLDrawLine(path[x-1], path[x]);
			}
		}

		ge->SetColor(1.0, 0, 0);
		ida.Draw(display);
	}

	if (viewport == kTextView)
	{
		display.FillRect({-1, -1, 1, 1}, Colors::black);
		te.Draw(display);
	}
	if (viewport == kStateView)
	{
		display.FillRect({-1, -1, 1, 1}, Colors::black);
		if (hit != -1) // node selected
		{
			mnp.GetStateFromHash(t1, hit);
			mnp.Draw(display, t1);
		}
		else { // no node selected
			if (timer < 1)
				timer += rate;
			if (timer < 1 && currentPath.size() > 0 && lastPath.size() > 0)
			{
				mnp.GetStateFromHash(t1, currentPath.back());
				mnp.GetStateFromHash(t2, lastPath.back());
				mnp.Draw(display, t1, t2, timer);
			}
			else // no animation in progres
			{
				mnp.GetStateFromHash(t1, ida.GetCurrentState());
				mnp.Draw(display, t1);
			}
		}
	}

	if (viewport == kPDB1View && pdb1 && (whichHeuristic&0x2))
	{
		DrawPDB(display, pdb1);
	}
	if (viewport == kPDB2View && pdb1 && (whichHeuristic&0x4))
	{
		DrawPDB(display, pdb2);
	}
	if (viewport == kPDB3View && pdb1 && (whichHeuristic&0x8))
	{
		DrawPDB(display, pdb3);
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
	s += to_string_with_precision(ida.GetCurrentFLimit(), displayPrecision);
	te.AddLine(s.c_str());
	s = "Next Limit: ";
	s += to_string_with_precision(ida.GetNextFLimit(), displayPrecision);
	te.AddLine(s.c_str());
	te.AddLine("");
	s = "Current f-cost: ";
	if (currentPath.size() > 0)
	{
		s += to_string_with_precision((h.HCost(currentPath.back(), 0)+ge->GetPathLength(currentPath)), displayPrecision);
		//g->GetNode(currentPath.back())->GetLabelL(GraphSearchConstants::kTemporaryLabel)));
		s += " (g: "+to_string_with_precision(ge->GetPathLength(currentPath), displayPrecision);
		//to_string_with_precision(g->GetNode(currentPath.back())->GetLabelL(GraphSearchConstants::kTemporaryLabel));
		s += " h: "+to_string_with_precision(h.HCost(currentPath.back(), 0), displayPrecision)+")";
	}
	else if (hit != -1)
	{
		s += to_string_with_precision((h.HCost(hit, 0))+g->GetNode(hit)->GetLabelF(GraphSearchConstants::kTemporaryLabel), displayPrecision);
		s += " (g: ";//to_string_with_precision(currentPath.size()-1);
		s += to_string_with_precision(g->GetNode(hit)->GetLabelF(GraphSearchConstants::kTemporaryLabel), displayPrecision);
		s += " h: "+to_string_with_precision(h.HCost(hit, 0), displayPrecision)+")";
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
	s = "Nodes exp: ";
	s += std::to_string(ida.GetNodesExpanded());
	s += " New last iter: ";
	s += std::to_string(ida.GetNewNodesLastIteration());
	te.AddLine(s.c_str());
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 'u':
			if (mnp.GetWeighted() == kUnitWeight)
			{
				mnp.SetWeighted(kUnitPlusFrac);
				displayPrecision = 2;
				submitTextToBuffer("Using weighted puzzle");
			}
			else {
				mnp.SetWeighted(kUnitWeight);
				displayPrecision = 0;
				submitTextToBuffer("Using unweighted puzzle");
			}
			ida.InitializeSearch(ge, from, 0, &h, path);
			RedrawTextDisplay();
			BuildGraphFromPuzzle();
			path.resize(0);
			currentPath.clear();
			path.clear();
			lastPath.clear();
			running = false;
			break;
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
			BuildGraphFromPuzzle();
			ida.InitializeSearch(ge, from, 0, &h, path);
			RedrawTextDisplay();

			currentPath.clear();
			path.clear();
			lastPath.clear();
			//MyDisplayHandler(windowID, kNoModifier, 'o');
		}
			break;
		case '[':
		{
			from = (from+mnp.GetMaxStateHash()-1)%mnp.GetMaxStateHash();
			printf("Root: %lu\n", from);
			BuildGraphFromPuzzle();
			ida.InitializeSearch(ge, from, 0, &h, path);
			RedrawTextDisplay();

			currentPath.clear();
			path.clear();
			lastPath.clear();
			//MyDisplayHandler(windowID, kNoModifier, 'o');
		}
			break;
		case '|':
		{
			ida.InitializeSearch(ge, from, 0, &h, path);
			RedrawTextDisplay();

			BuildGraphFromPuzzle();
			path.resize(0);
			currentPath.clear();
			path.clear();
			lastPath.clear();
			running = false;
			submitTextToBuffer("");
		}
			break;
		case 'r':
			recording = !recording;
			running = true;
			submitTextToBuffer("Running automatically");
			break;
		case '0':
		{
			whichHeuristic = 0;
			ida.InitializeSearch(ge, from, 0, &h, path);
			RedrawTextDisplay();

			currentPath.clear();
			path.clear();
			lastPath.clear();
			RedrawTextDisplay();
		}
			break;
		case '1':
		{
			whichHeuristic ^= 0x1;
			ida.InitializeSearch(ge, from, 0, &h, path);
			RedrawTextDisplay();

			currentPath.clear();
			path.clear();
			lastPath.clear();
			RedrawTextDisplay();
		}
			break;
		case '2':
		{
			whichHeuristic ^= 0x2;
			ida.InitializeSearch(ge, from, 0, &h, path);
			RedrawTextDisplay();

			currentPath.clear();
			path.clear();
			lastPath.clear();
			RedrawTextDisplay();
		}
			break;

		case '3':
		{
			whichHeuristic ^= 0x4;
			ida.InitializeSearch(ge, from, 0, &h, path);
			RedrawTextDisplay();

			currentPath.clear();
			path.clear();
			lastPath.clear();
			RedrawTextDisplay();
		}
			break;

		case '4':
		{
			whichHeuristic ^= 0x8;
			ida.InitializeSearch(ge, from, 0, &h, path);
			RedrawTextDisplay();

			currentPath.clear();
			path.clear();
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
			if (running)
				submitTextToBuffer("Running automatically");
			else
				submitTextToBuffer("");

			break;
		case 'o':
		{
			ida.GetCurrentPath(lastPath);
			bool done = ida.DoSingleSearchStep(path);
			if (done && running)
				running = false;
			ida.GetCurrentPath(currentPath);
			timer = 0;
			RedrawTextDisplay();
		}
			break;
		case '?':
		{
		}
			break;
		case 'c':
		{
			if (compressedPDBs)
			{
				BuildPDBs();
				compressedPDBs = false;
			}
			else {
				if (pdb1)
					pdb1->DivCompress(2, false);
				if (pdb2)
					pdb2->DivCompress(2, false);
				if (pdb3)
					pdb3->DivCompress(2, false);
				compressedPDBs = true;
			}
			if (compressedPDBs)
				submitTextToBuffer("PDBs are compressed");
			else
				submitTextToBuffer("PDBs are NOT compressed");
		}
		default:
			break;
	}
	
}

void BuildGraphFromPuzzle()
{
	uint64_t maxVal = from;
	g->Reset();
	for (uint64_t x = 0; x < mnp.GetMaxStateHash(); x++)
	{
		node *n;
		g->AddNode(n = new node(""));
		n->SetLabelF(GraphSearchConstants::kTemporaryLabel, -1);
	}
	for (uint64_t x = 0; x < mnp.GetMaxStateHash(); x++)
	{
		mnp.GetStateFromHash(start, x);
		mnp.GetActions(start, acts);
		for (auto a : acts)
		{
			double cost = mnp.GCost(start, a);
			mnp.ApplyAction(start, a);
			uint64_t child = mnp.GetStateHash(start);
			if (g->FindEdge(x, child) == 0)
			{
				g->AddEdge(new edge(x, child, cost));
				//printf("Adding edge between %d and %d [%f]\n", x, child, cost);
			}
			mnp.UndoAction(start, a);
		}
	}
	std::vector<int> distribution;
	g->GetNode(maxVal)->SetLabelF(GraphSearchConstants::kTemporaryLabel, 0);
	std::deque<int> bfsQueue;
	bfsQueue.push_front(maxVal);
	while (bfsQueue.size() > 0)
	{
		node *n = g->GetNode(bfsQueue.back());
		bfsQueue.pop_back();
		int depth = n->GetLabelF(GraphSearchConstants::kTemporaryLabel);
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
			
			if (g->GetNode(neighbor)->GetLabelF(GraphSearchConstants::kTemporaryLabel) == -1)
			{
				g->GetNode(neighbor)->SetLabelF(GraphSearchConstants::kTemporaryLabel, n->GetLabelF(GraphSearchConstants::kTemporaryLabel)+e->GetWeight());
				bfsQueue.push_front(neighbor);
			}
		}
	}
	
	for (uint64_t x = 0; x < mnp.GetMaxStateHash(); x++)
	{
		node *n = g->GetNode(x);
		long depth = n->GetLabelF(GraphSearchConstants::kTemporaryLabel);
		long location = n->GetLabelL(GraphSearchConstants::kFirstData);
		//printf("%d at depth %ld loc %ld\n", n->GetNum(), depth, location);
		n->SetLabelF(GraphSearchConstants::kXCoordinate, ((2.0*(location+1.0))/(distribution[depth]+1.0)-1.0)*0.9);
		n->SetLabelF(GraphSearchConstants::kYCoordinate, (-1.0+2.0*depth/distribution.size())*0.9);
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
			if (running)
				submitTextToBuffer("Stopped running.");
			running = false;
			ida.InitializeSearch(ge, from, 0, &h, path);
			RedrawTextDisplay();

			currentPath.clear();
			path.clear();
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

