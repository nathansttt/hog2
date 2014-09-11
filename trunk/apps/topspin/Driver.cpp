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
#include "PuzzleSample.h"
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include "Plot2D.h"
#include "RandomUnit.h"
#include "TopSpin.h"
#include "IDAStar.h"
#include "Timer.h"

void BuildTS_PDB(unsigned long windowID, tKeyboardModifier , char);
void TSTest(unsigned long , tKeyboardModifier , char);

TopSpin *ts = 0;

bool recording = false;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 640, 640);
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

	InstallKeyboardHandler(TSTest, "TS Test", "Test the TS PDBs", kNoModifier, 'd');
	InstallKeyboardHandler(BuildTS_PDB, "Build TS PDBs", "Build PDBs for the TS", kNoModifier, 'a');

	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	
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
		SetNumPorts(windowID, 1);
		ts = new TopSpin(5, 4);
	}
}

TopSpinState s(5,4), t(5, 4);
std::vector<TopSpinAction> moves;
double v = 1;
TopSpinAction lastMove = 0;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	ts->OpenGLDraw(s);
//	v += 0.1;
//	if (v > 1)
//	{
//		s = t;
//		ts->GetActions(t, moves);
//		TopSpinAction tmp = 0;
//		do {
//			tmp = moves[random()%moves.size()];
//		} while (tmp == lastMove);
//		ts->ApplyAction(t, tmp);
//		lastMove = tmp;
//		ts->InvertAction(lastMove);
//		v = 0;
//	}
//	ts->OpenGLDraw(t, s, v);

	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		static int cnt = 999;
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/%d%d%d%d", (cnt/1000)%10, (cnt/100)%10, (cnt/10)%10, cnt%10);
		SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
		cnt--;
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
		case 'r': recording = !recording; break;
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
			ts->ApplyAction(s, key-'0');
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
			break;
		case 'o':
		{
			if (!ts) break;
			if (mod == kShiftDown)
			{
				IDAStar<TopSpinState, TopSpinAction> ida;
				std::vector<TopSpinAction> path1;
				std::vector<TopSpinState> path2;
				TopSpinState s(4, 4);
				TopSpinState g(4, 4);
				for (unsigned int x = 0; x < 500; x++)
				{
					std::vector<TopSpinAction> acts;
					ts->GetActions(s, acts);
					ts->ApplyAction(s, acts[random()%acts.size()]);
				}
				std::cout << "Searching from: " << std::endl << s << std::endl << g << std::endl;
				Timer t;
				t.StartTimer();
				ida.GetPath(ts, s, g, path1);
				t.EndTimer();
				std::cout << "Path found, length " << path1.size() << " time:" << t.GetElapsedTime() << std::endl;
			}
		}
			break;
		default:
			break;
	}
}

#include "MNAgentPuzzle.h"
#include "SequenceAlignment.h"
#include "BFS.h"
#include "DFID.h"
#include "DFS.h"
#include "NaryTree.h"

void GetTSInstance(const TopSpin &ts, TopSpinState &theState, int which)
{
	srandom(which);
	theState.Reset();
	std::vector<TopSpinAction> acts;
	ts.GetActions(theState, acts);
	for (int x = 0; x < 50; x++)
	{
		ts.ApplyAction(theState, acts[random()%acts.size()]);
	}
}

void BuildTS_PDB(unsigned long windowID, tKeyboardModifier , char)
{
	int N = 16, k = 4;
	TopSpinState start(N, k);
	std::vector<int> tiles;

	{
		TopSpin tse(N, k);

		tiles.resize(0);
		start.Reset();
		for (int x = 0; x < 4; x++)
			tiles.push_back(x);
		
		tse.Build_Regular_PDB(start, tiles, "/Users/nathanst/Desktop/TS_0-3.pdb");
		tse.lookups.push_back({kLeafNode, 0, 0, 0});

		tiles.resize(0);
		start.Reset();
		for (int x = 0; x < 6; x++)
			tiles.push_back(x);
		tse.Build_Regular_PDB(start, tiles, "/Users/nathanst/Desktop/TS_0-5+.pdb");
	}

	{
		TopSpin tse(N, k);
		
		tiles.resize(0);
		start.Reset();
		for (int x = 6; x < 10; x++)
			tiles.push_back(x);
		
		tse.Build_Regular_PDB(start, tiles, "/Users/nathanst/Desktop/TS_6-9.pdb");
		tse.lookups.push_back({kLeafNode, 0, 0, 0});
		
		tiles.resize(0);
		start.Reset();
		for (int x = 6; x < 12; x++)
			tiles.push_back(x);
		tse.Build_Regular_PDB(start, tiles, "/Users/nathanst/Desktop/TS_6-11+.pdb");
	}

	{
		tiles.resize(0);
		start.Reset();
		for (int x = 12; x < 16; x++)
			tiles.push_back(x);
		TopSpin tse(N, k);
		tse.Build_Regular_PDB(start, tiles, "/Users/nathanst/Desktop/TS_12-15.pdb");
	}
}

void TSTest(unsigned long , tKeyboardModifier , char)
{
	int N = 16, k = 4;
	std::vector<int> tiles;

	TopSpin tse(N, k);
	TopSpinState s(N, k);
	TopSpinState g(N, k);

//	if (ts->PDB.size() == 0)
//	{
//		tse.Load_Regular_PDB("/Users/nathanst/Desktop/TS_0-5.pdb", g, true);
//		tse.Load_Regular_PDB("/Users/nathanst/Desktop/TS_12-15.pdb", g, true);
//		tse.Load_Regular_PDB("/Users/nathanst/Desktop/TS_6-11.pdb", g, true);
//		tse.lookups.push_back({kMaxNode, 3, 1, 0});
//		tse.lookups.push_back({kLeafNode, 0, 0, 0});
//		tse.lookups.push_back({kLeafNode, 0, 0, 1});
//		tse.lookups.push_back({kLeafNode, 0, 0, 2});
//	}

	if (ts->PDB.size() == 0)
	{
		tse.Load_Regular_PDB("/Users/nathanst/Desktop/TS_0-3.pdb", g, true);
		tse.Load_Regular_PDB("/Users/nathanst/Desktop/TS_0-5+.pdb", g, true);
		tse.Load_Regular_PDB("/Users/nathanst/Desktop/TS_6-9.pdb", g, true);
		tse.Load_Regular_PDB("/Users/nathanst/Desktop/TS_6-11+.pdb", g, true);
		tse.Load_Regular_PDB("/Users/nathanst/Desktop/TS_12-15.pdb", g, true);
		tse.lookups.push_back({kMaxNode, 3, 1, 0});
		tse.lookups.push_back({kAddNode, 2, 4, 0});
		tse.lookups.push_back({kAddNode, 2, 6, 0});
		tse.lookups.push_back({kLeafNode, 0, 0, 4});
		tse.lookups.push_back({kLeafNode, 0, 0, 0});
		tse.lookups.push_back({kLeafNode, 0, 0, 1});
		tse.lookups.push_back({kLeafNode, 0, 0, 2});
		tse.lookups.push_back({kLeafNode, 0, 0, 3});
	}

	
	IDAStar<TopSpinState, TopSpinAction> ida;
	std::vector<TopSpinAction> path1;


	tse.SetPruneSuccessors(true);
	for (int x = 0; x < 100; x++)
	{
		GetTSInstance(tse, s, x);
		std::cout << "Problem " << x << std::endl;
		std::cout << "Searching from: " << std::endl << s << std::endl << g << std::endl;
		Timer t;
		t.StartTimer();
		ida.GetPath(&tse, s, g, path1);
		t.EndTimer();
		std::cout << "Path found, length " << path1.size() << " time:" << t.GetElapsedTime() << " ";
		std::cout << ida.GetNodesExpanded() << " nodes expanded" << std::endl;
//		for (int x = 0; x < ts->histogram.size(); x++)
//		{
//			printf("%2d  %d\n", x, ts->histogram[x]);
//		}
		
//		tse.PrintHStats();
	}
}

bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	return false;
}

