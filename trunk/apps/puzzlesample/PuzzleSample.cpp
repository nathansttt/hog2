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
#include "MNPuzzle.h"
#include "FlipSide.h"
#include "IDAStar.h"
#include "Timer.h"
#include "RubiksCubeEdges.h"
#include "RubiksCubeCorners.h"

void BuildSTP_PDB(unsigned long windowID, tKeyboardModifier , char);
void STPTest(unsigned long , tKeyboardModifier , char);

MNPuzzle *mnp = 0;

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

	InstallKeyboardHandler(STPTest, "STP Test", "Test the STP PDBs", kNoModifier, 'd');
	InstallKeyboardHandler(BuildSTP_PDB, "Build STP PDBs", "Build PDBs for the STP", kNoModifier, 'a');

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
		if (mnp == 0)
			mnp = new MNPuzzle(4, 4);
	}
}

MNPuzzleState s(4,4), t(4, 4);
std::vector<slideDir> moves;
double v = 1;
slideDir lastMove = kUp;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	v += 0.1;
	if (v > 1)
	{
		s = t;
		mnp->GetActions(t, moves);
		slideDir tmp = kUp;
		do {
			tmp = moves[random()%moves.size()];
		} while (tmp == lastMove);
		mnp->ApplyAction(t, tmp);
		lastMove = tmp;
		mnp->InvertAction(lastMove);
		v = 0;
	}
	mnp->OpenGLDraw(t, s, v);

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
		{
		}
			break;
		case '1':
		{
		}
			break;
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
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
			if (!mnp) break;
			if (mod == kShiftDown)
			{
				IDAStar<MNPuzzleState, slideDir> ida;
				std::vector<slideDir> path1;
				std::vector<MNPuzzleState> path2;
				MNPuzzleState s(4, 4);
				MNPuzzleState g(4, 4);
				for (unsigned int x = 0; x < 500; x++)
				{
					std::vector<slideDir> acts;
					mnp->GetActions(s, acts);
					mnp->ApplyAction(s, acts[random()%acts.size()]);
				}
				std::cout << "Searching from: " << std::endl << s << std::endl << g << std::endl;
				Timer t;
				t.StartTimer();
				ida.GetPath(mnp, s, g, path1);
				t.EndTimer();
				std::cout << "Path found, length " << path1.size() << " time:" << t.GetElapsedTime() << std::endl;
//				t.StartTimer();
//				ida.GetPath(mnp, s, g, path2);
//				t.EndTimer();
//				std::cout << "Path found, length " << path2.size() << " time:" << t.GetElapsedTime() << std::endl;
//				for (unsigned int x = 0; x < path1.size(); x++)
//					std::cout << path1[x] << std::endl;
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

void BuildSTP_PDB(unsigned long windowID, tKeyboardModifier , char)
{
	MNPuzzleState tmp(4, 4);
	mnp->StoreGoal(tmp);

	std::vector<int> tiles;

	tmp.Reset();
	tiles.resize(0);
	tiles.push_back(1);
	tiles.push_back(4);
	tiles.push_back(5);
	mnp->Build_Additive_PDB(tmp, tiles, "/Users/nathanst/Desktop/STP_145_add.pdb", true);

	tmp.Reset();
	tiles.resize(0);
	tiles.push_back(2);
	tiles.push_back(3);
	tiles.push_back(6);
	tiles.push_back(7);
	mnp->Build_Additive_PDB(tmp, tiles, "/Users/nathanst/Desktop/STP_2367_add.pdb", true);

	tmp.Reset();
	tiles.resize(0);
	for (int x = 1; x <= 7; x++)
		tiles.push_back(x);
//	tiles.push_back(0);
//	tiles.push_back(10);
//	tiles.push_back(11);
//	tiles.push_back(14);
//	tiles.push_back(15);
//	tiles.push_back(7);
//	tiles.push_back(13);
//	tiles.push_back(14);
//	tiles.push_back(15);
	
	
	//mnp->Build_Additive_PDB(tmp, tiles, "/Users/nathanst/Desktop/STP_1-7_add.pdb", true);
	mnp->Build_Additive_PDB(tmp, tiles, "/Users/nathanst/Desktop/STP_1-7+145+2367_add.pdb", true);
	//mnp->Build_Additive_PDB(tmp, tiles, "/Users/nathanst/Desktop/STP_8-15_add.pdb", true);
	//mnp->Build_Regular_PDB(tmp, tiles, "/Users/nathanst/Desktop/pdb7-nosub-fringe.bin");
}

void STPTest(unsigned long , tKeyboardModifier , char)
{
	MNPuzzleState tmp(4, 4);
	mnp->StoreGoal(tmp);

	std::vector<int> tiles;

	if (mnp->PDB.size() == 0)
	{
		tiles.resize(0);
		tiles.push_back(1);
		tiles.push_back(2);
		tiles.push_back(3);
		tiles.push_back(4);
		tiles.push_back(5);
		tiles.push_back(6);
		tiles.push_back(7);
//		mnp->Load_Additive_PDB(tmp, "/Users/nathanst/Desktop/STP_1-7_add.pdb");
		mnp->Load_Additive_PDB(tmp, "/Users/nathanst/Desktop/STP_1-7+145+2367_add.pdb");
		
		tiles.resize(0);
		tiles.push_back(1);
		tiles.push_back(4);
		tiles.push_back(5);
		mnp->Load_Additive_PDB(tmp, "/Users/nathanst/Desktop/STP_145_add.pdb");
		
		tiles.resize(0);
		tiles.push_back(2);
		tiles.push_back(3);
		tiles.push_back(6);
		tiles.push_back(7);
		mnp->Load_Additive_PDB(tmp, "/Users/nathanst/Desktop/STP_2367_add.pdb");


		tiles.resize(0);
		tiles.push_back(8);
		tiles.push_back(9);
		tiles.push_back(12);
		tiles.push_back(13);
		mnp->Load_Additive_PDB(tmp, "/Users/nathanst/Desktop/STP_891213_add.pdb");

		tiles.resize(0);
		tiles.push_back(10);
		tiles.push_back(11);
		tiles.push_back(14);
		tiles.push_back(15);
		mnp->Load_Additive_PDB(tmp, "/Users/nathanst/Desktop/STP_10111415_add.pdb");
	}
	
	IDAStar<MNPuzzleState, slideDir> ida;
	std::vector<slideDir> path1;
	MNPuzzleState s(4, 4);
	MNPuzzleState g(4, 4);

	//15 2 12 11 14 13 9 5 1 3 8 7 0 10 6 4
	s.puzzle[0] = 15;
	s.puzzle[1] = 2;
	s.puzzle[2] = 12;
	s.puzzle[3] = 11;
	s.puzzle[4] = 14;
	s.puzzle[5] = 13;
	s.puzzle[6] = 9;
	s.puzzle[7] = 5;
	s.puzzle[8] = 1;
	s.puzzle[9] = 3;
	s.puzzle[10] = 8;
	s.puzzle[11] = 7;
	s.puzzle[12] = 0;
	s.puzzle[13] = 10;
	s.puzzle[14] = 6;
	s.puzzle[15] = 4;
	s.blank = 12;
	
//	srandom(81);
//	static int rand = 83;
//	srandom(rand++);
//	printf("Seed: %d\n", rand);
//	for (unsigned int x = 0; x < 500; x++)
//	{
//		std::vector<slideDir> acts;
//		mnp->GetActions(s, acts);
//		mnp->ApplyAction(s, acts[random()%acts.size()]);
//	}
	std::cout << "Searching from: " << std::endl << s << std::endl << g << std::endl;
	Timer t;
	t.StartTimer();
	ida.GetPath(mnp, s, g, path1);
	t.EndTimer();
	std::cout << "Path found, length " << path1.size() << " time:" << t.GetElapsedTime() << std::endl;

	for (int x = 0; x < mnp->histogram.size(); x++)
	{
		printf("%2d  %d\n", x, mnp->histogram[x]);
	}

	mnp->PrintHStats();
	
	
//	static int t = 0;
//	rk_e.GetStateFromHash(t, rk_se);
//	t++;
//	return;
//	uint64_t totNodes = 0;
//	for (int x = 0; x < 362880; x++)
//	{
//		BFS<MNPuzzleState, slideDir> bfs;
//		DFS<MNPuzzleState, slideDir> dfs;
//		DFID<MNPuzzleState, slideDir> dfid2;
//		IDAStar<MNPuzzleState, slideDir> ida;
//		MNPuzzle mnae33(3, 3);
////		MNPuzzle mnae34(3, 4);
////		MNPuzzle mnae55(5, 5);
//		MNPuzzleState mnps33(3, 3);
//		MNPuzzleState mnps33g(3, 3);
////		for (unsigned int x = 0; x < mnps33g.puzzle.size(); x++)
////			if (mnps33g.puzzle[x] != 0)
////				mnps33g.puzzle[x] = 1+(14-mnps33g.puzzle[x])%8;
//
//		mnae33.GetStateFromHash(mnps33, x);
//		if (mnae33.GetParity(mnps33) != mnae33.GetParity(mnps33g))
//			continue;
//		
////		MNPuzzleState mnps34(3, 4);
//		std::vector<MNPuzzleState> s;
//		std::vector<slideDir> s2;
//
////		dfid2.GetPath(&mnae33, mnps33, mnps33, s2);
////		dfid2.GetPath(&mnae34, mnps34, mnps34, s2);
////		bfs.GetPath(&mnae33, mnps33, mnps33, s);
////		bfs.GetPath(&mnae34, mnps34, mnps34, s);
//
//		std::cout << mnps33 << " to " << mnps33g << std::endl;
//		ida.GetPath(&mnae33, mnps33, mnps33g, s2);
//		printf("%d: %lld nodes expanded\n", x, ida.GetNodesExpanded());
//		totNodes += ida.GetNodesExpanded();
////		for (unsigned int x = 0; x < s.size(); x++)
////			std::cout << s[x] << std::endl;
//		
////		std::cout << mnps33 << std::endl << mnps33g << std::endl;
////		ida.GetPath(&mnae33, mnps33g, mnps33, s);
////		printf("%lld nodes expanded\n", ida.GetNodesExpanded());
////		for (unsigned int x = 0; x < s.size(); x++)
////			std::cout << s[x] << std::endl;
//	}
//	printf("%llu total nodes\n", totNodes);
//	exit(0);
//	
//	if (0)
//	{
//		std::vector<NaryState> narypath;
//		DFS<NaryState, NaryAction> dfs1;
//		DFID<NaryState, NaryAction> dfid;
//		NaryTree t1(2, 10);
//		NaryTree t2(3, 10);
//		NaryTree t3(4, 10);
//		NaryState s1 = 0, g1;
//		dfs1.GetPath(&t1, s1, s1, narypath);
//		std::cout << dfs1.GetNodesExpanded() << " total nodes expanded" << std::endl;
//		dfs1.GetPath(&t2, s1, s1, narypath);
//		std::cout << dfs1.GetNodesExpanded() << " total nodes expanded" << std::endl;
//		dfs1.GetPath(&t3, s1, s1, narypath);
//		std::cout << dfs1.GetNodesExpanded() << " total nodes expanded" << std::endl;
//
//		g1 = 2048-2;
//		dfid.GetPath(&t1, s1, g1, narypath);
//		std::cout << dfid.GetNodesExpanded() << " total nodes expanded" << std::endl;
//		g1 = 88573-2;
//		dfid.GetPath(&t2, s1, g1, narypath);
//		std::cout << dfid.GetNodesExpanded() << " total nodes expanded" << std::endl;
//		g1 = 1398101-2;
//		dfid.GetPath(&t3, s1, g1, narypath);
//		std::cout << dfid.GetNodesExpanded() << " total nodes expanded" << std::endl;
//	}
//
//	if (0)
//	{
//		std::vector<SequenceAlignmentState> statePath;
//		BFS<SequenceAlignmentState, SequenceAlignmentAction> bfs;
//		DFS<SequenceAlignmentState, SequenceAlignmentAction> dfs;
//		DFID<SequenceAlignmentState, SequenceAlignmentAction> dfid;
//
//		for (unsigned int x = 1; x < 13; x++)
//		{
//			SequenceAlignment sa10(x);
//			SequenceAlignmentState s1 = 0, g1 = -1;
//
//			bfs.GetPath(&sa10, s1, s1, statePath);
//			std::cout << bfs.GetNodesExpanded() << " total BFS nodes expanded" << std::endl;
//			
//			dfs.GetPath(&sa10, s1, s1, statePath);
//			std::cout << dfs.GetNodesExpanded() << " total DFS nodes expanded" << std::endl;
//			
//			dfid.GetPath(&sa10, s1, g1, statePath);
//			std::cout << dfid.GetNodesExpanded() << " total DFID nodes expanded" << std::endl;
//		}
//	}
}

bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	return false;
}

