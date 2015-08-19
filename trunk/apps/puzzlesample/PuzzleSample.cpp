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
MNPuzzleState GetKorfInstance(int which);
void CompareToMinCompression();
void CompareToSmallerPDB();
void RunStandardTest();
void RunCompressedTest();

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
	//CompareToSmallerPDB();
	//RunStandardTest();
	RunCompressedTest();
	//CompareToMinCompression();
	exit(0);
	MNPuzzleState tmp(4, 4);
	mnp->StoreGoal(tmp);

	std::vector<int> tiles;

	tmp.Reset();
	tiles.resize(0);
	tiles.push_back(0);
	tiles.push_back(1);
	tiles.push_back(4);
	tiles.push_back(5);
	mnp->Build_PDB(tmp, tiles, "/Users/nathanst/Desktop/wSTP_0145a.pdb", std::thread::hardware_concurrency(), false);
	//mnp->Build_PDB(tmp, tiles, "/Users/nathanst/Desktop/wSTP_0145a.pdb", 1, false);

	tmp.Reset();
	tiles.resize(0);
	tiles.push_back(0);
	tiles.push_back(8);
	tiles.push_back(9);
	tiles.push_back(12);
	tiles.push_back(13);
	mnp->Build_PDB(tmp, tiles, "/Users/nathanst/Desktop/wSTP_0891213a.pdb", std::thread::hardware_concurrency(), false);

	tmp.Reset();
	tiles.resize(0);
	tiles.push_back(0);
	tiles.push_back(1);
	tiles.push_back(4);
	tiles.push_back(5);
	tiles.push_back(8);
	tiles.push_back(9);
	tiles.push_back(12);
	tiles.push_back(13);
	//mnp->Build_PDB(tmp, tiles, "/Users/nathanst/Desktop/STD_0145a.pdb", 1, false);
	mnp->Build_PDB(tmp, tiles, "/Users/nathanst/Desktop/wSTP_0-13a.pdb", std::thread::hardware_concurrency(), false);

	//tmp.Reset();
	//mnp->Build_Regular_PDB(tmp, tiles, "/Users/nathanst/Desktop/STD_0145b.pdb");
	exit(0);
//	mnp->Build_Additive_PDB(tmp, tiles, "/Users/nathanst/Desktop/STP_145_add.pdb", true);

	tmp.Reset();
	tiles.resize(0);
	tiles.push_back(2);
	tiles.push_back(3);
	tiles.push_back(6);
	tiles.push_back(7);
//	mnp->Build_Additive_PDB(tmp, tiles, "/Users/nathanst/Desktop/STP_2367_add.pdb", true);

	tmp.Reset();
	tiles.resize(0);
	for (int x = 8; x <= 15; x++)
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
	//mnp->Build_Additive_PDB(tmp, tiles, "/Users/nathanst/Desktop/STP_1-7+145+2367_add.pdb", true);
	mnp->Build_Additive_PDB(tmp, tiles, "/Users/nathanst/Desktop/STP_8-15_add.pdb", true);
	//mnp->Build_Regular_PDB(tmp, tiles, "/Users/nathanst/Desktop/pdb7-nosub-fringe.bin");
}

void STPTest(unsigned long , tKeyboardModifier , char)
{
	MNPuzzleState tmp(4, 4);
	mnp->StoreGoal(tmp);

	IDAStar<MNPuzzleState, slideDir> ida;
	std::vector<slideDir> path1;
	MNPuzzleState s(4, 4);
	MNPuzzleState g(4, 4);
	//ida.SetUseBDPathMax(true);
	Timer t;
	t.StartTimer();
	uint64_t nodes = 0;
	for (int x = 0; x < 100; x++)
	{
		s = GetKorfInstance(x);
		g.Reset();

		std::cout << "Searching from: " << std::endl << s << std::endl << g << std::endl;
		Timer t;
		t.StartTimer();
		ida.GetPath(mnp, s, g, path1);
		t.EndTimer();
		std::cout << "Path found, length " << path1.size() << " time:" << t.GetElapsedTime() << std::endl;
		nodes += ida.GetNodesExpanded();
	}
	printf("%1.2fs elapsed; %llu nodes expanded\n", t.EndTimer(), nodes);
//	for (int x = 0; x < mnp->histogram.size(); x++)
//	{
//		printf("%2d  %d\n", x, mnp->histogram[x]);
//	}
//
//	mnp->PrintHStats();
	
}

#include <sys/stat.h>
bool fileExists(const char *name)
{
	struct stat buffer;
	return (stat(name, &buffer) == 0);
}

void CompareToMinCompression()
{
	MNPuzzleState tmp(4, 4);
	mnp->StoreGoal(tmp);
	mnp->ClearPDBs();
	
	std::vector<int> tiles;
	
	const char *PDB1 = "/Users/nathanst/Desktop/STP_0-1-2-3-4-5-6-7.pdb";
	if (!fileExists(PDB1))
	{
		tmp.Reset();
		tiles.resize(0);
		tiles.push_back(0);
		tiles.push_back(1);
		tiles.push_back(2);
		tiles.push_back(3);
		tiles.push_back(4);
		tiles.push_back(5);
		tiles.push_back(6);
		tiles.push_back(7);
		mnp->Build_PDB(tmp, tiles, PDB1, std::thread::hardware_concurrency(), false);
	}
	else {
		mnp->ClearPDBs();
		tmp.Reset();
		const int factor = 10;
		mnp->Load_Regular_PDB(PDB1, tmp, true);
		mnp->Min_Compress_PDB(0, factor, true);
		//mnp->Load_Regular_PDB_Min_Compressed(PDB1, tmp, factor, true);
		//mnp->Load_Regular_PDB("/Users/nathanst/Desktop/STP_0-1-4-5-8-9-12-13.pdb", tmp, true);

		mnp->ClearPDBs();
		tmp.Reset();
		mnp->Load_Regular_PDB(PDB1, tmp, true);
		mnp->lookups.push_back({kLeafMinCompress, factor, 0, 0});
		mnp->Delta_Compress_PDB(tmp, 1, true);
		exit(0);
	}
	
	const char *PDB2 = "/Users/nathanst/Desktop/STP_0-8-9-10-11-12-13-14-15.pdb";
	if (!fileExists(PDB2))
	{
		tmp.Reset();
		tiles.resize(0);
		tiles.push_back(0);
		tiles.push_back(2);
		tiles.push_back(3);
		tiles.push_back(6);
		tiles.push_back(7);
		tiles.push_back(10);
		tiles.push_back(11);
		tiles.push_back(14);
		tiles.push_back(15);
		mnp->Build_PDB(tmp, tiles, PDB2, std::thread::hardware_concurrency(), false);
	}

	STPTest(0, kNoModifier, 't');
}

void RunStandardTest()
{
	MNPuzzleState tmp(4, 4);
	mnp->StoreGoal(tmp);
	mnp->ClearPDBs();
	
	std::vector<int> tiles;
	
	const char *PDB1 = "/Users/nathanst/Desktop/STP_0-1-2-3-4-5-6-7.pdb";
	if (!fileExists(PDB1))
	{
		tmp.Reset();
		tiles.resize(0);
		tiles.push_back(0);
		tiles.push_back(1);
		tiles.push_back(2);
		tiles.push_back(3);
		tiles.push_back(4);
		tiles.push_back(5);
		tiles.push_back(6);
		tiles.push_back(7);
		mnp->Build_PDB(tmp, tiles, PDB1, std::thread::hardware_concurrency(), false);
	}
	else {
		tmp.Reset();
		mnp->Load_Regular_PDB(PDB1, tmp, true);
	}
	
	const char *PDB2 = "/Users/nathanst/Desktop/STP_0-8-9-10-11-12-13-14-15.pdb";
	if (!fileExists(PDB2))
	{
		tmp.Reset();
		tiles.resize(0);
		tiles.push_back(0);
		tiles.push_back(8);
		tiles.push_back(9);
		tiles.push_back(10);
		tiles.push_back(11);
		tiles.push_back(12);
		tiles.push_back(13);
		tiles.push_back(14);
		tiles.push_back(15);
		mnp->Build_PDB(tmp, tiles, PDB2, std::thread::hardware_concurrency(), false);
	}
	else {
		tmp.Reset();
		mnp->Load_Regular_PDB(PDB2, tmp, true);
	}
	mnp->lookups.push_back({kMaxNode, 2, 1, 0});
	mnp->lookups.push_back({kLeafNode, 2, 0, 0});
	mnp->lookups.push_back({kLeafNode, 2, 0, 1});

	STPTest(0, kNoModifier, 't');
}

void RunCompressedTest()
{
	MNPuzzleState tmp(4, 4);
	mnp->StoreGoal(tmp);
	mnp->ClearPDBs();
	
	std::vector<int> tiles;
	
	const char *PDB2 = "/Users/nathanst/Desktop/STP_0-8-9-10-11-12-13-14-15.pdb";
	if (!fileExists(PDB2))
	{
		tmp.Reset();
		tiles.resize(0);
		tiles.push_back(0);
		tiles.push_back(8);
		tiles.push_back(9);
		tiles.push_back(10);
		tiles.push_back(11);
		tiles.push_back(12);
		tiles.push_back(13);
		tiles.push_back(14);
		tiles.push_back(15);
		mnp->Build_PDB(tmp, tiles, PDB2, std::thread::hardware_concurrency(), false);
		mnp->ClearPDBs();
	}

	const char *PDB3 = "/Users/nathanst/Desktop/STP_0-8-9-12-13.pdb";
	if (!fileExists(PDB3))
	{
		tmp.Reset();
		tiles.resize(0);
		tiles.push_back(0);
		tiles.push_back(8);
		tiles.push_back(9);
		tiles.push_back(12);
		tiles.push_back(13);
		mnp->Build_PDB(tmp, tiles, PDB3, std::thread::hardware_concurrency(), false);
		mnp->ClearPDBs();
	}
	else {
		mnp->Load_Regular_PDB(PDB3, tmp, true);
	}

	const char *PDB4 = "/Users/nathanst/Desktop/STP_0-10-11-14-15.pdb";
	if (!fileExists(PDB4))
	{
		tmp.Reset();
		tiles.resize(0);
		tiles.push_back(0);
		tiles.push_back(10);
		tiles.push_back(11);
		tiles.push_back(14);
		tiles.push_back(15);
		mnp->Build_PDB(tmp, tiles, PDB4, std::thread::hardware_concurrency(), false);
		mnp->ClearPDBs();
	}
	else {
		mnp->Load_Regular_PDB(PDB4, tmp, true);
	}
	
	if (fileExists(PDB2))
	{
		tmp.Reset();
		mnp->lookups.push_back({kMaxNode, 2, 1, 0});
		mnp->lookups.push_back({kLeafNode, 2, 0, 0});
		mnp->lookups.push_back({kLeafNode, 2, 0, 1});
		//mnp->Load_Regular_PDB_as_Delta(PDB2, tmp, true);
		assert(!"Need to update code to use new functions");
	}

	const char *PDB1 = "/Users/nathanst/Desktop/STP_0-1-2-3-4-5-6-7.pdb";
	if (!fileExists(PDB1))
	{
		tmp.Reset();
		tiles.resize(0);
		tiles.push_back(0);
		tiles.push_back(1);
		tiles.push_back(2);
		tiles.push_back(3);
		tiles.push_back(4);
		tiles.push_back(5);
		tiles.push_back(6);
		tiles.push_back(7);
		mnp->Build_PDB(tmp, tiles, PDB1, std::thread::hardware_concurrency(), false);
	}
	else {
		tmp.Reset();
		mnp->Load_Regular_PDB(PDB1, tmp, true);
	}

	mnp->lookups.resize(0);
	// max (2 + max of (0, 1) , 3)
	mnp->lookups.push_back({kMaxNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
	mnp->lookups.push_back({kLeafNode, -0, -0, 3});
	mnp->lookups.push_back({kAddNode, 2, 3, -0}); // max of 2 children starting at 1 in the tree
	mnp->lookups.push_back({kMaxNode, 2, 5, -0}); // max of 2 children starting at 1 in the tree
	mnp->lookups.push_back({kLeafNode, -0, -0, 2});
	mnp->lookups.push_back({kLeafNode, -0, -0, 0});
	mnp->lookups.push_back({kLeafNode, -0, -0, 1});
	
	STPTest(0, kNoModifier, 't');
}

void CompareToSmallerPDB()
{
	MNPuzzleState tmp(4, 4);
	mnp->StoreGoal(tmp);
	mnp->ClearPDBs();
	
	std::vector<int> tiles;
	
	if (!fileExists("/Users/nathanst/Desktop/STP_0-1-4-5-8-9-12-13.pdb"))
	{
		tmp.Reset();
		tiles.resize(0);
		tiles.push_back(0);
		tiles.push_back(1);
		tiles.push_back(4);
		tiles.push_back(5);
		tiles.push_back(8);
		tiles.push_back(9);
		tiles.push_back(12);
		tiles.push_back(13);
		mnp->Build_PDB(tmp, tiles, "/Users/nathanst/Desktop/STP_0-1-4-5-8-9-12-13.pdb",
					   std::thread::hardware_concurrency(), false);
		mnp->ClearPDBs();
	}

	if (!fileExists("/Users/nathanst/Desktop/STP_0-1-4-5-8-9.pdb"))
	{
		tmp.Reset();
		tiles.resize(0);
		tiles.push_back(0);
		tiles.push_back(1);
		tiles.push_back(4);
		tiles.push_back(5);
		tiles.push_back(8);
		tiles.push_back(9);
		mnp->Build_PDB(tmp, tiles, "/Users/nathanst/Desktop/STP_0-1-4-5-8-9.pdb", std::thread::hardware_concurrency(), false);
	}
	else {
		mnp->Load_Regular_PDB("/Users/nathanst/Desktop/STP_0-1-4-5-8-9.pdb", tmp, true);
	}
	
	if (!fileExists("/Users/nathanst/Desktop/STP_0-4-5-8-9-12-13.pdb"))
	{
		tmp.Reset();
		tiles.resize(0);
		tiles.push_back(0);
		tiles.push_back(4);
		tiles.push_back(5);
		tiles.push_back(8);
		tiles.push_back(9);
		tiles.push_back(12);
		tiles.push_back(13);
		mnp->Build_PDB(tmp, tiles, "/Users/nathanst/Desktop/STP_0-4-5-8-9-12-13.pdb", std::thread::hardware_concurrency(), false);
	}
	else {
		mnp->Load_Regular_PDB("/Users/nathanst/Desktop/STP_0-4-5-8-9-12-13.pdb", tmp, true);
	}

	if (fileExists("/Users/nathanst/Desktop/STP_0-1-4-5-8-9-12-13.pdb"))
	{
		tmp.Reset();
//		mnp->lookups.push_back({kLeafDefaultHeuristic, 0, 0, 0});
		mnp->lookups.push_back({kMaxNode, 2, 1, 0});
		mnp->lookups.push_back({kLeafNode, 2, 0, 0});
		mnp->lookups.push_back({kLeafNode, 2, 0, 1});
		//mnp->Load_Regular_PDB_as_Delta("/Users/nathanst/Desktop/STP_0-1-4-5-8-9-12-13.pdb", tmp, true);
		assert(!"Need to update code to use new functions");
	}
	exit(0);
}

bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	return false;
}

MNPuzzleState GetKorfInstance(int which)
{
	int instances[100][16] =
	{{14, 13, 15, 7, 11, 12, 9, 5, 6, 0, 2, 1, 4, 8, 10, 3},
		{13, 5, 4, 10, 9, 12, 8, 14, 2, 3, 7, 1, 0, 15, 11, 6},
		{14, 7, 8, 2, 13, 11, 10, 4, 9, 12, 5, 0, 3, 6, 1, 15},
		{5, 12, 10, 7, 15, 11, 14, 0, 8, 2, 1, 13, 3, 4, 9, 6},
		{4, 7, 14, 13, 10, 3, 9, 12, 11, 5, 6, 15, 1, 2, 8, 0},
		{14, 7, 1, 9, 12, 3, 6, 15, 8, 11, 2, 5, 10, 0, 4, 13},
		{2, 11, 15, 5, 13, 4, 6, 7, 12, 8, 10, 1, 9, 3, 14, 0},
		{12, 11, 15, 3, 8, 0, 4, 2, 6, 13, 9, 5, 14, 1, 10, 7},
		{3, 14, 9, 11, 5, 4, 8, 2, 13, 12, 6, 7, 10, 1, 15, 0},
		{13, 11, 8, 9, 0, 15, 7, 10, 4, 3, 6, 14, 5, 12, 2, 1},
		{5, 9, 13, 14, 6, 3, 7, 12, 10, 8, 4, 0, 15, 2, 11, 1},
		{14, 1, 9, 6, 4, 8, 12, 5, 7, 2, 3, 0, 10, 11, 13, 15},
		{3, 6, 5, 2, 10, 0, 15, 14, 1, 4, 13, 12, 9, 8, 11, 7},
		{7, 6, 8, 1, 11, 5, 14, 10, 3, 4, 9, 13, 15, 2, 0, 12},
		{13, 11, 4, 12, 1, 8, 9, 15, 6, 5, 14, 2, 7, 3, 10, 0},
		{1, 3, 2, 5, 10, 9, 15, 6, 8, 14, 13, 11, 12, 4, 7, 0},
		{15, 14, 0, 4, 11, 1, 6, 13, 7, 5, 8, 9, 3, 2, 10, 12},
		{6, 0, 14, 12, 1, 15, 9, 10, 11, 4, 7, 2, 8, 3, 5, 13},
		{7, 11, 8, 3, 14, 0, 6, 15, 1, 4, 13, 9, 5, 12, 2, 10},
		{6, 12, 11, 3, 13, 7, 9, 15, 2, 14, 8, 10, 4, 1, 5, 0},
		{12, 8, 14, 6, 11, 4, 7, 0, 5, 1, 10, 15, 3, 13, 9, 2},
		{14, 3, 9, 1, 15, 8, 4, 5, 11, 7, 10, 13, 0, 2, 12, 6},
		{10, 9, 3, 11, 0, 13, 2, 14, 5, 6, 4, 7, 8, 15, 1, 12},
		{7, 3, 14, 13, 4, 1, 10, 8, 5, 12, 9, 11, 2, 15, 6, 0},
		{11, 4, 2, 7, 1, 0, 10, 15, 6, 9, 14, 8, 3, 13, 5, 12},
		{5, 7, 3, 12, 15, 13, 14, 8, 0, 10, 9, 6, 1, 4, 2, 11},
		{14, 1, 8, 15, 2, 6, 0, 3, 9, 12, 10, 13, 4, 7, 5, 11},
		{13, 14, 6, 12, 4, 5, 1, 0, 9, 3, 10, 2, 15, 11, 8, 7},
		{9, 8, 0, 2, 15, 1, 4, 14, 3, 10, 7, 5, 11, 13, 6, 12},
		{12, 15, 2, 6, 1, 14, 4, 8, 5, 3, 7, 0, 10, 13, 9, 11},
		{12, 8, 15, 13, 1, 0, 5, 4, 6, 3, 2, 11, 9, 7, 14, 10},
		{14, 10, 9, 4, 13, 6, 5, 8, 2, 12, 7, 0, 1, 3, 11, 15},
		{14, 3, 5, 15, 11, 6, 13, 9, 0, 10, 2, 12, 4, 1, 7, 8},
		{6, 11, 7, 8, 13, 2, 5, 4, 1, 10, 3, 9, 14, 0, 12, 15},
		{1, 6, 12, 14, 3, 2, 15, 8, 4, 5, 13, 9, 0, 7, 11, 10},
		{12, 6, 0, 4, 7, 3, 15, 1, 13, 9, 8, 11, 2, 14, 5, 10},
		{8, 1, 7, 12, 11, 0, 10, 5, 9, 15, 6, 13, 14, 2, 3, 4},
		{7, 15, 8, 2, 13, 6, 3, 12, 11, 0, 4, 10, 9, 5, 1, 14},
		{9, 0, 4, 10, 1, 14, 15, 3, 12, 6, 5, 7, 11, 13, 8, 2},
		{11, 5, 1, 14, 4, 12, 10, 0, 2, 7, 13, 3, 9, 15, 6, 8},
		{8, 13, 10, 9, 11, 3, 15, 6, 0, 1, 2, 14, 12, 5, 4, 7},
		{4, 5, 7, 2, 9, 14, 12, 13, 0, 3, 6, 11, 8, 1, 15, 10},
		{11, 15, 14, 13, 1, 9, 10, 4, 3, 6, 2, 12, 7, 5, 8, 0},
		{12, 9, 0, 6, 8, 3, 5, 14, 2, 4, 11, 7, 10, 1, 15, 13},
		{3, 14, 9, 7, 12, 15, 0, 4, 1, 8, 5, 6, 11, 10, 2, 13},
		{8, 4, 6, 1, 14, 12, 2, 15, 13, 10, 9, 5, 3, 7, 0, 11},
		{6, 10, 1, 14, 15, 8, 3, 5, 13, 0, 2, 7, 4, 9, 11, 12},
		{8, 11, 4, 6, 7, 3, 10, 9, 2, 12, 15, 13, 0, 1, 5, 14},
		{10, 0, 2, 4, 5, 1, 6, 12, 11, 13, 9, 7, 15, 3, 14, 8},
		{12, 5, 13, 11, 2, 10, 0, 9, 7, 8, 4, 3, 14, 6, 15, 1},
		{10, 2, 8, 4, 15, 0, 1, 14, 11, 13, 3, 6, 9, 7, 5, 12},
		{10, 8, 0, 12, 3, 7, 6, 2, 1, 14, 4, 11, 15, 13, 9, 5},
		{14, 9, 12, 13, 15, 4, 8, 10, 0, 2, 1, 7, 3, 11, 5, 6},
		{12, 11, 0, 8, 10, 2, 13, 15, 5, 4, 7, 3, 6, 9, 14, 1},
		{13, 8, 14, 3, 9, 1, 0, 7, 15, 5, 4, 10, 12, 2, 6, 11},
		{3, 15, 2, 5, 11, 6, 4, 7, 12, 9, 1, 0, 13, 14, 10, 8},
		{5, 11, 6, 9, 4, 13, 12, 0, 8, 2, 15, 10, 1, 7, 3, 14},
		{5, 0, 15, 8, 4, 6, 1, 14, 10, 11, 3, 9, 7, 12, 2, 13},
		{15, 14, 6, 7, 10, 1, 0, 11, 12, 8, 4, 9, 2, 5, 13, 3},
		{11, 14, 13, 1, 2, 3, 12, 4, 15, 7, 9, 5, 10, 6, 8, 0},
		{6, 13, 3, 2, 11, 9, 5, 10, 1, 7, 12, 14, 8, 4, 0, 15},
		{4, 6, 12, 0, 14, 2, 9, 13, 11, 8, 3, 15, 7, 10, 1, 5},
		{8, 10, 9, 11, 14, 1, 7, 15, 13, 4, 0, 12, 6, 2, 5, 3},
		{5, 2, 14, 0, 7, 8, 6, 3, 11, 12, 13, 15, 4, 10, 9, 1},
		{7, 8, 3, 2, 10, 12, 4, 6, 11, 13, 5, 15, 0, 1, 9, 14},
		{11, 6, 14, 12, 3, 5, 1, 15, 8, 0, 10, 13, 9, 7, 4, 2},
		{7, 1, 2, 4, 8, 3, 6, 11, 10, 15, 0, 5, 14, 12, 13, 9},
		{7, 3, 1, 13, 12, 10, 5, 2, 8, 0, 6, 11, 14, 15, 4, 9},
		{6, 0, 5, 15, 1, 14, 4, 9, 2, 13, 8, 10, 11, 12, 7, 3},
		{15, 1, 3, 12, 4, 0, 6, 5, 2, 8, 14, 9, 13, 10, 7, 11},
		{5, 7, 0, 11, 12, 1, 9, 10, 15, 6, 2, 3, 8, 4, 13, 14},
		{12, 15, 11, 10, 4, 5, 14, 0, 13, 7, 1, 2, 9, 8, 3, 6},
		{6, 14, 10, 5, 15, 8, 7, 1, 3, 4, 2, 0, 12, 9, 11, 13},
		{14, 13, 4, 11, 15, 8, 6, 9, 0, 7, 3, 1, 2, 10, 12, 5},
		{14, 4, 0, 10, 6, 5, 1, 3, 9, 2, 13, 15, 12, 7, 8, 11},
		{15, 10, 8, 3, 0, 6, 9, 5, 1, 14, 13, 11, 7, 2, 12, 4},
		{0, 13, 2, 4, 12, 14, 6, 9, 15, 1, 10, 3, 11, 5, 8, 7},
		{3, 14, 13, 6, 4, 15, 8, 9, 5, 12, 10, 0, 2, 7, 1, 11},
		{0, 1, 9, 7, 11, 13, 5, 3, 14, 12, 4, 2, 8, 6, 10, 15},
		{11, 0, 15, 8, 13, 12, 3, 5, 10, 1, 4, 6, 14, 9, 7, 2},
		{13, 0, 9, 12, 11, 6, 3, 5, 15, 8, 1, 10, 4, 14, 2, 7},
		{14, 10, 2, 1, 13, 9, 8, 11, 7, 3, 6, 12, 15, 5, 4, 0},
		{12, 3, 9, 1, 4, 5, 10, 2, 6, 11, 15, 0, 14, 7, 13, 8},
		{15, 8, 10, 7, 0, 12, 14, 1, 5, 9, 6, 3, 13, 11, 4, 2},
		{4, 7, 13, 10, 1, 2, 9, 6, 12, 8, 14, 5, 3, 0, 11, 15},
		{6, 0, 5, 10, 11, 12, 9, 2, 1, 7, 4, 3, 14, 8, 13, 15},
		{9, 5, 11, 10, 13, 0, 2, 1, 8, 6, 14, 12, 4, 7, 3, 15},
		{15, 2, 12, 11, 14, 13, 9, 5, 1, 3, 8, 7, 0, 10, 6, 4},
		{11, 1, 7, 4, 10, 13, 3, 8, 9, 14, 0, 15, 6, 5, 2, 12},
		{5, 4, 7, 1, 11, 12, 14, 15, 10, 13, 8, 6, 2, 0, 9, 3},
		{9, 7, 5, 2, 14, 15, 12, 10, 11, 3, 6, 1, 8, 13, 0, 4},
		{3, 2, 7, 9, 0, 15, 12, 4, 6, 11, 5, 14, 8, 13, 10, 1},
		{13, 9, 14, 6, 12, 8, 1, 2, 3, 4, 0, 7, 5, 10, 11, 15},
		{5, 7, 11, 8, 0, 14, 9, 13, 10, 12, 3, 15, 6, 1, 4, 2},
		{4, 3, 6, 13, 7, 15, 9, 0, 10, 5, 8, 11, 2, 12, 1, 14},
		{1, 7, 15, 14, 2, 6, 4, 9, 12, 11, 13, 3, 0, 8, 5, 10},
		{9, 14, 5, 7, 8, 15, 1, 2, 10, 4, 13, 6, 12, 0, 11, 3},
		{0, 11, 3, 12, 5, 2, 1, 9, 8, 10, 14, 15, 7, 4, 13, 6},
		{7, 15, 4, 0, 10, 9, 2, 5, 12, 11, 13, 6, 1, 3, 14, 8},
		{11, 4, 0, 8, 6, 10, 5, 13, 12, 7, 14, 3, 1, 2, 9, 15}};
	
	MNPuzzleState s(4,4);
	for (int x = 0; x < 16; x++)
	{
		s.puzzle[x] = instances[which][x];
		if (s.puzzle[x] == 0)
			s.blank = x;
	}
	return s;
}
