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
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include "Plot2D.h"
#include "RandomUnit.h"
#include "TopSpin.h"
#include "IDAStar.h"
#include "Timer.h"

void CompareToMinCompression();
void CompareToSmallerPDB();

void BuildTS_PDB(unsigned long windowID, tKeyboardModifier , char);
void TSTest(unsigned long , tKeyboardModifier , char);
TopSpinState GetInstance(int which);
void Test(TopSpin &tse);
void BaselineTest();
void BaselineTest2();
void LosslessTest();
void MinCompressionTest();
void Delta6CompressionTest();
void Delta7CompressionTest();
void Delta7ValueCompressionTest();
void DeltaMinCompressionTest();
void MeasureIR(TopSpin &tse);
void GetBitValueCutoffs(std::vector<int> &cutoffs, int bits);

void BitDeltaValueCompressionTest();
void ModValueCompressionTest();
void ModValueDeltaCompressionTest();
void DivValueCompressionTest();
void DivDeltaValueCompressionTest();

void FractionalNodesCompressionTest();
void FractionalModNodesCompressionTest();
void BitDeltaNodesCompressionTest();
void ModNodesCompressionTest();
void ModNodesDeltaCompressionTest();
void DivNodesCompressionTest();
void DivDeltaNodesCompressionTest();

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

	InstallCommandLineHandler(MyCLHandler, "-run", "-run", "Runs pre-set experiments.");
	
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
	BuildTS_PDB(0, kNoModifier, 'a');
	exit(0);
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
//	ModValueDeltaCompressionTest();
//	DivDeltaValueCompressionTest();
//	ModValueCompressionTest();
//	DivValueCompressionTest();
//	BitDeltaValueCompressionTest();

	FractionalModNodesCompressionTest();
	FractionalNodesCompressionTest();
//BitDeltaNodesCompressionTest();
//	DivNodesCompressionTest();
//	ModNodesCompressionTest();

	exit(0);
	
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

#include <sys/stat.h>
bool fileExists(const char *name)
{
	struct stat buffer;
	return (stat(name, &buffer) == 0);
}

#pragma mark pdb building code
const int N = 16, k = 4;
const char *pdb7a = "/Users/nathanst/Desktop/TS-0-6.pdb";
const char *pdb8a = "/Users/nathanst/Desktop/TS-0-7.pdb";

const char *pdb7b = "/Users/nathanst/Desktop/TS-8-14.pdb";
const char *pdb8b = "/Users/nathanst/Desktop/TS-8-15.pdb";

void BuildPDBs(bool aPDBs, bool bPDBs)
{
	TopSpin tse(N, k);
	std::vector<int> tiles;
	
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();

	if (aPDBs)
	{
		if (!fileExists(pdb7a))
		{
			g.Reset();
			tiles.resize(0);
			//for (int x = 0; x <= 12; x+=2)
			for (int x = 0; x <= 6; x++)
				tiles.push_back(x);
			
			tse.Build_PDB(g, tiles, pdb7a,
						  std::thread::hardware_concurrency(), false);
			tse.ClearPDBs();
		}
		
		if (!fileExists(pdb8a))
		{
			g.Reset();
			tiles.resize(0);
			//for (int x = 0; x <= 14; x+=2)
			for (int x = 0; x <= 7; x++)
				tiles.push_back(x);
			
			tse.Build_PDB(g, tiles, pdb8a,
						  std::thread::hardware_concurrency(), false);
			tse.ClearPDBs();
		}
	}
	
	if (bPDBs)
	{
		if (!fileExists(pdb7b))
		{
			g.Reset();
			tiles.resize(0);
			for (int x = 8; x <= 14; x++)
				//for (int x = 1; x <= 14; x+=2)
				tiles.push_back(x);
			
			tse.Build_PDB(g, tiles, pdb7b,
						  std::thread::hardware_concurrency(), false);
			tse.ClearPDBs();
		}
		
		if (!fileExists(pdb8b))
		{
			g.Reset();
			tiles.resize(0);
			for (int x = 8; x <= 15; x++)
			//for (int x = 1; x <= 15; x+=2)
				tiles.push_back(x);
			
			tse.Build_PDB(g, tiles, pdb8b,
						  std::thread::hardware_concurrency(), false);
			tse.ClearPDBs();
		}
	}
}

#pragma mark heuristic value tests

void ModValueCompressionTest()
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	BuildPDBs(true, false);
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Compressing by factor of %d\n", x);
		tse.ClearPDBs();
		//tse.Load_Regular_PDB(pdb7a, g, true);
		uint64_t oldSize = tse.Get_PDB_Size(g, 8);
		uint64_t newSize = oldSize / x;
		tse.Load_Regular_PDB(pdb8a, g, true);
		tse.Mod_Compress_PDB(0, newSize, true);
		tse.lookups.push_back({kLeafModCompress, -0, -0, -0});
		MeasureIR(tse);
	}
}

void ModValueDeltaCompressionTest()
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	BuildPDBs(true, false);
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>MOD Compressing by factor of %d\n", x);
		tse.ClearPDBs();
		tse.Load_Regular_PDB(pdb7a, g, true);
		tse.lookups.push_back({kLeafNode, -0, -0, 0});

		uint64_t oldSize = tse.Get_PDB_Size(g, 8);
		uint64_t newSize = oldSize / x;
		tse.Load_Regular_PDB(pdb8a, g, true);
		tse.Delta_Compress_PDB(g, 1, true);
		tse.Mod_Compress_PDB(1, newSize, true);
//		tse.lookups.push_back({kLeafModCompress, -0, -0, -0});
//		MeasureIR(tse);
	}
}

void DivValueCompressionTest()
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	if (!fileExists(pdb7a))
	{
		g.Reset();
		tiles.resize(0);
		for (int x = 0; x <= 6; x++)
			tiles.push_back(x);
		
		tse.Build_PDB(g, tiles, pdb7a,
					  std::thread::hardware_concurrency(), false);
		tse.ClearPDBs();
	}
	
	if (!fileExists(pdb8a))
	{
		g.Reset();
		tiles.resize(0);
		for (int x = 0; x < 8; x++)
			tiles.push_back(x);
		
		tse.Build_PDB(g, tiles, pdb8a,
					  std::thread::hardware_concurrency(), false);
		tse.ClearPDBs();
	}
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Compressing by factor of %d\n", x);
		tse.ClearPDBs();
		tse.Load_Regular_PDB(pdb8a, g, true);
		tse.Min_Compress_PDB(0, x, true);
		tse.lookups.push_back({kLeafMinCompress, static_cast<uint8_t>(x), -0, 0});
		MeasureIR(tse);
	}
}

void DivDeltaValueCompressionTest()
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	if (!fileExists(pdb7a))
	{
		g.Reset();
		tiles.resize(0);
		for (int x = 0; x <= 6; x++)
			tiles.push_back(x);
		
		tse.Build_PDB(g, tiles, pdb7a,
					  std::thread::hardware_concurrency(), false);
		tse.ClearPDBs();
	}
	
	if (!fileExists(pdb8a))
	{
		g.Reset();
		tiles.resize(0);
		for (int x = 0; x < 8; x++)
			tiles.push_back(x);
		
		tse.Build_PDB(g, tiles, pdb8a,
					  std::thread::hardware_concurrency(), false);
		tse.ClearPDBs();
	}
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Compressing by factor of %d\n", x);
		tse.ClearPDBs();
		tse.Load_Regular_PDB(pdb7a, g, false);
		tse.lookups.push_back({kLeafNode, -0, -0, 0});

		tse.Load_Regular_PDB(pdb8a, g, false);
		tse.Delta_Compress_PDB(g, 1, true);
		tse.Min_Compress_PDB(1, x, true);
		//MeasureIR(tse);
	}
}

void BitDeltaValueCompressionTest()
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	BuildPDBs(true, false);
	
	for (int x = 1; x <= 4; x*=2)
	{
		g.Reset();
		printf("==>Compressing to %d bits\n", x);
		tse.ClearPDBs();
		tse.Load_Regular_PDB(pdb7a, g, false);
		tse.lookups.push_back({kLeafNode, -0, -0, 0});
		
		tse.Load_Regular_PDB(pdb8a, g, false);
		tse.Delta_Compress_PDB(g, 1, true);
		std::vector<int> cutoffs;
		GetBitValueCutoffs(cutoffs, x);
		tse.Value_Compress_PDB(1, cutoffs, true);
		//MeasureIR(tse);
	}
}

#pragma mark node expansion tests

//Fractional_Mod_Compress_PDB
void FractionalNodesCompressionTest()
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	BuildPDBs(true, true);
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Fractional (contiguous): Reducing by factor of %d\n", x);
		tse.ClearPDBs();
		uint64_t oldSize = tse.Get_PDB_Size(g, 8);
		uint64_t newSize = oldSize / x;
		tse.Load_Regular_PDB(pdb7a, g, true);
		tse.Load_Regular_PDB(pdb8a, g, true);
		tse.Load_Regular_PDB(pdb7b, g, true);
		tse.Load_Regular_PDB(pdb8b, g, true);
		tse.Fractional_Compress_PDB(1, newSize, true);
		tse.Fractional_Compress_PDB(3, newSize, true);
		tse.lookups.push_back({kMaxNode, 4, 1, -0}); // max of 2 children starting at 1 in the tree
		tse.lookups.push_back({kLeafNode, -0, -0, 0});
		tse.lookups.push_back({kLeafFractionalCompress, -0, -0, 1});
		tse.lookups.push_back({kLeafNode, -0, -0, 2});
		tse.lookups.push_back({kLeafFractionalCompress, -0, -0, 3});
		Test(tse);
		return;
	}
}

void FractionalModNodesCompressionTest()
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	BuildPDBs(true, true);
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Fractional MOD: Reducing by factor of %d\n", x);
		tse.ClearPDBs();
		tse.Load_Regular_PDB(pdb7a, g, true);
		tse.Load_Regular_PDB(pdb8a, g, true);
		tse.Load_Regular_PDB(pdb7b, g, true);
		tse.Load_Regular_PDB(pdb8b, g, true);
		tse.Fractional_Mod_Compress_PDB(1, x, true);
		tse.Fractional_Mod_Compress_PDB(3, x, true);
		tse.lookups.push_back({kMaxNode, 4, 1, -0}); // max of 2 children starting at 1 in the tree
		tse.lookups.push_back({kLeafNode, -0, -0, 0});
		tse.lookups.push_back({kLeafFractionalModCompress, static_cast<uint8_t>(x), -0, 1}); // factor, -- , id
		tse.lookups.push_back({kLeafNode, -0, -0, 2});
		tse.lookups.push_back({kLeafFractionalModCompress, static_cast<uint8_t>(x), -0, 3});
		Test(tse);
	}
}


void ModNodesCompressionTest()
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	BuildPDBs(true, true);
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Compressing by factor of %d\n", x);
		tse.ClearPDBs();
		uint64_t oldSize = tse.Get_PDB_Size(g, 8);
		uint64_t newSize = oldSize / x;
		tse.Load_Regular_PDB(pdb8a, g, true);
		tse.Load_Regular_PDB(pdb8b, g, true);
		tse.Mod_Compress_PDB(0, newSize, true);
		tse.Mod_Compress_PDB(1, newSize, true);
		tse.lookups.push_back({kMaxNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
		tse.lookups.push_back({kLeafModCompress, -0, -0, 0});
		tse.lookups.push_back({kLeafModCompress, -0, -0, 1});
		Test(tse);
	}
}

void ModNodesDeltaCompressionTest()
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	BuildPDBs(true, true);
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>MOD Compressing by factor of %d\n", x);
		tse.ClearPDBs();
		tse.Load_Regular_PDB(pdb7a, g, true);
		tse.lookups.push_back({kLeafNode, -0, -0, 0});
		
		uint64_t oldSize = tse.Get_PDB_Size(g, 8);
		uint64_t newSize = oldSize / x;
		tse.Load_Regular_PDB(pdb8a, g, true);
		tse.Delta_Compress_PDB(g, 1, true);
		tse.Mod_Compress_PDB(1, newSize, true);
		//		tse.lookups.push_back({kLeafModCompress, -0, -0, -0});
		//		MeasureIR(tse);
	}
}

void DivNodesCompressionTest()
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	BuildPDBs(true, true);
	
	for (int x = 1; x <= 10; x++)
	{
		g.Reset();
		printf("==>Compressing by factor of %d\n", x);
		tse.ClearPDBs();
		tse.Load_Regular_PDB(pdb8a, g, true);
		tse.Min_Compress_PDB(0, x, true);
		tse.Load_Regular_PDB(pdb8b, g, true);
		tse.Min_Compress_PDB(1, x, true);

		tse.lookups.push_back({kMaxNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
		tse.lookups.push_back({kLeafMinCompress, static_cast<uint8_t>(x), -0, 0});
		tse.lookups.push_back({kLeafMinCompress, static_cast<uint8_t>(x), -0, 1});

		Test(tse);
		//MeasureIR(tse);
	}
}

void DivDeltaNodesCompressionTest()
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	BuildPDBs(true, true);
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Compressing by factor of %d\n", x);
		tse.ClearPDBs();
		tse.Load_Regular_PDB(pdb7a, g, false);
		tse.lookups.push_back({kLeafNode, -0, -0, 0});
		
		tse.Load_Regular_PDB(pdb8a, g, false);
		tse.Delta_Compress_PDB(g, 1, true);
		tse.Min_Compress_PDB(1, x, true);
		//MeasureIR(tse);
	}
}


void BitDeltaNodesCompressionTest()
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	BuildPDBs(true, true);
	
	for (int x = 1; x <= 4; x*=2)
	{
		std::vector<int> cutoffs;
		GetBitValueCutoffs(cutoffs, x);

		g.Reset();
		printf("==>Compressing to %d bits\n", x);
		tse.ClearPDBs();
		tse.Load_Regular_PDB(pdb7a, g, false); // PDB 0
		tse.Load_Regular_PDB(pdb8a, g, false); // PDB 1
		tse.lookups.push_back({kLeafNode, -0, -0, 0});
		tse.Delta_Compress_PDB(g, 1, true);
		tse.Value_Compress_PDB(1, cutoffs, true);
		
		tse.Load_Regular_PDB(pdb7b, g, false); // PDB 2
		tse.Load_Regular_PDB(pdb8b, g, false); // PDB 3
		tse.lookups.back().PDBID = 2;
		tse.Delta_Compress_PDB(g, 3, true);
		tse.Value_Compress_PDB(3, cutoffs, true);


		tse.lookups.resize(0);
		tse.lookups.push_back({kMaxNode, 2, 1, -0});
		tse.lookups.push_back({kAddNode, 2, 3, -0});
		tse.lookups.push_back({kAddNode, 2, 5, -0});
		tse.lookups.push_back({kLeafNode, -0, -0, 0});
		tse.lookups.push_back({kLeafNode, -0, -0, 1});
		tse.lookups.push_back({kLeafNode, -0, -0, 2});
		tse.lookups.push_back({kLeafNode, -0, -0, 3});

		Test(tse);
	}
}

#pragma mark other utilities

bool UsingWeighted()
{
	TopSpin tse(N, k);
	TopSpinState g(N, 4);
	return (tse.GCost(g, 1) != tse.GCost(g, 2));
}

void GetBitValueCutoffs(std::vector<int> &cutoffs, int bits)
{
	TopSpin tse(N, k);
	TopSpinState g(N, k);
	g.Reset();
	
	if (tse.GCost(g, 1) != tse.GCost(g, 2)) // non-uniform costs
	{
		switch (bits)
		{
			case 1:
			{
				cutoffs.push_back(0);
				cutoffs.push_back(13);
				//cutoffs.push_back(14);
				cutoffs.push_back(1000); // higher than max value
			} break;
			case 2:
			{
				cutoffs.push_back(0);
				cutoffs.push_back(9);
				cutoffs.push_back(16);
				cutoffs.push_back(22);
//				cutoffs.push_back(0);
//				cutoffs.push_back(10);
//				cutoffs.push_back(17);
//				cutoffs.push_back(23);
				cutoffs.push_back(1000); // higher than max value
			} break;
			case 4:
			{
				cutoffs.push_back(0);
				cutoffs.push_back(2);
				cutoffs.push_back(4);
				cutoffs.push_back(6);
				cutoffs.push_back(8);
				cutoffs.push_back(10);
				cutoffs.push_back(12);
				cutoffs.push_back(14);
				cutoffs.push_back(16);
				cutoffs.push_back(18);
				cutoffs.push_back(20);
				cutoffs.push_back(22);
				cutoffs.push_back(24);
				cutoffs.push_back(26);
				cutoffs.push_back(29);
				cutoffs.push_back(34);
				cutoffs.push_back(1000); // higher than max value
			} break;
			default: printf("Unknown bits\n"); exit(0);
		}
	}
	else {
		for (int x = 0; x <= bits; x++)
			cutoffs.push_back(x);
		cutoffs.push_back(1000);
	}
}

uint64_t random64()
{
	uint64_t r1, r2;
	r1 = random();
	r2 = random();
	return (r1<<32)|r2;
}

void MeasureIR(TopSpin &tse)
{
	srandom(1234);
	TopSpinState s(16, 4);
	TopSpinState g(16, 4);
	g.Reset();
	tse.StoreGoal(g);
	//tse.SetPruneSuccessors(true);
	
	IDAStar<TopSpinState, TopSpinAction> ida;
	std::vector<TopSpinAction> path1;
	TopSpinState start;
	Timer t;
	t.StartTimer();
	uint64_t nodes = 0;

	uint64_t count = tse.Get_PDB_Size(s, 16);
	double sumg = 0, sumh = 0;
	int total = 1000000;
	for (int x = 0; x < total; x++)
	{
		s.Reset();
		tse.GetStateFromHash(s, random64()%count);
		tse.GetNextState(s, random()%16, g);
		
		sumg += tse.GCost(s, g);
		sumh += fabs(tse.PermutationPuzzleEnvironment<TopSpinState,TopSpinAction>::HCost(s)-
					 tse.PermutationPuzzleEnvironment<TopSpinState,TopSpinAction>::HCost(g));
//		printf("G: %1.0f ∆H: %1.0f\n", tse.GCost(s, g),
//			   fabs(tse.PermutationPuzzleEnvironment<TopSpinState,TopSpinAction>::HCost(s)-
//					tse.PermutationPuzzleEnvironment<TopSpinState,TopSpinAction>::HCost(g)));
	}
	printf("Average G: %1.1f, average H: %1.3f\n", sumg/total, sumh/total);
}


void Test(TopSpin &tse)
{
	TopSpinState s(16, 4);
	TopSpinState g(16, 4);
	g.Reset();
	tse.StoreGoal(g);
	tse.SetPruneSuccessors(true);
	
	IDAStar<TopSpinState, TopSpinAction> ida;
	ida.SetUseBDPathMax(true);
	std::vector<TopSpinAction> path1;
	TopSpinState start;
	Timer t1;
	t1.StartTimer();
	uint64_t nodes = 0;
	double totaltime = 0;
	for (int x = 0; x < 100; x++)
	{
		s = GetInstance(x);
		g.Reset();
		printf("Problem %d of %d\n", x+1, 100);
		std::cout << "Searching from: " << std::endl << s << std::endl << g << std::endl;
		Timer t;
		t.StartTimer();
		ida.GetPath(&tse, s, g, path1);
		t.EndTimer();
		totaltime += t.GetElapsedTime();
		std::cout << "Path found, length " << path1.size() << " time:" << t.GetElapsedTime() << std::endl;
		nodes += ida.GetNodesExpanded();
	}
	printf("%1.2fs elapsed; %llu nodes expanded\n", t1.EndTimer(), nodes);
}


TopSpinState GetInstance(int which)
{
	srandom(which*101+11);
	TopSpin tse(16, 4);
	TopSpinState s(16,4);
	s.Reset();
	for (int x = 0; x < 10000; x++)
	{
		tse.ApplyAction(s, random()%16);
	}
	return s;
}

#pragma mark old code here

//void Delta7CompressionTest2()
//{
//	const int factor = 3;
//	int N = 16, k = 4;
//	std::vector<int> tiles;
//
//	TopSpin tse(N, k);
//	TopSpinState s(N, k);
//	TopSpinState g(N, k);
//
//	tse.StoreGoal(g);
//	tse.ClearPDBs();
//
//	if (!fileExists(pdb7a))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 0; x <= 6; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb7a,
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	if (!fileExists(pdb8a))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 0; x < 8; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb8a,
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	if (!fileExists("/Users/nathanst/Desktop/TS-9-15.pdb"))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 9; x <= 15; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, "/Users/nathanst/Desktop/TS-9-15.pdb",
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	if (!fileExists(pdb8b))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 8; x < 16; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb8b,
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	g.Reset();
//	tse.lookups.resize(0);
//	tse.Load_Regular_PDB(pdb7a, g, true);
//	tse.lookups.push_back({kLeafNode, -0, -0, 0});
//	tse.Load_Regular_PDB_as_Delta_and_Min(pdb8a, g, factor, true);
//
//	tse.lookups.resize(0);
//	tse.Load_Regular_PDB("/Users/nathanst/Desktop/TS-9-15.pdb", g, true);
//	tse.lookups.push_back({kLeafNode, -0, -0, 2});
//	tse.Load_Regular_PDB_as_Delta_and_Min(pdb8b, g, factor, true);
//
//	tse.lookups.resize(0);
//	tse.lookups.push_back({kMaxNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
//	tse.lookups.push_back({kAddNode, 2, 3, -0}); // sum of 2 children starting at 3 in the tree
//	tse.lookups.push_back({kAddNode, 2, 5, -0}); // sum of 2 children starting at 5 in the tree
//	tse.lookups.push_back({kLeafNode, -0, -0, 0});
//	tse.lookups.push_back({kLeafMinCompress, factor, -0, 1});
//	tse.lookups.push_back({kLeafNode, -0, -0, 2});
//	tse.lookups.push_back({kLeafMinCompress, factor, -0, 3});
//	Test(tse);
//	exit(0);
//}
//
//void Delta7ValueCompressionTest()
//{
//	const int factor = 3;
//	int N = 16, k = 4;
//	std::vector<int> tiles;
//
//	TopSpin tse(N, k);
//	TopSpinState s(N, k);
//	TopSpinState g(N, k);
//
//	tse.StoreGoal(g);
//	tse.ClearPDBs();
//
//	if (!fileExists(pdb7a))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 0; x <= 6; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb7a,
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	if (!fileExists(pdb8a))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 0; x < 8; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb8a,
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	for (int x = 1; x <= 16; x*=2)
//	{
//		printf("==>Compressing by factor of %d\n", x);
//		tse.ClearPDBs();
//		tse.Load_Regular_PDB(pdb7a, g, true);
//		tse.lookups.push_back({kLeafNode, -0, -0, 0});
//		tse.Load_Regular_PDB_as_Delta(pdb8a, g, true);
//		tse.Value_Compress_PDB(1, x, true);
//	}
//}
//
//void Delta7ValueCompressionTest2()
//{
//	const int maxValue = 1;
//	int N = 16, k = 4;
//	std::vector<int> tiles;
//
//	TopSpin tse(N, k);
//	TopSpinState s(N, k);
//	TopSpinState g(N, k);
//
//	tse.StoreGoal(g);
//	tse.ClearPDBs();
//
//	if (!fileExists(pdb7a))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 0; x <= 6; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb7a,
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	if (!fileExists(pdb8a))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 0; x < 8; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb8a,
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	if (!fileExists("/Users/nathanst/Desktop/TS-9-15.pdb"))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 9; x <= 15; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, "/Users/nathanst/Desktop/TS-9-15.pdb",
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	if (!fileExists(pdb8b))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 8; x < 16; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb8b,
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	g.Reset();
//	tse.lookups.resize(0);
//	tse.Load_Regular_PDB(pdb7a, g, true);
//	tse.lookups.push_back({kLeafNode, -0, -0, 0});
//	tse.Load_Regular_PDB_as_Delta(pdb8a, g, true);
//	tse.Value_Compress_PDB(1, maxValue, true);
//
//	tse.lookups.resize(0);
//	tse.Load_Regular_PDB("/Users/nathanst/Desktop/TS-9-15.pdb", g, true);
//	tse.lookups.push_back({kLeafNode, -0, -0, 2});
//	tse.Load_Regular_PDB_as_Delta(pdb8b, g, true);
//	tse.Value_Compress_PDB(3, maxValue, true);
//
//	tse.lookups.resize(0);
//	tse.lookups.push_back({kMaxNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
//	tse.lookups.push_back({kAddNode, 2, 3, -0}); // sum of 2 children starting at 3 in the tree
//	tse.lookups.push_back({kAddNode, 2, 5, -0}); // sum of 2 children starting at 5 in the tree
//	tse.lookups.push_back({kLeafNode, -0, -0, 0});
//	//tse.lookups.push_back({kLeafValueCompress, maxValue, -0, 1});
//	tse.lookups.push_back({kLeafNode, -0, -0, 1});
//	tse.lookups.push_back({kLeafNode, -0, -0, 2});
//	//tse.lookups.push_back({kLeafValueCompress, maxValue, -0, 3});
//	tse.lookups.push_back({kLeafNode, -0, -0, 3});
//	Test(tse);
//	exit(0);
//}
//
//void DeltaMinCompressionTest()
//{
//	const int factor = 20;
//	int N = 16, k = 4;
//	std::vector<int> tiles;
//
//	TopSpin tse(N, k);
//	TopSpinState s(N, k);
//	TopSpinState g(N, k);
//
//	tse.StoreGoal(g);
//	tse.ClearPDBs();
//
//	if (!fileExists(pdb8a))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 0; x < 8; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb8a,
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	if (!fileExists(pdb8b))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 8; x < 16; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb8b,
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	g.Reset();
//	tse.lookups.resize(0);
//	tse.Load_Regular_PDB_Min_Compressed(pdb8a, g, factor, true);
//	tse.lookups.push_back({kLeafMinCompress, factor, -0, 0});
//	tse.Load_Regular_PDB_as_Delta_and_Min(pdb8a, g, factor/2, true);
//
//	tse.lookups.resize(0);
//	tse.Load_Regular_PDB_Min_Compressed(pdb8b, g, factor, true);
//	tse.lookups.push_back({kLeafMinCompress, factor, -0, 2});
//	tse.Load_Regular_PDB_as_Delta_and_Min(pdb8b, g, factor/2, true);
//
//	tse.lookups.resize(0);
//	tse.lookups.push_back({kMaxNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
//	tse.lookups.push_back({kAddNode, 2, 3, -0}); // sum of 2 children starting at 3 in the tree
//	tse.lookups.push_back({kAddNode, 2, 5, -0}); // sum of 2 children starting at 5 in the tree
//	tse.lookups.push_back({kLeafMinCompress, factor, -0, 0});
//	tse.lookups.push_back({kLeafMinCompress, factor/2, -0, 1});
//	tse.lookups.push_back({kLeafMinCompress, factor, -0, 2});
//	tse.lookups.push_back({kLeafMinCompress, factor/2, -0, 3});
//	Test(tse);
//	exit(0);
//}
//
//void MinCompressionTest()
//{
//	int N = 16, k = 4;
//	std::vector<int> tiles;
//
//	TopSpin tse(N, k);
//	TopSpinState s(N, k);
//	TopSpinState g(N, k);
//
//	tse.StoreGoal(g);
//	tse.ClearPDBs();
//
//	if (!fileExists(pdb8a))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 0; x < 8; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb8a,
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	if (!fileExists(pdb8b))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 8; x < 16; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb8b,
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	g.Reset();
//	tse.Load_Regular_PDB_Min_Compressed(pdb8a, g, 10, true);
//	tse.Load_Regular_PDB_Min_Compressed(pdb8b, g, 10, true);
//
//	tse.lookups.push_back({kMaxNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
//	tse.lookups.push_back({kLeafMinCompress, 10, -0, 0});
//	tse.lookups.push_back({kLeafMinCompress, 10, -0, 1});
//	Test(tse);
//	exit(0);
//}

//void CompareToMinCompression()
//{
//	int N = 16, k = 4;
//	std::vector<int> tiles;
//
//	TopSpin tse(N, k);
//	TopSpinState s(N, k);
//	TopSpinState g(N, k);
//
//	tse.StoreGoal(g);
//	tse.ClearPDBs();
//
//	if (!fileExists(pdb8a))
//	{
//		tiles.resize(0);
//		for (int x = 0; x < 8; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb8a,
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	if (fileExists(pdb8a))
//	{
//		//PDBTreeNodeType t;
//		//uint8_t numChildren;
//		//uint8_t firstChildID;
//		//uint8_t PDBID;
//
//		g.Reset();
//		const int factor = 1;
//		tse.Load_Regular_PDB_Min_Compressed(pdb8a, g, factor, true);
//		//mnp->Load_Regular_PDB("/Users/nathanst/Desktop/STP_0-1-4-5-8-9-12-13.pdb", tmp, true);
//
//		g.Reset();
////		tse.lookups.push_back({kLeafMinCompress, factor, 0, 0});
////		tse.Load_Regular_PDB_as_Delta(pdb8a, g, true);
//		exit(0);
//	}
//
//}
//
//void CompareToSmallerPDB()
//{
//	int N = 16, k = 4;
//	std::vector<int> tiles;
//
//	TopSpin tse(N, k);
//	TopSpinState s(N, k);
//	TopSpinState g(N, k);
//
//	tse.StoreGoal(g);
//	tse.ClearPDBs();
//
//	if (!fileExists(pdb8a))
//	{
//		tiles.resize(0);
//		for (int x = 0; x < 8; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb8a,
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	if (!fileExists(pdb7a))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 0; x <= 6; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb7a,
//					  std::thread::hardware_concurrency(), false);
//	}
//	else {
//		tse.Load_Regular_PDB(pdb7a, g, true);
//	}
//
//	if (!fileExists("/Users/nathanst/Desktop/TS-1-7.pdb"))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 1; x <= 7; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, "/Users/nathanst/Desktop/TS-1-7.pdb",
//					  std::thread::hardware_concurrency(), false);
//	}
//	else {
//		tse.Load_Regular_PDB("/Users/nathanst/Desktop/TS-1-7.pdb", g, true);
//	}
//
//	if (fileExists(pdb8a))
//	{
////		tse.lookups.push_back({kMaxNode, 2, 1, 0});
//		tse.lookups.push_back({kLeafNode, 2, 0, 0});
////		tse.lookups.push_back({kLeafNode, 2, 0, 1});
//		tse.Load_Regular_PDB_as_Delta(pdb8a, g, true);
//	}
//	exit(0);
//}

//void BaselineTest()
//{
//	int N = 16, k = 4;
//	std::vector<int> tiles;
//
//	TopSpin tse(N, k);
//	TopSpinState s(N, k);
//	TopSpinState g(N, k);
//
//	tse.StoreGoal(g);
//	tse.ClearPDBs();
//
//	if (!fileExists(pdb8a))
//	{
//		tiles.resize(0);
//		for (int x = 0; x < 8; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb8a,
//					  std::thread::hardware_concurrency(), false);
//	}
//	else {
//		g.Reset();
//		tse.Load_Regular_PDB(pdb8a, g, true);
//	}
//
//	if (!fileExists(pdb8b))
//	{
//		tiles.resize(0);
//		for (int x = 8; x < 16; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb8b,
//					  std::thread::hardware_concurrency(), false);
//	}
//	else {
//		g.Reset();
//		tse.Load_Regular_PDB(pdb8b, g, true);
//	}
//
//	if (fileExists(pdb8a) &&
//		fileExists(pdb8b))
//	{
//		tse.lookups.push_back({kMaxNode, 2, 1, 0});
//		tse.lookups.push_back({kLeafNode, 2, 0, 0});
//		tse.lookups.push_back({kLeafNode, 2, 0, 1});
//		Test(tse);
//	}
//	exit(0);
//}
//
//void BaselineTest2()
//{
//	int N = 16, k = 4;
//	std::vector<int> tiles;
//
//	TopSpin tse(N, k);
//	TopSpinState s(N, k);
//	TopSpinState g(N, k);
//
//	tse.StoreGoal(g);
//	tse.ClearPDBs();
//
//	if (!fileExists(pdb7a))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 0; x <= 6; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb7a,
//					  std::thread::hardware_concurrency(), false);
//	}
//	else {
//		g.Reset();
//		tse.Load_Regular_PDB(pdb7a, g, true);
//	}
//
//	if (!fileExists("/Users/nathanst/Desktop/TS-7-13.pdb"))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 7; x <= 13; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, "/Users/nathanst/Desktop/TS-7-13.pdb",
//					  std::thread::hardware_concurrency(), false);
//	}
//	else {
//		g.Reset();
//		tse.Load_Regular_PDB("/Users/nathanst/Desktop/TS-7-13.pdb", g, true);
//	}
//
//	if (!fileExists("/Users/nathanst/Desktop/TS-14-15.pdb"))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 14; x <= 15; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, "/Users/nathanst/Desktop/TS-14-15.pdb",
//					  std::thread::hardware_concurrency(), false);
//	}
//	else {
//		g.Reset();
//		tse.Load_Regular_PDB("/Users/nathanst/Desktop/TS-14-15.pdb", g, true);
//	}
//
//	if (fileExists(pdb7a) &&
//		fileExists("/Users/nathanst/Desktop/TS-7-13.pdb") &&
//		fileExists("/Users/nathanst/Desktop/TS-14-15.pdb"))
//	{
//		tse.lookups.push_back({kMaxNode, 3, 1, -0});
//		tse.lookups.push_back({kLeafNode,-0,-0, 0});
//		tse.lookups.push_back({kLeafNode,-0,-0, 1});
//		tse.lookups.push_back({kLeafNode,-0,-0, 2});
//		Test(tse);
//	}
//	exit(0);
//}
//
//void LosslessTest()
//{
//	int N = 16, k = 4;
//	std::vector<int> tiles;
//
//	TopSpin tse(N, k);
//	TopSpinState s(N, k);
//	TopSpinState g(N, k);
//
//	tse.StoreGoal(g);
//	tse.ClearPDBs();
//
//	if (!fileExists("/Users/nathanst/Desktop/TS-0-5.pdb"))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 0; x < 6; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, "/Users/nathanst/Desktop/TS-0-5.pdb",
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	if (!fileExists(pdb8a))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 0; x < 8; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb8a,
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	if (!fileExists("/Users/nathanst/Desktop/TS-10-15.pdb"))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 10; x < 16; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, "/Users/nathanst/Desktop/TS-10-15.pdb",
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	if (!fileExists(pdb8b))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 8; x < 16; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb8b,
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	g.Reset();
//	tse.lookups.resize(0);
//	tse.Load_Regular_PDB("/Users/nathanst/Desktop/TS-0-5.pdb", g, true);
//	tse.lookups.push_back({kLeafNode, -0, -0, 0});
//	tse.Load_Regular_PDB_as_Delta(pdb8a, g, true);
//
//	tse.lookups.resize(0);
//	tse.Load_Regular_PDB("/Users/nathanst/Desktop/TS-10-15.pdb", g, true);
//	tse.lookups.push_back({kLeafNode, -0, -0, 2});
//	tse.Load_Regular_PDB_as_Delta(pdb8b, g, true);
//
//	tse.lookups.resize(0);
//	tse.lookups.push_back({kMaxNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
//	tse.lookups.push_back({kAddNode, 2, 3, -0}); // sum of 2 children starting at 3 in the tree
//	tse.lookups.push_back({kAddNode, 2, 5, -0}); // sum of 2 children starting at 5 in the tree
//	tse.lookups.push_back({kLeafNode, -0, -0, 0});
//	tse.lookups.push_back({kLeafNode, -0, -0, 1});
//	tse.lookups.push_back({kLeafNode, -0, -0, 2});
//	tse.lookups.push_back({kLeafNode, -0, -0, 3});
//	Test(tse);
//	exit(0);
//}
//
//void Delta6CompressionTest()
//{
//	const int factor = 10;
//	int N = 16, k = 4;
//	std::vector<int> tiles;
//
//	TopSpin tse(N, k);
//	TopSpinState s(N, k);
//	TopSpinState g(N, k);
//
//	tse.StoreGoal(g);
//	tse.ClearPDBs();
//
//	if (!fileExists("/Users/nathanst/Desktop/TS-0-5.pdb"))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 0; x < 6; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, "/Users/nathanst/Desktop/TS-0-5.pdb",
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	if (!fileExists(pdb8a))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 0; x < 8; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb8a,
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	if (!fileExists("/Users/nathanst/Desktop/TS-10-15.pdb"))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 10; x < 16; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, "/Users/nathanst/Desktop/TS-10-15.pdb",
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	if (!fileExists(pdb8b))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 8; x < 16; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb8b,
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	g.Reset();
//	tse.lookups.resize(0);
//	tse.Load_Regular_PDB("/Users/nathanst/Desktop/TS-0-5.pdb", g, true);
//	tse.lookups.push_back({kLeafNode, -0, -0, 0});
//	tse.Load_Regular_PDB_as_Delta_and_Min(pdb8a, g, factor, true);
//
//	tse.lookups.resize(0);
//	tse.Load_Regular_PDB("/Users/nathanst/Desktop/TS-10-15.pdb", g, true);
//	tse.lookups.push_back({kLeafNode, -0, -0, 2});
//	tse.Load_Regular_PDB_as_Delta_and_Min(pdb8b, g, factor, true);
//
//	tse.lookups.resize(0);
//	tse.lookups.push_back({kMaxNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
//	tse.lookups.push_back({kAddNode, 2, 3, -0}); // sum of 2 children starting at 3 in the tree
//	tse.lookups.push_back({kAddNode, 2, 5, -0}); // sum of 2 children starting at 5 in the tree
//	tse.lookups.push_back({kLeafNode, -0, -0, 0});
//	tse.lookups.push_back({kLeafMinCompress, factor, -0, 1});
//	tse.lookups.push_back({kLeafNode, -0, -0, 2});
//	tse.lookups.push_back({kLeafMinCompress, factor, -0, 3});
//	Test(tse);
//	exit(0);
//}
//
//void Delta7CompressionTest()
//{
//	const int factor = 3;
//	int N = 16, k = 4;
//	std::vector<int> tiles;
//
//	TopSpin tse(N, k);
//	TopSpinState s(N, k);
//	TopSpinState g(N, k);
//
//	tse.StoreGoal(g);
//	tse.ClearPDBs();
//
//	if (!fileExists(pdb7a))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 0; x <= 6; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb7a,
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	if (!fileExists(pdb8a))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 0; x < 8; x++)
//			tiles.push_back(x);
//
//		tse.Build_PDB(g, tiles, pdb8a,
//					  std::thread::hardware_concurrency(), false);
//		tse.ClearPDBs();
//	}
//
//	for (int x = 1; x <= 10; x++)
//	{
//		printf("==>Compressing by factor of %d\n", x);
//		tse.ClearPDBs();
//		tse.Load_Regular_PDB(pdb7a, g, true);
//		tse.lookups.push_back({kLeafNode, -0, -0, 0});
//		tse.Load_Regular_PDB_as_Delta_and_Min(pdb8a, g, x, true);
//	}
//}