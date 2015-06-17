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
TopSpinState GetInstance(int which, bool weighted);
void Test(TopSpin &tse, const char *prefix);
void MinCompressionTest();
void MeasureIR(TopSpin &tse);
void GetBitValueCutoffs(std::vector<int> &cutoffs, int bits);

void BitDeltaValueCompressionTest(bool weighted);
void ModValueCompressionTest(bool weighted);
void ModValueDeltaCompressionTest(bool weighted);
void DivValueCompressionTest(bool weighted);
void DivDeltaValueCompressionTest(bool weighted);

void BaseHeuristicTest(bool weighted);
void FractionalNodesCompressionTest(bool weighted);
void FractionalModNodesCompressionTest(bool weighted);
void BitDeltaNodesCompressionTest(bool weighted);
void ModNodesCompressionTest(bool weighted);
void ModNodesDeltaCompressionTest(bool weighted);
void DivNodesCompressionTest(bool weighted);
void DivDeltaNodesCompressionTest(bool weighted);

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
//	TopSpin tse(16, 4);
//	TopSpinState s1(16, 4);
//	tse.Get_PDB_Size(s1, 1);
//	tse.Get_PDB_Size(s1, 15);
//	std::vector<int> distinct1 = {1, 2, 3};
//	std::vector<int> distinct2 = {0, 1, 2, 3};
//	std::vector<int> distinct3 = {0, 1, 2};
//	std::vector<int> distinct4 = {0};
//	std::vector<int> distinct5 = {3};
//	s1.Reset();
//	tse.ApplyAction(s1, 2);
//	std::cout << s1 << "\n";
//	std::cout << "Hash of 1,2,3 = " << tse.GetPDBHash(s1, distinct1) << "\n";
//	std::cout << "Hash of 0,1,2,3 = " << tse.GetPDBHash(s1, distinct2) << "\n";
//	std::cout << "Hash of 0,1,2 = " << tse.GetPDBHash(s1, distinct3) << "\n";
//	std::cout << "Hash of 0 = " << tse.GetPDBHash(s1, distinct4) << "\n";
//	std::cout << "Hash of 3 = " << tse.GetPDBHash(s1, distinct5) << "\n";
//	tse.ApplyAction(s1, 3);
//	std::cout << s1 << "\n";
//	std::cout << "Hash of 1,2,3 = " << tse.GetPDBHash(s1, distinct1) << "\n";
//	std::cout << "Hash of 0,1,2,3 = " << tse.GetPDBHash(s1, distinct2) << "\n";
//	std::cout << "Hash of 0,1,2 = " << tse.GetPDBHash(s1, distinct3) << "\n";
//	std::cout << "Hash of 0 = " << tse.GetPDBHash(s1, distinct4) << "\n";
//	std::cout << "Hash of 3 = " << tse.GetPDBHash(s1, distinct5) << "\n";
//	exit(0);

//	ModValueDeltaCompressionTest();
//	DivDeltaValueCompressionTest();
//	ModValueCompressionTest();
//	DivValueCompressionTest();
//	BitDeltaValueCompressionTest();

	bool weighted = false;
//	DivDeltaNodesCompressionTest(weighted);
//	ModNodesDeltaCompressionTest(weighted);

	weighted = true;
//	DivDeltaNodesCompressionTest(weighted);
	ModNodesDeltaCompressionTest(weighted);
//	BaseHeuristicTest(weighted);
//	FractionalModNodesCompressionTest(weighted);
//	FractionalNodesCompressionTest(weighted);
//	BitDeltaNodesCompressionTest(weighted);
//	DivNodesCompressionTest(weighted);
//	ModNodesCompressionTest(weighted);

	exit(0);
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
#include <string>
const int N = 16, k = 4;
const char *prefix = "/Users/nathanst/Desktop/";
const char *getPDB7a(bool weighted)
{
	static std::string s;
	s = prefix;
	if (!weighted)
		s += "TS-0-6.pdb";
	else
		s += "wTS-0-6.pdb";
	return s.c_str();
}

const char *getPDB8a(bool weighted)
{
	static std::string s;
	s = prefix;
	if (!weighted)
		s += "TS-0-7.pdb";
	else
		s += "wTS-0-7.pdb";
	return s.c_str();
}

const char *getPDB7b(bool weighted)
{
	static std::string s;
	s = prefix;
	if (!weighted)
		s += "TS-8-14.pdb";
	else
		s += "wTS-8-14.pdb";
	return s.c_str();
}
const char *getPDB8b(bool weighted)
{
	static std::string s;
	s = prefix;
	if (!weighted)
		s += "TS-8-15.pdb";
	else
		s += "wTS-8-15.pdb";
	return s.c_str();
}
void BuildPDBs(bool aPDBs, bool bPDBs, bool weighted)
{
	TopSpin tse(N, k);
	tse.SetWeighted(weighted);
	std::vector<int> tiles;
	
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();

	if (aPDBs)
	{
		if (!fileExists(getPDB7a(weighted)))
		{
			g.Reset();
			tiles.resize(0);
			//for (int x = 0; x <= 12; x+=2)
			for (int x = 0; x <= 6; x++)
				tiles.push_back(x);
			
			tse.Build_PDB(g, tiles, getPDB7a(weighted),
						  std::thread::hardware_concurrency(), false);
			tse.ClearPDBs();
		}
		
		if (!fileExists(getPDB8a(weighted)))
		{
			g.Reset();
			tiles.resize(0);
			//for (int x = 0; x <= 14; x+=2)
			for (int x = 0; x <= 7; x++)
				tiles.push_back(x);
			
			tse.Build_PDB(g, tiles, getPDB8a(weighted),
						  std::thread::hardware_concurrency(), false);
			tse.ClearPDBs();
		}
	}
	
	if (bPDBs)
	{
		if (!fileExists(getPDB7b(weighted)))
		{
			g.Reset();
			tiles.resize(0);
			for (int x = 8; x <= 14; x++)
				//for (int x = 1; x <= 14; x+=2)
				tiles.push_back(x);
			
			tse.Build_PDB(g, tiles, getPDB7b(weighted),
						  std::thread::hardware_concurrency(), false);
			tse.ClearPDBs();
		}
		
		if (!fileExists(getPDB8b(weighted)))
		{
			g.Reset();
			tiles.resize(0);
			for (int x = 8; x <= 15; x++)
			//for (int x = 1; x <= 15; x+=2)
				tiles.push_back(x);
			
			tse.Build_PDB(g, tiles, getPDB8b(weighted),
						  std::thread::hardware_concurrency(), false);
			tse.ClearPDBs();
		}
	}
}

#pragma mark heuristic value tests

void ModValueCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	tse.SetWeighted(weighted);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	BuildPDBs(true, false, weighted);
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Compressing (mod) by factor of %d\n", x);
		tse.ClearPDBs();
		//tse.Load_Regular_PDB(getPDB7a(weighted), g, true);
		uint64_t oldSize = tse.Get_PDB_Size(g, 8);
		uint64_t newSize = oldSize / x;
		tse.Load_Regular_PDB(getPDB8a(weighted), g, true);
		tse.Mod_Compress_PDB(0, newSize, true);
		tse.lookups.push_back({kLeafModCompress, -0, -0, -0});
		MeasureIR(tse);
	}
}

void ModValueDeltaCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	tse.SetWeighted(weighted);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	BuildPDBs(true, false, weighted);
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Compressing (mod-delta) by factor of %d\n", x);
		tse.ClearPDBs();
		tse.Load_Regular_PDB(getPDB7a(weighted), g, true);
		tse.lookups.push_back({kLeafNode, -0, -0, 0});

		uint64_t oldSize = tse.Get_PDB_Size(g, 8);
		uint64_t newSize = oldSize / x;
		tse.Load_Regular_PDB(getPDB8a(weighted), g, true);
		tse.Delta_Compress_PDB(g, 1, true);
		tse.Mod_Compress_PDB(1, newSize, true);
//		tse.lookups.push_back({kLeafModCompress, -0, -0, -0});
//		MeasureIR(tse);
	}
}

void DivValueCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	tse.SetWeighted(weighted);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	if (!fileExists(getPDB7a(weighted)))
	{
		g.Reset();
		tiles.resize(0);
		for (int x = 0; x <= 6; x++)
			tiles.push_back(x);
		
		tse.Build_PDB(g, tiles, getPDB7a(weighted),
					  std::thread::hardware_concurrency(), false);
		tse.ClearPDBs();
	}
	
	if (!fileExists(getPDB8a(weighted)))
	{
		g.Reset();
		tiles.resize(0);
		for (int x = 0; x < 8; x++)
			tiles.push_back(x);
		
		tse.Build_PDB(g, tiles, getPDB8a(weighted),
					  std::thread::hardware_concurrency(), false);
		tse.ClearPDBs();
	}
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Compressing (div) by factor of %d\n", x);
		tse.ClearPDBs();
		tse.Load_Regular_PDB(getPDB8a(weighted), g, true);
		tse.Min_Compress_PDB(0, x, true);
		tse.lookups.push_back({kLeafMinCompress, static_cast<uint8_t>(x), -0, 0});
		MeasureIR(tse);
	}
}

void DivDeltaValueCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	tse.SetWeighted(weighted);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	if (!fileExists(getPDB7a(weighted)))
	{
		g.Reset();
		tiles.resize(0);
		for (int x = 0; x <= 6; x++)
			tiles.push_back(x);
		
		tse.Build_PDB(g, tiles, getPDB7a(weighted),
					  std::thread::hardware_concurrency(), false);
		tse.ClearPDBs();
	}
	
	if (!fileExists(getPDB8a(weighted)))
	{
		g.Reset();
		tiles.resize(0);
		for (int x = 0; x < 8; x++)
			tiles.push_back(x);
		
		tse.Build_PDB(g, tiles, getPDB8a(weighted),
					  std::thread::hardware_concurrency(), false);
		tse.ClearPDBs();
	}
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Compressing (div delta) by factor of %d\n", x);
		tse.ClearPDBs();
		tse.Load_Regular_PDB(getPDB7a(weighted), g, false);
		tse.lookups.push_back({kLeafNode, -0, -0, 0});

		tse.Load_Regular_PDB(getPDB8a(weighted), g, false);
		tse.Delta_Compress_PDB(g, 1, true);
		tse.Min_Compress_PDB(1, x, true);
		//MeasureIR(tse);
	}
}

void BitDeltaValueCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	tse.SetWeighted(weighted);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	BuildPDBs(true, false, weighted);
	
	for (int x = 1; x <= 4; x*=2)
	{
		g.Reset();
		printf("==>Compressing (value-range-delta) to %d bits\n", x);
		tse.ClearPDBs();
		tse.Load_Regular_PDB(getPDB7a(weighted), g, false);
		tse.lookups.push_back({kLeafNode, -0, -0, 0});
		
		tse.Load_Regular_PDB(getPDB8a(weighted), g, false);
		tse.Delta_Compress_PDB(g, 1, true);
		std::vector<int> cutoffs;
		GetBitValueCutoffs(cutoffs, x);
		tse.Value_Compress_PDB(1, cutoffs, true);
		//MeasureIR(tse);
	}
}

#pragma mark node expansion tests

void BaseHeuristicTest(bool weighted)
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	tse.SetWeighted(weighted);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	
	tse.ClearPDBs();
	BuildPDBs(true, true, weighted);
	
	{
		tse.ClearPDBs();
		tse.Load_Regular_PDB(getPDB7a(weighted), g, true);
		tse.Load_Regular_PDB(getPDB7b(weighted), g, true);
		tse.lookups.push_back({kMaxNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
		tse.lookups.push_back({kLeafNode, -0, -0, 0});
		tse.lookups.push_back({kLeafNode, -0, -0, 1});
		std::string desc = "BASE7-";
		Test(tse, desc.c_str());
	}
	
	{
		tse.ClearPDBs();
		tse.Load_Regular_PDB(getPDB8a(weighted), g, true);
		tse.Load_Regular_PDB(getPDB8b(weighted), g, true);
		tse.lookups.push_back({kMaxNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
		tse.lookups.push_back({kLeafNode, -0, -0, 0});
		tse.lookups.push_back({kLeafNode, -0, -0, 1});
		std::string desc = "BASE8-";
		Test(tse, desc.c_str());
	}
}

//Fractional_Mod_Compress_PDB
void FractionalNodesCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	tse.SetWeighted(weighted);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	BuildPDBs(true, true, weighted);
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Fractional (contiguous): Reducing by factor of %d\n", x);
		tse.ClearPDBs();
		uint64_t oldSize = tse.Get_PDB_Size(g, 8);
		uint64_t newSize = oldSize / x;
		tse.Load_Regular_PDB(getPDB7a(weighted), g, true);
		tse.Load_Regular_PDB(getPDB8a(weighted), g, true);
		tse.Load_Regular_PDB(getPDB7b(weighted), g, true);
		tse.Load_Regular_PDB(getPDB8b(weighted), g, true);
		tse.Fractional_Compress_PDB(1, newSize, true);
		tse.Fractional_Compress_PDB(3, newSize, true);
		tse.lookups.push_back({kMaxNode, 4, 1, -0}); // max of 2 children starting at 1 in the tree
		tse.lookups.push_back({kLeafNode, -0, -0, 0});
		tse.lookups.push_back({kLeafFractionalCompress, -0, -0, 1});
		tse.lookups.push_back({kLeafNode, -0, -0, 2});
		tse.lookups.push_back({kLeafFractionalCompress, -0, -0, 3});
		std::string desc = "FCT-C-";
		desc += ('0'+x/10);
		desc += ('0'+x%10);
		Test(tse, desc.c_str());
	}
}

void FractionalModNodesCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	tse.SetWeighted(weighted);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	BuildPDBs(true, true, weighted);
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Fractional MOD: Reducing by factor of %d\n", x);
		tse.ClearPDBs();
		tse.Load_Regular_PDB(getPDB7a(weighted), g, true);
		tse.Load_Regular_PDB(getPDB8a(weighted), g, true);
		tse.Load_Regular_PDB(getPDB7b(weighted), g, true);
		tse.Load_Regular_PDB(getPDB8b(weighted), g, true);
		tse.Fractional_Mod_Compress_PDB(1, x, true);
		tse.Fractional_Mod_Compress_PDB(3, x, true);
		tse.lookups.push_back({kMaxNode, 4, 1, -0}); // max of 2 children starting at 1 in the tree
		tse.lookups.push_back({kLeafNode, -0, -0, 0});
		tse.lookups.push_back({kLeafFractionalModCompress, static_cast<uint8_t>(x), -0, 1}); // factor, -- , id
		tse.lookups.push_back({kLeafNode, -0, -0, 2});
		tse.lookups.push_back({kLeafFractionalModCompress, static_cast<uint8_t>(x), -0, 3});
		std::string desc = "FCT-M-";
		desc += ('0'+x/10);
		desc += ('0'+x%10);
		Test(tse, desc.c_str());
	}
}


void ModNodesCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	tse.SetWeighted(weighted);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	BuildPDBs(true, true, weighted);
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Compressing (mod) by factor of %d\n", x);
		tse.ClearPDBs();
		uint64_t oldSize = tse.Get_PDB_Size(g, 8);
		uint64_t newSize = oldSize / x;
		tse.Load_Regular_PDB(getPDB8a(weighted), g, true);
		tse.Load_Regular_PDB(getPDB8b(weighted), g, true);
		tse.Mod_Compress_PDB(0, newSize, true);
		tse.Mod_Compress_PDB(1, newSize, true);
		tse.lookups.push_back({kMaxNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
		tse.lookups.push_back({kLeafModCompress, -0, -0, 0});
		tse.lookups.push_back({kLeafModCompress, -0, -0, 1});

		std::string desc = "MOD-";
		desc += ('0'+x/10);
		desc += ('0'+x%10);
		Test(tse, desc.c_str());
	}
}

void ModNodesDeltaCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	tse.SetWeighted(weighted);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	BuildPDBs(true, true, weighted);
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Compressing (mod-delta by factor of %d\n", x);
		tse.ClearPDBs();
		uint64_t oldSize = tse.Get_PDB_Size(g, 8);
		uint64_t newSize = oldSize / x;
		tse.Load_Regular_PDB(getPDB7a(weighted), g, true);
		tse.Load_Regular_PDB(getPDB7b(weighted), g, true);
		tse.Load_Regular_PDB(getPDB8a(weighted), g, true);
		tse.Load_Regular_PDB(getPDB8b(weighted), g, true);
		tse.lookups.push_back({kLeafNode, -0, -0, 0});
		tse.Delta_Compress_PDB(g, 2, true);
		tse.lookups.back().PDBID = 1;
		tse.Delta_Compress_PDB(g, 3, true);
		tse.lookups.resize(0);
		
		tse.Mod_Compress_PDB(2, newSize, true);
		tse.Mod_Compress_PDB(3, newSize, true);

		tse.lookups.push_back({kMaxNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
		tse.lookups.push_back({kAddNode, 2, 3, -0}); // max of 2 children starting at 1 in the tree
		tse.lookups.push_back({kAddNode, 2, 5, -0}); // max of 2 children starting at 1 in the tree
		
		tse.lookups.push_back({kLeafNode, -0, -0, 0});
		tse.lookups.push_back({kLeafModCompress, -0, -0, 2});

		tse.lookups.push_back({kLeafNode, -0, -0, 1});
		tse.lookups.push_back({kLeafModCompress, -0, -0, 3});
		
		std::string desc = "MOD-D-";
		desc += ('0'+x/10);
		desc += ('0'+x%10);
		Test(tse, desc.c_str());
	}
}

void DivNodesCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	tse.SetWeighted(weighted);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	BuildPDBs(true, true, weighted);
	
	for (int x = 1; x <= 10; x++)
	{
		g.Reset();
		printf("==>Compressing (div) by factor of %d\n", x);
		tse.ClearPDBs();
		tse.Load_Regular_PDB(getPDB8a(weighted), g, true);
		tse.Min_Compress_PDB(0, x, true);
		tse.Load_Regular_PDB(getPDB8b(weighted), g, true);
		tse.Min_Compress_PDB(1, x, true);

		tse.lookups.push_back({kMaxNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
		tse.lookups.push_back({kLeafMinCompress, static_cast<uint8_t>(x), -0, 0});
		tse.lookups.push_back({kLeafMinCompress, static_cast<uint8_t>(x), -0, 1});

		std::string desc = "DIV-";
		desc += ('0'+x/10);
		desc += ('0'+x%10);
		Test(tse, desc.c_str());
		//MeasureIR(tse);
	}
}

void DivDeltaNodesCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	tse.SetWeighted(weighted);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	BuildPDBs(true, true, weighted);
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Compressing (div-delta) by factor of %d\n", x);
		tse.ClearPDBs();
		tse.Load_Regular_PDB(getPDB7a(weighted), g, true);
		tse.Load_Regular_PDB(getPDB7b(weighted), g, true);
		tse.Load_Regular_PDB(getPDB8a(weighted), g, true);
		tse.Load_Regular_PDB(getPDB8b(weighted), g, true);
		
		tse.lookups.push_back({kLeafNode, -0, -0, 0});
		tse.Delta_Compress_PDB(g, 2, true);
		tse.lookups.back().PDBID = 1;
		tse.Delta_Compress_PDB(g, 3, true);
		tse.lookups.resize(0);

		tse.Min_Compress_PDB(2, x, true);
		tse.Min_Compress_PDB(3, x, true);
		
		tse.lookups.push_back({kMaxNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
		tse.lookups.push_back({kAddNode, 2, 3, -0}); // max of 2 children starting at 1 in the tree
		tse.lookups.push_back({kAddNode, 2, 5, -0}); // max of 2 children starting at 1 in the tree

		tse.lookups.push_back({kLeafNode, -0, -0, 0});
		tse.lookups.push_back({kLeafMinCompress, static_cast<uint8_t>(x), -0, 2});
		tse.lookups.push_back({kLeafNode, -0, -0, 1});
		tse.lookups.push_back({kLeafMinCompress, static_cast<uint8_t>(x), -0, 3});
		
		std::string desc = "DIV-D-";
		desc += ('0'+x/10);
		desc += ('0'+x%10);
		Test(tse, desc.c_str());
	}
}


void BitDeltaNodesCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	TopSpin tse(N, k);
	tse.SetWeighted(weighted);
	TopSpinState s(N, k);
	TopSpinState g(N, k);
	
	tse.StoreGoal(g);
	tse.ClearPDBs();
	
	BuildPDBs(true, true, weighted);
	
	for (int x = 2; x <= 4; x*=2)
	{
		std::vector<int> cutoffs;
		GetBitValueCutoffs(cutoffs, x);

		g.Reset();
		printf("==>Compressing to %d bits\n", x);
		tse.ClearPDBs();
		tse.Load_Regular_PDB(getPDB7a(weighted), g, false); // PDB 0
		tse.Load_Regular_PDB(getPDB8a(weighted), g, false); // PDB 1
		tse.lookups.push_back({kLeafNode, -0, -0, 0});
		tse.Delta_Compress_PDB(g, 1, true);
		tse.Value_Range_Compress_PDB(1, x, true);
		tse.Load_Regular_PDB(getPDB7b(weighted), g, false); // PDB 2
		tse.Load_Regular_PDB(getPDB8b(weighted), g, false); // PDB 3
		tse.lookups.back().PDBID = 2;
		tse.Delta_Compress_PDB(g, 3, true);
		//tse.Value_Compress_PDB(3, cutoffs, true);
		tse.Value_Range_Compress_PDB(3, x, true);


		tse.lookups.resize(0);
		tse.lookups.push_back({kMaxNode, 2, 1, -0});
		tse.lookups.push_back({kAddNode, 2, 3, -0});
		tse.lookups.push_back({kAddNode, 2, 5, -0});
		tse.lookups.push_back({kLeafNode, -0, -0, 0});
		tse.lookups.push_back({kLeafNode, -0, -0, 1});
		tse.lookups.push_back({kLeafNode, -0, -0, 2});
		tse.lookups.push_back({kLeafNode, -0, -0, 3});

		std::string desc = "VALRNG-";
		desc += ('0'+x/10);
		desc += ('0'+x%10);
		Test(tse, desc.c_str());
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


void Test(TopSpin &tse, const char *prefix)
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
		s = GetInstance(x, tse.GetWeighted());
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
	printf("%s: %1.2fs elapsed; %llu nodes expanded\n", prefix, t1.EndTimer(), nodes);
}


TopSpinState GetInstance(int which, bool weighted)
{
	srandom(which*101+11);
	TopSpin tse(16, 4);
	TopSpinState s(16,4);
	s.Reset();
	int length = 10000;
	for (int x = 0; x < length; x++)
	{
		tse.ApplyAction(s, random()%16);
	}
	return s;
}

