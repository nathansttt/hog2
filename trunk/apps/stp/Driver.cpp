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
#include "MNPuzzle.h"
#include "IDAStar.h"
#include "Timer.h"

void CompareToMinCompression();
void CompareToSmallerPDB();

void BuildSTP_PDB(unsigned long windowID, tKeyboardModifier , char);
void STPTest(unsigned long , tKeyboardModifier , char);
MNPuzzleState GetInstance(int which, bool weighted);
void Test(MNPuzzle &mnp, const char *prefix);
void MinCompressionTest();
void MeasureIR(MNPuzzle &mnp);
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

MNPuzzle *ts = 0;

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
		ts = new MNPuzzle(5, 4);
	}
}

MNPuzzleState s(5,4), t(5, 4);
std::vector<slideDir> moves;
double v = 1;
slideDir lastMove = kUp;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	ts->OpenGLDraw(s);
//	v += 0.1;
//	if (v > 1)
//	{
//		s = t;
//		ts->GetActions(t, moves);
//		slideDir tmp = 0;
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
	BuildSTP_PDB(0, kNoModifier, 'a');
	exit(0);
	return 2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 'r': recording = !recording; break;
		case '0': ts->ApplyAction(s, kUp); break;
		case '1': ts->ApplyAction(s, kDown); break;
		case '2': ts->ApplyAction(s, kLeft); break;
		case '3': ts->ApplyAction(s, kRight); break;
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
			if (!ts) break;
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

void GetSTPInstance(const MNPuzzle &ts, MNPuzzleState &theState, int which)
{
	srandom(which);
	theState.Reset();
	std::vector<slideDir> acts;
	ts.GetActions(theState, acts);
	for (int x = 0; x < 50; x++)
	{
		ts.ApplyAction(theState, acts[random()%acts.size()]);
	}
}

void BuildSTP_PDB(unsigned long windowID, tKeyboardModifier , char)
{
//	MNPuzzle mnp(16, 4);
//	MNPuzzleState s1(16, 4);
//	mnp.Get_PDB_Size(s1, 1);
//	mnp.Get_PDB_Size(s1, 15);
//	std::vector<int> distinct1 = {1, 2, 3};
//	std::vector<int> distinct2 = {0, 1, 2, 3};
//	std::vector<int> distinct3 = {0, 1, 2};
//	std::vector<int> distinct4 = {0};
//	std::vector<int> distinct5 = {3};
//	s1.Reset();
//	mnp.ApplyAction(s1, 2);
//	std::cout << s1 << "\n";
//	std::cout << "Hash of 1,2,3 = " << mnp.GetPDBHash(s1, distinct1) << "\n";
//	std::cout << "Hash of 0,1,2,3 = " << mnp.GetPDBHash(s1, distinct2) << "\n";
//	std::cout << "Hash of 0,1,2 = " << mnp.GetPDBHash(s1, distinct3) << "\n";
//	std::cout << "Hash of 0 = " << mnp.GetPDBHash(s1, distinct4) << "\n";
//	std::cout << "Hash of 3 = " << mnp.GetPDBHash(s1, distinct5) << "\n";
//	mnp.ApplyAction(s1, 3);
//	std::cout << s1 << "\n";
//	std::cout << "Hash of 1,2,3 = " << mnp.GetPDBHash(s1, distinct1) << "\n";
//	std::cout << "Hash of 0,1,2,3 = " << mnp.GetPDBHash(s1, distinct2) << "\n";
//	std::cout << "Hash of 0,1,2 = " << mnp.GetPDBHash(s1, distinct3) << "\n";
//	std::cout << "Hash of 0 = " << mnp.GetPDBHash(s1, distinct4) << "\n";
//	std::cout << "Hash of 3 = " << mnp.GetPDBHash(s1, distinct5) << "\n";
//	exit(0);

//	ModValueDeltaCompressionTest();
//	DivDeltaValueCompressionTest();
//	ModValueCompressionTest();
//	DivValueCompressionTest();
//	BitDeltaValueCompressionTest();

	bool weighted = true;

	BaseHeuristicTest(weighted);
//	FractionalModNodesCompressionTest(weighted);
//	FractionalNodesCompressionTest(weighted);
//	BitDeltaNodesCompressionTest(weighted);
//	DivNodesCompressionTest(weighted);
//	ModNodesCompressionTest(weighted);

	exit(0);
}

void STPTest(unsigned long , tKeyboardModifier , char)
{
	int N = 16, k = 4;
	std::vector<int> tiles;

	MNPuzzle mnp(N, k);
	MNPuzzleState s(N, k);
	MNPuzzleState g(N, k);

//	if (ts->PDB.size() == 0)
//	{
//		mnp.Load_Regular_PDB("/Users/nathanst/Desktop/STP_0-5.pdb", g, true);
//		mnp.Load_Regular_PDB("/Users/nathanst/Desktop/STP_12-15.pdb", g, true);
//		mnp.Load_Regular_PDB("/Users/nathanst/Desktop/STP_6-11.pdb", g, true);
//		mnp.lookups.push_back({kMaxNode, 3, 1, 0});
//		mnp.lookups.push_back({kLeafNode, 0, 0, 0});
//		mnp.lookups.push_back({kLeafNode, 0, 0, 1});
//		mnp.lookups.push_back({kLeafNode, 0, 0, 2});
//	}

	if (ts->PDB.size() == 0)
	{
		mnp.Load_Regular_PDB("/Users/nathanst/Desktop/STP_0-3.pdb", g, true);
		mnp.Load_Regular_PDB("/Users/nathanst/Desktop/STP_0-5+.pdb", g, true);
		mnp.Load_Regular_PDB("/Users/nathanst/Desktop/STP_6-9.pdb", g, true);
		mnp.Load_Regular_PDB("/Users/nathanst/Desktop/STP_6-11+.pdb", g, true);
		mnp.Load_Regular_PDB("/Users/nathanst/Desktop/STP_12-15.pdb", g, true);
		mnp.lookups.push_back({kMaxNode, 3, 1, 0});
		mnp.lookups.push_back({kAddNode, 2, 4, 0});
		mnp.lookups.push_back({kAddNode, 2, 6, 0});
		mnp.lookups.push_back({kLeafNode, 0, 0, 4});
		mnp.lookups.push_back({kLeafNode, 0, 0, 0});
		mnp.lookups.push_back({kLeafNode, 0, 0, 1});
		mnp.lookups.push_back({kLeafNode, 0, 0, 2});
		mnp.lookups.push_back({kLeafNode, 0, 0, 3});
	}

	
	IDAStar<MNPuzzleState, slideDir> ida;
	std::vector<slideDir> path1;


	//mnp.SetPruneSuccessors(true);
	for (int x = 0; x < 100; x++)
	{
		GetSTPInstance(mnp, s, x);
		std::cout << "Problem " << x << std::endl;
		std::cout << "Searching from: " << std::endl << s << std::endl << g << std::endl;
		Timer t;
		t.StartTimer();
		ida.GetPath(&mnp, s, g, path1);
		t.EndTimer();
		std::cout << "Path found, length " << path1.size() << " time:" << t.GetElapsedTime() << " ";
		std::cout << ida.GetNodesExpanded() << " nodes expanded" << std::endl;
//		for (int x = 0; x < ts->histogram.size(); x++)
//		{
//			printf("%2d  %d\n", x, ts->histogram[x]);
//		}
		
//		mnp.PrintHStats();
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
		s += "STP-0-6.pdb";
	else
		s += "wSTP-0-6.pdb";
	return s.c_str();
}

const char *getPDB8a(bool weighted)
{
	static std::string s;
	s = prefix;
	if (!weighted)
		s += "STP-0-7.pdb";
	else
		s += "wSTP-0-7.pdb";
	return s.c_str();
}

const char *getPDB7b(bool weighted)
{
	static std::string s;
	s = prefix;
	if (!weighted)
		s += "STP-8-14.pdb";
	else
		s += "wSTP-8-14.pdb";
	return s.c_str();
}
const char *getPDB8b(bool weighted)
{
	static std::string s;
	s = prefix;
	if (!weighted)
		s += "STP-8-15.pdb";
	else
		s += "wSTP-8-15.pdb";
	return s.c_str();
}
void BuildPDBs(bool aPDBs, bool bPDBs, bool weighted)
{
	MNPuzzle mnp(N, k);
	mnp.SetWeighted(weighted);
	std::vector<int> tiles;
	
	MNPuzzleState s(N, k);
	MNPuzzleState g(N, k);
	
	mnp.StoreGoal(g);
	mnp.ClearPDBs();

	if (aPDBs)
	{
		if (!fileExists(getPDB7a(weighted)))
		{
			g.Reset();
			tiles.resize(0);
			//for (int x = 0; x <= 12; x+=2)
			for (int x = 0; x <= 6; x++)
				tiles.push_back(x);
			
			mnp.Build_PDB(g, tiles, getPDB7a(weighted),
						  std::thread::hardware_concurrency(), false);
			mnp.ClearPDBs();
		}
		
		if (!fileExists(getPDB8a(weighted)))
		{
			g.Reset();
			tiles.resize(0);
			//for (int x = 0; x <= 14; x+=2)
			for (int x = 0; x <= 7; x++)
				tiles.push_back(x);
			
			mnp.Build_PDB(g, tiles, getPDB8a(weighted),
						  std::thread::hardware_concurrency(), false);
			mnp.ClearPDBs();
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
			
			mnp.Build_PDB(g, tiles, getPDB7b(weighted),
						  std::thread::hardware_concurrency(), false);
			mnp.ClearPDBs();
		}
		
		if (!fileExists(getPDB8b(weighted)))
		{
			g.Reset();
			tiles.resize(0);
			for (int x = 8; x <= 15; x++)
			//for (int x = 1; x <= 15; x+=2)
				tiles.push_back(x);
			
			mnp.Build_PDB(g, tiles, getPDB8b(weighted),
						  std::thread::hardware_concurrency(), false);
			mnp.ClearPDBs();
		}
	}
}

#pragma mark heuristic value tests

void ModValueCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	MNPuzzle mnp(N, k);
	mnp.SetWeighted(weighted);
	MNPuzzleState s(N, k);
	MNPuzzleState g(N, k);
	
	mnp.StoreGoal(g);
	mnp.ClearPDBs();
	
	BuildPDBs(true, false, weighted);
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Compressing (mod) by factor of %d\n", x);
		mnp.ClearPDBs();
		//mnp.Load_Regular_PDB(getPDB7a(weighted), g, true);
		uint64_t oldSize = mnp.Get_PDB_Size(g, 8);
		uint64_t newSize = oldSize / x;
		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true);
		mnp.Mod_Compress_PDB(0, newSize, true);
		mnp.lookups.push_back({kLeafModCompress, -0, -0, -0});
		MeasureIR(mnp);
	}
}

void ModValueDeltaCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	MNPuzzle mnp(N, k);
	mnp.SetWeighted(weighted);
	MNPuzzleState s(N, k);
	MNPuzzleState g(N, k);
	
	mnp.StoreGoal(g);
	mnp.ClearPDBs();
	
	BuildPDBs(true, false, weighted);
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Compressing (mod-delta) by factor of %d\n", x);
		mnp.ClearPDBs();
		mnp.Load_Regular_PDB(getPDB7a(weighted), g, true);
		mnp.lookups.push_back({kLeafNode, -0, -0, 0});

		uint64_t oldSize = mnp.Get_PDB_Size(g, 8);
		uint64_t newSize = oldSize / x;
		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true);
		mnp.Delta_Compress_PDB(g, 1, true);
		mnp.Mod_Compress_PDB(1, newSize, true);
//		mnp.lookups.push_back({kLeafModCompress, -0, -0, -0});
//		MeasureIR(mnp);
	}
}

void DivValueCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	MNPuzzle mnp(N, k);
	mnp.SetWeighted(weighted);
	MNPuzzleState s(N, k);
	MNPuzzleState g(N, k);
	
	mnp.StoreGoal(g);
	mnp.ClearPDBs();
	
	if (!fileExists(getPDB7a(weighted)))
	{
		g.Reset();
		tiles.resize(0);
		for (int x = 0; x <= 6; x++)
			tiles.push_back(x);
		
		mnp.Build_PDB(g, tiles, getPDB7a(weighted),
					  std::thread::hardware_concurrency(), false);
		mnp.ClearPDBs();
	}
	
	if (!fileExists(getPDB8a(weighted)))
	{
		g.Reset();
		tiles.resize(0);
		for (int x = 0; x < 8; x++)
			tiles.push_back(x);
		
		mnp.Build_PDB(g, tiles, getPDB8a(weighted),
					  std::thread::hardware_concurrency(), false);
		mnp.ClearPDBs();
	}
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Compressing (div) by factor of %d\n", x);
		mnp.ClearPDBs();
		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true);
		mnp.Min_Compress_PDB(0, x, true);
		mnp.lookups.push_back({kLeafMinCompress, static_cast<uint8_t>(x), -0, 0});
		MeasureIR(mnp);
	}
}

void DivDeltaValueCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	MNPuzzle mnp(N, k);
	mnp.SetWeighted(weighted);
	MNPuzzleState s(N, k);
	MNPuzzleState g(N, k);
	
	mnp.StoreGoal(g);
	mnp.ClearPDBs();
	
	if (!fileExists(getPDB7a(weighted)))
	{
		g.Reset();
		tiles.resize(0);
		for (int x = 0; x <= 6; x++)
			tiles.push_back(x);
		
		mnp.Build_PDB(g, tiles, getPDB7a(weighted),
					  std::thread::hardware_concurrency(), false);
		mnp.ClearPDBs();
	}
	
	if (!fileExists(getPDB8a(weighted)))
	{
		g.Reset();
		tiles.resize(0);
		for (int x = 0; x < 8; x++)
			tiles.push_back(x);
		
		mnp.Build_PDB(g, tiles, getPDB8a(weighted),
					  std::thread::hardware_concurrency(), false);
		mnp.ClearPDBs();
	}
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Compressing (div delta) by factor of %d\n", x);
		mnp.ClearPDBs();
		mnp.Load_Regular_PDB(getPDB7a(weighted), g, false);
		mnp.lookups.push_back({kLeafNode, -0, -0, 0});

		mnp.Load_Regular_PDB(getPDB8a(weighted), g, false);
		mnp.Delta_Compress_PDB(g, 1, true);
		mnp.Min_Compress_PDB(1, x, true);
		//MeasureIR(mnp);
	}
}

void BitDeltaValueCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	MNPuzzle mnp(N, k);
	mnp.SetWeighted(weighted);
	MNPuzzleState s(N, k);
	MNPuzzleState g(N, k);
	
	mnp.StoreGoal(g);
	mnp.ClearPDBs();
	
	BuildPDBs(true, false, weighted);
	
	for (int x = 1; x <= 4; x*=2)
	{
		g.Reset();
		printf("==>Compressing (value-range-delta) to %d bits\n", x);
		mnp.ClearPDBs();
		mnp.Load_Regular_PDB(getPDB7a(weighted), g, false);
		mnp.lookups.push_back({kLeafNode, -0, -0, 0});
		
		mnp.Load_Regular_PDB(getPDB8a(weighted), g, false);
		mnp.Delta_Compress_PDB(g, 1, true);
		std::vector<int> cutoffs;
		GetBitValueCutoffs(cutoffs, x);
		mnp.Value_Compress_PDB(1, cutoffs, true);
		//MeasureIR(mnp);
	}
}

#pragma mark node expansion tests

void BaseHeuristicTest(bool weighted)
{
	std::vector<int> tiles;
	
	MNPuzzle mnp(N, k);
	mnp.SetWeighted(weighted);
	MNPuzzleState s(N, k);
	MNPuzzleState g(N, k);
	
	mnp.StoreGoal(g);
	
	mnp.ClearPDBs();
	BuildPDBs(true, true, weighted);
	
	{
		mnp.ClearPDBs();
		mnp.Load_Regular_PDB(getPDB7a(weighted), g, true);
		mnp.Load_Regular_PDB(getPDB7b(weighted), g, true);
		mnp.lookups.push_back({kMaxNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
		mnp.lookups.push_back({kLeafNode, -0, -0, 0});
		mnp.lookups.push_back({kLeafNode, -0, -0, 1});
		std::string desc = "BASE7-";
		Test(mnp, desc.c_str());
	}
	
	{
		mnp.ClearPDBs();
		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true);
		mnp.Load_Regular_PDB(getPDB8b(weighted), g, true);
		mnp.lookups.push_back({kMaxNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
		mnp.lookups.push_back({kLeafNode, -0, -0, 0});
		mnp.lookups.push_back({kLeafNode, -0, -0, 1});
		std::string desc = "BASE8-";
		Test(mnp, desc.c_str());
	}
}

//Fractional_Mod_Compress_PDB
void FractionalNodesCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	MNPuzzle mnp(N, k);
	mnp.SetWeighted(weighted);
	MNPuzzleState s(N, k);
	MNPuzzleState g(N, k);
	
	mnp.StoreGoal(g);
	mnp.ClearPDBs();
	
	BuildPDBs(true, true, weighted);
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Fractional (contiguous): Reducing by factor of %d\n", x);
		mnp.ClearPDBs();
		uint64_t oldSize = mnp.Get_PDB_Size(g, 8);
		uint64_t newSize = oldSize / x;
		mnp.Load_Regular_PDB(getPDB7a(weighted), g, true);
		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true);
		mnp.Load_Regular_PDB(getPDB7b(weighted), g, true);
		mnp.Load_Regular_PDB(getPDB8b(weighted), g, true);
		mnp.Fractional_Compress_PDB(1, newSize, true);
		mnp.Fractional_Compress_PDB(3, newSize, true);
		mnp.lookups.push_back({kMaxNode, 4, 1, -0}); // max of 2 children starting at 1 in the tree
		mnp.lookups.push_back({kLeafNode, -0, -0, 0});
		mnp.lookups.push_back({kLeafFractionalCompress, -0, -0, 1});
		mnp.lookups.push_back({kLeafNode, -0, -0, 2});
		mnp.lookups.push_back({kLeafFractionalCompress, -0, -0, 3});
		std::string desc = "FCT-C-";
		desc += ('0'+x);
		Test(mnp, desc.c_str());
	}
}

void FractionalModNodesCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	MNPuzzle mnp(N, k);
	mnp.SetWeighted(weighted);
	MNPuzzleState s(N, k);
	MNPuzzleState g(N, k);
	
	mnp.StoreGoal(g);
	mnp.ClearPDBs();
	
	BuildPDBs(true, true, weighted);
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Fractional MOD: Reducing by factor of %d\n", x);
		mnp.ClearPDBs();
		mnp.Load_Regular_PDB(getPDB7a(weighted), g, true);
		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true);
		mnp.Load_Regular_PDB(getPDB7b(weighted), g, true);
		mnp.Load_Regular_PDB(getPDB8b(weighted), g, true);
		mnp.Fractional_Mod_Compress_PDB(1, x, true);
		mnp.Fractional_Mod_Compress_PDB(3, x, true);
		mnp.lookups.push_back({kMaxNode, 4, 1, -0}); // max of 2 children starting at 1 in the tree
		mnp.lookups.push_back({kLeafNode, -0, -0, 0});
		mnp.lookups.push_back({kLeafFractionalModCompress, static_cast<uint8_t>(x), -0, 1}); // factor, -- , id
		mnp.lookups.push_back({kLeafNode, -0, -0, 2});
		mnp.lookups.push_back({kLeafFractionalModCompress, static_cast<uint8_t>(x), -0, 3});
		std::string desc = "FCT-M-";
		desc += ('0'+x);
		Test(mnp, desc.c_str());
	}
}


void ModNodesCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	MNPuzzle mnp(N, k);
	mnp.SetWeighted(weighted);
	MNPuzzleState s(N, k);
	MNPuzzleState g(N, k);
	
	mnp.StoreGoal(g);
	mnp.ClearPDBs();
	
	BuildPDBs(true, true, weighted);
	
	for (int x = 2; x <= 10; x++)
	{
		g.Reset();
		printf("==>Compressing (mod) by factor of %d\n", x);
		mnp.ClearPDBs();
		uint64_t oldSize = mnp.Get_PDB_Size(g, 8);
		uint64_t newSize = oldSize / x;
		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true);
		mnp.Load_Regular_PDB(getPDB8b(weighted), g, true);
		mnp.Mod_Compress_PDB(0, newSize, true);
		mnp.Mod_Compress_PDB(1, newSize, true);
		mnp.lookups.push_back({kMaxNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
		mnp.lookups.push_back({kLeafModCompress, -0, -0, 0});
		mnp.lookups.push_back({kLeafModCompress, -0, -0, 1});

		std::string desc = "MOD-";
		desc += ('0'+x);
		Test(mnp, desc.c_str());
	}
}

void ModNodesDeltaCompressionTest(bool weighted)
{
//	std::vector<int> tiles;
//	
//	MNPuzzle mnp(N, k);
//	MNPuzzleState s(N, k);
//	MNPuzzleState g(N, k);
//	
//	mnp.StoreGoal(g);
//	mnp.ClearPDBs();
//	
//	BuildPDBs(true, true, weighted);
//	
//	for (int x = 2; x <= 10; x++)
//	{
//		g.Reset();
//		printf("==>MOD Compressing by factor of %d\n", x);
//		mnp.ClearPDBs();
//		mnp.Load_Regular_PDB(getPDB7a(weighted), g, true);
//		mnp.lookups.push_back({kLeafNode, -0, -0, 0});
//		
//		uint64_t oldSize = mnp.Get_PDB_Size(g, 8);
//		uint64_t newSize = oldSize / x;
//		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true);
//		mnp.Delta_Compress_PDB(g, 1, true);
//		mnp.Mod_Compress_PDB(1, newSize, true);
//		//		mnp.lookups.push_back({kLeafModCompress, -0, -0, -0});
//		//		MeasureIR(mnp);
//	}
}

void DivNodesCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	MNPuzzle mnp(N, k);
	mnp.SetWeighted(weighted);
	MNPuzzleState s(N, k);
	MNPuzzleState g(N, k);
	
	mnp.StoreGoal(g);
	mnp.ClearPDBs();
	
	BuildPDBs(true, true, weighted);
	
	for (int x = 1; x <= 10; x++)
	{
		g.Reset();
		printf("==>Compressing (div) by factor of %d\n", x);
		mnp.ClearPDBs();
		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true);
		mnp.Min_Compress_PDB(0, x, true);
		mnp.Load_Regular_PDB(getPDB8b(weighted), g, true);
		mnp.Min_Compress_PDB(1, x, true);

		mnp.lookups.push_back({kMaxNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
		mnp.lookups.push_back({kLeafMinCompress, static_cast<uint8_t>(x), -0, 0});
		mnp.lookups.push_back({kLeafMinCompress, static_cast<uint8_t>(x), -0, 1});

		std::string desc = "DIV-";
		desc += ('0'+x);
		Test(mnp, desc.c_str());
		//MeasureIR(mnp);
	}
}

void DivDeltaNodesCompressionTest(bool weighted)
{
//	std::vector<int> tiles;
//	
//	MNPuzzle mnp(N, k);
//	MNPuzzleState s(N, k);
//	MNPuzzleState g(N, k);
//	
//	mnp.StoreGoal(g);
//	mnp.ClearPDBs();
//	
//	BuildPDBs(true, true, weighted);
//	
//	for (int x = 2; x <= 10; x++)
//	{
//		g.Reset();
//		printf("==>Compressing by factor of %d\n", x);
//		mnp.ClearPDBs();
//		mnp.Load_Regular_PDB(getPDB7a(weighted), g, false);
//		mnp.lookups.push_back({kLeafNode, -0, -0, 0});
//		
//		mnp.Load_Regular_PDB(getPDB8a(weighted), g, false);
//		mnp.Delta_Compress_PDB(g, 1, true);
//		mnp.Min_Compress_PDB(1, x, true);
//		//MeasureIR(mnp);
//	}
}


void BitDeltaNodesCompressionTest(bool weighted)
{
	std::vector<int> tiles;
	
	MNPuzzle mnp(N, k);
	mnp.SetWeighted(weighted);
	MNPuzzleState s(N, k);
	MNPuzzleState g(N, k);
	
	mnp.StoreGoal(g);
	mnp.ClearPDBs();
	
	BuildPDBs(true, true, weighted);
	
	for (int x = 1; x <= 4; x*=2)
	{
		std::vector<int> cutoffs;
		GetBitValueCutoffs(cutoffs, x);

		g.Reset();
		printf("==>Compressing to %d bits\n", x);
		mnp.ClearPDBs();
		mnp.Load_Regular_PDB(getPDB7a(weighted), g, false); // PDB 0
		mnp.Load_Regular_PDB(getPDB8a(weighted), g, false); // PDB 1
		mnp.lookups.push_back({kLeafNode, -0, -0, 0});
		mnp.Delta_Compress_PDB(g, 1, true);
		mnp.Value_Compress_PDB(1, cutoffs, true);
		
		mnp.Load_Regular_PDB(getPDB7b(weighted), g, false); // PDB 2
		mnp.Load_Regular_PDB(getPDB8b(weighted), g, false); // PDB 3
		mnp.lookups.back().PDBID = 2;
		mnp.Delta_Compress_PDB(g, 3, true);
		mnp.Value_Compress_PDB(3, cutoffs, true);


		mnp.lookups.resize(0);
		mnp.lookups.push_back({kMaxNode, 2, 1, -0});
		mnp.lookups.push_back({kAddNode, 2, 3, -0});
		mnp.lookups.push_back({kAddNode, 2, 5, -0});
		mnp.lookups.push_back({kLeafNode, -0, -0, 0});
		mnp.lookups.push_back({kLeafNode, -0, -0, 1});
		mnp.lookups.push_back({kLeafNode, -0, -0, 2});
		mnp.lookups.push_back({kLeafNode, -0, -0, 3});

		std::string desc = "VALRNG-";
		desc += ('0'+x);
		Test(mnp, desc.c_str());
	}
}

#pragma mark other utilities

void GetBitValueCutoffs(std::vector<int> &cutoffs, int bits)
{
	MNPuzzle mnp(N, k);
	MNPuzzleState g(N, k);
	g.Reset();
	
	if (mnp.GetWeighted()) // non-uniform costs
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

void MeasureIR(MNPuzzle &mnp)
{
	srandom(1234);
	MNPuzzleState s(16, 4);
	MNPuzzleState g(16, 4);
	g.Reset();
	mnp.StoreGoal(g);
	//mnp.SetPruneSuccessors(true);
	
	IDAStar<MNPuzzleState, slideDir> ida;
	std::vector<slideDir> path1;
	MNPuzzleState start;
	Timer t;
	t.StartTimer();

	uint64_t count = mnp.Get_PDB_Size(s, 16);
	double sumg = 0, sumh = 0;
	int total = 1000000;
	std::vector<slideDir> acts;
	for (int x = 0; x < total; x++)
	{
		s.Reset();
		mnp.GetStateFromHash(s, random64()%count);
		mnp.GetActions(s, acts);
		mnp.GetNextState(s, acts[random()%acts.size()], g);
		
		sumg += mnp.GCost(s, g);
		sumh += fabs(mnp.PermutationPuzzleEnvironment<MNPuzzleState,slideDir>::HCost(s)-
					 mnp.PermutationPuzzleEnvironment<MNPuzzleState,slideDir>::HCost(g));
//		printf("G: %1.0f ∆H: %1.0f\n", mnp.GCost(s, g),
//			   fabs(mnp.PermutationPuzzleEnvironment<MNPuzzleState,slideDir>::HCost(s)-
//					mnp.PermutationPuzzleEnvironment<MNPuzzleState,slideDir>::HCost(g)));
	}
	printf("Average G: %1.1f, average H: %1.3f\n", sumg/total, sumh/total);
}


void Test(MNPuzzle &mnp, const char *prefix)
{
	MNPuzzleState s(16, 4);
	MNPuzzleState g(16, 4);
	g.Reset();
	mnp.StoreGoal(g);
	
	IDAStar<MNPuzzleState, slideDir> ida;
	ida.SetUseBDPathMax(true);
	std::vector<slideDir> path1;
	MNPuzzleState start;
	Timer t1;
	t1.StartTimer();
	uint64_t nodes = 0;
	double totaltime = 0;
	for (int x = 0; x < 100; x++)
	{
		s = GetInstance(x, mnp.GetWeighted());
		g.Reset();
		printf("Problem %d of %d\n", x+1, 100);
		std::cout << "Searching from: " << std::endl << s << std::endl << g << std::endl;
		Timer t;
		t.StartTimer();
		ida.GetPath(&mnp, s, g, path1);
		t.EndTimer();
		totaltime += t.GetElapsedTime();
		std::cout << "Path found, length " << path1.size() << " time:" << t.GetElapsedTime() << std::endl;
		nodes += ida.GetNodesExpanded();
	}
	printf("%s: %1.2fs elapsed; %llu nodes expanded\n", prefix, t1.EndTimer(), nodes);
}


MNPuzzleState GetKorfInstance(int which);

MNPuzzleState GetInstance(int which, bool weighted)
{
	return GetKorfInstance(which);
//	srandom(which*101+11);
//	MNPuzzle mnp(16, 4);
//	MNPuzzleState s(16,4);
//	s.Reset();
//	int length = 10000;
//	for (int x = 0; x < length; x++)
//	{
//		mnp.ApplyAction(s, random()%16);
//	}
//	return s;
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
