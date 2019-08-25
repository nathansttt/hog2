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

#include <cstring>
#include "Common.h"
#include "PermutationPDB.h"
#include "LexPermutationPDB.h"
#include "MR1PermutationPDB.h"
#include "Driver.h"
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include "Plot2D.h"
#include "RandomUnit.h"
#include "MNPuzzle.h"
#include "IDAStar.h"
#include "ParallelIDAStar.h"
#include "Timer.h"
#include "STPInstances.h"

void CompareToMinCompression();
void CompareToSmallerPDB();

void BuildSTP_PDB(unsigned long windowID, tKeyboardModifier , char);
void STPTest(unsigned long , tKeyboardModifier , char);
void Test(MNPuzzle<4, 4> &mnp, const char *prefix);
void MinCompressionTest();
void MeasureIR(MNPuzzle<4, 4> &mnp);
void GetBitValueCutoffs(std::vector<int> &cutoffs, int bits);
void BaselineTest();
void WeightedTest(unsigned long , tKeyboardModifier , char);

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
void DivNodesDeltaCompressionTest(bool weighted);

MNPuzzle<4, 4> *mnp = 0;

bool recording = false;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 640, 640);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Test", "Basic test with MD heuristic", kAnyModifier, 't');
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');

	InstallKeyboardHandler(WeightedTest, "Weighted STP Test", "Test the STP with weights", kNoModifier, 'w');
	InstallKeyboardHandler(STPTest, "STP Test", "Test the STP PDBs", kNoModifier, 'd');
	InstallKeyboardHandler(BuildSTP_PDB, "Build STP PDBs", "Build PDBs for the STP", kNoModifier, 'a');

	InstallCommandLineHandler(MyCLHandler, "-run", "-run", "Runs pre-set experiments.");
	InstallCommandLineHandler(MyCLHandler, "-test", "-test", "Basic test with MD heuristic");
	
	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler);
}

MNPuzzleState<4, 4> s, t;
std::vector<slideDir> moves;
double v = 1;

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

		mnp = new MNPuzzle<4, 4>;
		if (0)
		{
			IDAStar<MNPuzzleState<4, 4>, slideDir> ida;
			s = STP::GetKorfInstance(1);
			ida.GetPath(mnp, s, t, moves);
			v = 5;
			std::cout << s << std::endl;
			for (int x = 0; x < moves.size(); x++)
			{
				std::cout << moves[x] << " ";
			}
			std::cout << std::endl;
			t = s;
			//		recording = true;
		}

		{
			//std::vector<int> pattern = {1, 2, 3, 4, 5, 6, 7, 8, 0};//{8, 9, 10, 11, 12, 13, 14, 15, 0};
			//std::vector<int> pattern = {1, 2, 3, 4, 5, 6, 7, 0};//{8, 9, 10, 11, 12, 13, 14, 15, 0};

			// Build & compress 8-15 Additive PDB
			if (0)
			{
				std::vector<int> pattern = {8, 9, 10, 11, 12, 13, 14, 15, 0};
				t.Reset();
				mnp->SetPattern(pattern);
				mnp->StoreGoal(t);
				LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb2(mnp, t, pattern);
				pdb2.BuildAdditivePDB(t, std::thread::hardware_concurrency());
				printf("Starting Delta Compress\n");
				pdb2.DeltaCompress(mnp, t, true);
				printf("Starting DIV Compress\n");
				pdb2.DivCompress(17-pattern.size(), true);
				printf("PDB After DIV:\n");
				for (int x = 0; x < 20; x++)
				{
					pdb2.GetStateFromPDBHash(x, t);
					std::cout << t << " - " << pdb2.HCost(t, t);
					std::cout << "\n";
					//pdb2.Heuristic<MNPuzzleState<4, 4> >::HCost(t, t);
				}
				pdb2.Save("/Users/nathanst/");
			}
			
			// Build & compress 8-15 Additive PDB
			for (int x = 1; x < 4; x++)
			{
				std::vector<std::vector<int>> patterns = {{2, 3, 4, 7, 8, 9, 0}, {1, 5, 6, 10, 11, 12, 0}, {13, 14, 18, 19, 23, 24, 0}, {15, 16, 17, 20, 21, 22, 0}};

				std::vector<int> pattern = patterns[x];
				MNPuzzle<5, 5> mnp55;
				MNPuzzleState<5, 5> t;
				t.Reset();
				mnp55.SetPattern(pattern);
				mnp55.StoreGoal(t);
				LexPermutationPDB<MNPuzzleState<5, 5>, slideDir, MNPuzzle<5, 5>> pdb(&mnp55, t, pattern);
				pdb.BuildAdditivePDB(t, std::thread::hardware_concurrency());
				printf("Starting Delta Compress\n");
				pdb.DeltaCompress(&mnp55, t, true);
				printf("Starting DIV Compress\n");
				pdb.DivCompress(26-pattern.size(), true);
				for (int x = 0; x < 20; x++)
				{
					pdb.GetStateFromPDBHash(x, t);
					std::cout << t << " - " << pdb.HCost(t, t);
					std::cout << "\n";
				}
				pdb.Save("/Users/nathanst/");
			}

			exit(0);
		}
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	//mnp->OpenGLDraw(s);

	v += 0.1;
	if (v > 1 && moves.size() > 0)
	{
		t = s;
		mnp->ApplyAction(s, moves[0]);
		v = 0;
		moves.erase(moves.begin());
	}
	if (v > 1 && moves.size() == 0)
	{
		v = 1;
		recording = false;
	}
	mnp->OpenGLDraw(s, t, v);

	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		static int cnt = 0;
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/%d%d%d%d", (cnt/1000)%10, (cnt/100)%10, (cnt/10)%10, cnt%10);
		SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
		cnt++;
	}
	return;
	
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-test") == 0)
	{
		BaselineTest();
		exit(0);
	}
	BuildSTP_PDB(0, kNoModifier, 'a');
	exit(0);
	return 2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 't': BaselineTest(); break;
		case 'r': recording = !recording; break;
		case '0': mnp->ApplyAction(s, kUp); break;
		case '1': mnp->ApplyAction(s, kDown); break;
		case '2': mnp->ApplyAction(s, kLeft); break;
		case '3': mnp->ApplyAction(s, kRight); break;
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
		{
			std::vector<int> pattern = {0, 1, 2, 3, 4, 5, 6};
			if (0)
			{
				t.Reset();
				MR1PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb(mnp, t, pattern);
				MR1PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb2(mnp, t, pattern);
				pdb.BuildPDB(t, std::thread::hardware_concurrency());
				pdb.PrintHistogram();
				pdb.DivCompress(5, true);
			}
			{
				t.Reset();
				LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb(mnp, t, pattern);
				pdb.BuildPDB(t, std::thread::hardware_concurrency());
				t.Reset();
				mnp->StoreGoal(t);
				pdb.DeltaCompress(mnp, t, true);
//				pdb.DivCompress(10, true);
//
//				PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle> pdb2(ts, t, pattern);
//				pdb2.BuildPDB(t, std::thread::hardware_concurrency());
//				pdb2.DeltaCompress(&pdb, t, true);
			}
		}
			break;
		case 'o':
		{
			if (!mnp) break;
			if (mod == kShiftDown)
			{
				IDAStar<MNPuzzleState<4, 4>, slideDir> ida;
				std::vector<slideDir> path1;
				std::vector<MNPuzzleState<4, 4>> path2;
				MNPuzzleState<4, 4> s;
				MNPuzzleState<4, 4> g;
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

void GetSTPInstance(const MNPuzzle<4, 4> &ts, MNPuzzleState<4, 4> &theState, int which)
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
//	MNPuzzle<4, 4> mnp;
//	MNPuzzleState s1(4, 4);
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

	bool weighted = false;

//	BaseHeuristicTest(weighted);
//	FractionalModNodesCompressionTest(weighted);
	FractionalNodesCompressionTest(weighted);
//	BitDeltaNodesCompressionTest(weighted);
//	DivNodesCompressionTest(weighted);
//	DivNodesDeltaCompressionTest(weighted);
//	ModNodesCompressionTest(weighted);
//	ModNodesDeltaCompressionTest(weighted);

	exit(0);
}

void WeightedTest(unsigned long , tKeyboardModifier , char)
{
	std::vector<slideDir> path;
	IDAStar<MNPuzzleState<4, 4>, slideDir> ida;
	MNPuzzle<4, 4> mnp;
	mnp.SetWeighted(kSquared);
	MNPuzzleState<4, 4> s, g;
	s = STP::GetKorfInstance(0);
	ida.GetPath(&mnp, s, g, path);
}

void STPTest(unsigned long , tKeyboardModifier , char)
{
	assert("!Code currently not using refactored PDB setup; needs to be re-written");
//	int N = 4, k = 4;
//	std::vector<int> tiles;
//
//	MNPuzzle<4, 4> mnp;
//	MNPuzzleState<4, 4> s;
//	MNPuzzleState<4, 4> g;
//
////	if (ts->PDB.size() == 0)
////	{
////		mnp.Load_Regular_PDB("/Users/nathanst/Desktop/STP_0-5.pdb", g, true);
////		mnp.Load_Regular_PDB("/Users/nathanst/Desktop/STP_12-15.pdb", g, true);
////		mnp.Load_Regular_PDB("/Users/nathanst/Desktop/STP_6-11.pdb", g, true);
////		mnp.lookups.push_back({kMaxNode, 3, 1, 0});
////		mnp.lookups.push_back({kLeafNode, 0, 0, 0});
////		mnp.lookups.push_back({kLeafNode, 0, 0, 1});
////		mnp.lookups.push_back({kLeafNode, 0, 0, 2});
////	}
//
//	if (ts->PDB.size() == 0)
//	{
//		mnp.Load_Regular_PDB("/Users/nathanst/Desktop/STP_0-3.pdb", g, true);
//		mnp.Load_Regular_PDB("/Users/nathanst/Desktop/STP_0-5+.pdb", g, true);
//		mnp.Load_Regular_PDB("/Users/nathanst/Desktop/STP_6-9.pdb", g, true);
//		mnp.Load_Regular_PDB("/Users/nathanst/Desktop/STP_6-11+.pdb", g, true);
//		mnp.Load_Regular_PDB("/Users/nathanst/Desktop/STP_12-15.pdb", g, true);
//		mnp.lookups.push_back({PermutationPuzzle::kMaxNode, 3, 1, 0});
//		mnp.lookups.push_back({PermutationPuzzle::kAddNode, 2, 4, 0});
//		mnp.lookups.push_back({PermutationPuzzle::kAddNode, 2, 6, 0});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, 0, 0, 4});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, 0, 0, 0});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, 0, 0, 1});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, 0, 0, 2});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, 0, 0, 3});
//	}
//
//	
//	IDAStar<MNPuzzleState<4, 4>, slideDir> ida;
//	std::vector<slideDir> path1;
//
//
//	//mnp.SetPruneSuccessors(true);
//	for (int x = 0; x < 100; x++)
//	{
//		GetSTPInstance(mnp, s, x);
//		std::cout << "Problem " << x << std::endl;
//		std::cout << "Searching from: " << std::endl << s << std::endl << g << std::endl;
//		Timer t;
//		t.StartTimer();
//		ida.GetPath(&mnp, s, g, path1);
//		t.EndTimer();
//		std::cout << "Path found, length " << path1.size() << " time:" << t.GetElapsedTime() << " ";
//		std::cout << ida.GetNodesExpanded() << " nodes expanded" << std::endl;
////		for (int x = 0; x < ts->histogram.size(); x++)
////		{
////			printf("%2d  %d\n", x, ts->histogram[x]);
////		}
//		
////		mnp.PrintHStats();
//	}
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
const int N = 4, k = 4;
const char *prefix = "/Users/nathanst/Desktop/";

//const char *getPDB7a(bool weighted)
//{
//	static std::string s;
//	s = prefix;
//	if (!weighted)
//		s += "STP-0-6.pdb";
//	else
//		s += "wSTP-0-6.pdb";
//	return s.c_str();
//}

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

const char *getPDB8b(bool weighted)
{
	static std::string s;
	s = prefix;
	if (!weighted)
		s += "STP-0+8-14.pdb";
	else
		s += "wSTP-0+8-14.pdb";
	return s.c_str();
}
const char *getPDB1b(bool weighted)
{
	static std::string s;
	s = prefix;
	if (!weighted)
		s += "STP-0+15.pdb";
	else
		s += "wSTP-0+15.pdb";
	return s.c_str();
}

const char *getPDB9b(bool weighted)
{
	static std::string s;
	s = prefix;
	if (!weighted)
		s += "STP-0+8-15.pdb";
	else
		s += "wSTP-0+8-15.pdb";
	return s.c_str();
}

void BuildPDBs(bool aPDBs, bool bPDBs, bool weighted)
{
	assert("!Code currently not using refactored PDB setup; needs to be re-written");
//	MNPuzzle<4, 4> mnp;
//	mnp.SetWeighted(weighted);
//	std::vector<int> tiles;
//	
//	MNPuzzleState<4, 4> s;
//	MNPuzzleState<4, 4> g;
//	
//	mnp.StoreGoal(g);
//	mnp.ClearPDBs();
//
//	if (aPDBs)
//	{
//		if (!fileExists(getPDB8a(weighted)))
//		{
//			g.Reset();
//			tiles.resize(0);
//			//for (int x = 0; x <= 14; x+=2)
//			for (int x = 0; x <= 7; x++)
//				tiles.push_back(x);
//			
//			mnp.Build_PDB(g, tiles, getPDB8a(weighted),
//						  std::thread::hardware_concurrency(), true);
//			mnp.ClearPDBs();
//		}
//	}
//	
//	if (bPDBs)
//	{
//		if (!fileExists(getPDB8b(weighted)))
//		{
//			g.Reset();
//			tiles.resize(0);
//			tiles.push_back(0);
//			for (int x = 8; x <= 14; x++)
//				//for (int x = 1; x <= 14; x+=2)
//				tiles.push_back(x);
//			
//			mnp.Build_PDB(g, tiles, getPDB8b(weighted),
//						  std::thread::hardware_concurrency(), true);
//			mnp.ClearPDBs();
//		}
//
//		if (!fileExists(getPDB1b(weighted)))
//		{
//			g.Reset();
//			tiles.resize(0);
//			tiles.push_back(0);
//			tiles.push_back(15);
//			
//			mnp.Build_PDB(g, tiles, getPDB1b(weighted),
//						  std::thread::hardware_concurrency(), true);
//			mnp.ClearPDBs();
//		}
//
//		if (!fileExists(getPDB9b(weighted)))
//		{
//			g.Reset();
//			tiles.resize(0);
//			tiles.push_back(0);
//			for (int x = 8; x <= 15; x++)
//			//for (int x = 1; x <= 15; x+=2)
//				tiles.push_back(x);
//			
//			mnp.Build_PDB(g, tiles, getPDB9b(weighted),
//						  std::thread::hardware_concurrency(), true);
//			mnp.ClearPDBs();
//		}
//	}
}

#pragma mark heuristic value tests

void ModValueCompressionTest(bool weighted)
{
	assert("!Code currently not using refactored PDB setup; needs to be re-written");

//	std::vector<int> tiles;
//	
//	MNPuzzle<4, 4> mnp;
//	mnp.SetWeighted(weighted);
//	MNPuzzleState<4, 4> s;
//	MNPuzzleState<4, 4> g;
//	
//	mnp.StoreGoal(g);
//	mnp.ClearPDBs();
//	
//	BuildPDBs(true, false, weighted);
//	
//	for (int x = 2; x <= 10; x++)
//	{
//		g.Reset();
//		printf("==>Compressing (mod) by factor of %d\n", x);
//		mnp.ClearPDBs();
//		//mnp.Load_Regular_PDB(getPDB7a(weighted), g, true);
//		uint64_t oldSize = mnp.Get_PDB_Size(g, 8);
//		uint64_t newSize = oldSize / x;
//		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true);
//		mnp.Mod_Compress_PDB(0, newSize, true);
//		mnp.lookups.push_back({PermutationPuzzle::kLeafModCompress, -0, -0, -0});
//		MeasureIR(mnp);
//	}
}

void ModValueDeltaCompressionTest(bool weighted)
{
//	std::vector<int> tiles;
//	
//	MNPuzzle<4, 4> mnp;
//	mnp.SetWeighted(weighted);
//	MNPuzzleState s(4, 4);
//	MNPuzzleState g(4, 4);
//	
//	mnp.StoreGoal(g);
//	mnp.ClearPDBs();
//	
//	BuildPDBs(true, false, weighted);
//	
//	for (int x = 2; x <= 10; x++)
//	{
//		g.Reset();
//		printf("==>Compressing (mod-delta) by factor of %d\n", x);
//		mnp.ClearPDBs();
//		mnp.Load_Regular_PDB(getPDB8b(weighted), g, true);
//		mnp.lookups.push_back({kLeafNode, -0, -0, 0});
//
//		uint64_t oldSize = mnp.Get_PDB_Size(g, 9);
//		uint64_t newSize = oldSize / x;
//		mnp.Load_Regular_PDB(getPDB9b(weighted), g, true);
//		mnp.Delta_Compress_PDB(g, 1, true);
//		mnp.Mod_Compress_PDB(1, newSize, true);
////		mnp.lookups.push_back({kLeafModCompress, -0, -0, -0});
////		MeasureIR(mnp);
//	}
}

void DivValueCompressionTest(bool weighted)
{
//	std::vector<int> tiles;
//	
//	MNPuzzle<4, 4> mnp;
//	mnp.SetWeighted(weighted);
//	MNPuzzleState s(4, 4);
//	MNPuzzleState g(4, 4);
//	
//	mnp.StoreGoal(g);
//	mnp.ClearPDBs();
//	
//	if (!fileExists(getPDB7a(weighted)))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 0; x <= 6; x++)
//			tiles.push_back(x);
//		
//		mnp.Build_PDB(g, tiles, getPDB7a(weighted),
//					  std::thread::hardware_concurrency(), false);
//		mnp.ClearPDBs();
//	}
//	
//	if (!fileExists(getPDB8a(weighted)))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 0; x < 8; x++)
//			tiles.push_back(x);
//		
//		mnp.Build_PDB(g, tiles, getPDB8a(weighted),
//					  std::thread::hardware_concurrency(), false);
//		mnp.ClearPDBs();
//	}
//	
//	for (int x = 2; x <= 10; x++)
//	{
//		g.Reset();
//		printf("==>Compressing (div) by factor of %d\n", x);
//		mnp.ClearPDBs();
//		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true);
//		mnp.Min_Compress_PDB(0, x, true);
//		mnp.lookups.push_back({kLeafMinCompress, static_cast<uint8_t>(x), -0, 0});
//		MeasureIR(mnp);
//	}
}

void DivDeltaValueCompressionTest(bool weighted)
{
//	std::vector<int> tiles;
//	
//	MNPuzzle<4, 4> mnp;
//	mnp.SetWeighted(weighted);
//	MNPuzzleState s(4, 4);
//	MNPuzzleState g(4, 4);
//	
//	mnp.StoreGoal(g);
//	mnp.ClearPDBs();
//	
//	if (!fileExists(getPDB7a(weighted)))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 0; x <= 6; x++)
//			tiles.push_back(x);
//		
//		mnp.Build_PDB(g, tiles, getPDB7a(weighted),
//					  std::thread::hardware_concurrency(), false);
//		mnp.ClearPDBs();
//	}
//	
//	if (!fileExists(getPDB8a(weighted)))
//	{
//		g.Reset();
//		tiles.resize(0);
//		for (int x = 0; x < 8; x++)
//			tiles.push_back(x);
//		
//		mnp.Build_PDB(g, tiles, getPDB8a(weighted),
//					  std::thread::hardware_concurrency(), false);
//		mnp.ClearPDBs();
//	}
//	
//	for (int x = 2; x <= 10; x++)
//	{
//		g.Reset();
//		printf("==>Compressing (div delta) by factor of %d\n", x);
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

void BitDeltaValueCompressionTest(bool weighted)
{
//	std::vector<int> tiles;
//	
//	MNPuzzle<4, 4> mnp;
//	mnp.SetWeighted(weighted);
//	MNPuzzleState s(4, 4);
//	MNPuzzleState g(4, 4);
//	
//	mnp.StoreGoal(g);
//	mnp.ClearPDBs();
//	
//	BuildPDBs(true, false, weighted);
//	
//	for (int x = 1; x <= 4; x*=2)
//	{
//		g.Reset();
//		printf("==>Compressing (value-range-delta) to %d bits\n", x);
//		mnp.ClearPDBs();
//		mnp.Load_Regular_PDB(getPDB7a(weighted), g, false);
//		mnp.lookups.push_back({kLeafNode, -0, -0, 0});
//		
//		mnp.Load_Regular_PDB(getPDB8a(weighted), g, false);
//		mnp.Delta_Compress_PDB(g, 1, true);
//		std::vector<int> cutoffs;
//		GetBitValueCutoffs(cutoffs, x);
//		mnp.Value_Compress_PDB(1, cutoffs, true);
//		//MeasureIR(mnp);
//	}
}

#pragma mark node expansion tests

void BaseHeuristicTest(bool weighted)
{
	assert("!Code currently not using refactored PDB setup; needs to be re-written");
//	std::vector<int> tiles;
//	
//	MNPuzzle<4, 4> mnp;
//	mnp.SetWeighted(weighted);
//	MNPuzzleState<4, 4> s;
//	MNPuzzleState<4, 4> g;
//	
//	mnp.StoreGoal(g);
//	
//	mnp.ClearPDBs();
//	BuildPDBs(true, true, weighted);
//
//	if (0)
//	{
//		printf("==>Solving with MD\n");
//		mnp.ClearPDBs();
//		mnp.lookups.push_back({PermutationPuzzle::kLeafDefaultHeuristic, -0, -0, -0});
//		std::string desc = "MD-";
//		Test(mnp, desc.c_str());
//	}
//
//	{
//		printf("==>Solving with 8-piece additive PDBs\n");
//		mnp.ClearPDBs();
//		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true);
//		mnp.Load_Regular_PDB(getPDB8b(weighted), g, true);
//		mnp.Load_Regular_PDB(getPDB1b(weighted), g, true);
//		mnp.lookups.push_back({PermutationPuzzle::kAddNode, 3, 1, -0}); // max of 2 children starting at 1 in the tree
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 0});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 1});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 2});
//		std::string desc = "BASE8-";
//		Test(mnp, desc.c_str());
//	}
//	
//	if (0)
//	{
//		printf("==>Solving with 8+9-piece additive PDBs\n");
//		mnp.ClearPDBs();
//		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true);
//		mnp.Load_Regular_PDB(getPDB9b(weighted), g, true);
//		mnp.lookups.push_back({PermutationPuzzle::kAddNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 0});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 1});
//		std::string desc = "BASE9-";
//		Test(mnp, desc.c_str());
//	}
}

//Fractional_Mod_Compress_PDB
void FractionalNodesCompressionTest(bool weighted)
{
	assert("!Code currently not using refactored PDB setup; needs to be re-written");

//	std::vector<int> tiles;
//	
//	MNPuzzle<4, 4> mnp;
//	mnp.SetWeighted(weighted);
//	MNPuzzleState<4, 4> s;
//	MNPuzzleState<4, 4> g;
//	
//	mnp.StoreGoal(g);
//	mnp.ClearPDBs();
//	
//	BuildPDBs(true, true, weighted);
//	
//	for (int x = 2; x <= 10; x++)
//	{
//		g.Reset();
//		printf("==>Fractional (contiguous): Reducing by factor of %d\n", x);
//		mnp.ClearPDBs();
//		uint64_t oldSize = mnp.Get_PDB_Size(g, 9);
//		uint64_t newSize = oldSize / x;
//		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true); // pdb 0
//		mnp.Load_Regular_PDB(getPDB8b(weighted), g, true); // pdb 1
//		mnp.Load_Regular_PDB(getPDB1b(weighted), g, true); // pdb 2
//		mnp.Load_Regular_PDB(getPDB9b(weighted), g, true); // pdb 3
//		mnp.Fractional_Compress_PDB(3, newSize, true);
//		mnp.lookups.push_back({PermutationPuzzle::kAddNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
//		mnp.lookups.push_back({PermutationPuzzle::kMaxNode, 2, 3, -0}); // max of 2 children starting at 3 in the tree
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 0});
//		mnp.lookups.push_back({PermutationPuzzle::kAddNode, 2, 5, -0});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafFractionalCompress, -0, -0, 3});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 1});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 2});
//		std::string desc = "FCT-C-";
//		desc += ('0'+x/10);
//		desc += ('0'+x%10);
//		Test(mnp, desc.c_str());
//	}
//
////	for (int x = 2; x <= 10; x++)
////	{
////		g.Reset();
////		printf("==>Fractional (contiguous over MD): Reducing by factor of %d\n", x);
////		mnp.ClearPDBs();
////		uint64_t oldSize = mnp.Get_PDB_Size(g, 9);
////		uint64_t newSize = oldSize / x;
////		mnp.lookups.push_back({kLeafDefaultHeuristic, -0, -0, -0});
////
////		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true); // pdb 0
////		mnp.Load_Regular_PDB(getPDB9b(weighted), g, true); // pdb 1
////		mnp.Delta_Compress_PDB(g, 0, true);
////		mnp.Delta_Compress_PDB(g, 1, true);
////		mnp.Fractional_Compress_PDB(1, newSize, true);
////		
////		mnp.lookups.resize(0);
////		mnp.lookups.push_back({kAddNode, 3, 1, -0}); // max of 3 children starting at 1 in the tree
////		mnp.lookups.push_back({kLeafDefaultHeuristic, -0, -0, -0});
////		mnp.lookups.push_back({kLeafNode, -0, -0, 0});
////		mnp.lookups.push_back({kLeafFractionalCompress, -0, -0, 1});
////		std::string desc = "FCT-MD-C-";
////		desc += ('0'+x/10);
////		desc += ('0'+x%10);
////		Test(mnp, desc.c_str());
////	}
}

void FractionalModNodesCompressionTest(bool weighted)
{
	assert("!Code currently not using refactored PDB setup; needs to be re-written");
//	std::vector<int> tiles;
//	
//	MNPuzzle<4, 4> mnp;
//	mnp.SetWeighted(weighted);
//	MNPuzzleState<4, 4> s;
//	MNPuzzleState<4, 4> g;
//	
//	mnp.StoreGoal(g);
//	mnp.ClearPDBs();
//	
//	BuildPDBs(true, true, weighted);
//	
//	for (int x = 2; x <= 10; x++)
//	{
//		g.Reset();
//		printf("==>Fractional MOD: Reducing by factor of %d\n", x);
//		mnp.ClearPDBs();
//		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true); // pdb 0
//		mnp.Load_Regular_PDB(getPDB8b(weighted), g, true); // pdb 1
//		mnp.Load_Regular_PDB(getPDB1b(weighted), g, true); // pdb 2
//		mnp.Load_Regular_PDB(getPDB9b(weighted), g, true); // pdb 3
//		mnp.Fractional_Mod_Compress_PDB(3, x, true);
//		mnp.lookups.push_back({PermutationPuzzle::kAddNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
//		mnp.lookups.push_back({PermutationPuzzle::kMaxNode, 2, 3, -0}); // max of 2 children starting at 3 in the tree
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 0});
//		mnp.lookups.push_back({PermutationPuzzle::kAddNode, 2, 5, -0});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafFractionalModCompress, static_cast<uint8_t>(x), -0, 3});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 1});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 2});
//		
//		std::string desc = "FCT-M-";
//		desc += ('0'+x/10);
//		desc += ('0'+x%10);
//		Test(mnp, desc.c_str());
//	}
//
//
////  over MD ineffective
////	for (int x = 2; x <= 10; x++)
////	{
////		g.Reset();
////		printf("==>Fractional MOD over MD: Reducing by factor of %d\n", x);
////		mnp.ClearPDBs();
////		mnp.lookups.push_back({kLeafDefaultHeuristic, -0, -0, -0});
////		
////		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true); // pdb 0
////		mnp.Load_Regular_PDB(getPDB9b(weighted), g, true); // pdb 1
////		mnp.Delta_Compress_PDB(g, 0, true);
////		mnp.Delta_Compress_PDB(g, 1, true);
////		mnp.Fractional_Mod_Compress_PDB(1, x, true);
////		
////		mnp.lookups.resize(0);
////		mnp.lookups.push_back({kAddNode, 2, 1, -0});
////		mnp.lookups.push_back({kLeafDefaultHeuristic, -0, -0, -0});
////		mnp.lookups.push_back({kLeafNode, -0, -0, 0});
////		mnp.lookups.push_back({kLeafFractionalModCompress, static_cast<uint8_t>(x), -0, 1});
////		
////		std::string desc = "FCT-MD-M-";
////		desc += ('0'+x/10);
////		desc += ('0'+x%10);
////		Test(mnp, desc.c_str());
////	}
//
}


void ModNodesCompressionTest(bool weighted)
{
	assert("!Code currently not using refactored PDB setup; needs to be re-written");
//	std::vector<int> tiles;
//	
//	MNPuzzle<4, 4> mnp;
//	mnp.SetWeighted(weighted);
//	MNPuzzleState<4, 4> s;
//	MNPuzzleState<4, 4> g;
//	
//	mnp.StoreGoal(g);
//	mnp.ClearPDBs();
//	
//	BuildPDBs(true, true, weighted);
//	
//	for (int x = 2; x <= 10; x++)
//	{
//		g.Reset();
//		printf("==>Compressing (mod) by factor of %d\n", x);
//		mnp.ClearPDBs();
//		uint64_t oldSize = mnp.Get_PDB_Size(g, 8);
//		uint64_t newSize = oldSize / x;
//		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true);
//		mnp.Load_Regular_PDB(getPDB9b(weighted), g, true);
//		mnp.Mod_Compress_PDB(1, newSize, true);
//		mnp.lookups.push_back({PermutationPuzzle::kAddNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 0});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafModCompress, -0, -0, 1});
//
//		std::string desc = "MOD-";
//		desc += ('0'+x/10);
//		desc += ('0'+x%10);
//		Test(mnp, desc.c_str());
//	}
}

void ModNodesDeltaCompressionTest(bool weighted)
{
	assert("!Code currently not using refactored PDB setup; needs to be re-written");
//	std::vector<int> tiles;
//	
//	MNPuzzle<4, 4> mnp;
//	mnp.SetWeighted(weighted);
//	MNPuzzleState<4, 4> s;
//	MNPuzzleState<4, 4> g;
//	
//	mnp.StoreGoal(g);
//	mnp.ClearPDBs();
//	
//	BuildPDBs(true, true, weighted);
//	
//	for (int x = 2; x <= 10; x++)
//	{
//		g.Reset();
//		printf("==>Compressing (mod+delta) by factor of %d\n", x);
//		mnp.ClearPDBs();
//		uint64_t oldSize = mnp.Get_PDB_Size(g, 8);
//		uint64_t newSize = oldSize / x;
//		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true);
//		mnp.Load_Regular_PDB(getPDB9b(weighted), g, true);
//		mnp.lookups.push_back({PermutationPuzzle::kLeafDefaultHeuristic, -0, -0, -0});
//		mnp.Delta_Compress_PDB(g, 0, true);
//		mnp.Delta_Compress_PDB(g, 1, true);
//		mnp.Mod_Compress_PDB(1, newSize, true);
//		mnp.lookups.clear();
//		mnp.lookups.push_back({PermutationPuzzle::kAddNode, 3, 1, -0}); // max of 2 children starting at 1 in the tree
//		mnp.lookups.push_back({PermutationPuzzle::kLeafDefaultHeuristic, -0, -0, -0});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 0});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafModCompress, -0, -0, 1});
//		
//		std::string desc = "MOD-D-";
//		desc += ('0'+x/10);
//		desc += ('0'+x%10);
//		Test(mnp, desc.c_str());
//	}
}

void DivNodesCompressionTest(bool weighted)
{
	assert("!Code currently not using refactored PDB setup; needs to be re-written");
//	std::vector<int> tiles;
//	
//	MNPuzzle<4, 4> mnp;
//	mnp.SetWeighted(weighted);
//	MNPuzzleState<4, 4> s;
//	MNPuzzleState<4, 4> g;
//	
//	mnp.StoreGoal(g);
//	mnp.ClearPDBs();
//	
//	BuildPDBs(true, true, weighted);
//	
//	for (int x = 1; x <= 10; x++)
//	{
//		g.Reset();
//		printf("==>Compressing (div) by factor of %d\n", x);
//		mnp.ClearPDBs();
//		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true);
//		mnp.Load_Regular_PDB(getPDB9b(weighted), g, true);
//		mnp.Min_Compress_PDB(1, x, true);
//
//		mnp.lookups.push_back({PermutationPuzzle::kAddNode, 2, 1, -0}); // max of 2 children starting at 1 in the tree
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 0});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafMinCompress, static_cast<uint8_t>(x), -0, 1});
//
//		std::string desc = "DIV-";
//		desc += ('0'+x/10);
//		desc += ('0'+x%10);
//		Test(mnp, desc.c_str());
//		//MeasureIR(mnp);
//	}
}

void DivNodesDeltaCompressionTest(bool weighted)
{
	assert("!Code currently not using refactored PDB setup; needs to be re-written");
//	std::vector<int> tiles;
//	
//	MNPuzzle<4, 4> mnp;
//	mnp.SetWeighted(weighted);
//	MNPuzzleState<4, 4> s;
//	MNPuzzleState<4, 4> g;
//	
//	mnp.StoreGoal(g);
//	mnp.ClearPDBs();
//	
//	BuildPDBs(true, true, weighted);
//
//	if (0)
//	{
//		for (int x = 2; x <= 10; x++)
//		{
//			g.Reset();
//			printf("==>Compressing (div+delta) by factor of %d\n", x);
//			mnp.ClearPDBs();
//			mnp.Load_Regular_PDB(getPDB8a(weighted), g, true);
//			mnp.Load_Regular_PDB(getPDB9b(weighted), g, true);
//			mnp.lookups.push_back({PermutationPuzzle::kLeafDefaultHeuristic, -0, -0, -0});
//			mnp.Delta_Compress_PDB(g, 0, true);
//			mnp.Delta_Compress_PDB(g, 1, true);
//			mnp.Min_Compress_PDB(1, x, true);
//			mnp.lookups.clear();
//			mnp.lookups.push_back({PermutationPuzzle::kAddNode, 3, 1, -0}); // max of 2 children starting at 1 in the tree
//			mnp.lookups.push_back({PermutationPuzzle::kLeafDefaultHeuristic, -0, -0, -0});
//			mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 0});
//			mnp.lookups.push_back({PermutationPuzzle::kLeafMinCompress, static_cast<uint8_t>(x), -0, 1});
//			
//			std::string desc = "MIN-D-";
//			desc += ('0'+x/10);
//			desc += ('0'+x%10);
//			Test(mnp, desc.c_str());
//		}
//	}
//	
//	for (int x = 2; x <= 10; x++)
//	{
//		g.Reset();
//		printf("==>Compressing (div+delta) by factor of %d\n", x);
//		mnp.ClearPDBs();
//		mnp.Load_Regular_PDB(getPDB8a(weighted), g, true);
//		mnp.Load_Regular_PDB(getPDB8b(weighted), g, true);
//		mnp.Load_Regular_PDB(getPDB9b(weighted), g, true);
//		mnp.lookups.push_back({PermutationPuzzle::kLeafDefaultHeuristic, -0, -0, -0});
//		mnp.Delta_Compress_PDB(g, 0, true);
//		mnp.Delta_Compress_PDB(g, 1, true);
//		mnp.Delta_Compress_PDB(g, 2, true);
//		mnp.lookups.clear();
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 1});
//		mnp.Delta_Compress_PDB(g, 2, true);
//		mnp.Min_Compress_PDB(2, x, true);
//
//		mnp.lookups.clear();
//		mnp.lookups.push_back({PermutationPuzzle::kAddNode, 4, 1, -0}); // max of 2 children starting at 1 in the tree
//		mnp.lookups.push_back({PermutationPuzzle::kLeafDefaultHeuristic, -0, -0, -0});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 0});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 1});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafMinCompress, static_cast<uint8_t>(x), -0, 2});
//		
//		std::string desc = "MIN-D8-";
//		desc += ('0'+x/10);
//		desc += ('0'+x%10);
//		Test(mnp, desc.c_str());
//	}
}


void BitDeltaNodesCompressionTest(bool weighted)
{
	assert("!Code currently not using refactored PDB setup; needs to be re-written");
//	std::vector<int> tiles;
//	
//	MNPuzzle<4, 4> mnp;
//	mnp.SetWeighted(weighted);
//	MNPuzzleState<4, 4> s;
//	MNPuzzleState<4, 4> g;
//	
//	mnp.StoreGoal(g);
//	mnp.ClearPDBs();
//	
//	BuildPDBs(true, true, weighted);
//	
//	if (0)
//	{
//		for (int x = 1; x <= 4; x*=2)
//		{
//			std::vector<int> cutoffs;
//			GetBitValueCutoffs(cutoffs, x);
//			
//			g.Reset();
//			printf("==>Compressing to %d bits\n", x);
//			mnp.ClearPDBs();
//			mnp.lookups.push_back({PermutationPuzzle::kLeafDefaultHeuristic, -0, -0, -0});
//			
//			mnp.Load_Regular_PDB(getPDB8a(weighted), g, false); // PDB 0
//			mnp.Delta_Compress_PDB(g, 0, true);
//			
//			mnp.Load_Regular_PDB(getPDB9b(weighted), g, false); // PDB 1
//			mnp.Delta_Compress_PDB(g, 1, true);
//			mnp.Value_Range_Compress_PDB(1, x, true);
//			
//			
//			mnp.lookups.resize(0);
//			mnp.lookups.push_back({PermutationPuzzle::kAddNode, 3, 1, -0});
//			mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 0});
//			mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 1});
//			mnp.lookups.push_back({PermutationPuzzle::kLeafDefaultHeuristic, -0, -0, -0});
//			
//			std::string desc = "VALRNG-DMD-";
//			desc += ('0'+x/10);
//			desc += ('0'+x%10);
//			Test(mnp, desc.c_str());
//		}
//	}
//
//	for (int x = 1; x <= 2; x*=2)
//	{
//		std::vector<int> cutoffs;
//		GetBitValueCutoffs(cutoffs, x);
//		
//		g.Reset();
//		printf("==>Compressing to %d bits\n", x);
//		mnp.ClearPDBs();
//		mnp.lookups.push_back({PermutationPuzzle::kLeafDefaultHeuristic, -0, -0, -0});
//		
//		mnp.Load_Regular_PDB(getPDB8a(weighted), g, false); // PDB 0
//		mnp.Delta_Compress_PDB(g, 0, true);
//
//		mnp.Load_Regular_PDB(getPDB8b(weighted), g, false); // PDB 1
//		mnp.Delta_Compress_PDB(g, 1, true);
//		
//		mnp.Load_Regular_PDB(getPDB9b(weighted), g, false); // PDB 2
//		mnp.Delta_Compress_PDB(g, 2, true);
//
//		mnp.lookups.clear();
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 1});
//		mnp.Delta_Compress_PDB(g, 2, true);
//
//		mnp.Value_Range_Compress_PDB(2, x, true);
//		
//		
//		mnp.lookups.resize(0);
//		mnp.lookups.push_back({PermutationPuzzle::kAddNode, 4, 1, -0});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 0});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 1});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafNode, -0, -0, 2});
//		mnp.lookups.push_back({PermutationPuzzle::kLeafDefaultHeuristic, -0, -0, -0});
//		
//		std::string desc = "VALRNG-D8-";
//		desc += ('0'+x/10);
//		desc += ('0'+x%10);
//		Test(mnp, desc.c_str());
//	}

}

#pragma mark other utilities

void GetBitValueCutoffs(std::vector<int> &cutoffs, int bits)
{
	MNPuzzle<4, 4> mnp;
	MNPuzzleState<4, 4> g;
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

void MeasureIR(MNPuzzle<4, 4> &mnp)
{
	srandom(1234);
	MNPuzzleState<4, 4> s;
	MNPuzzleState<4, 4> g;
	g.Reset();
	mnp.StoreGoal(g);
	//mnp.SetPruneSuccessors(true);
	
	IDAStar<MNPuzzleState<4, 4>, slideDir> ida;
	std::vector<slideDir> path1;
	MNPuzzleState<4, 4> start;
	Timer t;
	t.StartTimer();

	//uint64_t count = mnp.Get_PDB_Size(s, 16);
	assert("!Code currently not using refactored PDB setup; needs to be re-written");
	uint64_t count = 0;
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
		sumh += fabs(mnp.PermutationPuzzleEnvironment<MNPuzzleState<4, 4>,slideDir>::HCost(s)-
					 mnp.PermutationPuzzleEnvironment<MNPuzzleState<4, 4>,slideDir>::HCost(g));
//		printf("G: %1.0f ∆H: %1.0f\n", mnp.GCost(s, g),
//			   fabs(mnp.PermutationPuzzleEnvironment<MNPuzzleState,slideDir>::HCost(s)-
//					mnp.PermutationPuzzleEnvironment<MNPuzzleState,slideDir>::HCost(g)));
	}
	printf("Average G: %1.1f, average H: %1.3f\n", sumg/total, sumh/total);
}


void BaselineTest()
{
	MNPuzzle<4, 4> mnp;
	MNPuzzleState<4, 4> s;
	MNPuzzleState<4, 4> g;
	g.Reset();
	mnp.StoreGoal(g);
	
	{
		ParallelIDAStar<MNPuzzle<4, 4>, MNPuzzleState<4, 4>, slideDir> ida;
		std::vector<slideDir> path1;
		MNPuzzleState<4, 4> start;
		Timer t1;
		t1.StartTimer();
		uint64_t nodesExpanded = 0;
		uint64_t nodesGenerated = 0;
		double totaltime = 0;
		
		g.Reset();
		mnp.StoreGoal(g);
		for (int x = 0; x < 100; x++)
		{
			s = STP::GetKorfInstance(x);
			g.Reset();
			printf("Problem %d of %d\n", x+1, 100);
			std::cout << "Searching from: " << std::endl << s << std::endl << g << std::endl;
			Timer t;
			t.StartTimer();
			ida.GetPath(&mnp, s, g, path1);
			t.EndTimer();
			totaltime += t.GetElapsedTime();
			std::cout << "Path found, length " << path1.size() << " time:" << t.GetElapsedTime() << std::endl;
			nodesExpanded += ida.GetNodesExpanded();
			nodesGenerated += ida.GetNodesTouched();
		}
		printf("Parallel: %1.2fs elapsed; %llu nodes expanded; %llu nodes generated\n", t1.EndTimer(), nodesExpanded, nodesGenerated);
	}
	{
		IDAStar<MNPuzzleState<4, 4>, slideDir> ida;
		ida.SetUseBDPathMax(true);

		std::vector<slideDir> path1;
		MNPuzzleState<4, 4> start;
		Timer t1;
		t1.StartTimer();
		uint64_t nodesExpanded = 0;
		uint64_t nodesGenerated = 0;
		double totaltime = 0;
		
		g.Reset();
		mnp.StoreGoal(g);
		for (int x = 0; x < 100; x++)
		{
			s = STP::GetKorfInstance(x);
			g.Reset();
			printf("Problem %d of %d\n", x+1, 100);
			std::cout << "Searching from: " << std::endl << s << std::endl << g << std::endl;
			Timer t;
			t.StartTimer();
			ida.GetPath(&mnp, s, g, path1);
			t.EndTimer();
			totaltime += t.GetElapsedTime();
			std::cout << "Path found, length " << path1.size() << " time:" << t.GetElapsedTime() << std::endl;
			nodesExpanded += ida.GetNodesExpanded();
			nodesGenerated += ida.GetNodesTouched();
		}
		printf("Sequential: %1.2fs elapsed; %llu nodes expanded; %llu nodes generated\n", t1.EndTimer(), nodesExpanded, nodesGenerated);
	}
	
}

void Test(MNPuzzle<4, 4> &mnp, const char *prefix)
{
	MNPuzzleState<4, 4> s;
	MNPuzzleState<4, 4> g;
	g.Reset();
	mnp.StoreGoal(g);
	
	IDAStar<MNPuzzleState<4, 4>, slideDir> ida;
	ida.SetUseBDPathMax(true);
	std::vector<slideDir> path1;
	MNPuzzleState<4, 4> start;
	Timer t1;
	t1.StartTimer();
	uint64_t nodes = 0;
	double totaltime = 0;
	for (int x = 0; x < 100; x++)
	{
		s = STP::GetKorfInstance(x);
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

