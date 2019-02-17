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
#include "BID.h"
#include "TemplateAStar.h"
#include "PancakePuzzle.h"
#include "PancakeInstances.h"
#include "TOH.h"

MNPuzzle<4, 4> p;
MNPuzzleState<4, 4> s, t;
std::vector<slideDir> moves, tmpPath;
double v = 1;

bool recording = false;
void TestSTP(int instance, int algorithm, double minGrowth, double maxGrowth, double startEpsilon);
void TestPancake(int instance, int algorithm, double minGrowth, double maxGrowth, double startEpsilon);
void TestTOH(int instance, int algorithm, double minGrowth, double maxGrowth, double startEpsilon);

void ValidateWeights();

int main(int argc, char* argv[])
{
	ValidateWeights();
	
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

//	InstallKeyboardHandler(WeightedTest, "Weighted STP Test", "Test the STP with weights", kNoModifier, 'w');
//	InstallKeyboardHandler(STPTest, "STP Test", "Test the STP PDBs", kNoModifier, 'd');
//	InstallKeyboardHandler(BuildSTP_PDB, "Build STP PDBs", "Build PDBs for the STP", kNoModifier, 'a');

	InstallCommandLineHandler(MyCLHandler, "-stp", "-stp <instance> <algorithm>", "Runs STP with ");
	InstallCommandLineHandler(MyCLHandler, "-pancake", "-pancake <instance> <algorithm>", "Runs pancake with ");
	InstallCommandLineHandler(MyCLHandler, "-test", "-test", "Basic test with MD heuristic");
	
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
		p.SetWeighted(kUnitPlusFrac);
		TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> astar;
		IDAStar<MNPuzzleState<4, 4>, slideDir> ida;
		BID<MNPuzzleState<4, 4>, slideDir> ebis(2, 5, 1);
		s = STP::GetKorfInstance(1);
//		srandom(20181222);
//		s = STP::GetRandomInstance(135);
//		s = STP::GetRandomInstance(150);
//		s = STP::GetRandomInstance(175);
		Timer ti;
		
		std::vector<MNPuzzleState<4, 4>> path;
		std::cout << "A* searching from\n" << s << "\n" << t << "\n";
		ti.StartTimer();
		astar.GetPath(&p, s, t, path);
		printf("A*: %1.2fs elapsed; %llu expanded; solution length %f\n", ti.EndTimer(),
			   astar.GetNodesExpanded(), p.GetPathLength(path));
		
		std::cout << "EBIS searching from\n" << s << "\n" << t << "\n";
		ti.StartTimer();
		ebis.GetPath(&p, s, t, tmpPath);
		printf("EBIS: %1.2fs elapsed; solution length %f\n", ti.EndTimer(), p.GetPathLength(s, tmpPath));

		std::cout << s << std::endl;
		for (int x = 0; x < tmpPath.size(); x++)
		{
			std::cout << tmpPath[x] << " ";
		}
		std::cout << std::endl;

		std::cout << "IDA* searching from\n" << s << "\n" << t << "\n";
		ti.StartTimer();
		ida.GetPath(&p, s, t, moves);
		printf("IDA*: %1.2fs elapsed; solution length %f\n", ti.EndTimer(), p.GetPathLength(s, moves));

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
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	//p.OpenGLDraw(s);

	v += 0.1;
	if (v > 1 && moves.size() > 0)
	{
		t = s;
		p.ApplyAction(s, moves[0]);
		v = 0;
		moves.erase(moves.begin());
	}
	if (v > 1 && moves.size() == 0)
	{
		v = 1;
		recording = false;
	}
	p.OpenGLDraw(s, t, v);

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
	if (strcmp(argument[0], "-stp") == 0)
	{
		double c1 = 2, c2 = 5, ep = -1;
		if (maxNumArgs < 3)
		{
			printf("Error: didn't pass arguments: <instance> <algorithm>\n");
			exit(0);
		}
		if (maxNumArgs >= 4)
			c1 = atof(argument[3]);
		if (maxNumArgs >= 5)
			c2 = atof(argument[4]);
		if (maxNumArgs >= 6)
			ep = atof(argument[5]);

		TestSTP(atoi(argument[1]), atoi(argument[2]), c1, c2, ep);
	}
	if (strcmp(argument[0], "-pancake") == 0)
	{
		double c1 = 2, c2 = 5, ep = -1;
		if (maxNumArgs < 3)
		{
			printf("Error: didn't pass arguments: <instance> <algorithm>\n");
			exit(0);
		}
		if (maxNumArgs >= 4)
			c1 = atof(argument[3]);
		if (maxNumArgs >= 5)
			c2 = atof(argument[4]);
		if (maxNumArgs >= 6)
			ep = atof(argument[5]);
		
		TestPancake(atoi(argument[1]), atoi(argument[2]), c1, c2, ep);
	}
	if (strcmp(argument[0], "-test") == 0)
	{
		exit(0);
	}
	exit(0);
	return 2;
}

//void TestSTP(int instance, int algorithm, double minGrowth, double maxGrowth, double startEpsilon)
//{
//	MNPuzzle<4, 4> stp;
//	MNPuzzleState<4, 4> start, goal, testStart;
//	std::vector<slideDir> moves, tmpPath;
//
//	stp.SetWeighted(kUnitPlusFrac);
//	TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> astar;
//	IDAStar<MNPuzzleState<4, 4>, slideDir> ida;
//	BID<MNPuzzleState<4, 4>, slideDir> ebis(minGrowth, maxGrowth, startEpsilon);
//	std::vector<MNPuzzleState<4, 4>> path;
//
//	assert(instance >= 0 && instance < 100);
//	start = STP::GetKorfInstance(instance);
//
//	Timer t;
//	switch (algorithm)
//	{
//		case 0: // IDA*
//		{
//			std::cout << "IDA* searching from\n" << start << "\n" << goal << "\n";
//			t.StartTimer();
//			ida.GetPath(&stp, start, goal, moves);
//			t.EndTimer();
//			printf("IDA*: %1.2fs elapsed; %llu expanded; %llu generated; solution length %f\n", t.GetElapsedTime(), ida.GetNodesExpanded(), ida.GetNodesTouched(), stp.GetPathLength(start, moves));
//		}
//			break;
//		case 1: // EB Search
//		{
//			printf("EB(%1.2f, %1.2f, %1.2f): ", minGrowth, maxGrowth, startEpsilon);
//			std::cout << " search from\n" << start << "\n" << goal << "\n";
//			t.StartTimer();
//
//			ebis.GetPath(&stp, start, goal, tmpPath);
//			t.EndTimer();
//			printf("EB Search: %1.2fs elapsed; %llu expanded; %llu generated; solution length %f\n", t.GetElapsedTime(),
//				   ebis.GetNodesExpanded(), ebis.GetNodesTouched(), stp.GetPathLength(start, tmpPath));
//
//			t.StartTimer();
//			ebis.RedoMinWork();
//			t.EndTimer();
//			printf("DBDFS: %1.2fs elapsed; %llu expanded; %llu generated; solution length %f\n", t.GetElapsedTime(),
//				   ebis.GetNodesExpanded(), ebis.GetNodesTouched(), stp.GetPathLength(start, tmpPath));
//		}
//			break;
//		case 2: // A*
//		{
//			std::cout << "A* searching from\n" << start << "\n" << goal << "\n";
//			t.StartTimer();
//			astar.GetPath(&stp, start, goal, path);
//			t.EndTimer();
//			printf("A*: %1.2fs elapsed; %llu expanded; %llu generated; solution length %f\n", t.GetElapsedTime(),
//				   astar.GetNodesExpanded(), astar.GetNodesTouched(), stp.GetPathLength(path));
//		}
//			break;
//	}
//	exit(0);
//}

template <class state, class action, class environment>
void Test(environment *e, Heuristic<state> *h, const state &start, const state &goal, int algorithm, double minGrowth, double maxGrowth, double startEpsilon)
{
	std::vector<action> moves, tmpPath;
	
	TemplateAStar<state, action, environment> astar;
	IDAStar<state, action> ida;
	BID<state, action> ebis(minGrowth, maxGrowth, startEpsilon);
	std::vector<state> path;
	
	Timer t;
	switch (algorithm)
	{
		case 0: // IDA*
		{
			std::cout << "IDA* searching from\n" << start << "\n" << goal << "\n";
			ida.SetHeuristic(h);
			t.StartTimer();
			ida.GetPath(e, start, goal, moves);
			t.EndTimer();
			printf("IDA*: %1.2fs elapsed; %llu expanded; %llu generated; solution length %f\n", t.GetElapsedTime(), ida.GetNodesExpanded(), ida.GetNodesTouched(), e->GetPathLength(start, moves));
		}
			break;
		case 1: // EB Search
		{
			printf("EB(%1.2f, %1.2f, %1.2f): ", minGrowth, maxGrowth, startEpsilon);
			std::cout << " search from\n" << start << "\n" << goal << "\n";
			t.StartTimer();
			
			ebis.GetPath(e, h, start, goal, tmpPath);
			t.EndTimer();
			printf("EB Search: %1.2fs elapsed; %llu expanded; %llu generated; solution length %f\n", t.GetElapsedTime(),
				   ebis.GetNodesExpanded(), ebis.GetNodesTouched(), e->GetPathLength(start, tmpPath));
			
			t.StartTimer();
			ebis.RedoMinWork();
			t.EndTimer();
			printf("DBDFS: %1.2fs elapsed; %llu expanded; %llu generated; solution length %f\n", t.GetElapsedTime(),
				   ebis.GetNodesExpanded(), ebis.GetNodesTouched(), e->GetPathLength(start, tmpPath));
		}
			break;
		case 2: // A*
		{
			std::cout << "A* searching from\n" << start << "\n" << goal << "\n";
			astar.SetHeuristic(h);
			t.StartTimer();
			astar.GetPath(e, start, goal, path);
			t.EndTimer();
			printf("A*: %1.2fs elapsed; %llu expanded; %llu generated; solution length %f\n", t.GetElapsedTime(),
				   astar.GetNodesExpanded(), astar.GetNodesTouched(), e->GetPathLength(path));
		}
			break;
	}
	exit(0);
}

void TestSTP(int instance, int algorithm, double minGrowth, double maxGrowth, double startEpsilon)
{
	MNPuzzle<4, 4> stp;
	MNPuzzleState<4, 4> start, goal;
	
	stp.SetWeighted(kUnitPlusFrac);
	
	assert(instance >= 0 && instance < 100);
	start = STP::GetKorfInstance(instance);
	Test<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(&stp, &stp, start, goal, algorithm, minGrowth, maxGrowth, startEpsilon);
}

void TestPancake(int instance, int algorithm, double minGrowth, double maxGrowth, double startEpsilon)
{
	PancakePuzzle<20> pancake;
	PancakePuzzleState<20> start, goal;
	pancake.SetUseRealValueEdges(true);
	assert(instance >= 0 && instance < 100);
	GetPancakeInstance(start, instance);
	Test<PancakePuzzleState<20>, PancakePuzzleAction, PancakePuzzle<20>>(&pancake, &pancake, start, goal, algorithm, minGrowth, maxGrowth, startEpsilon);
}

//template <int numDisks, int pdb1Disks, int pdb2Disks = numDisks-pdb1Disks>
//Heuristic<TOHState<numDisks>> *BuildPDB(const TOHState<numDisks> &goal)
//{
//	TOH<numDisks> toh;
//	TOH<pdb1Disks> absToh1;
//	TOH<pdb2Disks> absToh2;
//	TOHState<pdb1Disks> absTohState1;
//	TOHState<pdb2Disks> absTohState2;
//	
//	
//	TOHPDB<pdb1Disks, numDisks, pdb2Disks> *pdb1 = new TOHPDB<pdb1Disks, numDisks, pdb2Disks>(&absToh1, goal); // top disks
//	TOHPDB<pdb2Disks, numDisks> *pdb2 = new TOHPDB<pdb2Disks, numDisks>(&absToh2, goal); // bottom disks
//	pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
//	pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
//	
//	Heuristic<TOHState<numDisks>> *h = new Heuristic<TOHState<numDisks>>;
//	
//	h->lookups.resize(0);
//	
//	h->lookups.push_back({kAddNode, 1, 2});
//	h->lookups.push_back({kLeafNode, 0, 0});
//	h->lookups.push_back({kLeafNode, 1, 1});
//	h->heuristics.resize(0);
//	h->heuristics.push_back(pdb1);
//	h->heuristics.push_back(pdb2);
//	
//	return h;
//}
//
//void GetTOH(int which, TOHState<20> &s)
//{
//	uint64_t randoms[] = {
//		0xe33041808371a397ull,0x9fa22bce4a50c1a5ull,
//		0x9e12ab456bf88c25ull,0x0c28f2040e0f16c9ull,
//		0x66e1d4b458ac98e1ull,0x2c554233132d01e4ull,
//		0xfa1eacfe4fa5b266ull,0x6479cfd1124fe453ull,
//		0x8728cb4b6c68ce34ull,0xc2d80c239bb5c28full,
//		0x3761c53e74868e3bull,0x8f6106070168f18eull,
//		0xc7bf1f59d04be4a6ull,0x6a875002dee8aebdull,
//		0xc4688d143dba0204ull,0xee28aa2ac92e78d0ull,
//		0xc11cfd8040085b5cull,0xad2766daffe16340ull,
//		0xb454d3947eb90a4cull,0x04d814945598dfabull,
//		0x874f7c1dcf4d5ccbull,0xe560eac1bf423a77ull,
//		0x8ca4c8896642102full,0x63874d7049d12d9dull,
//		0xe9b61ee51c823912ull,0xe9c0a25b55334311ull,
//		0x3f1376d6e8c82a2bull,0xc8daaf6238abe404ull,
//		0x1b1989268958dfd5ull,0x05283ad756f5fb7cull,
//		0x5a6adb2578f16593ull,0xb8b461fa5c710372ull,
//		0x764f7335218035a2ull,0x2b32cfbac0295d41ull,
//		0xc8757763ab2d85a7ull,0xb9d4fb04fb0094c9ull,
//		0x2db4c442ebea9f54ull,0xfa13eeaeec1d482full,
//		0xf996d7daf2766431ull,0x322f7d1be847a456ull,
//		0xccfad66220bb61bdull,0xfc101563a99c9cfaull,
//		0x6b0ebf7d896af7c4ull,0x0bcf482ce2786877ull,
//		0x56165adf828896fbull,0x6dfd737ef28c3f6eull,
//		0x6a590a174dc946b3ull,0xe430432381d2a521ull,
//		0x6015436d453fd67bull,0x0b544c09ddd42043ull,
//		0x891cca38465132b0ull,0xaecdce2fb98b8ff9ull,
//		0xf745b4decaa8b8d9ull,0xcf3a4362945e277dull,
//		0x55b54c36ed04d78eull,0x243839d1f8740d3aull,
//		0xb9ad8a1995b252c2ull,0x757e5f675f0e6d65ull,
//		0x26f19b21e41c61c8ull,0x5f7be658a180cb50ull,
//		0x2ab21dbc05f42237ull,0xbf7a403457a7c076ull,
//		0xaadc3a95a93adf65ull,0x36fb08506d5c985eull,
//		0xe7278d88ab02814dull,0xe3a880d4aa2bfc70ull,
//		0xaed82d9116bac6b9ull,0xd2e05feaae8093c8ull,
//		0xaeef8bfe2d629499ull,0xfaf7910cad546eb9ull,
//		0x88a51f8549a9f3cbull,0x1735bb86a4c972acull,
//		0xf92342acd65bddd3ull,0x05118cf84a7cdc97ull,
//		0x7332ef706f53f7edull,0x19ace3199e0c7573ull,
//		0x773cb2d931909f22ull,0x7cb680cb8433135bull,
//		0x3047032ed3bbd3a4ull,0xda8aa09c29f0ae38ull,
//		0xe7c8445c77cc29bdull,0x4b5b29041abcd6f4ull,
//		0xa30069eba2db45cfull,0x38cbf400dc424812ull,
//		0x237ea728ad3ad350ull,0x6e98db2ccf92c75bull,
//		0x08cac1762b12d7d2ull,0xc1ea3772670001a3ull,
//		0x4610c06b18ea56beull,0x09503a98cb955119ull,
//		0x123ebb16278fff55ull,0xd36c16cc1e601d91ull,
//		0x01877d5a1741028dull,0xeef1a40d741cd6c2ull,
//		0xa954766cd6164ab1ull,0xf6efbf6bc9a33e51ull,
//		0xffc1668142653e7eull,0x39c64746e7d4421full,
//		0x833f0738df718d68ull,0xfe62fda813d43a01ull };
//	TOH<20> t;
//	assert(which >= 0 && which < 100);
//	t.GetStateFromHash(randoms[which%t.GetMaxHash()], s);
//}
//
//void TestTOH(int instance, int algorithm, double minGrowth, double maxGrowth, double startEpsilon)
//{
//	TOH<20> toh;
//	TOHState<20> start, goal;
//	Heuristic<TOHState<20>> *h = BuildPDB<20, 14>(goal);
//
//	GetTOH(instance, start);
//	Test<TOHState<20>, TOHMove, TOH<20>>(&toh, h, start, goal, algorithm, minGrowth, maxGrowth, startEpsilon);
//}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 't': break;
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
			break;
		case '\t':
			break;
		case 'p':
		{}
			break;
		case 'o':
		{}
			break;
		default:
			break;
	}
}

bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	return false;
}

void ValidateWeights()
{
	MNPuzzle<4, 4> stp;
	MNPuzzleState<4, 4> s1, s2;
	std::vector<slideDir> moves;
	stp.SetWeighted(kUnitPlusFrac);
	for (int x = 0; x < 100000; x++)
	{
		stp.GetActions(s1, moves);
		auto i = moves[random()%moves.size()];
		s2 = s1;
		stp.ApplyAction(s2, i);
		assert(stp.GCost(s1, s2) == stp.GCost(s1, i));
		s1 = s2;
	}
	printf("Weights validated\n");
}
