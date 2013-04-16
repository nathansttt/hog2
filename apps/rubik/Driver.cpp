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
#include "IDAStar.h"
#include "Timer.h"
#include "RubiksCube.h"
#include "DiskBitFile.h"
#include "RubiksCube7Edges.h"
#include "BFS.h"


RubiksCube c;
RubiksAction a;
RubiksState s;

Rubik7Edge e7;
Rubik7EdgeState e7s;
Rubik7EdgeAction e7a;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv);
}


/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
void CreateSimulation(int id)
{
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');

	InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a randomly moving unit", kShiftDown, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, 1);

	InstallCommandLineHandler(MyCLHandler, "-test", "-test entries", "Test using 'entries' billion entries from edge pdb");
	
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
		//CreateSimulation(windowID);
		SetNumPorts(windowID, 2);
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (viewport == 1)
	{
		e7.OpenGLDraw(e7s);
	}
	else {
		c.OpenGLDraw(s);
	}
}

void RunTest(int billionEntriesToLoad);

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	RunTest(atoi(argument[1]));
	exit(0);
	//	strncpy(gDefaultMap, argument[1], 1024);
	return 2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
			//e7.GetStateFromHash(e7.GetStateHash(e7s), e7s);
			//printf("Old hash is %llu. ", c.GetStateHash(s));
			c.ApplyAction(s, key-'0'+10);
			e7.ApplyAction(e7s, key-'0'+10);
			//printf("New is %llu\n", c.GetStateHash(s));
			printf("rank is %llu\n", e7.GetStateHash(e7s));
			break;
		case '9':
		{
			static int a = 0;
			e7.GetStateFromHash(a++, e7s);
//			int a = random()%18;
//			c.ApplyAction(s, a);
//			e7.ApplyAction(e7s, a);
			printf("rank is %llu\n", e7.GetStateHash(e7s));
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
			s.Reset();
			printf("Resetting state\n");
			break;
		case 'o':
		{
		}
			break;
		case ']':
			s.Reset();
			for (int x = 0; x < 5; x++)
				c.ApplyAction(s, x);
			break;
		case '[':
			s.Reset();
			for (int x = 4; x >= 0; x--)
				c.ApplyAction(s, x);
			break;
//		case '{': unitSim->setPaused(true); unitSim->offsetDisplayTime(-0.5); break;
//		case '}': unitSim->offsetDisplayTime(0.5); break;
		default:
			//if (unitSim)
			//	unitSim->GetEnvironment()->GetMapAbstraction()->ToggleDrawAbstraction(((mod == kControlDown)?10:0)+(key-'0'));
			break;
	}
}

void LoadCornerPDB()
{
	if (c.GetCornerPDB().Size() != 0)
		return;
	FourBitArray &b = c.GetCornerPDB();
	
	//uint8_t *mem;
	std::vector<bucketInfo> data;
	std::vector<bucketData> buckets;
	
	uint64_t maxBuckSize = GetMaxBucketSize<RubiksCorner, RubiksCornerState>(true);
	InitTwoPieceData<RubiksCorner, RubiksCornerState>(data, maxBuckSize);
	InitBucketSize<RubiksCorner, RubiksCornerState>(buckets, maxBuckSize);
	int64_t totalSize = 0;
	for (unsigned int x = 0; x < buckets.size(); x++)
		totalSize += buckets[x].theSize;
	b.Resize(totalSize);
	//mem = new uint8_t[totalSize];
	DiskBitFile f("/home/sturtevant/sturtevant/code/cc/rubik/RC");
	int64_t index = 0;
	for (unsigned int x = 0; x < data.size(); x++)
	{
		for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
		{
			int val = f.ReadFileDepth(data[x].bucketID, y);
			//mem[index++] = val;
			b.Set(index++, val);
		}
	}
	//c.SetCornerPDB(mem);
}

void LoadEdgePDB(uint64_t sizeLimit)
{
	//
	if (c.GetEdgePDB().Size() != 0)
		return;
	FourBitArray &b = c.GetEdgePDB();

	//uint8_t *mem;
	std::vector<bucketInfo> data;
	std::vector<bucketData> buckets;
	
	uint64_t maxBuckSize = GetMaxBucketSize<RubikEdge, RubikEdgeState>(true);
	InitTwoPieceData<RubikEdge, RubikEdgeState>(data, maxBuckSize);
	InitBucketSize<RubikEdge, RubikEdgeState>(buckets, maxBuckSize);
	int64_t totalSize = 0;
//	const int64_t sizeLimit = 1000000000;
	for (unsigned int x = 0; x < buckets.size(); x++)
		totalSize += buckets[x].theSize;
	if (totalSize > sizeLimit)
		totalSize = sizeLimit;
	//mem = new uint8_t[totalSize];
	b.Resize(totalSize);
	DiskBitFile f("/data/cc/rubik/res/RC");
	int64_t index = 0;
	for (unsigned int x = 0; x < data.size(); x++)
	{
		for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
		{
			int val = f.ReadFileDepth(data[x].bucketID, y);
			b.Set(index++, val);
			//mem[index++] = val;
			if (index >= totalSize)
				break;
		}
		if (index >= totalSize)
			break;
	}
//	c.SetEdgePDB(mem, totalSize);
}

void LoadEdge7PDB()
{
	if (c.GetEdge7PDB().Size() != 0)
		return;
	FourBitArray &b = c.GetEdge7PDB();
	
	//uint8_t *mem;
	std::vector<bucketInfo> data;
	std::vector<bucketData> buckets;
	
	uint64_t maxBuckSize = GetMaxBucketSize<Rubik7Edge, Rubik7EdgeState>(true);
	InitTwoPieceData<Rubik7Edge, Rubik7EdgeState>(data, maxBuckSize);
	InitBucketSize<Rubik7Edge, Rubik7EdgeState>(buckets, maxBuckSize);
	int64_t totalSize = 0;
	//	const int64_t sizeLimit = 1000000000;
	for (unsigned int x = 0; x < buckets.size(); x++)
	  totalSize += buckets[x].theSize;
	//mem = new uint8_t[totalSize];
	b.Resize(totalSize);
	DiskBitFile f("/home/sturtevant/sturtevant/code/cc/rubik/RC-7edge");//RC-9edge");
	int64_t index = 0;
	for (unsigned int x = 0; x < data.size(); x++)
	{
		for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
		{
			int val = f.ReadFileDepth(data[x].bucketID, y);
			b.Set(index++, val);
			//mem[index++] = val;
			if (index >= totalSize)
				break;
		}
		if (index >= totalSize)
			break;
	}
	//	c.SetEdgePDB(mem, totalSize);
}

void MyRandomUnitKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	std::vector<RubiksAction> acts;
	c.SetPruneSuccessors(true);
	while (true)
	{
		c.GetActions(s, acts);
		for (unsigned int x = 0; x < acts.size(); x++)
			printf("%d) %d\n", x, acts[x]);
		printf("Choose action to apply: ");
		int which;
		std::cin >> which;
		c.ApplyAction(s, acts[which]);
	}
}

void GetInstance(RubiksState &start, int which);
void SolveOneProblem(int instance)
{
	RubiksState start, goal;
	goal.Reset();
	start.Reset();
	c.SetPruneSuccessors(true); // clears history
	GetInstance(start, instance);
	
	std::vector<RubiksAction> acts;
	c.SetPruneSuccessors(true);
	
	s = start;
	IDAStar<RubiksState, RubiksAction> ida;
	c.SetPruneSuccessors(true);
	Timer t;
	t.StartTimer();
	ida.SetUseBDPathMax(true);
	ida.GetPath(&c, start, goal, acts);
	t.EndTimer();
	printf("Problem %d - %llu expanded; %1.2f elapsed\n", instance+1, ida.GetNodesExpanded(), t.GetElapsedTime());
	for (unsigned int x = 0; x < acts.size(); x++)
	{
		printf("%d ", acts[x]);
		c.ApplyAction(s, acts[x]);
	}
	printf("\n");
	fflush(stdout);
}

void MyPathfindingKeyHandler(unsigned long , tKeyboardModifier , char)
{
	LoadCornerPDB();
	LoadEdge7PDB();
	//LoadEdgePDB(2000000000);

	for (int x = 3; x < 4; x++)
	{
		srandom(9283+x*23);
		SolveOneProblem(x);
	}
	
//	LoadCornerPDB();
	//	static int t = 0;
//	c.GetStateFromHash(t, s);
//	t++;
//	return;
}

void RunTest(int billionEntriesToLoad)
{
	uint64_t numEntries = 1000000000;
	numEntries *= billionEntriesToLoad;
	LoadCornerPDB();
	LoadEdgePDB(numEntries);
	LoadEdge7PDB();
	
	for (int x = 0; x < 100; x++)
	{
		srandom(9283+x*23);
		SolveOneProblem(x);
	}
}

bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	return false;
}


void GetInstance(RubiksState &start, int which)
{
	int instances[100][14] = {
	{4, 10, 5, 10, 17, 6, 1, 11, 0, 10, 16, 9, 14, 4},
	{15, 0, 12, 16, 1, 6, 17, 8, 17, 8, 2, 11, 4, 15},
	{6, 12, 2, 16, 9, 13, 8, 2, 15, 11, 4, 8, 4, 16},
	{9, 13, 16, 1, 14, 0, 11, 13, 4, 10, 12, 6, 5, 15},
	{1, 9, 4, 6, 0, 4, 15, 5, 12, 0, 14, 9, 4, 16},
	{5, 8, 4, 8, 5, 9, 5, 11, 0, 7, 11, 12, 8, 17},
	{14, 11, 4, 14, 5, 6, 4, 10, 1, 4, 15, 9, 13, 7},
	{17, 1, 4, 10, 4, 16, 4, 16, 8, 13, 15, 2, 4, 17},
	{7, 12, 11, 4, 15, 1, 17, 2, 5, 14, 9, 5, 8, 10},
	{10, 13, 15, 1, 16, 3, 7, 9, 17, 4, 9, 17, 6, 15},
	{0, 5, 10, 16, 8, 14, 7, 2, 6, 1, 4, 12, 8, 13},
	{4, 6, 5, 17, 7, 4, 17, 9, 17, 3, 9, 3, 17, 3},
	{15, 7, 13, 17, 11, 3, 10, 4, 12, 10, 0, 16, 2, 9},
	{6, 10, 16, 2, 4, 7, 0, 14, 16, 9, 2, 17, 2, 9},
	{9, 16, 0, 5, 16, 7, 1, 6, 0, 3, 10, 2, 12, 11},
	{1, 7, 3, 10, 2, 10, 2, 8, 4, 11, 13, 17, 5, 7},
	{5, 11, 16, 4, 9, 13, 4, 15, 8, 15, 0, 11, 12, 7},
	{14, 17, 11, 4, 8, 3, 10, 4, 10, 0, 5, 7, 13, 3},
	{17, 8, 16, 11, 14, 5, 13, 2, 16, 10, 3, 12, 0, 5},
	{8, 9, 2, 3, 11, 15, 0, 17, 9, 2, 13, 10, 3, 17},
	{10, 17, 2, 17, 10, 2, 11, 16, 8, 16, 10, 5, 16, 1},
	{0, 6, 17, 1, 15, 9, 4, 15, 3, 14, 1, 11, 1, 3},
	{4, 7, 17, 9, 17, 9, 0, 9, 17, 3, 13, 5, 15, 11},
	{13, 15, 1, 5, 16, 6, 15, 6, 1, 10, 2, 14, 3, 7},
	{6, 16, 1, 16, 3, 9, 1, 13, 8, 12, 7, 4, 9, 14},
	{9, 13, 2, 5, 16, 5, 9, 17, 7, 4, 6, 9, 14, 3},
	{1, 13, 11, 5, 15, 3, 7, 12, 5, 7, 12, 5, 14, 0},
	{5, 17, 10, 4, 9, 4, 13, 1, 5, 6, 0, 6, 3, 17},
	{14, 8, 10, 13, 11, 12, 6, 11, 0, 5, 12, 0, 14, 1},
	{17, 10, 0, 11, 16, 9, 5, 6, 17, 8, 3, 11, 13, 5},
	{8, 17, 10, 15, 4, 8, 0, 15, 1, 16, 10, 13, 15, 6},
	{10, 14, 15, 6, 13, 5, 13, 0, 3, 11, 5, 7, 5, 6},
	{0, 12, 11, 17, 9, 1, 11, 16, 7, 13, 5, 14, 17, 5},
	{4, 15, 11, 3, 17, 7, 13, 6, 17, 7, 12, 0, 5, 6},
	{13, 16, 2, 3, 11, 14, 2, 11, 17, 7, 2, 13, 8, 1},
	{6, 12, 3, 14, 8, 5, 12, 5, 17, 4, 14, 1, 12, 17},
	{9, 13, 7, 9, 5, 7, 2, 3, 6, 2, 15, 2, 13, 1},
	{1, 17, 7, 9, 14, 0, 3, 11, 15, 10, 0, 4, 15, 5},
	{3, 9, 2, 17, 5, 10, 17, 10, 1, 12, 9, 3, 8, 4},
	{14, 4, 15, 11, 5, 15, 11, 0, 14, 10, 1, 10, 12, 11},
	{17, 4, 11, 13, 4, 14, 9, 13, 11, 17, 1, 6, 16, 0},
	{8, 5, 8, 15, 6, 4, 15, 11, 14, 7, 4, 7, 4, 9},
	{10, 14, 8, 3, 15, 6, 5, 17, 3, 12, 11, 16, 6, 2},
	{0, 15, 3, 17, 0, 9, 3, 14, 16, 2, 9, 14, 0, 4},
	{4, 16, 5, 7, 2, 9, 4, 12, 6, 17, 2, 13, 7, 14},
	{13, 6, 9, 3, 16, 8, 9, 13, 15, 4, 12, 0, 15, 3},
	{6, 13, 6, 5, 7, 4, 11, 15, 10, 13, 3, 8, 10, 2},
	{9, 16, 0, 6, 1, 9, 12, 7, 9, 5, 16, 5, 15, 4},
	{1, 4, 9, 3, 8, 4, 8, 4, 6, 14, 5, 14, 16, 3},
	{3, 14, 17, 11, 13, 4, 8, 15, 6, 15, 8, 4, 10, 2},
	{14, 17, 0, 13, 9, 3, 17, 10, 3, 8, 3, 17, 3, 16},
	{17, 5, 8, 15, 2, 9, 3, 17, 6, 17, 0, 4, 13, 4},
	{8, 14, 2, 17, 7, 10, 5, 13, 0, 9, 16, 7, 17, 0},
	{11, 15, 2, 11, 17, 11, 4, 17, 1, 11, 12, 3, 14, 1},
	{0, 3, 15, 0, 7, 10, 3, 12, 6, 17, 9, 0, 7, 1},
	{4, 12, 15, 7, 3, 7, 15, 7, 13, 3, 15, 11, 5, 10},
	{13, 1, 6, 11, 16, 3, 14, 5, 16, 1, 9, 0, 16, 0},
	{6, 3, 7, 2, 15, 6, 2, 15, 6, 11, 17, 7, 0, 16},
	{9, 4, 13, 9, 5, 13, 11, 17, 9, 15, 11, 3, 14, 9},
	{1, 14, 9, 14, 4, 9, 1, 16, 10, 16, 11, 13, 17, 1},
	{3, 11, 13, 15, 9, 16, 8, 13, 10, 15, 0, 11, 5, 15},
	{14, 5, 11, 0, 13, 6, 1, 11, 3, 9, 14, 8, 10, 12},
	{17, 2, 9, 14, 11, 12, 10, 4, 9, 14, 0, 5, 17, 6},
	{8, 2, 14, 10, 0, 17, 7, 5, 8, 12, 7, 12, 10, 3},
	{11, 3, 14, 7, 0, 6, 13, 0, 6, 15, 5, 7, 3, 12},
	{0, 12, 9, 3, 8, 9, 14, 15, 0, 4, 6, 5, 10, 17},
	{4, 9, 16, 8, 1, 4, 12, 4, 15, 2, 4, 6, 3, 15},
	{13, 3, 12, 1, 7, 0, 5, 8, 17, 10, 17, 5, 9, 17},
	{16, 4, 12, 9, 4, 13, 9, 1, 8, 2, 3, 12, 10, 0},
	{9, 1, 7, 4, 14, 2, 13, 8, 4, 13, 5, 11, 13, 10},
	{1, 7, 13, 16, 10, 16, 8, 5, 11, 0, 14, 8, 5, 7},
	{3, 15, 8, 13, 3, 10, 3, 12, 16, 11, 17, 0, 5, 12},
	{14, 10, 5, 10, 2, 13, 17, 7, 16, 7, 0, 3, 8, 0},
	{17, 2, 8, 12, 2, 16, 5, 10, 0, 17, 5, 16, 10, 3},
	{8, 5, 8, 0, 7, 3, 12, 10, 12, 6, 17, 3, 16, 7},
	{11, 17, 6, 11, 2, 6, 4, 14, 0, 11, 17, 0, 6, 13},
	{0, 3, 8, 2, 9, 17, 5, 7, 12, 8, 0, 14, 4, 16},
	{4, 14, 7, 5, 11, 13, 4, 6, 1, 15, 1, 13, 1, 7},
	{13, 9, 16, 7, 12, 9, 4, 12, 8, 15, 1, 4, 11, 17},
	{16, 1, 6, 14, 15, 8, 16, 4, 11, 13, 2, 7, 1, 3},
	{9, 0, 14, 5, 6, 1, 15, 2, 12, 4, 7, 17, 10, 16},
	{1, 7, 0, 13, 1, 14, 0, 14, 7, 13, 9, 0, 6, 1},
	{3, 10, 17, 11, 1, 15, 2, 8, 11, 16, 5, 14, 16, 7},
	{12, 2, 17, 8, 2, 6, 10, 0, 12, 6, 12, 0, 11, 12},
	{17, 7, 1, 6, 9, 15, 9, 5, 9, 14, 15, 7, 10, 3},
	{8, 11, 2, 17, 6, 1, 7, 11, 3, 6, 10, 16, 3, 14},
	{11, 1, 14, 8, 0, 6, 5, 8, 2, 4, 14, 9, 3, 17},
	{2, 8, 15, 7, 4, 16, 8, 5, 16, 0, 10, 13, 2, 6},
	{4, 9, 15, 5, 12, 1, 15, 1, 5, 10, 5, 13, 5, 9},
	{13, 1, 10, 16, 3, 13, 9, 15, 0, 14, 3, 6, 16, 8},
	{16, 6, 10, 13, 4, 12, 3, 14, 11, 15, 0, 9, 12, 17},
	{9, 13, 9, 1, 13, 4, 17, 6, 4, 16, 4, 14, 1, 10},
	{1, 10, 13, 4, 16, 8, 10, 17, 5, 13, 8, 0, 15, 9},
	{3, 16, 11, 2, 8, 16, 4, 9, 5, 16, 11, 16, 5, 12},
	{12, 7, 11, 15, 11, 17, 6, 0, 16, 0, 17, 9, 15, 4},
	{17, 11, 12, 10, 2, 13, 2, 7, 2, 12, 4, 16, 0, 7},
	{8, 17, 8, 5, 14, 11, 14, 17, 8, 11, 1, 12, 7, 4},
	{11, 14, 8, 4, 11, 4, 7, 5, 9, 12, 10, 1, 14, 16},
	{2, 9, 16, 2, 5, 14, 10, 16, 2, 11, 0, 5, 6, 2},
	{4, 17, 9, 1, 12, 5, 17, 8, 11, 13, 15, 6, 12, 4}
	};

	for (int x = 13; x >= 0; x--)
	{
		c.UndoAction(start, instances[which][x]);
		printf("%d ", instances[which][x]);
	}
	printf("\n");
}
