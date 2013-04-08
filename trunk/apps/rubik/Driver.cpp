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
//#include "Plot2D.h"
//#include "RandomUnit.h"
//#include "MNPuzzle.h"
//#include "FlipSide.h"
#include "IDAStar.h"
#include "Timer.h"
#include "RubiksCube.h"
#include "DiskBitFile.h"

RubiksCube c;
RubiksAction a;
RubiksState s;

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

	InstallCommandLineHandler(MyCLHandler, "-test", "-test entries length", "Test using 'entries' billion entries from edge pdb and 'length' random walk");
	
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
		SetNumPorts(windowID, 1);
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	c.OpenGLDraw(s);
}

void RunTest(int billionEntriesToLoad, int randomWalkLength = 10);

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	if (maxNumArgs == 2)
	{
		RunTest(atoi(argument[1]));
	}
	else {
		RunTest(atoi(argument[1]),
				atoi(argument[2]));
		
	}
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
		case '9':
			printf("Old hash is %llu. ", c.GetStateHash(s));
			c.ApplyAction(s, key-'0');
			printf("New is %llu\n", c.GetStateHash(s));
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
	DiskBitFile f("/Users/nathanst/Development/cc/rubik/RC-corner");
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
	DiskBitFile f("/Users/nathanst/Development/cc/rubik/pdb/RC");
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

void SolveOneProblem(int walkLength, int id)
{
	RubiksState start, goal;
	goal.Reset();
	start.Reset();
	
	std::vector<RubiksAction> acts;
	c.SetPruneSuccessors(true);
	for (int x = 0; x < walkLength; x++)
	{
		c.GetActions(start, acts);
		RubiksAction a = acts[random()%acts.size()];
		c.UndoAction(start, a);
		printf("%d ", a);
	}
	printf("\n");
	
	s = start;
	IDAStar<RubiksState, RubiksAction> ida;
	c.SetPruneSuccessors(true);
	Timer t;
	t.StartTimer();
	ida.SetUseBDPathMax(true);
	ida.GetPath(&c, start, goal, acts);
	t.EndTimer();
	printf("Problem %d - %llu expanded; %1.2f elapsed\n", id, ida.GetNodesExpanded(), t.GetElapsedTime());
	for (unsigned int x = 0; x < acts.size(); x++)
	{
		printf("%d ", acts[x]);
		c.ApplyAction(s, acts[x]);
	}
	printf("\n");
}

void MyPathfindingKeyHandler(unsigned long , tKeyboardModifier , char)
{
	LoadCornerPDB();
	LoadEdgePDB(1000000000);

	for (int x = 0; x < 30; x++)
	{
		srandom(9283+x*23);
		SolveOneProblem(10, x+1);
	}
	
//	LoadCornerPDB();
	//	static int t = 0;
//	c.GetStateFromHash(t, s);
//	t++;
//	return;
}

void RunTest(int billionEntriesToLoad, int randomWalkLength)
{
	uint64_t numEntries = 1000000000;
	numEntries *= billionEntriesToLoad;
	LoadCornerPDB();
	LoadEdgePDB(numEntries);
	
	for (int x = 0; x < 50; x++)
	{
		srandom(9283+x*23);
		SolveOneProblem(randomWalkLength, x+1);
	}
}

bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	return false;
}

