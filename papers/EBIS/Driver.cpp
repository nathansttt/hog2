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
#include "EBSearch.h"
#include "TemplateAStar.h"
#include "PancakePuzzle.h"
#include "PancakeInstances.h"
#include "TOH.h"
#include "GraphEnvironment.h"
#include "GraphInconsistencyInstances.h"
#include "IBEX.h"

MNPuzzle<4, 4> p;
MNPuzzleState<4, 4> s, t;
std::vector<slideDir> moves, tmpPath;
double v = 1;

bool recording = false;
void TestSTP(int instance, int algorithm, int minGrowth, int maxGrowth, int startEpsilon, int scale);
void TestPancake(int instance, int algorithm, int minGrowth, int maxGrowth, int startEpsilon, int scale);
void TestTOH(int instance, int algorithm, int minGrowth, int maxGrowth, int startEpsilon, int scale);
void TestPolygraph(int instance, int algorithm, int minGrowth, int maxGrowth, int startEpsilon, int scale);

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

	InstallCommandLineHandler(MyCLHandler, "-stp", "-stp <instance> <algorithm> <c1> <c2> <ep>", "Runs weighted STP with MD");
	InstallCommandLineHandler(MyCLHandler, "-pancake", "-pancake <instance> <algorithm> <c1> <c2> <ep>", "Runs weighted pancake with GAP");
	InstallCommandLineHandler(MyCLHandler, "-polygraph", "-polygraph <instance> <algorithm> <c1> <c2> <ep>", "Runs worst-case inconsistency graph");
	
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
		EBSearch<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> ebis(2, 5, 1);
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
		int c1 = 2, c2 = 5, ep = -1, scale = 1;
		if (maxNumArgs < 3)
		{
			printf("Error: didn't pass arguments: <instance> <algorithm>\n");
			exit(0);
		}
		if (maxNumArgs >= 4)
			c1 = atoi(argument[3]);
		if (maxNumArgs >= 5)
			c2 = atoi(argument[4]);
		if (maxNumArgs >= 6)
			ep = atoi(argument[5]);
		if (maxNumArgs >= 7)
			scale = atoi(argument[6]);

		TestSTP(atoi(argument[1]), atoi(argument[2]), c1, c2, ep, scale);
	}
	if (strcmp(argument[0], "-pancake") == 0)
	{
		int c1 = 2, c2 = 5, ep = -1, scale = 1;
		if (maxNumArgs < 3)
		{
			printf("Error: didn't pass arguments: <instance> <algorithm>\n");
			exit(0);
		}
		if (maxNumArgs >= 4)
			c1 = atoi(argument[3]);
		if (maxNumArgs >= 5)
			c2 = atoi(argument[4]);
		if (maxNumArgs >= 6)
			ep = atoi(argument[5]);
		if (maxNumArgs >= 7)
			scale = atoi(argument[6]);

		TestPancake(atoi(argument[1]), atoi(argument[2]), c1, c2, ep, scale);
	}
	if (strcmp(argument[0], "-polygraph") == 0)
	{
		int c1 = 2, c2 = 5, ep = -1, scale = 1;
		if (maxNumArgs < 3)
		{
			printf("Error: didn't pass arguments: <instance> <algorithm>\n");
			exit(0);
		}
		if (maxNumArgs >= 4)
			c1 = atoi(argument[3]);
		if (maxNumArgs >= 5)
			c2 = atoi(argument[4]);
		if (maxNumArgs >= 6)
			ep = atoi(argument[5]);
		if (maxNumArgs >= 7)
			scale = atoi(argument[6]);

		TestPolygraph(atoi(argument[1]), atoi(argument[2]), c1, c2, ep, scale);
	}	exit(0);
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

template <class state, class action, class environment, bool DFS = true>
void Test(environment *e, Heuristic<state> *h, const state &start, const state &goal, int algorithm, int minGrowth, int maxGrowth, int startEpsilon, uint64_t multiple)
{
	std::vector<action> moves, tmpPath;
	
	TemplateAStar<state, action, environment> astar;
	IDAStar<state, action> ida;
	EBSearch<state, action, environment, DFS> ebis(minGrowth, maxGrowth, startEpsilon, multiple);
	double gamma = startEpsilon;
	while (algorithm == 3 && gamma > 10)
		gamma /= 10.0;
	IBEX::IBEX<state, action, environment, DFS> ibex(minGrowth, maxGrowth, gamma);
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
			if (DFS == false)
				astar.SetReopenNodes(true);
			t.StartTimer();
			astar.GetPath(e, start, goal, path);
			t.EndTimer();
			printf("A*: %1.2fs elapsed; %llu expanded; %llu generated; solution length %f\n", t.GetElapsedTime(),
				   astar.GetNodesExpanded(), astar.GetNodesTouched(), e->GetPathLength(path));
		}
			break;
		case 3: // IBEX Search
		{
			printf("IBEX(%d, %d, %1.2f): ", minGrowth, maxGrowth, gamma);
			std::cout << " search from\n" << start << "\n" << goal << "\n";
			t.StartTimer();
			
			ibex.GetPath(e, h, start, goal, tmpPath);
			t.EndTimer();
			printf("IBEX: %1.2fs elapsed; %llu expanded; %llu generated; solution length %f\n", t.GetElapsedTime(),
				   ibex.GetNodesExpanded(), ibex.GetNodesTouched(), e->GetPathLength(start, tmpPath));
			
			t.StartTimer();
			double cost = ibex.RedoMinWork();
			t.EndTimer();
			assert(fequal(cost, e->GetPathLength(start, tmpPath)));
			printf("Oracle: %1.2fs elapsed; %llu expanded; %llu generated; solution length %f\n", t.GetElapsedTime(),
				   ibex.GetNodesExpanded(), ibex.GetNodesTouched(), e->GetPathLength(start, tmpPath));
		}
			break;
	}
	exit(0);
}

void TestSTP(int instance, int algorithm, int minGrowth, int maxGrowth, int startEpsilon, int scale)
{
	MNPuzzle<4, 4> stp;
	MNPuzzleState<4, 4> start, goal;
	
	stp.SetWeighted(kUnitPlusFrac);
	
	assert(instance >= 0 && instance < 100);
	start = STP::GetKorfInstance(instance);
	Test<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(&stp, &stp, start, goal, algorithm, minGrowth, maxGrowth, startEpsilon, scale);
}

void TestPancake(int instance, int algorithm, int minGrowth, int maxGrowth, int startEpsilon, int scale)
{
	PancakePuzzle<20> pancake;
	PancakePuzzleState<20> start, goal;
	pancake.SetUseRealValueEdges(true);
	assert(instance >= 0 && instance < 100);
	GetPancakeInstance(start, instance);
	Test<PancakePuzzleState<20>, PancakePuzzleAction, PancakePuzzle<20>>(&pancake, &pancake, start, goal, algorithm, minGrowth, maxGrowth, startEpsilon, scale);
}

void TestPolygraph(int instance, int algorithm, int minGrowth, int maxGrowth, int startEpsilon, int scale)
{
	Graph *g = GraphInconsistencyExamples::GetPolyGraph(instance);
	GraphEnvironment *ge = new GraphEnvironment(g);
	GraphInconsistencyExamples::GraphHeuristic h(g);
	Test<graphState, graphMove, GraphEnvironment, false>(ge, &h, 0, g->GetNumNodes()-1, algorithm, minGrowth, maxGrowth, startEpsilon, scale);
}

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
