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

static FlipSideState fss(5);
static FlipSide fs(5);

RubiksCorner rk_c;
RubikEdge rk_e;
RubiksCornerState rk_sc;
RubikEdgeState rk_se;

bool mouseTracking;
int px1, py1, px2, py2;
int absType = 0;

std::vector<PuzzleSimulation *> unitSims;
MNPuzzle *mnp = 0;
//unit *cameraTarget = 0;

Plotting::Plot2D *plot = 0;
Plotting::Line *distLine = 0;

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
	Map *map;
	if (gDefaultMap[0] == 0)
		map = new Map(60, 60);
	else
		map = new Map(gDefaultMap);

	unitSims.resize(id+1);
	unitSims[id] = new PuzzleSimulation(mnp = new MNPuzzle(4, 4));
	unitSims[id]->SetStepType(kMinTime);
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
		CreateSimulation(windowID);
		SetNumPorts(windowID, 1);
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if ((windowID < unitSims.size()) && (unitSims[windowID] == 0))
		return;

	rk_e.OpenGLDraw(rk_se);
	rk_c.OpenGLDraw(rk_sc);
	return;
	
	if (viewport == 0)
	{
		unitSims[windowID]->StepTime(1.0/30.0);
	}
	if (viewport == 3)
	{
		fs.OpenGLDraw(fss);
		return;
	}
	if (unitSims[windowID]->GetUnit(viewport))
	{
		//printf("Drawing unit %d\n", viewport);
		MNPuzzleState s;
		unitSims[windowID]->GetUnit(viewport)->GetLocation(s);
		MNPuzzleState g(s.width, s.height);
		//printf("Distance: %f\n", unitSims[windowID]->GetEnvironment()->HCost(s, g));
		unitSims[windowID]->GetUnit(viewport)->OpenGLDraw(unitSims[windowID]->GetEnvironment(),
														  unitSims[windowID]->GetSimulationInfo() );
	}
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
		case '0':
		{
			static int x = 0;
			rk_e.ApplyAction(rk_se, (x/4)%18);
			rk_c.ApplyAction(rk_sc, (x/4)%18);
			x++;
		}
			break;
		case '1':
		{
			static uint64_t x = 0;
			rk_c.GetStateFromHash(x, rk_sc);
			printf("%llu : %llu\n", x, rk_c.GetStateHash(rk_sc));
			x+=12345;
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
			rk_e.ApplyAction(rk_se, key-'0'+9);
			rk_c.ApplyAction(rk_sc, key-'0'+9);
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
			rk_sc.Reset();
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
			else {
//				std::vector<flipMove> acts;
//				fs.GetActions(fss, acts);
//				fs.ApplyAction(fss, acts[random()%acts.size()]);
//				FlipSideState fg(5);
//				printf("Distance: %f\n", fs.HCost(fss, fg));
			}
			
			if (unitSims[windowID]->GetPaused())
			{
				unitSims[windowID]->SetPaused(false);
				unitSims[windowID]->StepTime(1.0/30.0);
				unitSims[windowID]->SetPaused(true);
			}
		}
			break;
		case ']': absType = (absType+1)%3; break;
		case '[': absType = (absType+4)%3; break;
//		case '{': unitSim->setPaused(true); unitSim->offsetDisplayTime(-0.5); break;
//		case '}': unitSim->offsetDisplayTime(0.5); break;
		default:
			//if (unitSim)
			//	unitSim->GetEnvironment()->GetMapAbstraction()->ToggleDrawAbstraction(((mod == kControlDown)?10:0)+(key-'0'));
			break;
	}
}

#include "MNAgentPuzzle.h"
#include "SequenceAlignment.h"
#include "BFS.h"
#include "DFID.h"
#include "DFS.h"
#include "NaryTree.h"

void MyRandomUnitKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	for (uint64_t x = 0; x < 1000000000; x++)
	{
		if (0 == x%100000)
			printf("%d\n", x);
		rk_e.GetStateFromHash(x, rk_se);
		assert(x == rk_e.GetStateHash(rk_se));
	}
	
	return;
	
	BFS<MNAgentPuzzleState, tAgentAction> bfs;
	for (unsigned int x = 1; x < 9; x++)
	{
		MNAgentEnvironment mnae;
		MNAgentPuzzleState mnps(3, 3);
		std::cout << mnps << std::endl;
		std::vector<MNAgentPuzzleState> s;
//		for (unsigned int y = 0; y < (9-x); y++)
//		{
//			std::cout << (9-x) << " agents, domain abstraction of " << y << " tiles." << std::endl;
			mnae.SetDomainAbstractionSize(3);
			bfs.GetPath(&mnae, mnps, mnps, s);
//		}
	}
}

void MyPathfindingKeyHandler(unsigned long , tKeyboardModifier , char)
{
	
	static int t = 0;
	rk_e.GetStateFromHash(t, rk_se);
	t++;
	return;
	uint64_t totNodes = 0;
	for (int x = 0; x < 362880; x++)
	{
		BFS<MNPuzzleState, slideDir> bfs;
		DFS<MNPuzzleState, slideDir> dfs;
		DFID<MNPuzzleState, slideDir> dfid2;
		IDAStar<MNPuzzleState, slideDir> ida;
		MNPuzzle mnae33(3, 3);
//		MNPuzzle mnae34(3, 4);
//		MNPuzzle mnae55(5, 5);
		MNPuzzleState mnps33(3, 3);
		MNPuzzleState mnps33g(3, 3);
//		for (unsigned int x = 0; x < mnps33g.puzzle.size(); x++)
//			if (mnps33g.puzzle[x] != 0)
//				mnps33g.puzzle[x] = 1+(14-mnps33g.puzzle[x])%8;

		mnae33.GetStateFromHash(mnps33, x);
		if (mnae33.GetParity(mnps33) != mnae33.GetParity(mnps33g))
			continue;
		
//		MNPuzzleState mnps34(3, 4);
		std::vector<MNPuzzleState> s;
		std::vector<slideDir> s2;

//		dfid2.GetPath(&mnae33, mnps33, mnps33, s2);
//		dfid2.GetPath(&mnae34, mnps34, mnps34, s2);
//		bfs.GetPath(&mnae33, mnps33, mnps33, s);
//		bfs.GetPath(&mnae34, mnps34, mnps34, s);

		std::cout << mnps33 << " to " << mnps33g << std::endl;
		ida.GetPath(&mnae33, mnps33, mnps33g, s2);
		printf("%d: %lld nodes expanded\n", x, ida.GetNodesExpanded());
		totNodes += ida.GetNodesExpanded();
//		for (unsigned int x = 0; x < s.size(); x++)
//			std::cout << s[x] << std::endl;
		
//		std::cout << mnps33 << std::endl << mnps33g << std::endl;
//		ida.GetPath(&mnae33, mnps33g, mnps33, s);
//		printf("%lld nodes expanded\n", ida.GetNodesExpanded());
//		for (unsigned int x = 0; x < s.size(); x++)
//			std::cout << s[x] << std::endl;
	}
	printf("%llu total nodes\n", totNodes);
	exit(0);
	
	if (0)
	{
		std::vector<NaryState> narypath;
		DFS<NaryState, NaryAction> dfs1;
		DFID<NaryState, NaryAction> dfid;
		NaryTree t1(2, 10);
		NaryTree t2(3, 10);
		NaryTree t3(4, 10);
		NaryState s1 = 0, g1;
		dfs1.GetPath(&t1, s1, s1, narypath);
		std::cout << dfs1.GetNodesExpanded() << " total nodes expanded" << std::endl;
		dfs1.GetPath(&t2, s1, s1, narypath);
		std::cout << dfs1.GetNodesExpanded() << " total nodes expanded" << std::endl;
		dfs1.GetPath(&t3, s1, s1, narypath);
		std::cout << dfs1.GetNodesExpanded() << " total nodes expanded" << std::endl;

		g1 = 2048-2;
		dfid.GetPath(&t1, s1, g1, narypath);
		std::cout << dfid.GetNodesExpanded() << " total nodes expanded" << std::endl;
		g1 = 88573-2;
		dfid.GetPath(&t2, s1, g1, narypath);
		std::cout << dfid.GetNodesExpanded() << " total nodes expanded" << std::endl;
		g1 = 1398101-2;
		dfid.GetPath(&t3, s1, g1, narypath);
		std::cout << dfid.GetNodesExpanded() << " total nodes expanded" << std::endl;
	}

	if (0)
	{
		std::vector<SequenceAlignmentState> statePath;
		BFS<SequenceAlignmentState, SequenceAlignmentAction> bfs;
		DFS<SequenceAlignmentState, SequenceAlignmentAction> dfs;
		DFID<SequenceAlignmentState, SequenceAlignmentAction> dfid;

		for (unsigned int x = 1; x < 13; x++)
		{
			SequenceAlignment sa10(x);
			SequenceAlignmentState s1 = 0, g1 = -1;

			bfs.GetPath(&sa10, s1, s1, statePath);
			std::cout << bfs.GetNodesExpanded() << " total BFS nodes expanded" << std::endl;
			
			dfs.GetPath(&sa10, s1, s1, statePath);
			std::cout << dfs.GetNodesExpanded() << " total DFS nodes expanded" << std::endl;
			
			dfid.GetPath(&sa10, s1, g1, statePath);
			std::cout << dfid.GetNodesExpanded() << " total DFID nodes expanded" << std::endl;
		}
	}
}

bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	return false;
}

