/*
 *  LLPDB.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/18/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
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

#include "LLPDB.h"
#include "Map2DEnvironment.h"
#include "MapQuadTreeAbstraction.h"
#include "Common.h"
#include "Sample.h"
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include <deque>
#include "IDAStar.h"
#include "TemplateAStar.h"
#include "GraphCanonicalHeuristic.h"
#include "Propagation.h"

void TestPDB();
GraphCanonicalHeuristic *gch = 0;
//void BuildPDB(Map *map);
//void GetCenters(MapQuadTreeAbstraction *msa, std::vector<int> &centers);
//void GetPDBValues(MapQuadTreeAbstraction *msa, const std::vector<int> &centers,
//				  std::vector<std::vector<double> > &lengths);


bool mouseTracking;
int px1, py1, px2, py2;
int absType = 0;

std::vector<UnitMapSimulation *> unitSims;

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
	{
		map = new Map(128, 128);
//		//MakeMaze(map);
//		for (int x = 0; x < 100; x++)
//		{
//			char name[255];
//			sprintf(name, "maze_%d%d%d.map", x/100, (x%100)/10, (x%10));
		MakeMaze(map);
//			map->save(name);
//		}
	}
	else
		map = new Map(gDefaultMap);
	
	unitSims.resize(id+1);
	unitSims[id] = new UnitMapSimulation(new MapEnvironment(map));
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
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kNoModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');
	
	InstallKeyboardHandler(MyPDBKeyHandler, "Build PDB", "Build a sliding-tile PDB", kNoModifier, 'b');
	InstallKeyboardHandler(MyPDBKeyHandler, "Test PDB", "Test a sliding-tile PDB", kNoModifier, 't');
	
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
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if ((windowID < unitSims.size()) && (unitSims[windowID] == 0))
		return;
	
	if (viewport == 0)
	{
		unitSims[windowID]->StepTime(1.0/30.0);
	}
	unitSims[windowID]->OpenGLDraw(windowID);
	
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
		case '\t':
			if (mod != kShiftDown)
				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
			else
			{
				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
			}
			break;
		case 'p': unitSims[windowID]->SetPaused(!unitSims[windowID]->GetPaused()); break;
		case 'o':
			if (unitSims[windowID]->GetPaused())
			{
				unitSims[windowID]->SetPaused(false);
				unitSims[windowID]->StepTime(1.0/30.0);
				unitSims[windowID]->SetPaused(true);
			}
			break;
		case ']': absType = (absType+1)%3; break;
		case '[': absType = (absType+4)%3; break;
			//		case '{': unitSim->setPaused(true); unitSim->offsetDisplayTime(-0.5); break;
			//		case '}': unitSim->offsetDisplayTime(0.5); break;
		default:
			break;
	}
}


void MyPDBKeyHandler(unsigned long, tKeyboardModifier, char key)
{
	if (key == 'b')
	{
		//BuildPDB(unitSims.back()->GetEnvironment()->GetMap());
		delete gch;
		gch = new GraphCanonicalHeuristic(unitSims.back()->GetEnvironment()->GetMap(), 8);
	}
	if (key == 't')
	{
		for (int x = 0; x < 200; x++)
		{
			TestPDB();
		}
	}
}


void TestPDB()
{
	Graph *g = gch->GetGraph();
	GraphMapHeuristic gmh(gch->GetMap(), gch->GetGraph());

	GraphEnvironment *env1 = new GraphEnvironment(g, gch);
	GraphEnvironment *env2 = new GraphEnvironment(g, &gmh);
	env1->SetDirected(false);
	env2->SetDirected(false);
	
	node *from = g->GetRandomNode();
	node *to = g->GetRandomNode();
	std::vector<graphState> thePath;
	Prop astar(PROP_BPMX);
	//TemplateAStar<graphState, graphMove, GraphEnvironment> astar;

	graphState start, goal;
	start = from->GetNum();
	goal = to->GetNum();
	astar.GetPath(env2, g, start, goal, thePath);
	//astar.GetPath(env2, start, goal, thePath);
	printf("%d\t%d\t%d\t",
		   astar.GetNodesExpanded(), astar.GetNodesTouched(), (int)thePath.size());//, p->GetNodesReopened());

	astar.GetPath(env1, g, start, goal, thePath);
	//astar.GetPath(env1, start, goal, thePath);
	printf("%d\t%d\t%d\n",
		   astar.GetNodesExpanded(), astar.GetNodesTouched(), (int)thePath.size());//, p->GetNodesReopened());
}


bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	return false;
}
