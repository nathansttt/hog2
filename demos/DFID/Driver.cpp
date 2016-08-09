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
#include "IDAStar.h"
#include "Timer.h"
#include "IncrementalDFID.h"
#include "IncrementalBFS.h"

bool recording = false;
bool running = false;

int drawMode = 0; // bit 0 [dfs] bit 1 [dfid] bit 2 [bfs]

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1440, 1080);
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

//	InstallKeyboardHandler(STPTest, "STP Test", "Test the STP PDBs", kNoModifier, 'd');
//	InstallKeyboardHandler(BuildSTP_PDB, "Build STP PDBs", "Build PDBs for the STP", kNoModifier, 'a');

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
		SetNumPorts(windowID, 1);
//		if (mnp == 0)
//			mnp = new MNPuzzle<4, 4>;
	}
}

#include "NaryTree.h"


double v = 1;

NaryTree tree(3, 5);
IncrementalDFID<NaryState, NaryAction> dfid(0);
IncrementalDFID<NaryState, NaryAction> dfs(10);
IncrementalBFS<NaryState, NaryAction> bfs;
std::vector<NaryState> path;

NaryState goal = 0;

int frameCnt = 0;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (running)
	{
		MyDisplayHandler(windowID, kNoModifier, 'o');
	}
	tree.SetWidthScale(double(GetContext(windowID)->globalCamera.viewWidth)/GetContext(windowID)->globalCamera.viewHeight);
	tree.OpenGLDraw();
	tree.SetColor(0.0, 1.0, 1.0);
	tree.OpenGLDraw(goal);
	
	tree.SetColor(1, 0, 0);
	if (drawMode&0x1)
		dfid.OpenGLDraw();
	tree.SetColor(0, 1, 0);
	if (drawMode&0x2)
		dfs.OpenGLDraw();
	tree.SetColor(0.75, 0, 0.5);
	if (drawMode&0x4)
		bfs.OpenGLDraw();

	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/dfid-%d-%d%d%d%d", tree.GetBranchingFactor(), (frameCnt/1000)%10, (frameCnt/100)%10, (frameCnt/10)%10, frameCnt%10);
		SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
		frameCnt++;
	}
	return;
	
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
		case '{':
		{
			goal = tree.GetParent(goal);
			break;
		}
		case ']':
		{
			drawMode = (drawMode+2)&0x7;
		}
		case '[':
		{
			drawMode = (drawMode+7)&0x7;
		}
			break;
		case 'r':
			recording = !recording; running = true;
			break;
		case '0':
		{
		}
			break;
		case '1':
		{
		}
			break;
		case '2':
		{
			frameCnt = 0;
			tree = NaryTree(2, 8);
			goal = tree.GetLastNode();
			dfid.Reset();
			dfs.Reset();
			bfs.Reset();
			break;
		}
		case '3':
		{
			frameCnt = 0;
			tree = NaryTree(3, 5);
			goal = tree.GetLastNode();
			dfid.Reset();
			dfs.Reset();
			bfs.Reset();
			break;
		}
		case '4':
		{
			frameCnt = 0;
			tree = NaryTree(4, 4);
			goal = tree.GetLastNode();
			dfid.Reset();
			dfs.Reset();
			bfs.Reset();
			break;
		}
		case '5':
		{
			frameCnt = 0;
			tree = NaryTree(5, 4);
			goal = tree.GetLastNode();
			dfid.Reset();
			dfs.Reset();
			bfs.Reset();
			break;
		}
		case '6':
		{
			frameCnt = 0;
			tree = NaryTree(6, 3);
			goal = tree.GetLastNode();
			dfid.Reset();
			dfs.Reset();
			bfs.Reset();
			break;
		}
		case '7':
		{
			frameCnt = 0;
			tree = NaryTree(7, 3);
			goal = tree.GetLastNode();
			dfid.Reset();
			dfs.Reset();
			bfs.Reset();
			break;
		}
		case '8':
		{
			frameCnt = 0;
			tree = NaryTree(8, 3);
			goal = tree.GetLastNode();
			dfid.Reset();
			dfs.Reset();
			bfs.Reset();
			break;
		}
		case '9':
		{
			frameCnt = 0;
			tree = NaryTree(9, 3);
			goal = tree.GetLastNode();
			dfid.Reset();
			dfs.Reset();
			bfs.Reset();
			break;
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
			running = !running;
			break;
		case 'o':
		{
			if (drawMode&0x1)
				dfid.DoSingleSearchStep(&tree, 0, goal, path);
			if (drawMode&0x2)
				dfs.DoSingleSearchStep(&tree, 0, goal, path);
			if (drawMode&0x4)
				bfs.DoSingleSearchStep(&tree, 0, goal, path);
		}
			break;
		default:
			break;
	}
	
	{
		char txt[] = "[bf 0]  ";
		txt[4] += tree.GetBranchingFactor();
		submitTextToBuffer(txt);
		switch(drawMode)
		{
			case 0: appendTextToBuffer("none"); break;
			case 1: appendTextToBuffer("DFID"); break;
			case 2: appendTextToBuffer("DFS"); break;
			case 3: appendTextToBuffer("DFID+DFS"); break;
			case 4: appendTextToBuffer("BFS"); break;
			case 5: appendTextToBuffer("BFS+DFID"); break;
			case 6: appendTextToBuffer("BFS+DFS"); break;
			case 7: appendTextToBuffer("all"); break;
		}
	}

}


bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	return false;
}
