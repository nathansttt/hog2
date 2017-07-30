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
#include "Sample.h"
#include "ScenarioLoader.h"
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include "Voxels.h"
#include <iostream>
#include "Timer.h"
#include "TemplateAStar.h"
#include "VoxelGrid.h"

recVec velocity;
std::string map;


//struct voxelWorld {
//	uint32_t header;
//	float voxelSize;
//	uint64_t numVoxelsGrids;
//	float minbounds[4];
//	float maxbounds[4];
//	uint64_t *morton;
//	uint64_t *grid;
//};
//
//voxelWorld LoadData();
//void Draw(voxelWorld w);
bool mouseTracking = false;
bool recording = false;
bool capture = false;
int frames = 60;

//recVec cameraOffset(0,0,0);
//Voxels *v = 0;
VoxelGrid *vg = 0;
//voxelWorld theWorld;
std::vector<voxelGridState> path;

int main(int argc, char* argv[])
{
//	theWorld = LoadData();
	InstallHandlers();
	RunHOGGUI(argc, argv, 512, 512);
}


/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
void CreateSimulation(int id)
{
//	//FILE *f = fopen("/Users/nathanst/Desktop/3DNavMaps/test3dnavC1.3dnav", "r");
//	//FILE *f = fopen("/Users/nathanst/Desktop/3dmaps/Full4_test3dnav.3dnav", "r");
//	//FILE *f = fopen("/Users/nathanst/Desktop/3dmaps/Full3_test3dnav.3dnav", "r");
//	//FILE *f = fopen("/Users/nathanst/Desktop/3dmaps/Full2_test3dnav.3dnav", "r");
//	//FILE *f = fopen("/Users/nathanst/Desktop/3dmaps/Full1_test3dnav.3dnav", "r");
//	FILE *f = fopen("/Users/nathanst/Desktop/3dmaps/Complex_test3dnav.3dnav", "r");
//	//FILE *f = fopen("/Users/nathanst/Desktop/3dmaps/Simple_test3dnav.3dnav", "r");

//	v = new Voxels("/Users/nathanst/Desktop/3dmaps/Complex_test3dnav.3dnav");
//	v = new Voxels("/Users/nathanst/Desktop/3DNavMaps/test3dnavTR1_1m.3dnav");

	//vg = new VoxelGrid("/Users/nathanst/Desktop/3dmaps/Complex_test3dnav.3dmap");
//	vg = new VoxelGrid("/Users/nathanst/hog2/maps/warframe/FC2.3dmap");
//	vg = new VoxelGrid("/Users/nathanst/hog2/maps/warframe/Norm1.3dmap");
//	vg = new VoxelGrid("/Users/nathanst/hog2/maps/warframe/TR12m.3dmap");
//	vg = new VoxelGrid("/Users/nathanst/hog2/maps/warframe/TR11m.3dmap");
//	vg = new VoxelGrid("/Users/nathanst/hog2/maps/warframe/A5.3dmap");
	if (map.size() > 0)
		vg = new VoxelGrid(map.c_str());
	else
//		vg = new VoxelGrid("/Users/nathanst/hog2/maps/warframe/Norm2.3dmap");
		vg = new VoxelGrid("/Users/nathanst/hog2/maps/warframe/TR2_1m.3dmap");
	//vg = new VoxelGrid("/Users/nathanst/hog2/maps/warframe/A5.3dmap");

	vg->efficient = true;

	//	v = new Voxels("/Users/nathanst/Desktop/3dmaps/Full4_test3dnav.3dnav");
//	Map *map;
//	if (gDefaultMap[0] == 0)
//	{
//		map = new Map(mazeSize, mazeSize);
//		MakeMaze(map, 10);
////		map->Scale(mazeSize, mazeSize);
//	}
//	else {
//		map = new Map(gDefaultMap);
//		//map->Scale(512, 512);
//	}
//	
//	if (0 && gDefaultMap[0] == 0)
//	{
//		MakeStaircase();
////		m3d = new Map3DGrid(map, 16);
//		//m3d->AddMap(map, 0);
//		m3d->SetDrawGrid(true);
//		m3d->PrintStats();
//		//m3d->AddMap(map, 50);
//	}
//	else {
//		m3d = new Map3DGrid(map, 16);
//	}
//	map->SetTileSet(kWinter);
//	msa = new MapSectorAbstraction(map, 8);
//	msa->ToggleDrawAbstraction(0);
//	//msa->ToggleDrawAbstraction(2);
//	//msa->ToggleDrawAbstraction(3);
//	unitSims.resize(id+1);
//	unitSims[id] = new UnitSimulation<xyLoc, tDirection, MapEnvironment>(new MapEnvironment(map));
//	unitSims[id]->SetStepType(kMinTime);
//	m3d->SetDrawGrid(true);
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
//	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Reset Cam", "Reset camera", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Begin/end movie recording", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Fly", "Fly camera forward", kAnyModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Fly", "Fly camera back", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Fly", "Slide camera left", kAnyModifier, 'a');
	InstallKeyboardHandler(MyDisplayHandler, "Fly", "Slide camera right", kAnyModifier, 'd');
	InstallKeyboardHandler(MyDisplayHandler, "Fly", "Stop moving", kAnyModifier, ' ');
	InstallKeyboardHandler(MyDisplayHandler, "Path", "Find random path", kAnyModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Efficient", "Switch to efficient drawing", kAnyModifier, 'e');
	InstallKeyboardHandler(MyDisplayHandler, "Inside", "Fill outside and invert to find inside", kAnyModifier, 'i');
	InstallWindowHandler(MyWindowHandler);
	InstallCommandLineHandler(MyCLHandler, "-convert", "-convert <source> <dest>", "Convert a map from the DE format to the repo format");
	InstallCommandLineHandler(MyCLHandler, "-map", "-map <source>", "Use the provided map");
	InstallCommandLineHandler(MyCLHandler, "-capture", "-capture", "Capture animation");
//
//	InstallMouseClickHandler(MyClickHandler);
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
		
		mouseTracking = false;
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		CreateSimulation(windowID);
		SetNumPorts(windowID, 1);
		SetLighting(0.65, 1.0, 0.2);
	}
}

double distance = 12.5;

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '|': resetCamera(); break;
		case 'r': recording = !recording; break;
		case ' ':
		{
			velocity *= 0;
			break;
		}
		case 'a':
		{
			pRecContext pContextInfo = getCurrentContext();
			pContextInfo->camera[pContextInfo->currPort].thirdPerson = false;
			recVec v = GetHeading(windowID, pContextInfo->currPort);
			recVec up = {0, 1, 0};
			recVec result = v*up;
			//cameraOffset(result.x, result.y, result.z);
			std::cout << result << "\n";
			velocity += result;
			if (velocity.length() > 1.0)
				velocity.normalise();
			break;
		}
		case 'd':
		{
			pRecContext pContextInfo = getCurrentContext();
			pContextInfo->camera[pContextInfo->currPort].thirdPerson = false;
			recVec v = GetHeading(windowID, pContextInfo->currPort);
			recVec up = {0, 1, 0};
			recVec result = v*up;
			std::cout << result << "\n";
			velocity -= result;
			if (velocity.length() > 1.0)
				velocity.normalise();
			//cameraOffset(-result.x, -result.y, -result.z);
			break;
		}
		case 'w':
		{
			pRecContext pContextInfo = getCurrentContext();
			pContextInfo->camera[pContextInfo->currPort].thirdPerson = false;
			recVec v = GetHeading(windowID, pContextInfo->currPort);
			//cameraOffset(v.x, v.y, v.z);
			if (mod == kShiftDown)
				v *= 0.05;
			velocity += v;
			if (velocity.length() > 1.0)
				velocity.normalise();
			break;
		}			
		case 's':
		{
			pRecContext pContextInfo = getCurrentContext();
			pContextInfo->camera[pContextInfo->currPort].thirdPerson = false;
			recVec v = GetHeading(windowID, pContextInfo->currPort);
			if (mod == kShiftDown)
				v *= 0.05;
			velocity -= v;
			if (velocity.length() > 1.0)
				velocity.normalise();
			//cameraOffset(-v.x, -v.y, -v.z);
			break;
		}
		case 'e':
		{
			vg->efficient = !vg->efficient;
			break;
		}
		case 'i':
		{
			printf("Inverting world\n");
			vg->Fill({0,0,0});
			vg->Invert();
			break;
		}
		case 'p':
			while (true)
			{
				// TODO: Make sure we don't generate the same child twice
				Timer t;
				voxelGridState s = vg->GetRandomState();
				voxelGridState g = vg->GetRandomState();
//(233, 391, 362) to (702, 170, 62)
				//				s = {400, 395, 150};
//				g = {291, 278, 238};
				std::cout << "-->Searching from " << s << " to " << g << ". Base h = " << vg->HCost(s, g) << " \n";
				static TemplateAStar<voxelGridState, voxelGridAction, VoxelGrid> astar;
				t.StartTimer();
				astar.GetPath(vg, s, g, path);
				t.EndTimer();
				printf("%llu nodes expanded\n", astar.GetNodesExpanded());
				printf("%llu nodes generated\n", astar.GetNodesTouched());
				printf("%1.2fms elapsed\n", t.GetElapsedTime()*1000.0);
				printf("Path length: %1.2f. Heuristic: %1.2f\n", vg->GetPathLength(path), vg->HCost(s, g));
				printf("---Done!---\n");
				if (!fequal(vg->GetPathLength(path), vg->HCost(s, g)))
				break;
			}
	}
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-convert") == 0 && maxNumArgs >= 3)
	{
		Voxels v(argument[1]);
		v.Export(argument[2]);
		exit(0);
	}
	if (strcmp(argument[0], "-capture") == 0)
	{
		capture = true;
		if (maxNumArgs > 1)
		{
			if (argument[1][0] != '-')
			{
				frames = atoi(argument[1]);
				return 2;
			}
		}
		return 1;
	}
	if (strcmp(argument[0], "-map") == 0 && maxNumArgs >= 2)
	{
		map = argument[1];
		return 2;
	}

	return 0;
}
Timer frameRate;
std::string te;
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (capture)
	{
		static int currentFrame = 0;
		glRotatef(currentFrame*360.0/frames, 0, 1, 0);
		vg->OpenGLDraw();

		char fname[255];
		sprintf(fname, "%s-%d%d%d%d",
				map.c_str(), (currentFrame/1000)%10, (currentFrame/100)%10, (currentFrame/10)%10, currentFrame%10);
		SaveScreenshot(windowID, fname);
		currentFrame++;
		if (currentFrame%frames == 0)
			exit(0);

		return;
	}
	frameRate.EndTimer();
	te = std::to_string(1.0/frameRate.GetElapsedTime());
	submitTextToBuffer(te.c_str());
	frameRate.StartTimer();

	
	if (viewport == 0)
	{
		cameraOffset(velocity.x, velocity.y, velocity.z);
		
//		v->OpenGLDraw();
	}
//	if (viewport == 1)
	{
		vg->OpenGLDraw();

		if (path.size() > 0)
		{
			vg->SetColor(1.0, 1.0, 1.0);
			glColor3f(1, 1, 1);
			glLineWidth(8.0);
			for (int x = 0; x < path.size()-1; x++)
				vg->GLDrawLine(path[x], path[x+1]);
			glLineWidth(1.0);
		}
	}
	//LoadData();
	//Draw(theWorld);

	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		static int cnt = 0;
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/%d%d%d%d", (cnt/1000)%10, (cnt/100)%10, (cnt/10)%10, cnt%10);
		SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
		cnt++;
	}
}


bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	return false;
}

void BuildBenchmarks(const char *map, int resolution)
{
	VoxelGrid v(map);
	
	exit(0);
}

