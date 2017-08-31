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
#include "GLUTHog.h"

recVec velocity;
std::string map;
void Maze3D(int d);


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
	Maze3D(11);
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
//		vg = new VoxelGrid("/Users/nathanst/hog2/maps/warframe/TR2_1m.3dmap");
		vg = new VoxelGrid("/Users/nathanst/hog2/maps/warframe/A5.3dmap");

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

#include "Map.h"
#include "MapGenerators.h"
#include "Graph.h"
#include "Prim.h"
#include "Map2DEnvironment.h"
#include "SVGUtil.h"
#include <time.h>

bool MakeMazeMST(int d);
void MazeMazeTree(int d);

void Maze3D(int d)
{
	srandom(time(0));
	int cnt = 0;
	while (true)
	{
		printf("Attept %d\n", ++cnt);
		bool result = MakeMazeMST(d);
		if (result)
			break;
	}
	MazeMazeTree(d);
	exit(0);
}

edge *GetEdge(Graph *g, int x1, int y1, int z1, int x2, int y2, int z2);

bool MakeMazeMST(int s)
{
	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
	std::vector<xyLoc> path;
	Graph *g;
	g = new Graph();
	int d = s*2+1;
//	int s = d/2;
//	d = s*2+1;
	for (int z = 0; z < 2; z++)
	{
		for (int x = 0; x < s; x++)
		{
			for (int y = 0; y < s; y++)
			{
				// skip top area for logo (no nodes at all)
				if (z == 0 && (x > s/4 && x < 3*s/4) && (y > s/4 && y < 3*s/4))
				{
					continue;
				}
				node *n;
				n = new node("");
				n->SetLabelL(0, x);
				n->SetLabelL(1, y);
				n->SetLabelL(2, z);
				g->AddNode(n);
			}
		}
	}
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		node *n1 = g->GetNode(x);
		for (int y = 0; y < g->GetNumNodes(); y++)
		{
			node *n2 = g->GetNode(y);
			// Node 1 id must be < node 2 id
			if (x == y || n1->GetNum() >= n2->GetNum())
				continue;

//			printf("Considering edge (%ld, %ld, %ld) to (%ld, %ld, %ld)\n",
//				   n1->GetLabelL(0), n1->GetLabelL(1), n1->GetLabelL(2),
//				   n2->GetLabelL(0), n2->GetLabelL(1), n2->GetLabelL(2));

			int sum = 0;
			sum += std::abs(n1->GetLabelL(0) - n2->GetLabelL(0));
			sum += std::abs(n1->GetLabelL(1) - n2->GetLabelL(1));
			sum += std::abs(n1->GetLabelL(2) - n2->GetLabelL(2));

			// only 1 coordinate changed
			if (sum == 1)
			{
//				printf("Adding edge (%ld, %ld, %ld) to (%ld, %ld, %ld)\n",
//					   n1->GetLabelL(0), n1->GetLabelL(1), n1->GetLabelL(2),
//					   n2->GetLabelL(0), n2->GetLabelL(1), n2->GetLabelL(2));
				edge *e;
				if (n1->GetLabelL(2) + n2->GetLabelL(2) == 1) // through bottom
					e = new edge(n1->GetNum(), n2->GetNum(), 100000);
				else
					e = new edge(n1->GetNum(), n2->GetNum(), random()%50);
				g->AddEdge(e);
			}
		}
	}

	
	// Reweight edges we don't want in the MST
	
	// New idea (to implement here)
			// 0. Board is split into
			//       Top    saaxxx  Bottom  bbbbbb
			//              ah  hx          bh  hb
			//              ha  xh          hc  ch
			//              aaaxxg          cccccc
			// 1. Limited holes from top to bottom. Fixed at:
			//    Corners (start/goal) [should goal be top right?]
			//    [[OTHER TBA]]
			// 2. Find MST as normal
			//
	{
		// Start gets no horizontal connections
		GetEdge(g,
				0, 0, 1,
				1, 0, 1)->setWeight(100000);
		GetEdge(g,
				0, 0, 1,
				0, 1, 1)->setWeight(100000);
		// No down move from start
		GetEdge(g,
				0, 0, 0,
				0, 1, 0)->setWeight(100000);
		// Start is connected to top
		GetEdge(g,
				0, 0, 1,
				0, 0, 0)->setWeight(0);
		// Goal gets no horizontal connections
		GetEdge(g,
				s-1, s-1, 1,
				s-2, s-1, 1)->setWeight(100000);
		GetEdge(g,
				s-1, s-1, 1,
				s-1, s-2, 1)->setWeight(100000);
		// Goal is connected to top
		GetEdge(g,
				0, 0, 1,
				0, 0, 0)->setWeight(0);
		GetEdge(g,
				s-1, s-1, 1,
				s-1, s-1, 0)->setWeight(0);
		// No left move from goal
		GetEdge(g,
				s-1, s-1, 0,
				s-2, s-1, 0)->setWeight(100000);
	}

	// Set pass-through points
	{
		// top middle-right
		GetEdge(g,
				3*s/4-1, 0, 0,
				3*s/4-1, 0, 1)->setWeight(0);
		// control edges
		GetEdge(g,
				3*s/4-1, 0, 1,
				3*s/4, 0, 1)->setWeight(100000);
		GetEdge(g,
				3*s/4-1, 0, 1,
				3*s/4-1, 1, 1)->setWeight(100000);
		// left middle-bottom
		GetEdge(g,
				0, 3*s/4-1, 0,
				0, 3*s/4-1, 1)->setWeight(0);
		// control edges
		GetEdge(g,
				0, 3*s/4-1, 1,
				0, 3*s/4, 1)->setWeight(100000);
		GetEdge(g,
				0, 3*s/4-1, 1,
				1, 3*s/4-1, 1)->setWeight(100000);

		// corners
		GetEdge(g,
				s-1, 0, 0,
				s-1, 0, 1)->setWeight(0);
		GetEdge(g,
				0, s-1, 0,
				0, s-1, 1)->setWeight(0);

		// bottom middle-right
		GetEdge(g,
				3*s/4-1, s-1, 0,
				3*s/4-1, s-1, 1)->setWeight(0);
		GetEdge(g,
				3*s/4-1, s-2, 1,
				3*s/4-1, s-1, 1)->setWeight(100000);
		GetEdge(g,
				3*s/4,   s-1, 1,
				3*s/4-1, s-1, 1)->setWeight(100000);
		// rightmiddle-bottom
		GetEdge(g,
				s-1, 3*s/4-1, 0,
				s-1, 3*s/4-1, 1)->setWeight(0);
		GetEdge(g,
				s-1, 3*s/4-1, 1,
				s-2, 3*s/4-1, 1)->setWeight(100000);
		GetEdge(g,
				s-1, 3*s/4-1, 1,
				s-1, 3*s/4, 1)->setWeight(100000);


		// final edge
		GetEdge(g,
				3*s/4, 3*s/4, 0,
				3*s/4, 3*s/4, 1)->setWeight(0);
		GetEdge(g,
				3*s/4-1, 3*s/4, 1,
				3*s/4, 3*s/4, 1)->setWeight(100000);
		GetEdge(g,
				3*s/4, 3*s/4+1, 1,
				3*s/4, 3*s/4, 1)->setWeight(100000);
		GetEdge(g,
				3*s/4, 3*s/4-1, 1,
				3*s/4, 3*s/4, 1)->setWeight(100000);

	}
	
	// Remove edges across middle
	{
		// split from middle across to right
		for (int x = 0; x < s; x++)
		{
			edge *e = GetEdge(g,
							  x, 3*s/4-1, 0,
							  x, 3*s/4, 0);
			if (e) e->setWeight(100000);
		}
		
		// split from middle across to right
		for (int y = 0; y < s; y++)
		{
			edge *e = GetEdge(g,
							  3*s/4, y, 0,
							  3*s/4-1, y, 0);
			if (e) e->setWeight(100000);
		}

		for (int t = s/2; t < s; t++)
		{
			edge *e;
			e = GetEdge(g,
						s/2, t, 1,
						s/2+1, t, 1);
			if (e) e->setWeight(100000);
//			else { printf("No edge (%d, %d, %d) (%d, %d, %d)\n", s/2, t, 1, s/2+1, t, 1); }
			e = GetEdge(g,
						t, s/2, 1,
						t, s/2+1, 1);
			if (e) e->setWeight(100000);
//			else { printf("No edge (%d %d %d) (%d %d %d)\n", t, s/2, 1, t, s/2+1, 1); }
			
		}
	}
	
	Map *m1, *m2, *m3;
	m1 = new Map(d, d);
	m2 = new Map(d, d);
	m3 = new Map(d, d);
	MapEnvironment me1(m1);
	MapEnvironment me2(m2);
	MapEnvironment me3(m3);
	for (int x = 0; x < d; x++)
		for (int y = 0; y < d; y++)
		{
			m1->SetTerrainType(x, y, kOutOfBounds);
			m2->SetTerrainType(x, y, kOutOfBounds);
			m3->SetTerrainType(x, y, kOutOfBounds);
		}

	Prim(g);
	const int edgeWidth = 2;
	const int edgeStart = 1;
	const int edgeEnd = 1;
	for (int x = 0; x < g->GetNumEdges(); x++)
	{
		edge *e = g->GetEdge(x);
		if (e->getMarked() == true)
		{
			node *n1 = g->GetNode(e->getFrom());
			node *n2 = g->GetNode(e->getTo());
			long x1, x2, y1, y2, z1, z2;
			x1 = n1->GetLabelL(0);
			x2 = n2->GetLabelL(0);
			y1 = n1->GetLabelL(1);
			y2 = n2->GetLabelL(1);
			z1 = n1->GetLabelL(2);
			z2 = n2->GetLabelL(2);
			if (z1 == z2 && z1 == 0)
			{
				m1->SetRectHeight(x1*edgeWidth+edgeStart, y1*edgeWidth+edgeStart, x2*edgeWidth+edgeEnd, y2*edgeWidth+edgeEnd, 0, kGround);
			}
			else if (z1 == z2 && z1 == 1)
			{
				m3->SetRectHeight(x1*edgeWidth+edgeStart, y1*edgeWidth+edgeStart, x2*edgeWidth+edgeEnd, y2*edgeWidth+edgeEnd, 0, kGround);
			}
			else if (z1 != z2)
			{
				m2->SetRectHeight(x1*edgeWidth+edgeStart, y1*edgeWidth+edgeStart, x1*edgeWidth+edgeEnd, y1*edgeWidth+edgeEnd, 0, kGround);
			}
		}
	}
	
	xyLoc start, goal;
	start = {1, 1};
	goal = {1, 15};
	astar.GetPath(&me1, start, goal, path);
	if (path.size() == 0)
		return false;
	start = {1, 1};
	goal = {15, 1};//{static_cast<uint16_t>(1), static_cast<uint16_t>((3*s/4+1)*2+1)};
	astar.GetPath(&me1, start, goal, path);
	if (path.size() == 0)
		return false;

	start = {15, 1};
	goal = {21, 1};//{static_cast<uint16_t>(1), static_cast<uint16_t>((3*s/4+1)*2+1)};
	if (m3->GetTerrainType(21, 1) != kGround)
		return false;
	astar.GetPath(&me3, start, goal, path);
	if (path.size() == 0)
		return false;
	start = {1, 15};
	goal = {1, 21};//{static_cast<uint16_t>(1), static_cast<uint16_t>((3*s/4+1)*2+1)};
	if (m3->GetTerrainType(1, 21) != kGround)
		return false;
	astar.GetPath(&me3, start, goal, path);
	if (path.size() == 0)
		return false;

	start = {21, 1};
	goal = {21, 15};
	if (m1->GetTerrainType(21, 15) != kGround)
		return false;
	astar.GetPath(&me1, start, goal, path);
	if (path.size() == 0)
		return false;
	start = {1, 21};
	goal = {15, 21};//{static_cast<uint16_t>(1), static_cast<uint16_t>((3*s/4+1)*2+1)};
	if (m1->GetTerrainType(15, 21) != kGround)
		return false;
	astar.GetPath(&me1, start, goal, path);
	if (path.size() == 0)
		return false;

	if (m3->GetTerrainType(15, 21) != kGround)
		return false;
	if (m3->GetTerrainType(21, 15) != kGround)
		return false;
	
	
	m1->Print();
	printf("\n\n");
	m2->Print();
	printf("\n\n");
	m3->Print();

	Graphics::Display disp;
	disp.StartFrame();
	me1.Draw(disp);
	disp.DrawText("SoCS", {0, -0.22}, Colors::lightgray, 0.35);
	disp.DrawText("2018", {0, 0.22}, Colors::lightgray, 0.4);
	disp.EndFrame();
	MakeSVG(disp, "/Users/nathanst/maze01.svg", 375, 375);
	disp.StartFrame();
	me2.Draw(disp);
	disp.EndFrame();
	MakeSVG(disp, "/Users/nathanst/maze02.svg", 375, 375);
	disp.StartFrame();
	me3.Draw(disp);
	disp.EndFrame();
	MakeSVG(disp, "/Users/nathanst/maze03.svg", 375, 375);
	
	m1->Save("/Users/nathanst/maze01.map");
	m2->Save("/Users/nathanst/maze02.map");
	m3->Save("/Users/nathanst/maze03.map");

	m3->SetRectHeight(0, 0, d-1, d-1, 0, kOutOfBounds);
	disp.StartFrame();
	me3.Draw(disp);
	disp.EndFrame();
	MakeSVG(disp, "/Users/nathanst/maze04.svg", 375, 375);

	
	{
		GLdouble px1, py1, t1, rad1;
		m1->GetOpenGLCoord(0, 0, px1, py1, t1, rad1);
		printf("Test: (0, 0) is (%f, %f)\n", px1-rad1, py1-rad1);
		m1->GetOpenGLCoord((int)m1->GetMapWidth()-1, (int)m1->GetMapHeight()-1, px1, py1, t1, rad1);
		printf("Test: (%d, %d) is (%f, %f)\n",
			   m1->GetMapWidth()-1,
			   m1->GetMapHeight()-1,
			   px1+rad1, py1+rad1);

	}
	
	return true;
}

// In practice we should use the ranking (constant time) instead of this
// n^2 algorithm. But, another time.
edge *GetEdge(Graph *g, int x1, int y1, int z1, int x2, int y2, int z2)
{
	for (int t = 0; t < g->GetNumEdges(); t++)
	{
		edge *e = g->GetEdge(t);
		node *n1 = g->GetNode(e->getFrom());
		node *n2 = g->GetNode(e->getTo());
//		printf("--Trying (%ld, %ld, %ld) (%ld, %ld, %ld)\n",
//			   n1->GetLabelL(0), n1->GetLabelL(1), n1->GetLabelL(2),
//			   n2->GetLabelL(0), n2->GetLabelL(1), n2->GetLabelL(2));
		if (n1->GetLabelL(0) == x1 &&
			n1->GetLabelL(1) == y1 &&
			n1->GetLabelL(2) == z1 &&
			n2->GetLabelL(0) == x2 &&
			n2->GetLabelL(1) == y2 &&
			n2->GetLabelL(2) == z2)
		{
//			printf("-Success\n");
			return e;
		}
		if (n2->GetLabelL(0) == x1 &&
			n2->GetLabelL(1) == y1 &&
			n2->GetLabelL(2) == z1 &&
			n1->GetLabelL(0) == x2 &&
			n1->GetLabelL(1) == y2 &&
			n1->GetLabelL(2) == z2)
		{
//			printf("-Success\n");
			return e;
		}
//		printf("---Failure\n");
	}
	return 0;
}

void MazeMazeTree(int d)
{
	return;
	
	Map *m1 = new Map(d, d);
	Map *tmp = new Map(d/2, d/2);

	MakeMaze(tmp, 1, 1, 1);
	tmp->Print();
	printf("\n--\n");

	MakeMaze(tmp, 1, d/2-2, 1);
	tmp->Print();
	printf("\n--\n");

	MakeMaze(tmp, 1, d/2-2, d/2-2);
	tmp->Print();
	printf("\n--\n");
}
