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
#include "Fling.h"
#include "BFS.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include "TextOverlay.h"
#include <pthread.h>

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv);
}


/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
TextOverlay text;
Fling f;
FlingBoard b, g;
FlingMove m;
std::vector<FlingBoard> path;
int pathLoc = 0;

void ReadData(std::vector<bool> &data, const char *fName);
void WriteData(std::vector<bool> &data, const char *fName);

void CreateSimulation(int id)
{
	for (int x = 0; x < 10; x++)
	{
		int x1 = random()%b.width;
		int y1 = random()%b.height;
		if (!b.HasPiece(x1, y1))
			b.AddFling(x1, y1);
	}
	std::cout << b << std::endl;
	uint64_t h1 = f.GetStateHash(b);
	f.GetStateFromHash(h1, g);
	uint64_t h2 = f.GetStateHash(g);
	if (h1 != h2)
	{
		std::cout << b << std::endl;
		exit(0);
	}
	if (!(b == g))
	{
		std::cout << b << std::endl;
		std::cout << g << std::endl;
		exit(0);
	}
	
//	std::vector<FlingBoard> path;
//	BFS<FlingBoard, FlingMove> bfs;
//	bfs.GetPath(&f, b, g, path);
//	IDAStar<FlingBoard, FlingMove> ida;
//	ida.GetPath(&f, b, g, path);
//	std::cout << ida.GetNodesExpanded() << " nodes expanded" << std::endl;
//	for (int x = 0; x < path.size(); x++)
//	{
//		std::cout << x << std::endl << path[x] << std::endl;
//	}
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

	InstallKeyboardHandler(MyDisplayHandler, "Reset Board", "Reset board on the screen", kAnyModifier, 'r');

	InstallKeyboardHandler(SolveAndSaveInstance, "Solve & Save", "Solve and save the current instance", kNoModifier, 's');
	InstallKeyboardHandler(SolveRandomFlingInstance, "Solve Random Instance", "Generates a random instance and uses a BFS to solve it", kNoModifier, 'a');
	InstallKeyboardHandler(TestRanking, "Test Ranking Function", "Test ranking function", kNoModifier, 't');
	InstallKeyboardHandler(BuildTables, "Build Exhaustive Tables", "Build Exhaustive Tables", kNoModifier, 'b');

	InstallCommandLineHandler(MyCLHandler, "-generate", "-generate n", "Generate a problem with n tiles and run a BFS.");
	InstallCommandLineHandler(MyCLHandler, "-solve", "-solve n", "Solve all boards up to size n.");
	
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

//ida.GetPath(&f, b, g, path);

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	f.OpenGLDraw(b);
	text.OpenGLDraw(windowID);
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
	{
		printf("Insufficient arguments\n");
		return 0;
	}
	//strncpy(gDefaultMap, argument[1], 1024);
	if (strcmp(argument[0], "-generate") == 0)
	{
		SolveRandomFlingInstance(0, kNoModifier, 'a');
	}
	else if (strcmp(argument[0], "-solve") == 0)
	{
		int cnt = atoi(argument[1]);
		for (int x = 2; x <= cnt; x++)
		{
			BuildTables(0, kNoModifier, 'b');
		}
	}
	exit(0);
	return 2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '[':
			pathLoc--;
			if (pathLoc < 0)
				pathLoc = 0;
			if (pathLoc < path.size())
				b = path[pathLoc];
			break;
		case ']':
			if (path.size() == 0)
				break;
			pathLoc++;
			if (pathLoc >= path.size())
				pathLoc = path.size()-1;
			if (pathLoc < path.size())
				b = path[pathLoc];
			break;
		case 'r':
			b.Reset();
			break;
		case '\t':
		{
			if (mod != kShiftDown)
				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
			else
			{
				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
			}
			break;
		}
			break;
		default:
			//if (unitSim)
			//	unitSim->GetEnvironment()->GetMapAbstraction()->ToggleDrawAbstraction(((mod == kControlDown)?10:0)+(key-'0'));
			break;
	}
}

void SolveRandomFlingInstance(unsigned long windowID, tKeyboardModifier , char)
{
	b.Reset();
	for (int x = 0; x < 10; x++)
	{
		int x1 = random()%b.width;
		int y1 = random()%b.height;
		if (!b.HasPiece(x1, y1))
			b.AddFling(x1, y1);
	}
	std::cout << b << std::endl;
	
	std::vector<FlingBoard> path;
	BFS<FlingBoard, FlingMove> bfs;
	bfs.GetPath(&f, b, g, path);
}

void SolveAndSaveInstance(unsigned long , tKeyboardModifier , char)
{
	std::ofstream myfile;
	myfile.open("/Users/nathanst/boards.txt", std::ios::app | std::ios::out );
	
	BFS<FlingBoard, FlingMove> bfs;
	bfs.GetPath(&f, b, g, path);

	myfile << "Hash: " << f.GetStateHash(b) << std::endl;
	myfile << b << std::endl;
	myfile << "\n----\n\n";
	myfile.close();
	
	for (unsigned int x = 0; x < path.size(); x++)
	{
		std::cout << "(" << x << ")" << std::endl;
		std::cout << path[x] << std::endl << std::endl;
	}
	
	//b.Reset();
	pathLoc = path.size()-1;
}

const int THREADS = 4;

std::vector<std::vector<bool> > table;
int currSize = 2;
pthread_mutex_t writeLock = PTHREAD_MUTEX_INITIALIZER;
int solvable;

void *ThreadedWorker(void *arg)
{
	int id = (long)arg;
	int64_t solved = 0;
	FlingBoard currState, tmp;
	std::vector<FlingBoard> succ;
	std::vector<FlingMove> acts;

	std::vector<int64_t> buffer;
	buffer.reserve(4*1024+100);
	for (int64_t val = id; val < f.getMaxSinglePlayerRank(56, currSize); val+=THREADS)
	{
		f.unrankPlayer(val, currSize, currState);
		//f.GetSuccessors(currState, succ);
		f.GetActions(currState, acts);
		if (currSize == 2)
		{
			if (acts.size() > 0)
			{
				buffer.push_back(val);
				//table[currSize][val] = true;
				solved++;
			}
		}
		else {
			for (int x = 0; x < acts.size(); x++)
			{
				f.GetNextState(currState, acts[x], tmp);
				if (table[tmp.locs.size()][f.rankPlayer(tmp)])
				{
					buffer.push_back(val);
					//table[currSize][val] = true;
					solved++;
					break;
				}
			}
		}
		// flush buffer
		if (buffer.size() > 4*1024)
		{
			pthread_mutex_lock (&writeLock);
			while (buffer.size() > 0)
			{
				table[currSize][buffer.back()] = true;
				buffer.pop_back();
			}
			pthread_mutex_unlock(&writeLock);
		}
	}

	pthread_mutex_lock (&writeLock);
	while (buffer.size() > 0)
	{
		table[currSize][buffer.back()] = true;
		buffer.pop_back();
	}
	solvable += solved;
	pthread_mutex_unlock(&writeLock);

	pthread_exit(NULL);
}


void BuildTables(unsigned long , tKeyboardModifier, char)
{
	table.resize(currSize+1);
	std::cout << "Starting work on board with " << currSize << " pieces. ";
	std::cout << f.getMaxSinglePlayerRank(56, currSize) << " entries." << std::endl;
	table[currSize].resize(f.getMaxSinglePlayerRank(56, currSize));

	solvable = 0;

	Timer t;
	t.StartTimer();
	std::vector<pthread_t> threads(THREADS);
	for (int x = 0; x < THREADS; x++)
	{
		pthread_create(&threads[x], NULL, ThreadedWorker, (void *)x);
	}
	for (int x = 0; x < THREADS; x++)
	{
		int result = pthread_join(threads[x], NULL);
		if (result != 0)
		{
			printf("Unknown error joining with thread %d\n", x);
		}
	}
	double perc = solvable;
	perc /= (double)f.getMaxSinglePlayerRank(56, currSize);
	printf("%lld are solvable (%3.1f%%)\n%3.2f s elapsed\n", solvable, 100*perc, t.EndTimer());
	char fname[255];
	sprintf(fname, "/Users/nathanst/fling-%d.dat", currSize);
	t.StartTimer();
	WriteData(table[currSize], fname);
	printf("%3.2f sec writing data to disk\n", t.EndTimer());
	//std::cout << solvable << " are solvable " << std::setprecision(3) << perc << std::endl;
	//std::cout << t.EndTimer() << " elapsed" << std::endl;
	currSize++;
}

void TestRanking(unsigned long , tKeyboardModifier, char)
{
	static int64_t currValue = 0;
	f.unrankPlayer(currValue, 10, b);
	int64_t res = f.rankPlayer(b);
	assert(res == currValue);
	printf("Ranking: %lld; unranked: %lld\n", currValue, res);

	std::vector<FlingBoard> succ;
	f.GetSuccessors(b, succ);
	FlingBoard tmp;
	for (unsigned int x = 0; x < succ.size(); x++)
	{
		assert(succ[x].locs.size() == 9);
	}
	for (unsigned int x = 0; x < succ.size(); x++)
	{
		f.unrankPlayer(f.rankPlayer(succ[x]), succ[x].locs.size(), tmp);
		assert(tmp == succ[x]);
	}
	
	currValue+=92837537;
	if (currValue > f.getMaxSinglePlayerRank(56, 10))
		currValue = 0;
}

bool MyClickHandler(unsigned long , int, int, point3d loc, tButtonType button, tMouseEventType event)
{
	if (event != kMouseDown)
		return false;
	
	if (button == kRightButton)
	{
		b.Reset();
		return true;
	}
	else {
		int x, y;
		if (f.GetXYFromPoint(b, loc, x, y))
		{
			if (b.HasPiece(x, y))
			{
				b.RemoveFling(x, y);
			}
			else {
				b.AddFling(x, y);
			}
			BFS<FlingBoard, FlingMove> bfs;
			bfs.GetPath(&f, b, g, path);
			pathLoc = path.size()-1;

			text.Clear();
			char line[255];
			sprintf(line, "%llu total nodes", bfs.GetNodesExpanded());
			text.AddLine(line);
			sprintf(line, "Problem %s solvable", (path.size() == b.locs.size())?"is":"is not");
			text.AddLine(line);
		}
		return true;
	}
	return false;
}


void WriteData(std::vector<bool> &data, const char *fName)
{
	FILE *f = fopen(fName, "w+");
	fprintf(f, "%llu\n", (uint64_t)data.size());
	
	uint8_t next = 0;
	for (uint64_t x = 0; x < data.size(); x++)
	{
		next = (next<<1)|(data[x]?1:0);
		if (7 == x%8)
		{
			fwrite(&next, sizeof(uint8_t), 1, f);
			next = 0;
		}
	}
	fwrite(&next, sizeof(uint8_t), 1, f);
	fclose(f);
}

void ReadData(std::vector<bool> &data, const char *fName)
{
	FILE *f = fopen(fName, "r+");
	if (f == 0)
	{
		printf("Unable to open file: '%s' aborting\n", fName);
		exit(0);
	}
	uint64_t dataSize;
	fscanf(f, "%llu\n", &dataSize);
	data.resize(dataSize);
	
	uint8_t next = 0;
	uint64_t x;
	for (x = 8; x < data.size(); x+=8)
	{
		fread(&next, sizeof(uint8_t), 1, f);
		data[x-8+0] = (next>>7)&0x1;
		data[x-8+1] = (next>>6)&0x1;
		data[x-8+2] = (next>>5)&0x1;
		data[x-8+3] = (next>>4)&0x1;
		data[x-8+4] = (next>>3)&0x1;
		data[x-8+5] = (next>>2)&0x1;
		data[x-8+6] = (next>>1)&0x1;
		data[x-8+7] = (next>>0)&0x1;
	}
	fread(&next, sizeof(uint8_t), 1, f);
	for (uint64_t y = 0; x-8+y < data.size(); y++)
		data[x-8+y] = (next>>(7-y))&0x1;
	fclose(f);
}
