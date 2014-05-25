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
#include <tr1/unordered_map>

uint64_t DoLimitedBFS(FlingBoard b, std::vector<FlingBoard> &path);

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
std::vector<FlingMove> wins;
std::vector<int> counts;
FlingBoard win;
FlingBoard valid;
int pathLoc = 0;

//bool ReadData(std::vector<bool> &data, const char *fName);
//void WriteData(BitVector *data, const char *fName);

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
	text.SetBold(true);
	
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
	InstallKeyboardHandler(MyDisplayHandler, "Reset Board", "Reset board on the screen", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "History", "Move ?? in history", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "History", "Move ?? in history", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Load Preset", "Load Preset State", kAnyModifier, '1', '9');

	InstallKeyboardHandler(SolveAndSaveInstance, "Solve & Save", "Solve and save the current instance", kNoModifier, 's');
	InstallKeyboardHandler(SolveRandomFlingInstance, "Solve Random Instance", "Generates a random instance and uses a BFS to solve it", kNoModifier, '0');
	InstallKeyboardHandler(FindLogicalMoves, "FindLogicalMoves", "In the sequence of moves, find those which are logical", kNoModifier, 'f');
	InstallKeyboardHandler(TestRanking, "Test Ranking Function", "Test ranking function", kNoModifier, 't');
	InstallKeyboardHandler(BuildTables, "Build Exhaustive Tables", "Build Exhaustive Tables", kNoModifier, 'e');
	InstallKeyboardHandler(ReadTables, "Load Tables", "Load tables sizes 2...9", kNoModifier, 'l');
	InstallKeyboardHandler(AnalyzeBoard, "Analyze Board", "Analyze board to generate statistics", kAnyModifier, 'a');
	InstallKeyboardHandler(AnalyzeBoard, "Analyze Board", "Analyze board to generate statistics", kAnyModifier, 'A');
	InstallKeyboardHandler(DeepAnalyzeBoard, "Deep Analyze Board", "Analyze board to generate statistics", kAnyModifier, 'd');
	InstallKeyboardHandler(DeepAnalyzeBoard, "Deep Analyze Board", "Analyze board to generate statistics", kAnyModifier, 'D');
	InstallKeyboardHandler(BFSearch, "Do BFS", "Analyze board with BFS", kAnyModifier, 'b');
	InstallKeyboardHandler(MassAnalysis, "Mass Analysis", "Analyze 100 boards", kNoModifier, 'm');
	InstallKeyboardHandler(CaptureScreen, "Capture Screen", "Capture Screen Shot", kNoModifier, 'c');

	InstallCommandLineHandler(MyCLHandler, "-generate", "-generate n", "Generate a problem with n tiles and run a BFS.");
	InstallCommandLineHandler(MyCLHandler, "-extract", "-extract n", "Extract unique boards at level n.");
	InstallCommandLineHandler(MyCLHandler, "-solve", "-solve n", "Solve all boards up to size n.");
	InstallCommandLineHandler(MyCLHandler, "-bfs", "-bfs theState", "Perform a BFS on theState");
	InstallCommandLineHandler(MyCLHandler, "-analyze", "-analyze theState", "Perform a move analysis on theState");
	InstallCommandLineHandler(MyCLHandler, "-fix", "-fix file", "fix the file format");
	
	
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
	for (int x = 0; x < wins.size(); x++)
		f.OpenGLDraw(win, wins[x]);
	//if (wins.size() > 0)
	{
		f.OpenGLDrawAlternate(valid);
	}
	for (int x = 0; x < counts.size(); x++)
	{
		char val[3] = {0, 0, 0};
		FlingBoard tmp;
		tmp.AddFling(x);
		val[0] = '0'+counts[x]/10;
		val[1] = '0'+counts[x]%10;
		f.GLLabelState(tmp, val);
	}
	
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
		SolveRandomFlingInstance(0, kNoModifier, '0');
	}
	else if (strcmp(argument[0], "-extract") == 0)
	{
		int cnt = atoi(argument[1]);
		ExtractUniqueStates(cnt);
	}
	else if (strcmp(argument[0], "-solve") == 0)
	{
		int cnt = atoi(argument[1]);
		for (int x = 2; x <= cnt; x++)
		{
			BuildTables(0, kNoModifier, 'e');
		}
	}
	else if (strcmp(argument[0], "-bfs") == 0)
	{
		unsigned long long which = strtoull(argument[1], 0, 10);
		std::cout << "Hash: " << which << std::endl;
		//std::cout << "Pieces: " << atoi(argument[1]) << std::endl;
		f.GetStateFromHash(which, b);
		//f.unrankPlayer(which, atoi(argument[1]), b);
		std::cout << b << std::endl;

		BFS<FlingBoard, FlingMove> bfs;
		bfs.GetPath(&f, b, g, path);
		printf("%llu total nodes expanded in pure bfs\n", bfs.GetNodesExpanded());
		uint64_t nodesExpanded = DoLimitedBFS(b, path);
		printf("%llu total nodes expanded in logically limited bfs\n", nodesExpanded);
	}
	else if (strcmp(argument[0], "-analyze") == 0)
	{
		ReadTables(0, kNoModifier, 'l');
		while (!feof(stdin))
		{
			uint64_t which;
			scanf("%llu", &which);
			//unsigned long long which = strtoull(argument[1], 0, 10);
			std::cout << "Hash: " << which << std::endl;
			//std::cout << "Pieces: " << atoi(argument[1]) << std::endl;
			f.GetStateFromHash(which, b);
			//f.unrankPlayer(which, atoi(argument[1]), b);
			std::cout << b << std::endl;
			
			AnalyzeBoard(0, kNoModifier, 'a');
			AnalyzeBoard(0, kNoModifier, 'A');
		}
	}
	else if (strcmp(argument[0], "-fix") == 0)
	{
		FILE *file = fopen(argument[1], "r");
		int next;
		int cnt = 0;
		do {
			next = fgetc(file);
			if (next == '.' || next == 'o')
			{
				if (next == 'o')
					b.SetPiece(cnt);
				else
					b.ClearPiece(cnt);
				cnt++;
			}
			if (cnt == 56)
			{
				std::cout << b << std::endl << std::endl;
				std::cout << "Hash: " << f.rankPlayer(b) << " " << b.locs.size() << std::endl;
				cnt = 0;
			}
		} while (next != EOF);
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
			if (b.locs.size() == 0)
			{
				counts.resize(0);
				for (int x = 0; x < 10; x++)
				{
					int x1 = random()%b.width;
					int y1 = random()%b.height;
					if (!b.HasPiece(x1, y1))
						b.AddFling(x1, y1);
				}
			}
			else {
				b.Reset();
				g.Reset();
				valid.Reset();
				win.Reset();
				counts.resize(0);
				wins.resize(0);
			}
			break;
		case '1':
			f.GetStateFromHash(45179843855354896ull, b);
			break;
		case '2':
			f.GetStateFromHash(6786211670857232ull, b);
			break;
		case '3':
			f.GetStateFromHash(11288156058953760ull, b);
			break;
		case '4':
			f.GetStateFromHash(27303083721900308ull, b);
			break;
		case '5':
			f.GetStateFromHash(18157695840586785ull, b);
			break;
		case '6':
			f.GetStateFromHash(37154696925872128ull, b);
			break;
		default:
			break;
	}
}

void GetReducedMoves(FlingBoard s1, FlingBoard s2, std::vector<FlingMove> &moves)
{
	std::vector<FlingMove> m1, m2;
	FlingMove between;
	between = f.GetAction(s1, s2);
	
	f.GetActions(s1, m1);
	f.GetActions(s2, m2);
	
	std::vector<FlingMove> commonMoves;
	
	moves.resize(0);
	// find common moves
	for (int x = 0; x < m1.size(); x++)
	{
		for (int y = 0; y < m2.size(); y++)
		{
			if (m1[x] == m2[y])
			{
				commonMoves.push_back(m1[x]);
				//std::cout << "Common moves include " << m1[x] << std::endl;
			}
		}
	}
	
	FlingBoard t1, t2;
	moves.resize(0);
	// apply common moves (besides between) to s1
	for (int x = 0; x < commonMoves.size(); x++)
	{
		f.GetNextState(s1, commonMoves[x], t1);
		if (f.LegalMove(t1, between))
		{
			f.ApplyAction(t1, between);
			if (f.LegalMove(s2, commonMoves[x]))
			{
				f.GetNextState(s2, commonMoves[x], t2);
				if (t1.board != t2.board)
				{
//					std::cout << commonMoves[x] << " and " << between <<
//					" in different orders result in different states" << std::endl;
					moves.push_back(commonMoves[x]);
					//					std::cout << t1 << "\n-and-\n"<< t2 << std::endl;
				}
			}
			else {
				moves.push_back(commonMoves[x]);
			}
		}
		else {
			moves.push_back(commonMoves[x]);
//			std::cout << commonMoves[x] << " and " << between <<
//			" can't be interchanged" << std::endl;
		}
	}
	// add non-common moves
	for (int x = 0; x < m2.size(); x++)
	{
		bool found = false;
		for (int y = 0; y < commonMoves.size(); y++)
		{
			if (m2[x] == commonMoves[y])
			{
				found = true;
				break;
			}
		}
		if (!found)
		{
			moves.push_back(m2[x]);
		}
	}
}

void FindLogicalMoves(unsigned long , tKeyboardModifier , char)
{
	printf("%d\n", pathLoc);
	if (pathLoc+1 >= path.size())
		return;
	FlingBoard s1, s2;
	s1 = path[pathLoc+1];
	s2 = path[pathLoc];
	// find moves taken between s1 and s2
	win = s2;
	GetReducedMoves(s1, s2, wins);
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
#include "BitVector.h"
std::vector<BitVector*> table;
std::vector<BitVector*> unique;
//std::vector<std::vector<BitVector*> > table;
int currSize = 2;
pthread_mutex_t writeLock = PTHREAD_MUTEX_INITIALIZER;
int64_t solvable;
int64_t uniqueSolvable;

void *ThreadedWorker(void *arg)
{
	int id = (long)arg;
	int64_t solved = 0;
	int64_t uniqueSolved = 0;
	FlingBoard currState, tmp;
	std::vector<FlingBoard> succ;
	std::vector<FlingMove> acts;

	std::vector<int64_t> buffer;
	std::vector<int64_t> uniqueBuffer;
	buffer.reserve(4*1024+100);
	uniqueBuffer.reserve(4*1024+100);
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
				uniqueBuffer.push_back(val);
				//table[currSize][val] = true;
				solved++;
				uniqueSolved++;
			}
		}
		else {
			int cnt = 0;
			int uniqueCnt = 0;
			for (int x = 0; x < acts.size(); x++)
			{
				f.GetNextState(currState, acts[x], tmp);
				uint64_t rank = f.rankPlayer(tmp);
				if (table[tmp.locs.size()]->Get(rank))
				{
					cnt++;
				}
				if (unique[tmp.locs.size()]->Get(rank))
				{
					uniqueCnt++;
				}
			}
			if (cnt > 0) {
				buffer.push_back(val);
				solved++;
			}
			if (cnt == 1 && uniqueCnt == 1)
			{
				uniqueBuffer.push_back(val);
				uniqueSolved++;
//				pthread_mutex_lock (&writeLock);
//				std::cout << currState << std::endl;
//				std::cout << acts.size() << " moves; ";
//				std::cout << cnt << " lead to solvable states; ";
//				std::cout << uniqueCnt << " of these are unique\n";
//				pthread_mutex_unlock(&writeLock);
			}
		}
		// flush buffer
		if (buffer.size() > 4*1024)
		{
			pthread_mutex_lock (&writeLock);
			while (buffer.size() > 0)
			{
				table[currSize]->Set(buffer.back(), true);// = true;
				buffer.pop_back();
			}
			while (uniqueBuffer.size() > 0)
			{
				unique[currSize]->Set(uniqueBuffer.back(), true);// = true;
				uniqueBuffer.pop_back();
			}
			pthread_mutex_unlock(&writeLock);
		}
	}

	pthread_mutex_lock (&writeLock);
	while (buffer.size() > 0)
	{
		table[currSize]->Set(buffer.back(), true);
		buffer.pop_back();
	}
	while (uniqueBuffer.size() > 0)
	{
		unique[currSize]->Set(uniqueBuffer.back(), true);// = true;
		if (uniqueBuffer.size() == 1)
		{
			f.unrankPlayer(uniqueBuffer.back(), currSize, b);
		}
		uniqueBuffer.pop_back();
	}
	solvable += solved;
	uniqueSolvable += uniqueSolved;
	pthread_mutex_unlock(&writeLock);

	pthread_exit(NULL);
}

void ExtractUniqueStates(int depth)
{
	char fname[255];
	sprintf(fname, "/Users/nathanst/hog2/apps/fling/fling-unique-%d.dat", depth);
	printf("Reading from '%s'\n", fname);
	BitVector *b = new BitVector(f.getMaxSinglePlayerRank(56, depth), fname, false);
	uint64_t maxVal = f.getMaxSinglePlayerRank(56, depth);
	FlingBoard board;
	for (uint64_t t = 0; t < maxVal; t++)
	{
		if (b->Get(t))
		{
			f.unrankPlayer(t, depth, board);
			bool ok = false;
			for (int x = 0; x < board.width; x++)
			{
				if (board.HasPiece(x, 0))
				{
					ok = true;
					break;
				}
			}
			if (ok == false)
				continue;
			ok = false;
			for (int y = 0; y < board.height; y++)
			{
				if (board.HasPiece(0, y))
				{
					ok = true;
					break;
				}
			}
			if (ok == false)
				continue;
			printf("Rank: %llu\n", t);
			printf("Board: %llu\n", board.board);

			//if (depth < 5)
			{
				BFS<FlingBoard, FlingMove> bfs;
				bfs.GetPath(&f, board, g, path);
				//printf("%llu total nodes expanded in pure bfs\n", bfs.GetNodesExpanded());
				uint64_t nodesExpanded = DoLimitedBFS(board, path);
				//printf("%llu total nodes expanded in logically limited bfs\n", nodesExpanded);
				printf("Board: %llu Full: %llu Reduced: %llu Ratio: %1.4f\n", board.board, bfs.GetNodesExpanded(), nodesExpanded, float(bfs.GetNodesExpanded())/float(nodesExpanded));
//				std::cout << board << std::endl;
			}
		}
	}
}

void BuildTables(unsigned long , tKeyboardModifier, char)
{
	table.resize(currSize+1);
	unique.resize(currSize+1);
	std::cout << "Starting work on board with " << currSize << " pieces. ";
	std::cout << f.getMaxSinglePlayerRank(56, currSize) << " entries." << std::endl;

	char fname[255];
	sprintf(fname, "/Users/nathanst/hog2/apps/fling/fling-%d.dat", currSize);
	table[currSize] = new BitVector(f.getMaxSinglePlayerRank(56, currSize), fname, true);
	sprintf(fname, "/Users/nathanst/hog2/apps/fling/fling-unique-%d.dat", currSize);
	unique[currSize] = new BitVector(f.getMaxSinglePlayerRank(56, currSize), fname, true);
	//table[currSize].resize(f.getMaxSinglePlayerRank(56, currSize));

	solvable = 0;
	uniqueSolvable = 0;
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
	printf("%lld are solvable (%3.1f%%)\n", solvable, 100*perc);

	perc = uniqueSolvable;
	perc /= (double)f.getMaxSinglePlayerRank(56, currSize);
	printf("%lld are uniquely solvable (%3.1f%%)\n%3.2f sec elapsed\n", uniqueSolvable, 100*perc, t.EndTimer());
	
//	char fname[255];
//	sprintf(fname, "/Users/nathanst/fling-%d.dat", currSize);
	//t.StartTimer();
	//WriteData(table[currSize], fname);
	//printf("%3.2f sec writing data to disk\n", t.EndTimer());
	//std::cout << solvable << " are solvable " << std::setprecision(3) << perc << std::endl;
	//std::cout << t.EndTimer() << " elapsed" << std::endl;
	currSize++;
}

void ReadTables(unsigned long , tKeyboardModifier, char)
{
	table.resize(0); // clear old data
//	table.resize(9);

	Timer t;
	for (int x = 2; x <= 10; x++)
	{
		table.resize(x+1);
		std::cout << "Loading board with " << x << " pieces. ";
		std::cout << f.getMaxSinglePlayerRank(56, x) << " entries." << std::endl;
		//table[x] = new BitVector(f.getMaxSinglePlayerRank(56, x));
		//table[x].resize(f.getMaxSinglePlayerRank(56, x));

		char fname[255];
		sprintf(fname, "/Users/nathanst/hog2/apps/fling/fling-%d.dat", x);
		t.StartTimer();
		table[x] = new BitVector(f.getMaxSinglePlayerRank(56, x), fname, false);
//		if (ReadData(table[x], fname) != true)
//		{
//			printf("Error reading data\n");
//		}
		printf("%3.2f sec reading data from disk\n", t.EndTimer());
	}
}

std::tr1::unordered_map<uint64_t,bool> visitedStates;
bool useTable = true;

bool IsSolvable(FlingBoard &theState)
{
	if (theState.locs.size() == 1)
		return true;
	if (useTable && theState.locs.size() < table.size())
	{
		// lookup
		//return (table[theState.locs.size()][f.rankPlayer(theState)]);
		if (table[theState.locs.size()] != 0)
			return (table[theState.locs.size()]->Get(f.rankPlayer(theState)));
		else {
			printf("Table size %d is null\n", theState.locs.size());
			return true;
		}
	}
	// in hash table
	if (visitedStates.find(f.rankPlayer(theState)) != visitedStates.end())
	{
		return visitedStates[f.rankPlayer(theState)];
	}
	std::vector<FlingMove> moves;
	f.GetActions(theState, moves);
	FlingBoard cpy;
	for (unsigned int x = 0; x < moves.size(); x++)
	{
		f.GetNextState(theState, moves[x], cpy);
		if (IsSolvable(cpy))
		{
			visitedStates[f.rankPlayer(theState)] = true;
			return true;
		}
	}
	visitedStates[f.rankPlayer(theState)] = false;
	return false;
}

uint64_t RecursiveBFS(FlingBoard from, std::vector<FlingBoard> &thePath, Fling *env)
{
	typedef __gnu_cxx::hash_map<uint64_t, uint64_t, Hash64> BFSClosedList;
	std::deque<FlingBoard> mOpen;
	std::deque<int> depth;
	BFSClosedList mClosed; // store parent id!
	thePath.resize(0);
	FlingBoard parentState;
	std::vector<FlingMove> moves;
	
	uint64_t nodesExpanded = 0;
	uint64_t weightedNodes = 0;
	int weight = from.locs.size();
	
	mOpen.clear();
	mClosed.clear();
	depth.clear();
	
	depth.push_back(0);
	mOpen.push_back(from);
	mClosed[env->GetStateHash(from)] = env->GetStateHash(from);
	//	printf("Setting parent of %llu to be %llu\n", env->GetStateHash(from),
	//		   env->GetStateHash(from));
	
	int currDepth = 0;
	uint64_t lastNodes = 0, lastIter = 0;
	FlingBoard s, tmp;
	while (mOpen.size() > 0)
	{
		assert(mOpen.size() == depth.size());
		s = mOpen.front();
		mOpen.pop_front();
		if (depth.front() != currDepth)
		{
//			printf("%d tot %llu inc %lld b %.2f\n", currDepth, nodesExpanded, nodesExpanded-lastNodes, (double)(nodesExpanded-lastNodes)/lastIter);
			lastIter = nodesExpanded-lastNodes;
			lastNodes = nodesExpanded;
		}
		currDepth = depth.front();
		depth.pop_front();
		
		nodesExpanded++;
		//weightedNodes += (weight-currDepth);
		if (mClosed.find(env->GetStateHash(s)) != mClosed.end())
		{
			moves.resize(0);
			env->GetStateFromHash(mClosed[env->GetStateHash(s)], parentState);
			GetReducedMoves(parentState, s, moves);
			for (int x = 0; x < moves.size(); x++)
			{
				env->GetNextState(s, moves[x], tmp);
				thePath.push_back(tmp);
			}
		}
		else {
			env->GetSuccessors(s, thePath);
		}
		
		for (unsigned int x = 0; x < thePath.size(); x++)
		{
			if (mClosed.find(env->GetStateHash(thePath[x])) == mClosed.end())
			{
				mOpen.push_back(thePath[x]);
				depth.push_back(currDepth+1);
				//				printf("Setting parent of %llu to be %llu\n", env->GetStateHash(thePath[x]),
				//					   env->GetStateHash(s));
				mClosed[env->GetStateHash(thePath[x])] = env->GetStateHash(s);
			}
		}
	}
//	printf("%d tot %llu inc %lld b %.2f\n", currDepth, nodesExpanded, nodesExpanded-lastNodes, (double)(nodesExpanded-lastNodes)/lastIter);
//	std::cout << "Final state:\n" << s << std::endl;
	
	thePath.resize(0);
	uint64_t parent, lastParent;
	//	std::cout << s << std::endl;
	do {
		lastParent = env->GetStateHash(s);
		thePath.push_back(s);
		parent = mClosed[env->GetStateHash(s)];
		env->GetStateFromHash(parent, s);
		//		std::cout << s << std::endl;
	} while (parent != lastParent);
//	printf("Final depth: %d, Nodes Expanded %llu, Exponential BF: %f\n", currDepth, nodesExpanded, pow(nodesExpanded, (double)1.0/currDepth));
	return nodesExpanded;
//	return weightedNodes;
}

uint64_t DoLimitedBFS(FlingBoard b, std::vector<FlingBoard> &path)
{
	return RecursiveBFS(b, path, &f);
}

void BFSearch(unsigned long windowID, tKeyboardModifier mod, char key)
{
	text.Clear();
	Timer t;
	t.StartTimer();
	uint64_t nodesExpanded;
	// do limited search
	if (mod == kShiftDown || key == 'B')
	{
		text.AddLine("Limited search");
		nodesExpanded = DoLimitedBFS(b, path);
	}
	else {
		text.AddLine("Full search");
		BFS<FlingBoard, FlingMove> bfs;
		bfs.GetPath(&f, b, g, path);
		pathLoc = path.size()-1;
		nodesExpanded = bfs.GetNodesExpanded();
	}
	t.EndTimer();
	
	char line[255];
	sprintf(line, "%llu total nodes", nodesExpanded);
	text.AddLine(line);
	sprintf(line, "Problem %s solvable (%4.3f elapsed)", (path.size() == b.locs.size())?"is":"is not", t.GetElapsedTime());
	text.AddLine(line);
}

void MassAnalysis(unsigned long w, tKeyboardModifier mod, char c)
{
	for (int x = 0; x < 100; x++)
	{
		// generate random board with N pieces
		b.Reset();
		while (b.locs.size() < 16)
		{
			int next = random()%56;
			if (!b.HasPiece(next))
				b.AddFling(next);
		}
		AnalyzeBoard(w, kNoModifier, 'a');
		AnalyzeBoard(w, kNoModifier, 'A');
	}
}

void CaptureScreen(unsigned long windowID, tKeyboardModifier mod, char c)
{
	static int cnt = 0;
	char fname[255];
	sprintf(fname, "/Users/nathanst/Movies/FLING-%d%d%d", (cnt/100)%10, (cnt/10)%10, cnt%10);
	SaveScreenshot(windowID, fname);
	printf("Saved %s\n", fname);
	cnt++;
}

// 1. count size of tree
// (store full and logically reduced size)

void GetWinningMoves(const FlingBoard &b, std::vector<FlingMove> &w)
{
	w.resize(0);
	std::vector<FlingMove> moves;
	f.GetActions(b, moves);
	FlingBoard cpy;
	//	std::cout << " Testing:\n" << b << std::endl;
	for (unsigned int x = 0; x < moves.size(); x++)
	{
		f.GetNextState(b, moves[x], cpy);
		//std::cout << "Move " << moves[x] << " leads to \n" << cpy << std::endl;
		if (IsSolvable(cpy))
		{
			w.push_back(moves[x]);
			//			std::cout << moves[x] << " is a solving move\n";
		}
	}
}

void AnalyzeBoard(unsigned long , tKeyboardModifier mod, char c)
{
	counts.resize(0);
	if (mod == kShiftDown || c == 'A')
	{
		// don't use db
		useTable = false;
		printf("NOT using pre-computed table\n");
	}
	else {
		useTable = true;
		printf("USING pre-computed table\n");
	}
	visitedStates.clear();
	// check every square to see if placing/removing a piece from there would make
	// the game solvable or not
	Timer t;
	t.StartTimer();
	valid.Reset();
	for (int x = 0; x < b.width*b.height; x++)
	{
		if (b.HasPiece(x))
		{
			b.RemoveFling(x);
			if (IsSolvable(b))
				valid.AddFling(x);
			b.AddFling(x);
		}
		else {
			b.AddFling(x);
			if (IsSolvable(b))
				valid.AddFling(x);
			b.RemoveFling(x);
		}
	}
//	std::cout << "Locs where we can add/remove pieces and still be solvable:" << std::endl;
//	std::cout << valid << std::endl;
	printf("%3.2f sec elapsed for placement analysis\n", t.EndTimer());
	// check to see what legal moves will solve the game
	// put them in wins array to be drawn on the screen
	t.StartTimer();
	GetWinningMoves(b, wins);
	win = b;
	printf("%3.2f sec elapsed for move analysis\n", t.EndTimer());
}

void DeepAnalyzeBoard(unsigned long , tKeyboardModifier mod, char c)
{
	if (mod == kShiftDown || c == 'D')
	{
		// don't use db
		useTable = false;
		printf("NOT using pre-computed table\n");
	}
	else {
		useTable = true;
		printf("USING pre-computed table\n");
	}
	visitedStates.clear();
	// check every square to see if placing/removing a piece from there would make
	// the game solvable or not
	Timer t;
	t.StartTimer();
	valid.Reset();
	std::vector<FlingMove> winCount;
	counts.resize(0);
	counts.resize(b.width*b.height);
	for (int x = 0; x < b.width*b.height; x++)
	{
		if (b.HasPiece(x))
		{
			b.RemoveFling(x);

			GetWinningMoves(b, winCount);
			if (winCount.size() > 0)
			{
				//if (IsSolvable(b))
				valid.AddFling(x);
				counts[x] = winCount.size();
			}
			b.AddFling(x);
		}
		else {
			b.AddFling(x);
//			if (IsSolvable(b))
//				valid.AddFling(x);
			GetWinningMoves(b, winCount);
			if (winCount.size() > 0)
			{
				//if (IsSolvable(b))
				valid.AddFling(x);
				counts[x] = winCount.size();
			}

			b.RemoveFling(x);
		}
	}
	//	std::cout << "Locs where we can add/remove pieces and still be solvable:" << std::endl;
	//	std::cout << valid << std::endl;
	printf("%3.2f sec elapsed for placement analysis\n", t.EndTimer());
	// check to see what legal moves will solve the game
	// put them in wins array to be drawn on the screen
	t.StartTimer();
	win = b;
	GetWinningMoves(b, wins);
	printf("%3.2f sec elapsed for move analysis\n", t.EndTimer());
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
//	if (event != kMouseDown)
//		return false;
	
	if (button == kRightButton)
	{
		static int lastx, lasty;
		static bool active = false;
		int currx, curry;
		switch (event)
		{
			case kMouseDown:
				if (f.GetXYFromPoint(b, loc, lastx, lasty))
				{
					if (b.HasPiece(lastx, lasty))
					{
						active = true;
						printf("Trying to move (%d, %d)\n", lastx, lasty);
					}
				}
				break;
			case kMouseUp:
				printf("Handling mouse up!\n");
				if (f.GetXYFromPoint(b, loc, currx, curry) && active)
				{
					active = false;
					std::vector<FlingMove> acts;
					f.GetActions(b, acts);
					tFlingDir d = kLeft;
					if (currx == lastx && curry < lasty)
					{
						d = kUp;
						printf("Trying to move Up\n");
					}
					if (currx == lastx && curry > lasty)
					{
						d = kDown;
						printf("Trying to move Down\n");
					}
					if (currx < lastx && curry == lasty)
					{
						d = kLeft;
						printf("Trying to move Left\n");
					}
					if (currx > lastx && curry == lasty)
					{
						d = kRight;
						printf("Trying to move Right\n");
					}
					for (unsigned int x = 0; x < acts.size(); x++)
					{
						if (acts[x].dir == d && b.locs[acts[x].startLoc] == lasty*b.width+lastx)
						{
							f.ApplyAction(b, acts[x]);
							break;
						}
					}
				}
				break;
			default: break;
				//b.Reset();
		}
		return true;
	}
	else {
		if (event != kMouseDown)
			return false;

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
		}
		return true;
	}
	return false;
}


//void WriteData(BitVector *data, const char *fName)
//{
//	FILE *f = fopen(fName, "w+");
//	fprintf(f, "%llu\n", (uint64_t)data.size());
//	
//	uint8_t next = 0;
//	for (uint64_t x = 0; x < data.size(); x++)
//	{
//		next = (next<<1)|(data[x]?1:0);
//		if (7 == x%8)
//		{
//			fwrite(&next, sizeof(uint8_t), 1, f);
//			next = 0;
//		}
//	}
//	fwrite(&next, sizeof(uint8_t), 1, f);
//	fclose(f);
//}

//bool ReadData(std::vector<bool> &data, const char *fName)
//{
//	FILE *f = fopen(fName, "r+");
//	if (f == 0)
//	{
//		printf("Unable to open file: '%s' aborting\n", fName);
//		//exit(0);
//		return false;
//	}
//	uint64_t dataSize;
//	if (fscanf(f, "%llu\n", &dataSize) == 0)
//	{
//		printf("Error reading array size from file\n");
//		fclose(f);
//		return false;
//	}
//	printf("%llu entries in file\n", dataSize);
//	data.resize(dataSize);
//	
//	uint8_t next = 0;
//	uint64_t x;
//	for (x = 8; x < data.size(); x+=8)
//	{
//		fread(&next, sizeof(uint8_t), 1, f);
//		data[x-8+0] = (next>>7)&0x1;
//		data[x-8+1] = (next>>6)&0x1;
//		data[x-8+2] = (next>>5)&0x1;
//		data[x-8+3] = (next>>4)&0x1;
//		data[x-8+4] = (next>>3)&0x1;
//		data[x-8+5] = (next>>2)&0x1;
//		data[x-8+6] = (next>>1)&0x1;
//		data[x-8+7] = (next>>0)&0x1;
//	}
//	fread(&next, sizeof(uint8_t), 1, f);
//	for (uint64_t y = 0; x-8+y < data.size(); y++)
//		data[x-8+y] = (next>>(7-y))&0x1;
//	fclose(f);
//
//	return true;
//}
