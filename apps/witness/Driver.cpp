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
 */

#include <cstring>
#include "Common.h"
#include "Driver.h"
#include "Timer.h"
#include <deque>
#include "Witness.h"
#include "Combinations.h"
#include "SVGUtil.h"

#include <sys/stat.h>
bool fileExists(const char *name)
{
	struct stat buffer;
	return (stat(name, &buffer) == 0);
}

bool recording = false;

// 2x2 + 5 (interesting)
// 2x5 + 5 [201/200][149][137/139][82][64][50]
// 3x4 + 5 [481/471][463!][399!][374! - 373][260][150][146][126]
const int puzzleWidth = 4;
const int puzzleHeight = 4;
const int minSolutions = 5000;
const int numRequiredPieces = 1; // 5
const int numSeparationPieces = 4; // 3x4 + 6 [4]
int currBoard = 0;

Witness<puzzleWidth, puzzleHeight> w;
InteractiveWitnessState<puzzleWidth, puzzleHeight> iws;
std::vector<Witness<puzzleWidth, puzzleHeight>> best;
//std::vector<uint64_t> otherbest;

void GetAllSolutions();
int CountSolutions(const Witness<puzzleWidth, puzzleHeight> &w,
				   const std::vector<WitnessState<puzzleWidth, puzzleHeight>> &allSolutions, int &len, int limit);
int CountSolutions(const Witness<puzzleWidth, puzzleHeight> &w,
				   const std::vector<WitnessState<puzzleWidth, puzzleHeight>> &allSolutions,
				   std::vector<int> &solutions,
				   const std::vector<int> &forbidden,
				   int &len, int limit);
void ExamineMustCross(int count);
void Load(uint64_t rank);
void ExamineMustCrossAndRegions(int crossCount, int regionCount);
void ExamineMustCrossAnd3Regions(int crossCount, int regionCount);
void ExamineTetris(int count);
void ExamineTriangles(int count);
void ExamineRegionsAndStars(int count);

template <int puzzleWidth, int puzzleHeight>
void GetAllSolutions(const Witness<puzzleWidth, puzzleHeight> &w, std::vector<WitnessState<puzzleWidth, puzzleHeight>> &puzzles);

int main(int argc, char* argv[])
{
//	GetAllSolutions();
	InstallHandlers();
	RunHOGGUI(argc, argv, 640, 640);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Solve", "Solve current board", kAnyModifier, 'v');
	InstallKeyboardHandler(MyDisplayHandler, "Test", "Test constraints", kAnyModifier, 't');
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Save", "Save current puzzle as svg", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Prev Board", "Jump to next found board.", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Next Board", "Jump to prev found board", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Prev 100 Board", "Jump to next 100 found board.", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Next 100 Board", "Jump to prev 100 found board", kAnyModifier, '}');

	InstallCommandLineHandler(MyCLHandler, "-run", "-run", "Runs pre-set experiments.");
	InstallCommandLineHandler(MyCLHandler, "-test", "-test", "Basic test with MD heuristic");
	
	InstallWindowHandler(MyWindowHandler);
	InstallMouseClickHandler(MyClickHandler, static_cast<tMouseEventType>(kMouseMove|kMouseUp|kMouseDrag));
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

		
//		w.AddTriangleConstraint(0, 0, 3);
//		w.AddTriangleConstraint(0, 1, 2);
//		w.AddTriangleConstraint(1, 0, 1);
//		ExamineMustCross(numRequiredPieces);
//		w.AddSeparationConstraint(0, 0, Colors::white);
//		w.AddSeparationConstraint(0, 4, Colors::white);
//		w.AddSeparationConstraint(4, 0, Colors::white);
//		w.AddSeparationConstraint(2, 2, Colors::pink);
//		w.AddSeparationConstraint(4, 4, Colors::black);
//		w.AddSeparationConstraint(2, 0, Colors::black);
//		w.AddSeparationConstraint(2, 4, Colors::black);
//		w.AddSeparationConstraint(0, 2, Colors::orange);
//		w.AddSeparationConstraint(4, 2, Colors::orange);

//		std::string s = "{\"dim\":\"3x3\",\"cc\":{\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"4;3;#385CDE\",\"3;10;#DCA700\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\"},\"mc\":\"0000000000000000000000000000000000000000\"}\"";
//		w.LoadFromHashString(s);

		std::string s2 = "{\"dim\":\"4x4\",\"cc\":{\"1;0;#FFFFFF\",\"1;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#FFFFFF\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\"},\"mc\":\"10000000000000000000000000001100000000000000000000000000000000000\"}";
		std::string s = "{\"dim\":\"4x4\",\"cc\":{\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"0;0;#000000\",\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"1;1077084160;#FFFFFF\",\"0;0;#000000\",\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"1;0;#000000\",\"0;0;#000000\",\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"1;0;#FFFFFF\",\"0;0;#000000\"},\"mc\":\"00000000010000010000000000000001000000000000000000000000000000000\"}";
		s = 	"{\"dim\":\"4x4\",\"cc\":{\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"0;0;#0000FF\",\"0;0;#0000FF\",\"0;0;#FFFFFF\",\"1;0;#FFFFFF\",\"0;0;#0000FF\",\"0;0;#0000FF\",\"0;0;#FFFFFF\",\"1;0;#000000\",\"0;0;#0000FF\",\"1;0;#0000FF\",\"0;0;#FFFFFF\",\"0;0;#0000FF\",\"0;0;#0000FF\",\"1;0;#FFFFFF\"},\"mc\":\"00000010000000000000000000000000000000000000000000000000000000000\"}";
		s = "{\"dim\":\"4x4\",\"cc\":{\"1;0;#0000FF\",\"0;0;#000026D\",\"0;0;#0029D00\",\"2;0;#0000FF\",\"1;0;#0000FF\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#0000FF\",\"2;0;#0000FF\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#0000FF\",\"1;0;#0000FF\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#0000FF\"},\"mc\":\"10000000000010000000000000000000100000000000000000000000000000000\"}";

//		w.LoadFromHashString(s);
		w.AddTriangleConstraint(0, 0, 1);
		w.AddTriangleConstraint(1, 1, 2);
		w.AddTriangleConstraint(2, 2, 3);
		w.AddStarConstraint(3, 3, Colors::orange);

		//		w.AddTetrisConstraint(1, 1, 10);
//		w.AddNegativeTetrisConstraint(1, 0, 3);
//		w.AddTetrisConstraint(2, 1, 2);
//		w.AddTetrisConstraint(1, 2, 2);
//		w.AddTetrisConstraint(2, 2, 2);
//		w.AddTetrisConstraint(1, 3, 2);
//		w.AddTetrisConstraint(2, 3, 2);
//		w.AddMustCrossConstraint(false, 2, 0);
//		w.AddMustCrossConstraint(false, 2, 4);


//		w.AddTetrisConstraint(0, 2, 2);
//		w.AddTetrisConstraint(0, 1, 1);
//		w.AddTetrisConstraint(3, 3, );
//		w.AddTetrisConstraint(0, 0, 10);
//		w.AddTetrisConstraint(3, 0, 12);
//		w.AddTetrisConstraint(3, 1, 14);
//		w.AddTetrisConstraint(2, 3, -8);
//		w.AddTetrisConstraint(2, 2, -12);
//		w.AddTetrisConstraint(1, 1, -13);
//		w.AddTetrisConstraint(1, 0, 4);
//		w.AddTetrisConstraint(1, 1, 5);
//		w.AddTetrisConstraint(1, 2, 6);
//		w.AddTetrisConstraint(1, 3, 7);
//		w.AddTetrisConstraint(2, 0, 8);
//		w.AddTetrisConstraint(2, 1, 9);
//		w.AddTetrisConstraint(2, 2, 10);
//		w.AddTetrisConstraint(2, 3, 11);
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &d = GetContext(windowID)->display;
	iws.IncrementTime();
	w.Draw(d);
	w.Draw(d, iws);
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
//	if (strcmp(argument[0], "-test") == 0)
//	{
//		BaselineTest();
//		exit(0);
//	}
	return 0;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 't':
//			ExamineMustCross(numRequiredPieces);
//			w.ClearTetrisConstraints();
//			ExamineTetris(3);
//			ExamineMustCrossAndRegions(numRequiredPieces, numSeparationPieces);
//			ExamineMustCrossAnd3Regions(numRequiredPieces, numSeparationPieces);
			ExamineTriangles(6);
//			ExamineRegionsAndStars(0);
			break;
		case 'v':
		{
			std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
			GetAllSolutions(w, allSolutions);
			if (allSolutions.size() > 0)
			{
				iws.ws = allSolutions[0];
				iws.currState = InteractiveWitnessState<puzzleWidth, puzzleHeight>::kWaitingRestart;
			}
		}
			break;
		case 's':
		{
			Graphics::Display d;
			//d.FillRect({-1, -1, 1, 1}, Colors::darkgray);
			w.Draw(d);
			w.Draw(d, iws);
			std::string fname = "/Users/nathanst/Desktop/SVG/witness_";
			int count = 0;
			while (fileExists((fname+std::to_string(count)+".svg").c_str()))
			{
				count++;
			}
			printf("Save to '%s'\n", (fname+std::to_string(count)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(count)+".svg").c_str(), 400, 400, 0, w.SaveToHashString().c_str());

			{
				int wide, high;
				w.GetDimensionsFromHashString(w.SaveToHashString(), wide, high);
			}
		}
			break;
		case 'r': recording = !recording; break;
		case '\t':
			if (mod != kShiftDown)
				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
			else
			{
				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
			}
			break;
		case '[':
			if (best.size() > 0)
			{
				currBoard = (currBoard+(int)(best.size())-1)%best.size();
				Load(currBoard);
				printf("%d of %lu\n", currBoard+1, best.size());
			}
			break;
		case ']':
			if (best.size() > 0)
			{
				currBoard = (currBoard+1)%best.size();
				Load(currBoard);//, numRequiredPieces, numSeparationPieces);
				printf("%d of %lu\n", currBoard+1, best.size());
			}
			break;
		case '{':
			if (best.size() > 0)
			{
				currBoard = (currBoard+(int)(100*best.size())-100)%best.size();
				Load(currBoard);
				printf("%d of %lu\n", currBoard+1, best.size());
			}
			break;
		case '}':
			if (best.size() > 0)
			{
				currBoard = (currBoard+100)%best.size();
				Load(currBoard);//, numRequiredPieces, numSeparationPieces);
				printf("%d of %lu\n", currBoard+1, best.size());
			}
			break;

		case 'o':
			if (iws.ws.path.size() == 0)
			{
				iws.ws.path.push_back({0, 0});
				iws.ws.path.push_back({0, 1});
				iws.ws.path.push_back({1, 1});
			}
			else {
				iws.Reset();
			}
			break;
		default:
			break;
	}
}

bool MyClickHandler(unsigned long, int, int, point3d p, tButtonType , tMouseEventType e)
{
	if (e == kMouseDrag) // ignore movement with mouse button down
		return true;
	
	if (e == kMouseUp)
	{
		if (w.Click(p, iws)) // found goal
		{
			if (w.GoalTest(iws.ws))
			{
				printf("Solved!\n");
			}
			else {
				printf("Invalid solution\n");
				iws.Reset();
			}
		}
	}
	if (e == kMouseMove)
	{
		w.Move(p, iws);
	}

	// Don't need any other mouse support
	return true;
}


template <int puzzleWidth, int puzzleHeight>
void DFS(const Witness<puzzleWidth, puzzleHeight> &w,
		 WitnessState<puzzleWidth, puzzleHeight> &s,
		 std::vector<WitnessState<puzzleWidth, puzzleHeight>> &puzzles)
{
	std::vector<WitnessAction> acts;

	if (w.GoalTest(s))
	{
		puzzles.push_back(s);
		return;
	}
	
	w.GetActions(s, acts);
	for (auto &a : acts)
	{
		w.ApplyAction(s, a);
		DFS(w, s, puzzles);
		w.UndoAction(s, a);
	}
}


template <int puzzleWidth, int puzzleHeight>
void GetAllSolutions(const Witness<puzzleWidth, puzzleHeight> &w, std::vector<WitnessState<puzzleWidth, puzzleHeight>> &puzzles)
{
	WitnessState<puzzleWidth, puzzleHeight> s;
	s.Reset();
	Timer t;
	t.StartTimer();
	DFS(w, s, puzzles);
	t.EndTimer();
	printf("%lu solutions found in %1.2fs\n", puzzles.size(), t.GetElapsedTime());
}

template <int puzzleWidth, int puzzleHeight>
void GetAllSolutions(std::vector<WitnessState<puzzleWidth, puzzleHeight>> &puzzles)
{
	Witness<puzzleWidth, puzzleHeight> w;
	GetAllSolutions(w, puzzles);
}

void GetAllSolutions()
{
//	std::vector<WitnessState<puzzleWidth, puzzleHeight>> puzzles;
//	GetAllSolutions(puzzles);

	std::vector<WitnessState<2, 2>> p1;
	GetAllSolutions(p1);
	std::vector<WitnessState<2, 3>> p2;
	GetAllSolutions(p2);
	std::vector<WitnessState<3, 3>> p3;
	GetAllSolutions(p3);
	std::vector<WitnessState<3, 4>> p4;
	GetAllSolutions(p4);
	std::vector<WitnessState<4, 4>> p5;
	GetAllSolutions(p5);
	std::vector<WitnessState<4, 5>> p6;
	GetAllSolutions(p6);
	std::vector<WitnessState<5, 5>> p7;
	GetAllSolutions(p7);

}

int CountSolutions(const Witness<puzzleWidth, puzzleHeight> &w,
				   const std::vector<WitnessState<puzzleWidth, puzzleHeight>> &allSolutions,
				   int &len, int limit)
{
	int count = 0;
	for (const auto &i : allSolutions)
	{
		if (w.GoalTest(i))
		{
			len = (int)i.path.size();
			count++;
		}
		if (count > limit)
			break;
	}
	return count;
}

int CountSolutions(const Witness<puzzleWidth, puzzleHeight> &w,
				   const std::vector<WitnessState<puzzleWidth, puzzleHeight>> &allSolutions,
				   std::vector<int> &solutions,
				   const std::vector<int> &forbidden,
				   int &len, int limit)
{
	solutions.resize(0);
	int count = 0;
	for (int x = 0; x < forbidden.size(); x++)
	{
		if (w.GoalTest(allSolutions[forbidden[x]]))
			return 0;
	}
	for (int x = 0; x < allSolutions.size(); x++)
	{
		if (w.GoalTest(allSolutions[x]))
		{
			len = (int)allSolutions[x].path.size();
			count++;
			solutions.push_back(x);
		}
		if (count > limit)
			break;
	}
	return count;
}

void Load(uint64_t which)//, int req, int sep)
{
	w = best[which];
	iws.Reset();
//	iws.Reset();
//
//		w.ClearMustCrossConstraints();
//		int *items = new int[req];
//		int *items2 = new int[sep];
//		Combinations<w.GetNumMustCrossConstraints()> c;
//		Combinations<w.GetNumSeparationConstraints()> regionCombs;
//
//	if (best.size() > 0)
//	{
//		c.Unrank(best[which], items, req);
//		for (int x = 0; x < req; x++)
//		{
//			w.SetMustCrossConstraint(items[x]);
//		}
//	}
//
//	if (otherbest.size() > 0)
//	{
//		w.ClearSeparationConstraints();
//		uint64_t bits = otherbest[which]; // will only use bottom bits
//		uint64_t hash = otherbest[which]/(1<<sep);
//		regionCombs.Unrank(hash, items2, sep);
//		for (int x = 0; x < sep; x++)
//		{
//			w.AddSeparationConstraint(items2[x], ((bits>>x)&1)?Colors::white:Colors::black);
//		}
//	}
//
//	delete [] items;
//	delete [] items2;
}

void ExamineMustCross(int count)
{
	Timer t;
	t.StartTimer();
	std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
	GetAllSolutions(allSolutions);

	uint64_t minCount = allSolutions.size();
	int *items = new int[count];
	Combinations<w.GetNumMustCrossConstraints()> c;


	Witness<puzzleWidth, puzzleHeight> w;
	WitnessState<puzzleWidth, puzzleHeight> s;
	
	uint64_t maxRank = c.MaxRank(count);
	for (uint64_t n = 0; n < maxRank; n++)
	{
		if (0 == n%50000)
			printf("%llu of %llu\n", n, maxRank);
		c.Unrank(n, items, count);
		for (int x = 0; x < count; x++)
		{
			w.SetMustCrossConstraint(items[x]);
		}
		
		int pathLen = 0;
		int result = CountSolutions(w, allSolutions, pathLen, minCount+1);
		if (result > minSolutions)
		{
			// ignore
		}
		else if (result < minCount && result > 0)
		{
			minCount = result;
			best.clear();
			best.push_back(w);
		}
		else if (result == minCount)
		{
			best.push_back(w);
		}
		
		for (int x = 0; x < count; x++)
		{
			w.ClearMustCrossConstraint(items[x]);
		}
	}

	printf("\n%lu boards with %llu solutions; %1.2fs elapsed\n", best.size(), minCount, t.EndTimer());
	if (best.size() > 0)
	{
		currBoard = 0;
		Load(currBoard);
	}
	
	delete [] items;
}

void ExamineMustCrossAndRegions(int crossCount, int regionCount)
{
	Timer t;
	t.StartTimer();
	std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
	GetAllSolutions(allSolutions);
	
	uint64_t minCount = allSolutions.size();
	int bestPathSize = 0;
	//	std::vector<uint64_t> best;
	int *crossItems = new int[crossCount];
	int *regionItems = new int[regionCount];
	Combinations<w.GetNumMustCrossConstraints()> c;
	Combinations<w.GetNumSeparationConstraints()> regionCombs;

	{
		
		Witness<puzzleWidth, puzzleHeight> w;
		WitnessState<puzzleWidth, puzzleHeight> s;
		
		uint64_t maxRank = c.MaxRank(crossCount);
		for (uint64_t n = 0; n < maxRank; n++)
		{
			if (0 == n%50000 && regionCount == 0)
				printf("%llu of %llu\n", n, maxRank);
			c.Unrank(n, crossItems, crossCount);
			for (int x = 0; x < crossCount; x++)
			{
				w.SetMustCrossConstraint(crossItems[x]);
			}
			uint64_t colorComb = pow(2, regionCount);
			for (uint64_t t = 0; t < regionCombs.MaxRank(regionCount)*colorComb; t++)
			{
				uint64_t globalPuzzle = n*regionCombs.MaxRank(regionCount)*colorComb+t;
				if ((0 == globalPuzzle%50000))
					printf("-->%llu of %llu\n", globalPuzzle, maxRank*regionCombs.MaxRank(regionCount)*colorComb);
				uint64_t bits = t; // will only use bottom bits
				uint64_t hash = t/colorComb;

				// easy way to reduce symmetry
				if (t&1) continue;
				
//				if (0 != bits%3)
//					continue;
//				if (1 != (bits/3)%3)
//					continue;
//				if (2 != (bits/9)%3)
//					continue;

				w.ClearSeparationConstraints();
				regionCombs.Unrank(hash, regionItems, regionCount);
				for (int x = 0; x < regionCount; x++)
				{
					int colour = bits%2;
					bits = bits/2;
//					w.AddSeparationConstraint(regionItems[x], ((bits>>x)&1)?Colors::white:Colors::black);
					w.AddSeparationConstraint(regionItems[x], (colour==0)?Colors::white:((colour==1)?Colors::black:Colors::blue));
				}
				
				int pathSize = 0;
				int result = CountSolutions(w, allSolutions, pathSize, minCount+1);

				if (result < minCount && result > 0)
				{
					minCount = result;
					bestPathSize = pathSize;
					best.clear();
					best.push_back(w);
				}
				else if (result == minCount && pathSize == bestPathSize)
//				else if (result == minCount)
				{
					best.push_back(w);
				}
			}
			
			w.ClearMustCrossConstraints();
		}
	}
	
	printf("\n%lu boards with %llu solutions len %d; %1.2fs elapsed\n", best.size(), minCount, bestPathSize, t.EndTimer());
	if (best.size() > 0)
	{
		currBoard = 0;
		Load(currBoard);
	}
	
	delete [] crossItems;
	delete [] regionItems;
}

void ExamineMustCrossAnd3Regions(int crossCount, int regionCount)
{
	Timer t;
	t.StartTimer();
	std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
	GetAllSolutions(allSolutions);
	
	uint64_t minCount = allSolutions.size();
	int bestPathSize = 0;
	//	std::vector<uint64_t> best;
	int *crossItems = new int[crossCount];
	int *regionItems = new int[regionCount];
	Combinations<w.GetNumMustCrossConstraints()> c;
	Combinations<w.GetNumSeparationConstraints()> regionCombs;
	
	{
		
		Witness<puzzleWidth, puzzleHeight> w;
		WitnessState<puzzleWidth, puzzleHeight> s;
		
		uint64_t maxRank = c.MaxRank(crossCount);
		for (uint64_t n = 0; n < maxRank; n++)
		{
			if (0 == n%50000 && regionCount == 0)
				printf("%llu of %llu\n", n, maxRank);
			c.Unrank(n, crossItems, crossCount);
			for (int x = 0; x < crossCount; x++)
			{
				w.SetMustCrossConstraint(crossItems[x]);
			}
			uint64_t colorComb = pow(3, regionCount);
			for (uint64_t t = 0; t < regionCombs.MaxRank(regionCount)*colorComb; t++)
			{
				uint64_t globalPuzzle = n*regionCombs.MaxRank(regionCount)*colorComb+t;
				if ((0 == globalPuzzle%50000))
					printf("-->%llu of %llu\n", globalPuzzle, maxRank*regionCombs.MaxRank(regionCount)*colorComb);
				uint64_t bits = t; // will only use bottom bits
				uint64_t hash = t/colorComb;
				
//				// easy way to reduce symmetry
//				if (t&1) continue;
//
				if (0 != bits%3)
					continue;
				if (1 != (bits/3)%3)
					continue;
				if (2 != (bits/9)%3)
					continue;
				
				w.ClearSeparationConstraints();
				regionCombs.Unrank(hash, regionItems, regionCount);
				for (int x = 0; x < regionCount; x++)
				{
					int colour = bits%3;
					bits = bits/3;
					w.AddSeparationConstraint(regionItems[x], (colour==0)?Colors::white:((colour==1)?Colors::black:Colors::blue));
				}
				
				int pathSize = 0;
				int result = CountSolutions(w, allSolutions, pathSize, minCount+1);
				
				if (result < minCount && result > 0)
				{
					minCount = result;
					bestPathSize = pathSize;
					best.clear();
					best.push_back(w);
				}
				else if (result == minCount && pathSize == bestPathSize)
					//				else if (result == minCount)
				{
					best.push_back(w);
				}
			}
			
			w.ClearMustCrossConstraints();
		}
	}
	
	printf("\n%lu boards with %llu solutions len %d; %1.2fs elapsed\n", best.size(), minCount, bestPathSize, t.EndTimer());
	if (best.size() > 0)
	{
		currBoard = 0;
		Load(currBoard);
	}
	
	delete [] crossItems;
	delete [] regionItems;
}

void ExamineRegionsAndStars(int count)
{
	Timer t;
	t.StartTimer();
	std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
	GetAllSolutions(allSolutions);
	
	uint64_t minCount = allSolutions.size();
	int bestPathSize = 0;
	//	std::vector<uint64_t> best;
	std::vector<int> items(count);
	Combinations<w.GetNumStarConstraints()> c;
	Combinations<w.GetNumMustCrossConstraints()> mc;
	std::vector<int> forbidden;
	std::vector<int> currSolutions;

	{
		Witness<puzzleWidth, puzzleHeight> w;
		WitnessState<puzzleWidth, puzzleHeight> s;
		
		static_assert((4==puzzleHeight)&&(puzzleWidth==4), "This code only works for 4x4");
//		uint64_t variations = pow(4, count-1)*2;
//		uint64_t maxRank = c.MaxRank(count)*variations;
		uint64_t variations = pow(4, 8);
		uint64_t mcRank = mc.MaxRank(count);
		uint64_t maxRank = mcRank*variations;//c.MaxRank(count)*variations;

		for (uint64_t n = 0; n < maxRank; n++)
		{
			if (0 == n%50000)
				printf("%llu of %llu\n", n, maxRank);
			w.ClearInnerConstraints();
			w.ClearMustCrossConstraints();
//			uint64_t rankPart = n/variations;
//			uint64_t varPart = n%variations;
//			c.Unrank(rankPart, &items[0], count);
//			for (int i : items)
//			{
//				bool colorPart = (varPart/2)%2;
//				bool shapePart = varPart%2;
//				varPart /= 4;
//				if (shapePart)
//					w.AddStarConstraint(i, colorPart?Colors::black:Colors::blue);
//				else
//					w.AddSeparationConstraint(i, colorPart?Colors::black:Colors::blue);
//			}

			uint64_t rank = n%variations;
			for (int x = 0; x < 4; x++)
			{
				rgbColor color;
				color = (rank&1)?Colors::white:Colors::orange;
				if (rank&2)
					w.AddStarConstraint(x, 0, color);
				else {
					w.AddTriangleConstraint(x, 0, 1+(rank&1));
					//w.AddSeparationConstraint(x, 0, color);
				}
				rank>>=2;

				color = (rank&1)?Colors::white:Colors::orange;
				if (rank&2)
					w.AddStarConstraint(x, 3, color);
				else {
//					w.AddSeparationConstraint(x, 3, color);
					w.AddTriangleConstraint(x, 3, 1+(rank&1));
				}
				rank>>=2;
			}
//			for (int y = 0; y < 4; y+=3)
//			{
//				rgbColor color;
//				color = (rank&2)?Colors::white:Colors::red;
//
//				if (rank&1)
//					w.AddStarConstraint(0, y, color);
//				else
//					w.AddSeparationConstraint(0, y, color);
//				rank>>=2;
//				if (rank&1)
//					w.AddStarConstraint(3, y, color);
//				else
//					w.AddSeparationConstraint(3, y, color);
//				rank>>=2;
//			}
			mc.Unrank(n/variations, &items[0], count);
			for (int x = 0; x < items.size(); x++)
			{
				w.SetMustCrossConstraint(items[x]);
			}
			
			int pathSize = 0;
			//int result = CountSolutions(w, allSolutions, pathSize, minCount+1);
			int result = CountSolutions(w, allSolutions, currSolutions, forbidden, pathSize, minCount+1);
			
			// don't return two puzzles with the same solution
			if (currSolutions.size() == 1)
				forbidden.push_back(currSolutions[0]);
			
			if (result < minCount && result > 0)
			{
				minCount = result;
				bestPathSize = pathSize;
				best.clear();
				best.push_back(w);
			}
			else if (result == minCount && pathSize == bestPathSize)
				//				else if (result == minCount)
			{
				best.push_back(w);
			}
		}
	}
	
	printf("\n%lu boards with %llu solutions len %d; %1.2fs elapsed\n", best.size(), minCount, bestPathSize, t.EndTimer());
	if (best.size() > 0)
	{
		currBoard = 0;
		Load(currBoard);
	}
	
}


//void ExamineRegions(int regionCount)
//{
//	Timer t;
//	t.StartTimer();
//	std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
//	GetAllSolutions(allSolutions);
//
//	uint64_t minCount = allSolutions.size();
//	int bestPathSize = 0;
//	//	std::vector<uint64_t> best;
//	int *regionItems = new int[regionCount];
//	Combinations<w.GetNumMustCrossConstraints()> c;
//	Combinations<w.GetNumSeparationConstraints()> regionCombs;
//
//	{
//
//		Witness<puzzleWidth, puzzleHeight> w;
//		WitnessState<puzzleWidth, puzzleHeight> s;
//
//		for (uint64_t t = 0; t < regionCombs.MaxRank(regionCount)*(1<<regionCount); t++)
//		{
//			if (0 == t%1000)
//				printf("-->%llu of %llu\n", t, regionCombs.MaxRank(regionCount)*(1<<regionCount));
//			uint64_t bits = t; // will only use bottom bits
//			uint64_t hash = t/(1<<regionCount);
//
//			// easy way to reduce symmetry
//			if (t&1) continue;
//
//			w.ClearSeparationConstraints();
//			regionCombs.Unrank(hash, regionItems, regionCount);
//			for (int x = 0; x < regionCount; x++)
//			{
//				w.AddSeparationConstraint(regionItems[x], ((bits>>x)&1)?Colors::white:Colors::black);
//			}
//
//			int pathSize = 0;
//			int result = CountSolutions(w, allSolutions, pathSize, minCount+1);
//			if (result > minSolutions)
//			{
//				// ignore
//			}
//			else if (result < minCount && result > 0)
//			{
//				minCount = result;
//				bestPathSize = pathSize;
//				best.clear();
//				otherbest.clear();
//				best.push_back(n);
//				otherbest.push_back(t);
//			}
//			else if (result == minCount && pathSize > bestPathSize)
//			{
//				minCount = result;
//				bestPathSize = pathSize;
//				best.clear();
//				otherbest.clear();
//				best.push_back(n);
//				otherbest.push_back(t);
//			}
//			else if (result == minCount && pathSize == bestPathSize)
//			{
//				best.push_back(n);
//				otherbest.push_back(t);
//			}
//		}
//
//		w.ClearMustCrossConstraints();
//	}
//
//	printf("\n%lu boards with %llu solutions len %d; %1.2fs elapsed\n", best.size(), minCount, bestPathSize, t.EndTimer());
//	if (best.size() > 0)
//	{
//		currBoard = 0;
//		Load(currBoard, 0, regionCount);
//	}
//
//	delete [] regionItems;
//}


void ExamineTetris(int count)
{
	Timer t;
	t.StartTimer();
	std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;

	Witness<puzzleWidth, puzzleHeight> wp;
	WitnessState<puzzleWidth, puzzleHeight> s;
	
	for (int y = 0; y < puzzleHeight+1; y++)
		for (int x = 0; x < puzzleWidth+1; x++)
			wp.AddMustCrossConstraint(x, y);

	GetAllSolutions(wp, allSolutions);
	
	uint64_t minCount = allSolutions.size();
	int *items = new int[count];
	Combinations<wp.GetNumTetrisConstraints()> c;
	
	uint64_t pCount = pow(24, count);
	uint64_t maxRank = c.MaxRank(count)*pCount;
	for (uint64_t rank = 0; rank < maxRank; rank++)
	{
		wp.ClearTetrisConstraints();
		if (0 == rank%50000)
			printf("%llu of %llu\n", rank, maxRank);
		uint64_t n = rank/pCount; // arrangement on board
		uint64_t pieces = rank%pCount; // pieces in locations
		c.Unrank(n, items, count);
		for (int x = 0; x < count; x++)
		{
			if (x == 0 && count > 1)
				wp.AddNegativeTetrisConstraint(items[x], (1+(pieces%24)));
			else
				wp.AddTetrisConstraint(items[x], 1+(pieces%24));
			pieces/=24;
		}
//		w = wp;
//		if (rank == 2)
//			break;
		int pathLen = 0;
		int result = CountSolutions(wp, allSolutions, pathLen, minCount+1);
		if (result > minSolutions)
		{
			// ignore
		}
		else if (result < minCount && result > 0)
		{
			minCount = result;
			best.clear();
			best.push_back(wp);
			w = wp;
		}
		else if (result == minCount)
		{
			best.push_back(wp);
		}
		
	}
	
	printf("\n%lu boards with %llu solutions; %1.2fs elapsed\n", best.size(), minCount, t.EndTimer());
//	return;

//	if (best.size() > 0)
//	{
//		currBoard = 0;
//		Load(currBoard, count, 0);
//	}
	
	delete [] items;
}


void ExamineTriangles(int count)
{
	Timer t;
	t.StartTimer();
	std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
	
	Witness<puzzleWidth, puzzleHeight> wp;
	WitnessState<puzzleWidth, puzzleHeight> s;
	
//	for (int y = 0; y < puzzleHeight+1; y++)
//		for (int x = 0; x < puzzleWidth+1; x++)
//			wp.AddMustCrossConstraint(x, y);
	
	GetAllSolutions(wp, allSolutions);
	
	uint64_t minCount = allSolutions.size();
	int *items = new int[count];
	Combinations<wp.GetNumTriangleConstraints()> c;
	
	uint64_t pCount = pow(3, count);
	uint64_t maxRank = c.MaxRank(count)*pCount;
	for (uint64_t rank = 0; rank < maxRank; rank++)
	{
		if (0 == rank%50000)
			printf("%llu of %llu\n", rank, maxRank);
		uint64_t n = rank/pCount; // arrangement on board
		uint64_t pieces = rank%pCount; // pieces in locations
		c.Unrank(n, items, count);
		for (int x = 0; x < count; x++)
		{
			wp.AddTriangleConstraint(items[x], 1+(pieces%3));
			pieces/=3;
		}
		int pathLen = 0;
		int result = CountSolutions(wp, allSolutions, pathLen, minCount+1);
		if (result > minSolutions)
		{
			// ignore
		}
		else if (result < minCount && result > 0)
		{
			minCount = result;
			best.clear();
			best.push_back(wp);
			w = wp;
		}
		else if (result == minCount)
		{
			best.push_back(wp);
		}
		
		wp.ClearTriangleConstraints();
	}
	
	printf("\n%lu boards with %llu solutions; %1.2fs elapsed\n", best.size(), minCount, t.EndTimer());
	//	return;
	
	//	if (best.size() > 0)
	//	{
	//		currBoard = 0;
	//		Load(currBoard, count, 0);
	//	}
	
	delete [] items;
}
