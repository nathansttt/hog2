/*
 *  $Id: Driver.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Modified by Nathan Sturtevant on 11/13/21.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include <cstring>
#include "Common.h"
#include "Driver.h"
#include "Timer.h"
#include "SVGUtil.h"
#include "Hexagon.h"
#include "Combinations.h"

bool recording = false;
bool saveAndExit = false;
std::string filename, outputFile;
Hexagon h;
HexagonState hs;
std::vector<HexagonSearchState> goals;

std::vector<std::vector<HexagonAction>> acts;
HexagonSearchState hss;
HexagonEnvironment he;
int currDepth = 0;

int main(int argc, char* argv[])
{
	setvbuf(stdout, NULL, _IONBF, 0);
	InstallHandlers();
	RunHOGGUI(argc, argv, 512, 1024);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	//	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Save goals", "Save all the currently generated goals", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Next Action", "Take Next action", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Prev Action", "Take prev action", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Next", "Next DFS step", kAnyModifier, 'n');
	InstallKeyboardHandler(MyDisplayHandler, "All", "Get All Goals", kAnyModifier, 'a');
	InstallKeyboardHandler(MyDisplayHandler, "Flip", "Flip Board", kAnyModifier, 'f');
	InstallKeyboardHandler(MyDisplayHandler, "Rotate", "Rotate Board", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Analyze1", "Analyze which piece to remove", kAnyModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Analyze2", "Analyze which pieces to make unflippable", kAnyModifier, 'o');

	
	InstallCommandLineHandler(MyCLHandler, "-loadPuzzle", "-loadPuzzle <file>", "Load level from file.");
	InstallCommandLineHandler(MyCLHandler, "-loadSolution", "-loadSolution <file>", "Load solution from file.");
	InstallCommandLineHandler(MyCLHandler, "-svg", "-svg <file>", "Write SVG to file. Also requires that a file is loaded.");

	InstallWindowHandler(MyWindowHandler);
//	InstallMouseClickHandler(MyClickHandler);
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
		ReinitViewports(windowID, {-1, -1, 0, 1}, kScaleToSquare);
		AddViewport(windowID, {0, -1, 1, 1}, kScaleToSquare);
//		if (load)
//		{
//			h.LoadSolution(filename.c_str(), hs);
//			h.LoadPuzzle(filename.c_str(), hs);
//		}
		acts.resize(1);
		he.GetActions(hss, acts[0]);
		currDepth = 0;
		
		if (saveAndExit)
		{
			Graphics::Display d;
			h.Draw(d);
			h.Draw(d, hs);
			MakeSVG(d, outputFile.c_str(), 1024, 1024);
			exit(0);
		}
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &d = GetContext(windowID)->display;
	if (viewport == 0)
	{
		he.Draw(d, hss);
//		h.Draw(d);
//		h.Draw(d, hs);
	}
	if (viewport == 1)
	{
		
//		h.DrawSetup(d);
	}
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-loadSolution") == 0)
	{
		if (maxNumArgs > 1)
		{
			h.LoadSolution(argument[1], hs);
			return 2;
		}
		printf("Error: too few arguments to -loadSolution. Input file required\n");
		exit(0);
	}
	else if (strcmp(argument[0], "-loadPuzzle") == 0)
	{
		if (maxNumArgs > 1)
		{
			h.LoadPuzzle(argument[1], hs);
			return 2;
		}
		printf("Error: too few arguments to -loadPuzzle. Input file required\n");
		exit(0);
	}
	else if (strcmp(argument[0], "-svg") == 0)
	{
		if (maxNumArgs > 1)
		{
			saveAndExit = true;
			outputFile = argument[1];
			return 2;
		}
		printf("Error: too few arguments to -svg. Output filename required.\n");
		exit(0);
	}
	return 0;
}

bool AddGoal(HexagonSearchState s)
{
	// faster ways to do this, but they require the hash function I haven't written yet
	for (int x = 0; x < goals.size(); x++)
	{
		HexagonSearchState tmp = goals[x];
		for (int y = 0; y < 6; y++)
		{
			if (s == tmp)
				return false;
			he.RotateCW(tmp);
		}
		he.Flip(tmp);
		for (int y = 0; y < 6; y++)
		{
			if (s == tmp)
				return false;
			he.RotateCW(tmp);
		}
	}
	goals.push_back(s);
	return true;
}

void AnalyzeWhichPiecesToUse()
{
	// Clear and then set the items
	std::array<tFlipType, numPieces> toFlip;
	for (int x = 0; x < numPieces; x++)
		toFlip[x] = kCanFlip;
	he.SetFlippable(toFlip);
	
	const std::vector<tPieceName> allPieces =
	{kHexagon, kElbow, kLine, kMountains, kWrench, kTriangle, kHook, kSnake, kButterfly, kTrapezoid, kTrapezoid};
	std::vector<tPieceName> pieces;
	
	for (int x = 0; x < 9; x++)
	{
		pieces = allPieces;
		
		printf("%s%s ", pieceNames[allPieces[x]].c_str(), (toFlip[x]==kSide1)?"1":(toFlip[x]==kSide2)?"2":"");
		pieces.erase(pieces.begin()+x);
		he.SetPieces(pieces);
		MyDisplayHandler(0, kNoModifier, 'a');
	}
	// now remove trapezoids
	pieces = allPieces;
	pieces.pop_back();
	pieces.pop_back();
	printf("%s ", pieceNames[allPieces[9]].c_str());
	//			printf("Piece set: ");
	//			for (auto i : pieces)
	//				printf("%d ", (int)i);
	//			printf("\n");
	he.SetPieces(pieces);
	MyDisplayHandler(0, kNoModifier, 'a');
}

void AnalyzeWhichPiecesToFlip()
{
	Combinations<6> c;
	int items[3];
	for (uint64_t i = 0; i < c.MaxRank(3)*8; i++)
	{
		uint8_t order = i%8;
		// Get which itesms to be able to flip this time
		c.Unrank(i/8, items, 3);

		// Clear and then set the items
		std::array<tFlipType, numPieces> toFlip;
		for (int x = 0; x < numPieces; x++)
			toFlip[x] = kCanFlip;
		for (int x = 0; x < 3; x++)
		{
			switch(order&1)
			{
				case 0:
					toFlip[items[x]] = kSide1;
					break;
				case 1:
					toFlip[items[x]] = kSide2;
					break;
			}
			order/=2;
		}
		he.SetFlippable(toFlip);

		const std::vector<tPieceName> allPieces =
		{kLine, kMountains, kWrench, kTriangle, kHook, kSnake, kHexagon, kElbow, kButterfly, kTrapezoid, kTrapezoid};
		std::vector<tPieceName> pieces;

		printf("\n---- %s%s, %s%s, %s%s ----\n",
			   pieceNames[allPieces[items[0]]].c_str(), (toFlip[items[0]]==kSide1)?"1":(toFlip[items[0]]==kSide2)?"2":"",
			   pieceNames[allPieces[items[1]]].c_str(), (toFlip[items[1]]==kSide1)?"1":(toFlip[items[1]]==kSide2)?"2":"",
			   pieceNames[allPieces[items[2]]].c_str(), (toFlip[items[2]]==kSide1)?"1":(toFlip[items[2]]==kSide2)?"2":"");

		for (int x = 0; x < 1; x++)
		{
			pieces = allPieces;

			printf("%s%s ", pieceNames[allPieces[x]].c_str(), (toFlip[x]==kSide1)?"1":(toFlip[x]==kSide2)?"2":"");
			pieces.erase(pieces.begin()+x);
			he.SetPieces(pieces);
			MyDisplayHandler(0, kNoModifier, 'a');
		}
		// now remove trapezoids
//		pieces = allPieces;
//		pieces.pop_back();
//		pieces.pop_back();
//		printf("%s ", pieceNames[allPieces[9]].c_str());
//		he.SetPieces(pieces);
//		MyDisplayHandler(0, kNoModifier, 'a');
	}
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 's':
		{
			for (auto g : goals)
			{
				Graphics::Display d;
				d.StartFrame();
				he.Draw(d, g);
				d.EndFrame();
				
			}
			break;
		}
		case 'f':
			he.Flip(hss);
			break;
		case 'r':
			he.RotateCW(hss);
			break;
		case 'n':
		{
			static int totalGoals = 0;
			while (true)
			{
				if (acts[currDepth].size() > 0)
				{
					he.ApplyAction(hss, acts[currDepth].back());
					currDepth++;
					acts.resize(currDepth+1);
					he.GetActions(hss, acts.back());
				}
				else if (currDepth > 0) {
					currDepth--;
					he.UndoAction(hss, acts[currDepth].back());
					acts[currDepth].pop_back();
				}
				else {
					printf("Done!\n");
					break;
				}
				if (he.GoalTest(hss))
				{
					if (AddGoal(hss))
					{
						totalGoals++;
						printf("%d total goals\n", totalGoals);
						break;
					}
				}
			}
		}
			break;
		case 'o':
			AnalyzeWhichPiecesToFlip();
			break;
		case 'p':
			AnalyzeWhichPiecesToUse();
			break;
//		{
//			const std::vector<tPieceName> allPieces =
//			{kHexagon, kElbow, kLine, kMountains, kWrench, kTriangle, kHook, kSnake, kButterfly, kTrapezoid, kTrapezoid};
//			std::vector<tPieceName> pieces;
//			for (int x = 0; x < 9; x++)
//			{
//				pieces = allPieces;
//				printf("%s ", pieceNames[allPieces[x]].c_str());
//				pieces.erase(pieces.begin()+x);
////				printf("Piece set: ");
////				for (auto i : pieces)
////					printf("%d ", (int)i);
////				printf("\n");
//				he.SetPieces(pieces);
//				MyDisplayHandler(windowID, mod, 'a');
//			}
//			// now remove trapezoids
//			pieces = allPieces;
//			pieces.pop_back();
//			pieces.pop_back();
//			printf("%s ", pieceNames[allPieces[9]].c_str());
////			printf("Piece set: ");
////			for (auto i : pieces)
////				printf("%d ", (int)i);
////			printf("\n");
//			he.SetPieces(pieces);
//			MyDisplayHandler(windowID, mod, 'a');
//		}
			break;
		case 'a':
		{
			static int totalGoals = 0;
			static int totalExpansions = 0;
			totalGoals = totalExpansions = 0;
			goals.clear();
			hss.Reset();
			acts.resize(1);
			he.GetActions(hss, acts[0]);
			currDepth = 0;
			Timer t;
			t.StartTimer();
			while (true)
			{
				if (acts[currDepth].size() > 0)
				{
					he.ApplyAction(hss, acts[currDepth].back());
					currDepth++;
					acts.resize(currDepth+1);
					he.GetActions(hss, acts.back());
					totalExpansions++;
				}
				else if (currDepth > 0) {
					currDepth--;
					he.UndoAction(hss, acts[currDepth].back());
					acts[currDepth].pop_back();
				}
				else {
					t.EndTimer();
					printf("& %lu & %d & %1.2f \\\\\n", goals.size(), totalGoals, totalExpansions/1000000.0);
					//printf("%1.2f elapsed\n", t.GetElapsedTime());
					break;
				}
				if (he.GoalTest(hss))
				{
					AddGoal(hss);
//					goals.push_back(hss);
					totalGoals++;
				}
			}
		}
		case ']':
			if (acts[currDepth].size() > 0)
			{
				he.ApplyAction(hss, acts[currDepth].back());
				currDepth++;
				acts.resize(currDepth+1);
				he.GetActions(hss, acts.back());
			}
			else {
				//printf("No more actions to apply at depth %d\n", currDepth);
			}
//			he.ApplyAction(hss, acts[currAct]);
//			currAct = (currAct+1)%acts.size();
//			he.ApplyAction(hss, acts[currAct]);
//			printf("%d\n", currDepth);
			break;
		case '[':
			if (acts[currDepth].size() == 0)
			{
				printf("Nothing to undo\n");
				break;
			}
				
			currDepth--;
			he.UndoAction(hss, acts[currDepth].back());
			acts[currDepth].pop_back();
//
//			he.ApplyAction(hss, acts[currAct]);
//			currAct = (currAct+acts.size()-1)%acts.size();
//			he.ApplyAction(hss, acts[currAct]);
			printf("%d\n", currDepth);
			break;
		default:
			break;
	}
}
