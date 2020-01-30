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
#include "Combinations.h"
#include "SVGUtil.h"

#include <sys/stat.h>
bool fileExists(const char *name)
{
	struct stat buffer;
	return (stat(name, &buffer) == 0);
}
#include "SnakeBird.h"

bool recording = false;
bool parallel = false;

SnakeBird::SnakeBird sb;
SnakeBird::SnakeBirdState snake;
int snakeControl = 0;
std::vector<SnakeBird::SnakeBirdState> history;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 640, 640);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
//	InstallKeyboardHandler(MyDisplayHandler, "Solve", "Solve current board", kAnyModifier, 'v');
//	InstallKeyboardHandler(MyDisplayHandler, "Test", "Test constraints", kAnyModifier, 't');
//	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
//	InstallKeyboardHandler(MyDisplayHandler, "Save", "Save current puzzle as svg", kAnyModifier, 's');
//	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
//	InstallKeyboardHandler(MyDisplayHandler, "Prev Board", "Jump to next found board.", kAnyModifier, '[');
//	InstallKeyboardHandler(MyDisplayHandler, "Next Board", "Jump to prev found board", kAnyModifier, ']');
//	InstallKeyboardHandler(MyDisplayHandler, "Prev 100 Board", "Jump to next 100 found board.", kAnyModifier, '{');
//	InstallKeyboardHandler(MyDisplayHandler, "Next 100 Board", "Jump to prev 100 found board", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Up", "Move up", kAnyModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Down", "Move down", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Left", "Move left", kAnyModifier, 'a');
	InstallKeyboardHandler(MyDisplayHandler, "Right", "Move right", kAnyModifier, 'd');
	InstallKeyboardHandler(MyDisplayHandler, "Undo", "Undo last move", kAnyModifier, 'q');
	InstallKeyboardHandler(MyDisplayHandler, "Next", "Select Next Snake", kAnyModifier, 'e');

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
		
		
		sb.SetGroundType(0, 0, SnakeBird::kEmpty);
		sb.SetGroundType(1, 0, SnakeBird::kGround);
		sb.SetGroundType(2, 0, SnakeBird::kSpikes);
		sb.SetGroundType(3, 0, SnakeBird::kPortal);
		sb.SetGroundType(4, 0, SnakeBird::kExit);
		sb.SetGroundType(5, 0, SnakeBird::kFruit);
		
		snake.SetNumSnakes(2);
		snake.SetSnakeHeadLoc(0, 6);
		snake.SetSnakeBodyEnd(0, 4);
		snake.SetSnakeDir(0, 0, SnakeBird::kRight);
		snake.SetSnakeDir(0, 1, SnakeBird::kUp);
		snake.SetSnakeDir(0, 2, SnakeBird::kRight);
		snake.SetSnakeDir(0, 3, SnakeBird::kUp);

		snake.SetSnakeHeadLoc(1, 61);
		snake.SetSnakeBodyEnd(1, 6);
		snake.SetSnakeDir(1, 2, SnakeBird::kRight);
		snake.SetSnakeDir(1, 3, SnakeBird::kDown);
		history.push_back(snake);
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &d = GetContext(windowID)->display;
	sb.Draw(d);
	sb.Draw(d, snake, snakeControl);
//	iws.IncrementTime();
//	w.Draw(d);
//	w.Draw(d, iws);
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	return 0;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	SnakeBird::SnakeBirdAction a;
	a.bird = snakeControl;
	switch (key)
	{
		case 'w':
			a.direction = SnakeBird::kUp;
			if (sb.Legal(snake, a))
			{
				sb.ApplyAction(snake, a);
				history.push_back(snake);
			}
			break;
		case 's':
			a.direction = SnakeBird::kDown;
			if (sb.Legal(snake, a))
			{
				sb.ApplyAction(snake, a);
				history.push_back(snake);
			}
			break;
		case 'a':
			a.direction = SnakeBird::kLeft;
			if (sb.Legal(snake, a))
			{
				sb.ApplyAction(snake, a);
				history.push_back(snake);
			}
			break;
		case 'd':
			a.direction = SnakeBird::kRight;
			if (sb.Legal(snake, a))
			{
				sb.ApplyAction(snake, a);
				history.push_back(snake);
			}
			break;
		case 'q':
			if (history.size() > 1)
				history.pop_back();
			snake = history.back();
			break;
		case 'e':
			snakeControl = (snakeControl+1)%2;
			break;
		case 't':
			break;
		case 'v':
		{
		}
			break;
//		case 's':
		{
//			Graphics::Display d;
//			//d.FillRect({-1, -1, 1, 1}, Colors::darkgray);
//			w.Draw(d);
//			w.Draw(d, iws);
//			std::string fname = "/Users/nathanst/Desktop/SVG/witness_";
//			int count = 0;
//			while (fileExists((fname+std::to_string(count)+".svg").c_str()))
//			{
//				count++;
//			}
//			printf("Save to '%s'\n", (fname+std::to_string(count)+".svg").c_str());
//			MakeSVG(d, (fname+std::to_string(count)+".svg").c_str(), 400, 400, 0, w.SaveToHashString().c_str());
//
//			{
//				int wide, high;
//				w.GetDimensionsFromHashString(w.SaveToHashString(), wide, high);
//			}
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
			break;
		case ']':
			break;
		case '{':
			break;
		case '}':
			break;

		case 'o':
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
	}
	if (e == kMouseMove)
	{
	}

	// Don't need any other mouse support
	return true;
}

