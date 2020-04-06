/*
 *  $Id: sample.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include <cstring>
#include "Common.h"
#include "Driver.h"
#include "Timer.h"
#include <deque>
#include "Combinations.h"
#include "SVGUtil.h"
#include "TemplateAStar.h"
#include "BFS.h"
#include "ScreenTransition.h"

#include <sys/stat.h>
bool fileExists(const char *name)
{
	struct stat buffer;
	return (stat(name, &buffer) == 0);
}
#include "SnakeBird.h"

std::string mapName;

bool recording = false;
bool autoSolve = false;
double globalTime = 0;
double frameTime = 0; // seconds elapsed in current frame
double timePerFrame = 0.1; // seconds for drawing each step of frame
double frameRate = 1.0/30.0;
SnakeBird::SnakeBirdAnimationStep actionInProgressStep;
bool actionInProgress = false;
SnakeBird::SnakeBirdAction inProgress;

Timer worldClock;
double lastFrameStart;
LineTransition transition(100, 60, Colors::white);

SnakeBird::SnakeBird sb(20, 20);
SnakeBird::SnakeBirdState snake;
SnakeBird::SnakeBirdState lastFrameSnake;
int snakeControl = 0;
std::vector<SnakeBird::SnakeBirdState> history;
std::vector<SnakeBird::SnakeBirdState> future;

void AnalyzeMapChanges();


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
	InstallKeyboardHandler(MyDisplayHandler, "Level", "Goto nth level", kAnyModifier, '0', '5');

	InstallKeyboardHandler(MyDisplayHandler, "Up", "Move up", kAnyModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Down", "Move down", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Left", "Move left", kAnyModifier, 'a');
	InstallKeyboardHandler(MyDisplayHandler, "Right", "Move right", kAnyModifier, 'd');
	InstallKeyboardHandler(MyDisplayHandler, "Undo", "Undo last move", kAnyModifier, 'q');
	InstallKeyboardHandler(MyDisplayHandler, "Next", "Select Next Snake", kAnyModifier, 'e');
	InstallKeyboardHandler(MyDisplayHandler, "Reset", "Reset Level", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Print", "Print screen to file", kAnyModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Movie", "Record movie frames", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Solve", "Build solution", kAnyModifier, 'b');
	InstallKeyboardHandler(MyDisplayHandler, "Run", "Run solution (build if needed)", kAnyModifier, 'n');
	InstallKeyboardHandler(MyDisplayHandler, "Step", "Take next step in solution", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Change", "Make one change to increase solution length", kAnyModifier, 'c');

	InstallCommandLineHandler(MyCLHandler, "-load", "-load <file>", "Run snake bird with the given file");
	InstallCommandLineHandler(MyCLHandler, "-svg", "-svg <input> <output>", "Make an .svg of the given level then quit");
	InstallCommandLineHandler(MyCLHandler, "-bfs", "-bfs <file>", "Run BFS on the given level and return the info");
//	InstallCommandLineHandler(MyCLHandler, "-test", "-test", "Basic test with MD heuristic");
	
	InstallWindowHandler(MyWindowHandler);
	InstallMouseClickHandler(MyClickHandler, static_cast<tMouseEventType>(kMouseMove|kMouseUp|kMouseDrag));
}

void Save(const SnakeBird::SnakeBird &sb, const SnakeBird::SnakeBirdState &sbs, std::string prefix)
{
	Graphics::Display d;
	sb.Draw(d);
	sb.Draw(d, sbs);
	std::string fname = "/Users/nathanst/Desktop/SVG/"+prefix;
	int count = 0;
	while (fileExists((fname+std::to_string(count)+".svg").c_str()))
	{
		count++;
	}
	printf("Save to '%s'\n", (fname+std::to_string(count)+".svg").c_str());
	MakeSVG(d, (fname+std::to_string(count)+".svg").c_str(), 400, 400, 0);
}

void UpdateActiveSnake()
{
	if (future.size() > 0)
	{
		auto act = sb.GetAction(snake, future.back());
		if (snake.IsInPlay(act.bird))
			snakeControl = act.bird;
	}

	if (snakeControl >= snake.GetNumSnakes())
		snakeControl = 0;
	
	// If snake went into goal, switch active snake
	for (int x = 0; x < snake.GetNumSnakes(); x++)
	{
		if (snake.IsInPlay(snakeControl))
			break;
		snakeControl = (snakeControl+1)%snake.GetNumSnakes();
	}
}

bool NoActionsAvailable()
{
	return !sb.LivingState(snake);
//	static std::vector<SnakeBird::SnakeBirdAction> acts;
//	if (sb.GoalTest(snake, snake))
//		return false;
//	sb.GetActions(snake, acts);
//	return acts.size() == 0;
}

void LoadLevel19()
{
	sb.Reset();
	sb.SetGroundType(14, 5, SnakeBird::kFruit);

	sb.SetGroundType(1, 4, SnakeBird::kSpikes);
	sb.SetGroundType(1, 5, SnakeBird::kGround);
	sb.SetGroundType(4, 3, SnakeBird::kSpikes);
	sb.SetGroundType(4, 4, SnakeBird::kGround);
	sb.SetGroundType(3, 5, SnakeBird::kExit);

	sb.SetGroundType(4, 9, SnakeBird::kGround);
	sb.SetGroundType(4, 10, SnakeBird::kGround);
	sb.SetGroundType(4, 11, SnakeBird::kGround);
	sb.SetGroundType(4, 12, SnakeBird::kGround);

	sb.AddSnake(4, 6, {SnakeBird::kRight});
	sb.AddSnake(5, 7, {SnakeBird::kLeft, SnakeBird::kLeft, SnakeBird::kDown, SnakeBird::kDown, SnakeBird::kDown});
	sb.AddSnake(4, 8, {SnakeBird::kRight, SnakeBird::kDown, SnakeBird::kDown, SnakeBird::kDown});


	snake = sb.GetStart();
	history.clear();
	history.push_back(snake);
	future.clear();
	actionInProgressStep.Reset();
	timePerFrame = 0.01;
	UpdateActiveSnake();
}

void LoadLevel22()
{
	sb.Reset();
	sb.SetGroundType(3, 7, SnakeBird::kGround);
	sb.SetGroundType(3, 8, SnakeBird::kGround);
	sb.SetGroundType(3, 9, SnakeBird::kGround);
	sb.SetGroundType(3, 10, SnakeBird::kGround);
	sb.SetGroundType(4, 7, SnakeBird::kGround);
	sb.SetGroundType(4, 8, SnakeBird::kGround);
	sb.SetGroundType(4, 9, SnakeBird::kGround);
	sb.SetGroundType(4, 10, SnakeBird::kGround);
	sb.SetGroundType(5, 10, SnakeBird::kGround);
	sb.SetGroundType(6, 8, SnakeBird::kGround);
	sb.SetGroundType(6, 10, SnakeBird::kGround);
	sb.SetGroundType(7, 10, SnakeBird::kGround);
	sb.SetGroundType(8, 9, SnakeBird::kGround);
	sb.SetGroundType(8, 10, SnakeBird::kGround);
	sb.SetGroundType(9, 9, SnakeBird::kGround);
	sb.SetGroundType(9, 10, SnakeBird::kGround);
	
	sb.SetGroundType(5, 1, SnakeBird::kExit);
	
	sb.AddSnake(4, 6, {SnakeBird::kLeft, SnakeBird::kLeft});
	//		sb.AddSnake(10, 10, {SnakeBird::kRight});
	//		sb.AddSnake(15, 10, {SnakeBird::kRight});
	//		sb.AddSnake(20, 10, {SnakeBird::kRight});
	
	sb.SetGroundType(8, 7, SnakeBird::kBlock1);
	sb.SetGroundType(8, 8, SnakeBird::kBlock1);
	sb.SetGroundType(9, 7, SnakeBird::kBlock1);
	sb.SetGroundType(9, 8, SnakeBird::kBlock1);
		
	snake = sb.GetStart();
	history.clear();
	history.push_back(snake);
	future.clear();
	actionInProgressStep.Reset();
	timePerFrame = 0.01;
	UpdateActiveSnake();
}

void LoadLevel39()
{
	sb.Reset();
	for (int x = 3; x <= 14; x++)
		sb.SetGroundType(x, 12, SnakeBird::kGround);
	for (int x = 3; x <= 6; x++)
		sb.SetGroundType(x, 11, SnakeBird::kGround);
	for (int x = 9; x <= 14; x++)
		sb.SetGroundType(x, 11, SnakeBird::kGround);
	for (int x = 11; x <= 14; x++)
	{
		sb.SetGroundType(x, 9, SnakeBird::kGround);
		sb.SetGroundType(x, 6, SnakeBird::kGround);
	}
	for (int x = 10; x <= 14; x++)
	{
		sb.SetGroundType(x, 10, SnakeBird::kGround);
		sb.SetGroundType(x, 8, SnakeBird::kGround);
		sb.SetGroundType(x, 7, SnakeBird::kGround);
		sb.SetGroundType(x, 5, SnakeBird::kGround);
		sb.SetGroundType(x, 4, SnakeBird::kGround);
	}

	
	sb.SetGroundType(12, 1, SnakeBird::kExit);
	
	sb.AddSnake(6, 8, {SnakeBird::kLeft, SnakeBird::kLeft, SnakeBird::kDown, SnakeBird::kDown});
	//		sb.AddSnake(10, 10, {SnakeBird::kRight});
	//		sb.AddSnake(15, 10, {SnakeBird::kRight});
	//		sb.AddSnake(20, 10, {SnakeBird::kRight});
	
	sb.SetGroundType(5, 10, SnakeBird::kBlock1);
	sb.SetGroundType(6, 10, SnakeBird::kBlock1);
	sb.SetGroundType(5, 9, SnakeBird::kBlock2);
	sb.SetGroundType(6, 9, SnakeBird::kBlock2);
		
	snake = sb.GetStart();
	history.clear();
	history.push_back(snake);
	future.clear();
	actionInProgressStep.Reset();
	timePerFrame = 0.01;
	UpdateActiveSnake();
}

void LoadLevel32()
{
	sb.Reset();
	for (int x = 5; x <= 10; x++)
		sb.SetGroundType(x, 9, SnakeBird::kGround);
	for (int x = 11; x <= 12; x++)
		sb.SetGroundType(x, 9, SnakeBird::kSpikes);
	for (int x = 12; x <= 14; x++)
		sb.SetGroundType(x, 7, SnakeBird::kGround);
	for (int y = 5; y <= 7; y++)
		sb.SetGroundType(8, y, SnakeBird::kGround);
	for (int y = 4; y <= 5; y++)
		sb.SetGroundType(13, y, SnakeBird::kGround);

	sb.SetGroundType(6, 6, SnakeBird::kExit);
	
	sb.AddSnake(9, 7, {SnakeBird::kRight});
	sb.AddSnake(10, 8, {SnakeBird::kLeft});

	sb.SetGroundType(12, 6, SnakeBird::kBlock1);

	sb.SetGroundType(8, 8, SnakeBird::kPortal1);
	sb.SetGroundType(13, 6, SnakeBird::kPortal2);

	snake = sb.GetStart();
	history.clear();
	history.push_back(snake);
	future.clear();
	actionInProgressStep.Reset();
	timePerFrame = 0.01;
	UpdateActiveSnake();
}

void LoadLevel44()
{
	sb.Reset();
	sb.SetGroundType(4, 6, SnakeBird::kSpikes);
	sb.SetGroundType(6, 7, SnakeBird::kSpikes);
	sb.SetGroundType(4, 10, SnakeBird::kSpikes);
	sb.SetGroundType(11, 10, SnakeBird::kSpikes);
	for (int x = 5; x <= 8; x++)
		sb.SetGroundType(x, 10, SnakeBird::kGround);
	sb.SetGroundType(12, 10, SnakeBird::kGround);

	sb.SetGroundType(5, 6, SnakeBird::kFruit);
	sb.SetGroundType(8, 5, SnakeBird::kFruit);

	sb.SetGroundType(12, 5, SnakeBird::kExit);
	
	sb.AddSnake(5, 9, {SnakeBird::kRight});
	sb.AddSnake(6, 8, {SnakeBird::kLeft});

	sb.SetGroundType(6, 4, SnakeBird::kPortal1);
	sb.SetGroundType(8, 8, SnakeBird::kPortal2);

	snake = sb.GetStart();
	history.clear();
	history.push_back(snake);
	future.clear();
	actionInProgressStep.Reset();
	timePerFrame = 0.01;
	UpdateActiveSnake();
}

void LoadLevel63()
{
	sb.Reset();
	for (int x = 5; x <= 11; x++)
		sb.SetGroundType(x, 6, SnakeBird::kGround);
	for (int x = 13; x <= 13; x++)
		sb.SetGroundType(x, 6, SnakeBird::kGround);
	for (int x = 14; x <= 15; x++)
		sb.SetGroundType(x, 5, SnakeBird::kGround);
	for (int x = 13; x <= 15; x++)
		sb.SetGroundType(x, 4, SnakeBird::kGround);

	sb.SetGroundType(14, 2, SnakeBird::kExit);
	
	sb.AddSnake(10, 5, {SnakeBird::kRight});

	sb.SetGroundType(10, 4, SnakeBird::kBlock1);
	sb.SetGroundType(9, 3, SnakeBird::kBlock1);
	sb.SetGroundType(10, 3, SnakeBird::kBlock1);
	sb.SetGroundType(11, 3, SnakeBird::kBlock1);

	sb.SetGroundType(6, 3, SnakeBird::kPortal1);
	sb.SetGroundType(13, 5, SnakeBird::kPortal2);

	snake = sb.GetStart();
	history.clear();
	history.push_back(snake);
	future.clear();
	actionInProgressStep.Reset();
	timePerFrame = 0.01;
	UpdateActiveSnake();
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
		if (mapName.size() != 0)
		{
			sb.Load(mapName.c_str());
			snake = sb.GetStart();
			history.push_back(snake);
		}
		else {
			//LoadLevel19();
			//LoadLevel22();
			//LoadLevel39();
			LoadLevel63();
			//LoadLevel32();
			//LoadLevel44();
		}
		worldClock.StartTimer();
		lastFrameStart = worldClock.GetElapsedTime();
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	SetNumPorts(windowID, 1);
	frameRate = worldClock.EndTimer()-lastFrameStart;
	lastFrameStart = worldClock.EndTimer();
	if (recording)
		frameRate = 1.0/60.0; // fixed frame rate when recording
//	printf("%f\n", frameRate);
	
	globalTime += frameRate;
	frameTime += frameRate;
	if (frameTime > timePerFrame)
	{
		frameTime = 0;
		if (actionInProgress == true)
		{
			lastFrameSnake = snake;
			bool complete = sb.ApplyPartialAction(snake, inProgress, actionInProgressStep);
			actionInProgress = !complete;
			timePerFrame = actionInProgressStep.animationDuration;

			if (!actionInProgress)
			{
				if (NoActionsAvailable())
					MyDisplayHandler(windowID, kNoModifier, 'q');
				UpdateActiveSnake();
			}
			
			if (actionInProgress == false && autoSolve == true)
			{
				MyDisplayHandler(windowID, kNoModifier, ']');
			}
		}
		else {
			autoSolve = false;
			lastFrameSnake = snake;
			if (NoActionsAvailable())
			{
				MyDisplayHandler(windowID, kNoModifier, 'q');
			}
		}
	}
	Graphics::Display &d = GetContext(windowID)->display;
		
	sb.Draw(d, globalTime);
	//sb.Draw(d, snake, snakeControl, globalTime);
	sb.Draw(d, lastFrameSnake, snake, snakeControl, frameTime/timePerFrame, globalTime);

	if (sb.GoalTest(snake))
	{
		transition.Draw(d);
		if (transition.Step(frameRate))
		{
			d.DrawText("Great Job", {0,0}, Colors::black, 0.25f, Graphics::textAlignCenter);
			d.DrawText("Level Passed!", {0,0.25}, Colors::black, 0.25f, Graphics::textAlignCenter);
		}
	}
	else {
		transition.Reset(0);
	}
	
	if (recording)
	{
		std::string fname = "/Users/nathanst/Movies/tmp/sb_";
		static int count = 0;
		while (fileExists((fname+std::to_string(count)+".svg").c_str()))
		{
			count++;
		}
		printf("Save to '%s'\n", (fname+std::to_string(count)+".svg").c_str());
		MakeSVG(d, (fname+std::to_string(count)+".svg").c_str(), 400, 400, 0);
	}
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-load") == 0)
	{
		if (maxNumArgs > 1)
		{
			mapName = argument[1];
			return 2;
		}
		printf("Failed -load <file>: missing file name");
		return 1;
	}
	if (strcmp(argument[0], "-svg") == 0)
	{
		if (maxNumArgs > 2)
		{
			Graphics::Display d;
			sb.Load(argument[1]);
			snake = sb.GetStart();
			sb.Draw(d);
			sb.Draw(d, snake, snakeControl);
			std::string fname = argument[2];
			MakeSVG(d, argument[2], 400, 400, 0);
		}
		else
			printf("Failed -svg <file>: missing file name");
		exit(0);
	}
	if (strcmp(argument[0], "-bfs") == 0)
	{
		if (maxNumArgs > 1)
		{
			sb.Load(argument[1]);			
			snake = sb.GetStart();
			MyDisplayHandler(0, tKeyboardModifier::kNoModifier, 'b');
			exit(0);	
		}	
	}

	return 0;
}

void HurryUpFinishAction()
{
	while (actionInProgress == true)
	{
		bool complete = sb.ApplyPartialAction(snake, inProgress, actionInProgressStep);
		actionInProgress = !complete;
	}
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
//	SnakeBird::SnakeBirdAction a;
	static std::vector<SnakeBird::SnakeBirdAction> acts;
//	a.bird = snakeControl;
	switch (key)
	{
		case '0': LoadLevel19(); break;
		case '1': LoadLevel22(); break;
		case '2': LoadLevel32(); break;
		case '3': LoadLevel39(); break;
		case '4': LoadLevel44(); break;
		case '5': LoadLevel63(); break;
		case 'w':
			future.clear();
			HurryUpFinishAction();
			sb.GetActions(snake, acts);
			for (auto &a : acts)
			{
				if (a.bird == snakeControl && a.direction == SnakeBird::kUp)
				{
					//sb.ApplyAction(snake, a);
					actionInProgress = true;
					actionInProgressStep.Reset();
					inProgress = a;
					frameTime = timePerFrame;
					history.push_back(snake);
				}
			}
			UpdateActiveSnake();
			break;
		case 's':
			future.clear();
			HurryUpFinishAction();
			sb.GetActions(snake, acts);
			for (auto &a : acts)
			{
				if (a.bird == snakeControl && a.direction == SnakeBird::kDown)
				{
					actionInProgress = true;
					actionInProgressStep.Reset();
					inProgress = a;
					frameTime = timePerFrame;
					history.push_back(snake);

//					sb.ApplyAction(snake, a);
//					history.push_back(snake);
				}
			}
			UpdateActiveSnake();
			break;
		case 'a':
			future.clear();
			HurryUpFinishAction();
			sb.GetActions(snake, acts);
			for (auto &a : acts)
			{
				if (a.bird == snakeControl && a.direction == SnakeBird::kLeft)
				{
					actionInProgress = true;
					actionInProgressStep.Reset();
					inProgress = a;
					frameTime = timePerFrame;
					history.push_back(snake);
//					sb.ApplyAction(snake, a);
//					history.push_back(snake);
				}
			}
			UpdateActiveSnake();
			break;
		case 'd':
			future.clear();
			HurryUpFinishAction();
			sb.GetActions(snake, acts);
			for (auto &a : acts)
			{
				if (a.bird == snakeControl && a.direction == SnakeBird::kRight)
				{
					actionInProgress = true;
					actionInProgressStep.Reset();
					inProgress = a;
					frameTime = timePerFrame;
					history.push_back(snake);
//					sb.ApplyAction(snake, a);
//					history.push_back(snake);
				}
			}
			UpdateActiveSnake();
			break;
		case 'q':
			future.clear();
			if (history.size() > 1)
			{
				snake = history.back();
				lastFrameSnake = snake;
				timePerFrame = 0.001;
				history.pop_back();
			}
			UpdateActiveSnake();
			break;
		case 'e':
			for (int x = 0; x < snake.GetNumSnakes(); x++)
			{
				snakeControl = (snakeControl+1)%snake.GetNumSnakes();
				if (snake.IsInPlay(snakeControl))
					break;
			}
			break;
		case ']':
			HurryUpFinishAction();
			if (future.size() > 0)
			{
				actionInProgress = true;
				actionInProgressStep.Reset();
				inProgress = sb.GetAction(snake, future.back());
				frameTime = timePerFrame;
				history.push_back(snake);
				//snake = future.back();
				future.pop_back();
			}
			break;
		case 'n':
			MyDisplayHandler(windowID, kNoModifier, 'b');
			autoSolve = true;
			MyDisplayHandler(windowID, kNoModifier, ']');
			break;
		case 'c':
			AnalyzeMapChanges();
			break;
		case 'b':
		{
			Timer t;
			BFS<SnakeBird::SnakeBirdState, SnakeBird::SnakeBirdAction, SnakeBird::SnakeBird> bfs;
			//bfs.SetNodeLimit(20000000);
			//bfs.SetNodeLimit(75000000); // max 75 million expansions
			bfs.SetNodeLimit(150000000); // max 150 million expansions
			//bfs.DoBFS(&sb, snake);
			t.StartTimer();
			bfs.GetPath(&sb, snake, snake, future);
			t.EndTimer();
			printf("BFS to goal complete in %1.2fs: %llu states reached; path length %lu.\n",
				   t.GetElapsedTime(), bfs.GetNodesExpanded(), future.size());

			if (future.size() > 0)
			{
//				for (int x = 0; x < future.size()-1; x++)
//				{
//					//Save(sb, future[x], "debug-");
//					std::cout << sb.GetAction(future[x], future[x+1]) << "\n";
//				}
//				std::cout << "\n";
				std::reverse(future.begin(), future.end());
				// remove the current state
				future.pop_back();
				UpdateActiveSnake();
			}
			break;
		}
		case 'p':
		{
			Graphics::Display d;
			sb.Draw(d, globalTime);
			sb.Draw(d, lastFrameSnake, snake, snakeControl, frameTime/timePerFrame, globalTime);
			std::string fname = "/Users/nathanst/Desktop/SVG/sb_";
			int count = 0;
			while (fileExists((fname+std::to_string(count)+".svg").c_str()))
			{
				count++;
			}
			printf("Save to '%s'\n", (fname+std::to_string(count)+".svg").c_str());
			MakeSVG(d, (fname+std::to_string(count)+".svg").c_str(), 400, 400, 0);
		}
		case 'o':
			recording = !recording;
			break;
		case 't':
			break;
		case 'v':
			break;
		case 'r':
			snake = history[0];
			timePerFrame = 0.01;
			history.resize(1);
			break;
		case '\t':
			if (mod != kShiftDown)
				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
			else
			{
				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
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
	}
	if (e == kMouseMove)
	{
	}

	// Don't need any other mouse support
	return true;
}

void AnalyzeMapChanges()
{
	BFS<SnakeBird::SnakeBirdState, SnakeBird::SnakeBirdAction, SnakeBird::SnakeBird> bfs;
	bfs.SetNodeLimit(1000000); // max 1 million expansions
	bfs.SetVerbose(false);
	std::vector<SnakeBird::SnakeBirdState> path;

	size_t maxLength = 0;
	SnakeBird::SnakeBird best;
	SnakeBird::SnakeBirdState current = sb.GetStart();
	SnakeBird::SnakeBird curr = sb;
	std::vector<SnakeBird::SnakeBirdWorldObject> order;
	for (int x = 0; x < sb.GetWidth(); x++)
	{
		for (int y = 0; y < sb.GetHeight()-1; y++)
		{
			auto renderedType = sb.GetRenderedGroundType(current, x, y);
			bool valid = false;
			switch (renderedType)
			{
				case SnakeBird::kGround:
				{
					valid = true;
//					order.push_back(SnakeBird::kGround);
					order.push_back(SnakeBird::kEmpty);
//					order.push_back(SnakeBird::kFruit);
					order.push_back(SnakeBird::kSpikes);
				}
				case SnakeBird::kSpikes:
				{
					valid = true;
//					order.push_back(SnakeBird::kSpikes);
					order.push_back(SnakeBird::kGround);
					order.push_back(SnakeBird::kEmpty);
//					order.push_back(SnakeBird::kFruit);
				}
//				case SnakeBird::kFruit:
//				{
////					order.push_back(SnakeBird::kFruit);
//					order.push_back(SnakeBird::kSpikes);
//					order.push_back(SnakeBird::kGround);
//					order.push_back(SnakeBird::kEmpty);
//				}
				case SnakeBird::kEmpty:
				{
					valid = true;
//					order.push_back(SnakeBird::kEmpty);
//					order.push_back(SnakeBird::kFruit);
					order.push_back(SnakeBird::kSpikes);
					order.push_back(SnakeBird::kGround);
				}
				default:
					break;
			}
	
			while (order.size() > 0)
			{
				sb.SetGroundType(x, y, order.back());
				order.pop_back();
				bfs.GetPath(&sb, snake, snake, path);
				if (path.size() > maxLength)
				{
					maxLength = path.size();
					best = sb;
				}
			}
			sb = curr;//sb.SetStart(current);
		}
	}
	bfs.GetPath(&sb, snake, snake, path);
	printf("Old: %lu, ", path.size());
	sb = best;
	bfs.GetPath(&sb, snake, snake, path);
	printf("New: %lu\n", path.size());
}
