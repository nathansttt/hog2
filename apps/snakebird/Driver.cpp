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

bool mapAlreadyLoaded = false;
bool recording = false;
bool quitWhenDoneRecording = false;
bool autoSolve = false;
double globalTime = 0;
double frameTime = 0; // seconds elapsed in current frame
double timePerFrame = 0.1; // seconds for drawing each step of frame
double frameRate = 1.0/30.0;
SnakeBird::SnakeBirdAnimationStep actionInProgressStep;
bool actionInProgress = false;
SnakeBird::SnakeBirdAction inProgress;
bool gRefreshBackground = true;
std::string message = "Welcome to Anhinga! Eat the grapes (if present) and leave via the yellow exit.";
double messageBeginTime = globalTime-1;
double messageExpireTime = globalTime+10;

Timer worldClock;
double lastFrameStart;
LineTransition transition(20, 60, Colors::white);

SnakeBird::SnakeBird sb(20, 20);
SnakeBird::SnakeBirdState snake;
SnakeBird::SnakeBirdState lastFrameSnake;
int snakeControl = 0;
std::vector<SnakeBird::SnakeBirdState> history;
std::vector<SnakeBird::SnakeBirdState> future;

void AnalyzeMapChanges(bool maximize, int nodeLimit=1000000);


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
	InstallKeyboardHandler(MyDisplayHandler, "Up", "Move up", kAnyModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Up", "Move up", kAnyModifier, kUpArrow);
	InstallKeyboardHandler(MyDisplayHandler, "Down", "Move down", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Down", "Move down", kAnyModifier, kDownArrow);
	InstallKeyboardHandler(MyDisplayHandler, "Left", "Move left", kAnyModifier, 'a');
	InstallKeyboardHandler(MyDisplayHandler, "Left", "Move left", kAnyModifier, kLeftArrow);
	InstallKeyboardHandler(MyDisplayHandler, "Right", "Move right", kAnyModifier, 'd');
	InstallKeyboardHandler(MyDisplayHandler, "Right", "Move right", kAnyModifier, kRightArrow);
	InstallKeyboardHandler(MyDisplayHandler, "Undo", "Undo last move", kAnyModifier, 'q');
	InstallKeyboardHandler(MyDisplayHandler, "Next", "Select Next Snake", kAnyModifier, 'e');
	InstallKeyboardHandler(MyDisplayHandler, "Run", "Run solution (build if needed)", kAnyModifier, 'n');
	InstallKeyboardHandler(MyDisplayHandler, "Reset", "Reset Level", kAnyModifier, 'r');

#ifndef __EMSCRIPTEN__
	InstallKeyboardHandler(MyDisplayHandler, "Level", "Goto nth level", kAnyModifier, '0', '5');
	InstallKeyboardHandler(MyDisplayHandler, "Print", "Print screen to file", kAnyModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Movie", "Record movie frames", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Solve", "Build solution", kAnyModifier, 'b');
	InstallKeyboardHandler(MyDisplayHandler, "Step", "Take next step in solution", kAnyModifier, ']');
#endif
	InstallKeyboardHandler(MyDisplayHandler, "Change", "Make one change to increase solution length", kAnyModifier, 'c');
	InstallKeyboardHandler(MyDisplayHandler, "Change", "Make one change to decrease solution length", kAnyModifier, 'x');
	
	
	InstallCommandLineHandler(MyCLHandler, "-change", "-change <input> <nodelimit> <iter>", "Run <iter> times to make the level harder or easier. Outputs the new depth and file coding.");
	InstallCommandLineHandler(MyCLHandler, "-load", "-load <file>", "Run snake bird with the given file");
	InstallCommandLineHandler(MyCLHandler, "-svg", "-svg <input> <output>", "Make an .svg of the given level then quit");
	InstallCommandLineHandler(MyCLHandler, "-bfs", "-bfs <file>", "Run BFS on the given level and return the info");
	InstallCommandLineHandler(MyCLHandler, "-encode", "-encode <file>", "Encode level as string");
	InstallCommandLineHandler(MyCLHandler, "-decode", "-decode <string>", "Decode level from string");
	InstallCommandLineHandler(MyCLHandler, "-solve", "-solve <level>", "Solve level and output .svg files for the full solution");
//	InstallCommandLineHandler(MyCLHandler, "-test", "-test", "Basic test with MD heuristic");
	
	InstallWindowHandler(MyWindowHandler);
	InstallMouseClickHandler(MyClickHandler, static_cast<tMouseEventType>(kMouseMove|kMouseUp|kMouseDown|kMouseDrag));
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

bool SnakeDead()
{
	return !sb.LivingState(snake);
}

bool NoActions()
{
	static std::vector<SnakeBird::SnakeBirdAction> acts;
	if (sb.GoalTest(snake, snake))
		return false;
	sb.GetActions(snake, acts);
	if (acts.size() == 0)
		return true;
	if (acts.size() > 1)
		return false;
	if (acts[0].direction != SnakeBird::kUp)
		return false;
	SnakeBird::SnakeBirdState tmp = snake;
	sb.ApplyAction(tmp, acts[0]);
	if (tmp == snake)
		return true;
	return false;
}

void LoadLevel19()
{
	sb.BeginEditing();
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


	lastFrameSnake = snake = sb.GetStart();
	history.clear();
	history.push_back(snake);
	future.clear();
	actionInProgressStep.Reset();
	timePerFrame = 0.01;
	UpdateActiveSnake();
	sb.EndEditing();
	gRefreshBackground = true;
}

void LoadLevel22()
{
	sb.BeginEditing();
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
		
	lastFrameSnake = snake = sb.GetStart();
	history.clear();
	history.push_back(snake);
	future.clear();
	actionInProgressStep.Reset();
	timePerFrame = 0.01;
	UpdateActiveSnake();
	sb.EndEditing();
	gRefreshBackground = true;
}

void LoadLevel39()
{
	sb.BeginEditing();
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
		
	lastFrameSnake = snake = sb.GetStart();
	history.clear();
	history.push_back(snake);
	future.clear();
	actionInProgressStep.Reset();
	timePerFrame = 0.01;
	UpdateActiveSnake();
	sb.EndEditing();
	gRefreshBackground = true;
}

void LoadLevel32()
{
	sb.BeginEditing();
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

	lastFrameSnake = snake = sb.GetStart();
	history.clear();
	history.push_back(snake);
	future.clear();
	actionInProgressStep.Reset();
	timePerFrame = 0.01;
	UpdateActiveSnake();
	sb.EndEditing();
	gRefreshBackground = true;
}

void LoadLevel44()
{
	sb.BeginEditing();
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

	lastFrameSnake = snake = sb.GetStart();
	history.clear();
	history.push_back(snake);
	future.clear();
	actionInProgressStep.Reset();
	timePerFrame = 0.01;
	UpdateActiveSnake();
	sb.EndEditing();
	gRefreshBackground = true;
}

void LoadLevel63()
{
	sb.BeginEditing();
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

	lastFrameSnake = snake = sb.GetStart();
	history.clear();
	history.push_back(snake);
	future.clear();
	actionInProgressStep.Reset();
	timePerFrame = 0.01;
	UpdateActiveSnake();
	sb.EndEditing();
	gRefreshBackground = true;
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
		if (!mapAlreadyLoaded)
		{
			//LoadLevel19();
			//LoadLevel22();
			//LoadLevel39();
			LoadLevel63();
			//LoadLevel32();
			//LoadLevel44();
		}
		if (sb.GetNumFruit() == 0)
			message = "Welcome to Anhinga! Leave via the yellow exit to pass the level.";
		else
			message = "Welcome to Anhinga! Eat the grapes and then leave via the yellow exit.";

		worldClock.StartTimer();
		lastFrameStart = worldClock.GetElapsedTime();
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	frameRate = worldClock.EndTimer()-lastFrameStart;
	lastFrameStart = worldClock.EndTimer();
	if (recording)
		frameRate = 1.0/60.0; // fixed frame rate when recording
//	printf("%f\n", frameRate);
//	frameRate = 1.0/120.0; // fixed frame rate when recording
//	frameRate /= 20;
	
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
				if (SnakeDead())
					MyDisplayHandler(windowID, kNoModifier, 'q');
				UpdateActiveSnake();
				
				if (autoSolve == false && NoActions())
				{
					message = "No legal actions; press undo or reset level to continue";
					messageBeginTime = globalTime+0.5;
					messageExpireTime = globalTime+10;
				}

			}
			
			if (actionInProgress == false && autoSolve == true)
			{
				MyDisplayHandler(windowID, kNoModifier, ']');
			}
		}
		else {
			autoSolve = false;
			lastFrameSnake = snake;
			if (SnakeDead())
			{
				MyDisplayHandler(windowID, kNoModifier, 'q');
			}
		}
	}
	Graphics::Display &d = GetContext(windowID)->display;
	if (gRefreshBackground)
	{
		d.StartBackground();
		sb.Draw(d);
		d.EndBackground();
	}
	//sb.Draw(d, snake, snakeControl, globalTime);
	sb.Draw(d, globalTime);
	sb.Draw(d, lastFrameSnake, snake, snakeControl, frameTime/timePerFrame, globalTime);

	if (sb.GoalTest(snake) && actionInProgress == false && quitWhenDoneRecording == false )
	{
		message = "";
		if (transition.Step(frameRate*3))
		{
			d.DrawText("Great Job", {0,0}, Colors::black, 0.25f, Graphics::textAlignCenter);
			d.DrawText("Level Passed!", {0,0.25}, Colors::black, 0.25f, Graphics::textAlignCenter);
#ifdef __EMSCRIPTEN__
			submitTextToBuffer("Done");
			d.DrawText("Window will close automatically", {0,0.4}, Colors::black, 0.075f, Graphics::textAlignCenter);
#endif
		}
		transition.Draw(d);
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

		if (!autoSolve && quitWhenDoneRecording)
		{
			exit(0);
		}
	}
	
	if (messageExpireTime >= globalTime && messageBeginTime <= globalTime)
	{
		rgbColor c = Colors::black;
		if (messageExpireTime-globalTime < 1.0)
		{
			rgbColor tmp = rgbColor::mix(Colors::cyan, Colors::lightblue, 0.5);
			c.mix(tmp, 1-(messageExpireTime-globalTime));
		}
		else if (globalTime - messageBeginTime < 1.0)
		{
			rgbColor tmp = rgbColor::mix(Colors::cyan, Colors::lightblue, 0.5);
			c.mix(tmp, 1-(globalTime-messageBeginTime));
		}
		d.DrawText(message.c_str(), {-1+sb.GetRadius(),-1+1.5f*sb.GetRadius()}, c, sb.GetRadius(), Graphics::textAlignLeft, "Helvetica");
	}
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-load") == 0)
	{
		if (maxNumArgs > 1)
		{
			sb.Load(argument[1]);
			lastFrameSnake = snake = sb.GetStart();
			history.push_back(snake);
			mapAlreadyLoaded = true;
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
			if (!sb.DecodeLevel(argument[1]))
				sb.Load(argument[1]);
			lastFrameSnake = snake = sb.GetStart();
			sb.Draw(d);
			sb.Draw(d, 0);
			sb.Draw(d, snake, snakeControl);
			std::string fname = argument[2];
			MakeSVG(d, argument[2], 400, 400, 0);
		}
		else
			printf("Failed -svg <file>: missing file name");
		exit(0);
	}
	// -change <input> <nodelimit> <iter>
	if (strcmp(argument[0], "-change") == 0)
	{
		if (maxNumArgs < 4)
		{
			printf("Insufficient arguments: -change <input> <nodelimit> <iter>\n");
			return 0;
		}
		if (maxNumArgs > 4)
			printf("Ignoring extra input: '%s'\n", argument[4]);

		if (!sb.DecodeLevel(argument[1]))
			sb.Load(argument[1]);
		lastFrameSnake = snake = sb.GetStart();
		
		BFS<SnakeBird::SnakeBirdState, SnakeBird::SnakeBirdAction, SnakeBird::SnakeBird> bfs;
		bfs.SetNodeLimit(atoi(argument[2])); // max 150 million expansions
		bfs.SetVerbose(false);
		bfs.GetPath(&sb, snake, snake, future);
		if (future.size() == 0)
		{
			printf("Length 0\n%s\n", sb.EncodeLevel().c_str());			
			exit(0);
		}
		printf("Length %lu\n", future.size());
		
		int iter = atoi(argument[3]);
		bool harden = iter>0;
		iter = abs(iter);
		for (int x = 0; x < iter; x++)
			AnalyzeMapChanges(harden, atoi(argument[2]));
		std::cout << sb.EncodeLevel() << "\n";
		exit(0);
	}
	if (strcmp(argument[0], "-bfs") == 0)
	{
		if (maxNumArgs > 1)
		{
			sb.Load(argument[1]);			
			lastFrameSnake = snake = sb.GetStart();
			MyDisplayHandler(0, tKeyboardModifier::kNoModifier, 'b');
			exit(0);	
		}	
	}
	if (strcmp(argument[0], "-encode") == 0)
	{
		if (maxNumArgs > 1)
		{
			sb.Load(argument[1]);
			auto str = sb.EncodeLevel();
			std::cout << str << "\n";
			sb.DecodeLevel(sb.EncodeLevel());
			auto str2 = sb.EncodeLevel();
			if (str != str2)
				std::cerr << "ENCODING FAILURE!\n";
			exit(0);
		}
	}
	if (strcmp(argument[0], "-decode") == 0)
	{
		if (maxNumArgs > 1)
		{
			sb.DecodeLevel(argument[1]);
			if (strcmp(argument[1], sb.EncodeLevel().c_str()) != 0)
			{
				printf("Decode failure:\n");
				printf("%s\n", argument[1]);
				printf("%s\n", sb.EncodeLevel().c_str());
			}
			lastFrameSnake = snake = sb.GetStart();
			history.push_back(snake);
			mapAlreadyLoaded = true;
			return 2;
		}
		return 1;
	}
	if (strcmp(argument[0], "-solve") == 0)
	{
		if (maxNumArgs > 1)
		{
			if (!sb.DecodeLevel(argument[1]))
				sb.Load(argument[1]);
			lastFrameSnake = snake = sb.GetStart();
			history.push_back(snake);
			mapAlreadyLoaded = true;
			recording = true;
			quitWhenDoneRecording = true;
			MyDisplayHandler(0, tKeyboardModifier::kNoModifier, 'n');
			return 2;
		}
		return 1;
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
		case kUpArrow:
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
		case kDownArrow:
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
		case kLeftArrow:
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
		case kRightArrow:
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
			//if (message == "No legal actions; press undo or reset level to continue")
			message = "";
			future.clear();
			if (history.size() > 1)
			{
				while (snake == history.back() && history.size() > 2)
					history.pop_back();
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
			AnalyzeMapChanges(true);
			break;
		case 'x':
			AnalyzeMapChanges(false);
			break;
		case 'b':
		{
			Timer t;
			BFS<SnakeBird::SnakeBirdState, SnakeBird::SnakeBirdAction, SnakeBird::SnakeBird> bfs;
			//bfs.SetNodeLimit(20000000);
			//bfs.SetNodeLimit(75000000); // max 75 million expansions
			bfs.SetNodeLimit(250000000); // max 250 million expansions
			//bfs.DoBFS(&sb, snake);
			t.StartTimer();
			bfs.GetPath(&sb, snake, snake, future);
			t.EndTimer();
			printf("BFS to goal complete in %1.2fs: %llu states reached; path length %lu.\n",
				   t.GetElapsedTime(), bfs.GetNodesExpanded(), future.size());

			if (future.size() == 0)
			{
				message = "Level unsolvable. Reset or undo actions and try again.";
				messageExpireTime = globalTime + 5;
			}
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
			sb.Draw(d);
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
			break;
		case 'o':
			recording = !recording;
			break;
		case 't':
			break;
		case 'v':
			break;
		case 'r':
			message = "";
			snake = history[0];
			timePerFrame = 0.01;
			history.resize(1);
			future.clear();
			autoSolve = false;
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
	
	if (e == kMouseDown)
	{
		int x, y;
		if (sb.GetPointFromCoordinate(p, x, y))
		{
			printf("Hit (%f, %f) -> (%d, %d)\n", p.x, p.y, x, y);
		}
	}
	if (e == kMouseUp)
	{
	}
	if (e == kMouseMove)
	{
	}

	// Don't need any other mouse support
	return true;
}

void AnalyzeMapChanges(bool maximize, int nodeLimit)
{
	BFS<SnakeBird::SnakeBirdState, SnakeBird::SnakeBirdAction, SnakeBird::SnakeBird> bfs;
	bfs.SetNodeLimit(nodeLimit); // max 1 million expansions
	bfs.SetVerbose(false);
	std::vector<SnakeBird::SnakeBirdState> path;

	size_t maxLength = 0;
	size_t minLength = 100000;
	SnakeBird::SnakeBird bestMin, bestMax;
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
					order.push_back(SnakeBird::kEmpty);
					order.push_back(SnakeBird::kSpikes);
					break;
				}
				case SnakeBird::kSpikes:
				{
					valid = true;
					order.push_back(SnakeBird::kGround);
					order.push_back(SnakeBird::kEmpty);
					break;
				}
				case SnakeBird::kEmpty:
				{
					valid = true;
					order.push_back(SnakeBird::kSpikes);
					order.push_back(SnakeBird::kGround);
					break;
				}
				default:
					break;
			}
	
			while (order.size() > 0)
			{
				sb.BeginEditing();
				sb.SetGroundType(x, y, order.back());
				sb.EndEditing();
				gRefreshBackground = true;
				order.pop_back();
				bfs.GetPath(&sb, snake, snake, path);
				if (path.size() < minLength && path.size() != 0)
				{
					minLength = path.size();
					bestMin = sb;
				}
				if (path.size() > maxLength)
				{
					maxLength = path.size();
					bestMax = sb;
				}
			}
			sb = curr;//sb.SetStart(current);
		}
	}
	bfs.GetPath(&sb, snake, snake, path);
	printf("Old: %lu, ", path.size());
	if (maximize)
		sb = bestMax;
	else
		sb = bestMin;
	bfs.GetPath(&sb, snake, snake, path);
	printf("New: %lu\n", path.size());
}
