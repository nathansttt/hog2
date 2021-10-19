/*
 *  $Id: Driver.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 7/16/21.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "Common.h"
#include "Driver.h"
#include <string>
#include "RC.h"

#include "cubeHolder.h"

RCState cube;
RC env;

//static CubeHolder *cubeHolder;
int moveType = 0;

int main(int argc, char* argv[])
{
	//MAKE THE CUBES (Drawing)
	//cubeHolder = new CubeHolder();

	InstallHandlers();
	RunHOGGUI(argc, argv, 1600, 800);

	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Optimal", "Show optimal solution", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Randomize", "Get Random State", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Turn0", "Turn Face 0", kAnyModifier, '0');
	InstallKeyboardHandler(MyDisplayHandler, "Turn1", "Turn Face 1", kAnyModifier, '1');
	InstallKeyboardHandler(MyDisplayHandler, "Turn2", "Turn Face 2", kAnyModifier, '2');
	InstallKeyboardHandler(MyDisplayHandler, "Turn3", "Turn Face 3", kAnyModifier, '3');
	InstallKeyboardHandler(MyDisplayHandler, "Turn4", "Turn Face 4", kAnyModifier, '4');
	InstallKeyboardHandler(MyDisplayHandler, "Turn5", "Turn Face 5", kAnyModifier, '5');
	InstallKeyboardHandler(MyDisplayHandler, "MoveType", "Choose Move Type", kAnyModifier, 'm');
	InstallKeyboardHandler(MyDisplayHandler, "TurnStop", "Stop Cube Passive Rotation", kAnyModifier, 'n');
	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler, static_cast<tMouseEventType>(kMouseMove|kMouseDown));
	srandom(time(0));
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

		//glClearColor(0.99, 0.99, 0.99, 1.0);
		InstallFrameHandler(MyFrameHandler, windowID, 0);

		ReinitViewports(windowID, {-1, -1, 1, 1}, kScaleToSquare);
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	display.FillRect({-1, -1, 1, 1}, Colors::white);

	//Draw here!!!
	env.Draw(display, cube);
	env.TestUpdate();
	env.DrawCubies(display);
	
	return;
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	return 2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 'm':
			moveType++;
			if (moveType > 2) moveType = 0;
			std::cout << "Set Movetype to: " + std::to_string(moveType) << '\n';
			break;	
		case 'n':
			env.passiveRot = !env.passiveRot;		
			break;	
		case '0':
			env.RotateFace(0,moveType);
			cube.RotateFace(0,moveType);
			break;	
		case '1':
			env.RotateFace(1,moveType);
			cube.RotateFace(1,moveType);
			break;	
		case '2':
			env.RotateFace(2,moveType);
			cube.RotateFace(2,moveType);
			break;	
		case '3':
			env.RotateFace(3,moveType);
			cube.RotateFace(3,moveType);
			break;	
		case '4':
			env.RotateFace(4,moveType);
			cube.RotateFace(4,moveType);
			break;	
		case '5':
			env.RotateFace(5,moveType);
			cube.RotateFace(5,moveType);
			break;
			
		case 'o':
			break;
		case 's':
		{
			// TODO: When hash function is finished you can use it to get random states 
			break;
		}
		case 'r':
		{
			// TODO: Reset state
			//curr = goal;
			break;
		}
		case '?':
		{
		}
			break;
		default:
			break;
	}
}


bool MyClickHandler(unsigned long , int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
	// TODO: add interactions later
	switch (mType)
	{
	case kMouseMove:
		{
		}
		break;
	case kMouseDown:
		{
		}
	default: return true;
	}
	return true;
}

