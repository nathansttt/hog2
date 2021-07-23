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

RCState cube;
RC env;

int main(int argc, char* argv[])
{
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
	
	env.Draw(display, cube);

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

