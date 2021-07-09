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

#include "Common.h"
#include "Driver.h"
#include "GraphEnvironment.h"
#include <string>
#include "Racetrack.h"

bool recording = false;
bool running = false;

Map *m = 0;
Racetrack *r = 0;
RacetrackState s;
RacetrackMove v;

// -------------- MAIN FUNCTION ----------- //
int main(int argc, char* argv[])
{
	InstallHandlers();
	std::cout << "This is working yep" << std::endl;
	RunHOGGUI(argc, argv, 1600, 800);
	return 0;
}

// -- Racecar -- //


/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Reset", "Reset to start state", kAnyModifier, 'r');
	// TODO: Add new handlers to enable use of WASD -- done
	InstallKeyboardHandler(MyDisplayHandler, "Up", "Accelerate upwards", kAnyModifier, kUpArrow);
	InstallKeyboardHandler(MyDisplayHandler, "Down", "Accelerate downwards", kAnyModifier, kDownArrow);
	InstallKeyboardHandler(MyDisplayHandler, "Left", "Accelerate left", kAnyModifier, kLeftArrow);
	InstallKeyboardHandler(MyDisplayHandler, "Right", "Accelerate right", kAnyModifier, kRightArrow);
	// --- WASD handlers --- //
	InstallKeyboardHandler(MyDisplayHandler, "W", "Accelerate upwards", kAnyModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "A", "Accelerate Left", kAnyModifier, 'a');
	InstallKeyboardHandler(MyDisplayHandler, "S", "Accelerate downwards", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "D", "Accelerate Right", kAnyModifier, 'd');

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
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		ReinitViewports(windowID, {-1, -1, 1, 1}, kScaleToSquare);
		
		m = new Map(11, 11);
		m->SetTerrainType(0, 0, kStartTerrain);
		for (int x = 0; x < 5; x++)
		{
			m->SetTerrainType(x, 5, kTrees);
		}

		for (int x = 0; x < 7; x++)
		{
			m->SetTerrainType(x, 9, kEndTerrain);
		}
		r = new Racetrack(m);
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	display.FillRect({-1, -1, 1, 1}, Colors::black);
	
	// Draw map
	r->Draw(display);
	// Draw "racecar"
	r->Draw(display, s);
	
	return;
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	strncpy(gDefaultMap, argument[1], 1024);
	return 2;
}



uint64_t random64()
{
	uint64_t r1 = random();
	uint64_t r2 = random();
	return (r1<<32)|r2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key) // handles keypresses that change display
{
	switch (key)
	{
		case 'r':
			// TODO: Reset to start state
			break;
			// TODO: Make appropriate movements
			// TODO: Add support for WASD here
		case kUpArrow: // y velocity goes up
			std::cout << "Up arrow!" << std::endl;
			s.yVelocity = s.yVelocity + 1;
			if (s.yVelocity > 5){
				s.yVelocity = 5;
			}
			std::cout << s.yVelocity << std::endl;
			break;
		case kDownArrow:
			std::cout << "Down arrow!" << std::endl;
			s.yVelocity = s.yVelocity - 1;
			if (s.yVelocity < 0){
				s.yVelocity = 0;
			}
			break;
		case kLeftArrow:
			break;
		case kRightArrow:
			break;
		// --- Support for WASD --- //
		case 'w':
			std::cout << "The W key!" << std::endl;
			
		case 'a':
			std::cout << "The A key!" << std::endl;
		case 's':
			std::cout << "The S key!" << std::endl;
		case 'd':
			std::cout << "The D key!" << std::endl;
		default:
			break;
	}
	
}

/*
 * Code runs when user clicks or moves mouse in window
 *
 * Application does not currently need mouse support
 */
bool MyClickHandler(unsigned long , int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
	switch (mType)
	{
		case kMouseMove:
		{
		}
			break;
		case kMouseDown:
		{
		}
			break;
		default:
			break;
	}
	return true;
}

