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
#include "TemplateAStar.h"

bool recording = false;
bool running = false;
bool move = false;

Map *m = 0;
Racetrack *r = 0;
RacetrackState s;
RacetrackMove v;
RacetrackState start;
RacetrackState end;
std::vector<RacetrackMove> actions;
std::vector<RacetrackState> path;
TemplateAStar<RacetrackState, RacetrackMove, Racetrack> astar;


// -------------- MAIN FUNCTION ----------- //
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

	InstallKeyboardHandler(MyDisplayHandler, "M", "Move automatically", kAnyModifier, 'm');
	InstallKeyboardHandler(MyDisplayHandler, "O", "Solve optimally", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "F", "Freeze", kAnyModifier, 'f');

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
		
		m = new Map(11, 11); // makes a map with the dimensions in the parentheses
		
		for (int x = 0; x <2; x++)
		{
			m->SetTerrainType(x, 0, kStartTerrain); // Start terrain placed down
		}
		for (int x = 0; x < 5; x++)
		{
			m->SetTerrainType(x, 5, kTrees); // Tree terrain placed down
		}
		
		
		for (int x = 0; x < 7; x++)
		{
			m->SetTerrainType(x, 9, kEndTerrain); // End terrain
		}
		
		
		r = new Racetrack(m);
		r->Reset(s);
		start.loc.x = 1;
		start.loc.y = 1;
		end.loc.x = 5;
		end.loc.y = 9;
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	display.FillRect({-1, -1, 1, 1}, Colors::black);
	
	// Draw map
	r->Draw(display);
	// Draw "racecar"
	r->Draw(display, s, v); //Draws the state of the racetrack
	if (move == true) // if m was pressed, makes the agent move automatically
	{
		r->ApplyAction(s, v);
	}
	r->SetColor(Colors::blue);
	for (int x=1; x < path.size(); x++)
	{
		r->DrawLine(display, path[x-1], path[x], 1);
	}
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
			r->Reset(s);
			break;
			// TODO: Make appropriate movements - done
			// TODO: Add support for WASD here - done
		
		
		case kUpArrow: case 'w': // y velocity goes up
			std::cout << "Up arrow!" << std::endl;
			v.xDelta = 0;
			v.yDelta = -1;
			r->ApplyAction(s, v);
			r->GetActions(s, actions);
			break;
		case kDownArrow: case 's':
			std::cout << "Down arrow!" << std::endl;
			v.xDelta = 0;
			v.yDelta = 1;
			r->ApplyAction(s, v);
			r->GetActions(s, actions);
			break;
		case kLeftArrow: case 'a':
			v.xDelta = -1;
			v.yDelta = 0;
			r->ApplyAction(s, v);
			r->GetActions(s, actions);
			break;
		case kRightArrow: case 'd':
			v.xDelta = 1;
			v.yDelta = 0;
			r->ApplyAction(s, v);
			r->GetActions(s, actions);
			break;

		case 'm':
			std::cout << "The M Key! \n";
			std::cout << move << " \n";
			if (move == true)
			{
				move = false;
			}
			else
			{
				move = true;
			}
			
			break;
		case 'o':
			astar.GetPath(r, s, s, path);
		case 'f':
			v.xDelta = 0;
			v.yDelta = 0;
			s.xVelocity = 0;
			s.yVelocity = 0;
			r->ApplyAction(s, v);
			r->GetActions(s, actions);
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
			
			//std::cout << "Mouse is being held down! \n";

			

		}
			break;
		default:
			break;
	}
	return true;
}

