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

// #include "cubeHolder.h"

RCState cube;
RC env;
//RC env2;

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
	// HASH FUNCTION
	InstallKeyboardHandler(MyDisplayHandler, "Test9", "HASH TEST", kAnyModifier, '9');
	

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
	InstallKeyboardHandler(MyDisplayHandler, "Reset", "Reset cube to solved", kAnyModifier, 'r');
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
	
	//Draw here
	env.TestUpdate();
	env.Draw(display, cube);
	env.TestDraw(display, cube);

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
	// TODO: Check if already rotating before allowing an input from 0-5
	switch (key)
	{
		// HASH FUNCTION
		case '9':
		{
			static int a = 0;
			env.GetStateFromPDBHashCorner(a++, cube, 0);
			printf("rank is %llu\n", env.GetPDBHashCorner(cube, 0));
		}
			break;
			
		case 'm':
			moveType++;
			if (moveType > 2) moveType = 0;
			std::cout << "Set Movetype to: " + std::to_string(moveType) << '\n';
			break;	
		case 'n':
			env.passiveRot = !env.passiveRot;		
			break;	
		case '0':	
		case '1':	
		case '2':
		case '3':
		case '4':
		case '5':
		{
			int ikey = key - '0';
			// TEMP: DO NOT ROTATE RC
			// env.RotateFace(ikey, moveType);

			cube.RotateFace(ikey*3+moveType);
			
			// Test
			std::cout << "State: " << '\n'; cube.PrintState();
			std::cout << '\n';
			break;
		}
		case 'o':
		{
			// TEMP: Print data from cubies from each RC
//			std::cout << "Env 1:" << std::endl;
//			env.cubies[0].PrintData();
//			std::cout << std::endl;
//			std::cout << "Env 2:" << std::endl;
//			env2.cubies[0].PrintData();
			// TEMP: Rotate on x axis
			float rot[3] = {0.3f, 0, 0};
			env.RotateCubies(rot);
			break;
		}
		case 's':
		{
			// TODO: When hash function is finished you can use it to get random states 
			// TEMP: Sets the state to a custom one
			for (int i = 0; i < 12; i++)
			{
				cube.rotation[i] = 1;
			}
			break;
		}
		case 'r':
		{
			// Reset state
			for (int i = 0; i < 20; i++)
			{
				cube.indices[i] = i;
				cube.rotation[i] = 0;
			}
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

