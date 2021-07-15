/*
 *  $Id: main.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/02/06.
 *  Modified by Nathan Sturtevant on 06/22/21.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include <SFML/Window.hpp>
#include <SFML/OpenGL.hpp>
#define GL_SILENCE_DEPRECATION

extern "C" {
	void glutStrokeCharacter(void *font, int character){}
	int glutStrokeWidth(void *font, int character) {return 0;}
}

int hog_main(int argc, char **argv);
int main(int argc, char **argv)
{
	return hog_main(argc, argv);
}


#include "SFML_HOG.h"
#include "Trackball.h"
#include "Common.h"
#include "TextBox.h"
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <cassert>
#include "MonoFont.h"

using namespace std;

pRecContext pContextInfo;
GLint gDollyPanStartPoint[2] = {0, 0};
GLfloat gTrackBallRotation [4] = {0.0f, 0.0f, 0.0f, 0.0f};
GLboolean gDolly = GL_FALSE;
GLboolean gPan = GL_FALSE;
GLboolean gTrackball = GL_FALSE;
pRecContext gTrackingContextInfo = NULL;
int gCurrButton = -1;
//bool pointpath = false;
//int ppMouseClicks = 0;
pRecContext backup;
double fps = 30.0;
// window width and height
int width=100, height=100;
Graphics::rect screenRect;
Graphics::point WindowToHOG(const Graphics::point &p);
MonoFont font;
std::vector<Graphics::Display::lineInfo> textLines;
	
pRecContext GetContext(unsigned long windowID)
{
	return pContextInfo;
}

pRecContext getCurrentContext()
{
	return pContextInfo;
}

Graphics::point convertToGlobalHogCoordinate(int x, int y)
{
	Graphics::point p;
	p.x = 2.0*x/width-1.0;
	p.y = 2.0*y/height-1.0;
	p.z = 0;
	return p;
}


void RunHOGGUI(int argc, char** argv, int windowDimension)
{
	RunHOGGUI(argc, argv, windowDimension, windowDimension);
}

void RunHOGGUI(int argc, char* argv[], int xDimension, int yDimension)
{
	int mousePressCount = 0;
	srandom(unsigned(time(0)));
	ProcessCommandLineArgs(argc, argv);
	pContextInfo = new recContext;

    sf::Window window(sf::VideoMode(xDimension, yDimension), "My window");
	window.setFramerateLimit(30); // call it once, after creating the window
/*
	glutReshapeFunc(resizeWindow);
	glutDisplayFunc(renderScene);
	glutIdleFunc(renderScene);
	glutMouseFunc(mousePressedButton);
	glutMotionFunc(mouseMovedButton);
	glutPassiveMotionFunc(mouseMovedNoButton);
	glutKeyboardFunc(keyPressed);*/
	initialConditions(pContextInfo);
	buildGL(xDimension, yDimension);
	pContextInfo->windowHeight = xDimension;
	pContextInfo->windowWidth = yDimension;
	
	HandleWindowEvent(pContextInfo, kWindowCreated);
	//	createMenus();

	while (window.isOpen())
    {
        // check all the window's events that were triggered since the last iteration of the loop
        sf::Event event;
        while (window.pollEvent(event))
        {
            // "close requested" event: we close the window
			switch (event.type)
			{
			case sf::Event::Closed:
                window.close();
				break;
			case sf::Event::TextEntered:
				if (event.text.unicode < 128)
					DoKeyboardCommand(pContextInfo, event.text.unicode, false, false, false);
				break;
			case sf::Event::KeyPressed:
				switch (event.key.code)
				{
				case sf::Keyboard::Left: DoKeyboardCommand(pContextInfo, kLeftArrow, event.key.shift, event.key.control, event.key.alt); break;
				case sf::Keyboard::Right: DoKeyboardCommand(pContextInfo, kRightArrow, event.key.shift, event.key.control, event.key.alt); break;
				case sf::Keyboard::Up: DoKeyboardCommand(pContextInfo, kUpArrow, event.key.shift, event.key.control, event.key.alt); break;
				case sf::Keyboard::Down: DoKeyboardCommand(pContextInfo, kDownArrow, event.key.shift, event.key.control, event.key.alt); break;
				default: break; // others handled by TextEntered
				}
				break;
			case sf::Event::Resized:
				resizeWindow(event.size.width, event.size.height);
				//printf("Window now %dx%d\n", event.size.width, event.size.height);
				xDimension = event.size.width;
				yDimension = event.size.height;
				pContextInfo->windowHeight = yDimension;
				pContextInfo->windowWidth = xDimension;
				resizeGL(pContextInfo, xDimension, yDimension); // forces projection matrix update
				break;
			case sf::Event::MouseButtonPressed:
				{
					Graphics::point p = WindowToHOG(Graphics::point(event.mouseButton.x, event.mouseButton.y));//convertToGlobalHogCoordinate(event.mouseButton.x, event.mouseButton.y);
					//printf("Click (%d, %d) - {%f, %f, %f}\n", event.mouseButton.x, event.mouseButton.y, p.x, p.y, p.z);
					mousePressCount+=1;
					switch (event.mouseButton.button)
					{
					case sf::Mouse::Right:
						//HandleMouseClick(pContextInfo, event.mouseButton.x, event.mouseButton.y, p, kRightButton, kMouseDown);
						//printf("Right (%d, %d) - {%f, %f, %f}\n", event.mouseButton.x, event.mouseButton.y, p.x, p.y, p.z);
						HandleMouse(pContextInfo, p, kRightButton, kMouseDown);
						break;
					case sf::Mouse::Left:
						//printf("Left (%d, %d) - {%f, %f, %f}\n", event.mouseButton.x, event.mouseButton.y, p.x, p.y, p.z);
						HandleMouse(pContextInfo, p, kLeftButton, kMouseDown);
						break;
					case sf::Mouse::Middle:
						HandleMouse(pContextInfo, p, kMiddleButton, kMouseDown);
						break;
					default: break;
					}
				}
				break;
			case sf::Event::MouseButtonReleased:
				{
					Graphics::point p = WindowToHOG(Graphics::point(event.mouseButton.x, event.mouseButton.y));
					mousePressCount-=1;
					switch (event.mouseButton.button)
					{
					case sf::Mouse::Right:
						HandleMouse(pContextInfo, p, kRightButton, kMouseUp);
						break;
					case sf::Mouse::Left:
						HandleMouse(pContextInfo, p, kLeftButton, kMouseUp);
						break;
					case sf::Mouse::Middle:
						HandleMouse(pContextInfo, p, kMiddleButton, kMouseUp);
						break;
					}
				}
				break;
			case sf::Event::MouseMoved:
				{
					Graphics::point p = WindowToHOG(Graphics::point(event.mouseMove.x, event.mouseMove.y));
					tButtonType bType = kNoButton;
					if (mousePressCount == 0)
					{
						//printf("Move (%d, %d) - {%f, %f, %f}\n", event.mouseMove.x, event.mouseMove.y, p.x, p.y, p.z);
						HandleMouse(pContextInfo, p, bType, kMouseMove);
					}
					else {
						//printf("Drag (%d, %d) - {%f, %f, %f}\n", event.mouseMove.x, event.mouseMove.y, p.x, p.y, p.z);
						HandleMouse(pContextInfo, p, bType, kMouseDrag);
					}
				}
				//std::cout << "new mouse x: " << event.mouseMove.x << std::endl;
				//std::cout << "new mouse y: " << event.mouseMove.y << std::endl;
				break;
			}
		}
		drawGL (pContextInfo, window);
    }
	
	
	//processStats(pContextInfo->unitLayer->getStats());
	//delete pContextInfo->unitLayer;
	delete pContextInfo;
}

void createMenus()
{
}

void processMenuEvents(int option)
{
}



/**
 * Called when a key is pressed, and no other keys are held down.
 */
void keyPressed(unsigned char key, int, int)
{
	//x+=y;
	bool shift = false;//(glutGetModifiers() == GLUT_ACTIVE_SHIFT);
	bool alt = false;//(glutGetModifiers() == GLUT_ACTIVE_ALT);
	bool cntrl = false;//(glutGetModifiers() == GLUT_ACTIVE_CTRL);
	DoKeyboardCommand(pContextInfo, key, shift, cntrl, alt);
}



void mouseMovedNoButton(int x, int y)
{
	//Graphics::point p = GetOGLPos(pContextInfo, x, y);
// 	tButtonType bType = kNoButton;
// 	if (HandleMouseClick(pContextInfo, x, y, p, bType, kMouseMove))
// 		return;

	
// 	if (!pContextInfo->camera[pContextInfo->currPort].thirdPerson)
// 	{
// 		if (gPan == GL_FALSE)
// 		{
// 			gDollyPanStartPoint[0] = (GLint)x;
// 			gDollyPanStartPoint[1] = (GLint)y;
// 			gTrackingContextInfo = pContextInfo;
// 			gPan = GL_TRUE;
// 			//glutSetCursor(GLUT_CURSOR_NONE);
// 		}
		
// 		float dx = gDollyPanStartPoint[0]-x;
// 		float dy = gDollyPanStartPoint[1]-y;
// 		gDollyPanStartPoint[0] = (GLint)x;
// 		gDollyPanStartPoint[1] = (GLint)y;
// 		float rotation[4] = {0.0f, 0.0f, 0.0f, 0.0f};
		
// //		rotation[0] = -dy/24;
// //		rotation[1] = 1;
// //		addToRotationTrackball(rotation, pContextInfo->camera[pContextInfo->currPort].rotations.cameraRotation);
// 		//pContextInfo->controlShip->addRotation(rotation);
// 		//addToRotationTrackball(rotation, pContextInfo->fRot);
// 		rotation[0] = -dx/24;
// 		rotation[1] = 0;
// 		rotation[2] = 1;
// 		//		if ((pContextInfo->controlShip)  && (rotation[0] != 0))
// 		//			pContextInfo->controlShip->addRotation(rotation);
		
// 		addToRotationTrackball(rotation, pContextInfo->camera[pContextInfo->currPort].rotations.cameraRotation);

		
// 		if (x > pContextInfo->globalCamera.viewWidth ||
// 			y > pContextInfo->globalCamera.viewHeight || x < 1 || y < 1)
// 		{
// 			//glutWarpPointer(pContextInfo->globalCamera.viewWidth/2, pContextInfo->globalCamera.viewHeight/2);
// 			gDollyPanStartPoint[0] = (GLint)pContextInfo->globalCamera.viewWidth/2;
// 			gDollyPanStartPoint[1] = (GLint)pContextInfo->globalCamera.viewHeight/2;
// 		}
// 	}
}

/**
 * Called when the mouse is moved with a button pressed down.
 */
void mouseMovedButton(int x, int y)
{
	// Graphics::point p = GetOGLPos(pContextInfo, x, y);
	// tButtonType bType = kLeftButton;
	// switch (gCurrButton)
	// {
	// 	//case GLUT_RIGHT_BUTTON: bType = kRightButton; break;
	// 	//case GLUT_LEFT_BUTTON: bType = kLeftButton; break;
	// 	//case GLUT_MIDDLE_BUTTON: bType = kMiddleButton; break;
	// }
	// bType = kLeftButton; // TODO: fix with SFML
	// if (HandleMouseClick(pContextInfo, x, y, p, bType, kMouseDrag))
	// 	return;
}


/**
 * Called when a mouse button is pressed.
 */
void mousePressedButton(int button, int state, int x, int y)
{
// 	gCurrButton = button;
// 	int modifiers = 0;//glutGetModifiers();
	
// 	//printf("Button = %d\n", button);
// 	if (state == GLUT_DOWN) {
// 		Graphics::point p = GetOGLPos(pContextInfo, x, y);
// 		tButtonType bType = kLeftButton;
// 		switch (gCurrButton)
// 		{
// 			case GLUT_RIGHT_BUTTON: bType = kRightButton; break;
// 			case GLUT_LEFT_BUTTON: bType = kLeftButton; break;
// 			case GLUT_MIDDLE_BUTTON: bType = kMiddleButton; break;
// 		}
// 		if (HandleMouseClick(pContextInfo, x, y, p, bType, kMouseDown))
// 			return;
	
// 		if (!pContextInfo->camera[pContextInfo->currPort].thirdPerson)
// 		{
// 			gDollyPanStartPoint[0] = (GLint)x;
// 			gDollyPanStartPoint[1] = (GLint)y;
// 			gPan = GL_TRUE;
// 			gTrackingContextInfo = pContextInfo;
// 		}
// 		else if ((button == GLUT_RIGHT_BUTTON) || ((button == GLUT_LEFT_BUTTON) && (modifiers == GLUT_ACTIVE_CTRL)))
// 		{ // pan
// 			if (gTrackball)
// 			{ // if we are currently tracking, end trackball
// 				gTrackball = GL_FALSE;
// 				if (gTrackBallRotation[0] != 0.0)
// 				{
// 					// Mouse moves world object
// 					if (pContextInfo->camera[pContextInfo->currPort].thirdPerson == true)
// 					{
// 						if (pContextInfo->moveAllPortsTogether)
// 						{
// 							for (int x = 0; x < pContextInfo->numPorts; x++)
// 								addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[x].rotations.worldRotation);
// 						}
						
// 						else {
// 							addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[pContextInfo->currPort].rotations.worldRotation);
// 						}
// 					}
// 					else {
// 						if (pContextInfo->moveAllPortsTogether)
// 						{
// 							for (int x = 0; x < pContextInfo->numPorts; x++)
// 								addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[x].rotations.cameraRotation);
// 						}
						
// 						else {
// 							addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[pContextInfo->currPort].rotations.cameraRotation);
// 						}
// 					}
// 				}
// 				gTrackBallRotation [0] = gTrackBallRotation [1] = gTrackBallRotation [2] = gTrackBallRotation [3] = 0.0f;
// 			}
// 			else if (gDolly)
// 			{ // if we are currently dollying, end dolly
// 				gDolly = GL_FALSE;
// 			}
// 			gDollyPanStartPoint[0] = (GLint)x;
// 			gDollyPanStartPoint[1] = (GLint)y;
// 			gPan = GL_TRUE;
// 			gTrackingContextInfo = pContextInfo;
// 		}
// 		else if ((button == GLUT_MIDDLE_BUTTON) || ((button == GLUT_LEFT_BUTTON) && (modifiers == GLUT_ACTIVE_SHIFT)))
// 		{ // dolly
// 			if (gTrackball)
// 			{ // if we are currently tracking, end trackball
// 				gTrackball = GL_FALSE;
// 				if (gTrackBallRotation[0] != 0.0)
// 				{
// 					// Mouse moves world object
// 					if (pContextInfo->camera[pContextInfo->currPort].thirdPerson == true)
// 					{
// 						if (pContextInfo->moveAllPortsTogether)
// 						{
// 							for (int x = 0; x < pContextInfo->numPorts; x++)
// 								addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[x].rotations.worldRotation);
// 						}
						
// 						else {
// 							addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[pContextInfo->currPort].rotations.worldRotation);
// 						}
// 					}
// 					else {
// 						if (pContextInfo->moveAllPortsTogether)
// 						{
// 							for (int x = 0; x < pContextInfo->numPorts; x++)
// 								addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[x].rotations.cameraRotation);
// 						}
						
// 						else {
// 							addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[pContextInfo->currPort].rotations.cameraRotation);
// 						}
// 					}
// 				}
// 				gTrackBallRotation [0] = gTrackBallRotation [1] = gTrackBallRotation [2] = gTrackBallRotation [3] = 0.0f;
// 			}
// 			else if (gPan)
// 			{ // if we are currently panning, end pan
// 				gPan = GL_FALSE;
// 			}
// 			gDollyPanStartPoint[0] = (GLint)x;
// 			gDollyPanStartPoint[1] = (GLint)y;
// 			gDolly = GL_TRUE;
// 			gTrackingContextInfo = pContextInfo;
// 		}
// 		else if (button == GLUT_LEFT_BUTTON)
// 		{ // trackball
// 			if (gDolly)
// 			{ // if we are currently dollying, end dolly
// 				gDolly = GL_FALSE;
// 				gTrackingContextInfo = NULL;
// 			}
// 			else if (gPan)
// 			{ // if we are currently panning, end pan
// 				gPan = GL_FALSE;
// 				gTrackingContextInfo = NULL;
// 			}
// 			startTrackball((long)x, (long)y,
// 										 (long)pContextInfo->camera[pContextInfo->currPort].viewOriginX,
// 										 (long)pContextInfo->camera[pContextInfo->currPort].viewOriginY,
// 										 pContextInfo->globalCamera.viewWidth,
// 										 pContextInfo->globalCamera.viewHeight);
// 			gTrackball = GL_TRUE;
// 			gTrackingContextInfo = pContextInfo;
// 		}
// 	}

	
	
// 	if (state == GLUT_UP)
// 	{
// 		// stop trackball, pan, or dolly
// 		Graphics::point p = GetOGLPos(pContextInfo, x, y);
// 		tButtonType bType = kLeftButton;
// 		switch (gCurrButton)
// 		{
// 			case GLUT_RIGHT_BUTTON: bType = kRightButton; break;
// 			case GLUT_LEFT_BUTTON: bType = kLeftButton; break;
// 			case GLUT_MIDDLE_BUTTON: bType = kMiddleButton; break;
// 		}
// 		if (HandleMouseClick(pContextInfo, x, y, p, bType, kMouseUp))
// 			return;

		
// 		// if we want to handle final movement when mouse is released
// //		if (!pContextInfo->camera[pContextInfo->currPort].thirdPerson)
// //		{
// //		}

		
// 		if (gDolly) { // end dolly
// 			gDolly = GL_FALSE;
// 		} 
// 		else if (gPan) { // end pan
// 			gPan = GL_FALSE;
// 		} 
// 		else if (gTrackball) { // end trackball
// 			gTrackball = GL_FALSE;
// 			if (gTrackBallRotation[0] != 0.0)
// 			{
// 				// Mouse moves world object
// 				if (pContextInfo->camera[pContextInfo->currPort].thirdPerson == true)
// 				{
// 					if (pContextInfo->moveAllPortsTogether)
// 					{
// 						for (int x = 0; x < pContextInfo->numPorts; x++)
// 							addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[x].rotations.worldRotation);
// 					}
					
// 					else {
// 						addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[pContextInfo->currPort].rotations.worldRotation);
// 					}
// 				}
// 				else {
// 					if (pContextInfo->moveAllPortsTogether)
// 					{
// 						for (int x = 0; x < pContextInfo->numPorts; x++)
// 							addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[x].rotations.cameraRotation);
// 					}
					
// 					else {
// 						addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[pContextInfo->currPort].rotations.cameraRotation);
// 					}
// 				}
// 			}
// 			gTrackBallRotation [0] = gTrackBallRotation [1] = gTrackBallRotation [2] = gTrackBallRotation [3] = 0.0f;
// 		} 
// 		gTrackingContextInfo = NULL;
// 	}
}


// move camera in x/y plane
static void mousePan (int x, int y, pRecContext pContextInfo)
{
}


// move camera in z axis
static void mouseDolly (int x, int y, pRecContext pContextInfo)
{
}

/**
 * Renders the scene.  Used by GLUT for it's display function.
 * Wraps the drawGL() function.
 */
void renderScene(void)
{

  // Update the tank model frame (basically the treads)
  //tankModelFrameUpdate();
  
  static double lastTime = ((double)clock()/CLOCKS_PER_SEC);
  double currTime = ((double)clock()/CLOCKS_PER_SEC);

  if ((currTime - lastTime) > (1/fps)) {
	  lastTime = currTime;
	  //drawGL(pContextInfo);

	  assert(false);
  }

}


/**
 * Called when the window is resized.  Specific format for GLUT.
 */
void resizeWindow(int x, int y)
{
	/*
	CGRect rect;
	while (0 != x%4)
		x++;
	while (0 != y%4)
		y++;
	int scale = 2;//glutGet(GLUT_WINDOW_SCALE);
	rect.size.width = scale*x;
	rect.size.height = scale*y;
	rect.origin.x = 0;
	rect.origin.y = 0;
	*/
	resizeGL(pContextInfo, x, y);
}


/**
 * Handles resizing of GL need context update and if the window dimensions change,
 * a window dimension update, reseting of viewport and an update of the projection matrix
 */
void resizeGL(pRecContext pContextInfo, int width, int height)
{
	if (!pContextInfo)
			return;

	pContextInfo->globalCamera.viewOriginX = 0;//viewRect.origin.x;
	pContextInfo->globalCamera.viewOriginY = 0;//viewRect.origin.y;
		
	pContextInfo->globalCamera.viewWidth = width;//(GLint)viewRect.size.width;
	pContextInfo->globalCamera.viewHeight = height;//(GLint)viewRect.size.height;
	// printf("Window size: {%d, %d}\n", width, height);
	for (int x = 0; x < pContextInfo->numPorts; x++)
	{
		setPortCamera(pContextInfo, x);
	}
	//		glViewport(0, 0, pContextInfo->camera.viewWidth, pContextInfo->camera.viewHeight);
	
	updateProjection(pContextInfo);  // update projection matrix
}



/**
 * Update the projection matrix based on camera and view info.
 * Should be called when viewport size, eye z position, or camera aperture changes.
 * Also call if far or near changes which is determined by shape size in this case.
 */
void updateProjection(pRecContext pContextInfo, int viewPort)
{
	GLdouble ratio, radians, wd2;
	int minVal, maxVal;
	if (viewPort == -1)
	{
		minVal = 0;
		maxVal = pContextInfo->numPorts-1;
	}
	else {
		minVal = maxVal = viewPort;
	}
	for (int x = minVal; x <= maxVal; x++)
	{
		pContextInfo->camera[x].frust.near = 0.01;
		pContextInfo->camera[x].frust.far = 20.0;
		
		radians = 0.0174532925 * pContextInfo->camera[x].aperture / 2; // half aperture degrees to radians 
		wd2 = pContextInfo->camera[x].frust.near * tan(radians);
		//ratio = pContextInfo->camera[x].viewWidth / (float) pContextInfo->camera[x].viewHeight;
		ratio = pContextInfo->windowWidth / (float) pContextInfo->windowHeight;
		// printf("[%d/%d]\n", pContextInfo->camera[x].viewWidth, pContextInfo->camera[x].viewHeight);
		if (ratio >= 1.0) // wider than tall
		{
			pContextInfo->camera[x].frust.left  = (ratio)*pContextInfo->camera[x].frust.near/pContextInfo->camera[x].viewPos.z;
			pContextInfo->camera[x].frust.right = -(ratio)*pContextInfo->camera[x].frust.near/pContextInfo->camera[x].viewPos.z;
			pContextInfo->camera[x].frust.top = -pContextInfo->camera[x].frust.near/pContextInfo->camera[x].viewPos.z;
			pContextInfo->camera[x].frust.bottom = pContextInfo->camera[x].frust.near/pContextInfo->camera[x].viewPos.z;
			screenRect.left = -ratio;
			screenRect.right = ratio;
			screenRect.top = -1;
			screenRect.bottom = 1;
			// std::cout << "window coordinates: {" << screenRect << "}\n";
			// pContextInfo->camera[x].frust.left  = -ratio * wd2;
			// pContextInfo->camera[x].frust.right = ratio * wd2;
			// pContextInfo->camera[x].frust.top = wd2;
			// pContextInfo->camera[x].frust.bottom = -wd2;
			
			// printf("=> l: %f r: %f t: %f b: %f\n",
			// 	   pContextInfo->camera[x].frust.left,
			// 	   pContextInfo->camera[x].frust.right,
			// 	   pContextInfo->camera[x].frust.top,
			// 	   pContextInfo->camera[x].frust.bottom);
			Graphics::rect t1, t2, r(-1, -1, 1, 1);
			t1 = ViewportToGlobalHOG(r, x);
			// std::cout << "Port " << x << " {" << r << "} -> {" << t1 << "}\n";
		} else {
			ratio = 1/ratio;
			pContextInfo->camera[x].frust.bottom  = (ratio)*pContextInfo->camera[x].frust.near/pContextInfo->camera[x].viewPos.z;
			pContextInfo->camera[x].frust.top = -(ratio)*pContextInfo->camera[x].frust.near/pContextInfo->camera[x].viewPos.z;
			pContextInfo->camera[x].frust.left = pContextInfo->camera[x].frust.near/pContextInfo->camera[x].viewPos.z;
			pContextInfo->camera[x].frust.right = -pContextInfo->camera[x].frust.near/pContextInfo->camera[x].viewPos.z;
			screenRect.left = -1;
			screenRect.right = 1;
			screenRect.top = -ratio;
			screenRect.bottom = ratio;

			// std::cout << "-->window coordinates: {" << screenRect << "}\n";
			// pContextInfo->camera[x].frust.left  = -ratio * wd2;
			// pContextInfo->camera[x].frust.right = ratio * wd2;
			// pContextInfo->camera[x].frust.top = wd2;
			// pContextInfo->camera[x].frust.bottom = -wd2;
			
			// printf("--> l: %f r: %f t: %f b: %f\n",
			// 	   pContextInfo->camera[x].frust.left,
			// 	   pContextInfo->camera[x].frust.right,
			// 	   pContextInfo->camera[x].frust.top,
			// 	   pContextInfo->camera[x].frust.bottom);
			Graphics::rect t1, t2, r(-1, -1, 1, 1);
			t1 = ViewportToGlobalHOG(r, x);
			// std::cout << "Port " << x << " {" << r << "} -> {" << t1 << "}\n";
		}
	}
}


// to perform cross product between 2 vectors in myGluLookAt 
void CrossProd(float x1, float y1, float z1, float x2, float y2, float z2, float res[3]) 
{ 
	res[0] = y1*z2 - y2*z1; 
	res[1] = x2*z1 - x1*z2; 
	res[2] = x1*y2 - x2*y1; 
} 

// my own implementation 
void MyGluLookAt(float eyeX, float eyeY, float eyeZ, float lookAtX, float lookAtY, float lookAtZ, float upX, float upY, float upZ) 
{ 
	// i am not using here proper implementation for vectors. 
	// if you want, you can replace the arrays with your own 
	// vector types 
	float f[3]; 
	
	// calculating the viewing vector 
	f[0] = lookAtX - eyeX; 
	f[1] = lookAtY - eyeY; 
	f[2] = lookAtZ - eyeZ; 
	
	float fMag, upMag; 
	fMag = sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]); 
	upMag = sqrt(upX*upX + upY*upY + upZ*upZ); 
	
	// normalizing the viewing vector 
	if( fMag != 0) 
	{ 
		f[0] = f[0]/fMag; 
		f[1] = f[1]/fMag; 
		f[2] = f[2]/fMag; 
	} 
	
	// normalising the up vector. no need for this here if you have your 
	// up vector already normalised, which is mostly the case. 
	if( upMag != 0 ) 
	{ 
		upX = upX/upMag; 
		upY = upY/upMag; 
		upZ = upZ/upMag; 
	} 
	
	float s[3], u[3]; 
	
	CrossProd(f[0], f[1], f[2], upX, upY, upZ, s); 
	CrossProd(s[0], s[1], s[2], f[0], f[1], f[2], u); 
	
	float M[]= 
	{ 
		s[0], u[0], -f[0], 0, 
		s[1], u[1], -f[1], 0, 
		s[2], u[2], -f[2], 0, 
		0, 0, 0, 1 
	}; 
	
	glMultMatrixf(M); 
	glTranslatef(-eyeX, -eyeY, -eyeZ); 
}


/**
 * Updates the viewpoint of the model.
 */
void updateModelView(pRecContext pContextInfo, int currPort)
{
	// move view
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	// mouse transforms object
	if (pContextInfo->camera[currPort].thirdPerson)
	{
		MyGluLookAt(pContextInfo->camera[currPort].viewPos.x,
				   pContextInfo->camera[currPort].viewPos.y,
				   pContextInfo->camera[currPort].viewPos.z,
				   pContextInfo->camera[currPort].viewPos.x + pContextInfo->camera[currPort].viewDir.x,
				   pContextInfo->camera[currPort].viewPos.y + pContextInfo->camera[currPort].viewDir.y,
				   pContextInfo->camera[currPort].viewPos.z + pContextInfo->camera[currPort].viewDir.z,
		 		   pContextInfo->camera[currPort].viewUp.x, pContextInfo->camera[currPort].viewUp.y ,pContextInfo->camera[currPort].viewUp.z);

		if ((gTrackingContextInfo == pContextInfo) && gTrackBallRotation[0] != 0.0f) // if we have trackball rotation to map (this IS the test I want as it can be explicitly 0.0f)
		{
			if (pContextInfo->currPort == currPort || pContextInfo->moveAllPortsTogether)
				glRotatef (gTrackBallRotation[0], gTrackBallRotation[1], gTrackBallRotation[2], gTrackBallRotation[3]);
		}
		else {
		}
		
		// accumlated world rotation via trackball
		glRotatef (pContextInfo->camera[currPort].rotations.worldRotation[0],
				   pContextInfo->camera[currPort].rotations.worldRotation[1],
				   pContextInfo->camera[currPort].rotations.worldRotation[2],
				   pContextInfo->camera[currPort].rotations.worldRotation[3]);
	}
	// if mouse moves whole world:
	else {
		glTranslatef(pContextInfo->camera[currPort].viewPos.x,
					 pContextInfo->camera[currPort].viewPos.y,
					 pContextInfo->camera[currPort].viewPos.z);
	}
	
}


/**
 * Draws a CString in OpenGL
 */
void drawCStringGL (char * cstrOut, GLuint fontList)
{
	GLint i = 0;
	if (!cstrOut)
		return;
	while (cstrOut [i])
		glCallList (fontList + cstrOut[i++]);
}

TextBox *myTextBox = 0;

void appendTextToBuffer(const char *tempStr)
{
	int ind = int(strlen(pContextInfo->message));
	pContextInfo->message[ind] = ' ';
	snprintf(&pContextInfo->message[ind+1], 256-(ind+2), "%s", tempStr);

	delete myTextBox;
	Graphics::point a(-.95, .95, -.95), b(.95, -.95, .95);
	rgbColor rc(1, 1, 1);
	myTextBox = new TextBox(pContextInfo->message, 120, a, b, 1000, true);
	myTextBox->setColor(rc);
}

void submitTextToBuffer(const char *val)
{
	strncpy(pContextInfo->message, val, 255);
	delete myTextBox;
	Graphics::point a(-.95, .95, -.95), b(.95, -.95, .95);
	rgbColor rc(1, 1, 1);
	myTextBox = new TextBox(pContextInfo->message, 120, a, b, 1000, true);
	myTextBox->setColor(rc);
}

Graphics::point ViewportToScreen(Graphics::point where, int viewport)
{
	auto p = ViewportToGlobalHOG(where, viewport);
	p.x *= screenRect.right;
	p.y *= screenRect.bottom;
	return p;
}

Graphics::rect ViewportToScreen(const Graphics::rect &loc, int viewport)
{
	auto r = ViewportToGlobalHOG(loc, viewport);
	r.left *= screenRect.right;
	r.right *= screenRect.right;
	r.top *= screenRect.bottom;
	r.bottom *= screenRect.bottom;
	return r;
}

float ViewportToScreenX(float x, int v)
{
	float val = ViewportToGlobalHOGX(x, v);
	return val*screenRect.right;
}

Graphics::point WindowToHOG(const Graphics::point &p)
{
	// Convert from 0,0 -> width/height range to HOG range
	// Just map to screenRect 
	float xperc = p.x/pContextInfo->windowWidth;
	float yperc = p.y/pContextInfo->windowHeight;
	// return Graphics::point(screenRect.left*(1-xperc)+screenRect.right*xperc,
	//					   screenRect.top*(1-yperc)+screenRect.bottom*yperc);
	return Graphics::point(-1*(1-xperc)+1*xperc,
						   -1*(1-yperc)+1*yperc);
}

void DoDrawCommands(Graphics::Display &display, int port, sf::Window &window, std::vector<Graphics::Display::data> &commands)
{
	for (auto &i: commands)
	{
		//auto &i = display.backgroundDrawCommands[x];
		if (i.viewport != port)
			continue;
		switch (i.what)
		{
		case Graphics::Display::kFillRectangle:
			{
				glColor3f(i.shape.c.r, i.shape.c.g, i.shape.c.b);
				//Graphics::rect tmp = GlobalHOGToViewport(i.shape.r, i.viewport);
				Graphics::rect tmp = ViewportToScreen(i.shape.r, i.viewport);
				//Graphics::rect tmp = ViewportToGlobalHOG(i.shape.r, i.viewport);
				//tmp *= {screenRect.right, screenRect.bottom}; // scale to screen
				glBegin(GL_QUADS);
				glVertex2f(tmp.left, tmp.bottom);
				glVertex2f(tmp.left, tmp.top);
				glVertex2f(tmp.right, tmp.top);
				glVertex2f(tmp.right, tmp.bottom);
				glEnd();
				//				epsilon -= de;
				break;
			}
		case Graphics::Display::kFrameRectangle:
			{
				glColor3f(i.shape.c.r, i.shape.c.g, i.shape.c.b);
				//Graphics::rect tmp = ViewportToGlobalHOG(i.shape.r, i.viewport);
				auto r2 = ViewportToScreen(i.shape.r.inset(i.shape.width/2), i.viewport);
				auto r1 = ViewportToScreen(i.shape.r.expand(i.shape.width/2), i.viewport);
				//tmp *= {screenRect.right, screenRect.bottom}; // scale to screen
					
				glBegin(GL_TRIANGLE_STRIP);
				//GLfloat rad = ViewportToGlobalHOGX(i.shape.width, i.viewport)/2.0;
				//GLfloat rad = ViewportToScreenX(i.shape.width, i.viewport)/2.0;
				glVertex2f(r1.left, r1.bottom);
				glVertex2f(r2.left, r2.bottom);
					
				glVertex2f(r1.left, r1.top);
				glVertex2f(r2.left, r2.top);
					
				glVertex2f(r1.right, r1.top);
				glVertex2f(r2.right, r2.top);
					
				glVertex2f(r1.right, r1.bottom);
				glVertex2f(r2.right, r2.bottom);
					
				glVertex2f(r1.left, r1.bottom);
				glVertex2f(r2.left, r2.bottom);


				glEnd();


				break;
			}
		case Graphics::Display::kFillOval:
			{
				//Graphics::rect tmp = ViewportToGlobalHOG(i.shape.r, i.viewport);
				Graphics::rect tmp = ViewportToScreen(i.shape.r, i.viewport);
				glColor3f(i.shape.c.r, i.shape.c.g, i.shape.c.b);
				DrawCircle((tmp.right+tmp.left)/2, (tmp.top+tmp.bottom)/2, fabs(tmp.top-tmp.bottom)/2.0, 64);
				break;
			}
		case Graphics::Display::kFillNGon:
			{
				glColor3f(i.polygon.c.r, i.polygon.c.g, i.polygon.c.b);
				//auto p = ViewportToGlobalHOG(i.polygon.center, i.viewport);
				Graphics::rect hog(i.polygon.center, i.polygon.radius);
				auto tmp = ViewportToScreen(hog, i.viewport);
				//auto p = ViewportToScreen(i.polygon.center, i.viewport);
				//DrawCircle(p.x, p.y, ViewportToGlobalHOGX(i.polygon.radius, i.viewport), i.polygon.segments, i.polygon.rotate);
				//DrawCircle(p.x, p.y, ViewportToScreenX(i.polygon.radius, i.viewport), i.polygon.segments, i.polygon.rotate);
				DrawCircle((tmp.right+tmp.left)/2, (tmp.top+tmp.bottom)/2, fabs(tmp.top-tmp.bottom)/2.0, i.polygon.segments, i.polygon.rotate);
				break;
			}
		case Graphics::Display::kFrameNGon:
			{
				glColor3f(i.polygon.c.r, i.polygon.c.g, i.polygon.c.b);
				//FrameCircle(i.polygon.p.x, i.polygon.p.y, i.polygon.radius, i.polygon.segments, , i.polygon.rotate);
				break;
			}
		case Graphics::Display::kFillTriangle:
			{		
				glColor3f(i.triangle.c.r, i.triangle.c.g, i.triangle.c.b);
				Graphics::point tmp1 = ViewportToScreen(i.triangle.p1, i.viewport);
				Graphics::point tmp2 = ViewportToScreen(i.triangle.p2, i.viewport);
				Graphics::point tmp3 = ViewportToScreen(i.triangle.p3, i.viewport);

				glBegin(GL_TRIANGLES);
				glVertex3f(tmp1.x, tmp1.y, tmp1.z);
				glVertex3f(tmp2.x, tmp2.y, tmp2.z);
				glVertex3f(tmp3.x, tmp3.y, tmp3.z);
				glEnd();
				break;
			}
		case Graphics::Display::kFrameOval:
			{		
				break;
			}
		case Graphics::Display::kLine:
			{
				//std::cout << i.line.start << " <=> " << i.line.end << " " << i.line.width << "\n";
				Graphics::point tmp1 = ViewportToScreen(i.line.start, i.viewport);
				Graphics::point tmp2 = ViewportToScreen(i.line.end, i.viewport);
				GLfloat xOff = tmp1.x-tmp2.x;
				GLfloat yOff = tmp2.y-tmp1.y;
				GLfloat ratio = ViewportToScreenX(0.5*i.line.width, i.viewport)/sqrt(xOff*xOff+yOff*yOff);

				glBegin(GL_QUADS);
				glColor3f(i.line.c.r, i.line.c.g, i.line.c.b);
				glVertex3f(tmp1.x-ratio*yOff, tmp1.y-ratio*xOff, tmp1.z);
				glVertex3f(tmp1.x+ratio*yOff, tmp1.y+ratio*xOff, tmp1.z);
				glVertex3f(tmp2.x+ratio*yOff, tmp2.y+ratio*xOff, tmp1.z);
				glVertex3f(tmp2.x-ratio*yOff, tmp2.y-ratio*xOff, tmp1.z);
				glEnd();
				printf("(%f, %f) to (%f, %f); ",
					   tmp1.x-ratio*yOff, tmp1.y-ratio*xOff,
					   tmp1.x+ratio*yOff, tmp1.y+ratio*xOff);
				printf("(%f, %f) to (%f, %f)\n",
					   tmp2.x+ratio*yOff, tmp2.y+ratio*xOff,
					   tmp2.x-ratio*yOff, tmp2.y-ratio*xOff);
				break;
			}
		}
	}

}

void DrawLines(std::vector<Graphics::Display::lineInfo> &textLines, int viewport)
{
	for (auto i : textLines)
	{
		Graphics::point tmp1 = ViewportToScreen(i.start, viewport);
		Graphics::point tmp2 = ViewportToScreen(i.end, viewport);
		GLfloat xOff = tmp1.x-tmp2.x;
		GLfloat yOff = tmp2.y-tmp1.y;
		GLfloat wide = ViewportToScreenX(0.5f*i.width, viewport);
		GLfloat ratio = wide/sqrt(xOff*xOff+yOff*yOff);
		
		glBegin(GL_QUADS);
		glColor3f(i.c.r, i.c.g, i.c.b);
		glVertex3f(tmp1.x-ratio*yOff, tmp1.y-ratio*xOff, tmp1.z);
		glVertex3f(tmp1.x+ratio*yOff, tmp1.y+ratio*xOff, tmp1.z);
		glVertex3f(tmp2.x+ratio*yOff, tmp2.y+ratio*xOff, tmp1.z);
		glVertex3f(tmp2.x-ratio*yOff, tmp2.y-ratio*xOff, tmp1.z);
		glEnd();
		DrawCircle(tmp1.x, tmp1.y, wide, 16);
		DrawCircle(tmp2.x, tmp2.y, wide, 16);
	}
}

void DrawGraphics(Graphics::Display &display, int port, sf::Window &window)
{
	for (auto &i : display.backgroundLineSegments)
	{
		if (i.viewport != port)
			continue;
		glColor3f(i.c.r, i.c.g, i.c.b);
		
		glBegin(GL_QUADS);
		for (int t = 0; t < i.points.size()-1; t++)
		{
			Graphics::point tmp1 = ViewportToScreen(i.points[t], i.viewport);
			Graphics::point tmp2 = ViewportToScreen(i.points[t+1], i.viewport);
			GLfloat xOff = tmp1.x-tmp2.x;
			GLfloat yOff = tmp2.y-tmp1.y;
			GLfloat ratio = 0.5f*i.size/sqrt(xOff*xOff+yOff*yOff);
			glVertex3f(tmp1.x-ratio*yOff, tmp1.y-ratio*xOff, tmp1.z);
			glVertex3f(tmp1.x+ratio*yOff, tmp1.y+ratio*xOff, tmp1.z);
			glVertex3f(tmp2.x+ratio*yOff, tmp2.y+ratio*xOff, tmp1.z);
			glVertex3f(tmp2.x-ratio*yOff, tmp2.y-ratio*xOff, tmp1.z);
		}
		glEnd();
	}
	DoDrawCommands(display, port, window, display.backgroundDrawCommands);
	for (auto &i : display.backgroundText)
	{
		if (i.viewport != port)
			continue;

		font.GetTextLines(textLines,
						  i.loc, i.s.c_str(), i.size,
						  i.c,
						  i.align, i.base);
		DrawLines(textLines, i.viewport);
	}

	
	for (auto &i : display.lineSegments)
	{
		if (i.viewport != port)
			continue;
		glColor3f(i.c.r, i.c.g, i.c.b);
		
		glBegin(GL_QUADS);
		for (int t = 0; t < i.points.size()-1; t++)
		{
			Graphics::point tmp1 = ViewportToScreen(i.points[t], i.viewport);
			Graphics::point tmp2 = ViewportToScreen(i.points[t+1], i.viewport);
			GLfloat xOff = tmp1.x-tmp2.x;
			GLfloat yOff = tmp1.y-tmp2.y;
			GLfloat ratio = 0.5f*i.size/sqrt(xOff*xOff+yOff*yOff);
			glVertex3f(tmp1.x-ratio*yOff, tmp1.y-ratio*xOff, tmp1.z);
			glVertex3f(tmp1.x+ratio*yOff, tmp1.y+ratio*xOff, tmp1.z);
			glVertex3f(tmp2.x+ratio*yOff, tmp2.y+ratio*xOff, tmp1.z);
			glVertex3f(tmp2.x-ratio*yOff, tmp2.y-ratio*xOff, tmp1.z);
		}
		glEnd();
		
	}	
	DoDrawCommands(display, port, window, display.drawCommands);
	for (auto &i : display.text)
	{
		if (i.viewport != port)
			continue;

		font.GetTextLines(textLines,
						  i.loc, i.s.c_str(), i.size,
						  i.c,
						  i.align, i.base);
		DrawLines(textLines, i.viewport);
	}

}

/**
 * Main OpenGL drawing function.
 */
void drawGL (pRecContext pContextInfo, sf::Window &window)
{
	if (!pContextInfo)
	 return;
	pContextInfo->display.StartFrame();
	//window.clear();
	// clear our drawable
	glClear(GL_COLOR_BUFFER_BIT);
	glClear(GL_DEPTH_BUFFER_BIT);
	
	for (int x = 0; x < pContextInfo->numPorts; x++)
	{
		updateProjection(pContextInfo, x);
		setViewport(pContextInfo, x);
		//if (pContextInfo->drawing)
		{
			// set projection
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			
			glFrustum(pContextInfo->camera[x].frust.left, pContextInfo->camera[x].frust.right,
								pContextInfo->camera[x].frust.bottom, pContextInfo->camera[x].frust.top,
								pContextInfo->camera[x].frust.near, pContextInfo->camera[x].frust.far);
			// projection matrix already set	
			updateModelView(pContextInfo, x);			
			HandleFrame(pContextInfo, x);
			DrawGraphics(pContextInfo->display, x, window);
		}
	}
	if (myTextBox)
	{
		myTextBox->stepTime(0.1);
		myTextBox->draw();
	}
	pContextInfo->display.EndFrame();
	//glutSwapBuffers();
	window.display();
}


/**
 * End OpenGL drawing function - for visualizing trajectory merging
 */
/*void trajectoryDrawGL (pRecContext pContextInfo)
{
 if (!pContextInfo)
        return;

	// clear our drawable
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// projection matrix already set	
	updateModelView (pContextInfo);

	glDisable(GL_LIGHTING);
	pContextInfo->unitLayer->GetMap()->OpenGLDraw(kPolygons);
	if (pContextInfo->unitLayer->getMapAbstractionDisplay())
	{
		pContextInfo->unitLayer->getMapAbstractionDisplay()->OpenGLDraw();
	}
	glEnable(GL_LIGHTING);

	static double lastTime = ((double)clock()/CLOCKS_PER_SEC);
	double currTime = ((double)clock()/CLOCKS_PER_SEC);

	pContextInfo->unitLayer->advanceTime(currTime-lastTime);
	lastTime = currTime;
	if (pContextInfo->drawing)
	{
		pContextInfo->unitLayer->OpenGLDraw();
		if (pContextInfo->info) {
			glDisable(GL_LIGHTING);
			drawInfo (pContextInfo);
		}
	}
	frameCallback(pContextInfo->unitLayer);

	//glFlush();
	glutSwapBuffers();

}*/


/**
 * Initializes OpenGL.
 */
void buildGL(int xDim, int yDim)
{ 
    	if (NULL == pContextInfo)
        	return;
		
	// build context
		//CGRect viewRect = {{0.0f, 0.0f}, {0.0f, 0.0f}};
	
//	switch (pContextInfo->modeFSAA) {
//		case kFSAAOff:
//#ifndef WIN32
//			//glDisable (GL_MULTISAMPLE_ARB);
//#endif
//			break;
//		case kFSAAFast:
////			glEnable (GL_MULTISAMPLE_ARB);
////			glHint (GL_MULTISAMPLE_FILTER_HINT_NV, GL_FASTEST);
//			break;
//		case kFSAANice:
//		        #ifndef WIN32
//			//glEnable (GL_MULTISAMPLE_ARB);
//			//glHint (GL_MULTISAMPLE_FILTER_HINT_NV, GL_NICEST);
//			#endif
//			break;
//	}

	// init GL stuff here
	//glEnable(GL_DEPTH_TEST);
	glDisable(GL_DEPTH_TEST);

	glShadeModel(GL_SMOOTH);    
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	glFrontFace(GL_CCW);
	glPolygonOffset (1.0, 1.0);
		
	glClearColor(0.0,0.0,0.0,0.0);

	//int scale = 2;//glutGet(GLUT_WINDOW_SCALE);
	//viewRect.size.width = xDim;//scale*glutGet(GLUT_WINDOW_WIDTH);
	//viewRect.size.height = yDim;//scale*glutGet(GLUT_WINDOW_HEIGHT);
	
	// setup viewport and prespective
	resizeGL(pContextInfo, xDim, yDim); // forces projection matrix update
		
	SetLighting();
}

