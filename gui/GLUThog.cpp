/*
 * $Id: main.cpp,v 1.28 2006/11/02 21:50:34 nathanst Exp $
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */


#include "Trackball.h"
#include "Common.h"
#include "GLUThog.h"
#include "TextBox.h"
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>

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

pRecContext GetContext(unsigned long windowID)
{
	return pContextInfo;
}

pRecContext getCurrentContext()
{
	return pContextInfo;
}

void RunHOGGUI(int argc, char** argv, int windowDimension)
{
	RunHOGGUI(argc, argv, windowDimension, windowDimension);
}

void RunHOGGUI(int argc, char* argv[], int xDimension, int yDimension)
{
  // Init traj global
  startTrajRecap = false;

	srandom(unsigned(time(0)));
	
//	InstallCommandLineHandler(processFramesPerSecond, "-fps", "-fps <int>", "[System Option] Specifies the maximum frames per second.");
	//initializeHandlers();
	ProcessCommandLineArgs(argc, argv);
	
	pContextInfo = new recContext;
	//resetCamera(&(pContextInfo->camera));

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(xDimension, yDimension);
	glutCreateWindow("Map Abstraction");
	glutReshapeFunc(resizeWindow);
	glutDisplayFunc(renderScene);
	glutIdleFunc(renderScene);
	glutMouseFunc(mousePressedButton);
	glutMotionFunc(mouseMovedButton);
	glutPassiveMotionFunc(mouseMovedNoButton);
	glutKeyboardFunc(keyPressed);
	initialConditions(pContextInfo);
	buildGL();
	HandleWindowEvent(pContextInfo, kWindowCreated);
	createMenus();
	
	// Initialize the tank model
	//initTankTextures();
	
	glutMainLoop();
	
	//processStats(pContextInfo->unitLayer->getStats());
	//delete pContextInfo->unitLayer;
	delete pContextInfo;
}

void createMenus()
{
//	int menu, submenu;
//	
//	// create the menu and
//	// tell glut that "processMenuEvents" will 
//	// handle the events
//	submenu = glutCreateMenu(processMenuEvents);
//	glutAddMenuEntry("Map1",'map1');
//	glutAddMenuEntry("Map2",'map2');
//	glutAddMenuEntry("Map3",'map3');
//	menu = glutCreateMenu(processMenuEvents);
//	
//	//add entries to our menu
//	glutAddSubMenu("Open Map",submenu);
//	glutAddMenuEntry("clear Map...",'clar');
//	
//	// attach the menu to the right button
//	glutAttachMenu(GLUT_MIDDLE_BUTTON);
}

void processMenuEvents(int option)
{
/*
	switch (option) {
		case 'map1':
			sprintf(gDefaultMap, "/Users/nathanst/Development/hog/maps/bgmaps/AR0011SR.map");
			//processStats(pContextInfo->unitLayer->getStats());
			//delete pContextInfo->unitLayer;
			//createSimulation(pContextInfo->unitLayer);
			break;
		case 'map2':
			sprintf(gDefaultMap, "/Users/nathanst/Development/hog/maps/wc3maps/divideandconquer.map");
			processStats(pContextInfo->unitLayer->getStats());
			delete pContextInfo->unitLayer;
			createSimulation(pContextInfo->unitLayer);
			break;
		case 'map3':
			sprintf(gDefaultMap, "/Users/nathanst/Development/hog/maps/wc3maps/mysticisles.map");
			processStats(pContextInfo->unitLayer->getStats());
			delete pContextInfo->unitLayer;
			createSimulation(pContextInfo->unitLayer);
			break;
		case 'clar':
			gDefaultMap[0] = 0;
			processStats(pContextInfo->unitLayer->getStats());
			delete pContextInfo->unitLayer;
			createSimulation(pContextInfo->unitLayer);
			break;
		default: printf("Unknown menu event: %d\n", option);
			break;
	}
 */
}

//int processFramesPerSecond(char *argument[], int maxNumArgs)
//{
//	if (maxNumArgs <= 1)
//		return 0;
//	fps = atof(argument[1]);
//	if (fps == 0)
//		fps = 1.0;
//	return 2;
//}

//void pointPath() {
//	
//	ppMouseClicks = 0;
//
//	if (!pointpath) {
//		glutSetCursor(GLUT_CURSOR_RIGHT_ARROW);
//		backup = new recContext;
//		*backup = *pContextInfo;
//		
//		// Reset the view so that you are looking straight down on the map
//		resetCamera (&pContextInfo->camera);
////		pContextInfo->worldRotation[0] = 180;
////		pContextInfo->worldRotation[1] = 0;
////		pContextInfo->worldRotation[2] = -.5;
////		pContextInfo->worldRotation[3] = .5;
//	}
//	else {
//		glutSetCursor(GLUT_CURSOR_INHERIT);
//		*pContextInfo = *backup;
//		if (backup) {
//			delete backup;
//			backup = 0;
//		}
//		
//	}
//	
//	pointpath = 1 - pointpath;
//}



/**
 * Called when a key is pressed, and no other keys are held down.
 */
void keyPressed(unsigned char key, int, int)
{
	//x+=y;
	bool shift = (glutGetModifiers() == GLUT_ACTIVE_SHIFT);
	bool alt = (glutGetModifiers() == GLUT_ACTIVE_ALT);
	bool cntrl = (glutGetModifiers() == GLUT_ACTIVE_CTRL);
	DoKeyboardCommand(pContextInfo, key, shift, cntrl, alt);
}



void mouseMovedNoButton(int x, int y)
{
	if (!pContextInfo->camera[pContextInfo->currPort].thirdPerson)
	{
		if (gPan == GL_FALSE)
		{
			gDollyPanStartPoint[0] = (GLint)x;
			gDollyPanStartPoint[1] = (GLint)y;
			gTrackingContextInfo = pContextInfo;
			gPan = GL_TRUE;
			glutSetCursor(GLUT_CURSOR_NONE);
		}
		
		float dx = gDollyPanStartPoint[0]-x;
		float dy = gDollyPanStartPoint[1]-y;
		gDollyPanStartPoint[0] = (GLint)x;
		gDollyPanStartPoint[1] = (GLint)y;
		float rotation[4] = {0.0f, 0.0f, 0.0f, 0.0f};
		
		rotation[0] = -dy/24;
		rotation[1] = 1;
		addToRotationTrackball(rotation, pContextInfo->camera[pContextInfo->currPort].rotations.cameraRotation);
		//pContextInfo->controlShip->addRotation(rotation);
		//addToRotationTrackball(rotation, pContextInfo->fRot);
		rotation[0] = -dx/24;
		rotation[1] = 0;
		rotation[2] = 1;
		//		if ((pContextInfo->controlShip)  && (rotation[0] != 0))
		//			pContextInfo->controlShip->addRotation(rotation);
		
		addToRotationTrackball(rotation, pContextInfo->camera[pContextInfo->currPort].rotations.cameraRotation);

		
		if (x > pContextInfo->globalCamera.viewWidth ||
			y > pContextInfo->globalCamera.viewHeight || x < 1 || y < 1)
		{
			glutWarpPointer(pContextInfo->globalCamera.viewWidth/2, pContextInfo->globalCamera.viewHeight/2);
			gDollyPanStartPoint[0] = (GLint)pContextInfo->globalCamera.viewWidth/2;
			gDollyPanStartPoint[1] = (GLint)pContextInfo->globalCamera.viewHeight/2;
		}
	}
}

/**
 * Called when the mouse is moved with a button pressed down.
 */
void mouseMovedButton(int x, int y)
{
	point3d p = GetOGLPos(pContextInfo, x, y);
	tButtonType bType = kLeftButton;
	switch (gCurrButton)
	{
		case GLUT_RIGHT_BUTTON: bType = kRightButton; break;
		case GLUT_LEFT_BUTTON: bType = kLeftButton; break;
		case GLUT_MIDDLE_BUTTON: bType = kMiddleButton; break;
	}
	if (HandleMouseClick(pContextInfo, x, y, p, bType, kMouseDrag))
		return;
	
	
	if (!pContextInfo->camera[pContextInfo->currPort].thirdPerson)
	{
		float dx = gDollyPanStartPoint[0]-x;
		float dy = gDollyPanStartPoint[1]-y;
		gDollyPanStartPoint[0] = (GLint)x;
		gDollyPanStartPoint[1] = (GLint)y;
		float rotation[4] = {0.0f, 0.0f, 0.0f, 0.0f};
		
		rotation[0] = -dy/24;
		rotation[1] = 1;
		addToRotationTrackball(rotation, pContextInfo->camera[pContextInfo->currPort].rotations.cameraRotation);
		//pContextInfo->controlShip->addRotation(rotation);
		//addToRotationTrackball(rotation, pContextInfo->fRot);
		rotation[0] = -dx/24;
		rotation[1] = 0;
		rotation[2] = 1;
//		if ((pContextInfo->controlShip)  && (rotation[0] != 0))
//			pContextInfo->controlShip->addRotation(rotation);
		
		addToRotationTrackball(rotation, pContextInfo->camera[pContextInfo->currPort].rotations.cameraRotation);
	}
	else if (gTrackball) {
		rollToTrackball((long) x, (long) y, gTrackBallRotation);
	} 
	else if (gDolly) {
		mouseDolly(x, y, pContextInfo);
	} 
	else if (gPan) {
		mousePan(x, y, pContextInfo);
	}
	
}


/**
 * Called when a mouse button is pressed.
 */
void mousePressedButton(int button, int state, int x, int y)
{
	gCurrButton = button;
	int modifiers = glutGetModifiers();
	
	//printf("Button = %d\n", button);
	if (state == GLUT_DOWN) {
		point3d p = GetOGLPos(pContextInfo, x, y);
		tButtonType bType = kLeftButton;
		switch (gCurrButton)
		{
			case GLUT_RIGHT_BUTTON: bType = kRightButton; break;
			case GLUT_LEFT_BUTTON: bType = kLeftButton; break;
			case GLUT_MIDDLE_BUTTON: bType = kMiddleButton; break;
		}
		if (HandleMouseClick(pContextInfo, x, y, p, bType, kMouseDown))
			return;
	
		if (!pContextInfo->camera[pContextInfo->currPort].thirdPerson)
		{
			gDollyPanStartPoint[0] = (GLint)x;
			gDollyPanStartPoint[1] = (GLint)y;
			gPan = GL_TRUE;
			gTrackingContextInfo = pContextInfo;
		}
		else if ((button == GLUT_RIGHT_BUTTON) || ((button == GLUT_LEFT_BUTTON) && (modifiers == GLUT_ACTIVE_CTRL)))
		{ // pan
			if (gTrackball)
			{ // if we are currently tracking, end trackball
				gTrackball = GL_FALSE;
				if (gTrackBallRotation[0] != 0.0)
				{
					// Mouse moves world object
					if (pContextInfo->camera[pContextInfo->currPort].thirdPerson == true)
					{
						if (pContextInfo->moveAllPortsTogether)
						{
							for (int x = 0; x < pContextInfo->numPorts; x++)
								addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[x].rotations.worldRotation);
						}
						
						else {
							addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[pContextInfo->currPort].rotations.worldRotation);
						}
					}
					else {
						if (pContextInfo->moveAllPortsTogether)
						{
							for (int x = 0; x < pContextInfo->numPorts; x++)
								addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[x].rotations.cameraRotation);
						}
						
						else {
							addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[pContextInfo->currPort].rotations.cameraRotation);
						}
					}
				}
				gTrackBallRotation [0] = gTrackBallRotation [1] = gTrackBallRotation [2] = gTrackBallRotation [3] = 0.0f;
			}
			else if (gDolly)
			{ // if we are currently dollying, end dolly
				gDolly = GL_FALSE;
			}
			gDollyPanStartPoint[0] = (GLint)x;
			gDollyPanStartPoint[1] = (GLint)y;
			gPan = GL_TRUE;
			gTrackingContextInfo = pContextInfo;
		}
		else if ((button == GLUT_MIDDLE_BUTTON) || ((button == GLUT_LEFT_BUTTON) && (modifiers == GLUT_ACTIVE_SHIFT)))
		{ // dolly
			if (gTrackball)
			{ // if we are currently tracking, end trackball
				gTrackball = GL_FALSE;
				if (gTrackBallRotation[0] != 0.0)
				{
					// Mouse moves world object
					if (pContextInfo->camera[pContextInfo->currPort].thirdPerson == true)
					{
						if (pContextInfo->moveAllPortsTogether)
						{
							for (int x = 0; x < pContextInfo->numPorts; x++)
								addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[x].rotations.worldRotation);
						}
						
						else {
							addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[pContextInfo->currPort].rotations.worldRotation);
						}
					}
					else {
						if (pContextInfo->moveAllPortsTogether)
						{
							for (int x = 0; x < pContextInfo->numPorts; x++)
								addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[x].rotations.cameraRotation);
						}
						
						else {
							addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[pContextInfo->currPort].rotations.cameraRotation);
						}
					}
				}
				gTrackBallRotation [0] = gTrackBallRotation [1] = gTrackBallRotation [2] = gTrackBallRotation [3] = 0.0f;
			}
			else if (gPan)
			{ // if we are currently panning, end pan
				gPan = GL_FALSE;
			}
			gDollyPanStartPoint[0] = (GLint)x;
			gDollyPanStartPoint[1] = (GLint)y;
			gDolly = GL_TRUE;
			gTrackingContextInfo = pContextInfo;
		}
		else if (button == GLUT_LEFT_BUTTON)
		{ // trackball
			if (gDolly)
			{ // if we are currently dollying, end dolly
				gDolly = GL_FALSE;
				gTrackingContextInfo = NULL;
			}
			else if (gPan)
			{ // if we are currently panning, end pan
				gPan = GL_FALSE;
				gTrackingContextInfo = NULL;
			}
			startTrackball((long)x, (long)y,
										 (long)pContextInfo->camera[pContextInfo->currPort].viewOriginX,
										 (long)pContextInfo->camera[pContextInfo->currPort].viewOriginY,
										 pContextInfo->globalCamera.viewWidth,
										 pContextInfo->globalCamera.viewHeight);
			gTrackball = GL_TRUE;
			gTrackingContextInfo = pContextInfo;
		}
	}

	
	
	if (state == GLUT_UP)
	{
		// stop trackball, pan, or dolly
		point3d p = GetOGLPos(pContextInfo, x, y);
		tButtonType bType = kLeftButton;
		switch (gCurrButton)
		{
			case GLUT_RIGHT_BUTTON: bType = kRightButton; break;
			case GLUT_LEFT_BUTTON: bType = kLeftButton; break;
			case GLUT_MIDDLE_BUTTON: bType = kMiddleButton; break;
		}
		if (HandleMouseClick(pContextInfo, x, y, p, bType, kMouseUp))
			return;

		
		// if we want to handle final movement when mouse is released
//		if (!pContextInfo->camera[pContextInfo->currPort].thirdPerson)
//		{
//		}

		
		if (gDolly) { // end dolly
			gDolly = GL_FALSE;
		} 
		else if (gPan) { // end pan
			gPan = GL_FALSE;
		} 
		else if (gTrackball) { // end trackball
			gTrackball = GL_FALSE;
			if (gTrackBallRotation[0] != 0.0)
			{
				// Mouse moves world object
				if (pContextInfo->camera[pContextInfo->currPort].thirdPerson == true)
				{
					if (pContextInfo->moveAllPortsTogether)
					{
						for (int x = 0; x < pContextInfo->numPorts; x++)
							addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[x].rotations.worldRotation);
					}
					
					else {
						addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[pContextInfo->currPort].rotations.worldRotation);
					}
				}
				else {
					if (pContextInfo->moveAllPortsTogether)
					{
						for (int x = 0; x < pContextInfo->numPorts; x++)
							addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[x].rotations.cameraRotation);
					}
					
					else {
						addToRotationTrackball(gTrackBallRotation, pContextInfo->camera[pContextInfo->currPort].rotations.cameraRotation);
					}
				}
			}
			gTrackBallRotation [0] = gTrackBallRotation [1] = gTrackBallRotation [2] = gTrackBallRotation [3] = 0.0f;
		} 
		gTrackingContextInfo = NULL;
	}
}


// move camera in x/y plane
static void mousePan (int x, int y, pRecContext pContextInfo)
{
	GLfloat panX = (gDollyPanStartPoint[0] - x) / (900.0f / -pContextInfo->camera[pContextInfo->currPort].viewPos.z);
	GLfloat panY = (gDollyPanStartPoint[1] - y) / (900.0f / -pContextInfo->camera[pContextInfo->currPort].viewPos.z);
	if (pContextInfo->moveAllPortsTogether)
	{
		for (int x = 0; x < pContextInfo->numPorts; x++)
		{
			pContextInfo->camera[x].viewPos.x += panX;
			pContextInfo->camera[x].viewPos.y += panY;
		}
	}
	else {
		pContextInfo->camera[pContextInfo->currPort].viewPos.x += panX;
		pContextInfo->camera[pContextInfo->currPort].viewPos.y += panY;
	}
	gDollyPanStartPoint[0] = (GLint) x;
	gDollyPanStartPoint[1] = (GLint) y;
}


// move camera in z axis
static void mouseDolly (int x, int y, pRecContext pContextInfo)
{
	GLfloat dolly = (gDollyPanStartPoint[1] - y)/20.f;// * -pContextInfo->camera[pContextInfo->currPort].viewPos.z / 300.0f;

	if (pContextInfo->moveAllPortsTogether)
	{
		for (int x = 0; x < pContextInfo->numPorts; x++)
		{
			pContextInfo->camera[x].aperture += dolly;
			if (pContextInfo->camera[x].aperture < 1)
				pContextInfo->camera[x].aperture = 1;
//			pContextInfo->camera[x].viewPos.z += dolly;
			if (pContextInfo->camera[x].viewPos.z == 0.0) // do not let z = 0.0
				pContextInfo->camera[x].viewPos.z = 0.0001;
			updateProjection(pContextInfo, x);  // update projection matrix
		}
	}
	else {
		pContextInfo->camera[pContextInfo->currPort].aperture += dolly;
		if (pContextInfo->camera[x].aperture < 1)
			pContextInfo->camera[x].aperture = 1;
//		pContextInfo->camera[pContextInfo->currPort].viewPos.z += dolly;
		if (pContextInfo->camera[pContextInfo->currPort].viewPos.z == 0.0) // do not let z = 0.0
			pContextInfo->camera[pContextInfo->currPort].viewPos.z = 0.0001;
		updateProjection(pContextInfo, pContextInfo->currPort);  // update projection matrix
	}
	gDollyPanStartPoint[0] = (GLint) x;
	gDollyPanStartPoint[1] = (GLint) y;
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
    drawGL(pContextInfo);
  }

}


/**
 * Called when the window is resized.  Specific format for GLUT.
 */
void resizeWindow(int x, int y)
{
	CGRect rect;
	while (0 != x%4)
		x++;
	while (0 != y%4)
		y++;
	rect.size.width = x;
	rect.size.height = y;
	rect.origin.x = 0;
	rect.origin.y = 0;
	resizeGL(pContextInfo, rect);
}


/**
 * Handles resizing of GL need context update and if the window dimensions change,
 * a window dimension update, reseting of viewport and an update of the projection matrix
 */
void resizeGL(pRecContext pContextInfo, CGRect viewRect)
{
	if (!pContextInfo)
			return;

		pContextInfo->globalCamera.viewOriginX = viewRect.origin.x;
		pContextInfo->globalCamera.viewOriginY = viewRect.origin.y;
		
		pContextInfo->globalCamera.viewWidth = (GLint)viewRect.size.width;
		pContextInfo->globalCamera.viewHeight = (GLint)viewRect.size.height;
		
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
		ratio = pContextInfo->camera[x].viewWidth / (float) pContextInfo->camera[x].viewHeight;
		if (ratio >= 1.0) {
			pContextInfo->camera[x].frust.left  = -ratio * wd2;
			pContextInfo->camera[x].frust.right = ratio * wd2;
			pContextInfo->camera[x].frust.top = wd2;
			pContextInfo->camera[x].frust.bottom = -wd2;
		} else {
			pContextInfo->camera[x].frust.left  = -wd2;
			pContextInfo->camera[x].frust.right = wd2;
			pContextInfo->camera[x].frust.top = wd2 / ratio;
			pContextInfo->camera[x].frust.bottom = -wd2 / ratio;
		}
	}
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
		gluLookAt (pContextInfo->camera[currPort].viewPos.x, pContextInfo->camera[currPort].viewPos.y, pContextInfo->camera[currPort].viewPos.z,
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
		glRotatef (pContextInfo->camera[currPort].rotations.cameraRotation[0],
				   pContextInfo->camera[currPort].rotations.cameraRotation[1],
				   pContextInfo->camera[currPort].rotations.cameraRotation[2],
				   pContextInfo->camera[currPort].rotations.cameraRotation[3]);

		if ((gTrackingContextInfo == pContextInfo) && gTrackBallRotation[0] != 0.0f) // if we have trackball rotation to map (this IS the test I want as it can be explicitly 0.0f)
		{
			if (pContextInfo->currPort == currPort || pContextInfo->moveAllPortsTogether)
				glRotatef (gTrackBallRotation[0], gTrackBallRotation[1], gTrackBallRotation[2], gTrackBallRotation[3]);
		}

		//glRotatef(fRot[0], fRot[1], fRot[2], fRot[3]);
//		glTranslated(-viewPos.x, -viewPos.y, -viewPos.z);

		glTranslatef(pContextInfo->camera[currPort].viewPos.x,
					 pContextInfo->camera[currPort].viewPos.y,
					 pContextInfo->camera[currPort].viewPos.z);

		if ((gTrackingContextInfo == pContextInfo) && gTrackBallRotation[0] != 0.0f) // if we have trackball rotation to map (this IS the test I want as it can be explicitly 0.0f)
		{
//			if (pContextInfo->currPort == currPort || pContextInfo->moveAllPortsTogether)
//				glRotatef (gTrackBallRotation[0], gTrackBallRotation[1], gTrackBallRotation[2], gTrackBallRotation[3]);
		}
		else {
		}
		
		// accumlated world rotation via trackball
		//	glRotatef (pContextInfo->rotations[currPort].worldRotation[0],
		//			   pContextInfo->rotations[currPort].worldRotation[1],
		//			   pContextInfo->rotations[currPort].worldRotation[2],
		//			   pContextInfo->rotations[currPort].worldRotation[3]);
		
		
//		glRotatef (pContextInfo->camera[currPort].rotations.worldRotation[0],
//				   pContextInfo->camera[currPort].rotations.worldRotation[1],
//				   pContextInfo->camera[currPort].rotations.worldRotation[2],
//				   pContextInfo->camera[currPort].rotations.worldRotation[3]);
//		
//		glTranslatef(pContextInfo->camera[currPort].viewPos.x,
//					 pContextInfo->camera[currPort].viewPos.y,
//					 pContextInfo->camera[currPort].viewPos.z);
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
	sprintf(&pContextInfo->message[ind+1], "%s", tempStr);

	delete myTextBox;
	point3d a(-.95, .95, -.95), b(.95, -.95, .95);
	recColor rc(1, 1, 1);
	myTextBox = new TextBox(pContextInfo->message, 120, a, b, 1000, true);
	myTextBox->setColor(rc);
}

void submitTextToBuffer(const char *val)
{
	strncpy(pContextInfo->message, val, 255);
	delete myTextBox;
	point3d a(-.95, .95, -.95), b(.95, -.95, .95);
	recColor rc(1, 1, 1);
	myTextBox = new TextBox(pContextInfo->message, 120, a, b, 1000, true);
	myTextBox->setColor(rc);
}


/**
 * Main OpenGL drawing function.
 */
void drawGL (pRecContext pContextInfo)
{
	if (!pContextInfo)
	 return;

	
	// clear our drawable
	glClear(GL_COLOR_BUFFER_BIT);
	
	for (int x = 0; x < pContextInfo->numPorts; x++)
	{
		glClear(GL_DEPTH_BUFFER_BIT);
		
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
			
			if (pContextInfo->currPort == x)
			{
				glMatrixMode(GL_PROJECTION); glLoadIdentity();
				glMatrixMode(GL_MODELVIEW); glLoadIdentity();
				gluOrtho2D(0, pContextInfo->camera[x].viewWidth, 0, pContextInfo->camera[x].viewHeight);
				glDisable(GL_LIGHTING);
				glColor3ub(255, 255, 255);
				glBegin(GL_LINE_LOOP);
				glVertex2i(0, 0);
				glVertex2i(pContextInfo->camera[x].viewWidth, 0);
				glVertex2i(pContextInfo->camera[x].viewWidth, pContextInfo->camera[x].viewHeight);
				glVertex2i(0, pContextInfo->camera[x].viewHeight);
				glEnd();
			}
		}
	}
	if (myTextBox)
	{
		myTextBox->stepTime(0.1);
		myTextBox->draw();
	}
	glutSwapBuffers();

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
void buildGL(void)
{ 
    	if (NULL == pContextInfo)
        	return;
		
	// build context
	CGRect viewRect = {{0.0f, 0.0f}, {0.0f, 0.0f}};
	
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
	glEnable(GL_DEPTH_TEST);

	glShadeModel(GL_SMOOTH);    
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	glFrontFace(GL_CCW);
	glPolygonOffset (1.0, 1.0);
		
	glClearColor(0.0,0.0,0.0,0.0);

	viewRect.size.width = glutGet(GLUT_WINDOW_WIDTH);
	viewRect.size.height = glutGet(GLUT_WINDOW_HEIGHT);
	
	// setup viewport and prespective
	resizeGL(pContextInfo, viewRect); // forces projection matrix update
		
	SetLighting (4);
}
