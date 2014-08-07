/*
 * $Id: common.cpp,v 1.22 2006/11/02 21:50:34 nathanst Exp $
 *
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

#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdint.h>

#include "GLUtil.h"
#include "Trackball.h"
#include "Common.h"

// For printing debug info
static bool const verbose = false;

static unsigned long gNextWindowID = 0;
char gDefaultMap[1024] = "";
const recVec gOrigin(0.0, 0.0, 0.0);

using namespace std;

static std::vector<commandLineCallbackData *> commandLineCallbacks;
static std::vector<joystickCallbackData *> joystickCallbacks;
static std::vector<mouseCallbackData *> mouseCallbacks;
static std::vector<windowCallbackData *> windowCallbacks;
static std::vector<frameCallbackData *> glDrawCallbacks;

static keyboardCallbackData *keyboardCallbacks[256] = 
{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

const char *getModifierText(tKeyboardModifier t)
{
	switch (t)
	{
		case kNoModifier: return "";
		case kAnyModifier: return "*-";
		case kShiftDown: return "Sft-";
		case kControlDown: return "Ctl-";
		case kAltDown: return "Alt-";
		default: return "?-";
	}
}

void InstallKeyboardHandler(KeyboardCallback kf, const char *title, const char *description,
														tKeyboardModifier mod, unsigned char firstKey, unsigned char lastKey)
{
	for (int x = firstKey; ; x++)
	{
		keyboardCallbacks[x] = new keyboardCallbackData(kf, title, description, mod, keyboardCallbacks[x]);
		if (x >= lastKey)
			break;
	}
}

void PrintKeyboardAssignments()
{
	printf("Legal Keyboard Commands\n-----------------------\n");
	for (int x = 0; x < 256; x++)
	{
		for (keyboardCallbackData *kd = keyboardCallbacks[x]; kd; kd = kd->next)
		{
			printf("%s%c\t%s\t%s\n", getModifierText(kd->mod), x, kd->title, kd->desc);
		}
	}
}


bool DoKeyboardCallbacks(pRecContext pContextInfo, unsigned char keyHit, tKeyboardModifier mod)
{
	bool called = false;
	for (keyboardCallbackData *kd = keyboardCallbacks[keyHit]; kd; kd = kd->next)
	{
		if ((mod == kd->mod) || (kd->mod == kAnyModifier))
		{
//			printf("(DEBUG) calling: %s%c\t%s\t%s\n", getModifierText(kd->mod), keyHit,
//						 kd->title, kd->desc);
			kd->call(pContextInfo->windowID, mod, keyHit);
			called = true;
		}
	}
	if (!called)
	{
		printf("Unknown keyboard command %s%c/%d\n\n", getModifierText(mod), keyHit, keyHit);
		PrintKeyboardAssignments();
	}
	return false;
}

void InstallFrameHandler(FrameCallback glCall, unsigned long windowID, void *userdata)
{
	glDrawCallbacks.push_back(new frameCallbackData(glCall, windowID, userdata));	
}

void RemoveFrameHandler(FrameCallback glCall, unsigned long windowID, void *userdata)
{
	for (unsigned int x = 0; x < glDrawCallbacks.size(); x++)
	{
		if ((glDrawCallbacks[x]->glCall == glCall) &&
				(glDrawCallbacks[x]->userData == userdata) &&
				(glDrawCallbacks[x]->windowID == windowID))
		{
			delete glDrawCallbacks[x];
			glDrawCallbacks[x] = glDrawCallbacks[glDrawCallbacks.size()-1];
			glDrawCallbacks.pop_back();
		}
	}
}

void HandleFrame(pRecContext pContextInfo, int viewport)
{
	glEnable(GL_BLEND); // for text fading
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // ditto
	for (unsigned int x = 0; x < glDrawCallbacks.size(); x++)
	{
		if (glDrawCallbacks[x]->windowID == pContextInfo->windowID)
			glDrawCallbacks[x]->glCall(pContextInfo->windowID, viewport, glDrawCallbacks[x]->userData);
	}
}

void InstallJoystickHandler(JoystickCallback jC, void *userdata)
{
	joystickCallbacks.push_back(new joystickCallbackData(jC, userdata));	
}

void RemoveJoystickHandler(JoystickCallback jC, void *userdata)
{
	for (unsigned int x = 0; x < joystickCallbacks.size(); x++)
	{
		if ((joystickCallbacks[x]->jC == jC) &&
				(joystickCallbacks[x]->userData == userdata))
		{
			delete joystickCallbacks[x];
			joystickCallbacks[x] = joystickCallbacks[joystickCallbacks.size()-1];
			joystickCallbacks.pop_back();
		}
	}
}

void HandleJoystickMovement(pRecContext pContextInfo, double panX, double panY)
{
	for (unsigned int x = 0; x < joystickCallbacks.size(); x++)
	{
		//printf("Calling joystick callback %d\n", x);
		joystickCallbacks[x]->jC(pContextInfo->windowID, panX, panY, joystickCallbacks[x]->userData);
	}
}

void InstallCommandLineHandler(CommandLineCallback CLC,
															 const char *arg, const char *param, const char *usage)
{
	commandLineCallbacks.push_back(new commandLineCallbackData(CLC, arg, param, usage));
}

void PrintCommandLineArguments()
{
	printf("Valid command-line flags:\n\n");
	for (unsigned int x = 0; x < commandLineCallbacks.size(); x++)
		commandLineCallbacks[x]->Print();
	printf("\n");
}

// Process command line arguments
void ProcessCommandLineArgs(int argc, char *argv[])
{
	for (int x = 0; x < argc; x++)
	{
		printf("%s ", argv[x]);
	}
	printf("\n");
	//initializeCommandLineHandlers();
	// printCommandLineArguments();

	int lastval;
	for (int y = 1; y < argc; )
	{
		lastval = y;
		for (unsigned int x = 0; x < commandLineCallbacks.size(); x++)
		{
			if (strcmp(commandLineCallbacks[x]->argument, argv[y]) == 0)
			{
				y += commandLineCallbacks[x]->CLC(&argv[y], argc-y);
				break;
			}
		}
		if (lastval == y)
		{
			printf("Error: unhandled command-line parameter (%s)\n\n", argv[y]);
			PrintCommandLineArguments();
			y++;
			//exit(10);
		}
	}
}

void InstallMouseClickHandler(MouseCallback mC)
{
	mouseCallbacks.push_back(new mouseCallbackData(mC));
}

void RemoveMouseClickHandler(MouseCallback mC)
{
	for (unsigned int x = 0; x < mouseCallbacks.size(); x++)
	{
		if (mouseCallbacks[x]->mC == mC)
		{
			delete mouseCallbacks[x];
			mouseCallbacks[x] = mouseCallbacks[mouseCallbacks.size()-1];
			mouseCallbacks.pop_back();
		}
	}
}

// this is called by the OS when it gets a click
bool HandleMouseClick(pRecContext pContextInfo, int x, int y, point3d where,
											tButtonType button, tMouseEventType mouse)
{
	for (unsigned int j = 0; j < mouseCallbacks.size(); j++)
	{
		//printf("Calling mouse callback %d\n", x);
		if (mouseCallbacks[j]->mC(pContextInfo->windowID, x, y, where,
															button, mouse))
			return true;
	}
	return false;
}

void InstallWindowHandler(WindowCallback wC)
{
	windowCallbacks.push_back(new windowCallbackData(wC));
}

void RemoveWindowHandler(WindowCallback wC)
{
	for (unsigned int x = 0; x < windowCallbacks.size(); x++)
	{
		if (windowCallbacks[x]->wC == wC)
		{
			delete windowCallbacks[x];
			windowCallbacks[x] = windowCallbacks[windowCallbacks.size()-1];
			windowCallbacks.pop_back();
		}
	}
}

void HandleWindowEvent(pRecContext pContextInfo, tWindowEventType e)
{
	for (unsigned int j = 0; j < windowCallbacks.size(); j++)
	{
		//printf("Calling window callback %d\n", x);
		windowCallbacks[j]->wC(pContextInfo->windowID, e);
	}
}

// intializes context conditions
void initialConditions(pRecContext pContextInfo)
{
	pContextInfo->moveAllPortsTogether = true;
	pContextInfo->info = kInfoState;
	pContextInfo->animate = kAnimateState;
	pContextInfo->drawCaps = 0;
	pContextInfo->drawHelp = 1;
	pContextInfo->polygons = 1;
	pContextInfo->lines = 0;
	pContextInfo->points = 0;
	pContextInfo->showCredits = 1;
	pContextInfo->lighting = 4;
	SetLighting(4);
//	pContextInfo->paused = false;
	pContextInfo->drawing = true;
	pContextInfo->numPorts = 3;
	pContextInfo->currPort = 0;
	for (int x = 0; x < MAXPORTS; x++)
	{
		resetCamera(&pContextInfo->camera[x]);
		for (int y = 0; y < 4; y++)
		{
			pContextInfo->rotations[x].worldRotation[y] = 0;
			pContextInfo->rotations[x].objectRotation[y] = 0;
		}
	}
	
#ifdef OS_MAC
	pContextInfo->timer = NULL;
#endif
	
	gTrackBallRotation [0] = gTrackBallRotation [1] = gTrackBallRotation [2] = gTrackBallRotation [3] = 0.0f;
	
	pContextInfo->surface = 0; 
	pContextInfo->colorScheme = 4;
	pContextInfo->subdivisions = 64;
	pContextInfo->xyRatio = 1;
	pContextInfo->modeFSAA = 1;

	pContextInfo->windowID = gNextWindowID++;
}

bool DoKeyboardCommand(pRecContext pContextInfo, unsigned char keyHit, bool shift, bool cntrl, bool alt)
{
	DoKeyboardCallbacks(pContextInfo, tolower(keyHit), 
											shift?kShiftDown:(cntrl?kControlDown:(alt?kAltDown:kNoModifier)));
	return false;
}

void SetLighting(unsigned int mode)
{
	GLfloat mat_specular[] = {0.2, 0.2, 0.2, 1.0};
	GLfloat mat_shininess[] = {50.0};
	
//	GLfloat position[4] = {7.0,-7.0,12.0,0.0};
//	GLfloat position[4] = {-1.0,-3.0,5.0,0.0};
	GLfloat position[4] = {-1.0,5.0,5.0,0.0};
	GLfloat ambient[4]  = {0.2, 0.2, 0.2, 1.0};
	GLfloat diffuse[4]  = {1.0, 1.0, 1.0, 1.0};
	GLfloat specular[4] = {1.0, 1.0, 1.0, 1.0};
	
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
	
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
	
	switch (mode) {
		case 0:
			break;
		case 1:
			glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_FALSE);
			glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,GL_FALSE);
			break;
		case 2:
			glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_FALSE);
			glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,GL_TRUE);
			break;
		case 3:
			glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);
			glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,GL_FALSE);
			break;
		case 4:
			glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);
			glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,GL_TRUE);
			break;
	}
	
	glLightfv(GL_LIGHT0,GL_POSITION,position);
	glLightfv(GL_LIGHT0,GL_AMBIENT,ambient);
	glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuse);
	glLightfv(GL_LIGHT0,GL_SPECULAR,specular);
	glEnable(GL_LIGHT0);
}

void resetCamera()
{
	pRecContext pContextInfo = getCurrentContext();
	if (!pContextInfo)
		return;
	pContextInfo->numPorts = 1;
	for (int x = 0; x < MAXPORTS; x++)
	{
		resetCamera(&pContextInfo->camera[x]);
		for (int y = 0; y < 4; y++)
		{
			pContextInfo->rotations[x].worldRotation[y] = 0;
			pContextInfo->rotations[x].objectRotation[y] = 0;
		}
	}
	
	gTrackBallRotation [0] = gTrackBallRotation [1] = gTrackBallRotation [2] = gTrackBallRotation [3] = 0.0f;
	gTrackingContextInfo = 0;

	updateProjection(pContextInfo);  // update projection matrix
//	updateModelView(pContextInfo);
}

// sets the camera data to initial conditions
void resetCamera(recCamera * pCamera)
{
	pCamera->aperture = 10.0;
	pCamera->rotPoint = gOrigin;
	
	pCamera->viewPos.x = 0.0;
	pCamera->viewPos.y = 0.0;
	pCamera->viewPos.z = -12.5;
	pCamera->viewDir.x = -pCamera->viewPos.x; 
	pCamera->viewDir.y = -pCamera->viewPos.y; 
	pCamera->viewDir.z = -pCamera->viewPos.z;
	
	pCamera->viewUp.x = 0;  
	pCamera->viewUp.y = -.1; 
	pCamera->viewUp.z = -1;
}


void cameraLookAt(GLfloat x, GLfloat y, GLfloat z, float cameraSpeed, int port)
{
	pRecContext pContextInfo = getCurrentContext();
	if (!pContextInfo)
		return; 
//	const float cameraSpeed = .1;
	if (port == -1)
	{
		pContextInfo->camera[pContextInfo->currPort].viewDir.x = (1-cameraSpeed)*pContextInfo->camera[pContextInfo->currPort].viewDir.x + cameraSpeed*(x - pContextInfo->camera[pContextInfo->currPort].viewPos.x);
		pContextInfo->camera[pContextInfo->currPort].viewDir.z = (1-cameraSpeed)*pContextInfo->camera[pContextInfo->currPort].viewDir.z + cameraSpeed*(z - pContextInfo->camera[pContextInfo->currPort].viewPos.z);
		pContextInfo->camera[pContextInfo->currPort].viewDir.y = (1-cameraSpeed)*pContextInfo->camera[pContextInfo->currPort].viewDir.y + cameraSpeed*(y - pContextInfo->camera[pContextInfo->currPort].viewPos.y);

		pContextInfo->rotations[pContextInfo->currPort].objectRotation[0] *= (1-cameraSpeed);
		pContextInfo->rotations[pContextInfo->currPort].worldRotation[0] *= (1-cameraSpeed);
		updateProjection(pContextInfo);
	}
	else {
		pContextInfo->camera[port].viewDir.x = (1-cameraSpeed)*pContextInfo->camera[port].viewDir.x + cameraSpeed*(x - pContextInfo->camera[port].viewPos.x);
		pContextInfo->camera[port].viewDir.z = (1-cameraSpeed)*pContextInfo->camera[port].viewDir.z + cameraSpeed*(z - pContextInfo->camera[port].viewPos.z);
		pContextInfo->camera[port].viewDir.y = (1-cameraSpeed)*pContextInfo->camera[port].viewDir.y + cameraSpeed*(y - pContextInfo->camera[port].viewPos.y);
		
		pContextInfo->rotations[port].objectRotation[0] *= (1-cameraSpeed);
		pContextInfo->rotations[port].worldRotation[0] *= (1-cameraSpeed);
		updateProjection(pContextInfo);
	}
}

void cameraMoveTo(GLfloat x, GLfloat y, GLfloat z, float cameraSpeed, int port)
{
	pRecContext pContextInfo = getCurrentContext();
	if (!pContextInfo)
		return;
//	const float cameraSpeed = .1;
	if (port == -1)
	{
		pContextInfo->camera[pContextInfo->currPort].viewPos.x = (1-cameraSpeed)*pContextInfo->camera[pContextInfo->currPort].viewPos.x + cameraSpeed*x;
		pContextInfo->camera[pContextInfo->currPort].viewPos.y = (1-cameraSpeed)*pContextInfo->camera[pContextInfo->currPort].viewPos.y + cameraSpeed*y;
		pContextInfo->camera[pContextInfo->currPort].viewPos.z = (1-cameraSpeed)*pContextInfo->camera[pContextInfo->currPort].viewPos.z + cameraSpeed*z;
		updateProjection(pContextInfo);
	}
	else {
		pContextInfo->camera[port].viewPos.x = (1-cameraSpeed)*pContextInfo->camera[port].viewPos.x + cameraSpeed*x;
		pContextInfo->camera[port].viewPos.y = (1-cameraSpeed)*pContextInfo->camera[port].viewPos.y + cameraSpeed*y;
		pContextInfo->camera[port].viewPos.z = (1-cameraSpeed)*pContextInfo->camera[port].viewPos.z + cameraSpeed*z;
		updateProjection(pContextInfo);
	}
}

void setPortCamera(pRecContext pContextInfo, int currPort)
{
	const double ratios[4][4][4] =
{{{0, 1, 0, 1}},
{{0, 0.5, 0, 1}, {0.5, 0.5, 0, 1}},
{{0, 0.5, 0.5, 0.5}, {0.5, 0.5, 0.5, 0.5}, {0, 1, 0, 0.5}},
{{0, 0.5, 0.5, 0.5}, {0.5, 0.5, 0.5, 0.5}, {0, 0.5, 0, 0.5}, {0.5, 0.5, 0, 0.5}}};
	
	const double *val = ratios[pContextInfo->numPorts-1][currPort];
	
	pContextInfo->camera[currPort].viewOriginX = pContextInfo->globalCamera.viewOriginX;
	pContextInfo->camera[currPort].viewOriginY = pContextInfo->globalCamera.viewOriginY;
	
	pContextInfo->camera[currPort].viewWidth = (GLint)(val[1]*pContextInfo->globalCamera.viewWidth);
	pContextInfo->camera[currPort].viewHeight = (GLint)(val[3]*pContextInfo->globalCamera.viewHeight);
	//	printf("Window %d port %d width: %d, height %d\n",
	//				 pContextInfo->windowID, currPort,
	//				 pContextInfo->camera[currPort].viewWidth,
	//				 pContextInfo->camera[currPort].viewHeight);
}

void setViewport(pRecContext pContextInfo, int currPort)
{
	const double ratios[4][4][4] =
{{{0, 1, 0, 1}}, // x, width%, y, height%
{{0, 0.5, 0, 1}, {0.5, 0.5, 0, 1}},
{{0, 0.5, 0.5, 0.5}, {0.5, 0.5, 0.5, 0.5}, {0, 1, 0, 0.5}},
{{0, 0.5, 0.5, 0.5}, {0.5, 0.5, 0.5, 0.5}, {0, 0.5, 0, 0.5}, {0.5, 0.5, 0, 0.5}}};
	
	const double *val = ratios[pContextInfo->numPorts-1][currPort];
	
	glViewport(val[0]*pContextInfo->globalCamera.viewWidth,
						 val[2]*pContextInfo->globalCamera.viewHeight,
						 val[1]*pContextInfo->globalCamera.viewWidth,
						 val[3]*pContextInfo->globalCamera.viewHeight);
}

point3d GetOGLPos(pRecContext pContextInfo, int x, int y)
{
	setViewport(pContextInfo, pContextInfo->currPort);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(pContextInfo->frust[pContextInfo->currPort].left, pContextInfo->frust[pContextInfo->currPort].right,
			  pContextInfo->frust[pContextInfo->currPort].bottom, pContextInfo->frust[pContextInfo->currPort].top,
			  pContextInfo->frust[pContextInfo->currPort].near, pContextInfo->frust[pContextInfo->currPort].far);
	// projection matrix already set	
	updateModelView(pContextInfo, pContextInfo->currPort);
	
	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];
	GLfloat winX, winY, winZ;
	GLdouble posX, posY, posZ;
	
	glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
	glGetDoublev( GL_PROJECTION_MATRIX, projection );
	glGetIntegerv( GL_VIEWPORT, viewport );
	winX = (float)x;
	winY = (float)viewport[3] - (float)y;
	glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
	
	if (gluUnProject( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ) == GL_FALSE)
		printf("WARNING: gluUnProject failed\n");
	
	return point3d(posX, posY, posZ);
}

void SetNumPorts(int windowID, int count)
{
	pRecContext pContextInfo = GetContext(windowID);
	if ((count <= MAXPORTS) && (count > 0))
	{
		if (pContextInfo->numPorts > count)
			pContextInfo->currPort = 0;

		pContextInfo->numPorts = count;
		for (int x = 0; x < pContextInfo->numPorts; x++)
		{
			setPortCamera(pContextInfo, x);
		}
		updateProjection(pContextInfo);
	}
}

int GetNumPorts(int windowID)
{
	pRecContext pContextInfo = GetContext(windowID);
	return pContextInfo->numPorts;
}

int GetActivePort(int windowID)
{
	pRecContext pContextInfo = GetContext(windowID);
	return pContextInfo->currPort;
}

void SetActivePort(int windowID, int which)
{
	pRecContext pContextInfo = GetContext(windowID);
	if ((which >= 0) && (which < pContextInfo->numPorts))
		pContextInfo->currPort = which;
}


class bmp_header {
public:
	bmp_header()
	://bfType(19778),
	zero(0), bfOffBits(sizeof(bmp_header)+2), biSize(40), biPlanes(1),
	biBitCount(32), biCompression(0), biSizeImage(0), biXPelsPerMeter(2835), biYPelsPerMeter(2835),
	biClrUsed(0), biClrImportant(0) {}
	
//	uint16_t bfType;//	19778
	uint32_t bfSize; //	??	specifies the size of the file in bytes.
	uint32_t zero; // 0
	uint32_t bfOffBits;
	//	11	4	bfOffBits	1078	specifies the offset from the beginning of the file to the bitmap data.
	
	uint32_t biSize; // 40
	uint32_t biWidth;
	uint32_t biHeight;
	uint16_t biPlanes; // 0 (1??)
	uint16_t biBitCount; // 24
	uint32_t biCompression; // 0
	uint32_t biSizeImage; // 0
	uint32_t biXPelsPerMeter; // 0
	uint32_t biYPelsPerMeter; // 0
	uint32_t biClrUsed; // 0
	uint32_t biClrImportant; // 0
};

void SaveScreenshot(int windowID, const char *filename)
{
	pRecContext pContextInfo = GetContext(windowID);

	char file[strlen(filename)+5];
	sprintf(file, "%s.bmp", filename);
	FILE *f = fopen(file, "w+");

	if (f == 0) return;
	
//	3	4	bfSize	??	specifies the size of the file in bytes.
//	19	4	biWidth	100	specifies the width of the image, in pixels.
//	23	4	biHeight	100	specifies the height of the image, in pixels.

	
	long width  = pContextInfo->globalCamera.viewWidth;
	long height  =pContextInfo->globalCamera.viewHeight;
	long rowBytes = width * 4;
	long imageSize = rowBytes * height;
	char image[imageSize];
	char zero[4] = {0, 0, 0, 0};
	glReadPixels(0, 0, width, height, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, &image);//GL_BGRA
	
	bmp_header h;
	h.biWidth = width;
	h.biHeight = height;
	int buffer = (4-width%4)%4;
	h.bfSize = sizeof(bmp_header)+2+(width+buffer)*height*4;
	h.biSizeImage = (width+buffer)*height*4;
	uint16_t bfType = 19778;
	fwrite(&bfType, sizeof(bfType), 1, f);
	fwrite(&h, sizeof(bmp_header), 1, f);
	for (int x = 0; x < height; x++)
	{
		fwrite(&image[x*width*4], sizeof(char), width*4, f);
		if (0 != width%4)
			fwrite(&zero, sizeof(char), buffer, f);
	}
	fclose(f);
}

void SetZoom(int windowID, float amount)
{
	pRecContext pContextInfo = GetContext(windowID);

	if (pContextInfo->moveAllPortsTogether)
	{
		for (int x = 0; x < pContextInfo->numPorts; x++)
		{
			pContextInfo->camera[x].viewPos.z = -12.5+amount;
			if (pContextInfo->camera[x].viewPos.z == 0.0) // do not let z = 0.0
				pContextInfo->camera[x].viewPos.z = 0.0001;
			updateProjection(pContextInfo, x);  // update projection matrix
		}
	}
	else {
		pContextInfo->camera[pContextInfo->currPort].viewPos.z = -12.5+amount;
		if (pContextInfo->camera[pContextInfo->currPort].viewPos.z == 0.0) // do not let z = 0.0
			pContextInfo->camera[pContextInfo->currPort].viewPos.z = 0.0001;
		updateProjection(pContextInfo, pContextInfo->currPort);  // update projection matrix
	}
}

