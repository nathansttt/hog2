/*
 * $Id: common.h,v 1.20 2006/11/02 00:12:16 nathanst Exp $
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

//#include "Map.h"
//#include "MapAbstraction.h"
//#include "UnitSimulation.h"
#include "GLUtil.h"
#include <stdio.h>
#include <cstring>
//#include "StatCollection.h"

#ifndef COMMON_H
#define COMMON_H

enum
{
	kMainMenu = 500,
	kCloseMenuItem = 2,
	kInfoMenuItem = 4,
	kAnimateMenuItem = 5,
	kInfoState = 1,
	kAnimateState = 0
};

enum {
	kFSAAOff = 0,
	kFSAAFast = 1,
	kFSAANice = 2,
	kSamples = 4
};

const int MAXPORTS = 4;
// camera/context info
//typedef struct {
//	GLdouble x,y,z;
//} recVec;

typedef struct {
	recVec viewPos; // View position
	recVec viewDir; // View direction vector
	recVec viewUp; // View up direction
	recVec rotPoint; // Point to rotate about
	GLdouble aperture; // camera aperture
	GLint viewWidth,viewHeight; // current window/screen height and width
	GLfloat viewOriginX, viewOriginY; // always 0 
} recCamera;

typedef struct {
	GLfloat worldRotation[4];
	GLfloat objectRotation[4];
} recRotation;	

typedef struct {
	GLdouble left, right, top, bottom, near, far;
} recFrustum;

// per view data
struct recContext
{
#ifdef OS_MAC
	AGLPixelFormat aglPixFmt;
	AGLContext aglContext;
	GLuint boldFontList;
	GLuint regFontList;
	EventLoopTimerRef timer;
	//DMExtendedNotificationUPP  windowEDMUPP;
	AbsoluteTime time;
#endif

	bool animate;
	bool info;
	bool drawHelp;
	bool drawCaps;
	bool polygons;
	bool lines;
	bool points;
	bool showCredits;
	long lighting;
//	bool paused;
	bool drawing;
	
	// spin
//	GLfloat fRot[3];
//	GLfloat fVel[3];
//	GLfloat fAccel[3];
	
	// camera handling
	recCamera globalCamera;
	recCamera camera[MAXPORTS];
	recRotation rotations[MAXPORTS];
	recFrustum frust[MAXPORTS];
	int numPorts, currPort;
	GLfloat shapeSize;
	bool moveAllPortsTogether;
	
	char message[256]; // buffer for message output
	float msgTime; // message posting time for expiration
	
	char * names;
	
	long surface; 
	long colorScheme;
	unsigned long subdivisions;
	unsigned long xyRatio;
	
	long modeFSAA;
	unsigned long windowID;
};
typedef struct recContext recContext;
typedef struct recContext * pRecContext;

// single set of interaction flags and states
extern GLfloat gTrackBallRotation [4];
extern pRecContext gTrackingContextInfo;

void updateProjection(pRecContext pContextInfo, int viewPort = -1);
void drawGL(pRecContext pContextInfo, bool swap);
#ifdef OS_MAC
pRecContext GetCurrentContextInfo (WindowRef window);
#endif

bool DoKeyboardCommand(pRecContext pContextInfo, unsigned char keyHit, bool shift, bool cntrl, bool alt);
void SetLighting(unsigned int mode);
void initialConditions(pRecContext pContextInfo);
void resetCamera(recCamera * pCamera);

void ProcessCommandLineArgs(int argc, char *argv[]);

extern char gDefaultMap[1024];

void SetNumPorts(unsigned long windowID, int count);
int GetNumPorts(unsigned long windowID);
void SetActivePort(unsigned long windowID, int which);
int GetActivePort(unsigned long windowID);

enum tKeyboardModifier {
	kNoModifier,
	kAnyModifier,
	kShiftDown,
	kControlDown,
	kAltDown // option on the mac
};

enum tButtonType {
	kLeftButton,
	kRightButton,
	kMiddleButton // option on the mac
};

enum tMouseEventType {
	kMouseDown,
	kMouseUp,
	kMouseDrag // option on the mac
};

enum tWindowEventType {
	kWindowCreated,
	kWindowDestroyed
};

/**
* install a FrameCallback to be called once per frame and viewport
 * This is where you should do any drawing and/or update the unit simulation time.
 * The viewports will be processed in order (0..#, max 4), so any global processing
 * should only be done when the viewport is 0.
 * The void* data is passed back to each call.
 */
typedef void (*FrameCallback)(unsigned long windowID, unsigned int viewport, void*);

/**
* a window callback handler called when a window is created or destroyed
 */
typedef void (*WindowCallback)(unsigned long windowID, tWindowEventType);

/**
 * a joystaick callback handler is passed the same data passed in when it was installed
 */
typedef void (*JoystickCallback)(unsigned long windowID, double offsetX, double offsetY, void*);

/**
 * a mouse callback handler passes the absolute and local mouse coordinates of the click
 * returns true if the click/movement was handled; false if ignored
 */
typedef bool (*MouseCallback)(unsigned long windowID, int x, int y, point3d loc, tButtonType, tMouseEventType);

/**
 * a keyboard callback handler is passed the current unit simulation, the key
 * that was hit, and any modifiers that were being held down. Note that the
 * modifiers can change the character being passed.
 */
typedef void (*KeyboardCallback)(unsigned long windowID, tKeyboardModifier, char);
/**
 * A command-line callback handler takes in a char** which points to the argument
 * which matches the argument that this handler was installed for. The second
 * argument is the number of remaining arguments available. The handler
 * can process and use as many of the remaining arguments, and returns the number
 * of arguments it processed.
 */
typedef int (*CommandLineCallback)(char**, int);

class commandLineCallbackData {
public:
	commandLineCallbackData(CommandLineCallback _CLC, const char *_argument,
													const char *_param, const char *_desc)
	:CLC(_CLC), argument(_argument), param(_param), desc(_desc) {}

	void Print()
	{ printf("%s : %s\n     %s\n", argument, param, desc); }
	
	CommandLineCallback CLC;
	const char *argument;
	const char *param;
	const char *desc;
};

class keyboardCallbackData {
public:
	keyboardCallbackData(KeyboardCallback kc, const char *_title, const char *_desc,
											 tKeyboardModifier _mod, keyboardCallbackData *_next = 0)
	:call(kc), title(_title), desc(_desc), mod(_mod), next(_next) {}
	
	KeyboardCallback call;
	const char *title;
	const char *desc;
	tKeyboardModifier mod;
	keyboardCallbackData *next;
};

class joystickCallbackData {
public:
	joystickCallbackData(JoystickCallback _jC, void *_userData)
	:jC(_jC)
	{ userData = _userData; }

	JoystickCallback jC;
	void *userData;
};

class mouseCallbackData {
public:
	mouseCallbackData(MouseCallback _mC)
	:mC(_mC)
	{}
	MouseCallback mC;
};

class windowCallbackData {
public:
	windowCallbackData(WindowCallback _wC)
	:wC(_wC)
	{}
	WindowCallback wC;
};

class frameCallbackData {
public:
	frameCallbackData(FrameCallback _glCall, unsigned long _windowID, void *_userData)
	:glCall(_glCall)
	{ windowID = _windowID;
		userData = _userData; }
	
	FrameCallback glCall;
	unsigned long windowID;
	void *userData;
};

void InstallFrameHandler(FrameCallback jC, unsigned long windowID, void *userdata);
void RemoveFrameHandler(FrameCallback jC, unsigned long windowID, void *userdata);
void HandleFrame(pRecContext pContextInfo, int viewport);

void InstallJoystickHandler(JoystickCallback jC, void *userdata);
void RemoveJoystickHandler(JoystickCallback jC, void *userdata);
void HandleJoystickMovement(pRecContext pContextInfo, double panX, double panY);

void InstallMouseClickHandler(MouseCallback mC);
void RemoveMouseClickHandler(MouseCallback mC);
bool HandleMouseClick(pRecContext pContextInfo, int x, int y, point3d where, tButtonType, tMouseEventType);

void InstallWindowHandler(WindowCallback wC);
void RemoveWindowHandler(WindowCallback wC);
void HandleWindowEvent(pRecContext pContextInfo, tWindowEventType);

void InstallKeyboardHandler(KeyboardCallback kf, const char *title, const char *description,
														tKeyboardModifier mod, unsigned char firstKey, unsigned char lastKey = 0);
void PrintKeyboardAssignments();
void InstallCommandLineHandler(CommandLineCallback, const char *, const char *, const char *);
void PrintCommandLineArguments();

void ProcessCommandLineArgs(int argc, char *argv[]);

void RunHOGGUI(int argc, char* argv[], int windowDimension = 1000);
void RunHOGGUI(int argc, char* argv[], int xDimension, int yDimension);
void SaveScreenshot(int windowID, const char *filename);
void SetZoom(int windowID, float amount);

void submitTextToBuffer(const char *val);
void appendTextToBuffer(char *);
pRecContext getCurrentContext();
pRecContext GetContext(unsigned int windowID);
void updateModelView(pRecContext pContextInfo, int currPort);
void cameraLookAt(GLfloat, GLfloat, GLfloat, float cameraSpeed = 0.1, int port = -1);
void cameraMoveTo(GLfloat x, GLfloat y, GLfloat z, float cameraSpeed = 0.1, int port = -1);
void resetCamera();
point3d GetOGLPos(pRecContext pContextInfo, int x, int y);

void setPortCamera(pRecContext pContextInfo, int currPort);
void setViewport(pRecContext pContextInfo, int currPort);

#endif
