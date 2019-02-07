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

#include "GLUtil.h"
#include "Graphics.h"
#include <stdio.h>
#include <cstring>

#ifndef COMMON_H
#define COMMON_H

#define main hog_main

const int MAXPORTS = 6;

typedef struct {
	GLfloat worldRotation[4];
	GLfloat cameraRotation[4];
} recRotation;

typedef struct {
	GLdouble left, right, top, bottom, near, far;
} recFrustum;

typedef struct {
	recVec viewPos; // View position

	bool thirdPerson;
	
	// third-person (?) camera that can be programatically controlled using gluLookAt
	recVec viewDir; // View direction vector
	recVec viewUp; // View up direction
	recRotation rotations; // object/camera rotations

	// Other settings
	recFrustum frust; // set in updateProjection
	GLdouble aperture; // camera aperture
	GLint viewWidth,viewHeight; // current window/screen height and width
	GLfloat viewOriginX, viewOriginY; // always 0 
} recCamera;

enum viewportType {
	kScaleToSquare,
	kScaleToFill
};

struct viewport {
	Graphics::rect bounds;
	viewportType type;
	bool active; // Is this viewport valid
};

// per view data
struct recContext
{
	// camera handling
	recCamera globalCamera; // has full screen size; see resizeGL()
	
	recCamera camera[MAXPORTS];

	int numPorts, currPort;
	bool moveAllPortsTogether;
	viewport viewports[MAXPORTS];
	int windowHeight, windowWidth;
	
	char message[256]; // buffer for message output
	float msgTime; // message posting time for expiration
	
	Graphics::Display display;
	unsigned long windowID;
};
typedef struct recContext recContext;
typedef struct recContext * pRecContext;

// single set of interaction flags and states
extern GLfloat gTrackBallRotation [4];
extern pRecContext gTrackingContextInfo;

void updateProjection(pRecContext pContextInfo, int viewPort = -1);
void drawGL(pRecContext pContextInfo, bool swap);

bool DoKeyboardCommand(pRecContext pContextInfo, unsigned char keyHit, bool shift, bool cntrl, bool alt);
void initialConditions(pRecContext pContextInfo);
void resetCamera(recCamera * pCamera);

void ProcessCommandLineArgs(int argc, char *argv[]);

extern char gDefaultMap[1024];

/* Window Viewports */
void SetNumPorts(unsigned long windowID, int count);
int GetNumPorts(unsigned long windowID);

/* Not currently functional, but purposes is to mark a
 *  viewport as the one that is receiving input
 */
void SetActivePort(unsigned long windowID, int which);
int GetActivePort(unsigned long windowID);


/* Removes all active viewports and adds this one as the first.
 * rect coordinates are in HOG coordinates.
 */
void ReinitViewports(unsigned long windowID, const Graphics::rect &r, viewportType v);
/* Adds a new viewport to the existing viewports and
 * returns the new viewport numbers
 */
int AddViewport(unsigned long windowID, const Graphics::rect &r, viewportType v);

point3d ViewportToGlobalHOG(pRecContext pContextInfo, const viewport &v, point3d where);
point3d GlobalHOGToViewport(pRecContext pContextInfo, const viewport &v, point3d where);

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
	kMiddleButton, // option on the mac
	kNoButton
};

enum tMouseEventType : int {
	kMouseDown = 0x1,
	kMouseUp = 0x2,
	kMouseDrag = 0x4, // option on the mac
	kMouseMove = 0x8
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
 * the new callback handler also passes the viewport frame in which the click occured.
 * The loc is relative to the frame which was clicked
 */
typedef bool (*MouseCallback2)(unsigned long windowID, int viewport, int x, int y, point3d loc, tButtonType, tMouseEventType);

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
	mouseCallbackData(MouseCallback _mC, tMouseEventType _which)
	:mC(_mC), which(_which)
	{}
	MouseCallback mC;
	tMouseEventType which;
};

class mouseCallbackData2 {
public:
	mouseCallbackData2(MouseCallback2 _mC, tMouseEventType _which)
	:mC(_mC), which(_which)
	{}
	MouseCallback2 mC;
	tMouseEventType which;
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

void InstallMouseClickHandler(MouseCallback mC, tMouseEventType which = static_cast<tMouseEventType>(kMouseDown|kMouseUp|kMouseDrag));
void InstallMouseClickHandler(MouseCallback2 mC, tMouseEventType which = static_cast<tMouseEventType>(kMouseDown|kMouseUp|kMouseDrag));
void RemoveMouseClickHandler(MouseCallback mC);
void RemoveMouseClickHandler(MouseCallback2 mC);
bool HandleMouse(pRecContext pContextInfo, point3d where, tButtonType button, tMouseEventType mouse);
bool HandleMouseClick(pRecContext pContextInfo, int x, int y, point3d where, tButtonType, tMouseEventType);
bool HandleMouseClick(pRecContext pContextInfo, int viewport, int x, int y, point3d where, tButtonType, tMouseEventType);

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

void SaveScreenshot(unsigned long windowID, const char *filename);
void SetZoom(int windowID, float amount);

void submitTextToBuffer(const char *val);
void appendTextToBuffer(const char *);
const char *getTextBuffer();
pRecContext getCurrentContext();
pRecContext GetContext(unsigned long windowID);
void updateModelView(pRecContext pContextInfo, int currPort);
void cameraLookAt(GLfloat, GLfloat, GLfloat, float cameraSpeed = 0.1, int port = -1);
recVec cameraLookingAt(int port = -1);
void cameraMoveTo(GLfloat x, GLfloat y, GLfloat z, float cameraSpeed = 0.1, int port = -1);
void cameraOffset(GLfloat x, GLfloat y, GLfloat z, float cameraSpeed = 0.1, int port = -1);
void resetCamera();

point3d GetOGLPos(pRecContext pContextInfo, int x, int y);
recVec GetHeading(unsigned long windowID, int which);
void GetHeading(unsigned long windowID, int which, GLdouble &hx, GLdouble &hy, GLdouble &hz);

void setPortCamera(pRecContext pContextInfo, int currPort);
void setViewport(pRecContext pContextInfo, int currPort);
//void rotateObject();

#endif
