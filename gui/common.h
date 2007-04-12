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

#include "map.h"
#include "mapAbstraction.h"
#include "unitSimulation.h"
#include "glUtil.h"
#include "statCollection.h"

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

// per view data
struct recContext
{
#ifdef OS_MAC
	AGLPixelFormat aglPixFmt;
	AGLContext aglContext;
	GLuint boldFontList;
	GLuint regFontList;
	EventLoopTimerRef timer;
	DMExtendedNotificationUPP  windowEDMUPP;
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
	bool showLand;
	long lighting;
//	bool paused;
	bool drawing;
	
	// spin
	GLfloat fRot [3];
	GLfloat fVel [3];
	GLfloat fAccel [3];
	
	// camera handling
	recCamera camera;
	GLfloat worldRotation [4];
	GLfloat objectRotation [4];
	GLfloat shapeSize;
	
	char message[256]; // buffer for message output
	float msgTime; // message posting time for expiration
	
	char * names;
	
	long surface; 
	long colorScheme;
	unsigned long subdivisions;
	unsigned long xyRatio;
	
	long modeFSAA;
	
	//Map *map;
	//mapAbstraction *abstrMap;
	unitSimulation *unitLayer;
  //humanUnit *human;
};
typedef struct recContext recContext;
typedef struct recContext * pRecContext;

// single set of interaction flags and states
extern GLfloat gTrackBallRotation [4];
extern pRecContext gTrackingContextInfo;

void updateProjection(pRecContext pContextInfo);
void drawGL(pRecContext pContextInfo, bool swap);
#ifdef OS_MAC
pRecContext GetCurrentContextInfo (WindowRef window);
#endif

bool doKeyboardCommand(pRecContext pContextInfo, unsigned char keyHit, bool shift, bool cntrl, bool alt);
void SetLighting(unsigned int mode);
void initialConditions(pRecContext pContextInfo);
void resetCamera(recCamera * pCamera);

void processCommandLineArgs(int argc, char *argv[]);
void handleBatchRTS(int argc, char *argv[], int mode, char *_map);
void handleBatchPRA(int argc, char *argv[], int mode, char *_map);
//void runBatchMode(char *mode, char *_map, int xxs, int yys, int xxg, 
//				  int yyg, int, double, double, unsigned int, bool _drawMap, double);
void runSimpleUnitTest(char *_map);
void runLRTS(char *_map, int xxs, int yys, int xxg, int yyg, 
			 int _d, double _gamma, double _LQ, unsigned _numRuns, 
			 bool _drawMap, int _dist);
void runPRLRTS(char *_map, int xxs, int yys, int xxg, int yyg, 
			 int _d, double _gamma, double _LQ, unsigned _numRuns, bool _drawMap,
			   int _dist);
void randomPathTesting(char *_map);
void pickStartEnd(int& xs, int& ys, int& xg, int& yg, 
				  Map *map, graph *g, mapAbstraction* aMap,
				  double& spDist, double& aStarDifficulty, int);
void scenarioGenerator(char *_map);
void discrepancy(char* scenName, char* algName, 
				 int _d=1, double _gamma=1.0, double _LQ=99999999999.0);

void runScenario(char* scenName, char* algName, 
				 int _d=1, double _gamma=1.0, double _LQ=99999999999.0);

void calculateAverageTimePerStep(char *_map, char *coordFile, int MAP_SIZE);

void calculateSubOptimality(char *_map, char *coordFile, int MAP_SIZE);
extern char gDefaultMap[1024];


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

/**
 * a joystaick callback handler is passed the same data passed in when it was installed
 */
typedef void (*joystickCallback)(unitSimulation *, double offsetX, double offsetY, void*);

/**
 * a mouse callback handler passes the absolute and local mouse coordinates of the click
 * returns true if the click/movement was handled; false if ignored
 */
typedef bool (*mouseCallback)(unitSimulation *, int x, int y, point3d loc, tButtonType, tMouseEventType);

/**
 * a keyboard callback handler is passed the current unit simulation, the key
 * that was hit, and any modifiers that were being held down. Note that the
 * modifiers can change the character being passed.
 */
typedef void (*keyboardCallback)(unitSimulation *,tKeyboardModifier,char);
/**
 * A command-line callback handler takes in a char** which points to the argument
 * which matches the argument that this handler was installed for. The second
 * argument is the number of remaining arguments available. The handler
 * can process and use as many of the remaining arguments, and returns the number
 * of arguments it processed.
 */
typedef int (*commandLineCallback)(char**, int);

class commandLineCallbackData {
public:
	commandLineCallbackData(commandLineCallback _CLC, const char *_argument,
													const char *_param, const char *_desc)
	:CLC(_CLC), argument(_argument), param(_param), desc(_desc) {}

	void Print()
	{ printf("%s : %s\n     %s\n", argument, param, desc); }
	
	commandLineCallback CLC;
	const char *argument;
	const char *param;
	const char *desc;
};

class keyboardCallbackData {
public:
	keyboardCallbackData(keyboardCallback kc, const char *_title, const char *_desc,
											 tKeyboardModifier _mod, keyboardCallbackData *_next = 0)
	:call(kc), title(_title), desc(_desc), mod(_mod), next(_next) {}
	
	keyboardCallback call;
	const char *title;
	const char *desc;
	tKeyboardModifier mod;
	keyboardCallbackData *next;
};

class joystickCallbackData {
public:
	joystickCallbackData(joystickCallback _jC, void *_userData)
	:jC(_jC)
	{ userData = _userData; }

	joystickCallback jC;
	void *userData;
};

class mouseCallbackData {
public:
	mouseCallbackData(mouseCallback _mC)
	:mC(_mC)
	{}
	mouseCallback mC;
};

void installJoystickHandler(joystickCallback jC, void *userdata);
void removeJoystickHandler(joystickCallback jC, void *userdata);
// this is called by the OS when it gets joystick movement
void handleJoystickMovement(pRecContext pContextInfo, double panX, double panY);

void installMouseClickHandler(mouseCallback jC);
void removeMouseClickHandler(mouseCallback jC);
// this is called by the OS when it gets a click
// returns true if the click was handled and the main app shouldn't handle it
bool handleMouseClick(pRecContext pContextInfo, int x, int y, point3d where, tButtonType, tMouseEventType);

void installKeyboardHandler(keyboardCallback kf, const char *title, const char *description,
														tKeyboardModifier mod, unsigned char firstKey, unsigned char lastKey = 0);
void printKeyboardAssignments();
void installCommandLineHandler(commandLineCallback, const char *, const char *, const char *);
void printCommandLineArguments();

//void initializeApplication(pRecContext pContextInfo);
void processCommandLineArgs(int argc, char *argv[]);

void createSimulation(unitSimulation * &unitLayer);
//void initializeKeyboardHandlers();
//void initializeCommandLineHandlers();
void initializeHandlers();
void processStats(statCollection *);
void frameCallback(unitSimulation *);

void submitTextToBuffer(const char *val);
void appendTextToBuffer(char *);
pRecContext getCurrentContext();
void updateModelView(pRecContext pContextInfo);
void cameraLookAt(GLfloat, GLfloat, GLfloat, float cameraSpeed = 0.1);
void cameraMoveTo(GLfloat x, GLfloat y, GLfloat z, float cameraSpeed = 0.1);
void resetCamera();
point3d GetOGLPos(int x, int y);

#endif
