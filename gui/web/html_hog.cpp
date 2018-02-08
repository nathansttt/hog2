#include <math.h>
#include <emscripten.h>
#include <stdio.h>
#include "Common.h"
#include "SVGUtil.h"

int fromx=10, fromy=10, tox=20, toy=20;

const char *HOGDoFrame();
void HOGDoMouse(int x, int y, bool up, bool down, bool drag);
void HOGInit();
void HOGHitKey(char key);
int hog_main(int argc, char **argv);
   
extern "C" {

	
	void EMSCRIPTEN_KEEPALIVE MouseEvent(int x, int y, bool up, bool down, bool drag)
	{
		HOGDoMouse(x, y, up, down, drag);
	}

	
	const char * EMSCRIPTEN_KEEPALIVE DoFrame()
	{
		return HOGDoFrame();
	}

	
	void EMSCRIPTEN_KEEPALIVE InitHOG()
	{
		HOGInit();
	}

	
	void EMSCRIPTEN_KEEPALIVE HitKey(char key)
	{
		HOGHitKey(key);
	}
}

const char *HOGDoFrame()
{
	pRecContext pContextInfo = getCurrentContext();
	for (int x = 0; x < pContextInfo->numPorts; x++)
	{
		setViewport(pContextInfo, x);
		HandleFrame(pContextInfo, x);
	}
	return MakeSVG(pContextInfo->display, 800, 800).c_str();
}

void HOGDoMouse(int x, int y, bool up, bool down, bool drag)
{
	point3d p = {static_cast<GLfloat>(2*(x)/800.0-1), static_cast<GLfloat>(2*(y/800.0)-1), 0};
	tMouseEventType t = kMouseDown;

	if  (up)
		t = kMouseUp;
	if (drag)
		t = kMouseDrag;

	HandleMouseClick(getCurrentContext(), x, y, p, kLeftButton, t);
}

void HOGInit()
{		
	hog_main(0, 0);
	
	pRecContext pContextInfo = getCurrentContext();
	initialConditions(pContextInfo);
	HandleWindowEvent(pContextInfo, kWindowCreated);
}

void HOGHitKey(char key)
{
	DoKeyboardCommand(getCurrentContext(), key, false, false, false);
}
