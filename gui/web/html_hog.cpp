#include <math.h>
#include <emscripten.h>
#include <stdio.h>
#include "Common.h"
#include "SVGUtil.h"


int canvasWidth = 640;
int canvasHeight = 640;
float canvasScale = 0;
int canvasXOffset = 0;
int canvasYOffset = 0;

const char *HOGDoFrame();
void HOGDoMouse(int x, int y, bool up, bool down, bool drag);
void HOGInit();
void HOGHitKey(const char *key);
int hog_main(int argc, char **argv);
const char *getTextBuffer();
void DrawToCanvas(const Graphics::Display &disp);

extern "C" {

	void EMSCRIPTEN_KEEPALIVE SetCanvasSize(int x, int y)
	{
		canvasWidth = x;
		canvasHeight = y;

		pRecContext pContextInfo = getCurrentContext();
		pContextInfo->windowHeight = canvasHeight;
		pContextInfo->windowWidth = canvasWidth;

		canvasScale = std::min(canvasWidth, canvasHeight);
		canvasXOffset = (canvasWidth>canvasHeight)?(canvasWidth-canvasHeight):0;
		canvasYOffset = (canvasHeight>canvasWidth)?(canvasHeight-canvasWidth):0;
		canvasXOffset /= 2;
		canvasYOffset /= 2;
	}

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
		SetCanvasSize(canvasWidth, canvasHeight);
	}
	
	void EMSCRIPTEN_KEEPALIVE HitKey(const char *key)
	{
		HOGHitKey(key);
	}

}

const char *HOGDoFrame()
{
	pRecContext pContextInfo = getCurrentContext();
	pContextInfo->display.StartFrame();
	for (int x = 0; x < pContextInfo->numPorts; x++)
	{
		setViewport(pContextInfo, x);
		HandleFrame(pContextInfo, x);
	}
//	if (getTextBuffer() != 0)
//		pContextInfo->display.DrawText(getTextBuffer(), {0,-0.95}, Colors::gray, 0.04);
	static std::string result;
	pContextInfo->display.EndFrame();
	DrawToCanvas(pContextInfo->display);
	return "";
}

///
//-(CGFloat)hogToScreenX:(CGFloat)x viewport:(int)v
//{
//	switch (_display->GetNumViewports())
//	{
//		case 1:
//			return x*xscale+xoffset;
//		case 2:
//			if (v == 0)
//				return (0.75*x*xscale+xoffset);
//			if (v == 1)
//				return (0.75*x*xscale+xoffset);
//	}
//	return 0;
//}
//
//-(CGFloat)hogToScreenY:(CGFloat)y viewport:(int)v
//{
//	switch (_display->GetNumViewports())
//	{
//		case 1:
//			return height-(y*yscale+yoffset);
//		case 2:
//			if (v == 0)
//				return (height-((y*0.75-0.25)*yscale+yoffset));
//			if (v == 1)
//				return (height-((y*0.25+0.75)*yscale+yoffset));
//			assert(!"Shouldn't get here");
//	}
//	return 0;
//}
//
//-(point3d)convertToHogCoordinate:(NSPoint)currPoint
//{
//	point3d p;
//	switch (_display->GetNumViewports())
//	{
//		case 1:
//			p.x = (currPoint.x-xoffset)/xscale;
//			p.y = (height-currPoint.y-yoffset)/yscale;
//			return p;
//		case 2:
//			int v = [self getViewport:currPoint];
//			if (v == 0)
//				p.y = (((height-currPoint.y)-yoffset)/yscale+0.25)/0.75;
//				if (v == 1)
//					p.y = (((height-currPoint.y)-yoffset)/yscale+0.75)/0.25;
//					}
//	//	(currPoint.x - xscale)/( = (0.75*p.x*xscale+xoffset);
//	p.x = (currPoint.x-xoffset)/(xscale*0.75);
//
//	return p;
//	//point3d p = [drawingView convertToHogCoordinate:curPoint];
//}
///

point3d CanvasToPoint(int x, int y)
{
	x -= canvasXOffset;
	y -= canvasYOffset;
	return {static_cast<GLfloat>(2.0*x/canvasScale-1.0), static_cast<GLfloat>(2.0*y/canvasScale-1.0), 0};
}

float PointXToCanvas(float p, int viewport)
{
	pRecContext pContextInfo = getCurrentContext();
	point3d input(p, 0.f, 0.f);
	point3d result = ViewportToGlobalHOG(pContextInfo, pContextInfo->viewports[viewport], input);
	//	if (v == 1)
	//printf("X:%f -> %f\n", x, ((result.x+1.0))/2.0);
	return ((result.x+1.0)*canvasWidth)/2.0;

	// old code
//	return ((p+1)*canvasScale/2.0)+canvasXOffset;
}

float PointYToCanvas(float p, int viewport)
{
	pRecContext pContextInfo = getCurrentContext();
	point3d input(0.f, p, 0.f);
	point3d result = ViewportToGlobalHOG(pContextInfo, pContextInfo->viewports[viewport], input);
	return ((result.y+1.0)*canvasHeight)/2.0;

	// old code
//	return ((p+1)*canvasScale/2.0)+canvasYOffset;
}

point3d ConvertCanvasToGlobalHOG(int x, int y)
{
	point3d p;
	p.x = 2.0*x/canvasWidth-1.0;
	p.y = 2.0*y/canvasHeight-1.0;
	return p;
	
}

void PointToCanvas(Graphics::point &p, int viewport)
{
	p.x = PointXToCanvas(p.x, viewport);
	p.y = PointYToCanvas(p.y, viewport);
//	p.x = ((p.x+1)*canvasScale/2.0)+canvasXOffset;
//	p.y = ((p.y+1)*canvasScale/2.0)+canvasYOffset;
}

void DoDrawCommand(const Graphics::Display::data &d, const char *which)
{
	switch (d.what)
	{
		case Graphics::Display::kLine:
		{
			const Graphics::Display::lineInfo &o = d.line; //disp.lines[x];
			int r = o.c.r*255.0;
			int g = o.c.g*255.0;
			int b = o.c.b*255.0;
			
			EM_ASM_({
				var c=document.getElementById(UTF8ToString($0));
				var ctx=c.getContext("2d");
				ctx.strokeStyle = "rgb("+$1+", "+$2+", "+$3+")";
				ctx.beginPath();
				ctx.moveTo($4, $5);
				ctx.lineTo($6, $7);
				ctx.lineWidth=$8;
				ctx.stroke();
				ctx.lineWidth=1;
			}, which, r,g,b,
					PointXToCanvas(o.start.x, d.viewport), PointYToCanvas(o.start.y, d.viewport),
					PointXToCanvas(o.end.x, d.viewport), PointYToCanvas(o.end.y, d.viewport), o.width);
			break;
			
			
			//				s += SVGDrawLine((o.start.x+1)*width/2.0, (o.start.y+1)*height/2.0, (o.end.x+1)*width/2.0, (o.end.y+1)*height/2.0, o.width, o.c);
			//		if (o.arrow)
			//		{
			//			Graphics::point newEnd = o.end*0.975f+o.start*0.025f;
			//			Graphics::point p1 = o.end-o.start;
			//			Graphics::point p2 = o.start;
			//			p2.z = 1;
			//			p2 = p1*p2;
			//			p2.normalise();
			//			p2 *= (o.end-newEnd).length();
			//			CGContextMoveToPoint(context, ((o.end.x)*xscale+xoffset), height-(o.end.y*yscale+yoffset));
			//			CGContextAddLineToPoint(context, ((newEnd.x+p2.x)*xscale+xoffset), height-((newEnd.y+p2.y)*yscale+yoffset));
			//			CGContextMoveToPoint(context, ((o.end.x)*xscale+xoffset), height-(o.end.y*yscale+yoffset));
			//			CGContextAddLineToPoint(context, ((newEnd.x-p2.x)*xscale+xoffset), height-((newEnd.y-p2.y)*yscale+yoffset));
			//		}
			
			break;
		}
		case Graphics::Display::kFillRectangle:
		{
			const Graphics::Display::drawInfo &o = d.shape;
			int r = o.c.r*255.0;
			int g = o.c.g*255.0;
			int b = o.c.b*255.0;
			float cWidth = ceilf(PointXToCanvas(o.r.right, d.viewport))-floorf(PointXToCanvas(o.r.left, d.viewport));
			float cHeight = ceilf(PointYToCanvas(o.r.bottom, d.viewport))-floorf(PointYToCanvas(o.r.top, d.viewport));
			EM_ASM_({
				var c=document.getElementById(UTF8ToString($0));
				var ctx=c.getContext("2d");
				ctx.fillStyle = "rgb("+$1+", "+$2+", "+$3+")";
				ctx.fillRect($4,$5,$6,$7);
			}, which, r,g,b,
					floorf(PointXToCanvas(o.r.left, d.viewport)), floorf(PointYToCanvas(o.r.top, d.viewport)),
					cWidth, cHeight);
			break;
		}
		case Graphics::Display::kFrameRectangle:
		{
			const Graphics::Display::drawInfo &o = d.shape;
			int r = o.c.r*255.0;
			int g = o.c.g*255.0;
			int b = o.c.b*255.0;
			float cWidth = PointXToCanvas(o.r.right, d.viewport)-PointXToCanvas(o.r.left, d.viewport);
			float cHeight = PointYToCanvas(o.r.bottom, d.viewport)-PointYToCanvas(o.r.top, d.viewport);
			EM_ASM_({
				var c=document.getElementById(UTF8ToString($0));
				var ctx=c.getContext("2d");
				ctx.strokeStyle = "rgb("+$1+", "+$2+", "+$3+")";
				ctx.strokeRect($4,$5,$6,$7);
			}, which, r,g,b,
					PointXToCanvas(o.r.left, d.viewport), PointYToCanvas(o.r.top, d.viewport),
					cWidth, cHeight);
			break;
		}
		case Graphics::Display::kFillOval:
		{
			const Graphics::Display::drawInfo &o = d.shape;
			int r = o.c.r*255.0;
			int g = o.c.g*255.0;
			int b = o.c.b*255.0;
			float cx = (o.r.left+o.r.right)/2.0;
			float cy = (o.r.top+o.r.bottom)/2.0;
			float rad = (PointXToCanvas(o.r.right, d.viewport)-PointXToCanvas(o.r.left, d.viewport))/2.0;
			EM_ASM_({
				var c=document.getElementById(UTF8ToString($0));
				var ctx=c.getContext("2d");
				ctx.fillStyle = "rgb("+$1+", "+$2+", "+$3+")";
				ctx.beginPath();
				ctx.arc($4,$5,$6,0,2*Math.PI);
				ctx.fill();
			}, which, r,g,b,
					PointXToCanvas(cx, d.viewport),
					PointYToCanvas(cy, d.viewport),
					rad);
			break;
		}
		case Graphics::Display::kFrameOval:
		{
			const Graphics::Display::drawInfo &o = d.shape;
			int r = o.c.r*255.0;
			int g = o.c.g*255.0;
			int b = o.c.b*255.0;
			float cx = (o.r.left+o.r.right)/2.0;
			float cy = (o.r.top+o.r.bottom)/2.0;
			EM_ASM_({
				var c=document.getElementById(UTF8ToString($0));
				var ctx=c.getContext("2d");
				ctx.strokeStyle = "rgb("+$1+", "+$2+", "+$3+")";
				ctx.beginPath();
				ctx.arc($4,$5,$6,0,2*Math.PI);
				ctx.stroke();
			}, which, r,g,b,
					PointXToCanvas(cx, d.viewport),
					PointYToCanvas(cy, d.viewport),
					(PointXToCanvas(o.r.right, d.viewport)-PointXToCanvas(o.r.left, d.viewport))/2.0);
			break;
		}
	}
}

void DrawToCanvas(const Graphics::Display &disp)
{
	if (disp.BackgroundNeedsRedraw())
	{
		printf("Drawing background\n");
		EM_ASM({
			var c=document.getElementById("bg");
			var ctx=c.getContext("2d");
			ctx.clearRect(0, 0, c.width, c.height);
			ctx.fillStyle = "rgb(0,0,0)";
			ctx.fillRect(0, 0, c.width, c.height);
		});
		for (int x = 0; x < disp.backgroundDrawCommands.size(); x++)
		{
			//printf("Background command %d\n", x);
			DoDrawCommand(disp.backgroundDrawCommands[x], "bg");
		}
	}
	
	EM_ASM({
		var c=document.getElementById("fg");
		var ctx=c.getContext("2d");
		ctx.clearRect(0, 0, c.width, c.height);
	});
	for (int x = 0; x < disp.drawCommands.size(); x++)
	{
		DoDrawCommand(disp.drawCommands[x], "fg");
	}
	
	for (int x = 0; x < disp.text.size(); x++)
	{
		const auto &i = disp.text[x];
		int r = i.c.r*255.0;
		int g = i.c.g*255.0;
		int b = i.c.b*255.0;

		if (i.align == Graphics::textAlignCenter)
		{
			EM_ASM_({
				var c=document.getElementById("fg");
				var ctx=c.getContext("2d");
				ctx.font = "15px Courier";
				ctx.textAlign = "center";
				ctx.fillStyle = "rgb("+$1+", "+$2+", "+$3+")";
				ctx.fillText(UTF8ToString($0),$4,$5);
			}, i.s.c_str(), r,g,b,
					PointXToCanvas(i.loc.x, i.viewport),
					PointYToCanvas(i.loc.y, i.viewport));
		}
		else {
			EM_ASM_({
				var c=document.getElementById("fg");
				var ctx=c.getContext("2d");
				ctx.font = "15px Courier";
				ctx.textAlign = "left";
				ctx.fillStyle = "rgb("+$1+", "+$2+", "+$3+")";
				ctx.fillText(UTF8ToString($0),$4,$5);
			}, i.s.c_str(), r,g,b,
					PointXToCanvas(i.loc.x, i.viewport),
					PointYToCanvas(i.loc.y, i.viewport));
		}

	}
	static std::vector<Graphics::point> outputPoints;
	for (int x = 0; x < disp.lineSegments.size(); x++)
	{
		const auto &i = disp.lineSegments[x];
		int r = i.c.r*255.0;
		int g = i.c.g*255.0;
		int b = i.c.b*255.0;
		outputPoints = i.points;
		EM_ASM_({
			var c=document.getElementById("fg");
			var ctx=c.getContext("2d");
			ctx.strokeStyle = "rgb("+$0+", "+$1+", "+$2+")";
			ctx.beginPath();
		}, r,g,b);
		for (int x = 0; x < outputPoints.size(); x++)
		{
			auto &j = outputPoints[x];
			PointToCanvas(j, i.viewport);
			if (x == 0)
			{
				EM_ASM_({
					var c=document.getElementById("fg");
					var ctx=c.getContext("2d");
					ctx.moveTo($0, $1);
				}, j.x, j.y);
			}
			else {
				//var c=document.getElementById(UTF8ToString($0));
				EM_ASM_({
					var c=document.getElementById("fg");
					var ctx=c.getContext("2d");
					ctx.lineTo($0, $1);
				}, j.x, j.y);
			}
		}
		EM_ASM_({
			var c=document.getElementById("fg");
			var ctx=c.getContext("2d");
			ctx.lineWidth=$0;
			ctx.stroke();
			ctx.lineWidth=1;
		}, i.size);
	}
}

//int GetViewport(int x, int y)
//{
//	pRecContext pContextInfo = getCurrentContext();
//	if (pContextInfo->display.GetNumViewports() == 1)
//		return 0;
//	point3d p = {static_cast<GLfloat>(2.0*x/canvasWidth-1), static_cast<GLfloat>(2.0*y/canvasHeight-1), 0};
//	if (p.y < 2.0*(1.0-0.75))
//		return 0;
//	return 1;
//}


void HOGDoMouse(int x, int y, bool up, bool down, bool drag)
{
	pRecContext pContextInfo = getCurrentContext();
	pContextInfo->windowHeight = canvasHeight;
	pContextInfo->windowWidth = canvasWidth;

	tMouseEventType t = kMouseDown;

	if  (up)
		t = kMouseUp;
	if (drag)
		t = kMouseDrag;
//	int viewport = GetViewport(x, y);
	
	HandleMouse(pContextInfo, ConvertCanvasToGlobalHOG(x, y), kLeftButton, t);
//	HandleMouseClick(getCurrentContext(), viewport, x, y, p, kLeftButton, t);
}

void HOGInit()
{		
	hog_main(0, 0);
	
	pRecContext pContextInfo = getCurrentContext();
	initialConditions(pContextInfo);
	HandleWindowEvent(pContextInfo, kWindowCreated);
}

void HOGHitKey(const char *key)
{
	if (key != 0)
	{
		printf("Hit key: '%c'\n", key[0]);
		DoKeyboardCommand(getCurrentContext(), key[0], false, false, false);
	}
}

recContext context;
pRecContext pContextInfo = &context;

void updateProjection(pRecContext pContextInfo, int viewPort)
{	
}

pRecContext GetContext(unsigned long windowID)
{
	return pContextInfo;
}

pRecContext getCurrentContext()
{
	return pContextInfo;
}

void updateModelView(pRecContext pContextInfo, int currPort)
{
	
}

GLfloat gTrackBallRotation [4] = {0.0f, 0.0f, 0.0f, 0.0f};

pRecContext gTrackingContextInfo = NULL;

void renderScene()
{
	
}

void RunHOGGUI(int argc, char* argv[], int windowDimension)
{
	RunHOGGUI(argc, argv, windowDimension, windowDimension);
}

void RunHOGGUI(int argc, char* argv[], int xDimension, int yDimension)
{
	// We don't handle command-line arguments here
	ProcessCommandLineArgs(argc, argv);
	//initialConditions(pContextInfo);
	//HandleWindowEvent(pContextInfo, kWindowCreated);
}

char *textBuffer = 0;

void submitTextToBuffer(const char *val)
{
	if (val)
	{
		delete [] textBuffer;
		int len = strlen(val);
		textBuffer = new char[len+1];
		strncpy(textBuffer, val, len);
		textBuffer[len] = 0;
		//printf("Buffer: '%s'\n", val);
		
		EM_ASM_({
			document.getElementById("message").innerHTML = UTF8ToString($0);
		}, textBuffer);

	}
}

void appendTextToBuffer(const char *next)
{
	if (textBuffer == 0)
	{
		submitTextToBuffer(next);
		return;
	}
	int oldLen = strlen(textBuffer);
	int newLen = strlen(next)+oldLen;
	char *newStr = new char[newLen+1];
	strcpy(newStr, textBuffer);
	strcpy(&newStr[oldLen], next);
	delete [] textBuffer;
	textBuffer = newStr;

	EM_ASM_({
		document.getElementById("message").innerHTML = UTF8ToString($0);
	}, textBuffer);

}

const char *getTextBuffer()
{
	return textBuffer;
}
