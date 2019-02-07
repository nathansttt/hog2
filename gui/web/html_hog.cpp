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
	p.z = 0;
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
		case Graphics::Display::kFillNGon:
		{
			const Graphics::Display::shapeInfo &o = d.polygon;
			int r = o.c.r*255.0;
			int g = o.c.g*255.0;
			int b = o.c.b*255.0;
			float cx = PointXToCanvas(o.center.x, d.viewport);
			float cy = PointYToCanvas(o.center.y, d.viewport);
			float radius = fabs(PointXToCanvas(o.radius, d.viewport)-PointXToCanvas(0, d.viewport));
			EM_ASM_({
				var c=document.getElementById(UTF8ToString($0));
				var ctx=c.getContext("2d");
				var resolution = 6.283185307/$8;

				ctx.fillStyle = "rgb("+$1+", "+$2+", "+$3+")";
				ctx.beginPath();
				for (var x = 0; x <= $8; x++)
				{
					var nextx = $4+Math.sin(resolution*x+$7*0.01745329252)*$6;
					var nexty = $5+Math.cos(resolution*x+$7*0.01745329252)*$6;

					if (x == 0)
						ctx.moveTo(nextx, nexty);
					else
						ctx.lineTo(nextx, nexty);
				}
				ctx.fill();
			}, which, r,g,b, cx, cy, radius, o.rotate, o.segments);

		}
	}
}

void DrawToCanvas(const Graphics::Display &disp)
{
	if (disp.BackgroundNeedsRedraw())
	{
		//printf("Drawing background\n");
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
		
		for (int x = 0; x < disp.backgroundText.size(); x++)
		{
			const auto &i = disp.backgroundText[x];
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

	if (up)
		t = kMouseUp;
	if (drag && down)
		t = kMouseDrag;
	if (!up && !down && drag)
		t = kMouseMove;
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
