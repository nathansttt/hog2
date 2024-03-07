//
//  Graphics.cpp
//  hog2
//
//  Created by Nathan Sturtevant on 7/10/16.
//  Copyright © 2016 NS Software. All rights reserved.
//

#include "Graphics.h"

namespace Graphics {
//bool PointInRect(const point3d &p, const rect &r)
//{
//	return p.y >= r.top && p.x >= r.left && p.y <= r.bottom && p.x <= r.right;
//}

bool PointInRect(const point &p, const rect &r)
{
	return p.y >= r.top && p.x >= r.left && p.y <= r.bottom && p.x <= r.right;
}

Display::Display()
{
	currViewport = 0;
	numViewports = 1;
	backgroundFrame = foregroundFrame = 0;
	drawingBackground = false;	}

void Display::StartFrame()
{
	drawCommands.clear();
	text.clear();
	lineSegments.clear();
	foregroundFrame++;
}

void Display::EndFrame()
{
}

void Display::StartBackground()
{
	if (backgroundFrame != foregroundFrame)
	{
		backgroundDrawCommands.clear();
		backgroundText.clear();
		backgroundLineSegments.clear();
	}
	backgroundFrame = foregroundFrame;
	drawingBackground = true;
}

void Display::EndBackground()
{
	drawingBackground = false;
}

bool Display::BackgroundNeedsRedraw() const
{
	return backgroundFrame == foregroundFrame;
}

void Display::SetNumViewports(uint8_t v)
{
	numViewports = v;
	while (viewports.size() < numViewports)
		AddViewport({-1, -1, 1, 1}, kScaleToSquare);
}

void Display::SetViewport(uint8_t v)
{
	currViewport = v;
}

void Display::FrameRect(rect r, rgbColor c, float lineWidth)
{
	drawInfo i = {r, c, lineWidth};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFrameRectangle, currViewport});
	else
		drawCommands.push_back({i, kFrameRectangle, currViewport});
}

void Display::FrameSquare(point p, float radius, rgbColor c, float lineWidth)
{
	drawInfo i = {{p.x-radius, p.y-radius, p.x+radius, p.y+radius}, c, lineWidth};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFrameRectangle, currViewport});
	else
		drawCommands.push_back({i, kFrameRectangle, currViewport});
	
}

void Display::FillSquare(point p, float radius, rgbColor c)
{
	drawInfo i = {{p.x-radius, p.y-radius, p.x+radius, p.y+radius}, c, 0};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFillRectangle, currViewport});
	else
		drawCommands.push_back({i, kFillRectangle, currViewport});
}

void Display::FillRect(rect r, rgbColor c)
{
	drawInfo i = {r, c, 0};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFillRectangle, currViewport});
	else
		drawCommands.push_back({i, kFillRectangle, currViewport});
}

void Display::FrameCircle(rect r, rgbColor c, float lineWidth)
{
	drawInfo i = {r, c, lineWidth};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFrameOval, currViewport});
	else
		drawCommands.push_back({i, kFrameOval, currViewport});
}

void Display::FrameCircle(point p, float radius, rgbColor c, float lineWidth)
{
	drawInfo i = {{p.x-radius, p.y-radius, p.x+radius, p.y+radius}, c, lineWidth};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFrameOval, currViewport});
	else
		drawCommands.push_back({i, kFrameOval, currViewport});
}

void Display::FillCircle(rect r, rgbColor c)
{
	drawInfo i = {r, c, 0};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFillOval, currViewport});
	else
		drawCommands.push_back({i, kFillOval, currViewport});
}

void Display::FillCircle(point p, float radius, rgbColor c)
{
	drawInfo i = {{p.x-radius, p.y-radius, p.x+radius, p.y+radius}, c, 0};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFillOval, currViewport});
	else
		drawCommands.push_back({i, kFillOval, currViewport});
}

void Display::FillTriangle(point p1, point p2, point p3, rgbColor c)
{
	triangleInfo i = {p1, p2, p3, c, 0};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFillTriangle, currViewport});
	else
		drawCommands.push_back({i, kFillTriangle, currViewport});
}

void Display::FrameTriangle(point p1, point p2, point p3, float lineWidth, rgbColor c)
{
	triangleInfo i = {p1, p2, p3, c, lineWidth};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFrameTriangle, currViewport});
	else
		drawCommands.push_back({i, kFrameTriangle, currViewport});
}

void Display::FillNGon(point p, float radius, int sides, float rotation, rgbColor c)
{
	shapeInfo i = {p, c, radius, sides, rotation, 0};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFillNGon, currViewport});
	else
		drawCommands.push_back({i, kFillNGon, currViewport});
}

void Display::FrameNGon(point p, float radius, float width, int sides, float rotation, rgbColor c)
{
	shapeInfo i = {p, c, radius, sides, rotation, width};
	
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFrameNGon, currViewport});
	else
		drawCommands.push_back({i, kFrameNGon, currViewport});
}


void Display::DrawLine(point start, point end, float lineWidth, rgbColor c)
{
	lineInfo i = {start, end, c, lineWidth, false};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, currViewport});
	else
		drawCommands.push_back({i, currViewport});
}

void Display::DrawArrow(point start, point end, float lineWidth, rgbColor c)
{
	lineInfo i = {start, end, c, lineWidth, true};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, currViewport});
	else
		drawCommands.push_back({i, currViewport});
}

void Display::DrawText(const char *textString, point location, rgbColor c, float height,
					   textAlign align, textBaseline base, const char *typeface)
{
	textInfo i = {std::string(textString), location, c, height, std::string((typeface==0)?"Helvetica":typeface), align, base, currViewport};
	if (drawingBackground)
		backgroundText.push_back(i);
	else
		text.push_back(i);
}

void Display::DrawText(const char *textString, point location, rgbColor c, float height, textAlign align, const char *typeface)
{
	textInfo i = {std::string(textString), location, c, height, std::string((typeface==0)?"Helvetica":typeface), align, textBaselineBottom, currViewport};
	if (drawingBackground)
		backgroundText.push_back(i);
	else
		text.push_back(i);
}

void Display::DrawText(const char *textString, point location, rgbColor c, float height, const char *typeface)
{
	textInfo i = {std::string(textString), location, c, height, std::string((typeface==0)?"Helvetica":typeface), textAlignLeft, textBaselineBottom, currViewport};
	if (drawingBackground)
		backgroundText.push_back(i);
	else
		text.push_back(i);
}

void Display::DrawLineSegments(const std::vector<point> &points, float lineWidth, rgbColor c)
{
	segments s = {c, lineWidth, points, false, currViewport};
	if (drawingBackground)
		backgroundLineSegments.push_back(s);
	else
		lineSegments.push_back(s);
}

void Display::FillLineSegments(const std::vector<point> &points, float lineWidth, rgbColor c)
{
	segments s = {c, lineWidth, points, true, currViewport};
	if (drawingBackground)
		backgroundLineSegments.push_back(s);
	else
		lineSegments.push_back(s);
}

}

void Graphics::Display::ReinitViewports(const Graphics::rect &r, viewportType v)
{
	viewports.resize(1);
	viewports[0].bounds = r;
	viewports[0].finalBound = r;
	viewports[0].type = v;
	viewports[0].active = true;
	numViewports = 1;
}

/* Adds a new viewport to the existing viewports and
 * returns the new viewport numbers
 */
int Graphics::Display::AddViewport(const Graphics::rect &r, viewportType v)
{
//	pRecContext pContextInfo = GetContext(windowID);
//	if (pContextInfo->numPorts >= MAXPORTS)
//	{
//		printf("Cannot add viewport - reached limit of %d [constant MAXPORTS]\n", MAXPORTS);
//		return -1;
//	}
	viewports.resize(viewports.size()+1);
	//pContextInfo->numPorts++;
	numViewports = viewports.size();
	viewports[numViewports-1].bounds = r;
	viewports[numViewports-1].finalBound = r;
	viewports[numViewports-1].type = v;
	viewports[numViewports-1].active = true;
	return numViewports-1;
}

/* Adds a new viewport to the existing viewports and
 * returns the new viewport numbers. Will animate from initial to final location
 */
int Graphics::Display::AddViewport(const Graphics::rect &initial, const Graphics::rect &fin, viewportType v)
{
//	pRecContext pContextInfo = GetContext(windowID);
//	if (pContextInfo->numPorts >= MAXPORTS)
//	{
//		printf("Cannot add viewport - reached limit of %d [constant MAXPORTS]\n", MAXPORTS);
//		return -1;
//	}
	numViewports++;
	viewports[numViewports-1].bounds = initial;
	viewports[numViewports-1].finalBound = fin;
	viewports[numViewports-1].type = v;
	viewports[numViewports-1].active = true;
	return numViewports-1;
}

void Graphics::Display::MoveViewport(int port, const Graphics::rect &newLocation)
{
	viewports[port].finalBound = newLocation;
}

float Graphics::Display::GlobalHOGToViewportX(float x, int v) const
{
	Graphics::point input(x, 0.f, 0.f);
	Graphics::point input2(0, 0.f, 0.f);
	Graphics::point result = ViewportToGlobalHOG(input, v);
	Graphics::point result2 = ViewportToGlobalHOG(input2, v);
	return result.x-result2.x;
	//	return ((result.x+1.0)*pContextInfo->windowWidth)/2.0;
}

float Graphics::Display::ViewportToGlobalHOGX(float x, int v) const
{
	Graphics::point input(x, 0.f, 0.f);
	Graphics::point input2(0, 0.f, 0.f);
	Graphics::point result = ViewportToGlobalHOG(input, v);
	Graphics::point result2 = ViewportToGlobalHOG(input2, v);
	return result.x-result2.x;
	//	return ((result.x+1.0)*pContextInfo->windowWidth)/2.0;
}

// This code doesn't look to be correct
//float Graphics::Display::GlobalHOGToViewportY(float y, int v)
//{
//	Graphics::point input(0.f, y, 0.f);
//	Graphics::point result = ViewportToGlobalHOG(input, v);
////	result.y -= pContextInfo->viewports[v].bounds.bottom;
////	result.y = pContextInfo->viewports[v].bounds.top - result.y;
//	//	if (v == 1)
////		printf("Y:%f -> %f\n", y, ((result.y+1.0))/2.0);
//	return -((result.y-1.0)*pContextInfo->windowHeight)/2.0;
//}

Graphics::point Graphics::Display::GlobalHOGToViewport(Graphics::point where, int viewport) const
{
	return GlobalHOGToViewport(viewports[viewport], where);
}

Graphics::point Graphics::Display::ViewportToGlobalHOG(Graphics::point where, int viewport) const
{
	return ViewportToGlobalHOG(viewports[viewport], where);
}

Graphics::rect Graphics::Display::GlobalHOGToViewport(const Graphics::rect &loc, int port) const
{
	return Graphics::rect(GlobalHOGToViewport(viewports[port], {loc.left, loc.top}),
						  GlobalHOGToViewport(viewports[port], {loc.right, loc.bottom}));
}

Graphics::rect Graphics::Display::ViewportToGlobalHOG(const Graphics::rect &loc, int port) const
{
	return Graphics::rect(ViewportToGlobalHOG(viewports[port], {loc.left, loc.top}),
						  ViewportToGlobalHOG(viewports[port], {loc.right, loc.bottom}));
}

Graphics::point Graphics::Display::GlobalHOGToViewport(const viewport &v, Graphics::point where) const
{
	if (v.type == kScaleToFill) 		// just scale regular -1/1 axes into the rectangle
	{
		// gets offset into rect
		where.x -= v.bounds.left;
		where.x /= (v.bounds.right-v.bounds.left);
		where.x = where.x*2.0-1.0;
		where.y -= v.bounds.bottom;
		where.y /= (v.bounds.top-v.bounds.bottom);
		where.y = -where.y*2.0+1.0;
		return where;
	}
	else if (v.type == kScaleToSquare)
	{
		double localWidth = v.bounds.right-v.bounds.left;
		double localHeight = v.bounds.bottom-v.bounds.top;
		double actualWidth = windowWidth*localWidth;
		double actualHeight = windowHeight*localHeight;
		double xRatio = actualWidth/actualHeight;
		double yRatio = actualHeight/actualWidth;
		xRatio = std::max(xRatio, 1.);
		yRatio = std::max(yRatio, 1.);
		
		where.x *= xRatio;
		where.x -= v.bounds.left;
		where.x /= (v.bounds.right-v.bounds.left);
		where.x = where.x*2.0-1.0;

		
		where.y *= yRatio;
		where.y -= v.bounds.bottom;
		where.y /= (v.bounds.top-v.bounds.bottom);
		where.y = (-where.y)*2.0+1.0;

		
		return where;

	}
	else {
		printf("Unknown scale type\n");
		exit(0);
	}
}

Graphics::point Graphics::Display::ViewportToGlobalHOG(const viewport &v, Graphics::point where) const
{
	if (v.type == kScaleToFill)
	{
		where.x = (where.x+1.0)/2.0;
		where.x *= (v.bounds.right-v.bounds.left);
		where.x += v.bounds.left;

		where.y = -(where.y-1.0)/2.0;
		where.y *= (v.bounds.top-v.bounds.bottom);
		where.y += v.bounds.bottom;
		return where;
	}
	else if (v.type == kScaleToSquare)
	{
		double localWidth = v.bounds.right-v.bounds.left;
		double localHeight = v.bounds.bottom-v.bounds.top;
		double actualWidth = windowWidth*localWidth;
		double actualHeight = windowHeight*localHeight;
//		double xRatio = pContextInfo->windowWidth*localWidth/pContextInfo->windowHeight*localHeight;
//		double yRatio = pContextInfo->windowHeight*localHeight/pContextInfo->windowWidth*localWidth;
		double xRatio = actualWidth/actualHeight;
		double yRatio = actualHeight/actualWidth;
		xRatio = std::max(xRatio, 1.);
		yRatio = std::max(yRatio, 1.);
		
		where.x = (where.x+1.0)/2.0;
		where.x *= (v.bounds.right-v.bounds.left);
		where.x += v.bounds.left;
		where.x /= xRatio;
		
		where.y = -(where.y-1.0)/2.0;
		where.y *= (v.bounds.top-v.bounds.bottom);
		where.y += v.bounds.bottom;
		where.y /= yRatio;
		return where;

	}
	else {
		printf("Unknown scale type\n");
		exit(0);
	}
}
