//
//  Graphics.cpp
//  hog2
//
//  Created by Nathan Sturtevant on 7/10/16.
//  Copyright © 2016 NS Software. All rights reserved.
//

#include "Graphics.h"

namespace Graphics {
	bool PointInRect(const point3d &p, const rect &r)
	{
		return p.y >= r.top && p.x >= r.left && p.y <= r.bottom && p.x <= r.right;
	}

	bool PointInRect(const point &p, const rect &r)
	{
		return p.y >= r.top && p.x >= r.left && p.y <= r.bottom && p.x <= r.right;
	}

	Display::Display()
	{
		viewport = 0;
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
	}

	void Display::SetViewport(uint8_t v)
	{
		viewport = v;
	}

	void Display::FrameRect(rect r, rgbColor c, float lineWidth)
	{
		drawInfo i = {r, c, lineWidth};
		if (drawingBackground)
			backgroundDrawCommands.push_back({i, kFrameRectangle, viewport});
		else
			drawCommands.push_back({i, kFrameRectangle, viewport});
	}

	void Display::FrameSquare(point p, float radius, rgbColor c, float lineWidth)
	{
		drawInfo i = {{p.x-radius, p.y-radius, p.x+radius, p.y+radius}, c, lineWidth};
		if (drawingBackground)
			backgroundDrawCommands.push_back({i, kFrameRectangle, viewport});
		else
			drawCommands.push_back({i, kFrameRectangle, viewport});

	}

	void Display::FillSquare(point p, float radius, rgbColor c)
	{
		drawInfo i = {{p.x-radius, p.y-radius, p.x+radius, p.y+radius}, c, 0};
		if (drawingBackground)
			backgroundDrawCommands.push_back({i, kFillRectangle, viewport});
		else
			drawCommands.push_back({i, kFillRectangle, viewport});
	}
	
	void Display::FillRect(rect r, rgbColor c)
	{
		drawInfo i = {r, c, 0};
		if (drawingBackground)
			backgroundDrawCommands.push_back({i, kFillRectangle, viewport});
		else
			drawCommands.push_back({i, kFillRectangle, viewport});
	}
	
	void Display::FrameCircle(rect r, rgbColor c, float lineWidth)
	{
		drawInfo i = {r, c, lineWidth};
		if (drawingBackground)
			backgroundDrawCommands.push_back({i, kFrameOval, viewport});
		else
			drawCommands.push_back({i, kFrameOval, viewport});
	}
	
	void Display::FrameCircle(point p, float radius, rgbColor c, float lineWidth)
	{
		drawInfo i = {{p.x-radius, p.y-radius, p.x+radius, p.y+radius}, c, lineWidth};
		if (drawingBackground)
			backgroundDrawCommands.push_back({i, kFrameOval, viewport});
		else
			drawCommands.push_back({i, kFrameOval, viewport});
	}

	void Display::FillCircle(rect r, rgbColor c)
	{
		drawInfo i = {r, c, 0};
		if (drawingBackground)
			backgroundDrawCommands.push_back({i, kFillOval, viewport});
		else
			drawCommands.push_back({i, kFillOval, viewport});
	}

	void Display::FillCircle(point p, float radius, rgbColor c)
	{
		drawInfo i = {{p.x-radius, p.y-radius, p.x+radius, p.y+radius}, c, 0};
		if (drawingBackground)
			backgroundDrawCommands.push_back({i, kFillOval, viewport});
		else
			drawCommands.push_back({i, kFillOval, viewport});
	}

	void Display::FillNGon(point p, float radius, int sides, float rotation, rgbColor c)
	{
		shapeInfo i = {p, c, radius, sides, rotation};
		if (drawingBackground)
			backgroundDrawCommands.push_back({i, kFillNGon, viewport});
		else
			drawCommands.push_back({i, kFillNGon, viewport});
	}
	
	void Display::FrameNGon(point p, float radius, int sides, float rotation, rgbColor c)
	{
		shapeInfo i = {p, c, radius, sides, rotation};
		if (drawingBackground)
			backgroundDrawCommands.push_back({i, kFrameNGon, viewport});
		else
			drawCommands.push_back({i, kFrameNGon, viewport});
	}

	
	void Display::DrawLine(point start, point end, float lineWidth, rgbColor c)
	{
		lineInfo i = {start, end, c, lineWidth, false};
		if (drawingBackground)
			backgroundDrawCommands.push_back({i, viewport});
		else
			drawCommands.push_back({i, viewport});
	}

	void Display::DrawArrow(point start, point end, float lineWidth, rgbColor c)
	{
		lineInfo i = {start, end, c, lineWidth, true};
		if (drawingBackground)
			backgroundDrawCommands.push_back({i, viewport});
		else
			drawCommands.push_back({i, viewport});
	}

	void Display::DrawText(const char *textString, point location, rgbColor c, float height, textAlign align, const char *typeface)
	{
		textInfo i = {std::string(textString), location, c, height, std::string((typeface==0)?"Helvetica":typeface), align, viewport};
		if (drawingBackground)
			backgroundText.push_back(i);
		else
			text.push_back(i);
	}

	void Display::DrawText(const char *textString, point location, rgbColor c, float height, const char *typeface)
	{
		textInfo i = {std::string(textString), location, c, height, std::string((typeface==0)?"Helvetica":typeface), textAlignLeft, viewport};
		if (drawingBackground)
			backgroundText.push_back(i);
		else
			text.push_back(i);
	}

	void Display::DrawLineSegments(const std::vector<point> &points, float lineWidth, rgbColor c)
	{
		segments s = {c, lineWidth, points, viewport};
		if (drawingBackground)
			backgroundLineSegments.push_back(s);
		else
			lineSegments.push_back(s);
	}

}
