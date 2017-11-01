//
//  Graphics.cpp
//  hog2
//
//  Created by Nathan Sturtevant on 7/10/16.
//  Copyright © 2016 NS Software. All rights reserved.
//

#include "Graphics.h"

namespace Graphics {
	void Display::StartFrame()
	{
//		framedRects.clear();
//		filledRects.clear();
//		framedCircles.clear();
//		filledCircles.clear();
//		lines.clear();
//		text.clear();
		drawCommands.clear();
		text.clear();
		lineSegments.clear();
	}
	
	void Display::EndFrame()
	{
	}

	
	void Display::FrameRect(rect r, rgbColor c, float lineWidth)
	{
		drawInfo i = {r, c, lineWidth};
		drawCommands.push_back({i, kFrameRectangle});
	}

	void Display::FillRect(rect r, rgbColor c)
	{
		drawInfo i = {r, c, 0};
		drawCommands.push_back({i, kFillRectangle});
	}
	
	void Display::FrameCircle(rect r, rgbColor c, float lineWidth)
	{
		drawInfo i = {r, c, lineWidth};
		drawCommands.push_back({i, kFrameOval});
	}
	
	void Display::FillCircle(rect r, rgbColor c)
	{
		drawInfo i = {r, c, 0};
		drawCommands.push_back({i, kFillOval});
	}
	
	void Display::DrawLine(point start, point end, float lineWidth, rgbColor c)
	{
		lineInfo i = {start, end, c, lineWidth, false};
		drawCommands.push_back(i);
		//drawCommands.push_back({start, end, c, lineWidth, false});
	}

	void Display::DrawArrow(point start, point end, float lineWidth, rgbColor c)
	{
		lineInfo i = {start, end, c, lineWidth, true};
		drawCommands.push_back(i);
		//drawCommands.push_back({start, end, c, lineWidth, true});
	}

	void Display::DrawText(const char *textString, point location, rgbColor c, float height, const char *typeface)
	{
		textInfo i = {std::string(textString), location, c, height, std::string((typeface==0)?"Helvetica":typeface)};
		text.push_back(i);
	}

	void Display::DrawLineSegments(const std::vector<point> &points, float lineWidth, rgbColor c)
	{
		segments s = {c, lineWidth, points};
		lineSegments.push_back(s);
	}

}
