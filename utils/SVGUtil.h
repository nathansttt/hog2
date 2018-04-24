//
//  SVGUtil.hpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/29/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#ifndef SVGUtil_h
#define SVGUtil_h

#include <stdio.h>
#include <string>
#include "GLUtil.h"
#include "Graphics.h"
#include <sstream>
#include <iomanip>

namespace SVG {
	enum svgAlignment {
		kLeft,
		kRight,
		kCenter
	};
}

std::string SVGFrameRect(int x, int y, int width, int height, int border, rgbColor c);

std::string SVGDrawRect(float x, float y, float width, float height, rgbColor c);
std::string SVGDrawRect(float x, float y, float width, float height, const char *gradient);
std::string SVGFrameCircle(double x, double y, double radius, int border, rgbColor c);
std::string SVGDrawCircle(double x, double y, double radius, rgbColor c);
std::string SVGDrawLine(int x1, int y1, int x2, int y2, int width, rgbColor c, bool center = true);
std::string SVGDrawLine(float x1, float y1, float x2, float y2, float width, rgbColor c);
std::string SVGDrawLineSegments(const std::vector<Graphics::point> &lines, float width, rgbColor c);
std::string SVGDrawText(float x1, float y1, const char *txt, rgbColor c, double size, const char *typeface = 0,
						SVG::svgAlignment align = SVG::kCenter);
std::string SVGDrawStrokedText(float x1, float y1, const char *txt, rgbColor c, rgbColor strokeColor, double size);

std::string SVGDefineGradient(bool horizontal, bool vertical, rgbColor c1, rgbColor c2, const char *name);

std::string MakeSVG(const Graphics::Display &disp, int width, int height);
void MakeSVG(const Graphics::Display &disp, const char *filename, int width, int height);

// Code from: https://stackoverflow.com/questions/16605967/set-precision-of-stdto-string-when-converting-floating-point-values/16606128#16606128
template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
	std::ostringstream out;
	out << std::setprecision(n) << a_value;
	return out.str();
}


#endif /* SVGUtil_hpp */
