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

std::string SVGFrameRect(int x, int y, int width, int height, int border, recColor c);

std::string SVGDrawRect(int x, int y, int width, int height, recColor c);
std::string SVGDrawCircle(double x, double y, double radius, recColor c);
std::string SVGDrawLine(int x1, int y1, int x2, int y2, int width, recColor c, bool center = true);
std::string SVGDrawText(int x1, int y1, const char *txt, recColor c, int size);

#endif /* SVGUtil_hpp */
