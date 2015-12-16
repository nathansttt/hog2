//
//  SVGUtil.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/29/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#include "SVGUtil.h"

std::string SVGGetRGB(recColor c)
{
	std::string s;
	s = "rgb(";
	s += std::to_string(int(c.r*255)) + "," + std::to_string(int(c.g*255)) + "," + std::to_string(int(c.b*255));
	s += ")";
	return s;
}

std::string SVGDrawRect(int x, int y, int width, int height, recColor c)
{
	double epsilon = 0.5;
	std::string s;
	s += "<rect x=\"" + std::to_string(10*x-epsilon);
	s += "\" y=\"" + std::to_string(10*y-epsilon);
	s += "\" width=\""+std::to_string(width*10+2*epsilon)+"\" height=\""+std::to_string(height*10+2*epsilon)+"\" style=\"fill:rgb(";//128,128,0
	s += std::to_string(int(c.r*255)) + "," + std::to_string(int(c.g*255)) + "," + std::to_string(int(c.b*255));
	s += ");stroke-width:1\" />";
	return s;
}

std::string SVGDrawLine(int x1, int y1, int x2, int y2, int width, recColor c, bool center)
{
	std::string s;
	int offset = center?5:0;
	s = "<line x1 = \"" + std::to_string(10*x1+offset) + "\" ";
	s +=      "y1 = \"" + std::to_string(10*y1+offset) + "\" ";
	s +=      "x2 = \"" + std::to_string(10*x2+offset) + "\" ";
	s +=      "y2 = \"" + std::to_string(10*y2+offset) + "\" ";
	s += "style=\"stroke:"+SVGGetRGB(c);
	s += ";stroke-width:"+std::to_string(width)+"\" stroke-linecap=\"round\" />";
	return s;
}

std::string SVGDrawText(int x1, int y1, const char *txt, recColor c, int size)
{
	std::string s;
	recColor notC = {1-c.r, 1-c.g, 1-c.b};
	s =  "<text x=\""+std::to_string(x1*10+2)+"\" y=\""+std::to_string(y1*10)+"\" style=\"fill:"+SVGGetRGB(c);
	s += "; font-family:Impact, sans-serif; font-size:"+std::to_string(size*10)+"px; stroke:"+SVGGetRGB(notC)+"; stroke-width:"+std::to_string(size)+"px; stroke-linecap:round;stroke-linejoin:round\">";
	s += txt;
	s += "</text>";
	s += "<text x=\""+std::to_string(x1*10+2)+"\" y=\""+std::to_string(y1*10)+"\" style=\"fill:"+SVGGetRGB(c);
	s += "; font-family:Impact, sans-serif; font-size:"+std::to_string(size*10)+"px; stroke:"+SVGGetRGB(notC)+"; stroke-width:0px\">";
	s += txt;
	s += "</text>";
	return s;
}