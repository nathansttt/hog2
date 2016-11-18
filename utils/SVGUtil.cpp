//
//  SVGUtil.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/29/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#include "SVGUtil.h"

//<svg height="500" width="500">
//<defs>
//<radialGradient id="grad1" cx="60%" cy="40%" r="50%" fx="70%" fy="40%">
//<stop offset="0%" style="stop-color:rgb(255,255,255);stop-opacity:0.5" />
//<stop offset="100%" style="stop-color:rgb(0,0,255);stop-opacity:1" />
//</radialGradient>
//</defs>
//<ellipse cx="200" cy="200" rx="55" ry="55" fill="url(#grad1)" />
//Sorry, your browser does not support inline SVG.
//</svg>


std::string SVGGetRGB(recColor c)
{
	std::string s;
	s = "rgb(";
	s += std::to_string(int(c.r*255)) + "," + std::to_string(int(c.g*255)) + "," + std::to_string(int(c.b*255));
	s += ")";
	return s;
}

std::string SVGDefineGradient(bool horizontal, bool vertical, recColor c1, recColor c2, const char *name)
{
	std::string s;
	s += "<defs><linearGradient id=\"";
	s += name;
	s += "\" x1=\"0%\" y1=\"0%\" x2=\"0%\" y2=\"100%\">";
	s += "<stop offset=\"0%\" style=\"stop-color:";
	s += SVGGetRGB(c1);
	s += ";stop-opacity:1\" />";
	s += "<stop offset=\"100%\" style=\"stop-color:";
	s += SVGGetRGB(c2);
	s += ";stop-opacity:1\" />";
	s += "</linearGradient></defs>";
	
	return s;
}

std::string SVGDrawCircle(double x, double y, double radius, recColor c)
{
	//double epsilon = 0.5;
	std::string s;
	s += "<circle cx=\"" + std::to_string(10*x);
	s += "\" cy=\"" + std::to_string(10*y);
	s += "\" r=\""+std::to_string(radius*10)+"\" style=\"fill:"+SVGGetRGB(c)+";stroke-width:1\" />";
	return s;
}

std::string SVGDrawRect(int x, int y, int width, int height, const char *gradient)
{
	double epsilon = 0.05;
	std::string s;
	s += "<rect x=\"" + std::to_string(10*x-epsilon);
	s += "\" y=\"" + std::to_string(10*y-epsilon);
	s += "\" width=\""+std::to_string(width*10+2*epsilon)+"\" height=\""+std::to_string(height*10+2*epsilon)+"\" ";
	s += "fill=\"url(#";
	s += gradient;
	s += ")\"";
	s += " style=\"stroke-width:1\" />";
	return s;
}

std::string SVGDrawRect(int x, int y, int width, int height, recColor c)
{
	double epsilon = 0.20;
	std::string s;
	s += "<rect x=\"" + std::to_string(10*x-epsilon);
	s += "\" y=\"" + std::to_string(10*y-epsilon);
	s += "\" width=\""+std::to_string(width*10+2*epsilon)+"\" height=\""+std::to_string(height*10+2*epsilon)+"\" style=\"fill:rgb(";//128,128,0
	s += std::to_string(int(c.r*255)) + "," + std::to_string(int(c.g*255)) + "," + std::to_string(int(c.b*255));
	s += ");stroke-width:1\" />";
	return s;
}

std::string SVGFrameRect(int x, int y, int width, int height, int border, recColor c)
{
	double epsilon = 0.05;//0.5;
	std::string s;
	s += "<rect x=\"" + std::to_string(10*x-epsilon+0);
	s += "\" y=\"" + std::to_string(10*y-epsilon+0);
	s += "\" width=\""+std::to_string(width*10+2*epsilon-0)+"\" height=\""+std::to_string(height*10+2*epsilon-0)+"\" style=\"fill:none;stroke:"+SVGGetRGB(c);
	s += ";stroke-width:"+std::to_string(border)+"\" />";
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

std::string SVGDrawText(float x1, float y1, const char *txt, recColor c, double size)
{
	std::string s;
	s =  "<text x=\""+std::to_string(x1*10+2)+"\" y=\""+std::to_string(y1*10-1)+"\" text-anchor=\"middle\" style=\"fill:"+SVGGetRGB(c);
	s += "; font-family:Helvetica, sans-serif; font-size:"+std::to_string(size*10)+"px\">";
	s += txt;
	s += "</text>\n";
	//	s += "<text x=\""+std::to_string(x1*10+2)+"\" y=\""+std::to_string(y1*10-1)+"\" style=\"fill:"+SVGGetRGB(c);
	//	s += "; font-family:Impact, sans-serif; font-size:"+std::to_string(size*10)+"px; stroke:"+SVGGetRGB(notC)+"; stroke-width:0px\">";
	//	s += txt;
	//	s += "</text>";
	return s;
}

std::string SVGDrawStrokedText(float x1, float y1, const char *txt, recColor c, recColor strokeColor, double size)
{
	std::string s;
	s =  "<text x=\""+std::to_string(x1*10+2)+"\" y=\""+std::to_string(y1*10-1)+"\" style=\"fill:"+SVGGetRGB(c);
	s += "; font-family:Impact, sans-serif; font-size:"+std::to_string(size*10)+"px; stroke:"+SVGGetRGB(strokeColor)+"; stroke-width:"+std::to_string(size)+"px; stroke-linecap:round;stroke-linejoin:round\">";
	s += txt;
	s += "</text>\n";
	return s;
}