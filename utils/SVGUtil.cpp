//
//  SVGUtil.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/29/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#include "SVGUtil.h"
#include <iostream>
#include <fstream>

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


std::string SVGGetRGB(rgbColor c)
{
	std::string s;
	s = "rgb(";
	s += std::to_string(int(c.r*255)) + "," + std::to_string(int(c.g*255)) + "," + std::to_string(int(c.b*255));
	s += ")";
	return s;
}

std::string SVGDefineGradient(bool horizontal, bool vertical, rgbColor c1, rgbColor c2, const char *name)
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

std::string SVGFrameCircle(double x, double y, double radius, int border, rgbColor c)
{
	std::string s;
	s += "<circle cx=\"" + std::to_string(x);
	s += "\" cy=\"" + std::to_string(y);
	s += "\" r=\""+std::to_string(radius)+"\" style=\"fill:none;stroke:"+SVGGetRGB(c)+";stroke-width:"+std::to_string(border)+"\" />";
	return s;
}

std::string SVGDrawCircle(double x, double y, double radius, rgbColor c)
{
	//double epsilon = 0.5;
	std::string s;
	s += "<circle cx=\"" + std::to_string(x);
	s += "\" cy=\"" + std::to_string(y);
	s += "\" r=\""+std::to_string(radius)+"\" style=\"fill:"+SVGGetRGB(c)+";stroke-width:1\" />";
	return s;

//	s += "<circle cx=\"" + std::to_string(10*x);
//	s += "\" cy=\"" + std::to_string(10*y);
//	s += "\" r=\""+std::to_string(radius*10)+"\" style=\"fill:"+SVGGetRGB(c)+";stroke-width:1\" />";
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

std::string SVGDrawRect(int x, int y, int width, int height, rgbColor c)
{
	double epsilon = 0.55;
	std::string s;
	s += "<rect x=\"" + std::to_string(x-epsilon);
	s += "\" y=\"" + std::to_string(y-epsilon);
	s += "\" width=\""+std::to_string(width+2*epsilon)+"\" height=\""+std::to_string(height+2*epsilon)+"\" style=\"fill:rgb(";//128,128,0
	s += std::to_string(int(c.r*255)) + "," + std::to_string(int(c.g*255)) + "," + std::to_string(int(c.b*255));
	s += ");stroke-width:1\" />";
	return s;
}

std::string SVGFrameRect(int x, int y, int width, int height, int border, rgbColor c)
{
	double epsilon = 0;//0.05;//0.5;
	std::string s;
	s += "<rect x=\"" + std::to_string(x-epsilon+0);
	s += "\" y=\"" + std::to_string(y-epsilon+0);
	s += "\" width=\""+std::to_string(width+2*epsilon-0)+"\" height=\""+std::to_string(height+2*epsilon-0)+"\" style=\"fill:none;stroke:"+SVGGetRGB(c);
	s += ";stroke-width:"+std::to_string(border)+"\" />";
	return s;
}

std::string SVGDrawLineSegments(const std::vector<Graphics::point> &lines, float width, rgbColor c)
{
	std::string s = "<path d=\"";

	s += "M "+std::to_string(lines[0].x)+" "+std::to_string(lines[0].y)+" L";
	
	for (int x = 1; x < lines.size(); x++)
	{
		s += " "+std::to_string(lines[x].x)+" "+std::to_string(lines[x].y)+" ";
	}
//	s += "\" stroke=\"black\" stroke-width=\""+std::to_string(width)+"\" fill=\"none\" vector-effect: \"non-scaling-stroke\";/>\n";
	s += "\" stroke=\"black\" stroke-width=\""+std::to_string(width)+"\" fill=\"none\" />\n";
	return s;
}


std::string SVGDrawLine(float x1, float y1, float x2, float y2, float width, rgbColor c)
{
	std::string s;
	s = "<line x1 = \"" + std::to_string(x1) + "\" ";
	s +=      "y1 = \"" + std::to_string(y1) + "\" ";
	s +=      "x2 = \"" + std::to_string(x2) + "\" ";
	s +=      "y2 = \"" + std::to_string(y2) + "\" ";
	s += "style=\"stroke:"+SVGGetRGB(c);
	s += ";stroke-width:"+std::to_string(width)+"\" stroke-linecap=\"round\" />";
	return s;
}

std::string SVGDrawLine(int x1, int y1, int x2, int y2, int width, rgbColor c, bool center)
{
	std::string s;
	int offset = center?0.5:0;
	s = "<line x1 = \"" + std::to_string(x1+offset) + "\" ";
	s +=      "y1 = \"" + std::to_string(y1+offset) + "\" ";
	s +=      "x2 = \"" + std::to_string(x2+offset) + "\" ";
	s +=      "y2 = \"" + std::to_string(y2+offset) + "\" ";
	s += "style=\"stroke:"+SVGGetRGB(c);
	s += ";stroke-width:"+std::to_string(width)+"\" stroke-linecap=\"round\" />";
	return s;
	
//	std::string s;
//	int offset = center?5:0;
//	s = "<line x1 = \"" + std::to_string(10*x1+offset) + "\" ";
//	s +=      "y1 = \"" + std::to_string(10*y1+offset) + "\" ";
//	s +=      "x2 = \"" + std::to_string(10*x2+offset) + "\" ";
//	s +=      "y2 = \"" + std::to_string(10*y2+offset) + "\" ";
//	s += "style=\"stroke:"+SVGGetRGB(c);
//	s += ";stroke-width:"+std::to_string(width)+"\" stroke-linecap=\"round\" />";
//	return s;
}

std::string SVGDrawText(float x1, float y1, const char *txt, rgbColor c, double size, const char *typeface, SVG::svgAlignment align)
{
	std::string s;
//	s =  "<text x=\""+std::to_string(x1*10+2)+"\" y=\""+std::to_string(y1*10-1)+"\" text-anchor=\"middle\" style=\"fill:"+SVGGetRGB(c);
//	s += "; font-family:Helvetica, sans-serif; font-size:"+std::to_string(size*10)+"px\">";
	// Helvetica Impact
	s =  "<text x=\""+std::to_string(x1)+"\" y=\""+std::to_string(y1)+"\" text-anchor=\"";
	switch (align) {
		case SVG::kLeft: s += "start"; break;
		case SVG::kRight: s += "end"; break;
		case SVG::kCenter: s += "middle"; break;
	}//middle
	s += "\" style=\"fill:"+SVGGetRGB(c);
	s += "; font-family:";
	if (typeface)
		s += typeface;
	else
		s += "Helvetica";
	s += ", sans-serif; font-weight:bold; font-size:"+std::to_string(size)+"px\"";
	s += " dominant-baseline=\"middle\">";
	s += txt;
	s += "</text>";
	//	s += "<text x=\""+std::to_string(x1*10+2)+"\" y=\""+std::to_string(y1*10-1)+"\" style=\"fill:"+SVGGetRGB(c);
	//	s += "; font-family:Impact, sans-serif; font-size:"+std::to_string(size*10)+"px; stroke:"+SVGGetRGB(notC)+"; stroke-width:0px\">";
	//	s += txt;
	//	s += "</text>";
	return s;
}

std::string SVGDrawStrokedText(float x1, float y1, const char *txt, rgbColor c, rgbColor strokeColor, double size)
{
	std::string s;
	s =  "<text x=\""+std::to_string(x1*10+2)+"\" y=\""+std::to_string(y1*10-1)+"\" style=\"fill:"+SVGGetRGB(c);
	s += "; font-family:Impact, sans-serif; font-size:"+std::to_string(size*10)+"px; stroke:"+SVGGetRGB(strokeColor)+"; stroke-width:"+std::to_string(size)+"px; stroke-linecap:round;stroke-linejoin:round\">";
	s += txt;
	s += "</text>\n";
	return s;
}


void MakeSVG(const Graphics::Display &disp, const char *filename, int width, int height)
{
	std::string s = MakeSVG(disp, width, height);
	std::fstream myFile;
	
	std::fstream svgFile;
	svgFile.open(filename, std::fstream::out | std::fstream::trunc);
	svgFile << s;
	svgFile.close();
}

float PointToSVG(float p, float multiplier)
{
	return (p+1)*multiplier/2.0;
}

void PointToSVG(Graphics::point &p, float xmultiplier, float ymultiplier)
{
	p.x = (p.x+1)*xmultiplier/2.0;
	p.y = (p.y+1)*ymultiplier/2.0;
}


std::string MakeSVG(const Graphics::Display &disp, int width, int height)
{
	std::string s;
	s = "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width = \""+std::to_string(width+2)+"\" height = \""+std::to_string(height+2)+"\" ";
	s += "viewBox = \"-1 -1 "+std::to_string(width+2)+" "+std::to_string(height+2)+"\" ";
	s += "preserveAspectRatio = \"none\" ";
	s += ">\n";

//	s += SVGDrawRect(0, 0, width, height, Colors::white);
	
	for (int x = 0; x < disp.drawCommands.size(); x++)
	{
		switch (disp.drawCommands[x].what)
		{
			case Graphics::Display::kLine:
			{
				const Graphics::Display::lineInfo &o = disp.drawCommands[x].line; //disp.lines[x];
				s += SVGDrawLine((o.start.x+1)*width/2.0, (o.start.y+1)*height/2.0, (o.end.x+1)*width/2.0, (o.end.y+1)*height/2.0, o.width, o.c);
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
				const Graphics::Display::drawInfo &o = disp.drawCommands[x].shape;
				//disp.filledRects[x];
				s += SVGDrawRect(PointToSVG(o.r.left, width), PointToSVG(o.r.bottom, height),
								 PointToSVG(o.r.right, width)-PointToSVG(o.r.left, width),
								 PointToSVG(o.r.top, height)-PointToSVG(o.r.bottom, height),
								 o.c);
				break;
			}
			case Graphics::Display::kFrameRectangle:
			{
				const Graphics::Display::drawInfo &o = disp.drawCommands[x].shape;
				s += SVGFrameRect(PointToSVG(o.r.left, width), PointToSVG(o.r.top, height),
								  PointToSVG(o.r.right, width)-PointToSVG(o.r.left, width),
								  PointToSVG(o.r.top, height)-PointToSVG(o.r.bottom, height),
								  o.width, o.c);
				break;
			}
			case Graphics::Display::kFillOval:
			{
				const Graphics::Display::drawInfo &o = disp.drawCommands[x].shape;
				s += SVGDrawCircle(((o.r.left+o.r.right)/2.0+1)*width/2.0, ((o.r.top+o.r.bottom)/2.0+1)*height/2.0, width*(o.r.right-o.r.left)/4.0, o.c);
				break;
			}
			case Graphics::Display::kFrameOval:
			{
				const Graphics::Display::drawInfo &o = disp.drawCommands[x].shape;
				s += SVGFrameCircle(((o.r.left+o.r.right)/2.0+1)*width/2.0, ((o.r.top+o.r.bottom)/2.0+1)*height/2.0, width*(o.r.right-o.r.left)/4.0, o.width, o.c);

				break;
			}
		}
		s += "\n";
	}
	for (int x = 0; x < disp.text.size(); x++)
	{
		const auto &i = disp.text[x];
		s += SVGDrawText(PointToSVG(i.loc.x, width), PointToSVG(i.loc.y, height), i.s.c_str(), i.c, i.size*width/2.0, i.typeface.c_str());
		s += "\n";
	}
	static std::vector<Graphics::point> outputPoints;
	for (int x = 0; x < disp.lineSegments.size(); x++)
	{
		const auto &i = disp.lineSegments[x];
		outputPoints = i.points;
		for (auto &j : outputPoints)
			PointToSVG(j, width, height);
		s += SVGDrawLineSegments(outputPoints, i.size, i.c);
		s += "\n";
	}

	
	s += "\n";
	s += "</svg>";
	return s;
}

