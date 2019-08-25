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
	s += "<circle cx=\"" + to_string_with_precision(x);
	s += "\" cy=\"" + to_string_with_precision(y);
	s += "\" r=\""+to_string_with_precision(radius)+"\" style=\"fill:none;stroke:"+SVGGetRGB(c)+";stroke-width:"+to_string_with_precision(border)+"\" />";
	return s;
}

std::string SVGDrawCircle(double x, double y, double radius, rgbColor c)
{
	//double epsilon = 0.5;
	std::string s;
	s += "<circle cx=\"" + to_string_with_precision(x);
	s += "\" cy=\"" + to_string_with_precision(y);
	s += "\" r=\""+to_string_with_precision(radius)+"\" style=\"fill:"+SVGGetRGB(c)+";stroke-width:1\" />";
	return s;

//	s += "<circle cx=\"" + std::to_string(10*x);
//	s += "\" cy=\"" + std::to_string(10*y);
//	s += "\" r=\""+std::to_string(radius*10)+"\" style=\"fill:"+SVGGetRGB(c)+";stroke-width:1\" />";
}

std::string SVGDrawRect(float x, float y, float width, float height, const char *gradient)
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

std::string SVGDrawRect(float x, float y, float width, float height, rgbColor c)
{
	double epsilon = 0.0;//0.55;
	std::string s;
	s += "<rect x=\"" + to_string_with_precision(x-epsilon);
	s += "\" y=\"" + to_string_with_precision(y-epsilon);
	s += "\" width=\""+to_string_with_precision(width+2*epsilon, 8)+"\" height=\""+to_string_with_precision(height+2*epsilon, 8)+"\" style=\"fill:"+SVGGetRGB(c)+";\" />";
//	s += "\" width=\""+to_string_with_precision(width+2*epsilon, 8)+"\" height=\""+to_string_with_precision(height+2*epsilon, 8)+"\" style=\"fill:"+SVGGetRGB(c)+";stroke:"+SVGGetRGB(c)+";stroke-width:5%\" />";
	return s;
}

std::string SVGFrameRect(int x, int y, int width, int height, int border, rgbColor c)
{
	double epsilon = 0;//0.05;//0.5;
	std::string s;
	s += "<rect x=\"" + to_string_with_precision(x-epsilon+0);
	s += "\" y=\"" + to_string_with_precision(y-epsilon+0);
	s += "\" width=\""+to_string_with_precision(width+2*epsilon-0)+"\" height=\""+to_string_with_precision(height+2*epsilon-0)+"\" style=\"fill:none;stroke:"+SVGGetRGB(c);
	s += ";stroke-width:"+std::to_string(border)+"\" />";
	return s;
}

std::string SVGFrameNGon(double _x, double _y, double radius, int segments, float rotation, int border, rgbColor c)
{
	double resolution = TWOPI/segments;
	std::string s;
	s += "<polygon points=\"";

	for (int x = 0; x <= segments; x++)
	{
		s += to_string_with_precision(_x+sin(resolution*x+rotation*TWOPI/360.0)*radius);
		s += ",";
		s += to_string_with_precision(_y+cos(resolution*x+rotation*TWOPI/360.0)*radius);
		s += " ";
	}
	s += "\" style=\"fill:none;stroke:purple;stroke-width:1\" />";
	return s;
}

std::string SVGDrawNGon(double _x, double _y, double radius, int segments, float rotation, rgbColor c)
{
	double resolution = TWOPI/segments;
	std::string s;
	s += "<polygon points=\"";
	
	for (int x = 0; x <= segments; x++)
	{
		s += to_string_with_precision(_x+sin(resolution*x+rotation*TWOPI/360.0)*radius);
		s += ",";
		s += to_string_with_precision(_y+cos(resolution*x+rotation*TWOPI/360.0)*radius);
		s += " ";
	}
	s += "\" style=\"fill:"+SVGGetRGB(c)+"\" />";
	return s;
}

//void DrawCircle(GLdouble _x, GLdouble _y, GLdouble tRadius, int segments = 32, float rotation = 0);
//void FrameCircle(GLdouble _x, GLdouble _y, GLdouble tRadius, GLdouble lineWidth, int segments = 32, float rotation = 0);

std::string SVGBeginLinePath(float width, rgbColor c)
{
	return "<path stroke-linecap=\"round\" stroke=\""+SVGGetRGB(c)+"\" stroke-width=\""+std::to_string(width)+"\" fill=\"none\" d=\"";
//
//	s += "M "+std::to_string(lines[0].x)+" "+std::to_string(lines[0].y)+" L";
//
//	for (int x = 1; x < lines.size(); x++)
//	{
//		s += " "+std::to_string(lines[x].x)+" "+std::to_string(lines[x].y)+" ";
//	}
//	//	s += "\" stroke=\"black\" stroke-width=\""+std::to_string(width)+"\" fill=\"none\" vector-effect: \"non-scaling-stroke\";/>\n";
//	s += "\" stroke=\""+SVGGetRGB(c)+"\" stroke-width=\""+std::to_string(width)+"\" fill=\"none\" />\n";
//
}

std::string SVGAddLinePath(float x1, float y1)
{
	return "L "+std::to_string(x1)+" "+std::to_string(y1)+" ";
}

std::string SVGAddLinePath(float x1, float y1, float x2, float y2)
{
	return "M "+std::to_string(x1)+" "+std::to_string(y1)+" L "+std::to_string(x2)+" "+std::to_string(y2)+" ";
}

std::string SVGEndLinePath()
{ return "\" />\n"; }


std::string SVGDrawLineSegments(const std::vector<Graphics::point> &lines, float width, rgbColor c)
{
	std::string s = "<path d=\"";

	s += "M "+std::to_string(lines[0].x)+" "+std::to_string(lines[0].y)+" L";
	
	for (int x = 1; x < lines.size(); x++)
	{
		s += " "+std::to_string(lines[x].x)+" "+std::to_string(lines[x].y)+" ";
	}
//	s += "\" stroke=\"black\" stroke-width=\""+std::to_string(width)+"\" fill=\"none\" vector-effect: \"non-scaling-stroke\";/>\n";
	s += "\" stroke=\""+SVGGetRGB(c)+"\" stroke-width=\""+std::to_string(width)+"\" fill=\"none\" />\n";
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
	float offset = center?0.5:0;
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


void MakeSVG(const Graphics::Display &disp, const char *filename, int width, int height, int viewport, const char *comment)
{
	std::string s = MakeSVG(disp, width, height, viewport, comment);
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


void HandleCommand(const std::vector<Graphics::Display::data> &drawCommands, std::string &s, int width, int height, int viewport)
{
	for (int x = 0; x < drawCommands.size(); x++)
	{
		if (drawCommands[x].viewport != viewport)
			continue;
		switch (drawCommands[x].what)
		{
			case Graphics::Display::kLine:
			{
				s += SVGBeginLinePath(drawCommands[x].line.width, drawCommands[x].line.c);
				bool first = true;
				while (drawCommands[x].what == Graphics::Display::kLine &&
					   (first ||
						(drawCommands[x].line.width == drawCommands[x-1].line.width &&
						 drawCommands[x].line.c == drawCommands[x-1].line.c)))
				{
					const Graphics::Display::lineInfo &o = drawCommands[x].line; //disp.lines[x];
					if (first)
					{
						s += SVGAddLinePath(PointToSVG(o.start.x, width), PointToSVG(o.start.y, height),
											PointToSVG(o.end.x, width), PointToSVG(o.end.y, height));
						first = false;
					}
					else if (drawCommands[x-1].line.end == o.start)
					{
						s += SVGAddLinePath(PointToSVG(o.end.x, width), PointToSVG(o.end.y, height));
					}
					else {
						s += SVGAddLinePath(PointToSVG(o.start.x, width), PointToSVG(o.start.y, height),
											PointToSVG(o.end.x, width), PointToSVG(o.end.y, height));
					}
//					s += SVGDrawLine((o.start.x+1)*width/2.0, (o.start.y+1)*height/2.0, (o.end.x+1)*width/2.0, (o.end.y+1)*height/2.0, o.width, o.c);

					x++;
				}
				x--;
				s += SVGEndLinePath();
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
				const Graphics::Display::drawInfo &o = drawCommands[x].shape;
				if (drawCommands.size() > x+1 && drawCommands[x+1].what == Graphics::Display::kFillRectangle &&
					drawCommands[x+1].shape.c == o.c)
				{
					s += "<g style=\"fill:"+SVGGetRGB(o.c)+";\">\n";
					
					while (true)
					{
						const Graphics::Display::drawInfo &o1 = drawCommands[x].shape;
						float x1 = PointToSVG(o1.r.left, width);
						float y1 = PointToSVG(o1.r.top, height);
						float w1 = PointToSVG(o1.r.right, width)-PointToSVG(o1.r.left, width);
						float h1 = PointToSVG(o1.r.bottom, height)-PointToSVG(o1.r.top, height);
						s += " <rect x=\"" + to_string_with_precision(x1);
						s += "\" y=\"" + to_string_with_precision(y1);
						s += "\" width=\""+to_string_with_precision(w1)+"\" height=\""+to_string_with_precision(h1)+"\"/>\n";
						
						x++;
						if (drawCommands.size() > x && drawCommands[x].what == Graphics::Display::kFillRectangle &&
							drawCommands[x].shape.c == o.c)
						{
							
						}
						else {
							x--;
							break;
						}
					}
					s += "</g>";
				}
				else {
					s += SVGDrawRect(PointToSVG(o.r.left, width), PointToSVG(o.r.top, height),
									 PointToSVG(o.r.right, width)-PointToSVG(o.r.left, width),
									 PointToSVG(o.r.bottom, height)-PointToSVG(o.r.top, height),
									 o.c);
				}
				break;
			}
			case Graphics::Display::kFrameRectangle:
			{
				const Graphics::Display::drawInfo &o = drawCommands[x].shape;
				s += SVGFrameRect(PointToSVG(o.r.left, width), PointToSVG(o.r.top, height),
								  PointToSVG(o.r.right, width)-PointToSVG(o.r.left, width),
								  PointToSVG(o.r.bottom, height)-PointToSVG(o.r.top, height),
								  o.width, o.c);
				break;
			}
			case Graphics::Display::kFillOval:
			{
				const Graphics::Display::drawInfo &o = drawCommands[x].shape;
				s += SVGDrawCircle(((o.r.left+o.r.right)/2.0+1)*width/2.0, ((o.r.top+o.r.bottom)/2.0+1)*height/2.0, width*(o.r.right-o.r.left)/4.0, o.c);
				break;
			}
			case Graphics::Display::kFrameOval:
			{
				const Graphics::Display::drawInfo &o = drawCommands[x].shape;
				s += SVGFrameCircle(((o.r.left+o.r.right)/2.0+1)*width/2.0, ((o.r.top+o.r.bottom)/2.0+1)*height/2.0, width*(o.r.right-o.r.left)/4.0, o.width, o.c);
				
				break;
			}
			case Graphics::Display::kFillNGon:
			{
				const Graphics::Display::shapeInfo &i = drawCommands[x].polygon;
				Graphics::point p = i.center;
				PointToSVG(p, width, height);
				s += SVGDrawNGon(p.x, p.y, width*i.radius/2.0, i.segments, i.rotate, i.c);
				//				s += SVGDrawNGon(i.center.x, i.center.y, i.radius, i.segments, i.rotate, i.c);
				break;
			}
			case Graphics::Display::kFrameNGon:
			{
				const Graphics::Display::shapeInfo &i = drawCommands[x].polygon;
				Graphics::point p = i.center;
				PointToSVG(p, width, height);
				s += SVGFrameNGon(p.x, p.y, width*i.radius/2.0, i.segments, i.rotate, 1/*border*/, i.c);
				break;
			}
		}
	}
}

std::string MakeSVG(const Graphics::Display &disp, int width, int height, int viewport, const char *comment)
{
	std::string s;
	s = "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width = \""+std::to_string(width+3)+"\" height = \""+std::to_string(height+3)+"\" ";
	s += "viewBox = \"-1 -1 "+std::to_string(width+2)+" "+std::to_string(height+2)+"\" ";
	s += "preserveAspectRatio = \"none\" shape-rendering=\"crispEdges\""; // crispEdges
	s += ">\n";

//	s += SVGDrawRect(0, 0, width, height, Colors::white);
	
	HandleCommand(disp.backgroundDrawCommands, s, width, height, viewport);
	HandleCommand(disp.drawCommands, s, width, height, viewport);

	for (int x = 0; x < disp.text.size(); x++)
	{
		const auto &i = disp.text[x];
		if (i.viewport == viewport)
		{
			s += SVGDrawText(PointToSVG(i.loc.x, width),
							 PointToSVG(i.loc.y, height),
							 i.s.c_str(), i.c, i.size*width/2.0, i.typeface.c_str());
			s += "\n";
		}
	}
	static std::vector<Graphics::point> outputPoints;
	for (int x = 0; x < disp.lineSegments.size(); x++)
	{
		const auto &i = disp.lineSegments[x];
		if (i.viewport == viewport)
		{
			outputPoints = i.points;
			for (auto &j : outputPoints)
				PointToSVG(j, width, height);
			s += SVGDrawLineSegments(outputPoints, i.size, i.c);
			s += "\n";
		}
	}

	
	s += "\n";
	if (comment)
	{
		s += "<!--";
		s += comment;
		s += "-->\n";
	}
	s += "</svg>";
	return s;
}



