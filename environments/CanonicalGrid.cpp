//
//  CanonicalGrid.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 7/30/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//

#include "CanonicalGrid.h"
#include <string.h>
#include "SVGUtil.h"
#include "Graphics2D.h"

namespace CanonicalGrid {
	
	CanonicalGrid::CanonicalGrid(Map *_m)
	{
		DIAGONAL_COST = ROOT_TWO;
		map = _m;
		fourConnected = false;
		grid.resize(_m->GetMapWidth()*_m->GetMapHeight());
		for (int x = 0; x < _m->GetMapWidth(); x++)
		{
			for (int y = 0; y < _m->GetMapHeight(); y++)
			{
				if ((_m->GetTerrainType(x, y)>>terrainBits) == (kGround>>terrainBits))
					grid[x+y*_m->GetMapWidth()] = true;
				else
					grid[x+y*_m->GetMapWidth()] = false;
			}
		}
	}
	
	void CanonicalGrid::GetSuccessors(const xyLoc &loc, std::vector<xyLoc> &neighbors) const
	{
		neighbors.resize(0);
		bool n1 = false, s1 = false, e1 = false, w1 = false;

		int w = map->GetMapWidth();
		int h = map->GetMapHeight();
		if (fourConnected)
		{
			printf("Four connected not implemented!\n");
			assert(false);
		}
		else {
			if (loc.parent&kN) // action that got me here
			{
				if (loc.y != 0 && grid[loc.x+(loc.y-1)*w])
				{
					bool a = false, b = false;
					if (loc.x != 0 && !grid[loc.x-1+(loc.y)*w] && grid[loc.x-1+(loc.y-1)*w])
						a = true;
					if (loc.x != w-1 && !grid[loc.x+1+(loc.y)*w] && grid[loc.x+1+(loc.y-1)*w])
						b = true;

					if (a && b)
						neighbors.push_back(xyLoc(loc.x, loc.y-1, tDirection(kNW|kE)));
					else if (a)
						neighbors.push_back(xyLoc(loc.x, loc.y-1, kNW));
					else if (b)
						neighbors.push_back(xyLoc(loc.x, loc.y-1, kNE));
					else
						neighbors.push_back(xyLoc(loc.x, loc.y-1, kN));
					n1 = true;
				}
			}
			if (loc.parent&kW) // action that got me here
			{
				if (loc.x != 0 && grid[loc.x-1+(loc.y)*w])
				{
					bool a = false, b = false;
					if (loc.y != 0 && !grid[loc.x+(loc.y-1)*w] && grid[loc.x-1+(loc.y-1)*w])
						a = true;
					if (loc.y != h-1 && !grid[loc.x+(loc.y+1)*w] && grid[loc.x-1+(loc.y+1)*w])
						b = true;
					
					if (a && b)
						neighbors.push_back(xyLoc(loc.x-1, loc.y, tDirection(kNW|kS)));
					else if (a)
						neighbors.push_back(xyLoc(loc.x-1, loc.y, kNW));
					else if (b)
						neighbors.push_back(xyLoc(loc.x-1, loc.y, kSW));
					else
						neighbors.push_back(xyLoc(loc.x-1, loc.y, kW));
					e1 = true;
				}
			}
			if (loc.parent&kS) // action that got me here
			{
				if (loc.y != h-1 && grid[loc.x+(loc.y+1)*w])
				{
					bool a = false, b = false;
					if (loc.x != 0 && !grid[loc.x-1+(loc.y)*w] && grid[loc.x-1+(loc.y+1)*w])
						a = true;
					if (loc.x != w-1 && !grid[loc.x+1+(loc.y)*w] && grid[loc.x+1+(loc.y+1)*w])
						b = true;

					if (a && b)
						neighbors.push_back(xyLoc(loc.x, loc.y+1, tDirection(kSE|kW)));
					else if (a)
						neighbors.push_back(xyLoc(loc.x, loc.y+1, kSW));
					else if (b)
						neighbors.push_back(xyLoc(loc.x, loc.y+1, kSE));
					else
						neighbors.push_back(xyLoc(loc.x, loc.y+1, kS));
					s1 = true;
				}
			}
			if (loc.parent&kE) // action that got me here
			{
				if (loc.x != w-1 && grid[loc.x+1+(loc.y)*w])
				{
					bool a = false, b = false;

					if (loc.y != 0 && !grid[loc.x+(loc.y-1)*w] && grid[loc.x+1+(loc.y-1)*w])
						a = true;
					if (loc.y != h-1 && !grid[loc.x+(loc.y+1)*w] && grid[loc.x+1+(loc.y+1)*w])
						b = true;
					
					if (a && b)
						neighbors.push_back(xyLoc(loc.x+1, loc.y, tDirection(kNE|kS)));
					else if (a)
						neighbors.push_back(xyLoc(loc.x+1, loc.y, kNE));
					else if (b)
						neighbors.push_back(xyLoc(loc.x+1, loc.y, kSE));
					else
						neighbors.push_back(xyLoc(loc.x+1, loc.y, kE));
					w1 = true;
				}
			}
			if (loc.parent&kNW)
			{
				if (loc.x != 0 && loc.y != 0 && grid[loc.x-1+(loc.y-1)*w] && n1 && e1)
				{
					neighbors.push_back(xyLoc(loc.x-1, loc.y-1, kNW));
				}
			}
			if (loc.parent&kNE)
			{
				if (loc.x != w-1 && loc.y != 0 && grid[loc.x+1+(loc.y-1)*w] && n1 && w1)
				{
					neighbors.push_back(xyLoc(loc.x+1, loc.y-1, kNE));
				}
			}
			if (loc.parent&kSW)
			{
				if (loc.x != 0 && loc.y != h-1 && grid[loc.x-1+(loc.y+1)*w] && s1 && e1)
				{
					neighbors.push_back(xyLoc(loc.x-1, loc.y+1, kSW));
				}
			}
			if (loc.parent&kSE)
			{
				if (loc.x != w-1 && loc.y != h-1 && grid[loc.x+1+(loc.y+1)*w] && s1 && w1)
				{
					neighbors.push_back(xyLoc(loc.x+1, loc.y+1, kSE));
				}
			}
		}
		
	}
	
	void CanonicalGrid::GetActions(const xyLoc &loc, std::vector<tDirection> &actions) const
	{
		bool up=false, down=false;
		if ((map->CanStep(loc.x, loc.y, loc.x, loc.y+1)))
		{
			down = true;
			actions.push_back(kS);
		}
		if ((map->CanStep(loc.x, loc.y, loc.x, loc.y-1)))
		{
			up = true;
			actions.push_back(kN);
		}
		if ((map->CanStep(loc.x, loc.y, loc.x-1, loc.y)))
		{
			if (!fourConnected)
			{
				if ((up && (map->CanStep(loc.x, loc.y, loc.x-1, loc.y-1))))
					actions.push_back(kNW);
				if ((down && (map->CanStep(loc.x, loc.y, loc.x-1, loc.y+1))))
					actions.push_back(kSW);
			}
			actions.push_back(kW);
		}
		if ((map->CanStep(loc.x, loc.y, loc.x+1, loc.y)))
		{
			if (!fourConnected)
			{
				if ((up && (map->CanStep(loc.x, loc.y, loc.x+1, loc.y-1))))
					actions.push_back(kNE);
				if ((down && (map->CanStep(loc.x, loc.y, loc.x+1, loc.y+1))))
					actions.push_back(kSE);
			}
			actions.push_back(kE);
		}
	}
	
	tDirection CanonicalGrid::GetAction(const xyLoc &s1, const xyLoc &s2) const
	{
		int result = kStay;
		switch (s1.x-s2.x)
		{
			case -1: result = kE; break;
			case 0: break;
			case 1: result = kW; break;
		}
		
		// Tack the vertical move onto it
		// Notice the exploit of particular encoding of kStay, kE, etc. labels
		switch (s1.y-s2.y)
		{
			case -1: result = result|kS; break;
			case 0: break;
			case 1: result = result|kN; break;
		}
		return (tDirection)result;
	}
	
	bool CanonicalGrid::InvertAction(tDirection &a) const
	{
		switch (a)
		{
			case kN: a = kS; break;
			case kNE: a = kSW; break;
			case kE: a = kW; break;
			case kSE: a = kNW; break;
			case kS: a = kN; break;
			case kSW: a = kNE; break;
			case kW: a = kE; break;
			case kNW: a = kSE; break;
			default: break;
		}
		return true;
	}
	
	void CanonicalGrid::ApplyAction(xyLoc &s, tDirection dir) const
	{
		//xyLoc old = s;
		switch (dir)
		{
			case kN: s.y-=1; break;
			case kS: s.y+=1; break;
			case kE: s.x+=1; break;
			case kW: s.x-=1; break;
			case kNW: s.y-=1; s.x-=1; break;
			case kSW: s.y+=1; s.x-=1; break;
			case kNE: s.y-=1; s.x+=1; break;
			case kSE: s.y+=1; s.x+=1; break;
			default: break;
		}
	}
	
	double CanonicalGrid::HCost(const xyLoc &l1, const xyLoc &l2) const
	{
		double h1;//, h2;
		if (fourConnected)
		{
			h1 = abs(l1.x-l2.x)+abs(l1.y-l2.y);
		}
		else {
			double a = ((l1.x>l2.x)?(l1.x-l2.x):(l2.x-l1.x));
			double b = ((l1.y>l2.y)?(l1.y-l2.y):(l2.y-l1.y));
			//return sqrt(a*a+b*b);
			h1 = (a>b)?(b*DIAGONAL_COST+a-b):(a*DIAGONAL_COST+b-a);
		}
		
		return h1;
	}
	
	double CanonicalGrid::GCost(const xyLoc &l, const tDirection &act) const
	{
		const double multiplier = 1.0;
		switch (act)
		{
			case kN: return 1.0*multiplier;
			case kS: return 1.0*multiplier;
			case kE: return 1.0*multiplier;
			case kW: return 1.0*multiplier;
			case kNW: return DIAGONAL_COST*multiplier;
			case kSW: return DIAGONAL_COST*multiplier;
			case kNE: return DIAGONAL_COST*multiplier;
			case kSE: return DIAGONAL_COST*multiplier;
			default: return 0;
		}
		return 0;
	}
	
	double CanonicalGrid::GCost(const xyLoc &l1, const xyLoc &l2) const
	{
		const double multiplier = 1.0;
		if (l1.x == l2.x) return 1.0*multiplier;
		if (l1.y == l2.y) return 1.0*multiplier;
		if (l1 == l2) return 0.0;
		return DIAGONAL_COST*multiplier;
	}
	
	bool CanonicalGrid::GoalTest(const xyLoc &node, const xyLoc &goal) const
	{
		return ((node.x == goal.x) && (node.y == goal.y));
	}
	
	uint64_t CanonicalGrid::GetMaxHash() const
	{
		return map->GetMapHeight()*map->GetMapWidth();
		
	}

	uint64_t CanonicalGrid::GetStateHash(const xyLoc &node) const
	{
		return node.y*map->GetMapWidth()+node.x;
		//return (((uint64_t)node.x)<<16)|node.y;
		//	return (node.x<<16)|node.y;
	}
	
	uint64_t CanonicalGrid::GetActionHash(tDirection act) const
	{
		return (uint32_t) act;
	}
	
	void CanonicalGrid::OpenGLDraw() const
	{
		map->OpenGLDraw();
	}
	
	void CanonicalGrid::OpenGLDraw(const xyLoc &l) const
	{
		GLdouble xx, yy, zz, rad;
		map->GetOpenGLCoord(l.x, l.y, xx, yy, zz, rad);
		GLfloat r, g, b, t;
		GetColor(r, g, b, t);
		glColor4f(r, g, b, t);
		//glColor3f(0.5, 0.5, 0.5);
		//DrawSphere(xx, yy, zz, rad);
		//DrawSquare(xx, yy, zz+rad, rad);
		DrawSquare(xx, yy, zz-rad, rad);
	}
	
	void CanonicalGrid::OpenGLDraw(const xyLoc &l1, const xyLoc &l2, float v) const
	{
		GLdouble xx, yy, zz, rad;
		GLdouble xx2, yy2, zz2;
		map->GetOpenGLCoord(l1.x, l1.y, xx, yy, zz, rad);
		map->GetOpenGLCoord(l2.x, l2.y, xx2, yy2, zz2, rad);
		xx = (1-v)*xx+v*xx2;
		yy = (1-v)*yy+v*yy2;
		zz = (1-v)*zz+v*zz2;
		GLfloat r, g, b, t;
		GetColor(r, g, b, t);
		glColor4f(r, g, b, t);
		DrawSquare(xx, yy, zz+rad, rad);
		//DrawSphere(xx, yy, zz, rad);
	}
	
	void CanonicalGrid::OpenGLDraw(const xyLoc& initial, const tDirection &dir) const
	{
		
		xyLoc s = initial;
		GLdouble xx, yy, zz, rad;
		map->GetOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
		
		glColor3f(0.5, 0.5, 0.5);
		glBegin(GL_LINE_STRIP);
		glVertex3f(xx, yy, zz-rad/2);
		
		switch (dir)
		{
			case kN: s.y-=1; break;
			case kS: s.y+=1; break;
			case kE: s.x+=1; break;
			case kW: s.x-=1; break;
			case kNW: s.y-=1; s.x-=1; break;
			case kSW: s.y+=1; s.x-=1; break;
			case kNE: s.y-=1; s.x+=1; break;
			case kSE: s.y+=1; s.x+=1; break;
			default: break;
		}
		
		
		map->GetOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
		glVertex3f(xx, yy, zz-rad/2);
		glEnd();
		
	}
	
	void CanonicalGrid::GLDrawLine(const xyLoc &a, const xyLoc &b) const
	{
		GLdouble xx1, yy1, zz1, rad;
		GLdouble xx2, yy2, zz2;
		map->GetOpenGLCoord(a.x, a.y, xx1, yy1, zz1, rad);
		map->GetOpenGLCoord(b.x, b.y, xx2, yy2, zz2, rad);
		
		double angle = atan2(yy1-yy2, xx1-xx2);
		double xoff = sin(2*PI-angle)*rad*0.1;
		double yoff = cos(2*PI-angle)*rad*0.1;
		
		
		
		GLfloat rr, gg, bb, t;
		GetColor(rr, gg, bb, t);
		glColor4f(rr, gg, bb, t);
		
		glBegin(GL_TRIANGLE_STRIP);
		
		glVertex3f(xx1+xoff, yy1+yoff, zz1-rad/2);
		glVertex3f(xx2+xoff, yy2+yoff, zz2-rad/2);
		glVertex3f(xx1-xoff, yy1-yoff, zz1-rad/2);
		glVertex3f(xx2-xoff, yy2-yoff, zz2-rad/2);
		glEnd();
	}
	
	void CanonicalGrid::GLLabelState(const xyLoc &s, const char *str, double scale) const
	{
		glPushMatrix();
		
		GLdouble xx, yy, zz, rad;
		map->GetOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
		GLfloat r, g, b, t;
		GetColor(r, g, b, t);
		glColor4f(r, g, b, t);
		
		glTranslatef(xx-rad, yy+rad/2, zz-2*rad);
		glScalef(scale*rad/(300.0), scale*rad/300.0, 1);
		glRotatef(180, 0.0, 0.0, 1.0);
		glRotatef(180, 0.0, 1.0, 0.0);
		glDisable(GL_LIGHTING);
		for (int which = 0; which < strlen(str); which++)
			glutStrokeCharacter(GLUT_STROKE_ROMAN, str[which]);
		glEnable(GL_LIGHTING);
		glPopMatrix();
	}
	
	void CanonicalGrid::GLLabelState(const xyLoc &s, const char *str) const
	{
		glPushMatrix();
		
		GLdouble xx, yy, zz, rad;
		map->GetOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
		GLfloat r, g, b, t;
		GetColor(r, g, b, t);
		glColor4f(r, g, b, t);
		
		glTranslatef(xx-rad, yy+rad/2, zz-rad);
		glScalef(rad/(300.0), rad/300.0, 1);
		glRotatef(180, 0.0, 0.0, 1.0);
		glRotatef(180, 0.0, 1.0, 0.0);
		glDisable(GL_LIGHTING);
		for (int which = 0; which < strlen(str); which++)
			glutStrokeCharacter(GLUT_STROKE_ROMAN, str[which]);
		glEnable(GL_LIGHTING);
		glPopMatrix();
	}

	std::string CanonicalGrid::SVGHeader()
	{
		std::string s;
		// 10% margin on all sides of image
		s = "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width = \""+std::to_string(10*map->GetMapWidth())+"\" height = \""+std::to_string(10*map->GetMapHeight())+"\" ";
		s += "viewBox=\""+std::to_string(-map->GetMapWidth())+" "+std::to_string(-map->GetMapHeight())+" ";
		s += std::to_string(12*map->GetMapWidth())+" "+std::to_string(12*map->GetMapHeight())+"\" ";
		s += "preserveAspectRatio = \"none\" ";
		s += ">\n";
		return s;
	}
	
	std::string CanonicalGrid::SVGDraw()
	{
		std::string s;
		recColor black = {0.0, 0.0, 0.0};
		
		// draw tiles
		for (int y = 0; y < map->GetMapHeight(); y++)
		{
			for (int x = 0; x < map->GetMapWidth(); x++)
			{
				bool draw = true;
				if (map->GetTerrainType(x, y) == kGround)
				{
					recColor c = {0.9, 0.9, 0.9};
					s += SVGDrawRect(x+1, y+1, 1, 1, c);
					s += "\n";
				}
				else if (map->GetTerrainType(x, y) == kTrees)
				{
					recColor c = {0.0, 0.5, 0.0};
					s += SVGDrawRect(x+1, y+1, 1, 1, c);
					s += "\n";
				}
				else if (map->GetTerrainType(x, y) == kWater)
				{
					recColor c = {0.0, 0.0, 1.0};
					s += SVGDrawRect(x+1, y+1, 1, 1, c);
					s += "\n";
				}
				else if (map->GetTerrainType(x, y) == kSwamp)
				{
					recColor c = {0.0, 0.3, 1.0};
					s += SVGDrawRect(x+1, y+1, 1, 1, c);
					s += "\n";
				}
				else {
					draw = false;
				}
			}
		}
		
		// draw cell boundaries for open terrain
		for (int y = 0; y < map->GetMapHeight(); y++)
		{
			for (int x = 0; x < map->GetMapWidth(); x++)
			{
				// mark cells on map
				if ((map->GetTerrainType(x, y)>>terrainBits) == (kGround>>terrainBits))
				{
					recColor c = {0.75, 0.75, 0.75};
					s += ::SVGFrameRect(x+1, y+1, 1, 1, 1, c);
					s += "\n";
				}
			}
		}
		
		// draw lines between different terrain types
		for (int y = 0; y < map->GetMapHeight(); y++)
		{
			for (int x = 0; x < map->GetMapWidth(); x++)
			{
				bool draw = true;
				if (map->GetTerrainType(x, y) == kGround)
				{
					if (x == map->GetMapWidth()-1)
						s += ::SVGDrawLine(x+1+1, y+1, x+1+1, y+1+1, 1, black, false);
					if (y == map->GetMapHeight()-1)
						s += ::SVGDrawLine(x+1, y+1+1, x+1+1, y+1+1, 1, black, false);
				}
				else if (map->GetTerrainType(x, y) == kTrees)
				{
					if (x == map->GetMapWidth()-1)
						s += ::SVGDrawLine(x+1+1, y+1, x+1+1, y+1+1, 1, black, false);
					if (y == map->GetMapHeight()-1)
						s += ::SVGDrawLine(x+1, y+1+1, x+1+1, y+1+1, 1, black, false);
				}
				else if (map->GetTerrainType(x, y) == kWater)
				{
					if (x == map->GetMapWidth()-1)
						s += ::SVGDrawLine(x+1+1, y+1, x+1+1, y+1+1, 1, black, false);
					if (y == map->GetMapHeight()-1)
						s += ::SVGDrawLine(x+1, y+1+1, x+1+1, y+1+1, 1, black, false);
				}
				else if (map->GetTerrainType(x, y) == kSwamp)
				{
				}
				else {
					draw = false;
				}
				
				if (draw)
				{
					SetColor(0.0, 0.0, 0.0);
					
					// Code does error checking, so this works with x == 0
					if (map->GetTerrainType(x, y) != map->GetTerrainType(x-1, y))
					{
						SetColor(0.0, 0.0, 0.0);
						s += ::SVGDrawLine(x+1, y+1, x+1, y+1+1, 1, black, false);
						s += "\n";
					}
					
					if (map->GetTerrainType(x, y) != map->GetTerrainType(x, y-1))
					{
						s += ::SVGDrawLine(x+1, y+1, x+1+1, y+1, 1, black, false);
						s += "\n";
					}
					
					if (map->GetTerrainType(x, y) != map->GetTerrainType(x+1, y))
					{
						s += ::SVGDrawLine(x+1+1, y+1, x+1+1, y+1+1, 1, black, false);
						s += "\n";
					}
					
					if (map->GetTerrainType(x, y) != map->GetTerrainType(x, y+1))
					{
						s += ::SVGDrawLine(x+1, y+1+1, x+1+1, y+1+1, 1, black, false);
						s += "\n";
					}
				}
				
			}
		}
		s += "\n";
		
		return s;
	}
	
	std::string CanonicalGrid::SVGDraw(const xyLoc &l)
	{
		std::string s;
		if (map->GetTerrainType(l.x, l.y) == kGround)
		{
			recColor c;// = {0.5, 0.5, 0};
			GLfloat t;
			GetColor(c.r, c.g, c.b, t);
			s += SVGDrawCircle(l.x+0.5+1, l.y+0.5+1, 0.5, c);
			//stroke-width="1" stroke="pink" />
		}
		return s;
	}
	
	std::string CanonicalGrid::SVGFrameRect(int left, int top, int right, int bottom, int width)
	{
		std::string s;
		
		recColor c;// = {0.5, 0.5, 0};
		GLfloat t;
		GetColor(c.r, c.g, c.b, t);
		s += ::SVGFrameRect(left+1, top+1, right-left+1, bottom-top+1, width, c);
		
		return s;
	}
	
	std::string CanonicalGrid::SVGLabelState(const xyLoc &l, const char *str, double scale) const
	{
		std::string s;
		recColor c;// = {0.5, 0.5, 0};
		GLfloat t;
		GetColor(c.r, c.g, c.b, t);
		s += SVGDrawText(l.x+0.5+1, l.y+0.5+1+1, str, c, scale);
		return s;
		//	std::string s;
		//	s =  "<text x=\"0\" y=\"15\" fill=\"black\">";
		//	s += str;
		//	s += "</text>";
		//	return s;
	}
	
	std::string CanonicalGrid::SVGDrawLine(const xyLoc &p1, const xyLoc &p2, int width) const
	{
		//<line x1="0" y1="0" x2="200" y2="200" style="stroke:rgb(255,255,255);stroke-width:1" />
		//std::string s;
		recColor c;// = {0.5, 0.5, 0};
		GLfloat t;
		GetColor(c.r, c.g, c.b, t);
		return ::SVGDrawLine(p1.x+1, p1.y+1, p2.x+1, p2.y+1, width, c);
		
		//	s = "<line x1 = \"" + std::to_string(p1.x) + "\" ";
		//	s +=      "y1 = \"" + std::to_string(p1.y) + "\" ";
		//	s +=      "x2 = \"" + std::to_string(p2.x) + "\" ";
		//	s +=      "y2 = \"" + std::to_string(p2.y) + "\" ";
		//	s += "style=\"stroke:"+SVGGetRGB(c)+";stroke-width:"+std::to_string(width)+"\" />";
		//	return s;
	}

	void CanonicalGrid::DrawOrdering(xyLoc l) const
	{
		recColor c;
		{
			GLfloat r,g,b,t;
			GetColor(r, g, b, t);
			c = {r, g, b};
		}
		std::deque<xyLoc> queue;
		queue.push_back(l);
		std::vector<xyLoc> v;
		std::vector<bool> visited(map->GetMapHeight()*map->GetMapWidth());
		while (!queue.empty())
		{
			GetSuccessors(queue.front(), v);
			for (auto &s : v)
			{
				if (!visited[s.x+s.y*map->GetMapWidth()])
				{
					queue.push_back(s);
				}
//				else {
//					ma1->SetColor(1.0, 0.0, 0.0);
//				}
				Graphics2D::point2d p1, p2;
				{
					GLdouble x, y, z, r;
					map->GetOpenGLCoord(queue.front().x, queue.front().y, x, y, z, r);
					p1.x = x;
					p1.y = y;
				}
				{
					GLdouble x, y, z, r;
					map->GetOpenGLCoord(s.x, s.y, x, y, z, r);
					p2.x = x;
					p2.y = y;
				}
				Graphics2D::DrawLine(p1, p2, 1, c);
				visited[s.x+s.y*map->GetMapWidth()] = true;
			}
			queue.pop_front();
		}
	}

	
	void CanonicalGrid::GetNextState(const xyLoc &currents, tDirection dir, xyLoc &news) const
	{
		news = currents;
		switch (dir)
		{
			case kN: news.y-=1; break;
			case kS: news.y+=1; break;
			case kE: news.x+=1; break;
			case kW: news.x-=1; break;
			case kNW: news.y-=1; news.x-=1; break;
			case kSW: news.y+=1; news.x-=1; break;
			case kNE: news.y-=1; news.x+=1; break;
			case kSE: news.y+=1; news.x+=1; break;
			default: break;
		}	
	}
	
}
