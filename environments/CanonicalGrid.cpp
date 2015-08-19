//
//  CanonicalGrid.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 7/30/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//

#include "CanonicalGrid.h"

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
		xyLoc old = s;
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
	
	double CanonicalGrid::HCost(const xyLoc &l1, const xyLoc &l2)
	{
		double h1, h2;
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
	
	double CanonicalGrid::GCost(const xyLoc &l, const tDirection &act)
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
	
	double CanonicalGrid::GCost(const xyLoc &l1, const xyLoc &l2)
	{
		const double multiplier = 1.0;
		if (l1.x == l2.x) return 1.0*multiplier;
		if (l1.y == l2.y) return 1.0*multiplier;
		if (l1 == l2) return 0.0;
		return DIAGONAL_COST*multiplier;
	}
	
	bool CanonicalGrid::GoalTest(const xyLoc &node, const xyLoc &goal)
	{
		return ((node.x == goal.x) && (node.y == goal.y));
	}
	
	uint64_t CanonicalGrid::GetStateHash(const xyLoc &node) const
	{
		return (((uint64_t)node.x)<<16)|node.y;
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
		DrawSphere(xx, yy, zz, rad);
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
		DrawSphere(xx, yy, zz, rad);
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
		double xoff = sin(2*PI-angle)*rad*0.2;
		double yoff = cos(2*PI-angle)*rad*0.2;
		
		
		
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
