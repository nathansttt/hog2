/*
 * $Id: glUtil.cpp,v 1.6 2006/10/18 23:52:53 nathanst Exp $
 *
 *  glUtil.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 6/8/05.
 *  Copyright 2005 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include "GLUtil.h"
#include "FPUtil.h"
#include <math.h>
#include <assert.h>
#include <vector>

bool fastCrossTest(float p0_x, float p0_y, float p1_x, float p1_y, 
				   float p2_x, float p2_y, float p3_x, float p3_y, float *i_x, float *i_y);

bool operator==(const recVec &l1, const recVec &l2)
{
	return (fequal(l1.x, l2.x) && fequal(l1.y, l2.y));
}

std::ostream& operator <<(std::ostream &out, const recVec &loc)
{
	out << "(" << loc.x << ", " << loc.y << ")";
	return out;
}
	

/**
* Normalize a vector.
 *
 * this really should be part of the recVec class -- normalizes a vector
 */
void recVec::normalise()
{
	double length;
	
	length = sqrt(x * x + y * y + z * z);
	if (length != 0)
	{
		x /= length;
		y /= length;
		z /= length;
	}
	else {
		x = 0;
		y = 0;
		z = 0;
	}
}

bool line2d::crosses(line2d which) const
{
	if ((which.start == start) || (which.end == end) ||
		(which.start == end) || (which.end == start))
		return false;

	//	//input x1,y1 input x2,y2
	//input u1,v1 input u2,v2
	line2d here(start, end);
	double maxx1, maxx2, maxy1, maxy2;
	double minx1, minx2, miny1, miny2;
	if (here.start.x > here.end.x)
	{ maxx1 = here.start.x; minx1 = here.end.x; }
	else
	{ maxx1 = here.end.x; minx1 = here.start.x; }
	
	if (here.start.y > here.end.y)
	{ maxy1 = here.start.y; miny1 = here.end.y; }
	else
	{ maxy1 = here.end.y; miny1 = here.start.y; }
	
	if (which.start.x > which.end.x)
	{ maxx2 = which.start.x; minx2 = which.end.x; }
	else
	{ maxx2 = which.end.x; minx2 = which.start.x; }
	
	if (which.start.y > which.end.y)
	{ maxy2 = which.start.y; miny2 = which.end.y; }
	else
	{ maxy2 = which.end.y; miny2 = which.start.y; }
	
	if (fless(maxx1,minx2) || fless(maxx2, minx1) || fless(maxy1, miny2) || fless(maxy2, miny1))
		return false;

	return fastCrossTest(start.x, start.y, end.x, end.y, 
						 which.start.x, which.start.y, which.end.x, which.end.y, 
						 0, 0);
	//	
	if (fequal(maxx1, minx1)) // this is "here"
	{
		// already know that they share bounding boxes
		// here, they must cross
		if ((maxy2 < maxy1) && (miny2 > miny1))
			return true;
		
		// y = mx + b
		double m = (which.end.y-which.start.y)/(which.end.x-which.start.x);
		double b = which.start.y - m*which.start.x;
		double y = m*here.start.x+b;
		if (fless(y, maxy1) && fgreater(y, miny1)) // on the line
			return true;
		return false;
	}
	if (fequal(maxx2, minx2)) // this is "which"
	{
		// already know that they share bounding boxes
		// here, they must cross
		if ((maxy1 < maxy2) && (miny1 > miny2))
			return true;
		
		// y = mx + b
		double m = (here.end.y-here.start.y)/(here.end.x-here.start.x);
		double b = here.start.y - m*here.start.x;
		double y = m*which.start.x+b;
		if (fless(y, maxy2) && fgreater(y, miny2)) // on the line
			return true;
		return false;
	}
	
	double b1 = (which.end.y-which.start.y)/(which.end.x-which.start.x);// (A)
	double b2 = (here.end.y-here.start.y)/(here.end.x-here.start.x);// (B)
	
	double a1 = which.start.y-b1*which.start.x;
	double a2 = here.start.y-b2*here.start.x;
	
	if (fequal(b1, b2))
		return false;
	double xi = - (a1-a2)/(b1-b2); //(C)
	double yi = a1+b1*xi;
	// these are actual >= but we exempt points
	if ((!fless((which.start.x-xi)*(xi-which.end.x), 0)) &&
		(!fless((here.start.x-xi)*(xi-here.end.x), 0)) &&
		(!fless((which.start.y-yi)*(yi-which.end.y), 0)) &&
		(!fless((here.start.y-yi)*(yi-here.end.y), 0)))
	{
		//printf("lines cross at (%f, %f)\n",xi,yi);
		return true;
	}
	else {
		return false;
		//print "lines do not cross";
	}
	assert(false);
}

// Returns 1 if the lines intersect, otherwise 0. In addition, if the lines 
// intersect the intersection point may be stored in the floats i_x and i_y.
// taken from: http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
//
bool fastCrossTest(float p0_x, float p0_y, float p1_x, float p1_y, 
				   float p2_x, float p2_y, float p3_x, float p3_y, float *i_x, float *i_y)
{
    float s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;
	
    float s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);
	
    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
    {
        // Collision detected
//        if (i_x != NULL)
//            *i_x = p0_x + (t * s1_x);
//        if (i_y != NULL)
//            *i_y = p0_y + (t * s1_y);
        return 1;
    }
	
    return 0; // No collision
}


recColor getColor(GLfloat v, GLfloat vmin, GLfloat vmax, int type)
{
	double dv,vmid;
	recColor c(1.0,1.0,1.0);
  recColor c1,c2,c3;
  double ratio;
	
	if (v < vmin)
		v = vmin;
	if (v > vmax)
		v = vmax;
	dv = vmax - vmin;
	
  switch (type%21) {
		case 0:
			c.r = 1.0f;
			c.b = 1.0f;
			c.g = 1.0f;
			break;
		case 1:
			if (v < (vmin + 0.25 * dv))
			{
        c.r = 0;
        c.g = 4 * (v - vmin) / dv;
				c.b = 1;
			}
			else if (v < (vmin + 0.5 * dv))
			{
        c.r = 0;
				c.g = 1;
        c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
			}
			else if (v < (vmin + 0.75 * dv))
			{
        c.r = 4 * (v - vmin - 0.5 * dv) / dv;
				c.g = 1;
        c.b = 0;
			}
			else {
				c.r = 1;
        c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
        c.b = 0;
			}
			break;
		case 2:
			c.r = (v - vmin) / dv;
			c.g = 0;
			c.b = (vmax - v) / dv;
			break;
		case 3:
			c.r = (v - vmin) / dv;
			c.b = c.r;
			c.g = c.r;
			break;
		case 4:
      if (v < (vmin + dv / 6.0)) {
				c.r = 1; 
				c.g = 6 * (v - vmin) / dv;
				c.b = 0;
      } else if (v < (vmin + 2.0 * dv / 6.0)) {
				c.r = 1 + 6 * (vmin + dv / 6.0 - v) / dv;
				c.g = 1;
				c.b = 0;
      } else if (v < (vmin + 3.0 * dv / 6.0)) {
				c.r = 0;
				c.g = 1;
				c.b = 6 * (v - vmin - 2.0 * dv / 6.0) / dv;
      } else if (v < (vmin + 4.0 * dv / 6.0)) {
				c.r = 0;
				c.g = 1 + 6 * (vmin + 3.0 * dv / 6.0 - v) / dv;
				c.b = 1;
      } else if (v < (vmin + 5.0 * dv / 6.0)) {
				c.r = 6 * (v - vmin - 4.0 * dv / 6.0) / dv;
				c.g = 0;
				c.b = 1;
      } else {
				c.r = 1;
				c.g = 0;
				c.b = 1 + 6 * (vmin + 5.0 * dv / 6.0 - v) / dv;
      }
			break;
		case 5:
      c.r = (v - vmin) / (vmax - vmin);
      c.g = 1;
      c.b = 0;
			break;
		case 6:
      c.r = (v - vmin) / (vmax - vmin);
      c.g = (vmax - v) / (vmax - vmin);
      c.b = c.r;
			break;
		case 7:
      if (v < (vmin + 0.25 * dv)) {
				c.r = 0;
				c.g = 4 * (v - vmin) / dv;
				c.b = 1 - c.g;
      } else if (v < (vmin + 0.5 * dv)) {
				c.r = 4 * (v - vmin - 0.25 * dv) / dv;
				c.g = 1 - c.r;
				c.b = 0;
      } else if (v < (vmin + 0.75 * dv)) {
				c.g = 4 * (v - vmin - 0.5 * dv) / dv;
				c.r = 1 - c.g;
				c.b = 0;
      } else {
				c.r = 0;
				c.b = 4 * (v - vmin - 0.75 * dv) / dv;
				c.g = 1 - c.b;
      }
      break;
		case 8:
      if (v < (vmin + 0.5 * dv)) {
				c.r = 2 * (v - vmin) / dv;
				c.g = c.r;
				c.b = c.r;
      } else {
				c.r = 1 - 2 * (v - vmin - 0.5 * dv) / dv;
				c.g = c.r;
				c.b = c.r;
      }
      break;
		case 9:
      if (v < (vmin + dv / 3)) {
				c.b = 3 * (v - vmin) / dv;
				c.g = 0;
				c.r = 1 - c.b;
      } else if (v < (vmin + 2 * dv / 3)) {
				c.r = 0;
				c.g = 3 * (v - vmin - dv / 3) / dv;
				c.b = 1;
      } else {
				c.r = 3 * (v - vmin - 2 * dv / 3) / dv;
				c.g = 1 - c.r;
				c.b = 1;
      }
      break;
		case 10:
      if (v < (vmin + 0.2 * dv)) {
				c.r = 0;
				c.g = 5 * (v - vmin) / dv;
				c.b = 1;
      } else if (v < (vmin + 0.4 * dv)) {
				c.r = 0;
				c.g = 1;
				c.b = 1 + 5 * (vmin + 0.2 * dv - v) / dv;
      } else if (v < (vmin + 0.6 * dv)) {
				c.r = 5 * (v - vmin - 0.4 * dv) / dv;
				c.g = 1;
				c.b = 0;
      } else if (v < (vmin + 0.8 * dv)) {
				c.r = 1;
				c.g = 1 - 5 * (v - vmin - 0.6 * dv) / dv;
				c.b = 0;
      } else {
				c.r = 1;
				c.g = 5 * (v - vmin - 0.8 * dv) / dv;
				c.b = 5 * (v - vmin - 0.8 * dv) / dv;
      }
      break;
		case 11:
			c1.r = 200 / 255.0; c1.g =  60 / 255.0; c1.b =   0 / 255.0;
			c2.r = 250 / 255.0; c2.g = 160 / 255.0; c2.b = 110 / 255.0;
      c.r = (c2.r - c1.r) * (v - vmin) / dv + c1.r;
      c.g = (c2.g - c1.g) * (v - vmin) / dv + c1.g;
      c.b = (c2.b - c1.b) * (v - vmin) / dv + c1.b;
      break;
		case 12:
			c1.r =  55 / 255.0; c1.g =  55 / 255.0; c1.b =  45 / 255.0;
			c2.r = 200 / 255.0; c2.g =  60 / 255.0; c2.b =   0 / 255.0; 
			//    c2.r = 235 / 255.0; c2.g =  90 / 255.0; c2.b =  30 / 255.0;
			c3.r = 250 / 255.0; c3.g = 160 / 255.0; c3.b = 110 / 255.0;
			ratio = 0.4;
			vmid = vmin + ratio * dv;
			if (v < vmid) {
        c.r = (c2.r - c1.r) * (v - vmin) / (ratio*dv) + c1.r;
        c.g = (c2.g - c1.g) * (v - vmin) / (ratio*dv) + c1.g;
        c.b = (c2.b - c1.b) * (v - vmin) / (ratio*dv) + c1.b;
			} else {
				c.r = (c3.r - c2.r) * (v - vmid) / ((1-ratio)*dv) + c2.r;
				c.g = (c3.g - c2.g) * (v - vmid) / ((1-ratio)*dv) + c2.g;
				c.b = (c3.b - c2.b) * (v - vmid) / ((1-ratio)*dv) + c2.b;
			}
				break;
		case 13:
      c1.r =   0 / 255.0; c1.g = 255 / 255.0; c1.b =   0 / 255.0;
      c2.r = 255 / 255.0; c2.g = 150 / 255.0; c2.b =   0 / 255.0;
      c3.r = 255 / 255.0; c3.g = 250 / 255.0; c3.b = 240 / 255.0;
      ratio = 0.3;
      vmid = vmin + ratio * dv;
      if (v < vmid) {
				c.r = (c2.r - c1.r) * (v - vmin) / (ratio*dv) + c1.r;
				c.g = (c2.g - c1.g) * (v - vmin) / (ratio*dv) + c1.g;
				c.b = (c2.b - c1.b) * (v - vmin) / (ratio*dv) + c1.b;
      } else {
				c.r = (c3.r - c2.r) * (v - vmid) / ((1-ratio)*dv) + c2.r;
				c.g = (c3.g - c2.g) * (v - vmid) / ((1-ratio)*dv) + c2.g;
				c.b = (c3.b - c2.b) * (v - vmid) / ((1-ratio)*dv) + c2.b;
      }
				break;
		case 14:
      c.r = 1;
      c.g = 1 - (v - vmin) / dv;
      c.b = 0;
      break;
		case 15:
      if (v < (vmin + 0.25 * dv)) {
				c.r = 0;
				c.g = 4 * (v - vmin) / dv;
				c.b = 1;
      } else if (v < (vmin + 0.5 * dv)) {
				c.r = 0;
				c.g = 1;
				c.b = 1 - 4 * (v - vmin - 0.25 * dv) / dv;
      } else if (v < (vmin + 0.75 * dv)) {
				c.r = 4 * (v - vmin - 0.5 * dv) / dv;
				c.g = 1;
				c.b = 0;
      } else {
				c.r = 1;
				c.g = 1;
				c.b = 4 * (v - vmin - 0.75 * dv) / dv;
      }
      break;
		case 16:
      if (v < (vmin + 0.5 * dv)) {
				c.r = 0.0;
				c.g = 2 * (v - vmin) / dv;
				c.b = 1 - 2 * (v - vmin) / dv;
      } else {
				c.r = 2 * (v - vmin - 0.5 * dv) / dv;
				c.g = 1 - 2 * (v - vmin - 0.5 * dv) / dv;
				c.b = 0.0;
      }
      break;
		case 17:
      if (v < (vmin + 0.5 * dv)) {
				c.r = 1.0;
				c.g = 1 - 2 * (v - vmin) / dv;
				c.b = 2 * (v - vmin) / dv;
      } else {
				c.r = 1 - 2 * (v - vmin - 0.5 * dv) / dv;
				c.g = 2 * (v - vmin - 0.5 * dv) / dv;
				c.b = 1.0;
      }
      break;
		case 18:
      c.r = 0;
      c.g = (v - vmin) / (vmax - vmin);
      c.b = 1;
      break;
		case 19:
      c.r = (v - vmin) / (vmax - vmin);
      c.g = c.r;
      c.b = 1;
      break;
		case 20:
      c1.r =   0 / 255.0; c1.g = 160 / 255.0; c1.b =   0 / 255.0;
      c2.r = 180 / 255.0; c2.g = 220 / 255.0; c2.b =   0 / 255.0;
      c3.r = 250 / 255.0; c3.g = 220 / 255.0; c3.b = 170 / 255.0;
      ratio = 0.3;
      vmid = vmin + ratio * dv;
      if (v < vmid) {
				c.r = (c2.r - c1.r) * (v - vmin) / (ratio*dv) + c1.r;
				c.g = (c2.g - c1.g) * (v - vmin) / (ratio*dv) + c1.g;
				c.b = (c2.b - c1.b) * (v - vmin) / (ratio*dv) + c1.b;
      } else {
				c.r = (c3.r - c2.r) * (v - vmid) / ((1-ratio)*dv) + c2.r;
				c.g = (c3.g - c2.g) * (v - vmid) / ((1-ratio)*dv) + c2.g;
				c.b = (c3.b - c2.b) * (v - vmid) / ((1-ratio)*dv) + c2.b;
      }
				break;
  }
  return(c);
}

void DrawPyramid(GLfloat x, GLfloat y, GLfloat z, GLfloat height, GLfloat width)
{
	glBegin(GL_TRIANGLES);
	//	glNormal3f(ROOT2D2, -ROOT2D2, 0);
	glNormal3f(-ROOT2D2, ROOT2D2, 0);
	glVertex3f(x, y, z-height);
	glVertex3f(x-width, y-width, z);
	glVertex3f(x-width, y+width, z);

	glNormal3f(0, ROOT2D2, ROOT2D2);
	glVertex3f(x, y, z-height);
	glVertex3f(x-width, y+width, z);
	glVertex3f(x+width, y+width, z);

	glNormal3f(ROOT2D2, ROOT2D2, 0);
	glVertex3f(x, y, z-height);
	glVertex3f(x+width, y+width, z);
	glVertex3f(x+width, y-width, z);

	glNormal3f(0, ROOT2D2, -ROOT2D2);
	glVertex3f(x, y, z-height);
	glVertex3f(x+width, y-width, z);
	glVertex3f(x-width, y-width, z);
	glEnd();
}

// interleaved vertex array for glDrawElements() & glDrawRangeElements() ======
// All vertex attributes (position, normal, color) are packed together as a
// struct or set, for example, ((V,N,C), (V,N,C), (V,N,C),...).
// It is called an array of struct, and provides better memory locality.
GLfloat vertices3[] = { 1, 1, 1,   0, 0, 1,   1, 1, 1,              // v0 (front)
	-1, 1, 1,   0, 0, 1,   1, 1, 0,              // v1
	-1,-1, 1,   0, 0, 1,   1, 0, 0,              // v2
	1,-1, 1,   0, 0, 1,   1, 0, 1,              // v3
	
	1, 1, 1,   1, 0, 0,   1, 1, 1,              // v0 (right)
	1,-1, 1,   1, 0, 0,   1, 0, 1,              // v3
	1,-1,-1,   1, 0, 0,   0, 0, 1,              // v4
	1, 1,-1,   1, 0, 0,   0, 1, 1,              // v5
	
	1, 1, 1,   0, 1, 0,   1, 1, 1,              // v0 (top)
	1, 1,-1,   0, 1, 0,   0, 1, 1,              // v5
	-1, 1,-1,   0, 1, 0,   0, 1, 0,              // v6
	-1, 1, 1,   0, 1, 0,   1, 1, 0,              // v1
	
	-1, 1, 1,  -1, 0, 0,   1, 1, 0,              // v1 (left)
	-1, 1,-1,  -1, 0, 0,   0, 1, 0,              // v6
	-1,-1,-1,  -1, 0, 0,   0, 0, 0,              // v7
	-1,-1, 1,  -1, 0, 0,   1, 0, 0,              // v2
	
	-1,-1,-1,   0,-1, 0,   0, 0, 0,              // v7 (bottom)
	1,-1,-1,   0,-1, 0,   0, 0, 1,              // v4
	1,-1, 1,   0,-1, 0,   1, 0, 1,              // v3
	-1,-1, 1,   0,-1, 0,   1, 0, 0,              // v2
	
	1,-1,-1,   0, 0,-1,   0, 0, 1,              // v4 (back)
	-1,-1,-1,   0, 0,-1,   0, 0, 0,              // v7
	-1, 1,-1,   0, 0,-1,   0, 1, 0,              // v6
	1, 1,-1,   0, 0,-1,   0, 1, 1 };            // v5

GLubyte indices[]  = { 0, 1, 2,   2, 3, 0,      // front
	4, 5, 6,   6, 7, 4,      // right
	8, 9,10,  10,11, 8,      // top
	12,13,14,  14,15,12,      // left
	16,17,18,  18,19,16,      // bottom
	20,21,22,  22,23,20 };    // back

void DrawBoxFrame(GLfloat xx, GLfloat yy, GLfloat zz, GLfloat rad)
{
	GLfloat vertices[] =
	{-1, -1, -1, -1, -1,  1,
	 -1, -1, -1, -1,  1, -1,
	 -1, -1, -1,  1, -1, -1,
	  1,  1,  1,  1,  1, -1,
	  1,  1,  1,  1, -1,  1,
	  1,  1,  1, -1,  1,  1,
	  1,  1, -1, -1,  1, -1,
	  1,  1, -1,  1, -1, -1,
	 -1,  1,  1, -1, -1,  1,
	 -1,  1,  1, -1,  1, -1,
	  1, -1, -1,  1, -1,  1,
	 -1, -1,  1,  1, -1,  1
	 }; // 36 of vertex coords

	// activate and specify pointer to vertex array
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, vertices);
	
	glPushMatrix();
	glTranslatef(xx, yy, zz);                // move to bottom-left
	glScalef(rad, rad, rad);
	glDrawArrays(GL_LINES, 0, 24);
	
	glPopMatrix();
	
	// deactivate vertex arrays after drawing
	glDisableClientState(GL_VERTEX_ARRAY);

}

void DrawBox(GLfloat xx, GLfloat yy, GLfloat zz, GLfloat rad)
{
	glEnable(GL_NORMALIZE);
	// enable and specify pointers to vertex arrays
	glEnableClientState(GL_NORMAL_ARRAY);
	//glEnableClientState(GL_COLOR_ARRAY);
	glEnableClientState(GL_VERTEX_ARRAY);
	glNormalPointer(GL_FLOAT, 9 * sizeof(GLfloat), vertices3 + 3);
	//glColorPointer(3, GL_FLOAT, 9 * sizeof(GLfloat), vertices3 + 6);
	glVertexPointer(3, GL_FLOAT, 9 * sizeof(GLfloat), vertices3);
	
	glPushMatrix();
	glTranslatef(xx, yy, zz);                // move to bottom-left
	glScalef(rad, rad, rad);
	glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_BYTE, indices);
	
	glPopMatrix();
	
	glDisableClientState(GL_VERTEX_ARRAY);  // disable vertex arrays
	//glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisable(GL_NORMALIZE);
}
//
//void DrawBox(GLfloat xx, GLfloat yy, GLfloat zz, GLfloat rad)
//{
//	glBegin(GL_QUAD_STRIP);
//	glVertex3f(xx-rad, yy-rad, zz-rad);
//	glVertex3f(xx-rad, yy+rad, zz-rad);
//
//	glVertex3f(xx+rad, yy-rad, zz-rad);
//	glVertex3f(xx+rad, yy+rad, zz-rad);
//
//	glVertex3f(xx+rad, yy-rad, zz+rad);
//	glVertex3f(xx+rad, yy+rad, zz+rad);
//
//	glVertex3f(xx-rad, yy-rad, zz+rad);
//	glVertex3f(xx-rad, yy+rad, zz+rad);
//
//	glVertex3f(xx-rad, yy-rad, zz-rad);
//	glVertex3f(xx-rad, yy+rad, zz-rad);
//
//	glEnd();
//
//	glBegin(GL_QUADS);
//	glVertex3f(xx-rad, yy+rad, zz-rad);
//	glVertex3f(xx+rad, yy+rad, zz-rad);
//	glVertex3f(xx+rad, yy+rad, zz+rad);
//	glVertex3f(xx-rad, yy+rad, zz+rad);
//	glEnd();
//
//	glBegin(GL_QUADS);
//	glVertex3f(xx-rad, yy-rad, zz-rad);
//	glVertex3f(xx+rad, yy-rad, zz-rad);
//	glVertex3f(xx+rad, yy-rad, zz+rad);
//	glVertex3f(xx-rad, yy-rad, zz+rad);
//	glEnd();
//}

void DrawSphere(GLdouble _x, GLdouble _y, GLdouble _z, GLdouble tRadius)
{
	static std::vector<double> px_cache;
	static std::vector<double> py_cache;
	static std::vector<double> pz_cache;
	glEnable(GL_LIGHTING);
	
	glTranslatef(_x, _y, _z);

	int i,j;
	int n = 16; // precision
	double theta1,theta2,theta3;
	point3d e,p;//,c(0, 0, 0);
	
	if (tRadius < 0) tRadius = -tRadius;
	if (n < 0) n = -n;
	if (n < 4 || tRadius <= 0)
	{
		glBegin(GL_POINTS);
		glVertex3f(0, 0, 0);
		glEnd();
	}
	else if (px_cache.size() == 0)
	{
		for (j=n/4;j<n/2;j++)
		{
			theta1 = j * TWOPI / n - PID2;
			theta2 = (j + 1) * TWOPI / n - PID2;
			
			glBegin(GL_QUAD_STRIP);
			//glBegin(GL_POINTS);
			//glBegin(GL_TRIANGLE_STRIP);
			//glBegin(GL_LINE_STRIP);
			for (i=0;i<=n;i++)
			{
				theta3 = i * TWOPI / n;
				
				e.x = cos(theta2) * cos(theta3);
				e.y = cos(theta2) * sin(theta3);
				e.z = sin(theta2);

				px_cache.push_back(e.x);
				py_cache.push_back(e.y);
				pz_cache.push_back(e.z);
				
				p.x = tRadius * e.x;
				p.y = tRadius * e.y;
				p.z = - tRadius * e.z;
				
				glNormal3f(-e.x,-e.y,e.z);
				//glTexCoord2f(i/(double)n,2*(j+1)/(double)n);
				glVertex3f(p.x,p.y,p.z);
				
				e.x = cos(theta1) * cos(theta3);
				e.y = cos(theta1) * sin(theta3);
				e.z = sin(theta1);

				px_cache.push_back(e.x);
				py_cache.push_back(e.y);
				pz_cache.push_back(e.z);

				p.x = tRadius * e.x;
				p.y = tRadius * e.y;
				p.z = - tRadius * e.z;
				
				glNormal3f(-e.x,-e.y,e.z);
				//glTexCoord2f(i/(double)n,2*j/(double)n);
				glVertex3f(p.x,p.y,p.z);
			}
			glEnd();
		}
	}
	else {
		int which = 0;
		for (j=n/4;j<n/2;j++)
		{
			glBegin(GL_QUAD_STRIP);
			for (i=0;i<=n;i++)
			{
				glNormal3d(-px_cache[which], -py_cache[which], pz_cache[which]);
				glVertex3d(tRadius*px_cache[which], tRadius*py_cache[which], -tRadius*pz_cache[which]);
				which++;
				glNormal3d(-px_cache[which], -py_cache[which], pz_cache[which]);
				glVertex3d(tRadius*px_cache[which], tRadius*py_cache[which], -tRadius*pz_cache[which]);
				which++;
			}
			glEnd();
		}
	}
	glTranslatef(-_x, -_y, -_z);
}	
