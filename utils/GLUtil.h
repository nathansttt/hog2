/*
 * $Id: glUtil.h,v 1.6 2006/09/18 06:20:15 nathanst Exp $
 *
 *  glUtil.h
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

#include "FPUtil.h"
#include <ostream>

#ifdef __APPLE__
#include "TargetConditionals.h"
#endif

#ifdef NO_OPENGL
#include "gl.h"
#include "glut.h"
#else

//#ifdef TARGET_OS_IPHONE
//#include <OpenGLES/ES1/gl.h>
//#include <OpenGLES/ES1/glext.h>
//#define GLdouble GLfloat
//#else

#ifdef OS_MAC
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#include <AGL/agl.h>
#else

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

//#endif
#endif

#ifndef GLUTIL_H
#define GLUTIL_H

//#pragma mark -
//#pragma mark OpenGL structures:

static const double ONE = 1.0;
static const double TWO = 2.0;
static const double ROOT_TWO = 1.414213562;//1.5f?
static const double ONE_OVER_ROOT_TWO = 1.0/ROOT_TWO;//0.707106781f;
static const double ROOT_THREE = 1.732050808;

static const double TWOPI = 6.283185307179586476925287;
static const double PI = 3.141592653589793238462643;
static const double PID180 = PI/180; // radian conversion
static const double PID2 = PI/2; // 90degree
static const double PID3 = PI/3; // 60degree
static const double PID4 = PI/4; // 45degree
static const double PID6 = PI/6; // 30degree
static const double PID8 = PI/8; // 22.25degree
static const double ROOT2D2 = 0.7071067811865475;

/**
 * A generic vector (essentially the same as a point, but offers normalization)
 */
class recVec {
public:
	recVec() { x = y = z = 0; }
	recVec(GLdouble x_i, GLdouble y_i, GLdouble z_i) :x(x_i), y(y_i), z(z_i) {}
	void normalise();
	double length() const;
	recVec GetNormal(recVec v)
	{
		recVec n;
			
		v.normalise();
		this->normalise();
			
		n.x = this->y * v.z - this->z * v.y;
		n.y = this->z * v.x - this->x * v.z;
		n.z = this->x * v.y - this->y * v.x;
		n.normalise();

		return n;
	}
	GLdouble x,y,z;
	recVec &operator+=(const recVec &v)
	{ x += v.x; y += v.y; z += v.z; return *this; }
	recVec &operator-=(const recVec &v)
	{ x -= v.x; y -= v.y; z -= v.z; return *this; }
	recVec operator-(const recVec &v)
	{ recVec n = *this; n-=v; return n; }
	recVec &operator*=(GLdouble val)
	{ x *= val; y *= val; z *= val; return *this; }
	recVec operator*(const recVec &val) const
	{
		recVec result;
		result.x = this->y*val.z - this->z*val.y;
		result.y = this->z*val.x - this->x*val.z;
		result.z = this->x*val.y - this->y*val.x;
		result.normalise();
		return result;
	}
};


bool operator==(const recVec &l1, const recVec &l2);

std::ostream& operator<<(std::ostream &out, const recVec &loc);


/**
 * A color; r/g/b are between 0...1
 */
class recColor {
public:
	recColor() {}
	recColor(GLfloat rr, GLfloat gg, GLfloat bb) :r(rr), g(gg), b(bb) {}
	GLfloat r,g,b;
};

/**
 * A point in 3d space. (OpenGL)
 */
class point3d {
public:
	point3d() {}
	point3d(GLfloat a, GLfloat b, GLfloat c) :x(a), y(b), z(c) {}
	GLfloat x, y, z;
	
	point3d operator+(const point3d &v) const
	{ point3d p(*this); p+=v; return p; }
	point3d operator-(const point3d &v) const
	{ point3d p(*this); p-=v; return p; }
	point3d &operator+=(const point3d &v)
	{ x += v.x; y += v.y; z += v.z; return *this; }
	point3d &operator-=(const point3d &v)
	{ x -= v.x; y -= v.y; z -= v.z; return *this; }
	point3d &operator+=(const int v)
	{ x += v; y += v; z += v; return *this; }
	point3d &operator-=(const int v)
	{ x -= v; y -= v; z -= v; return *this; }
	point3d &operator*=(const int v)
	{ x *= v; y *= v; z *= v; return *this; }
	point3d &operator/=(const int v)
	{ x /= v; y /= v; z /= v; return *this; }

};

class line2d {
public:
	line2d() {}
	line2d(recVec a, recVec b) :start(a), end(b) {}
	bool crosses(line2d which) const;
	recVec start;
	recVec end;
};

/**
* Given min/max values, get a color from a color schema
 */
recColor getColor(GLfloat v, GLfloat vmin, GLfloat vmax, int type);

/** Draw a pyramid with the tip at the given location, given height, and 
* width from center to edge as width.
*/
void DrawPyramid(GLfloat x, GLfloat y, GLfloat z, GLfloat height, GLfloat width);
void DrawBox(GLfloat x, GLfloat y, GLfloat z, GLfloat radius);
void DrawBoxFrame(GLfloat xx, GLfloat yy, GLfloat zz, GLfloat rad);
void DrawSphere(GLdouble _x, GLdouble _y, GLdouble _z, GLdouble tRadius);
void DrawSquare(GLdouble _x, GLdouble _y, GLdouble _z, GLdouble tRadius);
void DrawCylinder(GLfloat xx, GLfloat yy, GLfloat zz, GLfloat innerRad, GLfloat outerRad, GLfloat height);
void OutlineRect(GLdouble left, GLdouble top, GLdouble right, GLdouble bottom, double zz);

void DrawText(double x, double y, double z, double scale, const char *res);
void DrawTextCentered(double x, double y, double z, double scale, const char *res);


namespace colors
{
	const recColor black  = {0.0,0.0,0.0};
	const recColor white  = {1.0,1.0,1.0}; // white
	const recColor gray   = {0.5,0.5,0.5}; // green
	const recColor darkgray= {0.25,0.25,0.25}; // green
	const recColor lightgray={0.75,0.75,0.75}; // green

	const recColor red    = {1.0,0.0,0.0}; // red
	const recColor darkred= {0.5,0.0,0.0}; // red
	const recColor lightred= {1.0,0.5,0.5}; // red

	const recColor green  = {0.0,1.0,0.0}; // green
	const recColor darkgreen= {0.0,0.5,0.0}; // green
	const recColor lightgreen= {0.5,1.0,0.5}; // green

	const recColor blue   = {0.0,0.0,1.0}; // blue
	const recColor darkblue   = {0.0,0.0,0.5}; // blue
	const recColor lightblue   = {0.5,0.5,1.0}; // blue

	const recColor yellow = {1.0,1.0,0.0}; // yellow
	const recColor purple = {1.0,0.0,1.0}; // purple
	const recColor cyan   = {0.0,1.0,1.0}; // cyan

	const recColor orange = {1.0,0.5,0.0}; // orange
	const recColor pink   = {1.0,0.0,0.5}; // pink
}

//class OpenGLDrawable {
//public:
//	virtual ~OpenGLDrawable() {}
//	virtual void OpenGLDraw(unsigned long windowID) = 0;
//};


#endif
