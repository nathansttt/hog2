/*
 *  $Id: glUtil.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/8/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "FPUtil.h"
#include <ostream>
#include <iostream>
#include <sstream>
#include <iomanip>

// #ifdef __APPLE__
// #include "TargetConditionals.h"
// #endif

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
//#include <OpenGL/glu.h>
//#include <GLUT/glut.h>
//#include <AGL/agl.h>
#else

#include <GL/gl.h>
//#include <GL/glu.h>
//#include <GL/glut.h>
#endif

//#endif
#endif

#ifndef GLUTIL_H
#define GLUTIL_H

class point3d;

#include "Colors.h"
#include "Graphics.h"

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

#define point3d Graphics::point
///**
// * A point in 3d space. (OpenGL)
// */
//class point3d {
//public:
//	point3d() {}
//	point3d(Graphics::point p):x(p.x), y(p.y), z(p.z) {}
//	point3d(GLfloat a, GLfloat b, GLfloat c=0) :x(a), y(b), z(c) {}
//	GLfloat x, y, z;
//	
//	point3d operator+(const point3d &v) const
//	{ point3d p(*this); p+=v; return p; }
//	point3d operator-(const point3d &v) const
//	{ point3d p(*this); p-=v; return p; }
//	point3d &operator+=(const point3d &v)
//	{ x += v.x; y += v.y; z += v.z; return *this; }
//	point3d &operator-=(const point3d &v)
//	{ x -= v.x; y -= v.y; z -= v.z; return *this; }
//	point3d &operator+=(const float v)
//	{ x += v; y += v; z += v; return *this; }
//	point3d &operator-=(const float v)
//	{ x -= v; y -= v; z -= v; return *this; }
//	point3d &operator*=(const float v)
//	{ x *= v; y *= v; z *= v; return *this; }
//	point3d &operator/=(const float v)
//	{ x /= v; y /= v; z /= v; return *this; }
//
//};
//inline std::ostream &operator<<(std::ostream &o, const point3d&r)
//{ o << "(" << r.x << ", " << r.y  << ", " << r.z << ")"; return o; }

class line2d {
public:
	line2d() {}
	line2d(recVec a, recVec b) :start(a), end(b) {}
	bool crosses(line2d which) const;
	recVec start;
	recVec end;
};


/** Draw a pyramid with the tip at the given location, given height, and 
* width from center to edge as width.
*/
void DrawPyramid(GLfloat x, GLfloat y, GLfloat z, GLfloat height, GLfloat width);
void DrawBox(GLfloat x, GLfloat y, GLfloat z, GLfloat radius);
void DrawBoxFrame(GLfloat xx, GLfloat yy, GLfloat zz, GLfloat rad);
void DrawCircle(GLdouble _x, GLdouble _y, GLdouble tRadius, int segments = 32, float rotation = 0);
void FrameCircle(GLdouble _x, GLdouble _y, GLdouble tRadius, GLdouble lineWidth, int segments = 32, float rotation = 0);
void DrawSphere(GLdouble _x, GLdouble _y, GLdouble _z, GLdouble tRadius);
void DrawSquare(GLdouble _x, GLdouble _y, GLdouble _z, GLdouble tRadius);
void DrawCylinder(GLfloat xx, GLfloat yy, GLfloat zz, GLfloat innerRad, GLfloat outerRad, GLfloat height);
void OutlineRect(GLdouble left, GLdouble top, GLdouble right, GLdouble bottom, double zz);

void DrawText(double x, double y, double z, double scale, const char *res);
void DrawTextCentered(double x, double y, double z, double scale, const char *res);

void SetLighting(GLfloat ambientf = 0.2f, GLfloat diffusef = 1.0f, GLfloat specularf = 1.0f);

// Code from: https://stackoverflow.com/questions/16605967/set-precision-of-stdto-string-when-converting-floating-point-values/16606128#16606128
template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
	std::ostringstream out;
	out.precision(n);
	out << std::fixed << a_value;
	return out.str();
}



#endif
