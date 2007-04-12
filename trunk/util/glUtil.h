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

#ifdef NO_OPENGL
#include "gl.h"
#include "glut.h"
#else
#ifdef OS_MAC
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
//#include <OpenGL/glut.h>
#include <AGL/agl.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
//#include <GL/glut.h>
#endif
#endif

#ifndef GLUTIL_H
#define GLUTIL_H

//#pragma mark -
//#pragma mark OpenGL structures:

static const double TWOPI = 6.283185307179586476925287;
static const double PI = 3.141592653589793238462643;
static const double PID2 = PI/2;
static const double ROOT2D2 = 0.7071067811865475;


/**
 * A generic vector (essentially the same as a point, but offers normalization)
 */
class recVec {
public:
	void normalise();
  GLdouble x,y,z;
};

/**
 * A color; r/g/b are between 0...1
 */
class recColor {
public:
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
};

/**
* Given min/max values, get a color from a color schema
 */
recColor getColor(GLfloat v, GLfloat vmin, GLfloat vmax, int type);

/** Draw a pyramid with the tip at the given location, given height, and 
* width from center to edge as width.
*/
void drawPyramid(GLfloat x, GLfloat y, GLfloat z, GLfloat height, GLfloat width);
void drawBox(GLfloat x, GLfloat y, GLfloat z, GLfloat radius);

#endif
