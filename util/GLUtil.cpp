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

#include "glUtil.h"
#include <math.h>

#ifdef NO_OPENGL
#include "gl.cpp"
#include "glut.cpp"
#endif

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

recColor getColor(GLfloat v, GLfloat vmin, GLfloat vmax, int type)
{
	double dv,vmid;
  recColor c = {1.0,1.0,1.0};
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

void drawPyramid(GLfloat x, GLfloat y, GLfloat z, GLfloat height, GLfloat width)
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

void drawBox(GLfloat xx, GLfloat yy, GLfloat zz, GLfloat rad)
{
	glBegin(GL_QUAD_STRIP);
	glVertex3f(xx-rad, yy-rad, zz-rad);
	glVertex3f(xx-rad, yy+rad, zz-rad);

	glVertex3f(xx+rad, yy-rad, zz-rad);
	glVertex3f(xx+rad, yy+rad, zz-rad);

	glVertex3f(xx+rad, yy-rad, zz+rad);
	glVertex3f(xx+rad, yy+rad, zz+rad);

	glVertex3f(xx-rad, yy-rad, zz+rad);
	glVertex3f(xx-rad, yy+rad, zz+rad);

	glVertex3f(xx-rad, yy-rad, zz-rad);
	glVertex3f(xx-rad, yy+rad, zz-rad);

	glEnd();

	glBegin(GL_QUADS);
	glVertex3f(xx-rad, yy+rad, zz-rad);
	glVertex3f(xx+rad, yy+rad, zz-rad);
	glVertex3f(xx+rad, yy+rad, zz+rad);
	glVertex3f(xx-rad, yy+rad, zz+rad);
	glEnd();

	glBegin(GL_QUADS);
	glVertex3f(xx-rad, yy-rad, zz-rad);
	glVertex3f(xx+rad, yy-rad, zz-rad);
	glVertex3f(xx+rad, yy-rad, zz+rad);
	glVertex3f(xx-rad, yy-rad, zz+rad);
	glEnd();
}
