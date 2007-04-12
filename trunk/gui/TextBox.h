/*
 * $Id: TextBox.h,v 1.3 2006/10/18 23:53:10 nathanst Exp $
 *
 *  TextBox.h
 *  HOG
 *
 *  Created by Nathan Sturtevant on Mon Apr 12 2004.
 *  Copyright (c) Nathan Sturtevant.
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
#include <ctype.h>
#include <stdint.h>

#ifndef TEXTBOX_H
#define TEXTBOX_H

class TextBox {
public:
	TextBox(char *text, int charLine, point3d topLeft, point3d bottomRight, double duration=4, bool deform=true, bool scrolling = false); // start coord (3), velocity(3)
	~TextBox();
  void stepTime(double amount);
	void draw();
	point3d getLocation();
	void destroy();
	void setColor(recColor _myColor) { myColor = _myColor; }
private:
	void drawLine(point3d where, int startx, int starty, int offsetx, int offsety, double scale);
	void drawChar(char c, point3d where, double height);
	uint32_t getBitmap(char c);
	point3d tl, br;
	recColor myColor;
	char *text;
	int charLine;
	bool deform, scrolling;
	double elapsedTime;
	double duration;
	GLuint dList;
};

#endif

