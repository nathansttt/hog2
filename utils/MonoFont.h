/*
 *  $Id: TextBox.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/18/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "GLUtil.h"
#include <ctype.h>
#include <stdint.h>

#ifndef TEXTBOX_H
#define MONOFONT_H

#include "Graphics.h"

class MonoFont {
public:
	MonoFont();
	~MonoFont();
	void DrawText(Graphics::point location, const char *text, float height,
				  Graphics::textAlign align = Graphics::textAlignLeft,
				  Graphics::textBaseline base = Graphics::textBaselineBottom);
private:
	void DrawLine(point3d where, int startx, int starty, int offsetx, int offsety, double scale);
	void DrawChar(char c, point3d where, double height);
	uint32_t GetBitmap(char c);
};
#endif

