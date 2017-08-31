//
//  Graphics.h
//  hog2
//
//  Created by Nathan Sturtevant on 7/10/16.
//  Copyright © 2016 NS Software. All rights reserved.
//

#ifndef Graphics_h
#define Graphics_h

#include <vector>
#include <math.h>
#include "GLUtil.h" // TODO: needs to be renamed, if data structures are to be more widely re-used

namespace Graphics {

	struct rect {
		float left, top, right, bottom;
	};

	inline std::ostream &operator<<(std::ostream &o, const rect&r)
	{ o << r.left << ", " << r.top  << ", " << r.right << ", " << r.bottom; return o; }
	
	struct point {
		point(float x = 0, float y = 0, float z = 0)
		:x(x), y(y), z(z) {}
		float x, y, z;

		point &operator*=(float i)
		{ x*= i; y*=i; z*=i; return *this; }
		point &operator+=(const point &i)
		{ x+=i.x; y+=i.y; z+=i.z; return *this; }
		point &operator-=(const point &i)
		{ x-=i.x; y-=i.y; z-=i.z; return *this; }
		point operator*(float i)
		{ point p = *this; p*=i; return p; }
		point operator+(const point &i)
		{ point p = *this; p+=i; return p; }
		point operator-(const point &i)
		{ point p = *this; p-=i; return p; }
		float length() const
		{ return sqrtf(x * x + y * y + z * z); }
		void normalise()
		{
			float length = this->length();
			if (length != 0)
			{
				x /= length; y /= length; z /= length;
			}
			else {
				x = 0; y = 0; z = 0;
			}
		}
		point operator*(const point &val) const
		{
			point result;
			result.x = this->y*val.z - this->z*val.y;
			result.y = this->z*val.x - this->x*val.z;
			result.z = this->x*val.y - this->y*val.x;
			result.normalise();
			return result;
		}
	};
	/*
	 * This class represents an abstract display.
	 *
	 * All hog2 classes do their drawing to the display. This is an abstraction for a true
	 * display. Any actual display then must collect the data from this display and show it,
	 * as appropriate.
	 */
	class Display {
	public:
		void StartFrame();
		void EndFrame();
		
		void FrameRect(rect r, rgbColor c, float lineWidth);
		void FillRect(rect r, rgbColor c);
		void FrameCircle(rect r, rgbColor c, float lineWidth); // FIXME: Should be a point and a radius!
		void FillCircle(rect r, rgbColor c);
		
		void DrawLine(point start, point end, float lineWidth, rgbColor c);
		void DrawArrow(point start, point end, float lineWidth, rgbColor c);
		void DrawText(const char *text, point location, rgbColor c, float height);

//		enum shapeType {
//			kRectangle,
//			kOval,
//			kTriangle // up down left right?
//		};
		struct drawInfo {
			rect r;
			rgbColor c;
			float width;
		};
		struct lineInfo {
			point start, end;
			rgbColor c;
			float width;
			bool arrow;
		};
		struct textInfo {
			std::string s;
			point loc;
			rgbColor c;
			float size;
		};
		enum tDrawClass
		{
			kFillRectangle,
			kFrameRectangle,
			kFillOval,
			kFrameOval,
			kLine
		};
		struct data {
			data(drawInfo d, tDrawClass t)
			{
				what = t;
				shape = d;
			}
			data(lineInfo l)
			{
				what = kLine;
				line = l;
			}
			tDrawClass what;
			union {
				drawInfo shape;
				lineInfo line;
//				textInfo text;
			};
		};
		std::vector<data> drawCommands;
		std::vector<textInfo> text;
		
//		std::vector<drawInfo> framedRects;
//		std::vector<drawInfo> filledRects;
//		std::vector<drawInfo> framedCircles;
//		std::vector<drawInfo> filledCircles;
//		std::vector<lineInfo> lines;
	};

}

#endif /* Graphics_h */
