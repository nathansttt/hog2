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
#include "FPUtil.h"

namespace Graphics {
	class point;

	enum textAlign {
		textAlignCenter,
		textAlignLeft
	};


	struct point {
		point(point3d p) :x(p.x), y(p.y), z(p.z) {}
		point(float x = 0, float y = 0, float z = 0)
		:x(x), y(y), z(z) {}
		float x, y, z;

		bool operator==(const point &p) const
		{ return fequal(p.x, x) && fequal(p.y, y) && fequal(p.z, z); }

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
	
	struct rect {
		rect() {}
		rect(point center, float rad) : left(center.x-rad), top(center.y-rad), right(center.x+rad), bottom(center.y+rad) {}
		rect(float l, float t, float r, float b)
		:left(l), top(t), right(r), bottom(b) {}
		float left, top, right, bottom;
	};
	
	inline std::ostream &operator<<(std::ostream &o, const rect&r)
	{ o << r.left << ", " << r.top  << ", " << r.right << ", " << r.bottom; return o; }
	
	inline std::ostream &operator<<(std::ostream &o, const point&r)
	{ o << "(" << r.x << ", " << r.y  << ", " << r.z << ")"; return o; }

	bool PointInRect(const point3d &p, const rect &r);
	bool PointInRect(const point &p, const rect &r);

	/*
	 * This class represents an abstract display.
	 *
	 * All hog2 classes do their drawing to the display. This is an abstraction for a true
	 * display. Any actual display then must collect the data from this display and show it,
	 * as appropriate.
	 */
	class Display {
	public:
		Display();
		void StartFrame();
		void EndFrame();
		void StartBackground();
		void EndBackground();
		bool BackgroundNeedsRedraw() const;
		void SetViewport(uint8_t v);
		void SetNumViewports(uint8_t v);
		int GetNumViewports() { return numViewports; }
		void FrameRect(rect r, rgbColor c, float lineWidth);
		void FillRect(rect r, rgbColor c);
		void FrameSquare(point p, float radius, rgbColor c, float lineWidth);
		void FillSquare(point p, float radius, rgbColor c);
		void FrameCircle(rect r, rgbColor c, float lineWidth); // FIXME: Should be a point and a radius!
		void FrameCircle(point r, float radius, rgbColor c, float lineWidth); // FIXME: Should be a point and a radius!
		void FillCircle(rect r, rgbColor c);
		void FillCircle(point p, float radius, rgbColor c);
		void FillNGon(point p, float radius, int sides, float rotation, rgbColor c);
		void FrameNGon(point p, float radius, int sides, float rotation, rgbColor c);

		void DrawLine(point start, point end, float lineWidth, rgbColor c);
		void DrawLineSegments(const std::vector<point> &points, float lineWidth, rgbColor c);
		void DrawArrow(point start, point end, float lineWidth, rgbColor c);
		void DrawText(const char *text, point location, rgbColor c, float height, const char *typeface = 0);
		void DrawText(const char *text, point location, rgbColor c, float height, textAlign align, const char *typeface = 0);

		struct drawInfo {
			rect r;
			rgbColor c;
			float width;
		};
		struct shapeInfo {
			point center;
			rgbColor c;
			float radius;
			int segments;
			float rotate;
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
			std::string typeface;
			textAlign align;
			uint8_t viewport;
		};
		enum tDrawClass
		{
			kFillRectangle,
			kFrameRectangle,
			kFillOval,
			kFrameOval,
			kFillNGon,
			kFrameNGon,
			kLine
		};
		struct data {
			data(shapeInfo d, tDrawClass t, uint8_t view)
			{
				what = t;
				polygon = d;
				viewport = view;
			}
			data(drawInfo d, tDrawClass t, uint8_t view)
			{
				what = t;
				shape = d;
				viewport = view;
			}
			data(lineInfo l, uint8_t view)
			{
				what = kLine;
				line = l;
				viewport = view;
			}
			tDrawClass what;
			union {
				drawInfo shape;
				shapeInfo polygon;
				lineInfo line;
			};
			uint8_t viewport;
		};
		struct segments {
			rgbColor c;
			float size;
			std::vector<point> points;
			uint8_t viewport;
		};
		// These are dynamic items that change from frame to frame
		std::vector<data> drawCommands;
		std::vector<textInfo> text;
		std::vector<segments> lineSegments;
		// These are static items that don't usually change from frame to frame
		std::vector<data> backgroundDrawCommands;
		std::vector<textInfo> backgroundText;
		std::vector<segments> backgroundLineSegments;
		uint64_t backgroundFrame;
		uint64_t foregroundFrame;
	private:
		uint8_t viewport;
		uint8_t numViewports;
		bool drawingBackground;
	};

}

#endif /* Graphics_h */
