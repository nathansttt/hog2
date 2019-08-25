//
//  Colors.h
//  hog2
//
//  Created by Nathan Sturtevant on 7/29/17.
//  Copyright © 2017 NS Software. All rights reserved.
//

#ifndef Colors_h
#define Colors_h

#include <string>

/**
 * A color; r/g/b are between 0...1
 */
class rgbColor {
public:
	rgbColor() {}
	rgbColor(float rr, float gg, float bb) :r(rr), g(gg), b(bb) {}
	rgbColor &operator=(const rgbColor & ) = default;
	void mix(const rgbColor &c, float perc)
	{
		r = (1-perc)*r+c.r*perc;
		g = (1-perc)*g+c.g*perc;
		b = (1-perc)*b+c.b*perc;
	}
	std::string hex() const
	{ char tmp[255];
		sprintf(tmp, "#%X%X%X%X%X%X",
				(int(r*255))/16, (int(r*255))%16,
				(int(g*255))/16, (int(g*255))%16,
				(int(b*255))/16, (int(b*255))%16);
		return tmp;
	}
	void hex(const char *str) // get color from #RRGGBB format
	{
		if (str[0] != '#')
		{
			printf("Conversion failed\n");
			return;
		}
		r = (unhexdigit(str[1])*16+unhexdigit(str[2]))/255.0;
		g = (unhexdigit(str[3])*16+unhexdigit(str[4]))/255.0;
		b = (unhexdigit(str[5])*16+unhexdigit(str[6]))/255.0;
	}
	float r,g,b;
private:
	int unhexdigit(char c)
	{
		switch (c)
		{
			case '0': return 0;
			case '1': return 1;
			case '2': return 2;
			case '3': return 3;
			case '4': return 4;
			case '5': return 5;
			case '6': return 6;
			case '7': return 7;
			case '8': return 8;
			case '9': return 9;
			case 'A': return 10;
			case 'B': return 11;
			case 'C': return 12;
			case 'D': return 13;
			case 'E': return 14;
			case 'F': return 15;
		}
		return 0;
	}
};

bool operator==(const rgbColor &r1, const rgbColor &r2);
bool operator!=(const rgbColor &r1, const rgbColor &r2);

namespace Colors
{
	/**
	 * Given min/max values, get a color from a color schema
	 */
	rgbColor GetColor(float v, float vmin, float vmax, int type);

	
	const rgbColor black  = {0.0,0.0,0.0};
	const rgbColor white  = {1.0,1.0,1.0};
	const rgbColor gray   = {0.5,0.5,0.5};
	const rgbColor bluegray   = {0.4,0.5,0.6};
	const rgbColor darkgray= {0.25,0.25,0.25};
	const rgbColor darkbluegray= {0.15,0.25,0.35};
	const rgbColor lightgray={0.75,0.75,0.75};
	const rgbColor lightbluegray={0.65,0.75,0.85};

	const rgbColor red    = {1.0,0.0,0.0}; // red
	const rgbColor darkred= {0.5,0.0,0.0}; // red
	const rgbColor lightred= {1.0,0.5,0.5}; // red
	const rgbColor lighterred= {1.0,0.75,0.75}; // red
	const rgbColor brown   = {0.5,0.25,0.0}; // brown

	const rgbColor green  = {0.0,1.0,0.0}; // green
	const rgbColor darkgreen= {0.0,0.5,0.0}; // green
	const rgbColor lightgreen= {0.5,1.0,0.5}; // green

	const rgbColor bluegreen   = {0.0f,0.65f,0.5f}; // blue

	
	const rgbColor blue   = {0.0,0.0,1.0}; // blue
	const rgbColor darkblue   = {0.0,0.0,0.5}; // blue
	const rgbColor lightblue   = {0.5,0.5,1.0}; // blue
	const rgbColor lighterblue   = {0.75,0.75,1.0}; // blue

	const rgbColor lightyellow = {1.0,1.0,0.5}; // yellow
	const rgbColor yellow = {1.0,1.0,0.0}; // yellow
	const rgbColor darkyellow = {0.5,0.5,0.0}; // yellow
	const rgbColor purple = {1.0,0.0,1.0}; // purple
	const rgbColor cyan   = {0.0,1.0,1.0}; // cyan
	
	const rgbColor orange = {1.0,0.5,0.0}; // orange
	const rgbColor pink   = {1.0,0.0,0.5}; // pink

	const rgbColor cb0    = {0, 0, 0}; // black
	const rgbColor cb1    = {0, 114.0f/255.0f, 178/255.0f};   // blue
	const rgbColor cb2    = {204/255.0f, 121/255.0f, 167/255.0f}; // reddish purple
	const rgbColor cb3    = {230/255.0f, 159/255.0f, 0};   // orange
	const rgbColor cb4    = {86/255.0f, 180/255.0f, 233/255.0f};  // sky blue
	const rgbColor cb5    = {213/255.0f, 94/255.0f, 0};   // vermillion
}


#endif /* Colors_h */
