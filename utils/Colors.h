//
//  Colors.h
//  hog2
//
//  Created by Nathan Sturtevant on 7/29/17.
//  Copyright © 2017 NS Software. All rights reserved.
//

#ifndef Colors_h
#define Colors_h

#include <stdio.h>

/**
 * A color; r/g/b are between 0...1
 */
class rgbColor {
public:
	rgbColor() {}
	rgbColor(float rr, float gg, float bb) :r(rr), g(gg), b(bb) {}
	float r,g,b;
};


namespace Colors
{
	/**
	 * Given min/max values, get a color from a color schema
	 */
	rgbColor GetColor(float v, float vmin, float vmax, int type);

	
	const rgbColor black  = {0.0,0.0,0.0};
	const rgbColor white  = {1.0,1.0,1.0}; // white
	const rgbColor gray   = {0.5,0.5,0.5}; // green
	const rgbColor darkgray= {0.25,0.25,0.25}; // green
	const rgbColor lightgray={0.75,0.75,0.75}; // green
	
	const rgbColor red    = {1.0,0.0,0.0}; // red
	const rgbColor darkred= {0.5,0.0,0.0}; // red
	const rgbColor lightred= {1.0,0.5,0.5}; // red
	
	const rgbColor green  = {0.0,1.0,0.0}; // green
	const rgbColor darkgreen= {0.0,0.5,0.0}; // green
	const rgbColor lightgreen= {0.5,1.0,0.5}; // green

	const rgbColor bluegreen   = {0.0,0.65,0.5}; // blue

	
	const rgbColor blue   = {0.0,0.0,1.0}; // blue
	const rgbColor darkblue   = {0.0,0.0,0.5}; // blue
	const rgbColor lightblue   = {0.5,0.5,1.0}; // blue
	const rgbColor lighterblue   = {0.25,0.25,1.0}; // blue

	const rgbColor yellow = {1.0,1.0,0.0}; // yellow
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
