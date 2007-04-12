/*
 *  mapOverlay.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 3/21/06.
 *  Copyright 2006 Nathan Sturtevant. All rights reserved.
 *
 */

#include <vector>
#include "map.h"
#include <float.h>

#ifndef MAPOVERLAY_H
#define MAPOVERLAY_H

class MapOverlay {
public:
	MapOverlay(Map *m);
	Map *getMap() { return m; }
	void setOverlayValue(int x, int y, double value);
	double getOverlayValue(int x, int y);
	void openGLDraw();
	void setTransparentValue(double v) { ignoreVal = v; }
	void setColorMap(int val) { colorMap = val; }
	int getColorMap() { return colorMap; }
	void increaseColorMap() { colorMap++; }
	void decreaseColorMap() { colorMap--; }
	double getMaxValue() { return maxVal; }
	double getMinValue() { return minVal; }
private:
	void resetValues();
	Map *m;
	std::vector<double> values;
	double maxVal, minVal;
	double ignoreVal;
	int colorMap;
  GLuint displayList;
};

#endif
