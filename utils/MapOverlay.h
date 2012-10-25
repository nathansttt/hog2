/*
 *  mapOverlay.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 3/21/06.
 *  Copyright 2006 Nathan Sturtevant. All rights reserved.
 *
 */

#include <vector>
#include "Map.h"
#include <float.h>

#ifndef MAPOVERLAY_H
#define MAPOVERLAY_H

class MapOverlay {
public:
	MapOverlay(Map *m);
	Map *GetMap() { return m; }
	void Clear();
	
	void SetOverlayValue(int x, int y, double value);
	double GetOverlayValue(int x, int y);
	void OpenGLDraw() const;
	void SetTransparentValue(double v) { ignoreVal = v; }
	void SetColorMap(int val) { colorMap = val; }
	int GetColorMap() { return colorMap; }
	void IncreaseColorMap() { colorMap++; }
	void DecreaseColorMap() { colorMap--; }
	double GetMaxValue() { return maxVal; }
	double GetMinValue() { return minVal; }
private:
	void resetValues();
	Map *m;
	std::vector<double> values;
	double maxVal, minVal;
	double ignoreVal;
	int colorMap;
	mutable GLuint displayList;
};

#endif
