/*
 *  Plot2D.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/18/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef PLOT2D_H
#define PLOT2D_H

#include <vector>
#include "GLUtil.h"
#include "FPUtil.h"

namespace Plotting {

	enum tPlotType {
		kLinePlot, // draw from point to point
		kImpulsePlot // draw impulses at at y-axis value
	};

	class Line {
	public:
		Line(char *label, tPlotType = kLinePlot);
		void AddPoint(double x, double y);
		void AddPoint(double x);
		const char *GetLabel() { return name; }
		
		void ClearColor();
		void SetColor(double, double, double);
		void OpenGLDraw() const;
		void SetHidden( bool val) { hidden = val; }
		bool IsHidden() { return hidden; }

		inline void Smooth(unsigned int windowSize);
		inline bool GetChanged() { return changedLine; }
		inline void SetChange(bool val) { changedLine = val; }
		inline double GetMaxX() { return xMax; }
		inline double GetMaxY() { return yMax; }
		inline double GetMinX() { return xMin; }
		inline double GetMinY() { return yMin; }
		
		double DistanceToLine(double xp, double yp);
		double VerticalDistanceToLine(double xp, double yp);
	private:
		bool changedLine;
		tPlotType plotType;
		double xMin, xMax, yMin, yMax;
		double r, g, b;
		std::vector<double> x;
		std::vector<double> y;
		bool hidden;
		char name[1024];
	};

	class Plot2D {
	public:
		Plot2D();
		void AddLine(Line *);
//		void SetCurrMouse(double, double, Rect &winRect);
//		void Recenter(double, double, Rect &rectPort);
		void OffsetCurrMouse(double deltaX, double deltaY);
		void SetAxis(double, double, double, double);
		void Zoom(double);
		void ResetAxis();
		void OpenGLDraw() const;
		void SmoothLines();
	private:
			double mouseXLoc, mouseYLoc;
			double dLeft, dRight, dTop, dBottom;
			double xMin, xMax, yMin, yMax;
			bool forceAxis, drawMouse, recomputeBorders;
			int lastSelectedLine;
			std::vector<Line *> lines;
	};

}

#endif
