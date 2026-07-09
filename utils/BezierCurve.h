//
//  BezierCurve.h
//  hog2
//
//  Created by Nathan Sturtevant on 6/28/26.
//  Copyright © 2026 NS Software. All rights reserved.
//

#ifndef BezierCurve_h
#define BezierCurve_h

#include <vector>
#include <cstdint>
#include "Graphics.h"

class BezierCurve {
public:
	BezierCurve(const std::vector<Graphics::point> &pts);

	Graphics::point GetPoint(float t) const;
	Graphics::point GetSlope(float t) const;

	static Graphics::point GetPoint(const std::vector<Graphics::point> &pts, float t);
	static Graphics::point GetSlope(const std::vector<Graphics::point> &pts, float t);

private:
	static float bernstein(float t, int i, int n);
	static uint64_t binomial(int n, int k);

	// Pre-computed: coefficients[i] = binomial(N-1, i) * pts[i]
	// for the curve, and for the slope, binomial(N-2, i) * (pts[i+1] - pts[i]) * (N-1)
	std::vector<Graphics::point> curveCoeff;
	std::vector<Graphics::point> slopeCoeff;
	int N; // number of control points
};

#endif /* BezierCurve_h */
