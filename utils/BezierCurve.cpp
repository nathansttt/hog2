//
//  BezierCurve.cpp
//  hog2
//
//  Created by Nathan Sturtevant on 6/28/26.
//  Copyright © 2026 NS Software. All rights reserved.
//

#include "BezierCurve.h"
#include <cassert>

uint64_t BezierCurve::binomial(int n, int k)
{
	if (k < 0 || k > n)
		return 0;
	if (k == 0 || k == n)
		return 1;
	if (k > n - k)
		k = n - k;
	uint64_t result = 1;
	for (int i = 0; i < k; i++)
	{
		result *= (n - i);
		result /= (i + 1);
	}
	return result;
}

float BezierCurve::bernstein(float t, int i, int n)
{
	float b = static_cast<float>(binomial(n, i));
	float ti = (i == 0) ? 1.0f : std::pow(t, i);
	float tni = (n - i == 0) ? 1.0f : std::pow(1.0f - t, n - i);
	return b * ti * tni;
}

// Instance methods — pre-compute binomial * control-point products at construction.
BezierCurve::BezierCurve(const std::vector<Graphics::point> &pts)
:N(static_cast<int>(pts.size()))
{
	assert(N >= 2);
	int deg = N - 1;

	curveCoeff.resize(N);
	for (int i = 0; i < N; i++)
	{
		float c = static_cast<float>(binomial(deg, i));
		curveCoeff[i] = pts[i] * c;
	}

	// Derivative of degree-(N-1) Bezier is degree-(N-2) Bezier over
	// forward differences scaled by (N-1).
	int ddeg = deg - 1;
	slopeCoeff.resize(deg);
	for (int i = 0; i < deg; i++)
	{
		float c = static_cast<float>(binomial(ddeg, i)) * static_cast<float>(deg);
		Graphics::point diff = pts[i + 1] - pts[i];
		slopeCoeff[i] = diff * c;
	}
}

Graphics::point BezierCurve::GetPoint(float t) const
{
	int deg = N - 1;
	float s = 1.0f - t;
	Graphics::point result(0, 0, 0);
	float ti = 1.0f;
	float sni = std::pow(s, deg);
	float sInv = (s == 0.0f) ? 0.0f : 1.0f / s;
	for (int i = 0; i <= deg; i++)
	{
		result += curveCoeff[i] * (ti * sni);
		ti *= t;
		sni *= sInv;
	}
	return result;
}

Graphics::point BezierCurve::GetSlope(float t) const
{
	int ddeg = N - 2;
	float s = 1.0f - t;
	Graphics::point result(0, 0, 0);
	float ti = 1.0f;
	float sni = std::pow(s, ddeg);
	float sInv = (s == 0.0f) ? 0.0f : 1.0f / s;
	for (int i = 0; i <= ddeg; i++)
	{
		result += slopeCoeff[i] * (ti * sni);
		ti *= t;
		sni *= sInv;
	}
	return result;
}

// Static methods — compute everything on the fly.
Graphics::point BezierCurve::GetPoint(const std::vector<Graphics::point> &pts, float t)
{
	int deg = static_cast<int>(pts.size()) - 1;
	Graphics::point result(0, 0, 0);
	for (int i = 0; i <= deg; i++)
		result += pts[i] * bernstein(t, i, deg);
	return result;
}

Graphics::point BezierCurve::GetSlope(const std::vector<Graphics::point> &pts, float t)
{
	int deg = static_cast<int>(pts.size()) - 1;
	Graphics::point result(0, 0, 0);
	for (int i = 0; i < deg; i++)
	{
		Graphics::point diff = pts[i + 1] - pts[i];
		result += diff * (static_cast<float>(deg) * bernstein(t, i, deg - 1));
	}
	return result;
}