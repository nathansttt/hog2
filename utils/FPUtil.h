/*
 *  $Id: fpUtil.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 09/18/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef fpUtil_H
#define fpUtil_H

#include "float.h"
#include <limits>

// Somehow DBL_MAX is not defined under Linux?
//#ifndef OS_MAC
//#define DBL_MAX std::numeric_limits<double>::max()//1.79769313486231500e+308;    // DBL_MAX for non Mac OS
//#define DBL_MIN std::numeric_limits<double>::min()// DBL_MIN for non Mac OS
//#define MAXFLOAT std::numeric_limits<float>::max()
//#endif

// Floating point comparisons 
static const double TOLERANCE = 0.000001;    // floating point tolerance

inline bool fless(double a, double b) { return (a < b - TOLERANCE); }
inline bool fgreater(double a, double b) { return (a > b + TOLERANCE); }
inline bool flesseq(double a, double b) { return !fgreater(a, b); }
inline bool fgreatereq(double a, double b) { return !fless(a, b); }
inline bool fequal(double a, double b)
{ return (a >= b - TOLERANCE) && (a <= b+TOLERANCE); }

inline double min(double a, double b) { return fless(a, b)?a:b; }
inline double max(double a, double b) { return fless(a, b)?b:a; }

#endif
