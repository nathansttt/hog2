/*
 * $Id: fpUtil.h,v 1.6 2006/09/18 06:20:15 nathanst Exp $
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef fpUtil_H
#define fpUtil_H

// Somehow DBL_MAX is not defined under Linux?
#ifndef OS_MAC
//static const double DBL_MAX = 1.79769313486231500e+308;    // DBL_MAX for non Mac OS
#endif

// Floating point comparisons 
static const double TOLERANCE = 0.000001;    // floating point tolerance

inline bool fless(double a, double b) { return (a < b - TOLERANCE); }
inline bool fgreater(double a, double b) { return (a > b + TOLERANCE); }
inline bool fequal(double a, double b)
{ return (a > b - TOLERANCE) && (a < b+TOLERANCE); }

#endif
