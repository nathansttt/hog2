/*
 * $Id: constants.h,v 1.7 2007/02/22 01:12:56 bulitko Exp $
 *
 *  constants.h
 *  HOG
 *
 *  Created by Nathan Sturtevant on 1/3/05.
 *  Copyright 2005 Nathan Sturtevant, University of Alberta. All rights reserved.
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
 *
 */

// All constant declarations can be placed here if they are needed widely.
// If globals are needed, they can be instantiated in constants.cpp
// this avoid any possible inconsistancies between different files/declarations.

#ifndef CONSTANTS_H
#define CONSTANTS_H

/*
// now in fpUtil.h

#pragma mark -
#pragma mark floating point arithmetic functions/constants:

// Floating point comparissons 
static const double TOLERANCE = 0.000001;			// floating point tolerance
inline bool fless(double a, double b) { return (a < b - TOLERANCE); }
inline bool fgreater(double a, double b) { return (a > b + TOLERANCE); }
inline bool fequal(double a, double b) { return (a > b - TOLERANCE) && (a < b+TOLERANCE); }
*/

//#pragma mark -
//#pragma mark bucketing constants

const int distBucketFirst = 10;	// first bucket: (distBucketFirst-delta,distBucketFirst]
const int distBucketLast = 100;	// last bucket: (distBucketLast-delta,distBucketLast]
const int distBucketDelta = 10;	// bucket size
const int distNumBuckets = (distBucketLast - distBucketFirst) / distBucketDelta + 1;

//#pragma mark -
//#pragma mark unit movement constants

enum tDirection {
	kN=0x8, kS=0x4, kE=0x2, kW=0x1, kNW=kN|kW, kNE=kN|kE, 
	kSE=kS|kE, kSW=kS|kW, kStay=0, kTeleport=kSW|kNE
};

enum tObjectType {
	kDisplayOnly, kIncidentalUnit, kWorldObject
};

const int numPrimitiveActions = 8;
const int numActions = 10;
const tDirection possibleDir[numActions] = { kN, kNE, kE, kSE, kS, kSW, kW, kNW, kStay, kTeleport };
const int kStayIndex = 8; // index of kStay

//#pragma mark -
//#pragma mark node/edge labels

/** Definitions for node labels */
enum {
  kAbstractionLevel = 0, // this is a LONG label
  kNumAbstractedNodes = 1, // nodes that we abstract; this is a LONG label
  kParent = 2, // node that abstracts us; this is a LONG label
  kNodeWidth = 3, // the maximum size object that can completely traverse this node; this is a LONG label
  kTemporaryLabel = 4, // for any temporary usage; this label can be LONG or FLOATING point
  kXCoordinate = 5, // cache for opengl drawing; this is a FLOATING POINT label
  kYCoordinate = 6,	// this is a FLOATING POINT label
  kZCoordinate = 7,	// this is a FLOATING POINT label
  kNodeBlocked = 8, // this is a LONG label
  kFirstData = 9
};

/** kFirstData & beyond:
* 
* in abstract graph these are the node numbers of the abstraction (LONG labels)
*
* in 0th level abstraction they are the x, y location of the tile
* along with which side (type tCorner)  (all LONG labels)
*/

/** Definitions for edge labels */
enum {
  kEdgeCapacity=2
};

const double kUnknownPosition = -50.0;

#endif
