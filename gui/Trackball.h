/*
 *  $Id: trackball.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/18/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */
#ifndef __trackball_h__
#define __trackball_h__

#ifdef __cplusplus
extern "C" {
#endif

// called with the start position and the window origin + size
void startTrackball (long x, long y, long originX, long originY, long width, long height);

// calculated rotation based on current mouse position
void rollToTrackball (long x, long y, float rot [4]); // rot is output rotation angle

// add a GL rotation (dA) to an existing GL rotation (A)
void addToRotationTrackball (float * dA, float * A);

#ifdef __cplusplus
}
#endif

#endif
