//
//  Hexagon.cpp
//  Hexagon
//
//  Created by Nathan Sturtevant on 11/13/21.
//  Copyright © 2021 MovingAI. All rights reserved.
//

#include "Hexagon.h"
#include <ctype.h>

#pragma mark -
#pragma mark Efficient Hexagon Search Code
#pragma mark -

constexpr uint64_t bits(uint64_t a, uint64_t b, uint64_t c)
{
//	const uint64_t one = 1;
	return ((((uint64_t)1)<<a)|(((uint64_t)1)<<b)|(((uint64_t)1)<<c));
}

constexpr uint64_t bits(uint64_t a, uint64_t b, uint64_t c, uint64_t d, uint64_t e, uint64_t f)
{
	return bits(a, b, c)|bits(d, e, f);
}

const int rotateCWTable[54] =
{5,6,14,15,25,26,37,3,4,12,13,23,24,35,36,46,1,2,10,11,21,22,33,34,44,45,53,0,8,9,19,20,31,32,42,43,51,52,7,17,18,29,30,40,41,49,50,16,27,28,38,39,47,48};

const int flipTable[54] =
{47,48,49,50,51,52,53,38,39,40,41,42,43,44,45,46,27,28,29,30,31,32,33,34,35,36,37,16,17,18,19,20,21,22,23,24,25,26,7,8,9,10,11,12,13,14,15,0,1,2,3,4,5,6};

const int noFlipMoveCount[numPieces] =
{
//	kHexagon = 0,
	4, // no flip symmetry
//	kButterfly = 1,
	42, // no flip symmetry
//	kElbow = 2,
	72, // no symmetry
//	kLine = 3,
	72/2,
//	kMountains = 4,
	156/2,
//	kWrench = 5,
	156/2,
//	kTriangle = 6,
	144/2,
//	kHook = 7,
	168/2,
//	kTrapezoid = 8,
	126,
//	kSnake = 9
	72/2
};

const uint64_t locations[numPieces][14*6*2+1] =
{ // pieces
	{ // hexagon piece - symmetric in all ways
//		4, // count
//		bits(0, 1, 2, 8, 9, 10), bits(2, 3, 4, 10, 11, 12),
//		bits(9, 10, 11, 19, 20, 21), bits(20, 21, 22, 31, 32, 33)

		// re-ordered so first 4 are the ones we want. The remaining ones are used for symmetry building
		4, // kHexagon count
		bits(0, 1, 2, 8, 9, 10), bits(2, 3, 4, 10, 11, 12), bits(9, 10, 11, 19, 20, 21), bits(20, 21, 22, 31, 32, 33),
		bits(7, 8, 9, 17, 18, 19), bits(11, 12, 13, 21, 22, 23), bits(13, 14, 15, 23, 24, 25), bits(4, 5, 6, 12, 13, 14),
		bits(16, 17, 18, 27, 28, 29), bits(18, 19, 20, 29, 30, 31), bits(22, 23, 24, 33, 34, 35), bits(24, 25, 26, 35, 36, 37),
		bits(28, 29, 30, 38, 39, 40), bits(30, 31, 32, 40, 41, 42), bits(32, 33, 34, 42, 43, 44), bits(34, 35, 36, 44, 45, 46),
		bits(39, 40, 41, 47, 48, 49), bits(41, 42, 43, 49, 50, 51), bits(43, 44, 45, 51, 52, 53)

		
//		19, // count
//		bits(0, 1, 2, 8, 9, 10), bits(2, 3, 4, 10, 11, 12), bits(4, 5, 6, 12, 13, 14),
//		bits(7, 8, 9, 17, 18, 19), bits(9, 10, 11, 19, 20, 21), bits(11, 12, 13, 21, 22, 23), bits(13, 14, 15, 23, 24, 25),
//		bits(16, 17, 18, 27, 28, 29), bits(18, 19, 20, 29, 30, 31), bits(20, 21, 22, 31, 32, 33), bits(22, 23, 24, 33, 34, 35), bits(24, 25, 26, 35, 36, 37),
//		bits(28, 29, 30, 38, 39, 40), bits(30, 31, 32, 40, 41, 42), bits(32, 33, 34, 42, 43, 44), bits(34, 35, 36, 44, 45, 46),
//		bits(39, 40, 41, 47, 48, 49), bits(41, 42, 43, 49, 50, 51), bits(43, 44, 45, 51, 52, 53)
	},
	{ // butterfly (not in my original set) - symmetric left to right and top to bottom
		42, // kButterfly count
		bits(1, 2, 3, 9, 10, 11), bits(3, 4, 5, 11, 12, 13),
		bits(8, 9, 10, 18, 19, 20), bits(10, 11, 12, 20, 21, 22), bits(12, 13, 14, 22, 23, 24),
		bits(17, 18, 19, 28, 29, 30), bits(19, 20, 21, 30, 31, 32), bits(21, 22, 23, 32, 33, 34), bits(23, 24, 25, 34, 35, 36),
		bits(29, 30, 31, 39, 40, 41), bits(31, 32, 33, 41, 42, 43), bits(33, 34, 35, 43, 44, 45),
		bits(40, 41, 42, 48, 49, 50), bits(42, 43, 44, 50, 51, 52),
		// rotated clockwise
		bits(6, 14, 15, 12, 13, 23), bits(15, 25, 26, 23, 24, 35),
		bits(4, 12, 13, 10, 11, 21), bits(13, 23, 24, 21, 22, 33), bits(24, 35, 36, 33, 34, 44),
		bits(2, 10, 11, 8, 9, 19), bits(11, 21, 22, 19, 20, 31), bits(22, 33, 34, 31, 32, 42), bits(34, 44, 45, 42, 43, 51),
		bits(9, 19, 20, 17, 18, 29), bits(20, 31, 32, 29, 30, 40), bits(32, 42, 43, 40, 41, 49),
		bits(18, 29, 30, 27, 28, 38), bits(30, 40, 41, 38, 39, 47),
		// rotated CCW
		bits(16, 17, 7, 29, 18, 19), bits(7, 8, 0, 19, 9, 10),
		bits(28, 29, 18, 40, 30, 31), bits(18, 19, 9, 31, 20, 21), bits(9, 10, 2, 21, 11, 12),
		bits(39, 40, 30, 49, 41, 42), bits(30, 31, 20, 42, 32, 33), bits(20, 21, 11, 33, 22, 23), bits(11, 12, 4, 23, 13, 14),
		bits(41, 42, 32, 51, 43, 44), bits(32, 33, 22, 44, 34, 35), bits(22, 23, 13, 35, 24, 25),
		bits(43, 44, 34, 53, 45, 46), bits(34, 35, 24, 46, 36, 37)
	},
	{ // elbow piece - note this has rotational symmetry with fliping the piece over
		// so it has 6 30 degree rotations possible
		12*6, // kElbow count
		bits(0, 1, 2, 3, 7, 8), bits(2, 3, 4, 5, 9, 10),
		bits(7, 8, 9, 10, 16, 17), bits(9, 10, 11, 12, 18, 19), bits(11, 12, 13, 14, 20, 21),
		bits(18, 19, 20, 21, 28, 29), bits(20, 21, 22, 23, 30, 31), bits(22, 23, 24, 25, 32, 33),
		bits(30, 31, 32, 33, 39, 40), bits(32, 33, 34, 35, 41, 42), bits(34, 35, 36, 37, 43, 44),
		bits(41, 42, 43, 44, 48, 49),
		// rotated 60
		bits(1, 2, 3, 4, 12, 13), bits(3, 4, 5, 6, 14, 15),
		bits(8, 9, 10, 11, 21, 22), bits(10, 11, 12, 13, 23, 24), bits(12, 13, 14, 15, 25, 26),
		bits(17, 18, 19, 20, 31, 32), bits(19, 20, 21, 22, 33, 34), bits(21, 22, 23, 24, 35, 36),
		bits(27, 28, 29, 30, 40, 41), bits(29, 30, 31, 32, 42, 43), bits(31, 32, 33, 34, 44, 45),
		bits(40, 41, 42, 43, 51, 52),
		// rotated 120 (elbow points right)
		bits(15, 25, 26, 37, 36, 46), bits(24, 35, 36, 46, 45, 53),
		bits(6, 14, 15, 25, 24, 35), bits(13, 23, 24, 35, 34, 44), bits(22, 33, 34, 44, 43, 51),
		bits(4, 12, 13, 23, 22, 33), bits(11, 21, 22, 33, 32, 42), bits(20, 31, 32, 42, 41, 49),
		bits(2, 10, 11, 21, 20, 31), bits(9, 19, 20, 31, 30, 40), bits(18, 29, 30, 40, 39, 47),
		bits(0, 8, 9, 19, 18, 29),
		// rotated 180 (elbow bottom right)
		bits(48, 49, 50, 51, 43, 44), bits(50, 51, 52, 53, 45, 46),
		bits(43, 44, 45, 46, 36, 37), bits(41, 42, 43, 44, 34, 35), bits(39, 40, 41, 42, 32, 33),
		bits(32, 33, 34, 35, 24, 25), bits(30, 31, 32, 33, 22, 23), bits(28, 29, 30, 31, 20, 21),
		bits(20, 21, 22, 23, 13, 14), bits(18, 19, 20, 21, 11, 12), bits(16, 17, 18, 19, 9, 10),
		bits(9, 10, 11, 12, 4, 5),
		// rotated 240 (elbow bottom left)
		bits(47, 48, 49, 50, 38, 39), bits(49, 50, 51, 52, 40, 41),
		bits(38, 39, 40, 41, 27, 28), bits(40, 41, 42, 43, 29, 30), bits(42, 43, 44, 45, 31, 32),
		bits(29, 30, 31, 32, 17, 18), bits(31, 32, 33, 34, 19, 20), bits(33, 34, 35, 36, 21, 22),
		bits(19, 20, 21, 22, 8, 9), bits(21, 22, 23, 24, 10, 11), bits(23, 24, 25, 26, 12, 13),
		bits(10, 11, 12, 13, 1, 2),
		// rotated 300 (elbow left)
		bits(7, 17, 16, 27, 28, 38), bits(0, 8, 7, 17, 18, 29),
		bits(18, 29, 28, 38, 39, 47), bits(9, 19, 18, 29, 30, 40), bits(2, 10, 9, 19, 20, 31),
		bits(20, 31, 30, 40, 41, 49), bits(11, 21, 20, 31, 32, 42), bits(4, 12, 11, 21, 22, 33),
		bits(22, 33, 32, 42, 43, 51), bits(13, 23, 22, 33, 34, 44), bits(6, 14, 13, 23, 24, 35),
		bits(24, 35, 34, 44, 45, 53)
	},
	{ // long parallelogram - symmetric left to right
		24*3, // kLine count
		// regular - flipped
		bits(0, 1, 2, 3, 4, 5),
		bits(7, 8, 9, 10, 11, 12), bits(9, 10, 11, 12, 13, 14),
		bits(16, 17, 18, 19, 20, 21), bits(18, 19, 20, 21, 22, 23), bits(20, 21, 22, 23, 24, 25),
		bits(28, 29, 30, 31, 32, 33), bits(30, 31, 32, 33, 34, 35), bits(32, 33, 34, 35, 36, 37),
		bits(39, 40, 41, 42, 43, 44), bits(41, 42, 43, 44, 45, 46),
		bits(48, 49, 50, 51, 52, 53),
		// one clockwise rotation, flipped
		bits(5, 6, 14, 15, 25, 26),
		bits(3, 4, 12, 13, 23, 24), bits(12, 13, 23, 24, 35, 36),
		bits(1, 2, 10, 11, 21, 22), bits(10, 11, 21, 22, 33, 34), bits(21, 22, 33, 34, 44, 45),
		bits(8, 9, 19, 20, 31, 32), bits(19, 20, 31, 32, 42, 43), bits(31, 32, 42, 43, 51, 52),
		bits(17, 18, 29, 30, 40, 41), bits(29, 30, 40, 41, 49, 50),
		bits(27, 28, 38, 39, 47, 48),
		// one CCW rotation, flipped
		bits(27, 16, 17, 7, 8, 0),
		bits(38, 28, 29, 18, 19, 9), bits(29, 18, 19, 9, 10, 2),
		bits(47, 39, 40, 30, 31, 20), bits(40, 30, 31, 20, 21, 11), bits(31, 20, 21, 11, 12, 4),
		bits(49, 41, 42, 32, 33, 22), bits(42, 32, 33, 22, 23, 13), bits(33, 22, 23, 13, 14, 6),
		bits(51, 43, 44, 34, 35, 24), bits(44, 34, 35, 24, 25, 15),
		bits(53, 45, 46, 36, 37, 26),
		// kLine initial
		bits(1, 2, 3, 4, 5, 6),
		bits(8, 9, 10, 11, 12, 13), bits(10, 11, 12, 13, 14, 15),
		bits(17, 18, 19, 20, 21, 22), bits(19, 20, 21, 22, 23, 24), bits(21, 22, 23, 24, 25, 26),
		bits(27, 28, 29, 30, 31, 32), bits(29, 30, 31, 32, 33, 34), bits(31, 32, 33, 34, 35, 36),
		bits(38, 39, 40, 41, 42, 43), bits(40, 41, 42, 43, 44, 45),
		bits(47, 48, 49, 50, 51, 52),
		// one clockwise rotation
		bits(6, 14, 15, 25, 26, 37),
		bits(4, 12, 13, 23, 24, 35), bits(13, 23, 24, 35, 36, 46),
		bits(2, 10, 11, 21, 22, 33), bits(11, 21, 22, 33, 34, 44), bits(22, 33, 34, 44, 45, 53),
		bits(0, 8, 9, 19, 20, 31), bits(9, 19, 20, 31, 32, 42), bits(20, 31, 32, 42, 43, 51),
		bits(7, 17, 18, 29, 30, 40), bits(18, 29, 30, 40, 41, 49),
		bits(16, 27, 28, 38, 39, 47),
		// one CCW rotation
		bits(16, 17, 7, 8, 0, 1),
		bits(28, 29, 18, 19, 9, 10), bits(18, 19, 9, 10, 2, 3),
		bits(39, 40, 30, 31, 20, 21), bits(30, 31, 20, 21, 11, 12), bits(20, 21, 11, 12, 4, 5),
		bits(48, 49, 41, 42, 32, 33), bits(41, 42, 32, 33, 22, 23), bits(32, 33, 22, 23, 13, 14),
		bits(50, 51, 43, 44, 34, 35), bits(43, 44, 34, 35, 24, 25),
		bits(52, 53, 45, 46, 36, 37),	},
	{ // two trapezoids on top of each other [kMountain] (note some are removed because they leave unfillable gaps
		52*3, // kMountains count
		// original
		bits(0, 1, 2, 7, 8, 9), bits(2, 3, 4, 9, 10, 11), bits(4, 5, 6, 11, 12, 13),
		bits(7, 8, 9, 16, 17, 18), bits(9, 10, 11, 18, 19, 20), bits(11, 12, 13, 20, 21, 22), bits(13, 14, 15, 22, 23, 24),
		bits(18, 19, 20, 28, 29, 30), bits(20, 21, 22, 30, 31, 32), bits(22, 23, 24, 32, 33, 34), //bits(24, 25, 26, 34, 35, 36),
		bits(30, 31, 32, 39, 40, 41), bits(32, 33, 34, 41, 42, 43), //bits(34, 35, 36, 43, 44, 45),
		bits(41, 42, 43, 48, 49, 50), //bits(43, 44, 45, 50, 51, 52),
		// original flipped both left/right and up/down
		/*bits(1, 2, 3, 8, 9, 10),*/ bits(3, 4, 5, 10, 11, 12),
		/*bits(8, 9, 10, 17, 18, 19),*/ bits(10, 11, 12, 19, 20, 21), bits(12, 13, 14, 21, 22, 23),
		/*bits(17, 18, 19, 27, 28, 29),*/ bits(19, 20, 21, 29, 30, 31), bits(21, 22, 23, 31, 32, 33), bits(23, 24, 25, 33, 34, 35),
		bits(29, 30, 31, 38, 39, 40), bits(31, 32, 33, 40, 41, 42), bits(33, 34, 35, 42, 43, 44), bits(35, 36, 37, 44, 45, 46),
		bits(40, 41, 42, 47, 48, 49), bits(42, 43, 44, 49, 50, 51), bits(44, 45, 46, 51, 52, 53),
		
		// original rotated CW - 52+
		bits(5, 6, 14, 3, 4, 12), bits(14, 15, 25, 12, 13, 23), bits(25, 26, 37, 23, 24, 35),
		bits(3, 4, 12, 1, 2, 10), bits(12, 13, 23, 10, 11, 21), bits(23, 24, 35, 21, 22, 33), bits(35, 36, 46, 33, 34, 44),
		bits(10, 11, 21, 8, 9, 19), bits(21, 22, 33, 19, 20, 31), bits(33, 34, 44, 31, 32, 42),
		bits(19, 20, 31, 17, 18, 29), bits(31, 32, 42, 29, 30, 40),
		bits(29, 30, 40, 27, 28, 38),
		// flipped top to bottom and left to right
		bits(15, 25, 26, 13, 23, 24),
		bits(13, 23, 24, 11, 21, 22), bits(24, 35, 36, 22, 33, 34),
		bits(11, 21, 22, 9, 19, 20), bits(22, 33, 34, 20, 31, 32), bits(34, 44, 45, 32, 42, 43),
		bits(9, 19, 20, 7, 17, 18), bits(20, 31, 32, 18, 29, 30), bits(32, 42, 43, 30, 40, 41), bits(43, 51, 52, 41, 49, 50),
		bits(18, 29, 30, 16, 27, 28), bits(30, 40, 41, 28, 38, 39), bits(41, 49, 50, 39, 47, 48),
		
		// original rotated CCW - 104+
		bits(27, 16, 17, 38, 28, 29), bits(17, 7, 8, 29, 18, 19), bits(8, 0, 1, 19, 9, 10),
		bits(38, 28, 29, 47, 39, 40), bits(29, 18, 19, 40, 30, 31), bits(19, 9, 10, 31, 20, 21), bits(10, 2, 3, 21, 11, 12),
		bits(40, 30, 31, 49, 41, 42), bits(31, 20, 21, 42, 32, 33), bits(21, 11, 12, 33, 22, 23),
		bits(42, 32, 33, 51, 43, 44), bits(33, 22, 23, 44, 34, 35),
		bits(44, 34, 35, 53, 45, 46),
		// flipped top to bottom and left to right (180 degree rotation)
		bits(7, 8, 0, 18, 19, 9),
		bits(18, 19, 9, 30, 31, 20), bits(9, 10, 2, 20, 21, 11),
		bits(30, 31, 20, 41, 42, 32), bits(20, 21, 11, 32, 33, 22), bits(11, 12, 4, 22, 23, 13),
		bits(41, 42, 32, 50, 51, 43), bits(32, 33, 22, 43, 44, 34), bits(22, 23, 13, 34, 35, 24), bits(13, 14, 6, 24, 25, 15),
		bits(43, 44, 34, 52, 53, 45), bits(34, 35, 24, 45, 46, 36), bits(24, 25, 15, 36, 37, 26),

		// original flipped top to bottom
		bits(1, 2, 3, 10, 11, 12), //bits(3, 4, 5, 12, 13, 14),
		bits(8, 9, 10, 19, 20, 21), bits(10, 11, 12, 21, 22, 23), //bits(12, 13, 14, 23, 24, 25),
		bits(17, 18, 19, 29, 30, 31), bits(19, 20, 21, 31, 32, 33), bits(21, 22, 23, 33, 34, 35), //bits(23, 24, 25, 35, 36, 37),
		bits(27, 28, 29, 38, 39, 40), bits(29, 30, 31, 40, 41, 42), bits(31, 32, 33, 42, 43, 44), bits(33, 34, 35, 44, 45, 46),
		bits(38, 39, 40, 47, 48, 49), bits(40, 41, 42, 49, 50, 51), bits(42, 43, 44, 51, 52, 53),
		// original flipped left to right
		bits(0, 1, 2, 9, 10, 11), bits(2, 3, 4, 11, 12, 13), bits(4, 5, 6, 13, 14, 15),
		bits(7, 8, 9, 18, 19, 20), bits(9, 10, 11, 20, 21, 22), bits(11, 12, 13, 22, 23, 24), bits(13, 14, 15, 24, 25, 26),
		/*16-18,28-30,*/ bits(18, 19, 20, 30, 31, 32), bits(20, 21, 22, 32, 33, 34), bits(22, 23, 24, 34, 35, 36),
		/*28-30,39-41,*/ bits(30, 31, 32, 41, 42, 43), bits(32, 33, 34, 43, 44, 45),
		/*39-41,48-50,*/ bits(41, 42, 43, 50, 51, 52),
		
		// original rotated CW flipped top to bottom
		bits(6, 14, 15, 13, 23, 24),
		bits(4, 12, 13, 11, 21, 22), bits(13, 23, 24, 22, 33, 34),
		bits(2, 10, 11, 9, 19, 20), bits(11, 21, 22, 20, 31, 32), bits(22, 33, 34, 32, 42, 43),
		bits(0, 8, 9, 7, 17, 18), bits(9, 19, 20, 18, 29, 30), bits(20, 31, 32, 30, 40, 41), bits(32, 42, 43, 41, 49, 50),
		bits(7, 17, 18, 16, 27, 28), bits(18, 29, 30, 28, 38, 39), bits(30, 40, 41, 39, 47, 48),
		// original rotated CW flipped left to right
		bits(5, 6, 14, 12, 13, 23), bits(14, 15, 25, 23, 24, 35), bits(25, 26, 37, 35, 36, 46),
		bits(3, 4, 12, 10, 11, 21), bits(12, 13, 23, 21, 22, 33), bits(23, 24, 35, 33, 34, 44), bits(35, 36, 46, 44, 45, 53),
		bits(10, 11, 21, 19, 20, 31), bits(21, 22, 33, 31, 32, 42), bits(33, 34, 44, 42, 43, 51),
		bits(19, 20, 31, 29, 30, 40), bits(31, 32, 42, 40, 41, 49),
		bits(29, 30, 40, 38, 39, 47),
		
		// original rotated CCW flipped top to bottom
		bits(16, 17, 7, 18, 19, 9),
		bits(28, 29, 18, 30, 31, 20), bits(18, 19, 9, 20, 21, 11),
		bits(39, 40, 30, 41, 42, 32), bits(30, 31, 20, 32, 33, 22), bits(20, 21, 11, 22, 23, 13),
		bits(48, 49, 41, 50, 51, 43), bits(41, 42, 32, 43, 44, 34), bits(32, 33, 22, 34, 35, 24), bits(22, 23, 13, 24, 25, 15),
		bits(50, 51, 43, 52, 53, 45), bits(43, 44, 34, 45, 46, 36), bits(34, 35, 24, 36, 37, 26),
		// original rotated CCW flipped left to right
		bits(27, 16, 17, 29, 18, 19), bits(17, 7, 8, 19, 9, 10), bits(8, 0, 1, 10, 2, 3),
		bits(38, 28, 29, 40, 30, 31), bits(29, 18, 19, 31, 20, 21), bits(19, 9, 10, 21, 11, 12), bits(10, 2, 3, 12, 4, 5),
		bits(40, 30, 31, 42, 32, 33), bits(31, 20, 21, 33, 22, 23), bits(21, 11, 12, 23, 13, 14),
		bits(42, 32, 33, 44, 34, 35), bits(33, 22, 23, 35, 24, 25),
		bits(44, 34, 35, 46, 36, 37),
	},
	{ // long piece a bit like the elbow but asymmetric
		13*6*2, // kWrench count
		// meaning there are 6 rotations x 2 after flipping
		// standard
		bits(0, 1, 2, 3, 4, 8), bits(2, 3, 4, 5, 6, 10),
		bits(7, 8, 9, 10, 11, 17), bits(9, 10, 11, 12, 13, 19), bits(11, 12, 13, 14, 15, 21),
		bits(16, 17, 18, 19, 20, 27), bits(18, 19, 20, 21, 22, 29), bits(20, 21, 22, 23, 24, 31), bits(22, 23, 24, 25, 26, 33),
		bits(28, 29, 30, 31, 32, 38), bits(30, 31, 32, 33, 34, 40), bits(32, 33, 34, 35, 36, 42),
		bits(39, 40, 41, 42, 43, 47),
		// rotated 60
		bits(4, 5, 6, 14, 15, 25), bits(13, 14, 15, 25, 26, 37),
		bits(2, 3, 4, 12, 13, 23), bits(11, 12, 13, 23, 24, 35), bits(22, 23, 24, 35, 36, 46),
		bits(0, 1, 2, 10, 11, 21), bits(9, 10, 11, 21, 22, 33), bits(20, 21, 22, 33, 34, 44), bits(32, 33, 34, 44, 45, 53),
		bits(7, 8, 9, 19, 20, 31), bits(18, 19, 20, 31, 32, 42), bits(30, 31, 32, 42, 43, 51),
		bits(16, 17, 18, 29, 30, 40),
		// rotated 120
		bits(25, 26, 36, 37, 45, 46), bits(35, 36, 45, 46, 52, 53),
		bits(14, 15, 24, 25, 34, 35), bits(23, 24, 34, 35, 43, 44), bits(33, 34, 43, 44, 50, 51),
		bits(5, 6, 13, 14, 22, 23), bits(12, 13, 22, 23, 32, 33), bits(21, 22, 32, 33, 41, 42), bits(31, 32, 41, 42, 48, 49),
		bits(3, 4, 11, 12, 20, 21), bits(10, 11, 20, 21, 30, 31), bits(19, 20, 30, 31, 39, 40),
		bits(1, 2, 9, 10, 18, 19),
		// rotated 180
		bits(47, 48, 49, 50, 51, 43), bits(49, 50, 51, 52, 53, 45),
		bits(38, 39, 40, 41, 42, 32), bits(40, 41, 42, 43, 44, 34), bits(42, 43, 44, 45, 46, 36),
		bits(27, 28, 29, 30, 31, 20), bits(29, 30, 31, 32, 33, 22), bits(31, 32, 33, 34, 35, 24), bits(33, 34, 35, 36, 37, 26),
		bits(17, 18, 19, 20, 21, 11), bits(19, 20, 21, 22, 23, 13), bits(21, 22, 23, 24, 25, 15),
		bits(10, 11, 12, 13, 14, 6),
		// rotated 240
		bits(28, 38, 39, 47, 48, 49), bits(16, 27, 28, 38, 39, 40),
		bits(30, 40, 41, 49, 50, 51), bits(18, 29, 30, 40, 41, 42), bits(7, 17, 18, 29, 30, 31),
		bits(32, 42, 43, 51, 52, 53), bits(20, 31, 32, 42, 43, 44), bits(9, 19, 20, 31, 32, 33), bits(0, 8, 9, 19, 20, 21),
		bits(22, 33, 34, 44, 45, 46), bits(11, 21, 22, 33, 34, 35), bits(2, 10, 11, 21, 22, 23),
		bits(13, 23, 24, 35, 36, 37),
		// rotated 300
		bits(0, 1, 7, 8, 17, 18), bits(7, 8, 16, 17, 27, 28),
		bits(2, 3, 9, 10, 19, 20), bits(9, 10, 18, 19, 29, 30), bits(18, 19, 28, 29, 38, 39),
		bits(4, 5, 11, 12, 21, 22), bits(11, 12, 20, 21, 31, 32), bits(20, 21, 30, 31, 40, 41), bits(30, 31, 39, 40, 47, 48),
		bits(13, 14, 22, 23, 33, 34), bits(22, 23, 32, 33, 42, 43), bits(32, 33, 41, 42, 49, 50),
		bits(34, 35, 43, 44, 51, 52),
		// standard - flipped
		bits(0, 1, 2, 3, 4, 12), bits(2, 3, 4, 5, 6, 14),
		bits(7, 8, 9, 10, 11, 21), bits(9, 10, 11, 12, 13, 23), bits(11, 12, 13, 14, 15, 25),
		bits(16, 17, 18, 19, 20, 31), bits(18, 19, 20, 21, 22, 33), bits(20, 21, 22, 23, 24, 35), bits(22, 23, 24, 25, 26, 37),
		bits(28, 29, 30, 31, 32, 42), bits(30, 31, 32, 33, 34, 44), bits(32, 33, 34, 35, 36, 46),
		bits(41, 42, 43, 44, 45, 53),
		// rotated 60 - flipped
		bits(24, 5, 6, 14, 15, 25), bits(36, 14, 15, 25, 26, 37),
		bits(22, 3, 4, 12, 13, 23), bits(34, 12, 13, 23, 24, 35), bits(45, 23, 24, 35, 36, 46),
		bits(20, 1, 2, 10, 11, 21), bits(32, 10, 11, 21, 22, 33), bits(43, 21, 22, 33, 34, 44), bits(52, 33, 34, 44, 45, 53),
		bits(30, 8, 9, 19, 20, 31), bits(41, 19, 20, 31, 32, 42), bits(50, 31, 32, 42, 43, 51),
		bits(48, 29, 30, 40, 41, 49),
		// rotated 120 - flipped
		bits(44, 26, 36, 37, 45, 46), bits(51, 36, 45, 46, 52, 53),
		bits(33, 15, 24, 25, 34, 35), bits(42, 24, 34, 35, 43, 44), bits(49, 34, 43, 44, 50, 51),
		bits(21, 6, 13, 14, 22, 23), bits(31, 13, 22, 23, 32, 33), bits(40, 22, 32, 33, 41, 42), bits(47, 32, 41, 42, 48, 49),
		bits(19, 4, 11, 12, 20, 21), bits(29, 11, 20, 21, 30, 31), bits(38, 20, 30, 31, 39, 40),
		bits(27, 9, 19, 18, 28, 29),
		// rotated 180 - flipped
		bits(47, 48, 49, 50, 51, 39), bits(49, 50, 51, 52, 53, 41),
		bits(38, 39, 40, 41, 42, 28), bits(40, 41, 42, 43, 44, 30), bits(42, 43, 44, 45, 46, 32),
		bits(27, 28, 29, 30, 31, 16), bits(29, 30, 31, 32, 33, 18), bits(31, 32, 33, 34, 35, 20), bits(33, 34, 35, 36, 37, 22),
		bits(17, 18, 19, 20, 21, 7), bits(19, 20, 21, 22, 23, 9), bits(21, 22, 23, 24, 25, 11),
		bits(8, 9, 10, 11, 12, 0),
		// rotated 240 - flipped
		bits(28, 38, 39, 47, 48, 29), bits(16, 27, 28, 38, 39, 17),
		bits(30, 40, 41, 49, 50, 31), bits(18, 29, 30, 40, 41, 19), bits(7, 17, 18, 29, 30, 8),
		bits(32, 42, 43, 51, 52, 33), bits(20, 31, 32, 42, 43, 21), bits(9, 19, 20, 31, 32, 10), bits(0, 8, 9, 19, 20, 1),
		bits(22, 33, 34, 44, 45, 23), bits(11, 21, 22, 33, 34, 12), bits(2, 10, 11, 21, 22, 3),
		bits(4, 12, 13, 23, 24, 5),
		// rotated 300 - flipped
		bits(0, 1, 7, 8, 17, 2), bits(7, 8, 16, 17, 27, 9),
		bits(2, 3, 9, 10, 19, 4), bits(9, 10, 18, 19, 29, 11), bits(18, 19, 28, 29, 38, 20),
		bits(4, 5, 11, 12, 21, 6), bits(11, 12, 20, 21, 31, 13), bits(20, 21, 30, 31, 40, 22), bits(30, 31, 39, 40, 47, 32),
		bits(13, 14, 22, 23, 33, 15), bits(22, 23, 32, 33, 42, 24), bits(32, 33, 41, 42, 49, 34),
		bits(24, 25, 34, 35, 44, 26),
	},
	{ // triangle piece
		12*6*2, // kTriangle count
		// regular
		bits(1, 2, 3, 4, 5, 10),
		bits(8, 9, 10, 11, 12, 19), bits(10, 11, 12, 13, 14, 21),
		bits(17, 18, 19, 20, 21, 29), bits(19, 20, 21, 22, 23, 31), bits(21, 22, 23, 24, 25, 33),
		bits(27, 28, 29, 30, 31, 38), bits(29, 30, 31, 32, 33, 40), bits(31, 32, 33, 34, 35, 42), bits(33, 34, 35, 36, 37, 44),
		bits(38, 39, 40, 41, 42, 47), bits(40, 41, 42, 43, 44, 49),
		// rotated 60
		bits(6, 13, 14, 15, 25, 26),
		bits(4, 11, 12, 13, 23, 24), bits(13, 22, 23, 24, 35, 36),
		bits(2, 9, 10, 11, 21, 22), bits(11, 20, 21, 22, 33, 34), bits(22, 32, 33, 34, 44, 45),
		bits(0, 7, 8, 9, 19, 20), bits(9, 18, 19, 20, 31, 32), bits(20, 30, 31, 32, 42, 43), bits(32, 41, 42, 43, 51, 52),
		bits(7, 16, 17, 18, 29, 30), bits(18, 28, 29, 30, 40, 41),
		// rotated 120
		bits(35, 36, 37, 45, 46, 53),
		bits(23, 24, 25, 34, 35, 44), bits(33, 34, 35, 43, 44, 51),
		bits(12, 13, 14, 22, 23, 33), bits(21, 22, 23, 32, 33, 42), bits(31, 32, 33, 41, 42, 49),
		bits(3, 4, 5, 11, 12, 21), bits(10, 11, 12, 20, 21, 31), bits(19, 20, 21, 30, 31, 40), bits(29, 30, 31, 39, 40, 47),
		bits(1, 2, 3, 9, 10, 19), bits(8, 9, 10, 18, 19, 29),
		// rotated 180
		bits(48, 49, 50, 51, 52, 43),
		bits(41, 42, 43, 44, 45, 34), bits(39, 40, 41, 42, 43, 32),
		bits(32, 33, 34, 35, 36, 24), bits(30, 31, 32, 33, 34, 22), bits(28, 29, 30, 31, 32, 20),
		bits(22, 23, 24, 25, 26, 15), bits(20, 21, 22, 23, 24, 13), bits(18, 19, 20, 21, 22, 11), bits(16, 17, 18, 19, 20, 9),
		bits(6, 11, 12, 13, 14, 15), bits(9, 10, 11, 12, 13, 4),
		// rotated 240
		bits(27, 28, 38, 39, 40, 47),
		bits(29, 30, 40, 41, 42, 49), bits(17, 18, 29, 30, 31, 40),
		bits(31, 32, 42, 43, 44, 51), bits(19, 20, 31, 32, 33, 42), bits(8, 9, 19, 20, 21, 31),
		bits(33, 34, 44, 45, 46, 53), bits(21, 22, 33, 34, 35, 44), bits(10, 11, 21, 22, 23, 33), bits(1, 2, 10, 11, 12, 21),
		bits(23, 24, 35, 36, 37, 46), bits(12, 13, 23, 24, 25, 35),
		// rotated 300
		bits(0, 7, 8, 16, 17, 18),
		bits(9, 18, 19, 28, 29, 30), bits(2, 9, 10, 18, 19, 20),
		bits(20, 30, 31, 39, 40, 41), bits(11, 20, 21, 30, 31, 32), bits(4, 11, 12, 20, 21, 22),
		bits(32, 41, 42, 48, 49, 50), bits(22, 32, 33, 41, 42, 43), bits(13, 22, 23, 32, 33, 34), bits(6, 13, 14, 22, 23, 24),
		bits(34, 43, 44, 50, 51, 52), bits(24, 34, 35, 43, 44, 45),
		// flipped
		bits(0, 7, 8, 9, 10, 11), bits(2, 9, 10, 11, 12, 13),
		bits(7, 16, 17, 18, 19, 20), bits(9, 18, 19, 20, 21, 22), bits(11, 20, 21, 22, 23, 24), bits(13, 22, 23, 24, 25, 26),
		bits(18, 28, 29, 30, 31, 32), bits(20, 30, 31, 32, 33, 34), bits(22, 32, 33, 34, 35, 36),
		bits(30, 39, 40, 41, 42, 43), bits(32, 41, 42, 43, 44, 45),
		bits(41, 48, 49, 50, 51, 52),
		// rotated 60 - flipped
		bits(3, 4, 5, 12, 13, 23), bits(12, 13, 14, 23, 24, 35),
		bits(1, 2, 3, 10, 11, 21), bits(10, 11, 12, 21, 22, 33), bits(21, 22, 23, 33, 34, 44), bits(33, 34, 35, 44, 45, 53),
		bits(8, 9, 10, 19, 20, 31), bits(19, 20, 21, 31, 32, 42), bits(31, 32, 33, 42, 43, 51),
		bits(17, 18, 19, 29, 30, 40), bits(29, 30, 31, 40, 41, 49),
		bits(27, 28, 29, 38, 39, 47),
		// rotated 120 - flipped
		bits(15, 24, 25, 26, 34, 35), bits(24, 34, 35, 36, 43, 44),
		bits(6, 13, 14, 15, 22, 23), bits(13, 22, 23, 24, 32, 33), bits(22, 32, 33, 34, 41, 42), bits(32, 41, 42, 43, 48, 49),
		bits(4, 11, 12, 13, 20, 21), bits(11, 20, 21, 22, 30, 31), bits(20, 30, 31, 32, 39, 40),
		bits(2, 9, 10, 11, 18, 19), bits(9, 18, 19, 20, 28, 29),
		bits(0, 7, 8, 9, 16, 17),
		// rotated 180 - flipped
		bits(42, 43, 44, 45, 46, 53), bits(40, 41, 42, 43, 44, 51),
		bits(33, 34, 35, 36, 37, 46), bits(31, 32, 33, 34, 35, 44), bits(29, 30, 31, 32, 33, 42), bits(27, 28, 29, 30, 31, 40),
		bits(21, 22, 23, 24, 25, 35), bits(19, 20, 21, 22, 23, 33), bits(17, 18, 19, 20, 21, 31),
		bits(10, 11, 12, 13, 14, 23), bits(8, 9, 10, 11, 12, 21),
		bits(1, 2, 3, 4, 5, 12),
		// rotated 240 - flipped
		bits(30, 40, 41, 48, 49, 50), bits(18, 29, 30, 39, 40, 41),
		bits(32, 42, 43, 50, 51, 52), bits(20, 31, 32, 41, 42, 43), bits(9, 19, 20, 30, 31, 32), bits(0, 8, 9, 18, 19, 20),
		bits(22, 33, 34, 43, 44, 45), bits(11, 21, 22, 32, 33, 34), bits(2, 10, 11, 20, 21, 22),
		bits(13, 23, 24, 34, 35, 36), bits(4, 12, 13, 22, 23, 24),
		bits(6, 14, 15, 24, 25, 26),
		// rotated 300 - flipped
		bits(18, 19, 27, 28, 29, 38), bits(9, 10, 17, 18, 19, 29),
		bits(30, 31, 38, 39, 40, 47), bits(20, 21, 29, 30, 31, 40), bits(11, 12, 19, 20, 21, 31), bits(4, 5, 10, 11, 12, 21),
		bits(32, 33, 40, 41, 42, 49), bits(22, 23, 31, 32, 33, 42), bits(13, 14, 21, 22, 23, 33),
		bits(34, 35, 42, 43, 44, 51), bits(24, 25, 33, 34, 35, 44),
		bits(36, 37, 44, 45, 46, 53),
	},
	{
		14*6*2, // kHook count

		// V-like piece-flip
		bits(0, 1, 8, 9, 10, 11), bits(2, 3, 10, 11, 12, 13),
		bits(7, 8, 17, 18, 19, 20), bits(9, 10, 19, 20, 21, 22), bits(11, 12, 21, 22, 23, 24),
		bits(16, 17, 27, 28, 29, 30), bits(18, 19, 29, 30, 31, 32), bits(20, 21, 31, 32, 33, 34), bits(22, 23, 33, 34, 35, 36),
		bits(28, 29, 38, 39, 40, 41), bits(30, 31, 40, 41, 42, 43), bits(32, 33, 42, 43, 44, 45),
		bits(39, 40, 47, 48, 49, 50), bits(41, 42, 49, 50, 51, 52),
		// 60-degrees-flip
		bits(4, 5, 6, 12, 13, 23), bits(13, 14, 15, 23, 24, 35),
		bits(2, 3, 4, 10, 11, 21), bits(11, 12, 13, 21, 22, 33), bits(22, 23, 24, 33, 34, 44),
		bits(0, 1, 2, 8, 9, 19), bits(9, 10, 11, 19, 20, 31), bits(20, 21, 22, 31, 32, 42), bits(32, 33, 34, 42, 43, 51),
		bits(7, 8, 9, 17, 18, 29), bits(18, 19, 20, 29, 30, 40), bits(30, 31, 32, 40, 41, 49),
		bits(16, 17, 18, 27, 28, 38), bits(28, 29, 30, 38, 39, 47),
		// 120-degree-flip
		bits(24, 25, 26, 34, 35, 37), bits(13, 14, 15, 22, 23, 25), bits(4, 5, 6, 11, 12, 14),
		bits(34, 35, 36, 43, 44, 46), bits(22, 23, 24, 32, 33, 35), bits(11, 12, 13, 20, 21, 23), bits(2, 3, 4, 9, 10, 12),
		bits(32, 33, 34, 41, 42, 44), bits(20, 21, 22, 30, 31, 33), bits(9, 10, 11, 18, 19, 21), bits(0, 1, 2, 7, 8, 10),
		bits(30, 31, 32, 39, 40, 42), bits(18, 19, 20, 28, 29, 31), bits(7, 8, 9, 16, 17, 19),
		// 180-degree-flip
		bits(42, 43, 44, 45, 52, 53), bits(40, 41, 42, 43, 50, 51),
		bits(33, 34, 35, 36, 45, 46), bits(31, 32, 33, 34, 43, 44), bits(29, 30, 31, 32, 41, 42),
		bits(23, 24, 25, 26, 36, 37), bits(21, 22, 23, 24, 34, 35), bits(19, 20, 21, 22, 32, 33), bits(17, 18, 19, 20, 30, 31),
		bits(12, 13, 14, 15, 24, 25), bits(10, 11, 12, 13, 22, 23), bits(8, 9, 10, 11, 20, 21),
		bits(3, 4, 5, 6, 13, 14), bits(1, 2, 3, 4,11, 12),
		// 240-degree-flip
		bits(30, 40, 41, 47, 48, 49), bits(32, 42, 43, 49, 50, 51), bits(34, 44, 45, 51, 52, 53),
		bits(18, 29, 30, 38, 39, 40), bits(20, 31, 32, 40, 41, 42), bits(22, 33, 34, 42, 43, 44), bits(24, 35, 36, 44, 45, 46),
		bits(9, 19, 20, 29, 30, 31), bits(11, 21, 22, 31, 32, 33), bits(13, 23, 24, 33, 34, 35), bits(15, 25, 26, 35, 36, 37),
		bits(2, 10, 11, 19, 20, 21), bits(4, 12, 13, 21, 22, 23), bits(6, 14, 15, 23, 24, 25),
		// 300-degree-flip
		bits(16, 18, 19, 27, 28, 29), bits(7, 9, 10, 17, 18, 19),
		bits(28, 30, 31, 38, 39, 40), bits(18, 20, 21, 29, 30, 31), bits(9, 11, 12, 19, 20, 21),
		bits(39, 41, 42, 47, 48, 49), bits(30, 32, 33, 40, 41, 42), bits(20, 22, 23, 31, 32, 33), bits(11, 13, 14, 21, 22, 23),
		bits(41, 43, 44, 49, 50, 51), bits(32, 34, 35, 42, 43, 44), bits(22, 24, 25, 33, 34, 35),
		bits(43, 45, 46, 51, 52, 53), bits(34, 36, 37, 44, 45, 46),
		// hook / V-like piece
		bits(0, 1, 2, 3, 8, 9), bits(2, 3, 4, 5, 10, 11),
		bits(7, 8, 9, 10, 17, 18), bits(9, 10, 11, 12, 19, 20), bits(11, 12, 13, 14, 21, 22),
		bits(16, 17, 18, 19, 27, 28), bits(18, 19, 20, 21, 29, 30), bits(20, 21, 22, 23, 31, 32), bits(22, 23, 24, 25, 33, 34),
		bits(28, 29, 30, 31, 38, 39), bits(30, 31, 32, 33, 40, 41), bits(32, 33, 34, 35, 42, 43),
		bits(39, 40, 41, 42, 47, 48), bits(41, 42, 43, 44, 49, 50),
		// 60-degrees
		bits(4, 5, 6, 12, 14, 15), bits(13, 14, 15, 23, 25, 26),
		bits(2, 3, 4, 10, 12, 13), bits(11, 12, 13, 21, 23, 24), bits(22, 23, 24, 33, 35, 36),
		bits(0, 1, 2, 8, 10, 11), bits(9, 10, 11, 19, 21, 22), bits(20, 21, 22, 31, 33, 34), bits(32, 33, 34, 42, 44, 45),
		bits(7, 8, 9, 17, 19, 20), bits(18, 19, 20, 29, 31, 32), bits(30, 31, 32, 40, 42, 43),
		bits(16, 17, 18, 27, 29, 30), bits(28, 29, 30, 38, 40, 41),
		// 120-degree
		bits(24, 25, 26, 36, 37, 46), bits(13, 14, 15, 24, 25, 35), bits(4, 5, 6, 13, 14, 23),
		bits(34, 35, 36, 45, 46, 53), bits(22, 23, 24, 34, 35, 44), bits(11, 12, 13, 22, 23, 33), bits(2, 3, 4, 11, 12, 21),
		bits(32, 33, 34, 43, 44, 51), bits(20, 21, 22, 32, 33, 42), bits(9, 10, 11, 20, 21, 31), bits(0, 1, 2, 9, 10, 19),
		bits(30, 31, 32, 41, 42, 49), bits(18, 19, 20, 30, 31, 40), bits(7, 8, 9, 18, 19, 29),
		// 180-degree
		bits(44, 45, 50, 51, 52, 53), bits(42, 43, 48, 49, 50, 51),
		bits(35, 36, 43, 44, 45, 46), bits(33, 34, 41, 42, 43, 44), bits(31, 32, 39, 40, 41, 42),
		bits(25, 26, 34, 35, 36, 37), bits(23, 24, 32, 33, 34, 35), bits(21, 22, 30, 31, 32, 33), bits(19, 20, 28, 29, 30, 31),
		bits(14, 15, 22, 23, 24, 25), bits(12, 13, 20, 21, 22, 23), bits(10, 11, 18, 19, 20, 21),
		bits(5, 6, 11, 12, 13, 14), bits(3, 4, 9, 10, 11, 12),
		// 240-degree
		bits(38, 39, 41, 47, 48, 49), bits(40, 41, 43, 49, 50, 51), bits(42, 43, 45, 51, 52, 53),
		bits(27, 28, 30, 38, 39, 40), bits(29, 30, 32, 40, 41, 42), bits(31, 32, 34, 42, 43, 44), bits(33, 34, 36, 44, 45, 46),
		bits(17, 18, 20, 29, 30, 31), bits(19, 20, 22, 31, 32, 33), bits(21, 22, 24, 33, 34, 35), bits(23, 24, 26, 35, 36, 37),
		bits(8, 9, 11, 19, 20, 21), bits(10, 11, 13, 21, 22, 23), bits(12, 13, 15, 23, 24, 25),
		// 300-degree
		bits(7, 16, 17, 27, 28, 29), bits(0, 7, 8, 17, 18, 19),
		bits(18, 28, 29, 38, 39, 40), bits(9, 18, 19, 29, 30, 31), bits(2, 9, 10, 19, 20, 21),
		bits(30, 39, 40, 47, 48, 49), bits(20, 30, 31, 40, 41, 42), bits(11, 20, 21, 31, 32, 33), bits(4, 11, 12, 21, 22, 23),
		bits(32, 41, 42, 49, 50, 51), bits(22, 32, 33, 42, 43, 44), bits(13, 22, 23, 33, 34, 35),
		bits(34, 43, 44, 51, 52, 53), bits(24, 34, 35, 44, 45, 46),
	},
	{ // small trapezoids -- symmetric left/right
		126, // kTrapezoid count
		// upright pieces
		bits(0, 1, 2), bits(2, 3, 4), bits(4, 5, 6),
		bits(7, 8, 9), bits(9, 10, 11), bits(11, 12, 13), bits(13, 14, 15),
		bits(16, 17, 18), bits(18, 19, 20), bits(20, 21, 22), bits(22, 23, 24), bits(24, 25, 26),
		bits(28, 29, 30), bits(30, 31, 32), bits(32, 33, 34), bits(34, 35, 36),
		bits(39, 40, 41), bits(41, 42, 43), bits(43, 44, 45),
		bits(48, 49, 50), bits(50, 51, 52),
		// downward pieces
		bits(1, 2, 3), bits(3, 4, 5),
		bits(8, 9, 10), bits (10, 11, 12), bits(12, 13, 14),
		bits(17, 18, 19), bits(19, 20, 21), bits(21, 22, 23), bits(23, 24, 25),
		bits(27, 28, 29), bits(29, 30, 31), bits(31, 32, 33), bits(33, 34, 35), bits(35, 36, 37),
		bits(38, 39, 40), bits(40, 41, 42), bits(42, 43, 44), bits(44, 45, 46),
		bits(47, 48, 49), bits(49, 50, 51), bits(51, 52, 53),
		// one clockwise rotation
		bits(5, 6, 14), bits(14, 15, 25), bits(25, 26, 37),
		bits(3, 4, 12), bits(12, 13, 23), bits(23, 24, 35), bits(35, 36, 46),
		bits(1, 2, 10), bits(10, 11, 21), bits(21, 22, 33), bits(33, 34, 44), bits(44, 45, 53),
		bits(8, 9, 19), bits(19, 20, 31), bits(31, 32, 42), bits(42, 43, 51),
		bits(17, 18, 29), bits(29, 30, 40), bits(40, 41, 49),
		bits(27, 28, 38), bits(38, 39, 47),
		// one clockwise rotation downward
		bits(6, 14, 15), bits(15, 25, 26),
		bits(4, 12, 13), bits(13, 23, 24), bits(24, 35, 36),
		bits(2, 10, 11), bits(11, 21, 22), bits(22, 33, 34), bits(34, 44, 45),
		bits(0, 8, 9), bits(9, 19, 20), bits(20, 31, 32), bits(32, 42, 43), bits(43, 51, 52),
		bits(7, 17, 18), bits(18, 29, 30), bits(30, 40, 41), bits(41, 49, 50),
		bits(16, 27, 28), bits(28, 38, 39), bits(39, 47, 48),
		// two clockwise rotations
		bits(27, 16, 17), bits(17, 7, 8), bits(8, 0, 1),
		bits(38, 28, 29), bits(29, 18, 19), bits(19, 9, 10), bits(10, 2, 3),
		bits(47, 39, 40), bits(40, 30, 31), bits(31, 20, 21), bits(21, 11, 12), bits(12, 4, 5),
		bits(49, 41, 42), bits(42, 32, 33), bits(33, 22, 23), bits(23, 13, 14),
		bits(51, 43, 44), bits(44, 34, 35), bits(35, 24, 25),
		bits(53, 45, 46), bits(46, 36, 37),
		// two clockwise rotations downward
		bits(16, 17, 7), bits(7, 8, 0),
		bits(28, 29, 18), bits(18, 19, 9), bits(9, 10, 2),
		bits(39, 40, 30), bits(30, 31, 20), bits(20, 21, 11), bits(11, 12, 4),
		bits(48, 49, 41), bits(41, 42, 32), bits(32, 33, 22), bits(22, 23, 13), bits(13, 14, 6),
		bits(50, 51, 43), bits(43, 44, 34), bits(34, 35, 24), bits(24, 25, 15),
		bits(52, 53, 45), bits(45, 46, 36), bits(36, 37, 26)
	},
	{ // funny last piece
		12*3*2, // kSnake count (72)
		bits(0, 1, 2, 10, 11, 12),
		bits(7, 8, 9, 19, 20, 21), bits(9, 10, 11, 21, 22, 23), bits(11, 12, 13, 23, 24, 25),
		bits(16, 17, 18, 29, 30, 31), bits(18, 19, 20, 31, 32, 33), bits(20, 21, 22, 33, 34, 35), bits(22, 23, 24, 35, 36, 37),
		bits(28, 29, 30, 40, 41, 42), bits(30, 31, 32, 42, 43, 44), bits(32, 33, 34, 44, 45, 46),
		bits(41, 42, 43, 51, 52, 53),
		// rotated 60 degrees
		bits(5, 6, 13, 14, 23, 24),
		bits(3, 4, 11, 12, 21, 22), bits(12, 13, 22, 23, 33, 34), bits(23, 24, 34, 35, 44, 45),
		bits(1, 2, 9, 10, 19, 20), bits(10, 11, 20, 21, 31, 32), bits(21, 22, 32, 33, 42, 43), bits(33, 34, 43, 44, 51, 52),
		bits(8, 9, 18, 19, 29, 30), bits(19, 20, 30, 31, 40, 41), bits(31, 32, 41, 42, 49, 50),
		bits(29, 30, 39, 40, 47, 48),
		// rotated 120 degress
		bits(26, 34, 35, 36, 37, 44),
		bits(15, 22, 23, 24, 25, 33), bits(24, 32, 33, 34, 35, 42), bits(34, 41, 42, 43, 44, 49),
		bits(6, 11, 12, 13, 14, 21), bits(13, 20, 21, 22, 23, 31), bits(22, 30, 31, 32, 33, 40), bits(32, 39, 40, 41, 42, 47),
		bits(4, 9, 10, 11, 12, 19), bits(11, 18, 19, 20, 21, 29), bits(20, 28, 29, 30, 31, 38),
		bits(9, 16, 17, 18, 19, 27),
		// flipped
		bits(4, 5, 6, 10, 11, 12), bits(9, 10, 11, 17, 18, 19), bits(11, 12, 13, 19, 20, 21), bits(13, 14, 15, 21, 22, 23),
		bits(18, 19, 20, 27, 28, 29), bits(20, 21, 22, 29, 30, 31), bits(22, 23, 24, 31, 32, 33), bits(24, 25, 26, 33, 34, 35),
		bits(30, 31, 32, 38, 39, 40), bits(32, 33, 34, 40, 41, 42), bits(34, 35, 36, 42, 43, 44),
		bits(41, 42, 43, 47, 48, 49),
		// rotated 60 degrees - flipped
		bits(13, 23, 24, 25, 26, 37),
		bits(2, 10, 11, 12, 13, 23), bits(11, 21, 22, 23, 24, 35), bits(22, 33, 34, 35, 36, 46),
		bits(0, 8, 9, 10, 11, 21), bits(9, 19, 20, 21, 22, 33), bits(20, 31, 32, 33, 34, 44), bits(32, 42, 43, 44, 45, 53),
		bits(7, 17, 18, 19, 20, 31), bits(18, 29, 30, 31, 32, 42), bits(30, 40, 41, 42, 43, 51),
		bits(16, 27, 28, 29, 30, 40),
		// rotated 120 degress - flipped
		bits(34, 35, 44, 45, 52, 53),
		bits(13, 14, 23, 24, 34, 35), bits(22, 23, 33, 34, 43, 44), bits(32, 33, 42, 43, 50, 51),
		bits(4, 5, 12, 13, 22, 23), bits(11, 12, 21, 22, 32, 33), bits(20, 21, 31, 32, 41, 42), bits(30, 31, 40, 41, 48, 49),
		bits(2, 3, 10, 11, 20, 21), bits(9, 10, 19, 20, 30, 31), bits(18, 19, 29, 30, 39, 40),
		bits(0, 1, 8, 9, 18, 19)
	}
};

HexagonEnvironment::HexagonEnvironment()
{
//	pieces = {0, 2, 3, 4, 5, 6, 7, 8, 8, 8, 8};
//	pieces = {kHexagon, kElbow, kLine, kMountains, kWrench, kTriangle, kHook, kTrapezoid, kTrapezoid, kSnake};
//	pieces = {kHexagon, kElbow, kLine, kMountains, kWrench, kTriangle, kHook, kTrapezoid, kTrapezoid, kButterfly};
	pieces = {kHexagon, kElbow, kSnake, kMountains, kWrench, kTriangle, kHook, kTrapezoid, kTrapezoid, kButterfly};
	flippable = {kCanFlip, kCanFlip, kSide1, kSide1, kSide1, kCanFlip, kCanFlip, kCanFlip, kCanFlip, kCanFlip};
//	pieces = {0, 1, 2, 3, 4, 5, 6, 7, 8, 8};
//	pieces = {0, 1, 2, 3, 4, 5, 6, 7, 9};
//	pieces = {9, 0, 1, 2, 3, 4, 5, 6, 7};
	BuildFlipTable();
	BuildRotationTable();
}

void HexagonEnvironment::SetPieces(const std::vector<tPieceName> &pieces)
{
	this->pieces.resize(0);
	for (int x = 0; x < pieces.size(); x++)
		this->pieces.push_back((int)pieces[x]);
}

void HexagonEnvironment::SetFlippable(const std::array<tFlipType, numPieces> &flips)
{
	flippable = flips;
}


HexagonEnvironment::~HexagonEnvironment()
{
	
}

void HexagonEnvironment::BuildRotationTable()
{
	uint64_t one = 1;
	//	int rotate30Map[12][14*6*2+1];
	// for all pieces
	for (int x = 0; x < numPieces; x++)
	{
		int total = locations[x][0];
		if (total == 4) total = 19;
		
		// and all locations/rotations
		for (int y = 1; y <= total; y++)
		{
			// rotate the piece from the given location
			uint64_t loc = locations[x][y];
			uint64_t rot_loc = 0;
			for (int x = 0; x < 64; x++) // simple and dumb
			{
				if ((loc>>x)&1)
				{
					rot_loc |= (one<<rotateCWTable[x]);
				}
			}
			// find match to rotation
			bool success = false;
			for (int z = 1; z <= total; z++)
			{
				if (locations[x][z] == rot_loc)
				{
					rotate30Map[x][y] = z;
					success = true;
					break;
				}
			}
			assert(success == true);
		}
	}
}

void HexagonEnvironment::BuildFlipTable()
{
	uint64_t one = 1;
	//	int flipMap[12][14*6*2+1];
	// for all pieces
	for (int x = 0; x < numPieces; x++)
	{
		int total = locations[x][0];
		if (total == 4) total = 19;

		// and all locations/rotations
		for (int y = 1; y <= total; y++)
		{
			// flip the piece from the given location
			uint64_t loc = locations[x][y];
			uint64_t flip_loc = 0;
			for (int x = 0; x < 64; x++) // simple and dumb
			{
				if ((loc>>x)&1)
				{
					flip_loc |= (one<<flipTable[x]);
				}
			}
			// find match to flip
			bool success = false;
			for (int z = 1; z <= total; z++)
			{
				if (locations[x][z] == flip_loc)
				{
					flipMap[x][y] = z;
					success = true;
					break;
				}
			}
			assert(success == true);
		}
	}
}

void HexagonEnvironment::RotateCW(HexagonSearchState &s) const
{
	HexagonSearchState tmp;
	for (int x = 0; x < s.cnt; x++)
	{
		ApplyAction(tmp, RotateCW(s.state[x]));
	}
	s = tmp;
}

HexagonAction HexagonEnvironment::RotateCW(HexagonAction a) const
{
	a.location = rotate30Map[a.piece][a.location];
	return a;
}

void HexagonEnvironment::Flip(HexagonSearchState &s) const
{
	HexagonSearchState tmp;
	for (int x = 0; x < s.cnt; x++)
	{
		ApplyAction(tmp, Flip(s.state[x]));
	}
	s = tmp;
}

HexagonAction HexagonEnvironment::Flip(HexagonAction a) const
{
	a.location = flipMap[a.piece][a.location];
	return a;
}

void HexagonEnvironment::GetSuccessors(const HexagonSearchState &nodeID, std::vector<HexagonSearchState> &neighbors) const
{
	static std::vector<HexagonAction> actions;
	GetActions(nodeID, actions);
	neighbors.resize(0);
	for (auto i : actions)
	{
		HexagonSearchState next;
		GetNextState(nodeID, i, next);
		neighbors.push_back(next);
	}
}

void HexagonEnvironment::GetActions(const HexagonSearchState &nodeID, std::vector<HexagonAction> &actions) const
{
	actions.clear();
//	for (int x = 0; x < 126; x++)
//		actions.push_back({0, x});
//	for (int x = 0; x < 19; x++)
//		actions.push_back({1, x});
//	const int piece = 8;
	unsigned int piece = pieces[nodeID.cnt];

//	for (unsigned int x = 1; x <= locations[piece][0]; x++)
	switch (flippable[nodeID.cnt])
	{
		case kCanFlip:
			for (unsigned int x = 1; x <= locations[piece][0]; x++)
			{
				if ((nodeID.bits&locations[piece][x]) == 0)
					actions.push_back({piece, x});
			}
			break;
		case kSide1:
			for (unsigned int x = 1; x <= noFlipMoveCount[piece]; x++)
			{
				if ((nodeID.bits&locations[piece][x]) == 0)
					actions.push_back({piece, x});
			}
			break;
		case kSide2:
			for (unsigned int x = noFlipMoveCount[piece]+1; x <= locations[piece][0]; x++)
			{
				if ((nodeID.bits&locations[piece][x]) == 0)
					actions.push_back({piece, x});
			}
			break;
	}
//	for (unsigned int x = 1; x <= (flippable[nodeID.cnt]?locations[piece][0]:noFlipMoveCount[piece]); x++)
//	{
//		if ((nodeID.bits&locations[piece][x]) == 0)
//			actions.push_back({piece, x});
//	}
}

//QUE: is this on purpose?
HexagonAction HexagonEnvironment::GetAction(const HexagonSearchState &s1, const HexagonSearchState &s2) const
{
	assert(false);
	return {0, 0};
}

void HexagonEnvironment::ApplyAction(HexagonSearchState &s, HexagonAction a) const
{
	s.state[s.cnt] = a;
	s.cnt++;
	s.bits ^= locations[a.piece][a.location];
}

void HexagonEnvironment::UndoAction(HexagonSearchState &s, HexagonAction a) const
{
	s.cnt--;
	s.bits ^= locations[a.piece][a.location];
}


void HexagonEnvironment::GetNextState(const HexagonSearchState &s1, HexagonAction a, HexagonSearchState &s2) const
{
	s2 = s1;
	ApplyAction(s2, a);
}

//QUE: how does this work?
bool HexagonEnvironment::InvertAction(HexagonAction &a) const
{
	return true;
}

/** Goal Test if the goal is stored **/
bool HexagonEnvironment::GoalTest(const HexagonSearchState &node) const
{
	return node.bits == (((1ull)<<54)-1); // all 54 1s set
//	return false;
}


uint64_t HexagonEnvironment::GetStateHash(const HexagonSearchState &node) const
{
	return 0;
}

uint64_t HexagonEnvironment::GetActionHash(HexagonAction act) const
{
	return 0;
}

/** Prints out the triangles used for this piece in HOG2 coordinates */
void HexagonEnvironment::GeneratePieceCoordinates(tPieceName p)
{
	// 1. Start with environment with only one piece type
	HexagonEnvironment e;
	std::vector<tPieceName> v;
	v.push_back(p);
	e.SetPieces(v);
	
	// 2. Get legal actions
	std::vector<HexagonSearchState> succ;
	HexagonSearchState s;
	e.GetSuccessors(s, succ);

	// 3. Place the piece on the board
	s = succ[0];
	
	assert(s.cnt == 1); // should only be 1 piece on the board
	
	uint64_t state = s.bits;
	
	std::cout << pieceNames[p] << "\t";
//	// count how many objects we have
	int count = 0;
	for (int t = 0; t < 54; t++)
	{
		if (((state>>t)&1) == 1)
		{
			count++;
		}
	}
	std::cout << count << "\n";
	// drawing doesn't have to be so fast! (compared to enumeration operations)
	for (int t = 0; t < 54; t++)
	{
		if (((state>>t)&1) == 1)
		{
			int x, y;
			Graphics::point p1, p2, p3;
			IndexToXY(t, x, y);
			GetCorners(x, y, p1, p2, p3);
			
			std::cout.precision(20);
			std::cout << std::fixed;

			std::cout << " " << p1 << " " << p2 << " " << p3 << "\t";
		}
	}
	std::cout << "\n";
}

/** Prints out the outer coorsinates of the board*/
void HexagonEnvironment::GenerateBoardBorder()
{
	// 1. Start with environment with only one piece type
	HexagonEnvironment e;
	
	std::cout << "Board\t";
//	// count how many objects we have
	int count = 54;
	std::cout << count << "\n";
	// drawing doesn't have to be so fast! (compared to enumeration operations)
	for (int t = 0; t < 54; t++)
	{
		int x, y;
		Graphics::point p1, p2, p3;
		IndexToXY(t, x, y);
		GetCorners(x, y, p1, p2, p3);
		
		std::cout.precision(20);
		std::cout << std::fixed;
		
		std::cout << " " << p1 << " " << p2 << " " << p3 << "\t";
	}
	std::cout << "\n";
}

void HexagonEnvironment::Draw(Graphics::Display &display) const
{
	
}

/** Draws available pieces and constraints */
void HexagonEnvironment::DrawSetup(Graphics::Display &display) const
{
	hex.DrawSetup(display);
}

void HexagonEnvironment::Draw(Graphics::Display &display, const HexagonSearchState &s) const
{
	display.FillSquare({0,0}, 1.0, Colors::white);
	for (int t = 0; t < 54; t++)
	{
		int x, y;
		Graphics::point p1, p2, p3;
		IndexToXY(t, x, y);
		GetCorners(x, y, p1, p2, p3);
		display.FillTriangle(p1, p2, p3, Colors::lightgray);
	}

	HexagonSearchState tmp;
	for (int i = 0; i < s.cnt; i++)
	{
		tmp.Reset();
		ApplyAction(tmp, s.state[i]);
		uint64_t state = tmp.bits;

		// drawing doesn't have to be so fast! (compared to enumeration operations)
		for (int t = 0; t < 54; t++)
		{
			if (((state>>t)&1) == 1)
			{
				int x, y;
				Graphics::point p1, p2, p3;
				IndexToXY(t, x, y);
				GetCorners(x, y, p1, p2, p3);
				display.FillTriangle(p1, p2, p3, rgbColor::hsl((float)i/12, (i%2)?0.75f:0.25f, 0.5f));
			}
		}
	}
}

void HexagonEnvironment::IndexToXY(int index, int &x, int &y)  const
{
	switch (index)
	{
		case 0: x = 2; y = 0; return;
		case 1: x = 3; y = 0; return;
		case 2: x = 4; y = 0; return;
		case 3: x = 5; y = 0; return;
		case 4: x = 6; y = 0; return;
		case 5: x = 7; y = 0; return;
		case 6: x = 8; y = 0; return;

		case 7: x = 1; y = 1; return;
		case 8: x = 2; y = 1; return;
		case 9: x = 3; y = 1; return;
		case 10: x = 4; y = 1; return;
		case 11: x = 5; y = 1; return;
		case 12: x = 6; y = 1; return;
		case 13: x = 7; y = 1; return;
		case 14: x = 8; y = 1; return;
		case 15: x = 9; y = 1; return;

		case 16: x = 0; y = 2; return;
		case 17: x = 1; y = 2; return;
		case 18: x = 2; y = 2; return;
		case 19: x = 3; y = 2; return;
		case 20: x = 4; y = 2; return;
		case 21: x = 5; y = 2; return;
		case 22: x = 6; y = 2; return;
		case 23: x = 7; y = 2; return;
		case 24: x = 8; y = 2; return;
		case 25: x = 9; y = 2; return;
		case 26: x = 10; y = 2; return;

		case 27: x = 0; y = 3; return;
		case 28: x = 1; y = 3; return;
		case 29: x = 2; y = 3; return;
		case 30: x = 3; y = 3; return;
		case 31: x = 4; y = 3; return;
		case 32: x = 5; y = 3; return;
		case 33: x = 6; y = 3; return;
		case 34: x = 7; y = 3; return;
		case 35: x = 8; y = 3; return;
		case 36: x = 9; y = 3; return;
		case 37: x = 10; y = 3; return;

		case 38: x = 1; y = 4; return;
		case 39: x = 2; y = 4; return;
		case 40: x = 3; y = 4; return;
		case 41: x = 4; y = 4; return;
		case 42: x = 5; y = 4; return;
		case 43: x = 6; y = 4; return;
		case 44: x = 7; y = 4; return;
		case 45: x = 8; y = 4; return;
		case 46: x = 9; y = 4; return;

		case 47: x = 2; y = 5; return;
		case 48: x = 3; y = 5; return;
		case 49: x = 4; y = 5; return;
		case 50: x = 5; y = 5; return;
		case 51: x = 6; y = 5; return;
		case 52: x = 7; y = 5; return;
		case 53: x = 8; y = 5; return;
	}
	x=-1;y=-1;
}


void HexagonEnvironment::GetCorners(int x, int y, Graphics::point &p1, Graphics::point &p2, Graphics::point &p3) const
{
	return hex.GetCorners(x, y, p1, p2, p3);
}

bool HexagonEnvironment::GetBorder(int x, int y, int xoff, int yoff, Graphics::point &p1, Graphics::point &p2) const
{
	return hex.GetBorder(x, y, xoff, yoff, p1, p2);
}

bool HexagonEnvironment::Valid(int x, int y) const
{
	return hex.Valid(x, y);
}

//std::vector<rgbColor> pieceColors;

#pragma mark -
#pragma mark Hexagon Display Code
#pragma mark -


Hexagon::Hexagon()
{
	
}
Hexagon::~Hexagon()
{
	
}

void Hexagon::LoadSolution(const char *file, HexagonState &s)
{
	Load(file, s, true);
}

void Hexagon::LoadPuzzle(const char *file, HexagonState &s)
{
	Load(file, s, false);
}

void Hexagon::Load(const char *file, HexagonState &s, bool solution)
{
	s.state.FillMax();
	this->solution.state.FillMax();
	FILE *f = fopen(file, "r");
	if (f == 0)
	{
		printf("Error opening file for read\n");
		return;
	}
	int h, w;
	fscanf(f, "type triangle\n");
	int cnt = fscanf(f, "height %d\nwidth %d\nmap\n", &h, &w);
	if (cnt != 2)
	{
		printf("Error reading height/width from file\n");
		return;
	}
	if (h != 6 || w != 11)
	{
		printf("Invalid height/width from file. Requires 6/11.\n");
		return;
	}
	// read constraints (if any)
	char buffer[255];
	fgets(buffer, 255, f);
	if (strncmp("Constraint:", buffer, 11) != 0)
	{
		printf("Error: expected 'Constraint:'\n");
		return;
	}
	char *tmp;
	
	// must touch diagonally
	tmp = strcasestr(buffer, "Diag");
	if (tmp != NULL)
	{
		diagPieces.resize(3);
		sscanf(&tmp[5], "%d %d %d", &diagPieces[0], &diagPieces[1], &diagPieces[2]);
	}
	
	tmp = strcasestr(buffer, "NoFlip");

	if (tmp != NULL)
	{
		noFlipPieces.resize(3);
		sscanf(&tmp[7], "%d %d %d", &noFlipPieces[0], &noFlipPieces[1], &noFlipPieces[2]);
	}

	tmp = strcasestr(buffer, "NotTouch");

	if (tmp != NULL)
	{
		notTouchPieces.resize(3);
		sscanf(&tmp[9], "%d %d %d", &notTouchPieces[0], &notTouchPieces[1], &notTouchPieces[2]);
	}

	tmp = strcasestr(buffer, " Touch"); // Add space -- otherwise matches NotTouch

	if (tmp != NULL)
	{
		touchPieces.resize(3);
		sscanf(&tmp[7], "%d %d %d", &touchPieces[0], &touchPieces[1], &touchPieces[2]);
	}
	
	for (int x = 0; x < 66; x++)
	{
		char c = fgetc(f);
		while (isspace(c))
			c = fgetc(f);
		if (c == 'x')
		{
			s.state.Set(x, -1);
			this->solution.state.Set(x, -1);
		}
		else if (isalpha(c))
		{
			this->solution.state.Set(x, tolower(c)-'a');
			if (solution)
			{
				s.state.Set(x, tolower(c)-'a');
			}
			else {
				s.state.Set(x, 11);
			}
		}
		else if (isdigit(c))
		{
			this->solution.state.Set(x, tolower(c)-'0');
			s.state.Set(x, tolower(c)-'0');
		}
	}
	fclose(f);
//	this->solution = s;
	
	// Get all of the colors
	for (int y = 0; y < 6; y++)
	{
		for (int x = 0; x < 11; x++)
		{
			if (!Valid(x, y))
				continue;
			int piece = this->solution.state.Get(y*11+x);
			if (piece > pieceColors.size())
				pieceColors.resize(piece+1);
			pieceColors[piece] = rgbColor::hsl((piece)/11.0, (piece%2)?1.0:0.5, 0.5);
		}
	}
	for (auto i : diagPieces)
		pieceColors[i] = rgbColor::hsl((diagPieces[0])/11.0, (diagPieces[0]%2)?1.0:0.5, 0.5);
	for (auto i : noFlipPieces)
		pieceColors[i] = rgbColor::hsl((noFlipPieces[0])/11.0, (noFlipPieces[0]%2)?1.0:0.5, 0.5);
	for (auto i : touchPieces)
		pieceColors[i] = rgbColor::hsl((touchPieces[0])/11.0, (touchPieces[0]%2)?1.0:0.5, 0.5);
	for (auto i : notTouchPieces)
		pieceColors[i] = rgbColor::hsl((notTouchPieces[0])/11.0, (notTouchPieces[0]%2)?1.0:0.5, 0.5);
}

void Hexagon::GetSuccessors(const HexagonState &nodeID, std::vector<HexagonState> &neighbors) const
{
	
}

void Hexagon::GetActions(const HexagonState &nodeID, std::vector<HexagonAction> &actions) const
{
	
}

HexagonAction Hexagon::GetAction(const HexagonState &s1, const HexagonState &s2) const
{
	HexagonAction a;
	return a;
}

void Hexagon::ApplyAction(HexagonState &s, HexagonAction a) const
{
	
}

void Hexagon::GetNextState(const HexagonState &, HexagonAction , HexagonState &) const
{
	
}

bool Hexagon::InvertAction(HexagonAction &a) const
{
	return true;
}


uint64_t Hexagon::GetStateHash(const HexagonState &node) const
{
	return 0;
}

uint64_t Hexagon::GetActionHash(HexagonAction act) const
{
	return 0;
}

/** Goal Test if the goal is stored **/
bool Hexagon::GoalTest(const HexagonState &node) const
{
	return true;
}

void Hexagon::RotateCW(HexagonState &s) const
{
	
}


bool Hexagon::GetBorder(int x, int y, int xoff, int yoff, Graphics::point &p1, Graphics::point &p2) const
{
	const float triangleWidth = 2.0/7;
	const float triangleHeight = triangleWidth/1.154700538379252; // 2/sqrt(3)
	const float xMargin = triangleWidth/2;
	const float yMargin = (2.0-triangleHeight*6)/2;
//	const float triangleHeight = 2.0/7;
//	const float triangleWidth = 1.154700538379252*triangleHeight; // 2/sqrt(3)
//	const float margin = triangleHeight/2;
	
	if (0==(x+y)%2) // points up
	{
		if (yoff < 0)
		{
			return false;
		}
		else if (xoff > 0)
		{
			p1 = {
				-1+xMargin+(x+2)*triangleWidth/2,
				-1+yMargin+(y+1)*triangleHeight
			};
			p2 = {
				-1+xMargin+(x+1)*triangleWidth/2,
				-1+yMargin+(y)*triangleHeight
			};
		}
		else if (xoff < 0)
		{
			p1 = {
				-1+xMargin+x*triangleWidth/2,
				-1+yMargin+(y+1)*triangleHeight
			};
			p2 = {
				-1+xMargin+(x+1)*triangleWidth/2,
				-1+yMargin+(y)*triangleHeight
			};
		}
		else if (yoff > 0)
		{
			p1 = {
				-1+xMargin+x*triangleWidth/2,
				-1+yMargin+(y+1)*triangleHeight
			};
			p2 = {
				-1+xMargin+(x+2)*triangleWidth/2,
				-1+yMargin+(y+1)*triangleHeight
			};
		}
		return true;
	}
	else { // points down
		if (yoff > 0)
		{
			return false;
		}
		else if (xoff > 0)
		{
			// upper right
			p1 = {
				-1+xMargin+(x+2)*triangleWidth/2,
				-1+yMargin+(y)*triangleHeight
			};
			// bottom tip
			p2 = {
				-1+xMargin+(x+1)*triangleWidth/2,
				-1+yMargin+(y+1)*triangleHeight
			};
		}
		else if (xoff < 0)
		{
			// upper left
			p1 = {
				-1+xMargin+(x)*triangleWidth/2,
				-1+yMargin+(y)*triangleHeight
			};
			// bottom tip
			p2 = {
				-1+xMargin+(x+1)*triangleWidth/2,
				-1+yMargin+(y+1)*triangleHeight
			};
		}
		else if (yoff < 0)
		{
			// upper right
			p1 = {
				-1+xMargin+(x+2)*triangleWidth/2,
				-1+yMargin+(y)*triangleHeight
			};
			// upper left
			p2 = {
				-1+xMargin+(x)*triangleWidth/2,
				-1+yMargin+(y)*triangleHeight
			};
		}
		return true;
//		// upper right
//		p1 = {
//			-1+margin+(x+2)*triangleHeight/2,
//			-1+margin+(y)*triangleHeight
//		};
//		// upper left
//		p2 = {
//			-1+margin+(x)*triangleHeight/2,
//			-1+margin+(y)*triangleHeight
//		};
//		// bottom tip
//		p3 = {
//			-1+margin+(x+1)*triangleHeight/2,
//			-1+margin+(y+1)*triangleHeight
//		};
//		return false;
	}
}

void Hexagon::GetCorners(int x, int y, Graphics::point &p1, Graphics::point &p2, Graphics::point &p3) const
{
	const float triangleWidth = 2.0/7;
	const float triangleHeight = triangleWidth/1.154700538379252; // 2/sqrt(3)
	const float xMargin = triangleWidth/2;
	const float yMargin = (2.0-triangleHeight*6)/2;

	if (0==(x+y)%2) // points up
	{
		p1 = {
			-1+xMargin+x*triangleWidth/2,
			-1+yMargin+(y+1)*triangleHeight
		};
		p2 = {
			-1+xMargin+(x+2)*triangleWidth/2,
			-1+yMargin+(y+1)*triangleHeight
		};
		p3 = {
			-1+xMargin+(x+1)*triangleWidth/2,
			-1+yMargin+(y)*triangleHeight
		};
	}
	else { // points down
		p1 = {
			-1+xMargin+(x+2)*triangleWidth/2,
			-1+yMargin+(y)*triangleHeight
		};
		p2 = {
			-1+xMargin+(x)*triangleWidth/2,
			-1+yMargin+(y)*triangleHeight
		};
		p3 = {
			-1+xMargin+(x+1)*triangleWidth/2,
			-1+yMargin+(y+1)*triangleHeight
		};
	}
}

bool Hexagon::Valid(int x, int y) const
{
	static bool valid[6][11] =
	{
		{false, false, true, true, true, true, true, true, true, false, false},
		{false, true, true, true, true, true, true, true, true, true, false},
		{true, true, true, true, true, true, true, true, true, true, true},
		{true, true, true, true, true, true, true, true, true, true, true},
		{false, true, true, true, true, true, true, true, true, true, false},
		{false, false, true, true, true, true, true, true, true, false, false}
	};
	return valid[y][x];
}

void Hexagon::Draw(Graphics::Display &display) const
{
	// Draw board
	display.FillRect(Graphics::rect({0,0}, 1.0), Colors::white);
	for (int y = 0; y < 6; y++)
	{
		for (int x = 0; x < 11; x++)
		{
			if (!Valid(x, y))
				//if (!valid[y][x])
				continue;
			Graphics::point p1, p2, p3;
			GetCorners(x, y, p1, p2, p3);
			display.FillTriangle(p1, p2, p3, Colors::lightgray);
			display.FrameTriangle(p1, p2, p3, 0.01f, Colors::darkgray);
		}
	}
	float xLoc = -1, yLoc = -1;
	float dim = 0.1;
	
	if (noFlipPieces.size() != 0)
	{
		display.FillRect({xLoc, yLoc, xLoc+dim, yLoc+dim}, pieceColors[noFlipPieces[0]]);
		display.DrawText("Cannot Flip", {xLoc+1.2f*dim, yLoc+dim/2.0f}, Colors::black, dim*0.8f, Graphics::textAlignLeft, Graphics::textBaselineMiddle);
		xLoc = 0;
	}
	if (diagPieces.size() != 0)
	{
		display.FillRect({xLoc, yLoc, xLoc+dim, yLoc+dim}, pieceColors[diagPieces[0]]);
		display.DrawText("Must Touch Diagonally", {xLoc+1.2f*dim, yLoc+dim/2.0f}, Colors::black, dim*0.8f, Graphics::textAlignLeft, Graphics::textBaselineMiddle);
		xLoc = 0;
	}
	if (notTouchPieces.size() != 0)
	{
		display.FillRect({xLoc, yLoc, xLoc+dim, yLoc+dim}, pieceColors[notTouchPieces[0]]);
		display.DrawText("Cannot Touch", {xLoc+1.2f*dim, yLoc+dim/2.0f}, Colors::black, dim*0.8f, Graphics::textAlignLeft, Graphics::textBaselineMiddle);
		xLoc = 0;
	}
	if (touchPieces.size() != 0)
	{
		display.FillRect({xLoc, yLoc, xLoc+dim, yLoc+dim}, pieceColors[touchPieces[0]]);
		display.DrawText("Must Touch Edges", {xLoc+1.2f*dim, yLoc+dim/2.0f}, Colors::black, dim*0.8f, Graphics::textAlignLeft, Graphics::textBaselineMiddle);
		xLoc = 0;
	}
}

void Hexagon::DrawSetup(Graphics::Display &display) const
{
	// Draw board
	display.FillRect(Graphics::rect({0,0}, 1.0), Colors::white);

	// Horribly inefficient for now; just looking for correctness
	// Could all be pre-computed and cached
	for (int piece = 0; piece < 10; piece++)
	{
		bool found = false;
		Graphics::rect bounds;
		// get bounding box for piece
		for (int y = 0; y < 6; y++)
		{
			for (int x = 0; x < 11; x++)
			{
				if (!Valid(x, y))
					continue;
				int p = solution.state.Get(y*11+x);
				if (piece == p)
				{
					Graphics::point p1, p2, p3;
					GetCorners(x, y, p1, p2, p3);

					if (!found)
					{
						found = true;
						bounds = Graphics::rect(p1, p2);
					}
					else {
						bounds |= Graphics::rect(p1, p2);
					}
					bounds |= Graphics::rect(p2, p1);
					bounds |= Graphics::rect(p2, p3);
					bounds |= Graphics::rect(p3, p2);
					bounds |= Graphics::rect(p1, p3);
					bounds |= Graphics::rect(p3, p1);
				}
			}
		}

		float newWidth = 0.45f;
		float newHeight = 0.45f;
		float border = 0.01;
		float scale = 0.45f/std::max(bounds.right-bounds.left, bounds.bottom-bounds.top);
		float baseX = -0.9f+(piece%4)*(newWidth+border);
		float baseY = -0.7f+(piece>>2)*(newHeight+border);
//		std::cout << "Piece " << piece << " rect: " << bounds << "\n";
		// draw piece scaled
		for (int y = 0; y < 6; y++)
		{
			for (int x = 0; x < 11; x++)
			{
				if (!Valid(x, y))
					continue;
				int p = solution.state.Get(y*11+x);
				if (piece == p)
				{
					Graphics::point p1, p2, p3;
					GetCorners(x, y, p1, p2, p3);
					p1.x = baseX+(p1.x-bounds.left)*scale;
					p1.y = baseY+(p1.y-bounds.top)*scale;
					p2.x = baseX+(p2.x-bounds.left)*scale;
					p2.y = baseY+(p2.y-bounds.top)*scale;
					p3.x = baseX+(p3.x-bounds.left)*scale;
					p3.y = baseY+(p3.y-bounds.top)*scale;
					display.FillTriangle(p1, p2, p3, pieceColors[piece]);
				}
			}
		}
	}
	
	// Draw constraints at top
	{
		float xLoc = -1, yLoc = -1;
		float dim = 0.1;
		
		if (noFlipPieces.size() != 0)
		{
			display.FillRect({xLoc, yLoc, xLoc+dim, yLoc+dim}, pieceColors[noFlipPieces[0]]);
			display.DrawText("Cannot Flip", {xLoc+1.2f*dim, yLoc+dim/2.0f}, Colors::black, dim*0.8f, Graphics::textAlignLeft, Graphics::textBaselineMiddle);
			xLoc = 0;
		}
		if (diagPieces.size() != 0)
		{
			display.FillRect({xLoc, yLoc, xLoc+dim, yLoc+dim}, pieceColors[diagPieces[0]]);
			display.DrawText("Must Touch Diagonally", {xLoc+1.2f*dim, yLoc+dim/2.0f}, Colors::black, dim*0.8f, Graphics::textAlignLeft, Graphics::textBaselineMiddle);
			xLoc = 0;
		}
		if (notTouchPieces.size() != 0)
		{
			display.FillRect({xLoc, yLoc, xLoc+dim, yLoc+dim}, pieceColors[notTouchPieces[0]]);
			display.DrawText("Cannot Touch", {xLoc+1.2f*dim, yLoc+dim/2.0f}, Colors::black, dim*0.8f, Graphics::textAlignLeft, Graphics::textBaselineMiddle);
			xLoc = 0;
		}
		if (touchPieces.size() != 0)
		{
			display.FillRect({xLoc, yLoc, xLoc+dim, yLoc+dim}, pieceColors[touchPieces[0]]);
			display.DrawText("Must Touch Edges", {xLoc+1.2f*dim, yLoc+dim/2.0f}, Colors::black, dim*0.8f, Graphics::textAlignLeft, Graphics::textBaselineMiddle);
			xLoc = 0;
		}
	}
}

void Hexagon::Draw(Graphics::Display &display, const HexagonState &s) const
{
	// Draw objects
	for (int y = 0; y < 6; y++)
	{
		for (int x = 0; x < 11; x++)
		{
			if (!Valid(x, y))
				continue;
			Graphics::point p1, p2, p3;
			GetCorners(x, y, p1, p2, p3);
			int piece = s.state.Get(y*11+x);
			if (piece < 10)
			{
				display.FillTriangle(p1, p2, p3, pieceColors[piece]);
			}
		}
	}
	// Draw Borders
	for (int y = 0; y < 6; y++)
	{
		for (int x = 0; x < 11; x++)
		{
			if (!Valid(x, y))
				//if (!valid[y][x])
				continue;
			Graphics::point p1, p2, p3;
			GetCorners(x, y, p1, p2, p3);
			int piece = s.state.Get(y*11+x);

			if ((x == 10) || (piece != s.state.Get(y*11+x+1)))
			{
				if (GetBorder(x, y, 1, 0, p1, p2))
				{
					display.DrawLine(p1, p2, 0.02, Colors::black);
				}
			}
			if ((x == 0) || (x > 0 && piece != s.state.Get(y*11+x-1)))
			{
				if (GetBorder(x, y, -1, 0, p1, p2))
				{
					display.DrawLine(p1, p2, 0.02, Colors::black);
				}
			}
			if ((y == 5) || (y < 5 && piece != s.state.Get((y+1)*11+x)))
			{
				if (GetBorder(x, y, 0, 1, p1, p2))
				{
					display.DrawLine(p1, p2, 0.02, Colors::black);
				}
			}
			if ((y == 0) || (y > 0 && piece != s.state.Get((y-1)*11+x)))
			{
				if (GetBorder(x, y, 0, -1, p1, p2))
				{
					display.DrawLine(p1, p2, 0.02, Colors::black);
				}
			}
		}
	}
}

