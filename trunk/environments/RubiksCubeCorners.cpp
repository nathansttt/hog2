//
//  RubiksCorner.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/11/13.
//  Copyright (c) 2013 University of Denver. All rights reserved.
//

#include "RubiksCubeCorners.h"
#include "GLUtil.h"
#include <assert.h>

#define LINEAR_RANK 1
#define POLYNOMIAL_RANK 0

/**
 *
 * Implementation details:
 *
 * This just implements the 8 Corners.
 * The RubiksCornerState has 24*4 = 48 bits for where each face is located
 *
 * Faces are numbered 0...5 as follows.
 *
 *  +-------+
 *  |\       \
 *  | \   0   \
 *  |  \_______\
 *  | 1|       |
 *  \  |   2   |
 *   \ |       |
 *    \|_______|
 *
 *
 *  +-------+
 *  |       |\
 *  |   4   | \
 *  |       |  \
 *  |_______|3 |
 *  \       \  |
 *   \   5   \ |
 *    \_______\|
 *
 *
 * Corners are numbered as follows:
 *
 *
 *  4-------3
 *  |\       \
 *  | \       \
 *  |  \1_____2\
 *  8  |       |
 *  \  |       |
 *   \ |       |
 *    \5_______6
 *
 *
 *  4-------3
 *  |       |\
 *  |       | \
 *  |       |  2
 *  8_______7  |
 *  \       \  |
 *   \       \ |
 *    \5______6|
 *
 *
 * The moves for each face are labeled with comments. There are three legal
 * moves which rotate +90, -90 and +/-180.
 *
 * The rotation starts with the top/bottm face and goes counter-clockwise
 *
 */


inline int get(uint64_t state, int whichLoc)
{
	//return (state>>(4*whichLoc))&0xF;
	return (state>>((whichLoc<<2)))&0xF;
}

inline void set(uint64_t &state, int whichLoc, int cube)
{
	const uint64_t blank = 0xF;
	uint64_t value = cube;//&0xF;
	state = state&(~(blank<<((whichLoc<<2))));
	state |= (value<<((whichLoc<<2)));
}

inline void swap(uint64_t &state, int loc1, int loc2)
{
	loc1<<=2;
	loc2<<=2;
	uint64_t val1 = (state>>(loc1))&0xF;
	uint64_t val2 = (state>>(loc2))&0xF;
	const uint64_t blank = 0xF;
	uint64_t mask = (blank<<(loc1))|(blank<<(loc2));
	state = state&(~mask);
	state = state|(val1<<(loc2))|(val2<<(loc1));
}


void RubiksCornerState::Rotate(uint64_t a, uint64_t b, uint64_t c, uint64_t d)
{
	a = (a<<2) + 16; // multiply by 4 for shifting purposes; add 16 for rotations
	b = (b<<2) + 16;
	c = (c<<2) + 16;
	d = (d<<2) + 16;
	
	const uint64_t blank = 0xF;
	
	uint64_t mask;
	uint64_t bits;
	bits =
	(((state>>a)&blank)<<b)|(((state>>b)&blank)<<c)|
	(((state>>c)&blank)<<d)|(((state>>d)&blank)<<a);
	mask = (blank<<(a))|(blank<<(b))|(blank<<(c))|(blank<<(d));
	state = (state&(~mask))|bits;
}

void RubiksCornerState::Swap(uint64_t a, uint64_t b, uint64_t c, uint64_t d)
{
	a = (a<<2) + 16; // multiply by 4 for shifting purposes; add 16 for rotations
	b = (b<<2) + 16;
	c = (c<<2) + 16;
	d = (d<<2) + 16;
	
	const uint64_t blank = 0xF;
	
	uint64_t mask;
	uint64_t bits;
	bits =
	(((state>>a)&blank)<<b)|(((state>>b)&blank)<<a)|
	(((state>>c)&blank)<<d)|(((state>>d)&blank)<<c);
	mask = (blank<<(a))|(blank<<(b))|(blank<<(c))|(blank<<(d));
	state = (state&(~mask))|bits;
}

void RubiksCorner::GetSuccessors(const RubiksCornerState &nodeID, std::vector<RubiksCornerState> &neighbors) const
{
	RubiksCornerState s;
	for (int x = 0; x < 18; x++)
	{
		GetNextState(nodeID, x, s);
		neighbors.push_back(s);
	}
}

void RubiksCorner::GetActions(const RubiksCornerState &nodeID, std::vector<RubiksCornersAction> &actions) const
{
	actions.resize(0);
	for (int x = 0; x < 18; x++)
	{
		actions.push_back(x);
	}
}

RubiksCornersAction RubiksCorner::GetAction(const RubiksCornerState &s1, const RubiksCornerState &s2) const
{
	assert(false);
	RubiksCornersAction a;
	return a;
}

void RubiksCorner::ApplyAction(RubiksCornerState &s, RubiksCornersAction a) const
{
	switch (a)
	{
		case 0: // face 0
		{
			s.Rotate(0, 1, 2, 3);
		}
			break;
		case 1:
		{
			s.Rotate(3, 2, 1, 0);
		}
			break;
		case 2:
		{
			s.Swap(0, 2, 1, 3);
		}
			break;
		case 3: // face 5
		{
			s.Rotate(4, 5, 6, 7);
		}
			break;
		case 4:
		{
			s.Rotate(7, 6, 5, 4);
		}
			break;
		case 5:
		{
			s.Swap(4, 6, 5, 7);
		}
			break;
			
		case 6: // face 2
		{
			s.Rotate(0, 1, 5, 4);
			s.SetCubeOrientation(s.GetCubeInLoc(1), (s.GetCubeOrientation(s.GetCubeInLoc(1))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(5), (s.GetCubeOrientation(s.GetCubeInLoc(5))+1)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(4), (s.GetCubeOrientation(s.GetCubeInLoc(4))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(0), (s.GetCubeOrientation(s.GetCubeInLoc(0))+1)%3);
		}
			break;
		case 7:
		{
			s.Rotate(4, 5, 1, 0);
			s.SetCubeOrientation(s.GetCubeInLoc(1), (s.GetCubeOrientation(s.GetCubeInLoc(1))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(5), (s.GetCubeOrientation(s.GetCubeInLoc(5))+1)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(4), (s.GetCubeOrientation(s.GetCubeInLoc(4))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(0), (s.GetCubeOrientation(s.GetCubeInLoc(0))+1)%3);
		}
			break;
		case 8:
		{
			s.Swap(0, 5, 1, 4);
		}
			break;
			
		case 9: // face 4
		{
			s.Rotate(2, 3, 7, 6);
			s.SetCubeOrientation(s.GetCubeInLoc(3), (s.GetCubeOrientation(s.GetCubeInLoc(3))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(2), (s.GetCubeOrientation(s.GetCubeInLoc(2))+1)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(6), (s.GetCubeOrientation(s.GetCubeInLoc(6))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(7), (s.GetCubeOrientation(s.GetCubeInLoc(7))+1)%3);
		}
			break;
		case 10:
		{
			s.Rotate(6, 7, 3, 2);
			s.SetCubeOrientation(s.GetCubeInLoc(3), (s.GetCubeOrientation(s.GetCubeInLoc(3))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(2), (s.GetCubeOrientation(s.GetCubeInLoc(2))+1)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(6), (s.GetCubeOrientation(s.GetCubeInLoc(6))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(7), (s.GetCubeOrientation(s.GetCubeInLoc(7))+1)%3);
		}
			break;
		case 11:
		{
			s.Swap(6, 3, 2, 7);
		}
			break;
			
		case 12: // face 1
		{
			s.Rotate(0, 4, 7, 3);
			s.SetCubeOrientation(s.GetCubeInLoc(0), (s.GetCubeOrientation(s.GetCubeInLoc(0))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(4), (s.GetCubeOrientation(s.GetCubeInLoc(4))+1)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(7), (s.GetCubeOrientation(s.GetCubeInLoc(7))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(3), (s.GetCubeOrientation(s.GetCubeInLoc(3))+1)%3);
		}
			break;
		case 13:
		{
			s.Rotate(3, 7, 4, 0);
			s.SetCubeOrientation(s.GetCubeInLoc(0), (s.GetCubeOrientation(s.GetCubeInLoc(0))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(4), (s.GetCubeOrientation(s.GetCubeInLoc(4))+1)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(7), (s.GetCubeOrientation(s.GetCubeInLoc(7))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(3), (s.GetCubeOrientation(s.GetCubeInLoc(3))+1)%3);
		}
			break;
		case 14:
		{
			s.Swap(0, 7, 4, 3);
		}
			break;
		case 15: // face 3
		{
			s.Rotate(1, 2, 6, 5);
			s.SetCubeOrientation(s.GetCubeInLoc(2), (s.GetCubeOrientation(s.GetCubeInLoc(2))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(6), (s.GetCubeOrientation(s.GetCubeInLoc(6))+1)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(5), (s.GetCubeOrientation(s.GetCubeInLoc(5))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(1), (s.GetCubeOrientation(s.GetCubeInLoc(1))+1)%3);
		}
			break;
		case 16:
		{
			s.Rotate(5, 6, 2, 1);
			s.SetCubeOrientation(s.GetCubeInLoc(2), (s.GetCubeOrientation(s.GetCubeInLoc(2))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(6), (s.GetCubeOrientation(s.GetCubeInLoc(6))+1)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(5), (s.GetCubeOrientation(s.GetCubeInLoc(5))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(1), (s.GetCubeOrientation(s.GetCubeInLoc(1))+1)%3);
		}
			break;
		case 17:
		{
			s.Swap(1, 6, 2, 5);
		}
			break;
		default:
			break;
	}
	
}

void RubiksCorner::GetNextState(const RubiksCornerState &s0, RubiksCornersAction a, RubiksCornerState &s1) const
{
	s1 = s0;
	ApplyAction(s1, a);
}


/** Goal Test if the goal is stored **/
bool RubiksCorner::GoalTest(const RubiksCornerState &node)
{
	return node.state == 0;
}

void RubiksCorner::ApplyMove(RubiksCornerState &s, RubikCornerMove *a)
{
	ApplyAction(s, a->act);
}
void RubiksCorner::UndoMove(RubiksCornerState &s, RubikCornerMove *a)
{
	RubiksCornersAction todo = a->act;
	if (0 == todo%3)
	{
		todo += 1;
	}
	else if (1 == todo%3)
	{
		todo -= 1;
	}
	ApplyAction(s, todo);
}

int64_t RubiksCorner::getMaxSinglePlayerRank()
{
	//	static uint64_t Factorial[21] =
	//	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
	//		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
	//		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
	//
	//	return Factorial[8]*(2187); // 3^7
	return 88179840;
}
int64_t RubiksCorner::getMaxSinglePlayerRank2()
{
	return 9;
}
int64_t RubiksCorner::getMaxSinglePlayerRank2(int64_t firstIndex)
{
	return 9797760;
}

void RubiksCorner::rankPlayerFirstTwo(const RubiksCornerState &s, int who, int64_t &rank)
{
	rank = s.GetCubeOrientation(0)*3+s.GetCubeOrientation(1);
}

void RubiksCorner::rankPlayerRemaining(const RubiksCornerState &node, int who, int64_t &rank)
{
	static uint64_t Factorial[21] =
	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
#if POLYNOMIAL_RANK == 1
	
	int puzzle[12];
	for (int x = 0; x < 8; x++)
		puzzle[x] = node.GetCubeInLoc(x);
	
	uint64_t hashVal = 0;
	uint64_t part2 = 0;
	int numEntriesLeft = 8;
	for (unsigned int x = 0; x < 8; x++)
	{
		hashVal += puzzle[x]*Factorial[numEntriesLeft-1];
		numEntriesLeft--;
		for (unsigned y = x; y < 8; y++)
		{
			if (puzzle[y] > puzzle[x])
				puzzle[y]--;
		}
	}
	for (int x = 2; x < 7; x++)
	{
		part2 = part2*3+node.GetCubeOrientation(x);
	}
	rank = part2*Factorial[8]+hashVal;
#endif
#if LINEAR_RANK == 1
	uint64_t perm = 0, dual = 0;
	for (int x = 0; x < 8; x++)
	{
		set(perm, x, node.GetCubeInLoc(x));
		set(dual, node.GetCubeInLoc(x), x);
	}
	
	uint64_t hashVal = 0;
	for (int x = 2; x < 7; x++)
	{
		hashVal = hashVal*3+node.GetCubeOrientation(x);
	}
	hashVal = hashVal*Factorial[8]+MRRank(8, perm, dual);
	rank = hashVal;
#endif
}


uint64_t RubiksCorner::GetStateHash(const RubiksCornerState &node) const
{
	static uint64_t Factorial[21] =
	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
	
#if POLYNOMIAL_RANK == 1
	int puzzle[12];
	for (int x = 0; x < 8; x++)
		puzzle[x] = node.GetCubeInLoc(x);
	
	uint64_t hashVal = 0;
	uint64_t part2 = 0;
	int numEntriesLeft = 8;
	for (unsigned int x = 0; x < 8; x++)
	{
		hashVal += puzzle[x]*Factorial[numEntriesLeft-1];
		numEntriesLeft--;
		for (unsigned y = x; y < 8; y++)
		{
			if (puzzle[y] > puzzle[x])
				puzzle[y]--;
		}
	}
	for (int x = 0; x < 7; x++)
	{
		part2 = part2*3+node.GetCubeOrientation(x);
	}
	return part2*Factorial[8]+hashVal;
	//	return hashVal;
#endif
#if LINEAR_RANK == 1
	uint64_t perm = 0, dual = 0;
	for (int x = 0; x < 8; x++)
	{
		int val = node.GetCubeInLoc(x);
		set(perm, x, val);
		set(dual, val, x);
	}
	
	uint64_t hashVal = 0;
	for (int x = 0; x < 7; x++)
	{
		hashVal = hashVal*3+node.GetCubeOrientation(x);
	}
	//	return (MRRank(12, perm, dual)<<11)|hashVal;
	hashVal = hashVal*Factorial[8]+MRRank(8, perm, dual);
	return hashVal;
#endif
}

/////

uint64_t RubiksCorner::MRRank(int n, uint64_t perm, uint64_t dual) const
{
	int ss[12];
	int ssLoc = 0;
	
	int s;
	for (int i = n; i > 1; i--)
	{
		s = get(perm, i-1);
		ss[ssLoc] = s;
		ssLoc++;
		
		swap(perm, i-1, get(dual, i-1));
		swap(dual, s, i-1);
	}
	uint64_t result = 0;
	int cnt = 2;
	for (int i = (int)ssLoc-1; i >= 0; i--)
	{
		result *= cnt;
		result += ss[i];
		cnt++;
	}
	return result;
}


/////


void RubiksCorner::GetStateFromHash(uint64_t hash, RubiksCornerState &node) const
{
	static uint64_t Factorial[21] =
	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
#if POLYNOMIAL_RANK == 1
	
	int puzzle[12];
	uint64_t hashVal = hash;
	hash /= Factorial[8]; // for rotations
	hashVal = hashVal%Factorial[8]; // for pieces
	
	int cnt = 0;
	for (int x = 6; x >= 0; x--)
	{
		node.SetCubeOrientation(x, hash%3);
		cnt += hash%3;
		hash/=3;
	}
	node.SetCubeOrientation(7, 2-(cnt%3));
	
	int numEntriesLeft = 1;
	for (int x = 8-1; x >= 0; x--)
	{
		puzzle[x] = hashVal%numEntriesLeft;
		hashVal /= numEntriesLeft;
		numEntriesLeft++;
		for (int y = x+1; y < 8; y++)
		{
			if (puzzle[y] >= puzzle[x])
				puzzle[y]++;
		}
	}
	for (int x = 0; x < 8; x++)
		node.SetCubeInLoc(x, puzzle[x]);
#endif
#if LINEAR_RANK == 1
	uint64_t bits = hash/Factorial[8];
	uint64_t hVal = hash%Factorial[8];
	
	int cnt = 0;
	for (int x = 6; x >= 0; x--)
	{
		node.SetCubeOrientation(x, bits%3);
		cnt += bits%3;
		bits/=3;
	}
	node.SetCubeOrientation(7, 2-(cnt%3));
	
	uint64_t val = 0;
	for (int x = 0; x < 8; x++)
		set(val, x, x);
	MRUnrank2(8, hVal, val);
	for (int x = 0; x < 8; x++)
	{
		node.SetCubeInLoc(x, get(val, x));
	}
#endif
}

void RubiksCorner::MRUnrank2(int n, uint64_t r, uint64_t &perm) const
{
	for (int i = n; i > 0; i--)
	{
		swap(perm, i-1, r%i);
		r = r/i;
	}
}


void RubiksCorner::OpenGLDraw() const
{
	
}

void RubiksCorner::OpenGLDraw(const RubiksCornerState&s) const
{
	float scale = 0.3;
	float offset = 2.0*scale/3.0;
	glBegin(GL_QUADS);
	// top
	//// Face 0
	//SetCubeColor(0);
	// 0 - top
	SetFaceColor(0, s);
	glVertex3f(-scale, -scale, -scale+offset);
	glVertex3f(-scale, -scale, -scale);
	glVertex3f(-scale+offset, -scale, -scale);
	glVertex3f(-scale+offset, -scale, -scale+offset);

	// 9 - top
	SetFaceColor(9, s);
	glVertex3f(-scale, -scale, scale);
	glVertex3f(-scale, -scale, scale-offset);
	glVertex3f(-scale+offset, -scale, scale-offset);
	glVertex3f(-scale+offset, -scale, scale);

	// 3 - top
	SetFaceColor(3, s);
	glVertex3f(scale-offset, -scale, -scale+offset);
	glVertex3f(scale-offset, -scale, -scale);
	glVertex3f(scale, -scale, -scale);
	glVertex3f(scale, -scale, -scale+offset);

	// 6 - top
	SetFaceColor(6, s);
	glVertex3f(scale-offset, -scale, scale);
	glVertex3f(scale-offset, -scale, scale-offset);
	glVertex3f(scale, -scale, scale-offset);
	glVertex3f(scale, -scale, scale);
	
	//// Face 5
	// 12 - top
	SetFaceColor(12, s);
	glVertex3f(-scale, scale, -offset/2.0);
	glVertex3f(-scale, scale, -3*offset/2.0);
	glVertex3f(-scale+offset, scale, -3*offset/2.0);
	glVertex3f(-scale+offset, scale, -offset/2.0);
	
	// 21 - top
	SetFaceColor(21, s);
	glVertex3f(-scale, scale, 3*offset/2.0);
	glVertex3f(-scale, scale, offset/2.0);
	glVertex3f(-scale+offset, scale, offset/2.0);
	glVertex3f(-scale+offset, scale, 3*offset/2.0);
	
	// 15 - top
	SetFaceColor(15, s);
	glVertex3f(scale-offset, scale, -scale+offset);
	glVertex3f(scale-offset, scale, -scale);
	glVertex3f(scale, scale, -scale);
	glVertex3f(scale, scale, -scale+offset);
	
	// 18 - top
	SetFaceColor(18, s);
	glVertex3f(scale-offset, scale, scale);
	glVertex3f(scale-offset, scale, scale-offset);
	glVertex3f(scale, scale, scale-offset);
	glVertex3f(scale, scale, scale);

	//// Face 1
	// 1 - side
	SetFaceColor(1, s);
	glVertex3f(-scale, -scale+offset, -scale);
	glVertex3f(-scale, -scale+offset, -scale+offset);
	glVertex3f(-scale, -scale, -scale+offset);
	glVertex3f(-scale, -scale, -scale);

	SetFaceColor(11, s);
	glVertex3f(-scale, -scale+offset, scale);
	glVertex3f(-scale, -scale+offset, scale-offset);
	glVertex3f(-scale, -scale, scale-offset);
	glVertex3f(-scale, -scale, scale);

	SetFaceColor(14, s);
	glVertex3f(-scale, scale-offset, -scale);
	glVertex3f(-scale, scale-offset, -scale+offset);
	glVertex3f(-scale, scale, -scale+offset);
	glVertex3f(-scale, scale, -scale);

	SetFaceColor(22, s);
	glVertex3f(-scale, scale-offset, scale);
	glVertex3f(-scale, scale-offset, scale-offset);
	glVertex3f(-scale, scale, scale-offset);
	glVertex3f(-scale, scale, scale);

	//// Face 3
	SetFaceColor(5, s);
	glVertex3f(scale, -scale+offset, -scale);
	glVertex3f(scale, -scale+offset, -scale+offset);
	glVertex3f(scale, -scale, -scale+offset);
	glVertex3f(scale, -scale, -scale);
	
	SetFaceColor(7, s);
	glVertex3f(scale, -scale+offset, scale);
	glVertex3f(scale, -scale+offset, scale-offset);
	glVertex3f(scale, -scale, scale-offset);
	glVertex3f(scale, -scale, scale);
	
	SetFaceColor(16, s);
	glVertex3f(scale, scale-offset, -scale);
	glVertex3f(scale, scale-offset, -scale+offset);
	glVertex3f(scale, scale, -scale+offset);
	glVertex3f(scale, scale, -scale);
	
	SetFaceColor(20, s);
	glVertex3f(scale, scale-offset, scale);
	glVertex3f(scale, scale-offset, scale-offset);
	glVertex3f(scale, scale, scale-offset);
	glVertex3f(scale, scale, scale);


//	//// Face 2
	SetFaceColor(2, s);
	glVertex3f(-scale, -scale+offset, -scale);
	glVertex3f(-scale+offset, -scale+offset, -scale);
	glVertex3f(-scale+offset, -scale, -scale);
	glVertex3f(-scale, -scale, -scale);

	SetFaceColor(4, s);
	glVertex3f(scale, -scale+offset, -scale);
	glVertex3f(scale-offset, -scale+offset, -scale);
	glVertex3f(scale-offset, -scale, -scale);
	glVertex3f(scale, -scale, -scale);

	SetFaceColor(13, s);
	glVertex3f(-scale, scale-offset, -scale);
	glVertex3f(-scale+offset, scale-offset, -scale);
	glVertex3f(-scale+offset, scale, -scale);
	glVertex3f(-scale, scale, -scale);

	SetFaceColor(17, s);
	glVertex3f(scale, scale-offset, -scale);
	glVertex3f(scale-offset, scale-offset, -scale);
	glVertex3f(scale-offset, scale, -scale);
	glVertex3f(scale, scale, -scale);


	//// Face 4
	SetFaceColor(10, s);
	glVertex3f(-scale, -scale+offset, scale);
	glVertex3f(-scale+offset, -scale+offset, scale);
	glVertex3f(-scale+offset, -scale, scale);
	glVertex3f(-scale, -scale, scale);
	
	SetFaceColor(8, s);
	glVertex3f(scale, -scale+offset, scale);
	glVertex3f(scale-offset, -scale+offset, scale);
	glVertex3f(scale-offset, -scale, scale);
	glVertex3f(scale, -scale, scale);
	
	SetFaceColor(23, s);
	glVertex3f(-scale, scale-offset, scale);
	glVertex3f(-scale+offset, scale-offset, scale);
	glVertex3f(-scale+offset, scale, scale);
	glVertex3f(-scale, scale, scale);
	
	SetFaceColor(19, s);
	glVertex3f(scale, scale-offset, scale);
	glVertex3f(scale-offset, scale-offset, scale);
	glVertex3f(scale-offset, scale, scale);
	glVertex3f(scale, scale, scale);

	
//	//// Face 2
//	SetCubeColor(3, true, s);
//	// 3 - side
//	glVertex3f(-offset/2.0, -scale+offset, -scale);
//	glVertex3f(offset/2.0, -scale+offset, -scale);
//	glVertex3f(offset/2.0, -scale, -scale);
//	glVertex3f(-offset/2.0, -scale, -scale);
//	
//	SetCubeColor(10, false, s);
//	// 10 - side
//	glVertex3f(-offset/2.0, scale-offset, -scale);
//	glVertex3f(offset/2.0, scale-offset, -scale);
//	glVertex3f(offset/2.0, scale, -scale);
//	glVertex3f(-offset/2.0, scale, -scale);
//	
//	SetCubeColor(2, true, s);
//	// 2 - side
//	glVertex3f(-scale+offset, -offset/2.0, -scale);
//	glVertex3f(-scale+offset, offset/2.0, -scale);
//	glVertex3f(-scale, offset/2.0, -scale);
//	glVertex3f(-scale, -offset/2.0, -scale);
//	
//	SetCubeColor(4, false, s);
//	// 4 - side
//	glVertex3f(scale-offset, -offset/2.0, -scale);
//	glVertex3f(scale-offset, offset/2.0, -scale);
//	glVertex3f(scale, offset/2.0, -scale);
//	glVertex3f(scale, -offset/2.0, -scale);
//	
//	
//	//// Face 4
//	SetCubeColor(7, true, s);
//	// 7 - side
//	glVertex3f(-offset/2.0, -scale+offset, scale);
//	glVertex3f(offset/2.0, -scale+offset, scale);
//	glVertex3f(offset/2.0, -scale, scale);
//	glVertex3f(-offset/2.0, -scale, scale);
//	
//	SetCubeColor(12, false, s);
//	// 12 - side
//	glVertex3f(-offset/2.0, scale-offset, scale);
//	glVertex3f(offset/2.0, scale-offset, scale);
//	glVertex3f(offset/2.0, scale, scale);
//	glVertex3f(-offset/2.0, scale, scale);
//	
//	SetCubeColor(8, true, s);
//	// 8 - side
//	glVertex3f(-scale+offset, -offset/2.0, scale);
//	glVertex3f(-scale+offset, offset/2.0, scale);
//	glVertex3f(-scale, offset/2.0, scale);
//	glVertex3f(-scale, -offset/2.0, scale);
//	
//	SetCubeColor(6, true, s);
//	// 6 - side
//	glVertex3f(scale-offset, -offset/2.0, scale);
//	glVertex3f(scale-offset, offset/2.0, scale);
//	glVertex3f(scale, offset/2.0, scale);
//	glVertex3f(scale, -offset/2.0, scale);
	
	glEnd();
	
}
/** Draw the transition at some percentage 0...1 between two states */

void RubiksCorner::OpenGLDraw(const RubiksCornerState&, const RubiksCornerState&, float) const
{
	
}

void RubiksCorner::OpenGLDraw(const RubiksCornerState&, const RubiksCornersAction&) const
{
	
}

void RubiksCorner::SetFaceColor(int face, const RubiksCornerState &s) const
{
	int theColor = -1;
	int cube = s.GetFaceInLoc(face);
	switch (cube)
	{
		case 0: theColor = 0; break;
		case 1: theColor = 1; break;
		case 2: theColor = 2; break;
		case 3: theColor = 0; break;
		case 4: theColor = 2; break;
		case 5: theColor = 3; break;
		case 6: theColor = 0; break;
		case 7: theColor = 3; break;
		case 8: theColor = 4; break;
		case 9: theColor = 0; break;
		case 10: theColor = 4; break;
		case 11: theColor = 1; break;
		case 12: theColor = 5; break;
		case 13: theColor = 2; break;
		case 14: theColor = 1; break;
		case 15: theColor = 5; break;
		case 16: theColor = 3; break;
		case 17: theColor = 2; break;
		case 18: theColor = 5; break;
		case 19: theColor = 4; break;
		case 20: theColor = 3; break;
		case 21: theColor = 5; break;
		case 22: theColor = 1; break;
		case 23: theColor = 4; break;
	}
	
	switch (theColor)
	{
		case 0: glColor3f(1.0, 0.0, 0.0); break;
		case 1: glColor3f(0.0, 1.0, 0.0); break;
		case 2: glColor3f(0.0, 0.0, 1.0); break;
		case 3: glColor3f(1.0, 1.0, 0.0); break;
		case 4: glColor3f(1.0, 0.5, 0.0); break;
		case 5: glColor3f(1.0, 1.0, 1.0); break;
		default: assert(false);
	}
}
