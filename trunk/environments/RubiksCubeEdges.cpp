//
//  RubikEdge.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/4/13.
//  Copyright (c) 2013 University of Denver. All rights reserved.
//

#include "RubiksCubeEdges.h"
#include "GLUtil.h"
#include <cassert>

void RubikEdgeState::GetDual(RubikEdgeState &s) const
{
	for (int x = 0; x < 12; x++)
	{
		s.SetCubeInLoc(GetCubeInLoc(x), x);
		s.SetCubeOrientation(x, GetCubeOrientation(GetCubeInLoc(x)));
	}
}

void RubikEdge::GetSuccessors(const RubikEdgeState &nodeID, std::vector<RubikEdgeState> &neighbors) const
{
	RubikEdgeState s;
	for (int x = 0; x < 18; x++)
	{
		GetNextState(nodeID, x, s);
		neighbors.push_back(s);
	}
}

void RubikEdge::GetActions(const RubikEdgeState &nodeID, std::vector<RubikEdgeAction> &actions) const
{
	actions.resize(0);
	for (int x = 0; x < 18; x++)
	{
		actions.push_back(x);
	}
}

RubikEdgeAction RubikEdge::GetAction(const RubikEdgeState &s1, const RubikEdgeState &s2) const
{
	assert(false);
	RubikEdgeAction a;
	return a;
}

void RubikEdge::ApplyMove(RubikEdgeState &s, RubikEdgeMove *a)
{
	ApplyAction(s, a->act);
}

void RubikEdge::UndoMove(RubikEdgeState &s, RubikEdgeMove *a)
{
	RubikEdgeAction todo = a->act;
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

void RubikEdge::UndoAction(RubikEdgeState &s, RubikEdgeAction a) const
{
	RubikEdgeAction todo = a;
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

void RubikEdge::ApplyAction(RubikEdgeState &s, RubikEdgeAction a) const
{
	switch (a)
	{
		case 0: // face 0
		{
			int a = s.GetCubeInLoc(1-1);
			int b = s.GetCubeInLoc(3-1);
			int c = s.GetCubeInLoc(5-1);
			int d = s.GetCubeInLoc(7-1);
			s.SetCubeInLoc(1-1, d);
			s.SetCubeInLoc(3-1, a);
			s.SetCubeInLoc(5-1, b);
			s.SetCubeInLoc(7-1, c);
		}
			break;
		case 1:
		{
			int a = s.GetCubeInLoc(1-1);
			int b = s.GetCubeInLoc(3-1);
			int c = s.GetCubeInLoc(5-1);
			int d = s.GetCubeInLoc(7-1);
			s.SetCubeInLoc(1-1, b);
			s.SetCubeInLoc(3-1, c);
			s.SetCubeInLoc(5-1, d);
			s.SetCubeInLoc(7-1, a);
		}
			break;
		case 2:
		{
			int a = s.GetCubeInLoc(1-1);
			int b = s.GetCubeInLoc(3-1);
			int c = s.GetCubeInLoc(5-1);
			int d = s.GetCubeInLoc(7-1);
			s.SetCubeInLoc(1-1, c);
			s.SetCubeInLoc(3-1, d);
			s.SetCubeInLoc(5-1, a);
			s.SetCubeInLoc(7-1, b);
		}
			break;
		case 3: // face 5
		{
			int a = s.GetCubeInLoc(9-1);
			int b = s.GetCubeInLoc(10-1);
			int c = s.GetCubeInLoc(11-1);
			int d = s.GetCubeInLoc(12-1);
			s.SetCubeInLoc(9-1, d);
			s.SetCubeInLoc(10-1, a);
			s.SetCubeInLoc(11-1, b);
			s.SetCubeInLoc(12-1, c);
		}
			break;
		case 4:
		{
			int a = s.GetCubeInLoc(9-1);
			int b = s.GetCubeInLoc(10-1);
			int c = s.GetCubeInLoc(11-1);
			int d = s.GetCubeInLoc(12-1);
			s.SetCubeInLoc(9-1, b);
			s.SetCubeInLoc(10-1, c);
			s.SetCubeInLoc(11-1, d);
			s.SetCubeInLoc(12-1, a);
		}
			break;
		case 5:
		{
			int a = s.GetCubeInLoc(9-1);
			int b = s.GetCubeInLoc(10-1);
			int c = s.GetCubeInLoc(11-1);
			int d = s.GetCubeInLoc(12-1);
			s.SetCubeInLoc(9-1, c);
			s.SetCubeInLoc(10-1, d);
			s.SetCubeInLoc(11-1, a);
			s.SetCubeInLoc(12-1, b);
		}
			break;
			
		case 6: // face 2
		{
			int a = s.GetCubeInLoc(2-1);
			int b = s.GetCubeInLoc(3-1);
			int c = s.GetCubeInLoc(4-1);
			int d = s.GetCubeInLoc(10-1);
			s.SetCubeInLoc(2-1, d);
			s.SetCubeInLoc(3-1, a);
			s.SetCubeInLoc(4-1, b);
			s.SetCubeInLoc(10-1, c);
			s.FlipCubeOrientation(b);
			s.FlipCubeOrientation(d);
		}
			break;
		case 7:
		{
			int a = s.GetCubeInLoc(2-1);
			int b = s.GetCubeInLoc(3-1);
			int c = s.GetCubeInLoc(4-1);
			int d = s.GetCubeInLoc(10-1);
			s.SetCubeInLoc(2-1, b);
			s.SetCubeInLoc(3-1, c);
			s.SetCubeInLoc(4-1, d);
			s.SetCubeInLoc(10-1, a);
			s.FlipCubeOrientation(a);
			s.FlipCubeOrientation(c);
		}
			break;
		case 8:
		{
			int a = s.GetCubeInLoc(2-1);
			int b = s.GetCubeInLoc(3-1);
			int c = s.GetCubeInLoc(4-1);
			int d = s.GetCubeInLoc(10-1);
			s.SetCubeInLoc(2-1, c);
			s.SetCubeInLoc(3-1, d);
			s.SetCubeInLoc(4-1, a);
			s.SetCubeInLoc(10-1, b);
			s.FlipCubeOrientation(a);
			s.FlipCubeOrientation(b);
			s.FlipCubeOrientation(c);
			s.FlipCubeOrientation(d);
		}
			break;
			
		case 9: // face 4
		{
			int a = s.GetCubeInLoc(6-1);
			int b = s.GetCubeInLoc(7-1);
			int c = s.GetCubeInLoc(8-1);
			int d = s.GetCubeInLoc(12-1);
			s.SetCubeInLoc(6-1, d);
			s.SetCubeInLoc(7-1, a);
			s.SetCubeInLoc(8-1, b);
			s.SetCubeInLoc(12-1, c);
			s.FlipCubeOrientation(c);
			s.FlipCubeOrientation(d);
		}
			break;
		case 10:
		{
			int a = s.GetCubeInLoc(6-1);
			int b = s.GetCubeInLoc(7-1);
			int c = s.GetCubeInLoc(8-1);
			int d = s.GetCubeInLoc(12-1);
			s.SetCubeInLoc(6-1, b);
			s.SetCubeInLoc(7-1, c);
			s.SetCubeInLoc(8-1, d);
			s.SetCubeInLoc(12-1, a);
			s.FlipCubeOrientation(a);
			s.FlipCubeOrientation(d);
		}
			break;
		case 11:
		{
			int a = s.GetCubeInLoc(6-1);
			int b = s.GetCubeInLoc(7-1);
			int c = s.GetCubeInLoc(8-1);
			int d = s.GetCubeInLoc(12-1);
			s.SetCubeInLoc(6-1, c);
			s.SetCubeInLoc(7-1, d);
			s.SetCubeInLoc(8-1, a);
			s.SetCubeInLoc(12-1, b);
			s.FlipCubeOrientation(b);
			s.FlipCubeOrientation(d);
		}
			break;
			
		case 12: // face 1
		{
			int a = s.GetCubeInLoc(1-1);
			int b = s.GetCubeInLoc(2-1);
			int c = s.GetCubeInLoc(9-1);
			int d = s.GetCubeInLoc(8-1);
			s.SetCubeInLoc(1-1, d);
			s.SetCubeInLoc(2-1, a);
			s.SetCubeInLoc(9-1, b);
			s.SetCubeInLoc(8-1, c);
			s.FlipCubeOrientation(a);
			s.FlipCubeOrientation(d);
		}
			break;
		case 13:
		{
			int a = s.GetCubeInLoc(1-1);
			int b = s.GetCubeInLoc(2-1);
			int c = s.GetCubeInLoc(9-1);
			int d = s.GetCubeInLoc(8-1);
			s.SetCubeInLoc(1-1, b);
			s.SetCubeInLoc(2-1, c);
			s.SetCubeInLoc(9-1, d);
			s.SetCubeInLoc(8-1, a);
			s.FlipCubeOrientation(a);
			s.FlipCubeOrientation(b);
		}
			break;
		case 14:
		{
			int a = s.GetCubeInLoc(1-1);
			int b = s.GetCubeInLoc(2-1);
			int c = s.GetCubeInLoc(9-1);
			int d = s.GetCubeInLoc(8-1);
			s.SetCubeInLoc(1-1, c);
			s.SetCubeInLoc(2-1, d);
			s.SetCubeInLoc(9-1, a);
			s.SetCubeInLoc(8-1, b);
			s.FlipCubeOrientation(a);
			s.FlipCubeOrientation(c);
		}
			break;
		case 15: // face 3
		{
			int a = s.GetCubeInLoc(4-1);
			int b = s.GetCubeInLoc(5-1);
			int c = s.GetCubeInLoc(6-1);
			int d = s.GetCubeInLoc(11-1);
			s.SetCubeInLoc(4-1, d);
			s.SetCubeInLoc(5-1, a);
			s.SetCubeInLoc(6-1, b);
			s.SetCubeInLoc(11-1, c);
			s.FlipCubeOrientation(b);
			s.FlipCubeOrientation(d);
		}
			break;
		case 16:
		{
			int a = s.GetCubeInLoc(4-1);
			int b = s.GetCubeInLoc(5-1);
			int c = s.GetCubeInLoc(6-1);
			int d = s.GetCubeInLoc(11-1);
			s.SetCubeInLoc(4-1, b);
			s.SetCubeInLoc(5-1, c);
			s.SetCubeInLoc(6-1, d);
			s.SetCubeInLoc(11-1, a);
			s.FlipCubeOrientation(a);
			s.FlipCubeOrientation(c);
		}
			break;
		case 17:
		{
			int a = s.GetCubeInLoc(4-1);
			int b = s.GetCubeInLoc(5-1);
			int c = s.GetCubeInLoc(6-1);
			int d = s.GetCubeInLoc(11-1);
			s.SetCubeInLoc(4-1, c);
			s.SetCubeInLoc(5-1, d);
			s.SetCubeInLoc(6-1, a);
			s.SetCubeInLoc(11-1, b);
			s.FlipCubeOrientation(a);
			s.FlipCubeOrientation(b);
			s.FlipCubeOrientation(c);
			s.FlipCubeOrientation(d);
		}
			break;
		default:
			break;
	}
	
}

void RubikEdge::GetNextState(const RubikEdgeState &s0, RubikEdgeAction a, RubikEdgeState &s1) const
{
	s1 = s0;
	ApplyAction(s1, a);
}

inline uint64_t Factorial(int n)
{
	static uint64_t Factorial[21] =
	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
	
	return Factorial[n];
}

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

int64_t RubikEdge::getMaxSinglePlayerRank() const
{
	static uint64_t Factorial[21] =
	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
	
	return Factorial[12]*(0x1<<11);
	//	return 980995276800ll;
}

int64_t RubikEdge::getMaxSinglePlayerRank2()
{
	//	return 12;
	//	return 16;
	//	return 64;
	return 128;
	//return 4;
}

int64_t RubikEdge::getMaxSinglePlayerRank2(int64_t firstIndex)
{
	return (Factorial(12)*(0x1<<4));
	//	return (Factorial(12)*(0x1<<7));
	//	return (Factorial(12)*(0x1<<9));
	//	return 81749606400ll;
}

void RubikEdge::rankPlayerFirstTwo(const RubikEdgeState &s, int, int64_t &rank)
{
	//	uint64_t hashVal = 0;
	//	for (int x = 0; x < 2; x++)
	//	{
	//		hashVal = (hashVal<<1)+s.GetCubeOrientation(x);
	//	}
	//	rank = (s.GetCubeOrientation(11)<<5)|(s.GetCubeOrientation(10)<<4)|
	//	(s.GetCubeOrientation(9)<<3)|(s.GetCubeOrientation(8)<<2)|
	//	(s.GetCubeOrientation(7)<<1)|s.GetCubeOrientation(6);
	rank = (s.GetCubeOrientation(11)<<6)|(s.GetCubeOrientation(10)<<5)|
	(s.GetCubeOrientation(9)<<4)|(s.GetCubeOrientation(8)<<3)|
	(s.GetCubeOrientation(7)<<2)|(s.GetCubeOrientation(6)<<1)|
	s.GetCubeOrientation(5);
	//	rank = s.GetCubeInLoc(0);
}

void RubikEdge::rankPlayerRemaining(const RubikEdgeState &node, int, int64_t &rank)
{
	uint64_t perm = 0, dual = 0;
	for (int x = 0; x < 12; x++)
	{
		set(perm, x, node.GetCubeInLoc(x));
		set(dual, node.GetCubeInLoc(x), x);
	}
	
	uint64_t hashVal = 0;
	for (int x = 7; x < 11; x++)
	{
		hashVal = (hashVal<<1)+node.GetCubeOrientation(11-x);
	}
	hashVal = hashVal*Factorial(12)+MRRank(12, perm, dual);
	rank = hashVal;
	
	//	static uint64_t Factorial[21] =
	//	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
	//		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
	//		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
	//
	//	int puzzle[11];
	//	for (int x = 1; x < 12; x++)
	//	{
	//		puzzle[x-1] = s.GetCubeInLoc(x);
	//		if (puzzle[x-1] > s.GetCubeInLoc(0))
	//			puzzle[x-1]--;
	//	}
	//
	//	rank = 0;
	//	int numEntriesLeft = 11;
	//	for (unsigned int x = 0; x < 11; x++)
	//	{
	//		rank += puzzle[x]*Factorial[numEntriesLeft-1];
	//		numEntriesLeft--;
	//		for (unsigned y = x; y < 11; y++)
	//		{
	//			if (puzzle[y] > puzzle[x])
	//				puzzle[y]--;
	//		}
	//	}
	//
	//	// orientations
	//	for (int x = 0; x < 11; x++)
	//	{
	//		rank = (rank<<1)+s.GetCubeOrientation(x);
	//	}
}


uint64_t RubikEdge::GetStateHash(const RubikEdgeState &node) const
{
	uint64_t perm = 0, dual = 0;
	for (int x = 0; x < 12; x++)
	{
		int val = node.GetCubeInLoc(x);
		set(perm, x, val);
		set(dual, val, x);
	}
	
	uint64_t hashVal = 0;
	for (int x = 0; x < 11; x++)
	{
		hashVal = (hashVal<<1)+node.GetCubeOrientation(11-x);
	}
	//	return (MRRank(12, perm, dual)<<11)|hashVal;
	hashVal = hashVal*Factorial(12)+MRRank(12, perm, dual);
	return hashVal;
}

// 38.522
uint64_t RubikEdge::MRRank(int n, uint64_t perm, uint64_t dual) const
{
	int ss[12];
	int ssLoc = 0;
	//	static std::vector<int> ss;
	//	ss.resize(0);
	int s;
	for (int i = n; i > 1; i--)
	{
		s = get(perm, i-1);
		ss[ssLoc] = s;
		ssLoc++;
		//		ss.push_back(s);
		swap(perm, i-1, get(dual, i-1));
		swap(dual, s, i-1);
	}
	uint64_t result = 0;
	int cnt = 2;
	//	printf("0");
	for (int i = (int)ssLoc-1; i >= 0; i--)
	{
		//		printf(")*%d + %d ", cnt, ss[i]);
		result *= cnt;
		result += ss[i];
		cnt++;
	}
	//	printf("\n");
	return result;
	//	if (n == 1)
	//		return 0;
	//	int s = get(perm, n-1);
	//	swap(perm, n-1, get(dual, n-1));
	//	swap(dual, s, n-1);
	//	return s+n*MRRank(n-1, perm, dual);
}

// 53.5% time
uint64_t RubikEdge::MRRank2(int n, uint64_t perm, uint64_t dual) const
{
	if (n == 1)
		return 0;
	int s = get(perm, n-1);
	swap(perm, n-1, get(dual, n-1));
	swap(dual, s, n-1);
	return s+n*MRRank2(n-1, perm, dual);
}

void RubikEdge::GetStateFromHash(uint64_t hash, RubikEdgeState &node) const
{
	int cnt = 0;
	uint64_t bits = hash/Factorial(12);
	uint64_t hVal = hash%Factorial(12);
	for (int x = 10; x >= 0; x--)
	{
		node.SetCubeOrientation(11-x, bits&0x1);
		cnt += bits & 0x1;
		bits >>= 1;
	}
	if (1 == cnt%2)
	{
		node.SetCubeOrientation(11-11, true);
	}
	
	uint64_t val = 0;
	for (int x = 0; x < 12; x++)
		set(val, x, x);
	MRUnrank2(12, hVal, val);
	for (int x = 0; x < 12; x++)
	{
		node.SetCubeInLoc(x, get(val, x));
	}
}

// 38.982 sec elapsed
// 3.5 % of time in function
// Overall locality: 936842 / 1800000 = 0.520468
void RubikEdge::MRUnrank(int n, uint64_t r, uint64_t &perm) const
{
	if (n > 0)
	{
		//		printf("Swap %d %llu\n", n-1, r%n);
		swap(perm, n-1, r%n);
		MRUnrank(n-1, r/n, perm);
	}
}

//38.152 sec elapsed
void RubikEdge::MRUnrank2(int n, uint64_t r, uint64_t &perm) const
{
	for (int i = n; i > 0; i--)
	{
		//		printf("Swap %d %llu\n", i-1, r%i);
		swap(perm, i-1, r%i);
		r = r/i;
	}
}


// 50.970 sec elapsed
// 73.9% of time inside this function
// Overall locality: 1044342 / 1800000 = 0.580190
//uint64_t RubikEdge::GetStateHash(const RubikEdgeState &node) const
//{
//	static uint64_t Factorial[21] =
//	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
//		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
//		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
//
//	int puzzle[12];
//	for (int x = 0; x < 12; x++)
//		puzzle[x] = node.GetCubeInLoc(x);
//
//	uint64_t hashVal = 0;
//	int numEntriesLeft = 12;
//	for (unsigned int x = 0; x < 12; x++)
//	{
//		hashVal += puzzle[x]*Factorial[numEntriesLeft-1];
//		numEntriesLeft--;
//		for (unsigned y = x; y < 12; y++)
//		{
//			if (puzzle[y] > puzzle[x])
//				puzzle[y]--;
//		}
//	}
//	for (int x = 0; x < 11; x++)
//	{
//		hashVal = (hashVal<<1)+node.GetCubeOrientation(x);
//	}
//	return hashVal;
//}
//
//void RubikEdge::GetStateFromHash(uint64_t hash, RubikEdgeState &node) const
//{
//	int puzzle[12];
//	uint64_t hashVal = hash;
//
//	int cnt = 0;
//	for (int x = 10; x >= 0; x--)
//	{
//		node.SetCubeOrientation(x, hashVal&0x1);
//		cnt += hashVal & 0x1;
//		hashVal >>= 1;
//	}
//	if (1 == cnt%2)
//	{
//		node.SetCubeOrientation(11, true);
//	}
//	int numEntriesLeft = 1;
//	for (int x = 12-1; x >= 0; x--)
//	{
//		puzzle[x] = hashVal%numEntriesLeft;
//		hashVal /= numEntriesLeft;
//		numEntriesLeft++;
//		for (int y = x+1; y < 12; y++)
//		{
//			if (puzzle[y] >= puzzle[x])
//				puzzle[y]++;
//		}
//	}
//	for (int x = 0; x < 12; x++)
//		node.SetCubeInLoc(x, puzzle[x]);
//}

void RubikEdge::OpenGLDraw() const
{
	
}

void RubikEdge::OpenGLDraw(const RubikEdgeState&s) const
{
	float scale = 0.3;
	float offset = 0.95*2.0*scale/3.0;
	glBegin(GL_QUADS);
	// top
	//// Face 0
	//SetCubeColor(0);
	// 1 - top
	SetCubeColor(1, false, s);
	glVertex3f(-scale, -scale, offset/2.0);
	glVertex3f(-scale, -scale, -offset/2.0);
	glVertex3f(-scale+offset, -scale, -offset/2.0);
	glVertex3f(-scale+offset, -scale, offset/2.0);
	
	SetCubeColor(5, false, s);
	// 5 - top
	glVertex3f(scale, -scale, offset/2.0);
	glVertex3f(scale, -scale, -offset/2.0);
	glVertex3f(scale-offset, -scale, -offset/2.0);
	glVertex3f(scale-offset, -scale, offset/2.0);
	
	SetCubeColor(3, false, s);
	// 3 - top
	glVertex3f(-offset/2.0, -scale, -scale);
	glVertex3f(-offset/2.0, -scale, -scale+offset);
	glVertex3f(offset/2.0, -scale, -scale+offset);
	glVertex3f(offset/2.0, -scale, -scale);
	
	SetCubeColor(7, false, s);
	// 7 - top
	glVertex3f(-offset/2.0, -scale, scale);
	glVertex3f(-offset/2.0, -scale, scale-offset);
	glVertex3f(offset/2.0, -scale, scale-offset);
	glVertex3f(offset/2.0, -scale, scale);
	
	//// Face 5
	SetCubeColor(9, true, s);
	// 9 - bottom
	glVertex3f(-scale, scale, offset/2.0);
	glVertex3f(-scale, scale, -offset/2.0);
	glVertex3f(-scale+offset, scale, -offset/2.0);
	glVertex3f(-scale+offset, scale, offset/2.0);
	
	SetCubeColor(11, true, s);
	// 11 - bottom
	glVertex3f(scale, scale, offset/2.0);
	glVertex3f(scale, scale, -offset/2.0);
	glVertex3f(scale-offset, scale, -offset/2.0);
	glVertex3f(scale-offset, scale, offset/2.0);
	
	SetCubeColor(10, true, s);
	// 10 - bottom
	glVertex3f(-offset/2.0, scale, -scale);
	glVertex3f(-offset/2.0, scale, -scale+offset);
	glVertex3f(offset/2.0, scale, -scale+offset);
	glVertex3f(offset/2.0, scale, -scale);
	
	SetCubeColor(12, true, s);
	// 12 - bottom
	glVertex3f(-offset/2.0, scale, scale);
	glVertex3f(-offset/2.0, scale, scale-offset);
	glVertex3f(offset/2.0, scale, scale-offset);
	glVertex3f(offset/2.0, scale, scale);
	
	//// Face 1
	SetCubeColor(1, true, s);
	// 1 - side
	glVertex3f(-scale, -scale+offset, -offset/2.0);
	glVertex3f(-scale, -scale+offset, offset/2.0);
	glVertex3f(-scale, -scale, offset/2.0);
	glVertex3f(-scale, -scale, -offset/2.0);
	
	SetCubeColor(9, false, s);
	// 9 - side
	glVertex3f(-scale, scale-offset, -offset/2.0);
	glVertex3f(-scale, scale-offset, offset/2.0);
	glVertex3f(-scale, scale, offset/2.0);
	glVertex3f(-scale, scale, -offset/2.0);
	
	SetCubeColor(2, false, s);
	// 2 - side(1)
	glVertex3f(-scale, -offset/2.0, -scale+offset);
	glVertex3f(-scale, offset/2.0, -scale+offset);
	glVertex3f(-scale, offset/2.0, -scale);
	glVertex3f(-scale, -offset/2.0, -scale);
	
	SetCubeColor(8, false, s);
	// 8 - side(1)
	glVertex3f(-scale, -offset/2.0, scale-offset);
	glVertex3f(-scale, offset/2.0, scale-offset);
	glVertex3f(-scale, offset/2.0, scale);
	glVertex3f(-scale, -offset/2.0, scale);
	
	//// Face 3
	SetCubeColor(5, true, s);
	// 5 - side
	glVertex3f(scale, -scale+offset, -offset/2.0);
	glVertex3f(scale, -scale+offset, offset/2.0);
	glVertex3f(scale, -scale, offset/2.0);
	glVertex3f(scale, -scale, -offset/2.0);
	
	SetCubeColor(11, false, s);
	// 11 - side
	glVertex3f(scale, scale-offset, -offset/2.0);
	glVertex3f(scale, scale-offset, offset/2.0);
	glVertex3f(scale, scale, offset/2.0);
	glVertex3f(scale, scale, -offset/2.0);
	
	SetCubeColor(4, true, s);
	// 4 - side(1)
	glVertex3f(scale, -offset/2.0, -scale+offset);
	glVertex3f(scale, offset/2.0, -scale+offset);
	glVertex3f(scale, offset/2.0, -scale);
	glVertex3f(scale, -offset/2.0, -scale);
	
	SetCubeColor(6, false, s);
	// 6 - side(1)
	glVertex3f(scale, -offset/2.0, scale-offset);
	glVertex3f(scale, offset/2.0, scale-offset);
	glVertex3f(scale, offset/2.0, scale);
	glVertex3f(scale, -offset/2.0, scale);
	
	//// Face 2
	SetCubeColor(3, true, s);
	// 3 - side
	glVertex3f(-offset/2.0, -scale+offset, -scale);
	glVertex3f(offset/2.0, -scale+offset, -scale);
	glVertex3f(offset/2.0, -scale, -scale);
	glVertex3f(-offset/2.0, -scale, -scale);
	
	SetCubeColor(10, false, s);
	// 10 - side
	glVertex3f(-offset/2.0, scale-offset, -scale);
	glVertex3f(offset/2.0, scale-offset, -scale);
	glVertex3f(offset/2.0, scale, -scale);
	glVertex3f(-offset/2.0, scale, -scale);
	
	SetCubeColor(2, true, s);
	// 2 - side
	glVertex3f(-scale+offset, -offset/2.0, -scale);
	glVertex3f(-scale+offset, offset/2.0, -scale);
	glVertex3f(-scale, offset/2.0, -scale);
	glVertex3f(-scale, -offset/2.0, -scale);
	
	SetCubeColor(4, false, s);
	// 4 - side
	glVertex3f(scale-offset, -offset/2.0, -scale);
	glVertex3f(scale-offset, offset/2.0, -scale);
	glVertex3f(scale, offset/2.0, -scale);
	glVertex3f(scale, -offset/2.0, -scale);
	
	
	//// Face 4
	SetCubeColor(7, true, s);
	// 7 - side
	glVertex3f(-offset/2.0, -scale+offset, scale);
	glVertex3f(offset/2.0, -scale+offset, scale);
	glVertex3f(offset/2.0, -scale, scale);
	glVertex3f(-offset/2.0, -scale, scale);
	
	SetCubeColor(12, false, s);
	// 12 - side
	glVertex3f(-offset/2.0, scale-offset, scale);
	glVertex3f(offset/2.0, scale-offset, scale);
	glVertex3f(offset/2.0, scale, scale);
	glVertex3f(-offset/2.0, scale, scale);
	
	SetCubeColor(8, true, s);
	// 8 - side
	glVertex3f(-scale+offset, -offset/2.0, scale);
	glVertex3f(-scale+offset, offset/2.0, scale);
	glVertex3f(-scale, offset/2.0, scale);
	glVertex3f(-scale, -offset/2.0, scale);
	
	SetCubeColor(6, true, s);
	// 6 - side
	glVertex3f(scale-offset, -offset/2.0, scale);
	glVertex3f(scale-offset, offset/2.0, scale);
	glVertex3f(scale, offset/2.0, scale);
	glVertex3f(scale, -offset/2.0, scale);
	
	glEnd();
	
}
/** Draw the transition at some percentage 0...1 between two states */

void RubikEdge::OpenGLDraw(const RubikEdgeState&, const RubikEdgeState&, float) const
{
	
}

void RubikEdge::OpenGLDraw(const RubikEdgeState&, const RubikEdgeAction&) const
{
	
}

void RubikEdge::SetCubeColor(int which, bool face, const RubikEdgeState &s) const
{
	int cubes_first[12] = { 0, 1, 0, 2, 0, 3, 0, 1, 1, 2, 3, 4};
	int cubes_second[12] = { 1, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 5};

	int theColor = -1;
	int cube = s.GetCubeInLoc(which-1);
	bool flipped = s.GetCubeOrientation(cube);
	if (flipped == face)
		theColor = cubes_first[cube];
	else
		theColor = cubes_second[cube];

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
