//
//  TOH.hpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/20/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#ifndef TOH_hpp
#define TOH_hpp

#include <stdio.h>
#include <cstdint>
#include "SearchEnvironment.h"

struct TOHMove {
	TOHMove(uint8_t s, uint8_t d) :source(s), dest(d) {}
	TOHMove() {}
	uint8_t source;
	uint8_t dest;
};

template <int numDisks>
struct TOHState {
	TOHState()
	{
		for (int x = 0; x < 4; x++)
		{
			counts[x] = 0;
		}
		for (int x = 0; x < numDisks; x++)
		{
			disks[0][x] = numDisks-x;
		}
		counts[0] = numDisks;
	}
	int GetDiskCountOnPeg(int whichPeg) const
	{
		return counts[whichPeg];
	}
	
	int GetDiskOnPeg(int whichPeg, int whichDisk) const
	{
		return disks[whichPeg][whichDisk];
	}

	// if this is slow, we can add a "big" disk to every peg to
	// avoid the "if" statement.
	int GetSmallestDiskOnPeg(int whichPeg) const
	{
		int count = GetDiskCountOnPeg(whichPeg);
		if (count == 0)
			return numDisks+1;
		return GetDiskOnPeg(whichPeg, count-1);
	}

	uint8_t disks[4][numDisks];
	uint8_t counts[4];
};

template <int disks>
class TOH : SearchEnvironment<TOHMove, TOHState<disks>> {
public:
	virtual void GetSuccessors(const TOHState<disks> &nodeID, std::vector<TOHState<disks>> &neighbors) const;
	virtual void GetActions(const TOHState<disks> &nodeID, std::vector<TOHMove> &actions) const;
	virtual void ApplyAction(TOHState<disks> &s, TOHMove a) const;
	virtual bool InvertAction(TOHMove &a) const;

	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const TOHState<disks> &node1, const TOHState<disks> &node2) const;
	virtual double GCost(const TOHState<disks> &node1, const TOHState<disks> &node2) { return 1; }
	virtual double GCost(const TOHState<disks> &node, const TOHMove &act) { return 1; }
	virtual bool GoalTest(const TOHState<disks> &node, const TOHState<disks> &goal);

	virtual uint64_t GetStateHash(const TOHState<disks> &node) const;
	virtual void GetStateFromHash(uint64_t parent, TOHState<disks> &s) const;
	virtual uint64_t GetActionHash(TOHMove act) const;


	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const TOHState<disks>&) const;
	/** Draw the transition at some percentage 0...1 between two TOHState<disks>s */
	virtual void OpenGLDraw(const TOHState<disks>&, const TOHState<disks>&, float) const;
	virtual void OpenGLDraw(const TOHState<disks>&, const TOHMove&) const;
protected:
};



template <int disks>
void TOH<disks>::GetSuccessors(const TOHState<disks> &nodeID, std::vector<TOHState<disks>> &neighbors) const
{
	neighbors.resize(0);
	std::vector<TOHMove> acts;
	TOHState<disks> tmp;
	GetActions(nodeID, acts);
	for (auto act : acts)
	{
		GetNextState(nodeID, act, tmp);
		neighbors.push_back(tmp);
	}
}

template <int disks>
void TOH<disks>::GetActions(const TOHState<disks> &s, std::vector<TOHMove> &actions) const
{
	actions.resize(0);
	if (s.GetSmallestDiskOnPeg(0) < s.GetSmallestDiskOnPeg(1))
	{
		actions.push_back(TOHMove(0, 1));
	}
	else {
		actions.push_back(TOHMove(1, 0));
	}
	if (s.GetSmallestDiskOnPeg(0) < s.GetSmallestDiskOnPeg(2))
	{
		actions.push_back(TOHMove(0, 2));
	}
	else {
		actions.push_back(TOHMove(2, 0));
	}
	if (s.GetSmallestDiskOnPeg(0) < s.GetSmallestDiskOnPeg(3))
	{
		actions.push_back(TOHMove(0, 3));
	}
	else {
		actions.push_back(TOHMove(3, 0));
	}
	if (s.GetSmallestDiskOnPeg(1) < s.GetSmallestDiskOnPeg(2))
	{
		actions.push_back(TOHMove(1, 2));
	}
	else {
		actions.push_back(TOHMove(2, 1));
	}
	if (s.GetSmallestDiskOnPeg(1) < s.GetSmallestDiskOnPeg(3))
	{
		actions.push_back(TOHMove(1, 3));
	}
	else {
		actions.push_back(TOHMove(3, 1));
	}
	if (s.GetSmallestDiskOnPeg(2) < s.GetSmallestDiskOnPeg(3))
	{
		actions.push_back(TOHMove(2, 3));
	}
	else {
		actions.push_back(TOHMove(3, 2));
	}
}


template <int disks>
void TOH<disks>::ApplyAction(TOHState<disks> &s, TOHMove m) const
{
	s.disks[m.dest][s.counts[m.dest]] = s.disks[m.source][s.counts[m.source]-1];
	s.counts[m.dest]++;
	s.counts[m.source]--;
}

template <int disks>
bool TOH<disks>::InvertAction(TOHMove &a) const
{
	uint8_t tmp = a.source;
	a.source = a.dest;
	a.dest = tmp;
	return true;
}


/** Heuristic value between two arbitrary nodes. **/
template <int disks>
double TOH<disks>::HCost(const TOHState<disks> &node1, const TOHState<disks> &node2) const
{
	// NOTE: this is using the standard goal state; arbitrary goal states
	// are more expensive to check
	return node1.GetDiskCountOnPeg(3) - disks;
}

template <int disks>
bool TOH<disks>::GoalTest(const TOHState<disks> &node, const TOHState<disks> &goal)
{
	// NOTE: this is using the standard goal state; arbitrary goal states
	// are more expensive to check
	return (node.GetDiskCountOnPeg(3)==disks);
}


template <int disks>
uint64_t TOH<disks>::GetStateHash(const TOHState<disks> &node) const
{
	uint64_t hash = 0;
	for (int x = 0; x < 4; x++)
	{
		for (int y = 0; y < node.GetDiskCountOnPeg(x); y++)
		{
			hash |= (uint64_t(x)<<(2*(node.GetDiskOnPeg(x, y)-1)));
		}
	}
	return hash;
}

template <int disks>
void TOH<disks>::GetStateFromHash(uint64_t hash, TOHState<disks> &s) const
{
	for (int x = 0; x < 4; x++)
		s.counts[x] = 0;
	for (int x = s.numDisks-1; x >= 0; x--)
	{
		int nextPeg = (hash>>(2*x))&0x3;
		s.disks[nextPeg][s.counts[nextPeg]] = x+1;
		s.counts[nextPeg]++;
	}
}

template <int disks>
uint64_t TOH<disks>::GetActionHash(TOHMove act) const
{
	return (act.source<<8)|act.dest;
}

template <int disks>
void TOH<disks>::OpenGLDraw() const
{
	
}

template <int disks>
void TOH<disks>::OpenGLDraw(const TOHState<disks>&) const
{
	
}

/** Draw the transition at some percentage 0...1 between two TOHState<disks>s */
template <int disks>
void TOH<disks>::OpenGLDraw(const TOHState<disks>&, const TOHState<disks>&, float) const
{
	
}

template <int disks>
void TOH<disks>::OpenGLDraw(const TOHState<disks>&, const TOHMove&) const
{
	
}


#endif /* TOH_hpp */
