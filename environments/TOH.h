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
#include "PDBHeuristic.h"

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
			disks[3][x] = numDisks-x;
		}
		counts[3] = numDisks;
	}
	
	void Reset()
	{
		for (int x = 0; x < 4; x++)
		{
			counts[x] = 0;
		}
		for (int x = 0; x < numDisks; x++)
		{
			disks[3][x] = numDisks-x;
		}
		counts[3] = numDisks;
	}
	void StandardStart()
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

template <int D>
static std::ostream &operator<<(std::ostream &out, const TOHState<D> &s)
{
	for (int x = 0; x < 4; x++)
	{
		out << "(" << x << ") ";
		for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
			out << s.GetDiskOnPeg(x, y) << " ";
	}
	return out;
}

template <int D>
static bool operator==(const TOHState<D> &l1, const TOHState<D> &l2) {
	for (int x = 0; x < 4; x++)
	{
		if (l1.GetDiskCountOnPeg(x) != l2.GetDiskCountOnPeg(x))
			return false;
		for (int y = 0; y < l1.GetDiskCountOnPeg(x); y++)
		{
			if (l1.GetDiskOnPeg(x, y)!= l2.GetDiskOnPeg(x, y))
				return false;
		}
	}
	return true;
}

static std::ostream &operator<<(std::ostream &out, const TOHMove &m)
{
	out << "(" << +m.source << ", " << +m.dest << ")";
	return out;
}

static bool operator==(const TOHMove &m1, const TOHMove &m2) {
	return m1.source == m2.source && m1.dest == m2.dest;
}


template <int disks>
class TOH : public SearchEnvironment<TOHState<disks>, TOHMove> {
public:
	~TOH() {}
	void GetSuccessors(const TOHState<disks> &nodeID, std::vector<TOHState<disks>> &neighbors) const;
	void GetActions(const TOHState<disks> &nodeID, std::vector<TOHMove> &actions) const;
	void ApplyAction(TOHState<disks> &s, TOHMove a) const;
	bool InvertAction(TOHMove &a) const;

	/** Heuristic value between two arbitrary nodes. **/
	double HCost(const TOHState<disks> &node1, const TOHState<disks> &node2) const;
	double GCost(const TOHState<disks> &node1, const TOHState<disks> &node2) const { return 1; }
	double GCost(const TOHState<disks> &node, const TOHMove &act) const { return 1; }
	bool GoalTest(const TOHState<disks> &node, const TOHState<disks> &goal) const;

	uint64_t GetStateHash(const TOHState<disks> &node) const;
	void GetStateFromHash(uint64_t parent, TOHState<disks> &s) const;
	uint64_t GetNumStates(TOHState<disks> &s) const;
	uint64_t GetActionHash(TOHMove act) const;


	void OpenGLDraw() const;
	void OpenGLDraw(const TOHState<disks>&) const;
	/** Draw the transition at some percentage 0...1 between two TOHState<disks>s */
	void OpenGLDraw(const TOHState<disks>&, const TOHState<disks>&, float) const;
	void OpenGLDraw(const TOHState<disks>&, const TOHMove&) const;
protected:
private:
	// caches
	mutable std::vector<TOHMove> acts;
	mutable TOHState<disks> tmp;

};



template <int disks>
void TOH<disks>::GetSuccessors(const TOHState<disks> &nodeID, std::vector<TOHState<disks>> &neighbors) const
{
	neighbors.resize(0);
	GetActions(nodeID, acts);
	for (auto act : acts)
	{
		this->GetNextState(nodeID, act, tmp);
		neighbors.push_back(tmp);
	}
}

template <int disks>
void TOH<disks>::GetActions(const TOHState<disks> &s, std::vector<TOHMove> &actions) const
{
	actions.resize(0);
	if (s.GetSmallestDiskOnPeg(0) < s.GetSmallestDiskOnPeg(1))
	{
		if (s.GetDiskCountOnPeg(0) > 0)
			actions.push_back(TOHMove(0, 1));
	}
	else {
		if (s.GetDiskCountOnPeg(1) > 0)
			actions.push_back(TOHMove(1, 0));
	}
	if (s.GetSmallestDiskOnPeg(0) < s.GetSmallestDiskOnPeg(2))
	{
		if (s.GetDiskCountOnPeg(0) > 0)
			actions.push_back(TOHMove(0, 2));
	}
	else {
		if (s.GetDiskCountOnPeg(2) > 0)
			actions.push_back(TOHMove(2, 0));
	}
	if (s.GetSmallestDiskOnPeg(0) < s.GetSmallestDiskOnPeg(3))
	{
		if (s.GetDiskCountOnPeg(0) > 0)
			actions.push_back(TOHMove(0, 3));
	}
	else {
		if (s.GetDiskCountOnPeg(3) > 0)
			actions.push_back(TOHMove(3, 0));
	}
	if (s.GetSmallestDiskOnPeg(1) < s.GetSmallestDiskOnPeg(2))
	{
		if (s.GetDiskCountOnPeg(1) > 0)
			actions.push_back(TOHMove(1, 2));
	}
	else {
		if (s.GetDiskCountOnPeg(2) > 0)
			actions.push_back(TOHMove(2, 1));
	}
	if (s.GetSmallestDiskOnPeg(1) < s.GetSmallestDiskOnPeg(3))
	{
		if (s.GetDiskCountOnPeg(1) > 0)
			actions.push_back(TOHMove(1, 3));
	}
	else {
		if (s.GetDiskCountOnPeg(3) > 0)
			actions.push_back(TOHMove(3, 1));
	}
	if (s.GetSmallestDiskOnPeg(2) < s.GetSmallestDiskOnPeg(3))
	{
		if (s.GetDiskCountOnPeg(2) > 0)
			actions.push_back(TOHMove(2, 3));
	}
	else {
		if (s.GetDiskCountOnPeg(3) > 0)
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
	return disks - node1.GetDiskCountOnPeg(3);
}

template <int disks>
bool TOH<disks>::GoalTest(const TOHState<disks> &node, const TOHState<disks> &goal) const
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
uint64_t TOH<disks>::GetNumStates(TOHState<disks> &s) const
{
	return 1ull<<(2*disks);
}

template <int disks>
void TOH<disks>::GetStateFromHash(uint64_t hash, TOHState<disks> &s) const
{
	for (int x = 0; x < 4; x++)
		s.counts[x] = 0;
	for (int x = disks-1; x >= 0; x--)
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
	glColor3f(0.5, 0.5, 0.5);
	DrawCylinder(-0.75, 0, 0, 0, 0.01, 0.8);
	DrawCylinder(-0.25, 0, 0, 0, 0.01, 0.8);
	DrawCylinder( 0.25, 0, 0, 0, 0.01, 0.8);
	DrawCylinder( 0.75, 0, 0, 0, 0.01, 0.8);
	glColor3f(0.6, 0.4, 0.2);
	glPushMatrix();
	glScalef(1.0, 0.05, 0.25);
	DrawBox(0, 0.4/0.05+1, 0, 1.0);
	glPopMatrix();
	//	DrawBox(0, 0, 0, 0.5);
}

template <int disks>
void TOH<disks>::OpenGLDraw(const TOHState<disks>&s) const
{
	glColor3f(0.0, 0.0, 1.0);
	double offset[4] = {-0.75, -0.25, 0.25, 0.75};
	for (int x = 0; x < 4; x++)
	{
		for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
		{
			int which = s.GetDiskOnPeg(x, y);
			glColor3f(0.0, 1.0-float(which)/float(disks), 1.0);
			DrawCylinder(offset[x], 0.4-0.4/(1+float(disks))-y*0.8/(1+float(disks)), 0,
						 0.02, 0.04+0.2*which/float(disks), 0.8/(1+float(disks)));
		}
	}
}

/** Draw the transition at some percentage 0...1 between two TOHState<disks>s */
template <int disks>
void TOH<disks>::OpenGLDraw(const TOHState<disks>&s, const TOHState<disks>&s2, float interval) const
{
	TOHMove m = this->GetAction(s, s2);
	int animatingDisk = s.GetSmallestDiskOnPeg(m.source);
	int initialHeight = s.GetDiskCountOnPeg(m.source)-1;
	int finalHeight = s.GetDiskCountOnPeg(m.dest);
	
	glColor3f(0.0, 0.0, 1.0);
	double offset[4] = {-0.75, -0.25, 0.25, 0.75};
	for (int x = 0; x < 4; x++)
	{
		for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
		{
			int which = s.GetDiskOnPeg(x, y);
			if (which != animatingDisk)
			{
				glColor3f(0.0, 1.0-float(which)/float(disks), 1.0);
				DrawCylinder(offset[x], 0.4-0.4/(1+float(disks))-y*0.8/(1+float(disks)), 0,
							 0.02, 0.04+0.2*which/float(disks), 0.8/(1+float(disks)));
			}
		}
	}
	glColor3f(0.0, 1.0-float(animatingDisk)/float(disks), 1.0);
	if (interval <= 0.333)
	{
		interval *= 3;
		DrawCylinder(offset[m.source], 0.4-0.4/(1+float(disks))-initialHeight*0.8/(1+float(disks)) - (interval)*(disks+1-initialHeight)*0.8/(1+float(disks)), 0,
					 0.02, 0.04+0.2*animatingDisk/float(disks), 0.8/(1+float(disks)));
	}
	else if (interval <= 0.666)
	{
		interval *= 3;
		DrawCylinder((2-interval)*offset[m.source]+(interval-1)*offset[m.dest], 0.4-0.4/(1+float(disks))-0.8-0.2*sin((interval-1)*PI), 0,
					 0.02, 0.04+0.2*animatingDisk/float(disks), 0.8/(1+float(disks)));
	}
	else {
		DrawCylinder(offset[m.dest], 0.4-0.4/(1+float(disks))-finalHeight*0.8/(1+float(disks)) -
					 ((1.0-interval)/0.334)*(disks+1-finalHeight)*0.8/(1+float(disks)), 0,
					 0.02, 0.04+0.2*animatingDisk/float(disks), 0.8/(1+float(disks)));
	}
}

template <int disks>
void TOH<disks>::OpenGLDraw(const TOHState<disks>&, const TOHMove&) const
{
	
}

template <int patternDisks, int totalDisks>
class TOHPDB : public PDBHeuristic<TOHState<patternDisks>, TOHMove, TOH<patternDisks>, TOHState<totalDisks>> {
public:
	TOHPDB(TOH<patternDisks> *e)
	:PDBHeuristic<TOHState<patternDisks>, TOHMove, TOH<patternDisks>, TOHState<totalDisks>>(e) {}
	virtual ~TOHPDB() {}

	TOHState<totalDisks> GetStateFromAbstractState(TOHState<patternDisks> &start) const
	{
		int diff = totalDisks - patternDisks;
		
		TOHState<totalDisks> tmp;
		for (int x = 0; x < 4; x++)
		{
			tmp.counts[x] = start.counts[x];
			for (int y = 0; y < tmp.counts[x]; y++)
			{
				tmp.disks[x][y] = start.disks[x][y]+diff;
			}
		}
		return tmp;
	}
	
	virtual uint64_t GetAbstractHash(const TOHState<totalDisks> &s, int threadID = 0) const
	{
		int diff = totalDisks - patternDisks;
		uint64_t hash = 0;
		for (int x = 0; x < 4; x++)
		{
			for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
			{
				// 6 total 2 pattern
				if (s.GetDiskOnPeg(x, y)-diff > 0)
					hash |= (uint64_t(x)<<(2*(s.GetDiskOnPeg(x, y)-1-diff)));
			}
		}
		return hash;
	}

	virtual uint64_t GetPDBSize() const
	{
		return 1ull<<(2*patternDisks);
	}
	virtual uint64_t GetPDBHash(const TOHState<patternDisks> &s, int threadID = 0) const
	{
		return this->env->GetStateHash(s);
	}
	virtual void GetStateFromPDBHash(uint64_t hash, TOHState<patternDisks> &s, int threadID = 0) const
	{
		this->env->GetStateFromHash(hash, s);
	}
	
	virtual bool Load(const char *prefix) {}
	virtual void Save(const char *prefix) {}
	virtual std::string GetFileName(const char *prefix) {}
};

#endif /* TOH_hpp */
