/*
 *  Fling.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 3/5/10.
 *  Copyright 2010 NS Software. All rights reserved.
 *
 */

#include <iostream>
#include "SearchEnvironment.h"

class FlingBoard
{
public:
	FlingBoard(unsigned int len=7, unsigned int high=8) :width(len), height(high) { board.resize(len*high); }
	void Reset() { board.resize(0); board.resize(width*height); locs.resize(0); }
	void AddFling(unsigned int x, unsigned int y);
	void AddFling(unsigned int offset);
	void RemoveFling(unsigned int x, unsigned int y);
	void RemoveFling(unsigned int offset);
	bool CanMove(int which, int x, int y) const;
	void Move(int which, int x, int y);
	bool HasPiece(int x, int y) const;
	unsigned int width;
	unsigned int height;
	std::vector<bool> board;
	std::vector<int> locs;
};

class FlingMove
{
public:
	uint8_t which;
	uint8_t dir;
};

static std::ostream& operator <<(std::ostream & out, const FlingBoard &loc)
{
	for (unsigned int y= 0; y < loc.height; y++)
	{
		for (unsigned int x = 0; x < loc.width; x++)
		{
			if (loc.board[y*loc.width+x])
				out << "o";
			else {
				out << ".";
			}
		}
		out << std::endl;
	}
	return out;
}

static bool operator==(const FlingBoard &l1, const FlingBoard &l2) {
	return (l1.width == l2.width && l1.height == l2.height && l1.board == l2.board);
}

static bool operator!=(const FlingBoard &l1, const FlingBoard &l2) {
	return !(l1.width == l2.width && l1.height == l2.height && l1.board == l2.board);
}


class Fling : public SearchEnvironment<FlingBoard, FlingMove> {
public:
	Fling();
	
	virtual void GetSuccessors(const FlingBoard &nodeID, std::vector<FlingBoard> &neighbors) const;
	virtual void GetActions(const FlingBoard &nodeID, std::vector<FlingMove> &actions) const;
	
	virtual FlingMove GetAction(const FlingBoard &s1, const FlingBoard &s2) const { assert(false); }
	virtual void ApplyAction(FlingBoard &s, FlingMove a) const;
	virtual void UndoAction(FlingBoard &s, FlingMove a) const;
	
	virtual bool InvertAction(FlingMove &a) const { assert(false); }
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const FlingBoard &node1, const FlingBoard &node2) { return 0; }
	
	virtual double GCost(const FlingBoard &node1, const FlingBoard &node2) { return 1; }
	virtual double GCost(const FlingBoard &node, const FlingMove &act) { return 1; }
	virtual bool GoalTest(const FlingBoard &node, const FlingBoard &goal) { return (node.locs.size() == 1); }
	
	virtual uint64_t GetStateHash(const FlingBoard &node) const;
	virtual void GetStateFromHash(uint64_t parent, FlingBoard &s) const;
	virtual uint64_t GetActionHash(FlingMove act) const;
	
	bool GetXYFromPoint(const FlingBoard &b, point3d loc, int &x, int &y) const;
	
	int64_t getMaxSinglePlayerRank(int spots, int numPieces);
	int64_t getMaxSinglePlayerRank2(int spots, int numPieces);
	int64_t getMaxSinglePlayerRank2(int spots, int numPieces, int64_t firstIndex);
	int64_t rankPlayer(FlingBoard &s);
	void rankPlayer(FlingBoard &s, int64_t &index1, int64_t &index2);
	void rankPlayerFirstTwo(FlingBoard &s, int64_t &index1);
	void rankPlayerRemaining(FlingBoard &s, int64_t &index2);
	// returns true if it is a valid unranking given existing pieces
	bool unrankPlayer(int64_t theRank, int pieces, FlingBoard &s);

//	void initBinomialSums();
	int64_t binomialSum(unsigned int n1, unsigned int n2, unsigned int k);
	void initBinomial();
	int64_t binomial(unsigned int n, unsigned int k);
	int64_t bi(unsigned int n, unsigned int k);

	
	virtual void OpenGLDraw() const {}
	virtual void OpenGLDraw(const FlingBoard&) const;
	virtual void OpenGLDraw(const FlingBoard&, const FlingMove&) const {}

private:
	std::vector<int64_t> theSums;
	std::vector<int64_t> binomials;

};
