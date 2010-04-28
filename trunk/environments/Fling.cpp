/*
 *  Fling.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 3/5/10.
 *  Copyright 2010 NS Software. All rights reserved.
 *
 */

#include "Fling.h"

void FlingBoard::AddFling(unsigned int x, unsigned int y)
{
	assert(x < width);
	assert(y < height);
	assert(board[y*width+x] == false);
	board[y*width+x] = true;
	locs.push_back(y*width+x);
}

bool FlingBoard::CanMove(int which, int x, int y) const
{
	int xx = locs[which]%width;
	int yy = locs[which]/width;
	xx+=x; yy += y;
	
	bool first = true;
	while ((xx >= 0) && (xx < width) && (yy >= 0) && (yy < height))
	{
		if (board[yy*width+xx])
			return !first;
		first = false;
		xx+=x; yy += y;
	}
	return false;
//	if (y == 0) // moving x is easy case
//	{
//		if (x == -1)
//		{
//			if ((locs[which]%width) == 0) // left edge
//				return false;
//			if (board[locs[which]+x]) // neighbor on one side
//				return false;
//			int offset = locs[which]%width;
//			for (int loc = 2; loc < offset; loc++)
//				if (board[locs[which]-offset])
//					return true;
//			return false;
//		}
//		if (x == 1)
//		{
//			if ((locs[which]%width) == width-1) // right edge
//				return false;
//			if (board[locs[which]+x]) // neighbor on one side
//				return false;
//			int offset = width-locs[which]%width;
//			for (int loc = 2; loc < offset; loc++)
//				if (board[locs[which]+offset])
//					return true;
//			return false;
//		}
//	}
//	else {
//		if (y == -1)
//		{
//			if ((locs[which]/width) == 0) // bottom edge
//				return false;
//			if (board[locs[which]-width]) // neighbor above
//				return false;
//			int offset = locs[which]/width;
//			for (int loc = 2; loc < offset; loc++)
//				if (board[locs[which]-offset*width])
//					return true;
//			return false;
//		}
//		if (y == 1)
//		{
//			if ((locs[which]/width) == height-1) // top edge
//				return false;
//			if (board[locs[which]+width]) // neighbor below
//				return false;
//			int offset = height-locs[which]/width;
//			for (int loc = 2; loc < offset; loc++)
//				if (board[locs[which]+offset*width])
//					return true;
//			return false;
//		}
//	}
//	return false;
}

void FlingBoard::Move(int which, int x, int y)
{
	int xx = locs[which]%width;
	int yy = locs[which]/width;

	int lastx = xx;
	int lasty = yy;
	xx+=x; yy += y;
	while ((xx >= 0) && (xx < width) && (yy >= 0) && (yy < height))
	{
		board[lasty*width+lastx] = board[yy*width+xx];

		lastx = xx;
		lasty = yy;
		xx+=x; yy += y;
	}
	board[lasty*width+lastx] = false;
	locs.resize(0);
	for (unsigned int t = 0; t < board.size(); t++)
		if (board[t])
			locs.push_back(t);
//	if (y == 0) // moving x is easy case
//	{
//		if (x == -1)
//		{
//			int offset = locs[which]%width;
//			for (int loc = 2; loc < offset; loc++)
//				if (board[locs[which]-offset])
//				{
//					board[locs[which]-offset] = false;
//					board[locs[which]-offset+1] = true;
//					board[locs[which]] = false;
//					locs[which] = locs[which]-offset+1;
//					return;
//				}
//		}
//		if (x == 1)
//		{
//			int offset = width-locs[which]%width;
//			for (int loc = 2; loc < offset; loc++)
//				if (board[locs[which]+offset])
//				{
//					board[locs[which]+offset] = false;
//					board[locs[which]+offset-1] = true;
//					board[locs[which]] = false;
//					locs[which] = locs[which]+offset-1;
//					return;
//				}
//		}
//	}
//	else {
//		if (y == -1)
//		{
//			int offset = locs[which]/width;
//			for (int loc = 2; loc < offset; loc++)
//				if (board[locs[which]-offset*width])
//				{
//					board[locs[which]-offset*width] = false;
//					board[locs[which]-(offset-1)*width] = true;
//					board[locs[which]] = false;
//					locs[which] = locs[which]-(offset-1)*width;
//					return;
//				}
//		}
//		if (y == 1)
//		{
//			int offset = height-locs[which]/width;
//			for (int loc = 2; loc < offset; loc++)
//				if (board[locs[which]+offset*width])
//				{
//					board[locs[which]+offset*width] = false;
//					board[locs[which]+(offset-1)*width] = true;
//					board[locs[which]] = false;
//					locs[which] = locs[which]+(offset-1)*width;
//					return;
//				}
//		}
//	}
}

void Fling::GetSuccessors(const FlingBoard &nodeID, std::vector<FlingBoard> &neighbors) const
{
	for (unsigned int x = 0; x < nodeID.locs.size(); x++)
	{
		if (nodeID.CanMove(x, 1, 0))
		{ FlingBoard b(nodeID); b.Move(x, 1, 0); neighbors.push_back(b); }
		if (nodeID.CanMove(x, -1, 0))
		{ FlingBoard b(nodeID); b.Move(x, -1, 0); neighbors.push_back(b); }
		if (nodeID.CanMove(x, 0, 1))
		{ FlingBoard b(nodeID); b.Move(x, 0, 1); neighbors.push_back(b); }
		if (nodeID.CanMove(x, 0, -1))
		{ FlingBoard b(nodeID); b.Move(x, 0, -1); neighbors.push_back(b); }
	}
}

void Fling::GetActions(const FlingBoard &nodeID, std::vector<FlingMove> &actions) const
{
	assert(false); 
}

void Fling::ApplyAction(FlingBoard &s, FlingMove a) const
{
}

void Fling::UndoAction(FlingBoard &s, FlingMove a) const
{
}

uint64_t Fling::GetStateHash(const FlingBoard &node) const
{
	return 0;
}

uint64_t Fling::GetActionHash(FlingMove act) const
{
	return 0;
}
