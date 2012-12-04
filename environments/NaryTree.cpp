/*
 *  NaryTree.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/20/10.
 *  Copyright 2010 University of Denver. All rights reserved.
 *
 */

#include "NaryTree.h"
#include <iostream>

void NaryTree::GetSuccessors(const NaryState &nodeID, std::vector<NaryState> &neighbors) const
{
//	std::cout << nodeID << " has depth " << GetDepth(nodeID) << std::endl;
	neighbors.resize(0);
	if (GetDepth(nodeID) >= d)
		return;
	for (int x = 0; x < b; x++)
		neighbors.push_back(nodeID*b+x+1);
}

void NaryTree::GetActions(const NaryState &nodeID, std::vector<NaryAction> &actions) const
{
	actions.resize(0);
	if (GetDepth(nodeID) >= d)
		return;
	actions.resize(b);
	for (unsigned x = 0; x < actions.size(); x++)
		actions[x] = x+1;
}

NaryAction NaryTree::GetAction(const NaryState &, const NaryState &) const
{
	assert(false);
	return 0;
}

void NaryTree::ApplyAction(NaryState &s, NaryAction a) const
{
	if (a > 0)
		s = s*b+a+1;
	else {
		s = (s-1)/b;
	}
}

void NaryTree::GetNextState(const NaryState &s1, NaryAction a, NaryState &s2) const
{
	if (a > 0)
		s2 = s1*b+a;
	else {
		s2 = (s1-1)/b;
	}

}

bool NaryTree::InvertAction(NaryAction &a) const
{ a = -a; return true; }

/** Heuristic value between two arbitrary nodes. **/
double NaryTree::HCost(const NaryState &node1, const NaryState &node2)
{ if (node1 == node2) return 0; return 1; }

double NaryTree::GCost(const NaryState &, const NaryState &)
{ return 1; }
double NaryTree::GCost(const NaryState &, const NaryAction &)
{ return 1; }
bool NaryTree::GoalTest(const NaryState &node, const NaryState &goal)
{ return node == goal; }


uint64_t NaryTree::GetStateHash(const NaryState &node) const
{ return node; }

uint64_t NaryTree::GetActionHash(NaryAction act) const
{ return act+b; }


void NaryTree::OpenGLDraw() const { }
void NaryTree::OpenGLDraw(const NaryState&) const { }
void NaryTree::OpenGLDraw(const NaryState&, const NaryState&, float) const { }
void NaryTree::OpenGLDraw(const NaryState&, const NaryAction&) const { }
