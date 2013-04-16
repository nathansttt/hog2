//
//  RubiksCube.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 4/6/13.
//  Copyright (c) 2013 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__RubiksCube__
#define __hog2_glut__RubiksCube__

#include <iostream>
#include <stdint.h>
#include <vector>
#include "RubiksCubeCorners.h"
#include "RubiksCubeEdges.h"
#include "SearchEnvironment.h"
#include "RubiksCube7Edges.h"
#include "FourBitArray.h"
#include "DiskBitFile.h"
#include "EnvUtil.h"

class RubiksState
{
public:
	RubiksState()
	{
		Reset();
	}
	void Reset()
	{
		corner.Reset();
		edge.Reset();
	}
	RubiksCornerState corner;
	RubikEdgeState edge;
	Rubik7EdgeState edge7;
};

static bool operator==(const RubiksState &l1, const RubiksState &l2)
{
	return l1.corner == l2.corner && l1.edge == l2.edge;
}


typedef int RubiksAction;

//class RubikCornerMove {
//public:
//	RubiksCornersAction act;
//	RubikCornerMove *next;
//	int length() { if (next == 0) return 1; return 1+next->length(); }
//};

class RubiksCube : public SearchEnvironment<RubiksState, RubiksAction>
{
public:
	RubiksCube()
	:f("/data/cc/rubik/res/RC")
	{
		pruneSuccessors = false;
		
		uint64_t maxBuckSize = GetMaxBucketSize<RubikEdge, RubikEdgeState>(true);
		InitTwoPieceData<RubikEdge, RubikEdgeState>(data, maxBuckSize);
		//InitBucketSize<RubikEdge, RubikEdgeState>(buckets, maxBuckSize);

		//		for (int x = 0; x < 18; x++)
//		{
//			moves[x].act = x;
//			if (x != 17)
//				moves[x].next = &moves[x+1];
//		} moves[17].next = 0;
	}
	~RubiksCube() {}
	void SetPruneSuccessors(bool val) { pruneSuccessors = val; history.resize(0); }
	virtual void GetSuccessors(const RubiksState &nodeID, std::vector<RubiksState> &neighbors) const;
	virtual void GetActions(const RubiksState &nodeID, std::vector<RubiksAction> &actions) const;
	virtual RubiksAction GetAction(const RubiksState &s1, const RubiksState &s2) const;
	virtual void ApplyAction(RubiksState &s, RubiksAction a) const;
	virtual void UndoAction(RubiksState &s, RubiksAction a) const;

	virtual void GetNextState(const RubiksState &, RubiksAction , RubiksState &) const;
	
	virtual bool InvertAction(RubiksAction &a) const;
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const RubiksState &node1, const RubiksState &node2);
	
	/** Heuristic value between node and the stored goal. Asserts that the
	 goal is stored **/
	virtual double HCost(const RubiksState &node);
	
	virtual double GCost(const RubiksState &node1, const RubiksState &node2) { return 1.0; }
	virtual double GCost(const RubiksState &node, const RubiksAction &act) { return 1.0; }
	virtual bool GoalTest(const RubiksState &node, const RubiksState &goal);
	
	/** Goal Test if the goal is stored **/
	virtual bool GoalTest(const RubiksState &node);
	
	virtual uint64_t GetStateHash(const RubiksState &node) const;
	virtual uint64_t GetActionHash(RubiksAction act) const { return act; }
	virtual void GetStateFromHash(uint64_t hash, RubiksState &node) const;
	
	FourBitArray &GetCornerPDB() { return cornerPDB; }
	FourBitArray &GetEdgePDB() { return edgePDB; }
	FourBitArray &GetEdge7PDB() { return edge7PDB; }

	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const RubiksState&) const;
	/** Draw the transition at some percentage 0...1 between two states */
	virtual void OpenGLDraw(const RubiksState&, const RubiksState&, float) const;
	virtual void OpenGLDraw(const RubiksState&, const RubiksAction&) const;
private:
	mutable std::vector<RubiksAction> history;
	RubiksCorner c;
	RubikEdge e;
	Rubik7Edge e7;
	FourBitArray cornerPDB;
	FourBitArray edgePDB;
	FourBitArray edge7PDB;

	DiskBitFile f;
	std::vector<bucketInfo> data;
	//std::vector<bucketData> buckets;

	bool pruneSuccessors;
};


#endif /* defined(__hog2_glut__RubiksCube__) */
