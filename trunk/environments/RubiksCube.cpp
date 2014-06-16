//
//  RubiksCube.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 4/6/13.
//  Copyright (c) 2013 University of Denver. All rights reserved.
//

#include "RubiksCube.h"
#include <cassert>
#include <cstdio>
#include <algorithm>

void RubiksCube::GetSuccessors(const RubiksState &nodeID, std::vector<RubiksState> &neighbors) const
{
	neighbors.resize(18);
	for (int x = 0; x < 18; x++)
	{
		GetNextState(nodeID, x, neighbors[x]);
	}
}

void RubiksCube::GetActions(const RubiksState &nodeID, std::vector<RubiksAction> &actions) const
{
	actions.resize(0);
	if (!pruneSuccessors || history.size() == 0)
	{
		for (int x = 0; x < 18; x++)
			actions.push_back(x);
	}
	else {
		// 0, 5, 2, 4, 1, 3

		for (int x = 0; x < 18; x++)
		{
			// 1. after any face you can't turn the same face again
			if (x/3 == history.back()/3)
				continue;

			// 2. after faces 5, 4, 3 you can't turn 0, 2, 1 respectively
			if ((1 == (history.back()/3)%2) &&
				(x/3+1 == history.back()/3))
				continue;
			
			actions.push_back(x);
		}
	}
//	std::random_shuffle(actions.begin(), actions.end());
}

RubiksAction RubiksCube::GetAction(const RubiksState &s1, const RubiksState &s2) const
{
	//std::vector<RubiksAction> succ;
	//GetActions(s1, succ);
	RubiksState tmp;
	for (int x = 0; x < 18; x++)
	{
		GetNextState(s1, x, tmp);
		if (tmp == s2)
			return x;
	}
	assert(false);
	return 0;
}

void RubiksCube::ApplyAction(RubiksState &s, RubiksAction a) const
{
	c.ApplyAction(s.corner, a);
	e.ApplyAction(s.edge, a);
	e7.ApplyAction(s.edge7, a);
	if (pruneSuccessors)
		history.push_back(a);
}

void RubiksCube::UndoAction(RubiksState &s, RubiksAction a) const
{
	if (pruneSuccessors && history.size() > 0)
	{
		assert(history.back() == a);
		history.pop_back();
	}
	InvertAction(a);
	c.ApplyAction(s.corner, a);
	e.ApplyAction(s.edge, a);
	e7.ApplyAction(s.edge7, a);

}

void RubiksCube::GetNextState(const RubiksState &s1, RubiksAction a, RubiksState &s2) const
{
	s2 = s1;
	ApplyAction(s2, a);
}

bool RubiksCube::InvertAction(RubiksAction &a) const
{
	if (2 == a%3)
		return true;
	if (1 == a%3)
	{
		a -= 1;
		return true;
	}
	a += 1;
	return true;
}

double RubiksCube::HCost(const RubiksState &node1, const RubiksState &node2, double parentHCost)
{
	double val = 0;
	
	// corner PDB
	uint64_t hash = c.GetStateHash(node1.corner);
	val = cornerPDB.Get(hash);
	if (val > parentHCost)
	{
		return val;
	}
	
	if (minBloomFilter)
	{
		int bloom = minBloom->Contains(node1.edge.state);
		if (bloom == 0xF) // not found
			bloom = 10;
		return max(val, double(bloom));
	}
	if (bloomFilter)
	{
		hash = node1.edge.state;//e.GetStateHash(node1.edge);

		// our maximum is 10
		if (val >= 10)
		{
			//if (val != HCost(node1, node2))
			//{ printf("Error! (1.5) [%f vs %f (%f)]\n", val, HCost(node1, node2), parentHCost); exit(0); }
			return val;
		}
		if (val < parentHCost-1) // we determined the heuristic
		{
			if (parentHCost >= 10)
			{
				if (depth9->Contains(hash))
				{
					val = max(val, 9);
					edgeDist[9]++;
				}
				else {
					val = max(val, 10);
					edgeDist[10]++;
				}
				// not an error - might be a 'faulty' bloom filter!
				//if (val != HCost(node1, node2))
				//{ printf("Error! (2) [%f vs %f (%f)]\n", val, HCost(node1, node2), parentHCost); exit(0); }
				return val;
			}	
			else if (parentHCost == 9)
			{
				if (depth8->Contains(hash))
				{
					val = max(val, 8);
					edgeDist[8]++;
				}
				else if (depth9->Contains(hash))
				{
					val = max(val, 9);
					edgeDist[9]++;
				}
				else {
					val = max(val, 10);
					edgeDist[10]++;
				}
				//if (val != HCost(node1, node2))
				//{ printf("Error! (3) [%f vs %f (%f)]\n", val, HCost(node1, node2), parentHCost); exit(0); }
				return val;
			}

		}

			auto depthLoc = depthTable.find(hash);
			if (depthLoc != depthTable.end())
			{
				val = max(val, depthLoc->second);
				edgeDist[depthLoc->second]++;
			}
			else if (depth8->Contains(hash))
			{
				val = max(val, 8);
				edgeDist[8]++;
			}
			else if (depth9->Contains(hash))
			{
				val = max(val, 9);
				edgeDist[9]++;
			}
			else {
				val = max(val, 10);
				edgeDist[10]++;
			}
			//if (val != HCost(node1, node2))
			//{ printf("Error! (4) [%f vs %f (%f)]\n", val, HCost(node1, node2), parentHCost); exit(0); }
			return val;
	}
	if (minCompression)
	{
		// edge PDB
		hash = e.GetStateHash(node1.edge);
		
		//	// edge PDB
		double val2 = edgePDB.Get(hash/compressionFactor);
		val = max(val, val2);
		
		node1.edge.GetDual(dual);
		hash = e.GetStateHash(dual);
		
		val2 = edgePDB.Get(hash/compressionFactor);
		val = max(val, val2);
	}
	else if (!minCompression) // interleave
	{
		// edge PDB
		hash = e7.GetStateHash(node1.edge7);
		
		//	// edge PDB
		if (0 == hash%compressionFactor)
		{
			double val2 = edge7PDBint.Get(hash/compressionFactor);
			val = max(val, val2);
		}
		node1.edge7.GetDual(e7dual);
		hash = e7.GetStateHash(e7dual);
		
		if (0 == hash%compressionFactor)
		{
			double val2 = edge7PDBint.Get(hash/compressionFactor);
			val = max(val, val2);
		}
	}
	
	return val;
}

/** Heuristic value between two arbitrary nodes. **/
double RubiksCube::HCost(const RubiksState &node1, const RubiksState &node2)
{
	double val = 0;
	
		// corner PDB
	uint64_t hash = c.GetStateHash(node1.corner);
	val = cornerPDB.Get(hash);

	// load PDB values directly from disk!
	// make sure that "data" is initialized with the right constructor calls for the data
    //int64_t r1, r2;
    //e.rankPlayer(node1.edge, 0, r1, r2);
    //double edge = f.ReadFileDepth(data[r1].bucketID, data[r1].bucketOffset+r2);
	//edgeDist[edge]++;
//    if (edge > 10)
//        edge = 10;
    //val = max(val, edge);
	//return val;

	if (bloomFilter)
	{
		/*  if (edge != depthLoc->second)
		  {
		  printf("Error - table doesn't match loaded PDB. table: %1.0f pdb: %1.0f\n", depthLoc->second, edge);
		  exit(0);
		  }*/

		hash = node1.edge.state;//e.GetStateHash(node1.edge);

		auto depthLoc = depthTable.find(hash);
		if (depthLoc != depthTable.end())
		{
			val = max(val, depthLoc->second);
			edgeDist[depthLoc->second]++;
		}
		else if (depth8->Contains(hash))
		{
			val = max(val, 8);
			edgeDist[8]++;
		}
		else if (depth9->Contains(hash))
		{
			val = max(val, 9);
			edgeDist[9]++;
		}
		else {
			val = max(val, 10);
			edgeDist[10]++;
		}
		return val;
	}
	
	if (minCompression)
	{
		// edge PDB
		hash = e.GetStateHash(node1.edge);

		//	// edge PDB
		double val2 = edgePDB.Get(hash/compressionFactor);
		val = max(val, val2);
		if (0)
		{
			node1.edge.GetDual(dual);
			hash = e.GetStateHash(dual);
			
			val2 = edgePDB.Get(hash/compressionFactor);
			val = max(val, val2);
		}
	}
	else if (!minCompression) // interleave
	{
		// edge PDB
		hash = e7.GetStateHash(node1.edge7);
		
		//	// edge PDB
		if (0 == hash%compressionFactor)
		{
			double val2 = edge7PDBint.Get(hash/compressionFactor);
			val = max(val, val2);
		}
		node1.edge7.GetDual(e7dual);
		hash = e7.GetStateHash(e7dual);
		
		if (0 == hash%compressionFactor)
		{
			double val2 = edge7PDBint.Get(hash/compressionFactor);
			val = max(val, val2);
		}
	}
	return val;
}

/** Heuristic value between node and the stored goal. Asserts that the
 goal is stored **/
double RubiksCube::HCost(const RubiksState &node)
{
	HCost(node, node);
}

bool RubiksCube::GoalTest(const RubiksState &node, const RubiksState &goal)
{
	return (node.corner.state == goal.corner.state &&
			node.edge.state == goal.edge.state);
}

/** Goal Test if the goal is stored **/
bool RubiksCube::GoalTest(const RubiksState &node)
{
	assert(false);
	return false;
}

uint64_t RubiksCube::GetStateHash(const RubiksState &node) const
{
	uint64_t hash = c.GetStateHash(node.corner);
	hash *= e.getMaxSinglePlayerRank();
	hash += e.GetStateHash(node.edge);
	return hash;
}

void RubiksCube::GetStateFromHash(uint64_t hash, RubiksState &node) const
{
	e.GetStateFromHash(hash%e.getMaxSinglePlayerRank(), node.edge);
	c.GetStateFromHash(hash/e.getMaxSinglePlayerRank(), node.corner);
}

void RubiksCube::OpenGLDraw() const
{
}

void RubiksCube::OpenGLDraw(const RubiksState&s) const
{
	e.OpenGLDraw(s.edge);
	c.OpenGLDraw(s.corner);
	OpenGLDrawCubeBackground();
	OpenGLDrawCenters();
}

void RubiksCube::OpenGLDrawCorners(const RubiksState&s) const
{
	c.OpenGLDraw(s.corner);
//	OpenGLDrawCenters();
	OpenGLDrawCubeBackground();
}

void RubiksCube::OpenGLDrawEdges(const RubiksState&s) const
{
//	Rubik7EdgeState e7tmp;
//	s.edge7.GetDual(e7tmp);
	e7.OpenGLDraw(s.edge7);
//	OpenGLDrawCenters();
	OpenGLDrawCubeBackground();
}

void RubiksCube::OpenGLDrawEdgeDual(const RubiksState&s) const
{
	s.edge.GetDual(dual);
	e.OpenGLDraw(dual);
//	OpenGLDrawCenters();
	OpenGLDrawCubeBackground();
}

void RubiksCube::OpenGLDrawCubeBackground() const
{
	float scale = 0.3;
	float offset = 0.95*scale/3.0;
	glBegin(GL_QUADS);
	
	glColor3f(0,0,0);
	offset = scale/3.0;
	scale*=0.99;
	glVertex3f(-3.0*offset, -scale, -3.0*offset);
	glVertex3f(3.0*offset, -scale, -3.0*offset);
	glVertex3f(3.0*offset, -scale, 3.0*offset);
	glVertex3f(-3.0*offset, -scale, 3.0*offset);
	
	glVertex3f(-3.0*offset, scale, -3.0*offset);
	glVertex3f(3.0*offset, scale, -3.0*offset);
	glVertex3f(3.0*offset, scale, 3.0*offset);
	glVertex3f(-3.0*offset, scale, 3.0*offset);
	
	glVertex3f(-scale, -3.0*offset, -3.0*offset);
	glVertex3f(-scale, 3.0*offset, -3.0*offset);
	glVertex3f(-scale, 3.0*offset, 3.0*offset);
	glVertex3f(-scale, -3.0*offset, 3.0*offset);
	
	//	SetFaceColor(3);
	glVertex3f(scale, -3.0*offset, -3.0*offset);
	glVertex3f(scale, 3.0*offset, -3.0*offset);
	glVertex3f(scale, 3.0*offset, 3.0*offset);
	glVertex3f(scale, -3.0*offset, 3.0*offset);
	
	//	SetFaceColor(2);
	glVertex3f(-3.0*offset, -3.0*offset, -scale);
	glVertex3f(3.0*offset, -3.0*offset, -scale);
	glVertex3f(3.0*offset, 3.0*offset, -scale);
	glVertex3f(-3.0*offset, 3.0*offset, -scale);
	
	//	SetFaceColor(4);
	glVertex3f(-3.0*offset, -3.0*offset, scale);
	glVertex3f(3.0*offset, -3.0*offset, scale);
	glVertex3f(3.0*offset, 3.0*offset, scale);
	glVertex3f(-3.0*offset, 3.0*offset, scale);
	glEnd();
}

void RubiksCube::OpenGLDrawCenters() const
{
	
	float scale = 0.3;
	float offset = 0.95*scale/3.0;
	glBegin(GL_QUADS);
	
	SetFaceColor(0);
	glVertex3f(-offset, -scale, -offset);
	glVertex3f(offset, -scale, -offset);
	glVertex3f(offset, -scale, offset);
	glVertex3f(-offset, -scale, offset);
	
	SetFaceColor(5);
	glVertex3f(-offset, scale, -offset);
	glVertex3f(offset, scale, -offset);
	glVertex3f(offset, scale, offset);
	glVertex3f(-offset, scale, offset);
	
	SetFaceColor(1);
	glVertex3f(-scale, -offset, -offset);
	glVertex3f(-scale, offset, -offset);
	glVertex3f(-scale, offset, offset);
	glVertex3f(-scale, -offset, offset);
	
	SetFaceColor(3);
	glVertex3f(scale, -offset, -offset);
	glVertex3f(scale, offset, -offset);
	glVertex3f(scale, offset, offset);
	glVertex3f(scale, -offset, offset);
	
	SetFaceColor(2);
	glVertex3f(-offset, -offset, -scale);
	glVertex3f(offset, -offset, -scale);
	glVertex3f(offset, offset, -scale);
	glVertex3f(-offset, offset, -scale);
	
	SetFaceColor(4);
	glVertex3f(-offset, -offset, scale);
	glVertex3f(offset, -offset, scale);
	glVertex3f(offset, offset, scale);
	glVertex3f(-offset, offset, scale);
	
//	glColor3f(0,0,0);
//	offset = scale/3.0;
//	scale*=0.99;
//	glVertex3f(-3.0*offset, -scale, -3.0*offset);
//	glVertex3f(3.0*offset, -scale, -3.0*offset);
//	glVertex3f(3.0*offset, -scale, 3.0*offset);
//	glVertex3f(-3.0*offset, -scale, 3.0*offset);
//	
//	glVertex3f(-3.0*offset, scale, -3.0*offset);
//	glVertex3f(3.0*offset, scale, -3.0*offset);
//	glVertex3f(3.0*offset, scale, 3.0*offset);
//	glVertex3f(-3.0*offset, scale, 3.0*offset);
//	
//	glVertex3f(-scale, -3.0*offset, -3.0*offset);
//	glVertex3f(-scale, 3.0*offset, -3.0*offset);
//	glVertex3f(-scale, 3.0*offset, 3.0*offset);
//	glVertex3f(-scale, -3.0*offset, 3.0*offset);
//	
//	//	SetFaceColor(3);
//	glVertex3f(scale, -3.0*offset, -3.0*offset);
//	glVertex3f(scale, 3.0*offset, -3.0*offset);
//	glVertex3f(scale, 3.0*offset, 3.0*offset);
//	glVertex3f(scale, -3.0*offset, 3.0*offset);
//	
//	//	SetFaceColor(2);
//	glVertex3f(-3.0*offset, -3.0*offset, -scale);
//	glVertex3f(3.0*offset, -3.0*offset, -scale);
//	glVertex3f(3.0*offset, 3.0*offset, -scale);
//	glVertex3f(-3.0*offset, 3.0*offset, -scale);
//	
//	//	SetFaceColor(4);
//	glVertex3f(-3.0*offset, -3.0*offset, scale);
//	glVertex3f(3.0*offset, -3.0*offset, scale);
//	glVertex3f(3.0*offset, 3.0*offset, scale);
//	glVertex3f(-3.0*offset, 3.0*offset, scale);
	glEnd();
}

void RubiksCube::SetFaceColor(int theColor) const
{
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


/** Draw the transition at some percentage 0...1 between two states */
void RubiksCube::OpenGLDraw(const RubiksState&, const RubiksState&, float) const
{
	
}

void RubiksCube::OpenGLDraw(const RubiksState&, const RubiksAction&) const
{
	
}
