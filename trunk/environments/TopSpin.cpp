/*
 *  TopSpin.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/4/14.
 *  Copyright 2014 Nathan Sturtevant, University of Denver. All rights reserved.
 *
 */

#include "TopSpin.h"
#include "Heap.h"
#include "GraphEnvironment.h"
#include <map>

TopSpin::TopSpin(unsigned int N, unsigned int k)
:PermutationPuzzleEnvironment<TopSpinState, TopSpinAction>(), goal(N, k)
{
	pruneSuccessors = false;
	numTiles = N;
	swapDiameter = k;
	for (int x = 0; x < N; x++)
		operators.push_back(x);
}

TopSpin::~TopSpin()
{
	ClearGoal();
}

const std::string TopSpin::GetName(){
	std::stringstream name;
	name << "TopSpin(" << numTiles << ", " << swapDiameter << ")";
	
	return name.str();
}

void TopSpin::ComputeMovePruning()
{
	TopSpinState start(numTiles, swapDiameter);
	movePrune.resize(numTiles*numTiles);
	history.resize(0);
	RecursiveMovePruning(2, start);
	pruningMap.clear();
}

void TopSpin::RecursiveMovePruning(int depth, TopSpinState &state)
{
	if (depth == 0)
	{
		if (pruningMap.find(GetStateHash(state)) != pruningMap.end())
		{
			int last = history.size()-1;
			assert(last >= 1);
			movePrune[history[last]*numTiles+history[last-1]] = true; // prune this
			printf("Pruning the sequence %d %d\n", history[last-1], history[last]);
		}
		pruningMap[GetStateHash(state)] = true;
		return;
	}

	for (int x = 0; x < numTiles; x++)
	{
		ApplyAction(state, x);
		history.push_back(x);
		RecursiveMovePruning(depth-1, state);
		history.pop_back();
		UndoAction(state, x);
	}
}

void TopSpin::StoreGoal(TopSpinState &s)
{
}

bool TopSpin::GoalTest(const TopSpinState &s)
{
	for (int x = 0; x < numTiles; x++)
		if (s.puzzle[x] != x)
			return false;
	return true;
}

void TopSpin::GetSuccessors(const TopSpinState &stateID,
                             std::vector<TopSpinState> &neighbors) const
{
	neighbors.resize(0);
	for (unsigned int i = 0; i < numTiles; i++)
	{
		TopSpinState s(stateID);
		ApplyAction(s, i);
		neighbors.push_back(s);
	}
}

void TopSpin::GetActions(const TopSpinState &stateID, std::vector<TopSpinAction> &actions) const
{
	if (!pruneSuccessors || history.size() < 2)
	{
		actions = operators;
	}
	else {
		actions.resize(0);

		for (int x = 0; x < numTiles; x++)
		{
			if (!movePrune[x*numTiles+history.back()])
				actions.push_back(x);
		}
	}
}

void TopSpin::ApplyAction(TopSpinState &s, TopSpinAction a) const
{
	for (int x = 0; x < swapDiameter/2; x++)
	{
		std::swap(s.puzzle[(a+x)%numTiles], s.puzzle[(a+x+swapDiameter-1-2*x)%numTiles]);
	}
	if (pruneSuccessors)
		history.push_back(a);
}

void TopSpin::UndoAction(TopSpinState &s, TopSpinAction a) const
{
	if (pruneSuccessors && history.size() > 0)
	{
		assert(history.back() == a);
		history.pop_back();
	}
	for (int x = 0; x < swapDiameter/2; x++)
	{
		std::swap(s.puzzle[(a+x)%numTiles], s.puzzle[(a+x+swapDiameter-1-2*x)%numTiles]);
	}
}

bool TopSpin::InvertAction(TopSpinAction &a) const
{
	return true;
}

//double TopSpin::HCost(const TopSpinState &state)
//{
//	
//	double hval = 0;
//
//	for (unsigned int x = 0; x < PDB.size(); x++)
//	{
//		uint64_t index = GetPDBHash(state, PDB_distincts[x]);
//		hval = max(hval, PDB[x][index]);
//	}
//
////	if (PDB.size() != 0) // select between PDB and Manhattan distance if given the chance
////		hval = std::max(hval, DoPDBLookup(state));
//
//	return hval;
//}

// TODO Remove PDB heuristic from this heuristic evaluator.
double TopSpin::HCost(const TopSpinState &state1, const TopSpinState &state2)
{
	return PermutationPuzzleEnvironment<TopSpinState, TopSpinAction>::HCost(state1);
}

double TopSpin::GCost(const TopSpinState &, const TopSpinState &)
{
	return 1;
}

bool TopSpin::GoalTest(const TopSpinState &state, const TopSpinState &theGoal)
{
	return (state == theGoal);
}

uint64_t TopSpin::GetActionHash(TopSpinAction act) const
{
	return act;
}

void TopSpin::OpenGLDraw() const
{
}

void DrawCircle(float x, float y, float r1, float r2, int segments)
{
	float angleStep = TWOPI/segments;
	//	glBegin(GL_LINE_LOOP);
	glBegin(GL_QUAD_STRIP);
	for (float angle = 0; angle < TWOPI; angle += angleStep)
	{
		glVertex3f(x+cos(angle)*r1, y+sin(angle)*r1, 0);
		glVertex3f(x+cos(angle)*r2, y+sin(angle)*r2, 0);
	}
	glVertex3f(x+r1, y, 0);
	glVertex3f(x+r2, y, 0);
	glEnd();
}

static void DrawTile(float x, float y, char c1, char c2, int w, int h)
{
	glLineWidth(1.0);
	int textWidth = 0;
	int textHeight = 0;
	if (c1 != 0)
		textWidth += glutStrokeWidth(GLUT_STROKE_ROMAN, c1);
	//if (c2 != 0)
	textWidth += glutStrokeWidth(GLUT_STROKE_ROMAN, c2);
	//printf("%d\n", textWidth);
	glPushMatrix();
	//glColor3f(0.0, 0.0, 1.0);
	//glTranslatef(x*2.0/w-1.0, (1+y)*2.0/h-1.0-0.15, -0.001);
	glTranslatef(x, y, -0.001);
	glScalef(1.0/(w*120.0), 1.0/(h*120.0), 1);
	glRotatef(180, 0.0, 0.0, 1.0);
	glRotatef(180, 0.0, 1.0, 0.0);
	glTranslatef(-textWidth/2, -60, 0);
	if (c1 != 0)
		glutStrokeCharacter(GLUT_STROKE_ROMAN, c1);
	if (c2 != 0)
		glutStrokeCharacter(GLUT_STROKE_ROMAN, c2);
	//glTranslatef(-x/width+0.5, -y/height+0.5, 0);
	glPopMatrix();
	
//	glLineWidth(1.0);
//	glColor3f(1, 1, 1);
////	DrawCircle((x+0.5)*2.0/w-1-.05/w, (y+0.5)*2.0/h-1-.05/h, 1/w, 20);
//	glBegin(GL_QUADS);
//	glVertex3f(x*2.0/w-1+.05/w, (y)*2.0/h-1+.05/h, 0.002);
//	glVertex3f((x+1)*2.0/w-1-.05/w, (y)*2.0/h-1+.05/h, 0.002);
//	glVertex3f((x+1)*2.0/w-1-.05/w, (y+1)*2.0/h-1-.05/h, 0.002);
//	glVertex3f(x*2.0/w-1+.05/w, (y+1)*2.0/h-1-.05/h, 0.002);
//	glEnd();
}

static void DrawFrame(int w, int h)
{
}

void TopSpin::OpenGLDraw(const TopSpinState &s) const
{
	glEnable(GL_LINE_SMOOTH);
	char c1=0, c2=0;
	double diam = 2.0;
	diam /= s.puzzle.size();
	for (int x = 0; x < s.puzzle.size(); x++)
	{
		if (s.puzzle[x] > 9)
			c1 = '0'+(((s.puzzle[x])/10)%10);
		if (s.puzzle[x] >= 0)
			c2 = '0'+((s.puzzle[x])%10);
		
		glColor3f(1.0, 1.0, 1.0);
		DrawTile(-1+x*diam + diam/2, 0, c1, c2, s.puzzle.size(), s.puzzle.size());
		glLineWidth(2.0);
		glColor3f(0.0, 0.3, 0.8);
		DrawCircle(-1+x*diam + diam/2, 0, diam/2-diam/20, diam/2+diam/20, 50);
	}
	//	for (unsigned int y = 0; y < height; y++)
//	{
//		for (unsigned int x = 0; x < width; x++)
//		{
//			char c1=0, c2=0;
//			if (s.puzzle[x+y*width] > 9)
//				c1 = '0'+(((s.puzzle[x+y*width])/10)%10);
//			if (s.puzzle[x+y*width] > 0)
//				c2 = '0'+((s.puzzle[x+y*width])%10);
//			DrawTile(x, y, c1, c2, width, height);
//		}
//	}
//	DrawFrame(width, height);
}

void TopSpin::OpenGLDraw(const TopSpinState &s1, const TopSpinState &s2, float v) const
{
//	glEnable(GL_LINE_SMOOTH);
//	for (unsigned int y = 0; y < height; y++)
//	{
//		for (unsigned int x = 0; x < width; x++)
//		{
//			char c1=0, c2=0;
//			if (s2.puzzle[x+y*width] > 9)
//				c1 = '0'+(((s2.puzzle[x+y*width])/10)%10);
//			if (s2.puzzle[x+y*width] > 0)
//				c2 = '0'+((s2.puzzle[x+y*width])%10);
//
//			if (s1.puzzle[x+y*width] == s2.puzzle[x+y*width])
//			{
//				DrawTile(x, y, c1, c2, width, height);
//			}
//			else {
//				switch (GetAction(s1, s2))
//				{
//					case kUp: DrawTile(x, (y-1)*v + (y)*(1-v), c1, c2, width, height); break;
//					case kDown: DrawTile(x, (y+1)*v + (y)*(1-v), c1, c2, width, height); break;
//					case kLeft: DrawTile((x)*(1-v)+(x-1)*v, y, c1, c2, width, height); break;
//					case kRight: DrawTile((x+1)*v+(x)*(1-v), y, c1, c2, width, height); break;
//					default: assert(!"action not found");
//				}
//			}
//		}
//	}
//	DrawFrame(width, height);
}
