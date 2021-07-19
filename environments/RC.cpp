//
//  RC.cpp (Alternate Rubik's Cube implementation)
//
//  Created by Nathan Sturtevant on 7/9/21.
//

#include "RC.h"
#include <cassert>
#include <cstdio>
#include <algorithm>
#include <string>

void RC::GetSuccessors(const RCState &nodeID, std::vector<RCState> &neighbors) const
{
	neighbors.resize(18);
	for (int x = 0; x < 18; x++)
	{
		GetNextState(nodeID, x, neighbors[x]);
	}
}

void RC::GetPrunedActions(const RCState &nodeID, RCAction lastAction, std::vector<RCAction> &actions) const
{
	actions.resize(0);
	for (int x = 0; x < 18; x++)
	{
		// 1. after any face you can't turn the same face again
		if (x/3 == lastAction/3)
			continue;
		
		// 2. after faces 5, 4, 3 you can't turn 0, 2, 1 respectively
		if ((1 == (lastAction/3)%2) &&
			(x/3+1 == lastAction/3))
			continue;
		
		actions.push_back(x);
	}
}


void RC::GetActions(const RCState &nodeID, std::vector<RCAction> &actions) const
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

RCAction RC::GetAction(const RCState &s1, const RCState &s2) const
{
	//std::vector<RCAction> succ;
	//GetActions(s1, succ);
	RCState tmp;
	for (int x = 0; x < 18; x++)
	{
		GetNextState(s1, x, tmp);
		if (tmp == s2)
			return x;
	}
	assert(false);
	return 0;
}

void RC::ApplyAction(RCState &s, RCAction a) const
{
	// TODO: Write
}

void RC::UndoAction(RCState &s, RCAction a) const
{
	// TODO: Write
}

void RC::GetNextState(const RCState &s1, RCAction a, RCState &s2) const
{
	s2 = s1;
	ApplyAction(s2, a);
}

bool RC::InvertAction(RCAction &a) const
{
	// TODO: Test
	if (2 == a%3)
		return true;
	if (1 == a%3)
	{
		a -= 1;
		return true;
	}
	a += 1;
	return true;
	return false;
}

double RC::HCost(const RCState &node1, const RCState &node2, double parentHCost) const
{
	return HCost(node1, node2);
}

/** Heuristic value between two arbitrary nodes. **/
double RC::HCost(const RCState &node1, const RCState &node2) const
{
	return 0;
}

/** Heuristic value between node and the stored goal. Asserts that the
 goal is stored **/
double RC::HCost(const RCState &node) const
{
	return HCost(node, node);
}

bool RC::GoalTest(const RCState &node, const RCState &goal) const
{
	return node == goal;
}

/** Goal Test if the goal is stored **/
bool RC::GoalTest(const RCState &node) const
{
	assert(false);
	return false;
}

uint64_t RC::GetStateHash(const RCState &node) const
{
	// TODO: write
	return 0;
}

void RC::GetStateFromHash(uint64_t hash, RCState &node) const
{
// TODO: write
}

void RC::OpenGLDraw() const
{
}

void RC::OpenGLDraw(const RCState&s) const
{
//	OpenGLDrawCubeBackground();
//	e.OpenGLDraw(s.edge);
//	c.OpenGLDraw(s.corner);
//	OpenGLDrawCenters();
}

void RC::OpenGLDrawCorners(const RCState&s) const
{
//	OpenGLDrawCubeBackground();
//	c.OpenGLDraw(s.corner);
//	OpenGLDrawCenters();
}

void RC::OpenGLDrawEdges(const RCState&s) const
{
//	OpenGLDrawCubeBackground();
////	Rubik7EdgeState e7tmp;
////	s.edge7.GetDual(e7tmp);
//	e.OpenGLDraw(s.edge);
//	OpenGLDrawCenters();
}

void RC::OpenGLDrawEdgeDual(const RCState&s) const
{
//	OpenGLDrawCubeBackground();
//	s.edge.GetDual(dual);
//	e.OpenGLDraw(dual);
//	OpenGLDrawCenters();
}

void RC::OpenGLDrawCubeBackground() const
{
//{
////	return;
//
//	float scale = 0.3;
//	float offset = 0.95*scale/3.0;
//	glBegin(GL_QUADS);
//
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
//	glEnd();
//}
}

void RC::OpenGLDrawCenters() const
{
//	glBegin(GL_QUADS);
//	OpenGLDrawCube(4);
//	OpenGLDrawCube(10);
//	OpenGLDrawCube(12);
//	OpenGLDrawCube(14);
//	OpenGLDrawCube(16);
//	OpenGLDrawCube(22);
//	glEnd();
}

void RC::OpenGLDrawCube(int cube) const
{
//{
//	float scale = 0.3;
//	float offset = 0.95*scale/3.0;
//	const float offset2 = scale/3.0;
//	const float epsilon = 0.002;
//
//	switch (cube)
//	{
//		case 4:
//		{
//			SetFaceColor(0);
//			glVertex3f(-offset, -scale, -offset);
//			glVertex3f(offset, -scale, -offset);
//			glVertex3f(offset, -scale, offset);
//			glVertex3f(-offset, -scale, offset);
//
//			SetFaceColor(-1);
//			glVertex3f(-offset2, -scale+epsilon, -offset2);
//			glVertex3f(offset2, -scale+epsilon, -offset2);
//			glVertex3f(offset2, -scale+epsilon, offset2);
//			glVertex3f(-offset2, -scale+epsilon, offset2);
//
//		} break;
//		case 22:
//		{
//			SetFaceColor(5);
//			glVertex3f(-offset, scale, -offset);
//			glVertex3f(offset, scale, -offset);
//			glVertex3f(offset, scale, offset);
//			glVertex3f(-offset, scale, offset);
//
//			SetFaceColor(-1);
//			glVertex3f(-offset2, scale-epsilon, -offset2);
//			glVertex3f(offset2, scale-epsilon, -offset2);
//			glVertex3f(offset2, scale-epsilon, offset2);
//			glVertex3f(-offset2, scale-epsilon, offset2);
//		} break;
//		case 12:
//		{
//			SetFaceColor(1);
//			glVertex3f(-scale, -offset, -offset);
//			glVertex3f(-scale, offset, -offset);
//			glVertex3f(-scale, offset, offset);
//			glVertex3f(-scale, -offset, offset);
//
//			SetFaceColor(-1);
//			glVertex3f(-scale+epsilon, -offset2, -offset2);
//			glVertex3f(-scale+epsilon, offset2, -offset2);
//			glVertex3f(-scale+epsilon, offset2, offset2);
//			glVertex3f(-scale+epsilon, -offset2, offset2);
//
//		} break;
//		case 16:
//		{
//			SetFaceColor(4);
//			glVertex3f(-offset, -offset, scale);
//			glVertex3f(offset, -offset, scale);
//			glVertex3f(offset, offset, scale);
//			glVertex3f(-offset, offset, scale);
//
//			SetFaceColor(-1);
//			glVertex3f(-offset2, -offset2, scale-epsilon);
//			glVertex3f(offset2, -offset2, scale-epsilon);
//			glVertex3f(offset2, offset2, scale-epsilon);
//			glVertex3f(-offset2, offset2, scale-epsilon);
//
//		} break;
//		case 10:
//		{
//			SetFaceColor(2);
//			glVertex3f(-offset, -offset, -scale);
//			glVertex3f(offset, -offset, -scale);
//			glVertex3f(offset, offset, -scale);
//			glVertex3f(-offset, offset, -scale);
//
//			SetFaceColor(-1);
//			glVertex3f(-offset2, -offset2, -scale+epsilon);
//			glVertex3f(offset2, -offset2, -scale+epsilon);
//			glVertex3f(offset2, offset2, -scale+epsilon);
//			glVertex3f(-offset2, offset2, -scale+epsilon);
//		} break;
//		case 14:
//		{
//			SetFaceColor(3);
//			glVertex3f(scale, -offset, -offset);
//			glVertex3f(scale, offset, -offset);
//			glVertex3f(scale, offset, offset);
//			glVertex3f(scale, -offset, offset);
//
//			SetFaceColor(-1);
//			glVertex3f(scale-epsilon, -offset2, -offset2);
//			glVertex3f(scale-epsilon, offset2, -offset2);
//			glVertex3f(scale-epsilon, offset2, offset2);
//			glVertex3f(scale-epsilon, -offset2, offset2);
//		} break;
//	}
//}
}

void RC::SetFaceColor(int theColor) const
{
//	switch (theColor)
//	{
//		case -1: glColor3f(0.0, 0.0, 0.0); break;
//		case 0: glColor3f(1.0, 0.0, 0.0); break;
//		case 1: glColor3f(0.0, 1.0, 0.0); break;
//		case 2: glColor3f(0.0, 0.0, 1.0); break;
//		case 3: glColor3f(1.0, 1.0, 0.0); break;
//		case 4: glColor3f(1.0, 0.75, 0.0); break;
//		case 5: glColor3f(1.0, 1.0, 1.0); break;
//		default: assert(false);
//	}
}


/** Draw the transition at some percentage 0...1 between two states */
void RC::OpenGLDraw(const RCState &s1, const RCState &s2, float t) const
{
////	glEnable( GL_POLYGON_SMOOTH );
////	glHint( GL_POLYGON_SMOOTH_HINT, GL_NICEST );
//	int vals[3] = {-90, 90, 180};
//	RCAction a = GetAction(s1, s2);
//	switch (a/3)
//	{
//		case 0: // 0
//		{
//			glPushMatrix();
//			glRotatef(vals[a%3]*t, 0, 1, 0); // parameterize
//			glBegin(GL_QUADS);
//			for (int x = 0; x < 9; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, x);
//				c.OpenGLDrawCube(s1.corner, x);
//				OpenGLDrawCube(x);
//			}
//			glEnd();
//			glPopMatrix();
//			
//			glBegin(GL_QUADS);
//			for (int x = 9; x < 27; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, x);
//				c.OpenGLDrawCube(s1.corner, x);
//				OpenGLDrawCube(x);
//			}
//			glEnd();
//		} break;
//		case 1: // 5
//		{
//			glPushMatrix();
//			glRotatef(vals[a%3]*t, 0, 1, 0); // parameterize
//			glBegin(GL_QUADS);
//			for (int x = 18; x < 27; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, x);
//				c.OpenGLDrawCube(s1.corner, x);
//				OpenGLDrawCube(x);
//			}
//			glEnd();
//			glPopMatrix();
//			
//			glBegin(GL_QUADS);
//			for (int x = 0; x < 18; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, x);
//				c.OpenGLDrawCube(s1.corner, x);
//				OpenGLDrawCube(x);
//			}
//			glEnd();
//		} break;
//		case 2: // 2
//		{
//			int which[9] = {0, 1, 2, 9, 10, 11, 18, 19, 20};
//			int others[18] = {3, 4, 5, 6, 7, 8, 12, 13, 14, 15, 16, 17, 21, 22, 23, 24, 25, 26};
//			glPushMatrix();
//			glRotatef(vals[a%3]*t, 0, 0, -1); // parameterize
//			glBegin(GL_QUADS);
//			for (int x = 0; x < 9; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, which[x]);
//				c.OpenGLDrawCube(s1.corner, which[x]);
//				OpenGLDrawCube(which[x]);
//			}
//			glEnd();
//			glPopMatrix();
//			
//			glBegin(GL_QUADS);
//			for (int x = 0; x < 18; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, others[x]);
//				c.OpenGLDrawCube(s1.corner, others[x]);
//				OpenGLDrawCube(others[x]);
//			}
//			glEnd();
//		} break;
//		case 3: // 4
//		{
//			int which[9] = {6, 7, 8, 15, 16, 17, 24, 25, 26};
//			int others[18] = {3, 4, 5, 0, 1, 2, 12, 13, 14, 9, 10, 11, 21, 22, 23, 18, 19, 20};
//			glPushMatrix();
//			glRotatef(vals[a%3]*t, 0, 0, 1); // parameterize
//			glBegin(GL_QUADS);
//			for (int x = 0; x < 9; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, which[x]);
//				c.OpenGLDrawCube(s1.corner, which[x]);
//				OpenGLDrawCube(which[x]);
//			}
//			glEnd();
//			glPopMatrix();
//			
//			glBegin(GL_QUADS);
//			for (int x = 0; x < 18; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, others[x]);
//				c.OpenGLDrawCube(s1.corner, others[x]);
//				OpenGLDrawCube(others[x]);
//			}
//			glEnd();
//		} break;
//		case 4: // 1
//		{
//			int which[9] = {0, 3, 6, 9, 12, 15, 18, 21, 24};
//			int others[18] = {1, 2, 4, 5, 7, 8, 10, 11, 13, 14, 16, 17, 19, 20, 22, 23, 25, 26};
//			glPushMatrix();
//			glRotatef(vals[a%3]*t, -1, 0, 0); // parameterize
//			glBegin(GL_QUADS);
//			for (int x = 0; x < 9; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, which[x]);
//				c.OpenGLDrawCube(s1.corner, which[x]);
//				OpenGLDrawCube(which[x]);
//			}
//			glEnd();
//			glPopMatrix();
//			
//			glBegin(GL_QUADS);
//			for (int x = 0; x < 18; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, others[x]);
//				c.OpenGLDrawCube(s1.corner, others[x]);
//				OpenGLDrawCube(others[x]);
//			}
//			glEnd();
//			
//		} break;
//		case 5: // 3
//		{
//			int which[9] = {2, 5, 8, 11, 14, 17, 20, 23, 26};
//			int others[18] = {0, 1, 3, 4, 6, 7, 9, 10, 12, 13, 15, 16, 18, 19, 21, 22, 24, 25};
//			glPushMatrix();
//			glRotatef(vals[a%3]*t, 1, 0, 0); // parameterize
//			glBegin(GL_QUADS);
//			for (int x = 0; x < 9; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, which[x]);
//				c.OpenGLDrawCube(s1.corner, which[x]);
//				OpenGLDrawCube(which[x]);
//			}
//			glEnd();
//			glPopMatrix();
//			
//			glBegin(GL_QUADS);
//			for (int x = 0; x < 18; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, others[x]);
//				c.OpenGLDrawCube(s1.corner, others[x]);
//				OpenGLDrawCube(others[x]);
//			}
//			glEnd();
//		} break;
//	}
}

void RC::OpenGLDraw(const RCState&, const RCAction&) const
{
	
}

// Draw a RCState. Internally do the 3d transform for the drawing.
void RC::Draw(Graphics::Display &display, const RCState&) const
{
}
