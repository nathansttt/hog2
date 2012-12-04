//
//  NQueens.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 9/18/11.
//  Copyright 2011 University of Denver. All rights reserved.
//

#include "NQueens.h"

void NQueens::GetSuccessors(const NQueenState &nodeID, std::vector<NQueenState> &neighbors) const
{
	
}

void NQueens::GetActions(const NQueenState &nodeID, std::vector<NQueenAction> &actions) const
{
	for (unsigned int x = 0; x < nodeID.locs.size(); x++)
		for (unsigned int y = 0; y < nodeID.locs.size(); y++)
		{
			if (x == y)
				continue;

			NQueenAction a;
			a.loc = x;
			a.value = y;
			actions.push_back(a);
		}
}

NQueenAction NQueens::GetAction(const NQueenState &s1, const NQueenState &s2) const
{
	assert(false);
	NQueenAction a;
	return a;
}

void NQueens::ApplyAction(NQueenState &s, NQueenAction a) const
{
	s.locs[a.loc] = a.value;
}

void NQueens::GetNextState(const NQueenState &s, NQueenAction a, NQueenState &s2) const
{
	s2 = s;
	ApplyAction(s2, a);
}

/** Goal Test if the goal is stored **/
bool NQueens::GoalTest(const NQueenState &node)
{
	for (int x = 0; x < node.locs.size(); x++)
	{
		int up = node.locs[x];
		if (up == -1) return false;		
		int down = up;
		int middle = up;
		
		for (int y = 1; y < node.locs.size(); y++)
		{
			up -= 1;
			down += 1;
			
			if ((x-y < 0) && (x+y >= node.locs.size()))
				break;

			if ((x+y < node.locs.size()) &&
				((node.locs[x+y] == up) ||
				 (node.locs[x+y] == down) ||
				 (node.locs[x+y] == middle)))
			{
				return false;
			}
			if ((x-y >= 0) &&
				((node.locs[x-y] == up) ||
				 (node.locs[x-y] == down) ||
				 (node.locs[x-y] == middle)))
			{
				return false;
			}
		}
	}
	return true;
}

int NQueens::NumCollisions(const NQueenState &node) const
{
	int count = 0;
	for (int x = 0; x < node.locs.size(); x++)
	{
		int up = node.locs[x];
		if (up == -1) 
		{
			// ill formed state
			continue;
		}
		int down = up;
		int middle = up;
		
		for (int y = 1; y < node.locs.size(); y++)
		{
			up -= 1;
			down += 1;

			if ((x-y < 0) && (x+y >= node.locs.size()))
				break;
			
			if (x+y < node.locs.size())
			{
				if (node.locs[x+y] == up) count++;
				if (node.locs[x+y] == down) count++;
				if (node.locs[x+y] == middle) count++;
			}
			if (x-y >= 0)
			{
				if (node.locs[x-y] == up) count++;
				if (node.locs[x-y] == down) count++;
				if (node.locs[x-y] == middle) count++;
			}
		}
	}
	return count;
}

void NQueens::OpenGLDraw() const
{
	// no basic environment to draw
}

void NQueens::OpenGLDraw(const NQueenState&s) const
{		
	double fract = s.locs.size();
	fract = 2.0/fract;
	glBegin(GL_LINES);
	glColor3f(0.0, 1.0, 0.0);
	for (unsigned int x = 0; x <= s.locs.size(); x++)
	{
		glVertex3d(-1.0+x*fract, -1, 0);
		glVertex3d(-1.0+x*fract,  1, 0);

		glVertex3d(-1, -1.0+x*fract, 0);
		glVertex3d( 1, -1.0+x*fract, 0);
	}
	glEnd();
	glBegin(GL_QUADS);
	if (NumCollisions(s) == 0)
		glColor3f(0.0, 0.1, 0.6);
	else
		glColor3f(1.0, 0.1, 0.6);
	for (unsigned int x = 0; x < s.locs.size(); x++)
	{
		glVertex3d(-1.0+x*fract, -1.0+s.locs[x]*fract, 0.0);
		glVertex3d(-1.0+x*fract, -1.0+(1+s.locs[x])*fract, 0.0);
		glVertex3d(-1.0+(x+1)*fract, -1.0+(1+s.locs[x])*fract, 0.0);
		glVertex3d(-1.0+(x+1)*fract, -1.0+s.locs[x]*fract, 0.0);
	}
	glEnd();


	glColor3f(1.0, 0, 0);
	glLineWidth(3.0);
	glBegin(GL_LINES);
	for (int x = 0; x < s.locs.size(); x++)
	{
		int up = s.locs[x];
		if (up == -1) continue;
		int down = up;		
		for (int y = 1; y < s.locs.size(); y++)
		{
			up -= 1;
			down += 1;
			if (y == 1)
				continue;
			
			// root location
			glVertex3d(-1.0+x*fract+fract*0.5+(y-1)*fract, -1.0+s.locs[x]*fract+fract*0.5, 0.0);
			// offset: right
			glVertex3d(-1.0+x*fract+fract*0.5+y*fract, -1.0+s.locs[x]*fract+fract*0.5, 0.0);

			// root location
			glVertex3d(-1.0+x*fract+fract*0.5-(y-1)*fract, -1.0+s.locs[x]*fract+fract*0.5, 0.0);
			// offset: left
			glVertex3d(-1.0+x*fract+fract*0.5-y*fract, -1.0+s.locs[x]*fract+fract*0.5, 0.0);

			// root location
			glVertex3d(-1.0+x*fract+fract*0.5-(y-1)*fract, -1.0+(up+1)*fract+fract*0.5, 0.0);
			// offset: up-left
			glVertex3d(-1.0+x*fract+fract*0.5-y*fract, -1.0+up*fract+fract*0.5, 0.0);
		
			// root location
			glVertex3d(-1.0+x*fract+fract*0.5-(y-1)*fract, -1.0+(down-1)*fract+fract*0.5, 0.0);
			// offset: down-left
			glVertex3d(-1.0+x*fract+fract*0.5-y*fract, -1.0+down*fract+fract*0.5, 0.0);

			// root location
			glVertex3d(-1.0+x*fract+fract*0.5+(y-1)*fract, -1.0+(up+1)*fract+fract*0.5, 0.0);
			// offset: up-right
			glVertex3d(-1.0+x*fract+fract*0.5+y*fract, -1.0+up*fract+fract*0.5, 0.0);
			
			// root location
			glVertex3d(-1.0+x*fract+fract*0.5+(y-1)*fract, -1.0+(down-1)*fract+fract*0.5, 0.0);
			// offset: down-right
			glVertex3d(-1.0+x*fract+fract*0.5+y*fract, -1.0+down*fract+fract*0.5, 0.0);

		}
	}
	glEnd();
	glLineWidth(1.0);
}

/** Draw the transition at some percentage 0...1 between two states */
void NQueens::OpenGLDraw(const NQueenState&, const NQueenState&, float) const
{
	
}

void NQueens::OpenGLDraw(const NQueenState&, const NQueenAction&) const
{
	return;
}
