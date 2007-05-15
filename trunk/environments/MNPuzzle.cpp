/*
 *  MNPuzzle.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/9/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "MNPuzzle.h"

MNPuzzle::MNPuzzle(int _width, int _height)
:width(_width), height(_height)
{
}

MNPuzzle::~MNPuzzle()
{
}

void MNPuzzle::GetSuccessors(MNPuzzleState &stateID, std::vector<MNPuzzleState> &neighbors)
{
}

void MNPuzzle::GetActions(MNPuzzleState &stateID, std::vector<slideDir> &actions)
{
	actions.resize(0);
	actions.push_back(kLeft);
	actions.push_back(kUp);
	actions.push_back(kDown);
	actions.push_back(kRight);
}

slideDir MNPuzzle::GetAction(MNPuzzleState &s1, MNPuzzleState &s2)
{
	return kUp;
}

void MNPuzzle::ApplyAction(MNPuzzleState &s, slideDir a)
{
	switch (a)
	{
		case kUp:
			if (s.blank >= s.width)
			{
				s.puzzle[s.blank] = s.puzzle[s.blank-s.width];
				s.blank -= s.width;
				s.puzzle[s.blank] = 0;
			}
			break;
		case kDown:
			if (s.blank < s.puzzle.size() - s.width)
			{
				s.puzzle[s.blank] = s.puzzle[s.blank+s.width];
				s.blank += s.width;
				s.puzzle[s.blank] = 0;
			}
			break;
		case kRight:
			if ((s.blank%s.width) < s.width-1)
			{
				s.puzzle[s.blank] = s.puzzle[s.blank+1];
				s.blank += 1;
				s.puzzle[s.blank] = 0;
			}
			break;
		case kLeft:
			if ((s.blank%s.width) > 0)
			{
				s.puzzle[s.blank] = s.puzzle[s.blank-1];
				s.blank -= 1;
				s.puzzle[s.blank] = 0;
			}
			break;
	}
}

double MNPuzzle::HCost(MNPuzzleState &state1, MNPuzzleState &state2)
{
	assert(state1.height==state2.height);
	assert(state1.width==state2.width);
	std::vector<int> xloc(state2.width*state2.height);
	std::vector<int> yloc(state2.width*state2.height);
	double hval = 0;

	for (unsigned int x = 0; x < state2.width; x++)
	{
		for (unsigned int y = 0; y < state2.height; y++)
		{
			xloc[state2.puzzle[x + y*state2.width]] = x;
			yloc[state2.puzzle[x + y*state2.width]] = y;
		}
	}
	for (unsigned int x = 0; x < state1.width; x++)
	{
		for (unsigned int y = 0; y < state1.height; y++)
		{
			if (state1.puzzle[x + y*state1.width] != 0)
			{
				hval += (abs(xloc[state1.puzzle[x + y*state1.width]] - x) +
								 abs(yloc[state1.puzzle[x + y*state1.width]] - y));
			}
		}
	}
	return hval;
}

double MNPuzzle::GCost(MNPuzzleState &state1, MNPuzzleState &state2)
{
	return 1;
}

bool MNPuzzle::GoalTest(MNPuzzleState &state, MNPuzzleState &goal)
{
	return (state == goal);
}

uint64_t MNPuzzle::GetStateHash(MNPuzzleState &state)
{
	uint64_t hashVal = 0;
	for (unsigned int x = 0; x < state.puzzle.size(); x++)
	{
		hashVal = hashVal*state.puzzle.size() + state.puzzle[x];
	}
	return hashVal;
}

uint64_t MNPuzzle::GetActionHash(slideDir act)
{
	switch (act)
	{
		case kUp: return 0;
		case kDown: return 1;
		case kRight: return 2;
		case kLeft: return 3;
	}
	return 4;
}

void MNPuzzle::OpenGLDraw(int window)
{
}


void MNPuzzle::OpenGLDraw(int window, MNPuzzleState &s)
{
	glLineWidth(1.0);
	glEnable(GL_LINE_SMOOTH);
	
	float w = width;
	float h = height;
	
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			glPushMatrix();
			glColor3f(0.0, 1.0, 0.0);
			glTranslatef(x*2.0/w-1.0, (1+y)*2.0/h-1.0, -0.001);
			glScalef(1.0/(w*120.0), 1.0/(h*120.0), 1);
			glRotatef(180, 0.0, 0.0, 1.0);
			glRotatef(180, 0.0, 1.0, 0.0);
			//glTranslatef((float)x/width-0.5, (float)y/height-0.5, 0);
			if (s.puzzle[x+y*width] > 9)
				glutStrokeCharacter(GLUT_STROKE_ROMAN, '0'+(((s.puzzle[x+y*width])/10)%10));
			if (s.puzzle[x+y*width] > 0)
				glutStrokeCharacter(GLUT_STROKE_ROMAN, '0'+((s.puzzle[x+y*width])%10));
			//glTranslatef(-x/width+0.5, -y/height+0.5, 0);
			glPopMatrix();
		}
	}
	
	glBegin(GL_LINES);
	for (int y = 0; y <= height; y++)
	{
		for (int x = 0; x <= width; x++)
		{
			glVertex3f(x*2.0/w-1.0, -1, -0.001);
			glVertex3f(x*2.0/w-1.0, 1, -0.001);
			glVertex3f(-1, (y)*2.0/h-1.0, -0.001);
			glVertex3f(1, (y)*2.0/h-1.0, -0.001);
		}
	}
	glEnd();
	
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glEnable(GL_BLEND);
	//glEnable(GL_LINE_SMOOTH);
	//output(200, 225, "This is antialiased.");
	
	//int width, height;
	//std::vector<int> puzzle;
}
