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

void MNPuzzle::GetSuccessors(MNPuzzleState stateID, std::vector<MNPuzzleState> &neighbors)
{
}

void MNPuzzle::GetActions(MNPuzzleState stateID, std::vector<slideDir> &actions)
{
}

slideDir MNPuzzle::GetAction(MNPuzzleState s1, MNPuzzleState s2)
{
}

void MNPuzzle::ApplyAction(MNPuzzleState &s, slideDir a)
{
}

double MNPuzzle::HCost(MNPuzzleState state1, MNPuzzleState state2)
{
}

double MNPuzzle::GCost(MNPuzzleState state1, MNPuzzleState state2)
{
}

bool MNPuzzle::GoalTest(MNPuzzleState state, MNPuzzleState goal)
{
}

uint64_t MNPuzzle::GetStateHash(MNPuzzleState state)
{
}

uint64_t MNPuzzle::GetActionHash(slideDir act)
{
}

void MNPuzzle::OpenGLDraw(int window)
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
			if (x+y*width > 9)
				glutStrokeCharacter(GLUT_STROKE_ROMAN, '0'+(((x+y*width)/10)%10));
			glutStrokeCharacter(GLUT_STROKE_ROMAN, '0'+((x+y*width)%10));
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

