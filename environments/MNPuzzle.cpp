/*
 *  MNPuzzle.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/9/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */


#include "MNPuzzle.h"

void DrawTile(float x, float y, char c1, char c2, int w, int h)
{
	//glLineWidth(10.0);
	int textWidth = 0;
	//if (c1 != 0)
	//	textWidth += glutStrokeWidth(GLUT_STROKE_ROMAN, c1);
	//	if (c2 != 0)
	//	textWidth += glutStrokeWidth(GLUT_STROKE_ROMAN, c2);
	if (textWidth == 0)
		return;
	//printf("%d\n", textWidth);
	glPushMatrix();
	glColor3f(0.0, 0.0, 1.0);
	glTranslatef(x*2.0/w-1.0, (1+y)*2.0/h-1.0-0.15, -0.001);
	glScalef(1.0/(w*120.0), 1.0/(h*120.0), 1);
	glRotatef(180, 0.0, 0.0, 1.0);
	glRotatef(180, 0.0, 1.0, 0.0);
	glTranslatef(120-textWidth/2, 0, 0);
	//if (c1 != 0)
	//	glutStrokeCharacter(GLUT_STROKE_ROMAN, c1);
	//if (c2 != 0)
	//	glutStrokeCharacter(GLUT_STROKE_ROMAN, c2);
	//glTranslatef(-x/width+0.5, -y/height+0.5, 0);
	glPopMatrix();
	
	glLineWidth(1.0);
	glColor3f(1, 1, 1);
	glBegin(GL_QUADS);
	glVertex3f(x*2.0/w-1+.05/w, (y)*2.0/h-1+.05/h, 0.002);
	glVertex3f((x+1)*2.0/w-1-.05/w, (y)*2.0/h-1+.05/h, 0.002);
	glVertex3f((x+1)*2.0/w-1-.05/w, (y+1)*2.0/h-1-.05/h, 0.002);
	glVertex3f(x*2.0/w-1+.05/w, (y+1)*2.0/h-1-.05/h, 0.002);
	glEnd();
}

void DrawFrame(int w, int h)
{
	// frame
	glLineWidth(3.0);
	glColor3f(1.0, 1.0, 1.0);
	glBegin(GL_LINE_LOOP);
	glVertex3f(-1-.05/w, -1-.05/h, 0.002);
	glVertex3f(1+.05/w, -1-.05/h, 0.002);
	glVertex3f(1+.05/w, 1+.05/h, 0.002);
	glVertex3f(-1-.05/w, 1+.05/h, 0.002);
	glEnd();
	
	glColor3f(0.25, 0.25, 0.25);
	glBegin(GL_QUADS);
	glVertex3f(-1-.05/w, -1-.05/h, 0.003);
	glVertex3f(1+.05/w, -1-.05/h, 0.003);
	glVertex3f(1+.05/w, 1+.05/h, 0.003);
	glVertex3f(-1-.05/w, 1+.05/h, 0.003);
	glEnd();
	glLineWidth(1.0);
}
