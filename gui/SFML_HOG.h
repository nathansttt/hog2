/*
 *  $Id: main.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/18/06.
 *  Modified by Nathan Sturtevant on 06/22/21.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef SFML_HOG_H
#define SFML_HOG_H

#include "Common.h"

void createMenus();
void processMenuEvents(int option);
void keyPressed(unsigned char key, int x, int y);
void mouseMovedNoButton(int x, int y);
void mouseMovedButton(int x, int y);
void mousePressedButton(int button, int state, int x, int y);
static void mousePan (int x, int y, pRecContext pContextInfo);
static void mouseDolly (int x, int y, pRecContext pContextInfo);
void renderScene(void);
void buildGL(int x, int y);
void drawGL (pRecContext pContextInfo, sf::Window &window);
void drawCStringGL (char * cstrOut, GLuint fontList);
static void drawInfo (pRecContext pContextInfo);
void resizeWindow(int x, int y);
void resizeGL(pRecContext pContextInfo, int width, int height);
//void updateProjection(pRecContext pContextInfo, int viewPort = -1);
void updateModelView (pRecContext pContextInfo);
void pointPath();

void SetLighting(GLfloat ambientf = 0.2f, GLfloat diffusef = 1.0f, GLfloat specularf = 1.0f);

pRecContext getCurrentContext();
//static void drawCaps (pRecContext pContextInfo) {}

#endif // SFML_HOG_H
