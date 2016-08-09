/*
 * $Id: main.h,v 1.10 2006/10/18 23:53:10 nathanst Exp $
 *
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#ifndef OS_MAC

/* Apple CGSize */
struct CGSize {
   float width;
   float height;
};

typedef struct CGSize CGSize;


/* Apple CGPoint */
struct CGPoint {
   float x;
   float y;
};
typedef struct CGPoint CGPoint;


/* Apple CGRect */
struct CGRect {
   CGPoint origin;
   CGSize size;
};
typedef struct CGRect CGRect;

#endif

void createMenus();
void processMenuEvents(int option);
void keyPressed(unsigned char key, int x, int y);
void mouseMovedNoButton(int x, int y);
void mouseMovedButton(int x, int y);
void mousePressedButton(int button, int state, int x, int y);
static void mousePan (int x, int y, pRecContext pContextInfo);
static void mouseDolly (int x, int y, pRecContext pContextInfo);
void renderScene(void);
void buildGL(void);
void drawGL(pRecContext pContextInfo);
//void trajectoryDrawGL(pRecContext pContextInfo);  // WESHACK - for drawing trajectories at the end of our 'multipleAStarMM' simulation run
void drawCStringGL (char * cstrOut, GLuint fontList);
static void drawInfo (pRecContext pContextInfo);
void resizeWindow(int x, int y);
void resizeGL(pRecContext pContextInfo, CGRect viewRect);
//void updateProjection (pRecContext pContextInfo);
void updateModelView (pRecContext pContextInfo);
void pointPath();
int processFramesPerSecond(char *argument[], int maxNumArgs);

pRecContext getCurrentContext();
//static void drawCaps (pRecContext pContextInfo) {}

bool startTrajRecap;
