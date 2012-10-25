/*
 * $Id: sample.cpp,v 1.23 2006/11/01 23:33:56 nathanst Exp $
 *
 *  sample.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Copyright 2005 Nathan Sturtevant, University of Alberta. All rights reserved.
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

#include "Common.h"
#include "Sample.h"
//#include "UnitSimulation.h"
//#include "EpisodicSimulation.h"
//#include "Map2DEnvironment.h"
//#include "RandomUnits.h"
//#include "AStar.h"
//#include "TemplateAStar.h"
//#include "GraphEnvironment.h"
//#include "MapSectorAbstraction.h"
//#include "GraphRefinementEnvironment.h"
#include "LinearRegression.h"
#include "NN.h"

#define CLASSIFIER

struct data
{
	point3d loc;
	bool val;
};

std::vector<data> points;

//LinearRegression l(1, 1, 0.01);
NN l(1, 10, 1, 0.1);
LinearRegression cl(2, 1, 0.01);

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv);
}


/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kNoModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');
	
	InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a randomly moving unit", kShiftDown, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, '1');
	
	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	InstallCommandLineHandler(MyCLHandler, "-problems", "-problems filename sectorMultiplier", "Selects the problem set to run.");
	InstallCommandLineHandler(MyCLHandler, "-problems2", "-problems2 filename sectorMultiplier", "Selects the problem set to run.");
	InstallCommandLineHandler(MyCLHandler, "-size", "-batch integer", "If size is set, we create a square maze with the x and y dimensions specified.");
	
	InstallWindowHandler(MyWindowHandler);
	
	InstallMouseClickHandler(MyClickHandler);
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		SetNumPorts(windowID, 1);
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	glLineWidth(8.0);
	glColor3f(1, 1, 1);
	glBegin(GL_LINES);
	glVertex3f(-1, 0, 0);
	glVertex3f(1, 0, 0);
	glVertex3f(0, 1, 0);
	glVertex3f(0, -1, 0);
	
	for (unsigned int x = 0; x < points.size(); x++)
	{
		if (points[x].val)
			glColor3f(0, 1, 0);
		else
			glColor3f(1, 0, 0);
		glVertex3f(points[x].loc.x-0.02, points[x].loc.y-0.02, 0);
		glVertex3f(points[x].loc.x+0.02, points[x].loc.y+0.02, 0);
		glVertex3f(points[x].loc.x-0.02, points[x].loc.y+0.02, 0);
		glVertex3f(points[x].loc.x+0.02, points[x].loc.y-0.02, 0);
	}
	glEnd();
	
#ifndef CLASSIFIER
	glBegin(GL_LINE_STRIP);
	for (double x = -1; x <= 1; x += 1.0/64.0)
	{
//		if (random()%2)
//			glColor3f(1, 1, 1);
//		else
			glColor3f(1, 1, 0);
		std::vector<double> in(1);
		in[0] = x;
		double res = *l.test(in);
		res = (res-0.5)*5;
		glVertex3f(x, res, 0);
	}
	glEnd();
#else
	glPointSize(4.0);
	glBegin(GL_POINTS);
	std::vector<double> in(2);
	for (double x = -1; x <= 1; x += 0.03125*0.5)
	{
		for (double y = -1; y <= 1; y += 0.03125*0.5)
		{
			in[0] = x;
			in[1] = y;
			double res = *cl.test(in);
//			if (res > 0.5)
//				glColor4f(0, 1, 0, 0.5);
//			else
//				glColor4f(1, 0, 0, 0.5);
			glColor3f((1-res)*(1-res), res*res, 0);
			glVertex3f(x, y, 0);
		}
	}
	glEnd();
#endif
}


int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp( argument[0], "-map" ) == 0 )
	{
		return 2;
	}
	else if (strcmp( argument[0], "-size" ) == 0 )
	{
		return 2;
	}
	else if (strcmp(argument[0], "-problems" ) == 0 )
	{
		return 3;
	}
	else if (strcmp(argument[0], "-problems2" ) == 0 )
	{
		return 3;
	}
	return 2; //ignore typos
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '[': 
		case ']': break;
		case '\t':
			cl.setOutputActivation(kStep);
			for (unsigned int x = 0; x < points.size(); x++)
			{
				int which = random()%points.size();
				std::vector<double> in(2);
				std::vector<double> out(1);
				in[0] = points[which].loc.x;
				in[1] = points[which].loc.y;
				out[0] = points[which].val?1:0;
				//out[0] = points[x].val?1:-1;
				cl.train(in, out);
			}
			
			break;
		case 'p':
			points.resize(0);
			break;
		case 'o':
		{
			for (int z = 0; z < 1000; z++)
			for (unsigned int x = 0; x < points.size(); x++)
			{
				std::vector<double> in(1);
				std::vector<double> out(1);
				in[0] = points[x].loc.x;
				out[0] = points[x].loc.y/5.0+0.5;
				//out[0] = points[x].val?1:-1;
				l.train(in, out);
			}
		}
			break;
		default:
			break;
	}
}

void MyRandomUnitKeyHandler(unsigned long, tKeyboardModifier , char)
{

}

void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	printf("Hit (%f, %f, %f)\n", loc.x, loc.y, loc.z);
	//	mouseTracking = false;
	if (button == kLeftButton && mType == kMouseDown)
	{
		printf("Hit (%f, %f, %f)\n", loc.x, loc.y, loc.z);
		data d;
		d.loc = loc;
		d.val = true;
		points.push_back(d);
		//printf("Mouse down at (%d, %d)\n", px1, py1);
		return true;
	}
	if (button == kRightButton && mType == kMouseDown)
	{
		printf("Hit (%f, %f, %f)\n", loc.x, loc.y, loc.z);
		data d;
		d.loc = loc;
		d.val = false;
		points.push_back(d);
		//printf("Mouse down at (%d, %d)\n", px1, py1);
		return true;
	}
	return false;
}
