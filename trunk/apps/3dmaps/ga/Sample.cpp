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
#include "NQueens.h"

const int populationSize = 8;
const int childSize = populationSize*(populationSize-1);

std::vector<NQueenState> parents(populationSize);
std::vector<NQueenState> children(childSize);

struct splitInfo {
	int parent1, parent2, splitLoc;
};

std::vector<splitInfo> childrenInfo(childSize);


NQueens q;

void BuildChildren();
void ChooseNewParents();

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv);
}

/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
void CreateSimulation(int id)
{
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Reset", "Reset to random state", kNoModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kNoModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');
	
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
		
		for (int x = 0; x < parents.size(); x++)
			parents[x] = NQueenState();

		BuildChildren();
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	NQueens env;
	for (int x = 0; x < populationSize; x++)
	{
		glPushMatrix();

		float scale = 1/float(populationSize);
		float inv = float(populationSize);
		glScalef(scale, scale, 1);
		glTranslatef(-inv+1./2.+2.1*x, -inv+1./2., 0);
		env.OpenGLDraw(parents[x]);
		recColor c = getColor(x, 0, populationSize-1, 1);// 1, 4
		env.OpenGLDrawBackground(c.r*0.5, c.g*0.5, c.b*0.5);
		glPopMatrix();
	}
	for (int x = 0; x < childSize; x++)
	{
		glPushMatrix();
		
		float scale = 1/float(populationSize);
		float inv = float(populationSize);
		glScalef(scale, scale, 1);
		float realx = x%populationSize;
		float realy = 1+x/populationSize;
		glTranslatef(-inv+1./2.+2.1*realx, -inv+1./2.+2.1*realy, 0);
		env.OpenGLDraw(children[x]);

		recColor c = getColor(childrenInfo[x].parent1, 0, populationSize-1, 1);// 1, 4
		env.OpenGLDrawBackground(children[x], c.r*0.5, c.g*0.5, c.b*0.5, 0, childrenInfo[x].splitLoc+1);
		c = getColor(childrenInfo[x].parent2, 0, populationSize-1, 1);// 1, 4
		env.OpenGLDrawBackground(children[x], c.r*0.5, c.g*0.5, c.b*0.5, childrenInfo[x].splitLoc+1, 8);
		glPopMatrix();
	}
	//	MyDisplayHandler(windowID, kNoModifier, 'o');
}


int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-map") == 0)
	{
		if (maxNumArgs <= 1)
			return 0;
		strncpy(gDefaultMap, argument[1], 1024);

		return 2;
	}
	return 0;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '[': break;
		case ']':  break;
		case '\t':
			if (mod != kShiftDown)
				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
			else
			{
				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
			}
			break;
		case 'r':
		{
			for (int x = 0; x < parents.size(); x++)
				parents[x] = NQueenState();
		}
			break;
		case 'p':
		{
			ChooseNewParents();
		}
			break;
		case 'o':
		{
			BuildChildren();
		}
			break;
		default:
			break;
	}
}

void MyRandomUnitKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
}

void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	if (button == kRightButton)
	{
		switch (mType)
		{
			case kMouseDown:
				break;
			case kMouseDrag:
				break;
			case kMouseUp:
				break;
		}
		return true;
	}
	return false;
}

void BuildChildren()
{
	for (int x = 0; x < children.size(); x++)
	{
		int p1 = random()%parents.size();
		int p2;
		do {
			p2 = random()%parents.size();
		} while (p1 == p2);

		int crossover = random()%(parents[p1].locs.size()-1);
		childrenInfo[x].parent1 = p1;
		childrenInfo[x].parent2 = p2;
		childrenInfo[x].splitLoc = crossover;
		for (int t = 0; t <= crossover; t++)
		{
			children[x].locs[t] = parents[p1].locs[t];
		}
		for (int t = crossover+1; t <= parents[p2].locs.size(); t++)
		{
			children[x].locs[t] = parents[p2].locs[t];
		}
		children[x].locs[random()%children[x].locs.size()] = random()%children[x].locs.size();
	}
}

void ChooseNewParents()
{
	std::vector<bool> chosen(childSize);

	for (int x = 0; x < populationSize; x++)
	{
		int best = 0;
		for (int y = 0; y < children.size(); y++)
		{
			if (chosen[best] || (q.NumCollisions(children[y]) < q.NumCollisions(children[best]) && !chosen[y]))
				best = y;
		}
		parents[x] = children[best];
		chosen[best] = true;
	}
}
