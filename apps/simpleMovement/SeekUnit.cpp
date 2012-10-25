/*
 *  SeekAgent.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/12/11.
 *  Copyright 2011 University of Denver. All rights reserved.
 *
 */

#include "SeekUnit.h"

bool SeekUnit::MakeMove(SteeringEnvironment *,
						OccupancyInterface<steeringState, steeringAction> *,
						SimulationInfo<steeringState, steeringAction, SteeringEnvironment> *,
						steeringAction& a)
{
	if (targets.size() > 0)
	{
		history.push_back(currentLoc);
		goalLoc = targets[0];
	}
//	printf("At: (%f, %f)\n", currentLoc.x, currentLoc.y);
//	printf("Goal: (%f, %f) [%f]\n", goalLoc.x, goalLoc.y,
//		   (currentLoc.x-goalLoc.x)*(currentLoc.x-goalLoc.x)+(currentLoc.y-goalLoc.y)*(currentLoc.y-goalLoc.y));
	float dx = goalLoc.x-currentLoc.x;
	float dy = goalLoc.y-currentLoc.y;

	a.dv = 1;
	float goalHeading;
	if (dx == 0)
	{
		if (dy > 0)
			goalHeading = 90;
		else {
			goalHeading = 270;
		}
	}
	else {
		goalHeading = 360*atan(dy/dx)/TWOPI;
		//printf("Goal initally at %d degrees\n", (int)goalHeading);
		if (dx < 0)
			goalHeading+=180;
		if (goalHeading < 0)
			goalHeading+=360;
	}
	int angleOffset = (((int)(goalHeading-currentLoc.heading+360))%360);
	if (angleOffset < 180)
	{
		a.dh = 16*angleOffset/180;
		if (a.dh < 1)
			a.dh = 1;
	}
	else {
		a.dh = -16*(360-angleOffset)/180;
		if (a.dh > -1)
			a.dh = -1;
	}

	if ((currentLoc.x-goalLoc.x)*(currentLoc.x-goalLoc.x)+(currentLoc.y-goalLoc.y)*(currentLoc.y-goalLoc.y) < 16)
	{
		a.dv = -currentLoc.v/5.0;
		if (targets.size() > 0)
		{
			targets.erase(targets.begin());
			if (targets.size() == 0)
				history.clear();
		}
		return true;
	}
	return true;
}

void SeekUnit::OpenGLDraw(const SteeringEnvironment *e,
						  const SimulationInfo<steeringState, steeringAction, SteeringEnvironment> *) const
{
	e->SetColor(0.0, 1.0, 0.0, 1.0);
	e->OpenGLDraw(currentLoc);
	//e->SetColor(1.0, 0.0, 0.0, 0.5);
	//e->OpenGLDraw(goalLoc);
	if (targets.size() > 0)
	{
		for (unsigned int x = 0; x < targets.size(); x++)
		{
			glColor4f(1.0, 0.0, 0.0, 0.5);
			DrawSphere(targets[x].x/worldRadius, targets[x].y/worldRadius, 0.0, 1/worldRadius);
		}
		glLineWidth(4);
		glBegin(GL_LINE_STRIP);
		glColor4f(1.0, 1.0, 1.0, 0.5);
		for (unsigned int x = 0; x < targets.size(); x++)
		{
			glVertex3f(targets[x].x/worldRadius, targets[x].y/worldRadius, 0);
		}
		glEnd();
		glLineWidth(1);

		for (unsigned int x = 0; x < history.size(); x++)
		{
			e->SetColor(0.0, 1.0, 0.0, 0.5);
			e->OpenGLDraw(history[x]);
		}
	}
	else {
		glColor4f(1.0, 0.0, 0.0, 0.5);
		DrawSphere(goalLoc.x/worldRadius, goalLoc.y/worldRadius, 0.0, 1/worldRadius);
	}
}
