/*
 *  FleeUnit.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/12/11.
 *  Copyright 2011 University of Denver. All rights reserved.
 *
 */

#include "FleeUnit.h"


bool FleeUnit::MakeMove(SteeringEnvironment *,
						OccupancyInterface<steeringState, steeringAction> *,
						SimulationInfo<steeringState, steeringAction, SteeringEnvironment> *,
						steeringAction& a)
{
	//printf("At (%f, %f: %f)\n", currentLoc.x, currentLoc.y, currentLoc.heading);
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
		a.dh = -10*angleOffset/180;
	}
	else {
		a.dh = 10*(360-angleOffset)/180;
	}
	return true;
}

void FleeUnit::OpenGLDraw(const SteeringEnvironment *e,
						  const SimulationInfo<steeringState, steeringAction, SteeringEnvironment> *) const
{
	e->SetColor(1.0, 1.0, 0.0, 1.0);
	e->OpenGLDraw(currentLoc);
	//e->SetColor(1.0, 0.0, 0.0, 0.5);
	//e->OpenGLDraw(goalLoc);
	glColor4f(1.0, 0.0, 0.0, 0.5);
	DrawSphere(goalLoc.x/worldRadius, goalLoc.y/worldRadius, 0.0, 1/worldRadius);
}
