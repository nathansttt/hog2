/*
 *  AlignUnit.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/12/11.
 *  Copyright 2011 University of Denver. All rights reserved.
 *
 */

#include "AlignUnit.h"

void AlignUnit::SetTarget(float targetx, float targety)
{
	float dx = goalLoc.x-currentLoc.x;
	float dy = goalLoc.y-currentLoc.y;
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

	goalLoc.x = targetx*worldRadius;
	goalLoc.y = targety*worldRadius;
	goalLoc.heading = goalHeading;
}

bool AlignUnit::MakeMove(SteeringEnvironment *,
						OccupancyInterface<steeringState, steeringAction> *,
						SimulationInfo<steeringState, steeringAction, SteeringEnvironment> *,
						steeringAction& a)
{
	a.dv = 1;
	float goalHeading = goalLoc.heading;
	int angleOffset = (((int)(goalHeading-currentLoc.heading+360))%360);
	if (angleOffset < 180)
	{
		a.dh = 16*angleOffset/180;
	}
	else {
		a.dh = -16*(360-angleOffset)/180;
	}
	return true;
}

void AlignUnit::OpenGLDraw(const SteeringEnvironment *e,
						  const SimulationInfo<steeringState, steeringAction, SteeringEnvironment> *) const
{
	e->SetColor(0.0, 1.0, 1.0, 1.0);
	e->OpenGLDraw(currentLoc);
	e->SetColor(1.0, 0.0, 0.0, 1.0);
	e->OpenGLDraw(goalLoc);
	//DrawSphere(0.0, 0.0, 0.0, 0.01);
}
