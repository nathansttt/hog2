/*
 *  SteeringUnit.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/12/11.
 *  Copyright 2011 University of Denver. All rights reserved.
 *
 */

#include "SteeringUnit.h"

bool lengthGreater(float x, float y, float d)
{
	if (x*x+y*y>d*d)
		return true;
	return false;
}

void SteeringUnit::SetTarget(float targetx, float targety)
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

bool SteeringUnit::MakeMove(SteeringEnvironment *,
							OccupancyInterface<steeringState, steeringAction> *,
							SimulationInfo<steeringState, steeringAction, SteeringEnvironment> *si,
							steeringAction& a)
{
//	printf("Currently at: (%f, %f), heading %f, speed %f\n",
//		   currentLoc.x, currentLoc.y, currentLoc.heading, currentLoc.v);
	switch (mode)
	{
		case kAlignHeading: turnSpeed = 6.0; a = GetAlignMove(); return true;
		case kAlignSpeed: turnSpeed = 6.0; a = GetMatchSpeedMove(); return true;
		case kSeek: turnSpeed = 16.0; a = GetSeekMove(); return true;
		case kFlee: turnSpeed = 10.0; a = GetFleeMove(); return true;
		case kSeparation: turnSpeed = 10.0; a = GetSeparationMove(si); return true;
		case kChase: turnSpeed = 16.0; a = GetChaseMove(); return true;
		case kWander: turnSpeed = 10.0; a = GetWanderMove(); return true;
		case kFlock: turnSpeed = 16.0; a = GetFlockMove(si); return true;
	}
	return false;
}

steeringAction SteeringUnit::GetAlignMove()
{
	steeringAction a;
	a.dv = 1;
	float goalHeading = goalLoc.heading;
	int angleOffset = (((int)(goalHeading-currentLoc.heading+360))%360);
	if (angleOffset < 180)
	{
		a.dh = turnSpeed*angleOffset/180;
		if (a.dh < 1)
			a.dh = 1;
	}
	else {
		a.dh = -turnSpeed*(360-angleOffset)/180;
		if (a.dh > -1)
			a.dh = -1;
	}
	return a;
}

steeringAction SteeringUnit::GetSeekMove()
{
	steeringAction a;
	
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
		a.dh = turnSpeed*angleOffset/180; // 6
		if (a.dh < 1)
			a.dh = 1;
	}
	else {
		a.dh = -turnSpeed*(360-angleOffset)/180; // 6
		if (a.dh > -1)
			a.dh = -1;
	}
	//	printf("Goal at %d degrees\n", (int)goalHeading);
	if ((currentLoc.x-goalLoc.x)*(currentLoc.x-goalLoc.x)+(currentLoc.y-goalLoc.y)*(currentLoc.y-goalLoc.y) < 16)
	{
		//		a.dx = -currentLoc.dx/8;
		//		a.dy = -currentLoc.dy/8;
		a.dv = -currentLoc.v/5.0;
	}
	return a;
}

steeringAction SteeringUnit::GetMatchSpeedMove()
{
	steeringAction a;
	a.dh = 0;
	a.dv = goalLoc.v-currentLoc.v;

	return a;
}


steeringAction SteeringUnit::GetFleeMove()
{
	steeringAction a;

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
		a.dh = -turnSpeed*angleOffset/180;
	}
	else {
		a.dh = +turnSpeed*(360-angleOffset)/180;
	}
	return a;
}

steeringAction SteeringUnit::GetWanderMove()
{
	if (lengthGreater(currentLoc.x-goalLoc.x, currentLoc.y-goalLoc.y, 10))
	{
		steeringAction a = GetSeekMove();
		return a;
	}
	goalLoc.x = currentLoc.x+cos(TWOPI*(currentLoc.heading-40+random()%80)/360)*30;
	goalLoc.y = currentLoc.y+sin(TWOPI*(currentLoc.heading-40+random()%80)/360)*30;
	if (goalLoc.x > worldRadius)
		goalLoc.x -= worldRadius*2;
	if (goalLoc.x < -worldRadius)
		goalLoc.x += worldRadius*2;
	if (goalLoc.y > worldRadius)
		goalLoc.y -= worldRadius*2;
	if (goalLoc.y < -worldRadius)
		goalLoc.y += worldRadius*2;
	return GetSeekMove();
}

steeringAction SteeringUnit::GetChaseMove()
{
	if (target)
	{
		target->GetLocation(goalLoc);
		goalLoc.x = goalLoc.x+cos(TWOPI*(goalLoc.heading)/360)*10;
		goalLoc.y = goalLoc.y+sin(TWOPI*(goalLoc.heading)/360)*10;
		if (goalLoc.x > worldRadius)
			goalLoc.x -= worldRadius*2;
		if (goalLoc.x < -worldRadius)
			goalLoc.x += worldRadius*2;
		if (goalLoc.y > worldRadius)
			goalLoc.y -= worldRadius*2;
		if (goalLoc.y < -worldRadius)
			goalLoc.y += worldRadius*2;
	}
	return GetSeekMove();
}

steeringAction SteeringUnit::GetSeparationMove(SimulationInfo<steeringState, steeringAction, SteeringEnvironment> *si)
{
	steeringState s;
	unsigned int cnt = GetUnitGroup()->GetNumMembers();

	std::vector<steeringAction> avoids;

	for (unsigned int x = 0; x < cnt; x++)
	{
		if (GetUnitGroup()->GetMember(x) != this)
		{
			GetUnitGroup()->GetMember(x)->GetLocation(s);
			if (!lengthGreater(s.x-currentLoc.x, s.y-currentLoc.y, 20))
			{
				//printf("%p Fleeing from %p\n", this, GetUnitGroup()->GetMember(x));
				goalLoc = s;
				//avoids.push_back(GetFleeMove());
				avoiding = true;
				return GetFleeMove();
			}
		}
	}
	avoiding = false;
	//if (avoids.size() == 0)
	return GetWanderMove();
	
	//return avoids[random()%avoids.size()];
}

steeringAction SteeringUnit::GetFlockMove(SimulationInfo<steeringState, steeringAction, SteeringEnvironment> *si)
{
	std::vector<steeringAction> avoids;
	std::vector<steeringAction> aligns;
	steeringState average;
	steeringState tmp;
	average.x = 0;
	average.y = 0;
	
	unsigned int cnt = GetUnitGroup()->GetNumMembers();
	for (unsigned int x = 0; x < cnt; x++)
	{
		if (GetUnitGroup()->GetMember(x) != this)
		{
			GetUnitGroup()->GetMember(x)->GetLocation(tmp);
			goalLoc = tmp;
			aligns.push_back(GetAlignMove());
			average.x+=tmp.x;
			average.y+=tmp.y;
			if (!lengthGreater(tmp.x-currentLoc.x, tmp.y-currentLoc.y, 20))
			{
				//printf("%p Fleeing from %p\n", this, GetUnitGroup()->GetMember(x));
				avoids.push_back(GetFleeMove());
			}
		}
	}
	steeringAction final;
	steeringAction intermediate;
	if (avoids.size() > 0)
	{
		for (unsigned int x = 0; x < avoids.size(); x++)
		{
			intermediate.dh += avoids[x].dh;
			intermediate.dv += avoids[x].dv;
		}
		final.dh = intermediate.dh/(float)avoids.size();
		final.dv = intermediate.dv/(float)avoids.size();
		//printf("Average avoid action: heading: %f, vel: %f\n", intermediate.dh/(float)avoids.size(), intermediate.dv/(float)avoids.size());
	}
	intermediate.dh=0;
	intermediate.dv=0;
	if (aligns.size() > 0)
	{
		for (unsigned int x = 0; x < aligns.size(); x++)
		{
			intermediate.dh += aligns[x].dh;
			intermediate.dv += aligns[x].dv;
		}
		final.dh += 2*intermediate.dh/(float)aligns.size();
		final.dv += 2*intermediate.dv/(float)aligns.size();
		//printf("Average align action: heading: %f, vel: %f\n", intermediate.dh/(float)aligns.size(), intermediate.dv/(float)aligns.size());
	}

	if (cnt > 1)
	{
		average.x/=(float)(cnt-1);
		average.y/=(float)(cnt-1);
		goalLoc = average;
		intermediate = GetSeekMove();
		//printf("Seek action: heading: %f, vel: %f\n", intermediate.dh, intermediate.dv);

		final.dh += intermediate.dh;
		final.dv += intermediate.dv;
	}

	final.dh/=4.0;
	final.dv/=4.0;
	//printf("Final action: heading: %f, vel: %f\n", final.dh, final.dv);
	return final;
}

void SteeringUnit::OpenGLDraw(const SteeringEnvironment *e,
							  const SimulationInfo<steeringState, steeringAction, SteeringEnvironment> *) const
{
	if (avoiding)
		e->SetColor(1.0, 1.0, 0.0, 1.0);
	else if (mode == kChase)
		e->SetColor(0.0, 1.0, 0.0, 1.0);
	else
		e->SetColor(1.0, 1.0, 1.0, 1.0);
	e->OpenGLDraw(currentLoc);
	e->SetColor(1.0, 0.0, 0.0, 0.5);
	e->OpenGLDraw(goalLoc);
	//DrawSphere(0.0, 0.0, 0.0, 0.01);
}

