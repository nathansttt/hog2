/*
 *  RoboticArm.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/15/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "RoboticArm.h"

bool line2d::crosses(line2d which) const
{
	//input x1,y1 input x2,y2
	//input u1,v1 input u2,v2
	line2d here(start, end);
	double maxx1, maxx2, maxy1, maxy2;
	double minx1, minx2, miny1, miny2;
	if (here.start.x > here.end.x)
	{ maxx1 = here.start.x; minx1 = here.end.x; }
	else
	{ maxx1 = here.end.x; minx1 = here.start.x; }

	if (here.start.y > here.end.y)
	{ maxy1 = here.start.y; miny1 = here.end.y; }
	else
	{ maxy1 = here.end.y; miny1 = here.start.y; }

	if (which.start.x > which.end.x)
	{ maxx2 = which.start.x; minx2 = which.end.x; }
	else
	{ maxx2 = which.end.x; minx2 = which.start.x; }

	if (which.start.y > which.end.y)
	{ maxy2 = which.start.y; miny2 = which.end.y; }
	else
	{ maxy2 = which.end.y; miny2 = which.start.y; }
	
	if (fless(maxx1,minx2) || fless(maxx2, minx1) || fless(maxy1, miny2) || fless(maxy2, miny1))
		return false;
	
	if (fequal(maxx1, minx1)) // this is "here"
	{
		// already know that they share bounding boxes
		// here, they must cross
		if ((maxy2 < maxy1) && (miny2 > miny1))
			return true;
		
		// y = mx + b
		double m = (which.end.y-which.start.y)/(which.end.x-which.start.x);
		double b = which.start.y - m*which.start.x;
		double y = m*here.start.x+b;
		if (fless(y, maxy1) && fgreater(y, miny1)) // on the line
			return true;
		return false;
	}
	if (fequal(maxx2, minx2)) // this is "which"
	{
		// already know that they share bounding boxes
		// here, they must cross
		if ((maxy1 < maxy2) && (miny1 > miny2))
			return true;
		
		// y = mx + b
		double m = (here.end.y-here.start.y)/(here.end.x-here.start.x);
		double b = here.start.y - m*here.start.x;
		double y = m*which.start.x+b;
		if (fless(y, maxy2) && fgreater(y, miny2)) // on the line
			return true;
		return false;
	}
	
	double b1 = (which.end.y-which.start.y)/(which.end.x-which.start.x);// (A)
	double b2 = (here.end.y-here.start.y)/(here.end.x-here.start.x);// (B)
	
	double a1 = which.start.y-b1*which.start.x;
	double a2 = here.start.y-b2*here.start.x;

	if (fequal(b1, b2))
		return false;
	double xi = - (a1-a2)/(b1-b2); //(C)
	double yi = a1+b1*xi;
	// these are actual >= but we exempt points
	if ((fgreater((which.start.x-xi)*(xi-which.end.x), 0)) &&
		(fgreater((here.start.x-xi)*(xi-here.end.x), 0)) &&
		(!fless((which.start.y-yi)*(yi-which.end.y), 0)) &&
		(fgreater((here.start.y-yi)*(yi-here.end.y), 0)))
	{
		//printf("lines cross at (%f, %f)\n",xi,yi);
		return true;
	}
	else {
		return false;
		//print "lines do not cross";
	}
	assert(false);
}

tRotation armRotations::GetRotation(int which) const
{
	switch ((rotations>>(2*which))&0x3)
	{
		case 1: return kRotateCCW;
		case 0: return kNoRotation;
		case 2: return kRotateCW;
	}
	return kNoRotation;
}

void armRotations::SetRotation(int which, tRotation dir)
{
	int value = 0;
	switch (dir)
	{
		case kRotateCW: value = 2; break;
		case kRotateCCW: value = 1; break;
		case kNoRotation: value = 0; break;
	}
	rotations = rotations&((~0x3)<<(2*which))|(value<<2*which);
}

int armAngles::GetAngle(int which) const
{
	return (angles>>(10*which))&0x3FF;
}

void armAngles::SetAngle(int which, int value)
{
	value-=value&0x1;
	uint64_t mask = 0x3FFll<<(10*which);
	mask = ~mask;
	uint64_t val = (uint64_t)(value&0x3FF);
	val = val<<(10*which);
	angles = (angles&(mask));
	angles |= val;
}

int armAngles::GetNumArms() const
{
	return angles>>60;
}

void armAngles::SetNumArms(int count)
{
	uint64_t mask = 0xFFFFFFFFFFFFFFFll;
	uint64_t newCnt = count;
	newCnt = newCnt << 60;
	assert(count <= 6);
	angles = newCnt|(angles&mask);
}

void armAngles::SetGoal(double x, double y)
{
	uint64_t fixedDecx, fixedFracx;
	uint64_t fixedDecy, fixedFracy;
	x += 512; // avoid unsigned issues...
	y += 512;
	fixedDecx = ((uint32_t)x)&0x3FF; // 10 bits
	fixedFracx = (int)((x-(double)fixedDecx)*1024.0*1024.0); // 20 bits
	fixedDecy = ((uint32_t)y)&0x3FF; // 10 bits
	fixedFracy = (int)((y-(double)fixedDecy)*1024.0*1024.0); // 20 bits
	angles = (0xFll<<60)|(fixedDecx<<50)|(fixedFracx<<30)|(fixedDecy<<20)|fixedFracy;
}

void armAngles::GetGoal(double &x, double &y) const
{
	assert(IsGoalState());
	
	x = ((angles>>50)&0x3FF) + (double)((angles>>30)&0xFFFFF)/(1024.0*1024.0);
	y = ((angles>>20)&0x3FF) + (double)((angles)&0xFFFFF)/(1024.0*1024.0);
	x-= 512;
	y-= 512;
}


bool armAngles::IsGoalState() const
{
	return ((angles>>60) == 0xF);
}


RoboticArm::RoboticArm(int dof, double armlength, double fTolerance)
:DOF(dof), armLength(armlength), tolerance(fTolerance)
{
	m_TableComplete = false;
	BuildSinCosTables();
	GenerateCPDB();
}

RoboticArm::~RoboticArm()
{
}

void RoboticArm::GetSuccessors(armAngles &nodeID, std::vector<armAngles> &neighbors)
{
	neighbors.resize(0);
	for (int x = 0; x < nodeID.GetNumArms(); x++)
	{
		armAngles s = nodeID;
		armRotations a;
		a.SetRotation(x, kRotateCW);
		ApplyAction(s, a);

		if (LegalState(s))
			neighbors.push_back(s);
		
		s = nodeID;
		a.SetRotation(x, kRotateCCW);
		ApplyAction(s, a);

		if (LegalState(s))
			neighbors.push_back(s);
	}
}

void RoboticArm::GetActions(armAngles &nodeID, std::vector<armRotations> &actions)
{
	actions.resize(0);
	for (int x = 0; x < nodeID.GetNumArms(); x++)
	{
		armAngles s = nodeID;
		armRotations a;
		a.SetRotation(x, kRotateCW);
		ApplyAction(s, a);
		
		if (LegalState(s))
			actions.push_back(a);
		
		s = nodeID;
		a.SetRotation(x, kRotateCCW);
		ApplyAction(s, a);
		
		if (LegalState(s))
			actions.push_back(a);
	}

//	actions.resize(0);
//	for (int x = 0; x < nodeID.GetNumArms(); x++)
//	{
//		armAngles a(nodeID);
//		a.SetAngle(x, nodeID.GetAngle(x)+2);
//		if (LegalState(a))
//		{
//			armRotations rot;
//			rot.SetRotation(x, kRotateCW);
//			actions.push_back(rot);
//		}
//		a.SetAngle(x, nodeID.GetAngle(x)-2);
//		if (LegalState(a))
//		{
//			armRotations rot;
//			rot.SetRotation(x, kRotateCCW);
//			actions.push_back(rot);
//		}
//	}
}

armRotations RoboticArm::GetAction(armAngles &s1, armAngles &s2)
{
	armRotations ar;
	for (int x = 0; x < s1.GetNumArms(); x++)
	{
		ar.SetRotation(x, (tRotation)(s2.GetAngle(x)-s1.GetAngle(x)));
	}
	return ar;
}

void RoboticArm::ApplyAction(armAngles &s, armRotations dir)
{
	armAngles newState = s;
	for (int x = 0; x < newState.GetNumArms(); x++)
		newState.SetAngle(x, newState.GetAngle(x)+2*dir.GetRotation(x));
	//if (LegalState(newState))
	s = newState;
}

bool RoboticArm::InvertAction(armRotations &a)
{
	for (int x = 0; x < 6; x++)
		a.SetRotation(x, (tRotation)(-(int)a.GetRotation(x)));
	return true;
}

double RoboticArm::HCost(armAngles &node1, armAngles &node2)
{
//	if (node1.IsGoalState()) return HCost(node2, node1);
//	assert(node2.IsGoalState());
//
//	std::vector<line2d> armSegments1;
//	GenerateLineSegments(node1, armSegments1);
//	recVec a = armSegments1.back().end;
//	double x, y;
//	node2.GetGoal(x, y);
//	double actDistance = sqrt((x-a.x)*(x-a.x)+(y-a.y)*(y-a.y));
//	double movementAmount = (node1.GetNumArms()*armLength*sin(TWOPI*4.0/1024.0));
//	return actDistance / movementAmount;

	recVec a, b;
	if (node1.IsGoalState())
		node1.GetGoal(a.x, a.y);
	else {
		GenerateLineSegments(node1, armSegments);
		a = armSegments.back().end;
	}
	
	if (node2.IsGoalState())
		node2.GetGoal(b.x, b.y);
	else {
		GenerateLineSegments(node2, armSegments);
		b = armSegments.back().end;
	}
	return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
}

double RoboticArm::GCost(armAngles &node1, armAngles &node2)
{
	recVec a, b;
	if (node1.IsGoalState())
		node1.GetGoal(a.x, a.y);
	else {
		GenerateLineSegments(node1, armSegments);
		a = armSegments.back().end;
	}

	if (node2.IsGoalState())
		node2.GetGoal(b.x, b.y);
	else {
		GenerateLineSegments(node2, armSegments);
		b = armSegments.back().end;
	}
	return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
}

double RoboticArm::GCost(armAngles &node1, armRotations &act)
{
	armAngles node2;
	GetNextState(node1, act, node2);
	return GCost(node1, node2);
}

bool RoboticArm::GoalTest(armAngles &node, armAngles &goal)
{
	assert(goal.IsGoalState());
	GenerateLineSegments(node, armSegments);

	recVec a;
	a = armSegments.back().end;
	double x, y;
	goal.GetGoal(x, y);
	return (fabs(x-a.x) < tolerance && fabs(y-a.y) < tolerance);
}

uint64_t RoboticArm::GetStateHash(armAngles &node)
{
	return node.angles;
}

uint64_t RoboticArm::GetActionHash(armRotations act)
{
	return act.rotations;
}

void RoboticArm::OpenGLDraw(int)
{
	glBegin(GL_QUADS);
	glColor3f(0, 0, 0.1);
	glVertex3f(-1, -1, 0.1);
	glVertex3f(1, -1, 0.1);
	glVertex3f(1, 1, 0.1);
	glVertex3f(-1, 1, 0.1);
	glEnd();
	
	glColor3f(1, 0, 0);
	for (unsigned int x = 0; x < obstacles.size(); x++)
	{
		DrawLine(obstacles[x]);
	}
}

void RoboticArm::OpenGLDraw(int , armAngles &a)
{
	recVec e;
	if (a.IsGoalState())
	{
		e.z = 0;
		a.GetGoal(e.x, e.y);
	}
	else {
		GenerateLineSegments(a, armSegments);
		
		for (unsigned int x = 0; x < armSegments.size(); x++)
		{
			glColor3f(1, 1, 1);
			//printf("Drawing line segment %d of %d\n", x, armSegments.size());
			DrawLine(armSegments[x]);
		}
		e = armSegments.back().end;
	}
	
	glBegin(GL_LINE_LOOP);
	glColor3f(0, 1.0, 0);
	glVertex3f(e.x+tolerance, e.y+tolerance, 0);
	glVertex3f(e.x-tolerance, e.y+tolerance, 0);
	glVertex3f(e.x-tolerance, e.y-tolerance, 0);
	glVertex3f(e.x+tolerance, e.y-tolerance, 0);
	glEnd();
	
}

void RoboticArm::OpenGLDraw(int, armAngles &, armRotations &)
{
}

void RoboticArm::OpenGLDraw(int, armAngles &, armRotations &, GLfloat, GLfloat, GLfloat)
{
}

void RoboticArm::OpenGLDraw(int, armAngles &, GLfloat, GLfloat, GLfloat)
{
}

void RoboticArm::DrawLine(line2d l)
{
	glBegin(GL_LINES);
	glVertex3f(l.start.x, l.start.y, 0);
	glVertex3f(l.end.x, l.end.y, 0);
	glEnd();
}

void RoboticArm::GetNextState(armAngles &currents, armRotations dir, armAngles &news)
{
	news = currents;
	ApplyAction(news, dir);
}

bool RoboticArm::LegalState(armAngles &a)
{
	GenerateLineSegments(a, armSegments);
	for (unsigned int x = 0; x < armSegments.size(); x++)
	{
		for (unsigned int y = 0; y < obstacles.size(); y++)
		{
			if (armSegments[x].crosses(obstacles[y]))
				return false;
		}
		for (unsigned int y = x+2; y < armSegments.size(); y++)
			if (armSegments[x].crosses(armSegments[y]))
				return false;

		if ((x > 0) && (a.GetAngle(x) == 0))
			return false;
	}
	return true;
}

bool RoboticArm::LegalArmConfig(armAngles &a)
{
	if (m_TableComplete)
		return legals[a.GetAngle(1)][a.GetAngle(2)];
	GenerateLineSegments(a, armSegments);
	for (unsigned int x = 0; x < armSegments.size(); x++)
	{
		for (unsigned int y = x+2; y < armSegments.size(); y++)
			if (armSegments[x].crosses(armSegments[y]))
				return false;
		
		if ((x > 0) && (a.GetAngle(x) == 0))
			return false;
	}
	return true;
}

void RoboticArm::GenerateLineSegments(armAngles &a, std::vector<line2d> &armSegments1)
{
	armSegments1.resize(0);
	for (int x = 0; x < a.GetNumArms(); x++)
	{
		recVec prev;
		recVec start;
		recVec end;

		//double angle = TWOPI*a.GetAngle(x)/1024.0;
		int angle = a.GetAngle(x);
		if (x == 0)
		{
			start.x = 0;
			start.y = 0;
			start.z = 0;
			prev = start;
			prev.y = -armLength;
		}
		else {
			start = armSegments1.back().end;
			prev = armSegments1.back().start;
		}

		// offset to origin
		prev.x -= start.x;
		prev.y -= start.y;
		
		end.x = prev.x*GetCos(angle) - prev.y*GetSin(angle) + start.x;
		end.y = prev.x*GetSin(angle) + prev.y*GetCos(angle) + start.y;
		
		armSegments1.push_back(line2d(start, end));
	}
	assert(armSegments1.size() > 0);
}

double RoboticArm::GetSin(int angle)
{
	return sinTable[angle];
}

double RoboticArm::GetCos(int angle)
{
	return cosTable[angle];
}

void RoboticArm::BuildSinCosTables()
{
	sinTable.resize(1024);
	cosTable.resize(1024);
	for (int x = 0; x < 1024; x++)
	{
		sinTable[x] = sin(TWOPI*(double)x/1024.0);
		cosTable[x] = cos(TWOPI*(double)x/1024.0);
	}
}

void RoboticArm::GenerateCPDB()
{
	int numArms = 3;
	//void GenerateLegalArmConfigs();
	armAngles a;
	a.SetNumArms(numArms);
	a.SetAngle(0, 0);
	// 0, 512, 512
	std::vector<std::vector<bool> > legals;
	legals.resize(512);
	for (unsigned int x = 0; x < legals.size(); x++)
	{
		legals[x].resize(512);
		printf("Building table for %d\n", x);
		for (unsigned int y = 0; y < legals[x].size(); y++)
		{
			a.SetAngle(1, 2*x);
			a.SetAngle(2, 2*y);
			legals[x][y] = LegalArmConfig(a);
		}
	}
	m_TableComplete = true;
}
