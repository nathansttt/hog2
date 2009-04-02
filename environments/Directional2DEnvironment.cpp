/*
 *  Directional2DEnvironment.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 2/20/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "Directional2DEnvironment.h"
#include "FPUtil.h"


Directional2DEnvironment::Directional2DEnvironment(Map *_m)
{
	BuildHTable();
	SetColor(1.0, 0.25, 0.25, 1.0);
	map = _m;
}

Directional2DEnvironment::~Directional2DEnvironment()
{
	delete map;
}

void Directional2DEnvironment::GetSuccessors(xySpeedHeading &loc, std::vector<xySpeedHeading> &neighbors) const
{
	std::vector<deltaSpeedHeading> acts;
	GetActions(loc, acts);
	for (unsigned int x = 0; x < acts.size(); x++)
	{
		xySpeedHeading newLoc(loc);
		ApplyAction(newLoc, acts[x]);
		neighbors.push_back(newLoc);
	}
}

void Directional2DEnvironment::GetActions(xySpeedHeading &loc, std::vector<deltaSpeedHeading> &actions) const
{
	// oops, not checking map yet...
	deltaSpeedHeading sh;
	if (loc.speed < 2)
	{
		sh.speed = 0-loc.speed;
//		sh.turn = -3;
//		actions.push_back(sh);
//		sh.turn = -2;
//		actions.push_back(sh);

//		sh.turn = -1;
//		actions.push_back(sh);
//		sh.turn = 1;
//		actions.push_back(sh);
		sh.turn = 0;
		actions.push_back(sh);

//		sh.turn = 2;
//		actions.push_back(sh);
//		sh.turn = 3;
//		actions.push_back(sh);
	}
	if (loc.speed > 0) // fast moves
	{
		sh.speed = 2-loc.speed;
		sh.turn = -1;
		actions.push_back(sh);
		sh.turn = 0;
		actions.push_back(sh);
		sh.turn = 1;
		actions.push_back(sh);
	}
	if (loc.speed > 1) // fastest moves
	{
		sh.speed = 3-loc.speed;
		sh.turn = 0;
		actions.push_back(sh);
	}
	// medium moves
	if (loc.speed < 3)
	{
		sh.speed = 1-loc.speed;
		sh.turn = -3;
		actions.push_back(sh);
		sh.turn = -2;
		actions.push_back(sh);
		sh.turn = -1;
		actions.push_back(sh);
		sh.turn = 0;
		actions.push_back(sh);
		sh.turn = 1;
		actions.push_back(sh);
		sh.turn = 2;
		actions.push_back(sh);
		sh.turn = 3;
		actions.push_back(sh);
	}
}

deltaSpeedHeading Directional2DEnvironment::GetAction(xySpeedHeading &one, xySpeedHeading &two) const
{
	std::vector<deltaSpeedHeading> acts;
	GetActions(one, acts);
	for (unsigned int x = 0; x < acts.size(); x++)
	{
		ApplyAction(one, acts[x]);
		if (two == one)
		{
			UndoAction(one, acts[x]);
			return acts[x];
		}
		UndoAction(one, acts[x]);
	}
	assert(false);
	return acts[0];
}

bool Directional2DEnvironment::InvertAction(deltaSpeedHeading &) const
{
	assert(false);
	return false;
}

void Directional2DEnvironment::ApplyAction(xySpeedHeading &s, deltaSpeedHeading dir) const
{
	s.speed += dir.speed;
	if (s.speed > 0)
	{
		s.rotation = (16+s.rotation+dir.turn)%16;
		double tmp = cos(TWOPI*s.rotation/16.0)*s.speed;
		s.x += tmp;
		tmp = sin(TWOPI*s.rotation/16.0)*s.speed;
		s.y += tmp;
	}
}

void Directional2DEnvironment::UndoAction(xySpeedHeading &s, deltaSpeedHeading dir) const
{
	if (s.speed > 0)
	{
		s.x -= cos(TWOPI*s.rotation/16.0)*s.speed;
		s.y -= sin(TWOPI*s.rotation/16.0)*s.speed;
		s.rotation = (16+s.rotation-dir.turn)%16;
	}
	s.speed -= dir.speed;
}

double Directional2DEnvironment::HCost(xySpeedHeading &l1, xySpeedHeading &l2)
{
	float dist = sqrt((l1.x-l2.x)*(l1.x-l2.x)+(l1.y-l2.y)*(l1.y-l2.y));
	dist = ceil(dist/3.0);
	return max(dist, LookupStateHeuristic(l1, l2));
//	double val = ceil(dist/3.0)+(3-l1.speed)+(3-l2.speed);
//	int angle = (16+l1.rotation-l2.rotation)%16;
//	return val+angle;
}

double Directional2DEnvironment::GCost(xySpeedHeading &, deltaSpeedHeading &)
{
	return 1.0;
}

double Directional2DEnvironment::GCost(xySpeedHeading &, xySpeedHeading &)
{
	return 1.0;
//	double h = HCost(l1, l2);
//	if (fgreater(h, ROOT_TWO))
//		return DBL_MAX;
//	return h;
}

bool Directional2DEnvironment::GoalTest(xySpeedHeading &node, xySpeedHeading &goal)
{
	return ((((int)node.x == (int)goal.x) && ((int)node.y == (int)goal.y)) &&
			(node.speed == 0) && (node.rotation == goal.rotation));
}

uint64_t Directional2DEnvironment::GetStateHash(xySpeedHeading &node) const
{
	// rotation is 0..15
	// speed is 0..3
	//	float x;	float y;	uint8_t speed;	uint8_t rotation;
	uint64_t hval = (((int)node.x*4)<<16)|((int)node.y*4);
	hval = (hval<<32)|(node.rotation<<8)|node.speed;
	return hval;
}

uint64_t Directional2DEnvironment::GetActionHash(deltaSpeedHeading act) const
{
	return (act.turn<<8)+act.speed;
}

void Directional2DEnvironment::OpenGLDraw() const
{
//	std::cout<<"drawing\n";
	map->OpenGLDraw();
}


void Directional2DEnvironment::OpenGLDraw(const xySpeedHeading& oldState, const xySpeedHeading &newState, float perc) const
{
	GLfloat r, g, b, t;
	GetColor(r, g, b, t);
	//printf("Drawing %f percent\n", perc);
//	std::cout << oldState << std::endl;
//	std::cout << newState << std::endl;

	GLdouble xx, yy, zz, rad;
	
	map->getOpenGLCoord(perc*newState.x + (1-perc)*oldState.x, perc*newState.y + (1-perc)*oldState.y, xx, yy, zz, rad);
	
	float rot = (1-perc)*oldState.rotation+perc*newState.rotation;
	if ((oldState.rotation >= 12) && (newState.rotation <= 5))
	{
		rot = (1-perc)*oldState.rotation+perc*(newState.rotation+16);
		if (rot >= 16)
			rot -= 16;
	}
	else if ((newState.rotation >= 12) && (oldState.rotation <= 5))
	{
		rot = (1-perc)*(oldState.rotation+16)+perc*(newState.rotation);
		if (rot >= 16)
			rot -= 16;
	}
	GLdouble yoffset = sin(TWOPI*rot/16)*rad;
	GLdouble xoffset = cos(TWOPI*rot/16)*rad;
	
	glBegin(GL_TRIANGLES);
	glColor4f(r, g, b/2, t);
	glVertex3f(xx+xoffset, yy+yoffset, zz);
	glColor4f(r, g/2, b, t);
	glVertex3f(xx-xoffset, yy-yoffset, zz-rad);
	glColor4f(r, g, b/2, t);
	glVertex3f(xx-xoffset+0.5*yoffset, yy-yoffset-0.5*xoffset, zz);
	
	glColor4f(r, g/2, b, t);
	glVertex3f(xx+xoffset, yy+yoffset, zz);
	glColor4f(r, g, b/2, t);
	glVertex3f(xx-xoffset, yy-yoffset, zz-rad);
	glColor4f(r, g/2, b, t);
	glVertex3f(xx-xoffset-0.5*yoffset, yy-yoffset+0.5*xoffset, zz);
	glEnd();
}


void Directional2DEnvironment::OpenGLDraw(const xySpeedHeading &l) const
{
	GLdouble xx, yy, zz, rad;
	GLfloat r, g, b, t;
	GetColor(r, g, b, t);
	map->getOpenGLCoord(l.x, l.y, xx, yy, zz, rad);

	GLdouble yoffset = sin(TWOPI*l.rotation/16)*rad;
	GLdouble xoffset = cos(TWOPI*l.rotation/16)*rad;

	glBegin(GL_TRIANGLES);
	recVec surfaceNormal;
	surfaceNormal.x = (((-0.5*xoffset) * (-rad)) - ((+rad) - (-2*yoffset)));
	surfaceNormal.y = (((rad) * (-2*xoffset)) - ((0.5*yoffset) - (rad)));
	surfaceNormal.z = (((0.5*yoffset) * (-2*yoffset)) - ((-0.5*xoffset) - (-2*xoffset)));
	surfaceNormal.normalise();
	glNormal3f(surfaceNormal.x, surfaceNormal.y, surfaceNormal.z);
	glColor4f(r, g, b/2, t);
	glVertex3f(xx+xoffset, yy+yoffset, zz);
	glColor4f(r, g/2, b, t);
	glVertex3f(xx-xoffset, yy-yoffset, zz-rad);
	glColor4f(r, g, b/2, t);
	glVertex3f(xx-xoffset+0.5*yoffset, yy-yoffset-0.5*xoffset, zz);
	
	surfaceNormal.x = (((+0.5*xoffset) * (-rad)) - ((+rad) - (-2*yoffset)));
	surfaceNormal.y = (((rad) * (-2*xoffset)) - ((-0.5*yoffset) - (rad)));
	surfaceNormal.z = (((-0.5*yoffset) * (-2*yoffset)) - ((+0.5*xoffset) - (-2*xoffset)));
	surfaceNormal.normalise();
	glNormal3f(surfaceNormal.x, surfaceNormal.y, surfaceNormal.z);
	glColor4f(r, g/2, b, t);
	glVertex3f(xx+xoffset, yy+yoffset, zz);
	glColor4f(r, g, b/2, t);
	glVertex3f(xx-xoffset, yy-yoffset, zz-rad);
	glColor4f(r, g/2, b, t);
	glVertex3f(xx-xoffset-0.5*yoffset, yy-yoffset+0.5*xoffset, zz);
	glEnd();	
}


void Directional2DEnvironment::OpenGLDraw(const xySpeedHeading& l, const deltaSpeedHeading &) const
{
	GLdouble xx, yy, zz, rad;
	map->getOpenGLCoord(l.x, l.y, xx, yy, zz, rad);
	glColor3f(0.5, 0.5, 0.5);
	DrawSphere(xx-rad+l.x, yy-rad+l.y, zz, rad);
}


void Directional2DEnvironment::GetNextState(xySpeedHeading &currents, deltaSpeedHeading dir, xySpeedHeading &news) const
{
}

void Directional2DEnvironment::BuildHTable()
{
	// rotation is 0..15
	// speed is 0..3

	// 9x9
	//std::vector<std::vector<int> > hTable;
	hTable.resize(81);
	for (unsigned int x = 0; x < hTable.size(); x++)
	{
		hTable[x].resize(64);
		for (unsigned int y = 0; y < hTable[x].size(); y++)
		{
			hTable[x][y] = -1;
		}
	}
	// goal is in the middle
	hTable[(hTable.size()-1)/2][0] = 0;
		   
	for (int t = 0; t <= 15; t++)
	{
		for (unsigned int x = 0; x < hTable.size(); x++)
		{
			for (unsigned int y = 0; y < hTable[x].size(); y++)
			{
				xySpeedHeading s;
				s.x = (float)(x%9)-3.5f;
				s.y = (float)(x/9)-3.5f;
				s.rotation = y&0xF;
				s.speed = (y>>4)&0x3;
				
				//std::cout << "Getting successors of " << s << ":\n";

				std::vector<xySpeedHeading> succ;
				GetSuccessors(s, succ);
				for (unsigned int z = 0; z < succ.size(); z++)
				{
					int val = LookupStateHash(succ[z]);
					//std::cout << succ[z] << " has val " << val << "\n";
					if (val != -1)
					{
//						printf("Assigning %d to (%d, %d) [%d:%d]\n", val+1,
//							   x%9-4, x/9-4, s.rotation, s.speed);
						if (hTable[x][y] == -1)
							hTable[x][y] = val+1;
						else
							hTable[x][y] = std::min(hTable[x][y], val+1);
					}
				}
//				if ((t == 15) && (hTable[x][y] == -1))
//					hTable[x][y] = 16;
			}
		}		
	}

	for (unsigned int x = 0; x < hTable.size(); x++)
	{
		for (unsigned int y = 0; y < hTable[x].size(); y++)
		{
			xySpeedHeading s;
			s.x = (float)(x%9)-3.5f;
			s.y = (float)(x/9)-3.5f;
			s.rotation = y&0xF;
			s.speed = (y>>4)&0x3;
			std::cout << s << " has value of: " << hTable[x][y] << std::endl;
		}
		std::cout << "---------------" << std::endl;
	}
}

int Directional2DEnvironment::LookupStateHash(xySpeedHeading &s)
{
	int x = s.x;
	int y = s.y;
	
	if ((x < -4) || (x > 4) || (y < -4) || (y > 4))
		return -1;
	int index1 = (x+4)+(y+4)*9;
	int index2 = (s.rotation&0xF)|((s.speed&0x3)<<4);
	return hTable[index1][index2];
}

int Directional2DEnvironment::LookupStateHeuristic(xySpeedHeading &s1, xySpeedHeading &s2)
{
	//return 0;
	if ((s2.speed != 0) || (s2.rotation != 0))
		return 0;
	xySpeedHeading s3;
	s3 = s1;
	s3.x = s2.x - s3.x;
	s3.y = s2.y - s3.y;
//	std::cout << "Heuristic looking up: " << s3 << std::endl;
	int val = LookupStateHash(s3);
	if (val != -1)
	{
//		printf("Looked up %d\n", val);
		return val;
	}
	return 0;
}
