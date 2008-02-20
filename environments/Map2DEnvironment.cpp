/*
 *  Map2DEnvironment.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/20/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */
#include "Map2DEnvironment.h"
#include "FPUtil.h"


MapEnvironment::MapEnvironment(Map *_m)
{
	map = _m;
	oi = new BaseMapOccupancyInterface(map);
}

MapEnvironment::~MapEnvironment()
{
	delete map;
}

void MapEnvironment::GetSuccessors(xyLoc &loc, std::vector<xyLoc> &neighbors)
{
	bool up=false, down=false;
	if ((map->getTerrainType(loc.x, loc.y+1) == kGround))
	{
		down = true;
		neighbors.push_back(xyLoc(loc.x, loc.y+1));
	}
	if ((map->getTerrainType(loc.x, loc.y-1) == kGround))
	{
		up = true;
		neighbors.push_back(xyLoc(loc.x, loc.y-1));
	}
	if ((map->getTerrainType(loc.x-1, loc.y) == kGround))
	{
		if ((up && (map->getTerrainType(loc.x-1, loc.y-1) == kGround)))
			neighbors.push_back(xyLoc(loc.x-1, loc.y-1));
		if ((down && (map->getTerrainType(loc.x-1, loc.y+1) == kGround)))
			neighbors.push_back(xyLoc(loc.x-1, loc.y+1));
		neighbors.push_back(xyLoc(loc.x-1, loc.y));
	}
	if ((map->getTerrainType(loc.x+1, loc.y) == kGround))
	{
		if ((up && (map->getTerrainType(loc.x+1, loc.y-1) == kGround)))
			neighbors.push_back(xyLoc(loc.x+1, loc.y-1));
		if ((down && (map->getTerrainType(loc.x+1, loc.y+1) == kGround)))
			neighbors.push_back(xyLoc(loc.x+1, loc.y+1));
		neighbors.push_back(xyLoc(loc.x+1, loc.y));
	}
}

void MapEnvironment::GetActions(xyLoc &loc, std::vector<tDirection> &actions)
{
	bool up=false, down=false;
	if ((map->getTerrainType(loc.x, loc.y+1) == kGround))
	{
		down = true;
		actions.push_back(kS);
	}
	if ((map->getTerrainType(loc.x, loc.y-1) == kGround))
	{
		up = true;
		actions.push_back(kN);
	}
	if ((map->getTerrainType(loc.x-1, loc.y) == kGround))
	{
		if ((up && (map->getTerrainType(loc.x-1, loc.y-1) == kGround)))
			actions.push_back(kNW);
		if ((down && (map->getTerrainType(loc.x-1, loc.y+1) == kGround)))
			actions.push_back(kSW);
		actions.push_back(kW);
	}
	if ((map->getTerrainType(loc.x+1, loc.y) == kGround))
	{
		if ((up && (map->getTerrainType(loc.x+1, loc.y-1) == kGround)))
			actions.push_back(kNE);
		if ((down && (map->getTerrainType(loc.x+1, loc.y+1) == kGround)))
			actions.push_back(kSE);
		actions.push_back(kE);
	}
}

tDirection MapEnvironment::GetAction(xyLoc &s1, xyLoc &s2)
{
	int result = kStay;
	switch (s1.x-s2.x)
	{
		case -1: result = kE; break;
		case 0: break;
		case 1: result = kW; break;
		default: return kTeleport;
	}
	
	// Tack the vertical move onto it
	// Notice the exploit of particular encoding of kStay, kE, etc. labels
	switch (s1.y-s2.y)
	{
		case -1: result = result|kS; break;
		case 0: break;
		case 1: result = result|kN; break;
		default: return kTeleport;
	}
	return (tDirection)result;
}

bool MapEnvironment::InvertAction(tDirection &a)
{
	switch (a)
	{
		case kN: a = kS; break;
		case kNE: a = kSW; break;
		case kE: a = kW; break;
		case kSE: a = kNW; break;
		case kS: a = kN; break;
		case kSW: a = kNE; break;
		case kW: a = kE; break;
		case kNW: a = kSE; break;
		default: break;
	}
	return true;
}

void MapEnvironment::ApplyAction(xyLoc &s, tDirection dir)
{
	xyLoc old = s;
	switch (dir)
	{
		case kN: s.y-=1; break;
		case kS: s.y+=1; break;
		case kE: s.x+=1; break;
		case kW: s.x-=1; break;
		case kNW: s.y-=1; s.x-=1; break;
		case kSW: s.y+=1; s.x-=1; break;
		case kNE: s.y-=1; s.x+=1; break;
		case kSE: s.y+=1; s.x+=1; break;
		default: break;
	}
	if (map->canStep(s.x, s.y, old.x, old.y) && !(oi->GetStateOccupied(s))) 
	{
		// UnitSim takes care of this
		// Make changes to the occupancy interface	
		//oi->SetStateOccupied(s, false);
		//oi->SetStateOccupied(old, true);
		return;
	}
	s = old;
}

double MapEnvironment::HCost(xyLoc &l1, xyLoc &l2)
{
	double a = ((l1.x>l2.x)?(l1.x-l2.x):(l2.x-l1.x));
	double b = ((l1.y>l2.y)?(l1.y-l2.y):(l2.y-l1.y));
	return (a>b)?(b*ROOT_TWO+a-b):(a*ROOT_TWO+b-a);
}

double MapEnvironment::GCost(xyLoc &, tDirection &act)
{
	switch (act)
	{
		case kN: return 1.0;
		case kS: return 1.0;
		case kE: return 1.0;
		case kW: return 1.0;
		case kNW: return ROOT_TWO;
		case kSW: return ROOT_TWO;
		case kNE: return ROOT_TWO;
		case kSE: return ROOT_TWO;
		default: return 0;
	}
	return 0;
}

double MapEnvironment::GCost(xyLoc &l1, xyLoc &l2)
{
	double h = HCost(l1, l2);
	if (fgreater(h, ROOT_TWO))
		return DBL_MAX;
	return h;
}

bool MapEnvironment::GoalTest(xyLoc &node, xyLoc &goal)
{
	return ((node.x == goal.x) && (node.y == goal.y));
}

uint64_t MapEnvironment::GetStateHash(xyLoc &node)
{
	return (node.x<<16)|node.y;
}

uint64_t MapEnvironment::GetActionHash(tDirection act)
{
	return (uint32_t) act;
}

void MapEnvironment::OpenGLDraw(int window)
{
	std::cout<<"drawing\n";
	map->OpenGLDraw(window);
	// Draw occupancy interface - occupied = white
	for(int i=0; i<map->getMapWidth(); i++)
		for(int j=0; j<map->getMapHeight(); j++)
		{
			xyLoc l;
				l.x = i;
				l.y = j;
			if(oi->GetStateOccupied(l))
			{
				OpenGLDraw(window, l, 1.0, 1.0, 1.0);
			}
		}
}
	


void MapEnvironment::OpenGLDraw(int , xyLoc &l)
{
	GLdouble xx, yy, zz, rad;
	map->getOpenGLCoord(l.x, l.y, xx, yy, zz, rad);
	glColor3f(0.5, 0.5, 0.5);
	DrawSphere(xx, yy, zz, rad);
}

void MapEnvironment::OpenGLDraw(int, xyLoc &l, GLfloat r, GLfloat g, GLfloat b)
{
	GLdouble xx, yy, zz, rad;
	map->getOpenGLCoord(l.x, l.y, xx, yy, zz, rad);
	glColor3f(r,g,b);
	DrawSphere(xx, yy, zz, rad);
}


void MapEnvironment::OpenGLDraw(int window, xyLoc& s, tDirection &dir)
{
	
	GLdouble xx, yy, zz, rad;
	map->getOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
	
	glColor3f(0.5, 0.5, 0.5);
	glBegin(GL_LINE_STRIP);
	glVertex3f(xx, yy, zz-rad/2);
	
	xyLoc initial = s;
	
	switch (dir)
	{
		case kN: s.y-=1; break;
		case kS: s.y+=1; break;
		case kE: s.x+=1; break;
		case kW: s.x-=1; break;
		case kNW: s.y-=1; s.x-=1; break;
		case kSW: s.y+=1; s.x-=1; break;
		case kNE: s.y-=1; s.x+=1; break;
		case kSE: s.y+=1; s.x+=1; break;
		default: break;
	}

	
	map->getOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
	glVertex3f(xx, yy, zz-rad/2);
	glEnd();
	
	s = initial;
}

void MapEnvironment::OpenGLDraw(int window, xyLoc& s, tDirection &dir, GLfloat r, GLfloat g, GLfloat b)
{
	GLdouble xx, yy, zz, rad;
	map->getOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
	
	glColor3f(r,g,b);
	glBegin(GL_LINE_STRIP);
	glVertex3f(xx, yy, zz-rad/2);
	
	xyLoc initial = s;
	
	switch (dir)
	{
		case kN: s.y-=1; break;
		case kS: s.y+=1; break;
		case kE: s.x+=1; break;
		case kW: s.x-=1; break;
		case kNW: s.y-=1; s.x-=1; break;
		case kSW: s.y+=1; s.x-=1; break;
		case kNE: s.y-=1; s.x+=1; break;
		case kSE: s.y+=1; s.x+=1; break;
		default: break;
	}

	
	map->getOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
	glVertex3f(xx, yy, zz-rad/2);
	glEnd();
	
	s = initial;
}

void MapEnvironment::GetNextState(xyLoc &currents, tDirection dir, xyLoc &news)
 {
	news = currents;
 	switch (dir)
	{
 		case kN: news.y-=1; break;
 		case kS: news.y+=1; break;
 		case kE: news.x+=1; break;
 		case kW: news.x-=1; break;
 		case kNW: news.y-=1; news.x-=1; break;
 		case kSW: news.y+=1; news.x-=1; break;
 		case kNE: news.y-=1; news.x+=1; break;
 		case kSE: news.y+=1; news.x+=1; break;
 		default: break;
	}	
}
/************************************************************/

AbsMapEnvironment::AbsMapEnvironment(MapAbstraction *_ma)
:MapEnvironment(_ma->GetMap())
{
	ma = _ma;
	
}

AbsMapEnvironment::~AbsMapEnvironment()
{
	map = 0;
	delete ma;
}

/************************************************************/

/** Constructor for the BaseMapOccupancyInterface
* 
* @author Renee Jansen
* @date 08/22/2007
*
* @param m The map to which the occupancy interface applies
*/
BaseMapOccupancyInterface::BaseMapOccupancyInterface(Map* m)
{
 	mapWidth = m->getMapWidth();
 	mapHeight = m->getMapHeight();
	bitvec = new bitVector(mapWidth * mapHeight);
	
	//initialize the bitvector
	for(int i=0; i<m->getMapWidth(); i++)
		for(int j=0; j<m->getMapHeight(); j++)
			bitvec->set(CalculateIndex(i,j), false);
}



/** Destructor for the BaseMapOccupancyInterface
* 
* @author Renee Jansen
* @date 08/22/2007
*/
BaseMapOccupancyInterface::~BaseMapOccupancyInterface()
{
	delete bitvec;
	bitvec = 0;
}

/** Sets the occupancy of a state.
* 
* @author Renee Jansen
* @date 08/22/2007
*
* @param s The state for which we want to set the occupancy
* @param occupied Whether or not the state is occupied
*/
void BaseMapOccupancyInterface::SetStateOccupied(xyLoc &s, bool occupied)
{
	// Make sure the location is valid
	assert(s.x>=0 && s.x<=mapWidth && s.y>=0 && s.y<=mapHeight);
	bitvec->set(CalculateIndex(s.x,s.y), occupied);
}

/** Returns the occupancy of a state.
* 
* @author Renee Jansen
* @date 08/22/2007
*
* @param s The state for which we want to know the occupancy information
* @return True if the state is occupied, false otherwise. 
*/
bool BaseMapOccupancyInterface::GetStateOccupied(xyLoc &s)
{
	assert(s.x>=0 && s.x<=mapWidth && s.y>=0 && s.y<=mapHeight);
	return bitvec->get(CalculateIndex(s.x,s.y));
}

/** Gets the index into the bitvector. 
*
* Converts (x,y) locations to a position in the bitvector. 
*
* @author Renee Jansen
* @date 08/22/2007
*
* @param x The x-coordinate of the location
* @param y The y-coordinate of the location
* @return The index into the bit vector
*/
//template <class state, class action>
long BaseMapOccupancyInterface::CalculateIndex(uint16_t x, uint16_t y)
{
	return (y * mapWidth) + x;
}

/** Updates the occupancy interface when a unit moves
*
* Sets the old location to be not occupied, and the new location
* to be occupied.
* 
* @author Renee Jansen
* @date 09/17/2007
*
* @param oldState The unit's previous state
* @param newState The unit's new state
*/
void BaseMapOccupancyInterface::MoveUnitOccupancy(xyLoc &oldState, xyLoc &newState)
{
	SetStateOccupied(oldState, false);
	SetStateOccupied(newState, true);
}

bool BaseMapOccupancyInterface::CanMove(xyLoc &l1, xyLoc &l2)
{
	if(!(GetStateOccupied(l2)))
	{
		return true;
	}
	else
	{		
		return false;
	}
	
}