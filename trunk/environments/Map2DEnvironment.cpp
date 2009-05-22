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
	h = 0;
}

MapEnvironment::~MapEnvironment()
{
//	delete map;
	delete oi;
}

GraphHeuristic *MapEnvironment::GetGraphHeuristic()
{
	return h;
}

void MapEnvironment::SetGraphHeuristic(GraphHeuristic *gh)
{
	h = gh;
}

void MapEnvironment::GetSuccessors(xyLoc &loc, std::vector<xyLoc> &neighbors) const
{
	bool up=false, down=false;
	// 
	if ((map->canStep(loc.x, loc.y, loc.x, loc.y+1)))
	{
		down = true;
		neighbors.push_back(xyLoc(loc.x, loc.y+1));
	}
	if ((map->canStep(loc.x, loc.y, loc.x, loc.y-1)))
	{
		up = true;
		neighbors.push_back(xyLoc(loc.x, loc.y-1));
	}
	if ((map->canStep(loc.x, loc.y, loc.x-1, loc.y)))
	{
		if ((up && (map->canStep(loc.x, loc.y, loc.x-1, loc.y-1))))
			neighbors.push_back(xyLoc(loc.x-1, loc.y-1));
		if ((down && (map->canStep(loc.x, loc.y, loc.x-1, loc.y+1))))
			neighbors.push_back(xyLoc(loc.x-1, loc.y+1));
		neighbors.push_back(xyLoc(loc.x-1, loc.y));
	}
	if ((map->canStep(loc.x, loc.y, loc.x+1, loc.y)))
	{
		if ((up && (map->canStep(loc.x, loc.y, loc.x+1, loc.y-1))))
			neighbors.push_back(xyLoc(loc.x+1, loc.y-1));
		if ((down && (map->canStep(loc.x, loc.y, loc.x+1, loc.y+1))))
			neighbors.push_back(xyLoc(loc.x+1, loc.y+1));
		neighbors.push_back(xyLoc(loc.x+1, loc.y));
	}
}

void MapEnvironment::GetActions(xyLoc &loc, std::vector<tDirection> &actions) const
{
	bool up=false, down=false;
	if ((map->canStep(loc.x, loc.y, loc.x, loc.y+1)))
	{
		down = true;
		actions.push_back(kS);
	}
	if ((map->canStep(loc.x, loc.y, loc.x, loc.y-1)))
	{
		up = true;
		actions.push_back(kN);
	}
	if ((map->canStep(loc.x, loc.y, loc.x-1, loc.y)))
	{
		if ((up && (map->canStep(loc.x, loc.y, loc.x-1, loc.y-1))))
			actions.push_back(kNW);
		if ((down && (map->canStep(loc.x, loc.y, loc.x-1, loc.y+1))))
			actions.push_back(kSW);
		actions.push_back(kW);
	}
	if ((map->canStep(loc.x, loc.y, loc.x+1, loc.y)))
	{
		if ((up && (map->canStep(loc.x, loc.y, loc.x+1, loc.y-1))))
			actions.push_back(kNE);
		if ((down && (map->canStep(loc.x, loc.y, loc.x+1, loc.y+1))))
			actions.push_back(kSE);
		actions.push_back(kE);
	}
}

tDirection MapEnvironment::GetAction(xyLoc &s1, xyLoc &s2) const
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

bool MapEnvironment::InvertAction(tDirection &a) const
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

void MapEnvironment::ApplyAction(xyLoc &s, tDirection dir) const
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
	double h1, h2;
	double a = ((l1.x>l2.x)?(l1.x-l2.x):(l2.x-l1.x));
	double b = ((l1.y>l2.y)?(l1.y-l2.y):(l2.y-l1.y));
	h1 = (a>b)?(b*ROOT_TWO+a-b):(a*ROOT_TWO+b-a);
	if (h == 0)
		return h1;
	
	graphState n1 = map->getNodeNum(l1.x, l1.y);
	graphState n2 = map->getNodeNum(l2.x, l2.y);
	if ((n1 != -1) && (n2 != -1))
		h2 = h->HCost(n1, n2);
	else
		h2 = 0;
	return std::max(h1, h2);
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

uint64_t MapEnvironment::GetStateHash(xyLoc &node) const
{
	return (((uint64_t)node.x)<<16)|node.y;
//	return (node.x<<16)|node.y;
}

uint64_t MapEnvironment::GetActionHash(tDirection act) const
{
	return (uint32_t) act;
}

void MapEnvironment::OpenGLDraw() const
{
	//std::cout<<"drawing\n";
	map->OpenGLDraw();
	// Draw occupancy interface - occupied = white
	for(int i=0; i<map->getMapWidth(); i++)
		for(int j=0; j<map->getMapHeight(); j++)
		{
			xyLoc l;
				l.x = i;
				l.y = j;
			if(oi->GetStateOccupied(l))
			{
				SetColor(1.0, 1.0, 1.0, 1.0);
				OpenGLDraw(l);//, 1.0, 1.0, 1.0);
			}
		}
}
	


void MapEnvironment::OpenGLDraw(const xyLoc &l) const
{
	GLdouble xx, yy, zz, rad;
	map->getOpenGLCoord(l.x, l.y, xx, yy, zz, rad);
	GLfloat r, g, b, t;
	GetColor(r, g, b, t);
	glColor4f(r, g, b, t);
	//glColor3f(0.5, 0.5, 0.5);
	DrawSphere(xx, yy, zz, rad);
}

void MapEnvironment::OpenGLDraw(const xyLoc &l1, const xyLoc &l2, double v) const
{
	GLdouble xx, yy, zz, rad;
	GLdouble xx2, yy2, zz2;
//	map->getOpenGLCoord((float)((1-v)*l1.x+v*l2.x),
//						(float)((1-v)*l1.y+v*l2.y), xx, yy, zz, rad);
	printf("%f between (%d, %d) and (%d, %d)\n", v, l1.x, l1.y, l2.x, l2.y);
	map->getOpenGLCoord(l1.x, l1.y, xx, yy, zz, rad);
	map->getOpenGLCoord(l2.x, l2.y, xx2, yy2, zz2, rad);
	//	map->getOpenGLCoord(perc*newState.x + (1-perc)*oldState.x, perc*newState.y + (1-perc)*oldState.y, xx, yy, zz, rad);
	xx = (1-v)*xx+v*xx2;
	yy = (1-v)*yy+v*yy2;
	zz = (1-v)*zz+v*zz2;
	GLfloat r, g, b, t;
	GetColor(r, g, b, t);
	glColor4f(r, g, b, t);
	DrawSphere(xx, yy, zz, rad);
}

//void MapEnvironment::OpenGLDraw(const xyLoc &l, GLfloat r, GLfloat g, GLfloat b) const
//{
//	GLdouble xx, yy, zz, rad;
//	map->getOpenGLCoord(l.x, l.y, xx, yy, zz, rad);
//	glColor3f(r,g,b);
//	DrawSphere(xx, yy, zz, rad);
//}


void MapEnvironment::OpenGLDraw(const xyLoc& initial, const tDirection &dir) const
{
	
	xyLoc s = initial;
	GLdouble xx, yy, zz, rad;
	map->getOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
	
	glColor3f(0.5, 0.5, 0.5);
	glBegin(GL_LINE_STRIP);
	glVertex3f(xx, yy, zz-rad/2);
		
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
	
}

//void MapEnvironment::OpenGLDraw(const xyLoc& initial, const tDirection &dir, GLfloat r, GLfloat g, GLfloat b) const
//{
//	xyLoc s = initial;
//	GLdouble xx, yy, zz, rad;
//	map->getOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
//	
//	glColor3f(r,g,b);
//	glBegin(GL_LINE_STRIP);
//	glVertex3f(xx, yy, zz-rad/2);
//	
//	
//	switch (dir)
//	{
//		case kN: s.y-=1; break;
//		case kS: s.y+=1; break;
//		case kE: s.x+=1; break;
//		case kW: s.x-=1; break;
//		case kNW: s.y-=1; s.x-=1; break;
//		case kSW: s.y+=1; s.x-=1; break;
//		case kNE: s.y-=1; s.x+=1; break;
//		case kSE: s.y+=1; s.x+=1; break;
//		default: break;
//	}
//
//	
//	map->getOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
//	glVertex3f(xx, yy, zz-rad/2);
//	glEnd();
//}

void MapEnvironment::GetNextState(xyLoc &currents, tDirection dir, xyLoc &news) const
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
	//delete ma;
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
	assert((s.x>=0) && (s.x<mapWidth) && (s.y>=0) && (s.y<mapHeight));
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
