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
#include <cstring>

MapEnvironment::MapEnvironment(Map *_m, bool useOccupancy)
{
	DIAGONAL_COST = ROOT_TWO;
	map = _m;
	if (useOccupancy)
		oi = new BaseMapOccupancyInterface(map);
	else
		oi = 0;
	h = 0;
	fourConnected = false;
}

MapEnvironment::MapEnvironment(MapEnvironment *me)
{
	map = me->map->Clone();
	h = 0;
	if (me->oi)
		oi = new BaseMapOccupancyInterface(map);
	else oi = 0;
	DIAGONAL_COST = me->DIAGONAL_COST;
	fourConnected = me->fourConnected;
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

void MapEnvironment::GetSuccessors(const xyLoc &loc, std::vector<xyLoc> &neighbors) const
{
	neighbors.resize(0);
	bool up=false, down=false;
	// 
	if ((map->CanStep(loc.x, loc.y, loc.x, loc.y+1)))
	{
		down = true;
		neighbors.push_back(xyLoc(loc.x, loc.y+1));
	}
	if ((map->CanStep(loc.x, loc.y, loc.x, loc.y-1)))
	{
		up = true;
		neighbors.push_back(xyLoc(loc.x, loc.y-1));
	}
	if ((map->CanStep(loc.x, loc.y, loc.x-1, loc.y)))
	{
		if (!fourConnected && (up && (map->CanStep(loc.x, loc.y, loc.x-1, loc.y-1))))
			neighbors.push_back(xyLoc(loc.x-1, loc.y-1));
		if (!fourConnected && (down && (map->CanStep(loc.x, loc.y, loc.x-1, loc.y+1))))
			neighbors.push_back(xyLoc(loc.x-1, loc.y+1));
		neighbors.push_back(xyLoc(loc.x-1, loc.y));
	}
	if ((map->CanStep(loc.x, loc.y, loc.x+1, loc.y)))
	{
		if (!fourConnected && (up && (map->CanStep(loc.x, loc.y, loc.x+1, loc.y-1))))
			neighbors.push_back(xyLoc(loc.x+1, loc.y-1));
		if (!fourConnected && (down && (map->CanStep(loc.x, loc.y, loc.x+1, loc.y+1))))
			neighbors.push_back(xyLoc(loc.x+1, loc.y+1));
		neighbors.push_back(xyLoc(loc.x+1, loc.y));
	}
}

bool MapEnvironment::GetNextSuccessor(const xyLoc &currOpenNode, const xyLoc &goal,
									  xyLoc &next, double &currHCost, uint64_t &special,
									  bool &validMove)
{
	if (fourConnected)
		return GetNext4Successor(currOpenNode, goal, next, currHCost, special, validMove);
	return GetNext8Successor(currOpenNode, goal, next, currHCost, special, validMove);
	
}

bool MapEnvironment::GetNext4Successor(const xyLoc &currOpenNode, const xyLoc &goal,
									   xyLoc &next, double &currHCost, uint64_t &special,
									  bool &validMove)
{
	validMove = false;
	if (special > 3)
		return false;
	// pass back next h-cost?
	// 4 connected:
	// case 2: above and right: Up, Right, Left, Down
	// case 3: directly right: Right, Down, Up, Left
	// case 4: below and right: Right, Down, Up, Left
	// case 5: directly below: Down, Left, Right, Up
	// case 6: below and left: Down, Left, Right, Up
	// case 7: directly left: Left, Up, Down, Right
	// case 8: above and left: Left, Up, Down, Right
	
	// 1,2. same y and different x (+/-)
	// 3,4. same x and different y (+/-)
	// 5,6,7,8. same x/y difference (+/-) combinations
	int theEntry = 0;
	const tDirection order[8][8] =
	{
		{kN, kE, kW, kS},
		{kS, kE, kW, kN},
		{kW, kN, kS, kE},
		{kE, kN, kS, kW},

		{kN, kW, kE, kS},
		{kW, kS, kN, kE},
		{kN, kE, kW, kS},
		{kS, kE, kW, kN},
	};
	const double hIncrease[8][8] = 
	{
		{1.0, 0.0, 0.0, 0.0},
		{1.0, 0.0, 0.0, 0.0},
		{1.0, 0.0, 0.0, 0.0},
		{1.0, 0.0, 0.0, 0.0},
		{0.0, 1.0, 0.0, 0.0},
		{0.0, 1.0, 0.0, 0.0},
		{0.0, 1.0, 0.0, 0.0},
		{0.0, 1.0, 0.0, 0.0},
	};
	
	if      (currOpenNode.x == goal.x && currOpenNode.y > goal.y)
	{ theEntry = 0; }
	else if (currOpenNode.x == goal.x && currOpenNode.y < goal.y)
	{ theEntry = 1; }
	else if (currOpenNode.x > goal.x && currOpenNode.y == goal.y)
	{ theEntry = 2; }
	else if (currOpenNode.x < goal.x && currOpenNode.y == goal.y)
	{ theEntry = 3; }
	else if (currOpenNode.x > goal.x && currOpenNode.y > goal.y)
	{ theEntry = 4; }
	else if (currOpenNode.x > goal.x && currOpenNode.y < goal.y)
	{ theEntry = 5; }
	else if (currOpenNode.x < goal.x && currOpenNode.y > goal.y)
	{ theEntry = 6; }
	else if (currOpenNode.x < goal.x && currOpenNode.y < goal.y)
	{ theEntry = 7; }

//	std::cout << special << " h from " << currHCost << " to "
//	<< currHCost + hIncrease[theEntry][special] << std::endl;
	switch (special) {
		case 0:
			next = currOpenNode;
			currHCost += hIncrease[theEntry][special];
			ApplyAction(next, order[theEntry][special]);
			special++;
			if (map->CanStep(currOpenNode.x, currOpenNode.y, next.x, next.y))
			{
				//std::cout << "Next successor of " << currOpenNode << " is " << next << std::endl;
				validMove = true;
				return true;
			}
		case 1:
			next = currOpenNode;
			currHCost += hIncrease[theEntry][special];
			ApplyAction(next, order[theEntry][special]);
			special++;
			if (map->CanStep(currOpenNode.x, currOpenNode.y, next.x, next.y))
			{
				//std::cout << "Next successor of " << currOpenNode << " is " << next << std::endl;
				validMove = true;
				return true;
			}
		case 2:
			next = currOpenNode;
			currHCost += 1;
			currHCost += hIncrease[theEntry][special];
			ApplyAction(next, order[theEntry][special]);
			special++;
			if (map->CanStep(currOpenNode.x, currOpenNode.y, next.x, next.y))
			{
				//std::cout << "Next successor of " << currOpenNode << " is " << next << std::endl;
				validMove = true;
				return true;
			}
		case 3:
			next = currOpenNode;
			currHCost += hIncrease[theEntry][special];
			ApplyAction(next, order[theEntry][special]);
			special++;
			if (map->CanStep(currOpenNode.x, currOpenNode.y, next.x, next.y))
			{
				//std::cout << "Next successor of " << currOpenNode << " is " << next << std::endl;
				validMove = true;
				return false;
			}
		default:
			return false;
	}

	return false;
}

bool MapEnvironment::GetNext8Successor(const xyLoc &currOpenNode, const xyLoc &goal,
									   xyLoc &next, double &currHCost, uint64_t &special,
									   bool &validMove)
{
	// in addition to the 16 obvious cases, when diagonal movess cross the 
	// diagonals we have separate cases. Thus, we don't implement this for now.
	
	// Diagonal movement when next to the goal could also be problematic, depending on the
	// map representation
	assert(false);
	
//	// it must be 1.5 for this code to be correct...
//	assert(DIAGONAL_COST == 1.5);
//	validMove = false;
//	if (special > 7) // moves
//		return false;
//	// pass back next h-cost?
//	// 4 connected:
//	// case 2: above and right: Up, Right, Left, Down
//	// case 3: directly right: Right, Down, Up, Left
//	// case 4: below and right: Right, Down, Up, Left
//	// case 5: directly below: Down, Left, Right, Up
//	// case 6: below and left: Down, Left, Right, Up
//	// case 7: directly left: Left, Up, Down, Right
//	// case 8: above and left: Left, Up, Down, Right
//	
//	// 1,2. same y and different x (+/-)
//	// 3,4. same x and different y (+/-)
//	// 5,6,7,8. same x/y difference (+/-) combinations
//	int theEntry = 0;
//	const tDirection order[8][8] =
//	{
//		// directly above
//		{kN, kNW, kNE, kE, kW, kS, kSE, kSW},
//		// more above to the right
//		{kN, kNE, kE, kNW, kW, kS, kSE, kSW},
//		// diagonally right
//		{kNE, kN, kE, kNW, kSE, kS, kW, kSW},
//		// more right and above
//		{kE, kNE, kN, kS, kSE, kW, kNW, kSW},
//
//		{kE, kN, kS, kW},
//		
//		{kN, kW, kE, kS},
//		{kW, kS, kN, kE},
//		{kN, kE, kW, kS},
//		{kS, kE, kW, kN},
//	};
//	const double hIncrease[8][8] = 
//	{
//		// directly above
//		{1.0, 0.0, 0.5, 0.0, 0.5, 1.0, 0.0, 0.0},
//		// more above to the right
//		{0.0, 0.5, 0.5, 0.5, 0.5, 0.0, 1.0, 0.0},
//		// diagonally right
//		{0.5, 0.0, 1.5, 0.0, 0.0, 0.0, 1.0, 0.0},
//		// more right and above
//		{0.0, 0.5, 0.5, 0.5, 0.5, 0.0, 1.0, 0.0},
//		{0.0, 1.0, 0.0, 0.0},
//		{0.0, 1.0, 0.0, 0.0},
//		{0.0, 1.0, 0.0, 0.0},
//		{0.0, 1.0, 0.0, 0.0},
//	};
//	
//	if      (currOpenNode.x == goal.x && currOpenNode.y > goal.y)
//	{ theEntry = 0; }
//	else if (currOpenNode.x == goal.x && currOpenNode.y < goal.y)
//	{ theEntry = 1; }
//	else if (currOpenNode.x > goal.x && currOpenNode.y == goal.y)
//	{ theEntry = 2; }
//	else if (currOpenNode.x < goal.x && currOpenNode.y == goal.y)
//	{ theEntry = 3; }
//	else if (currOpenNode.x > goal.x && currOpenNode.y > goal.y)
//	{ theEntry = 4; }
//	else if (currOpenNode.x > goal.x && currOpenNode.y < goal.y)
//	{ theEntry = 5; }
//	else if (currOpenNode.x < goal.x && currOpenNode.y > goal.y)
//	{ theEntry = 6; }
//	else if (currOpenNode.x < goal.x && currOpenNode.y < goal.y)
//	{ theEntry = 7; }
//	
//	//	std::cout << special << " h from " << currHCost << " to "
//	//	<< currHCost + hIncrease[theEntry][special] << std::endl;
//	switch (special) {
//		case 0:
//			next = currOpenNode;
//			currHCost += hIncrease[theEntry][special];
//			ApplyAction(next, order[theEntry][special]);
//			special++;
//			if (map->CanStep(currOpenNode.x, currOpenNode.y, next.x, next.y))
//			{
//				//std::cout << "Next successor of " << currOpenNode << " is " << next << std::endl;
//				validMove = true;
//				return true;
//			}
//		case 1:
//			next = currOpenNode;
//			currHCost += hIncrease[theEntry][special];
//			ApplyAction(next, order[theEntry][special]);
//			special++;
//			if (map->CanStep(currOpenNode.x, currOpenNode.y, next.x, next.y))
//			{
//				//std::cout << "Next successor of " << currOpenNode << " is " << next << std::endl;
//				validMove = true;
//				return true;
//			}
//		case 2:
//			next = currOpenNode;
//			currHCost += 1;
//			currHCost += hIncrease[theEntry][special];
//			ApplyAction(next, order[theEntry][special]);
//			special++;
//			if (map->CanStep(currOpenNode.x, currOpenNode.y, next.x, next.y))
//			{
//				//std::cout << "Next successor of " << currOpenNode << " is " << next << std::endl;
//				validMove = true;
//				return true;
//			}
//		case 3:
//			next = currOpenNode;
//			currHCost += hIncrease[theEntry][special];
//			ApplyAction(next, order[theEntry][special]);
//			special++;
//			if (map->CanStep(currOpenNode.x, currOpenNode.y, next.x, next.y))
//			{
//				//std::cout << "Next successor of " << currOpenNode << " is " << next << std::endl;
//				validMove = true;
//				return false;
//			}
//		default:
//			return false;
//	}
//	
//	return false;
}

void MapEnvironment::GetActions(const xyLoc &loc, std::vector<tDirection> &actions) const
{
	bool up=false, down=false;
	if ((map->CanStep(loc.x, loc.y, loc.x, loc.y+1)))
	{
		down = true;
		actions.push_back(kS);
	}
	if ((map->CanStep(loc.x, loc.y, loc.x, loc.y-1)))
	{
		up = true;
		actions.push_back(kN);
	}
	if ((map->CanStep(loc.x, loc.y, loc.x-1, loc.y)))
	{
		if (!fourConnected)
		{
			if ((up && (map->CanStep(loc.x, loc.y, loc.x-1, loc.y-1))))
				actions.push_back(kNW);
			if ((down && (map->CanStep(loc.x, loc.y, loc.x-1, loc.y+1))))
				actions.push_back(kSW);
		}
		actions.push_back(kW);
	}
	if ((map->CanStep(loc.x, loc.y, loc.x+1, loc.y)))
	{
		if (!fourConnected)
		{
			if ((up && (map->CanStep(loc.x, loc.y, loc.x+1, loc.y-1))))
				actions.push_back(kNE);
			if ((down && (map->CanStep(loc.x, loc.y, loc.x+1, loc.y+1))))
				actions.push_back(kSE);
		}
		actions.push_back(kE);
	}
}

tDirection MapEnvironment::GetAction(const xyLoc &s1, const xyLoc &s2) const
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
//	if (map->CanStep(s.x, s.y, old.x, old.y) &&
//		((!oi) || (oi && !(oi->GetStateOccupied(s)))))
//	{
//		return;
//	}
//	s = old;
}

double MapEnvironment::HCost(const xyLoc &l1, const xyLoc &l2)
{
	double h1, h2;
	if (fourConnected)
	{
		h1 = abs(l1.x-l2.x)+abs(l1.y-l2.y);
	}
	else {
		double a = ((l1.x>l2.x)?(l1.x-l2.x):(l2.x-l1.x));
		double b = ((l1.y>l2.y)?(l1.y-l2.y):(l2.y-l1.y));
		//return sqrt(a*a+b*b);
		h1 = (a>b)?(b*DIAGONAL_COST+a-b):(a*DIAGONAL_COST+b-a);
	}

	if (h == 0)
		return h1;
	
	int n1 = map->GetNodeNum(l1.x, l1.y);
	int n2 = map->GetNodeNum(l2.x, l2.y);
	if ((n1 != -1) && (n2 != -1))
	{
		graphState nn1 = n1;
		graphState nn2 = n2;
		h2 = h->HCost(nn1, nn2);
	}
	else
		h2 = 0;
	return std::max(h1, h2);
}

double MapEnvironment::GCost(const xyLoc &l, const tDirection &act)
{
	double multiplier = 1.0;
//	if (map->GetTerrainType(l.x, l.y) == kSwamp)
//	{
//		multiplier = 3.0;
//	}
	switch (act)
	{
		case kN: return 1.0*multiplier;
		case kS: return 1.0*multiplier;
		case kE: return 1.0*multiplier;
		case kW: return 1.0*multiplier;
		case kNW: return DIAGONAL_COST*multiplier;
		case kSW: return DIAGONAL_COST*multiplier;
		case kNE: return DIAGONAL_COST*multiplier;
		case kSE: return DIAGONAL_COST*multiplier;
		default: return 0;
	}
	return 0;
}

double MapEnvironment::GCost(const xyLoc &l1, const xyLoc &l2)
{
	double multiplier = 1.0;
//	if (map->GetTerrainType(l1.x, l1.y) == kSwamp)
//	{
//		multiplier = 3.0;
//	}
	if (l1.x == l2.x) return 1.0*multiplier;
	if (l1.y == l2.y) return 1.0*multiplier;
	if (l1 == l2) return 0.0;
	return DIAGONAL_COST*multiplier;
//	double h = HCost(l1, l2);
//	if (fgreater(h, DIAGONAL_COST))
//		return DBL_MAX;
//	return h;
}

bool MapEnvironment::GoalTest(const xyLoc &node, const xyLoc &goal)
{
	return ((node.x == goal.x) && (node.y == goal.y));
}

uint64_t MapEnvironment::GetStateHash(const xyLoc &node) const
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
//	for(int i=0; i<map->GetMapWidth(); i++)
//		for(int j=0; j<map->GetMapHeight(); j++)
//		{
//			xyLoc l;
//			l.x = i;
//			l.y = j;
//			if (oi && oi->GetStateOccupied(l))
//			{
//				SetColor(1.0, 1.0, 1.0, 1.0);
//				OpenGLDraw(l);//, 1.0, 1.0, 1.0);
//			}
//		}
}
	


void MapEnvironment::OpenGLDraw(const xyLoc &l) const
{
	GLdouble xx, yy, zz, rad;
	map->GetOpenGLCoord(l.x, l.y, xx, yy, zz, rad);
	GLfloat r, g, b, t;
	GetColor(r, g, b, t);
	glColor4f(r, g, b, t);
	//glColor3f(0.5, 0.5, 0.5);
	DrawSphere(xx, yy, zz, rad);
}

void MapEnvironment::OpenGLDraw(const xyLoc &l1, const xyLoc &l2, float v) const
{
	GLdouble xx, yy, zz, rad;
	GLdouble xx2, yy2, zz2;
//	map->GetOpenGLCoord((float)((1-v)*l1.x+v*l2.x),
//						(float)((1-v)*l1.y+v*l2.y), xx, yy, zz, rad);
//	printf("%f between (%d, %d) and (%d, %d)\n", v, l1.x, l1.y, l2.x, l2.y);
	map->GetOpenGLCoord(l1.x, l1.y, xx, yy, zz, rad);
	map->GetOpenGLCoord(l2.x, l2.y, xx2, yy2, zz2, rad);
	//	map->GetOpenGLCoord(perc*newState.x + (1-perc)*oldState.x, perc*newState.y + (1-perc)*oldState.y, xx, yy, zz, rad);
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
//	map->GetOpenGLCoord(l.x, l.y, xx, yy, zz, rad);
//	glColor3f(r,g,b);
//	DrawSphere(xx, yy, zz, rad);
//}


void MapEnvironment::OpenGLDraw(const xyLoc& initial, const tDirection &dir) const
{
	
	xyLoc s = initial;
	GLdouble xx, yy, zz, rad;
	map->GetOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
	
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

	
	map->GetOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
	glVertex3f(xx, yy, zz-rad/2);
	glEnd();
	
}

void MapEnvironment::GLDrawLine(const xyLoc &a, const xyLoc &b) const
{
	GLdouble xx1, yy1, zz1, rad;
	GLdouble xx2, yy2, zz2;
	map->GetOpenGLCoord(a.x, a.y, xx1, yy1, zz1, rad);
	map->GetOpenGLCoord(b.x, b.y, xx2, yy2, zz2, rad);
	
	double angle = atan2(yy1-yy2, xx1-xx2);
	double xoff = sin(2*PI-angle)*rad*0.1;
	double yoff = cos(2*PI-angle)*rad*0.1;

	
	
	GLfloat rr, gg, bb, t;
	GetColor(rr, gg, bb, t);
	glColor4f(rr, gg, bb, t);

//	glEnable(GL_BLEND);
//	glBlendFunc(GL_SRC_ALPHA_SATURATE, GL_ONE);
	//glEnable(GL_POLYGON_SMOOTH);
	glBegin(GL_TRIANGLE_STRIP);

	//glBegin(GL_QUADS);
	glVertex3f(xx1+xoff, yy1+yoff, zz1-rad/2);
	glVertex3f(xx2+xoff, yy2+yoff, zz2-rad/2);
	glVertex3f(xx1-xoff, yy1-yoff, zz1-rad/2);
	glVertex3f(xx2-xoff, yy2-yoff, zz2-rad/2);
	glEnd();
//	glDisable(GL_POLYGON_SMOOTH);
	//
//	glBegin(GL_LINES);
//	glVertex3f(xx, yy, zz-rad/2);
//	map->GetOpenGLCoord(b.x, b.y, xx, yy, zz, rad);
//	glVertex3f(xx, yy, zz-rad/2);
//	glEnd();
}

void MapEnvironment::GLLabelState(const xyLoc &s, const char *str) const
{
	glPushMatrix();

	GLdouble xx, yy, zz, rad;
	map->GetOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
	GLfloat r, g, b, t;
	GetColor(r, g, b, t);
	glColor4f(r, g, b, t);
	
	glTranslatef(xx-rad, yy+rad/2, zz-rad);
	glScalef(rad/(300.0), rad/300.0, 1);
	glRotatef(180, 0.0, 0.0, 1.0);
	glRotatef(180, 0.0, 1.0, 0.0);
	//glTranslatef((float)x/width-0.5, (float)y/height-0.5, 0);
	glDisable(GL_LIGHTING);
	for (int which = 0; which < strlen(str); which++)
		glutStrokeCharacter(GLUT_STROKE_ROMAN, str[which]);
	glEnable(GL_LIGHTING);
	//glTranslatef(-x/width+0.5, -y/height+0.5, 0);
	glPopMatrix();
}

//void MapEnvironment::OpenGLDraw(const xyLoc& initial, const tDirection &dir, GLfloat r, GLfloat g, GLfloat b) const
//{
//	xyLoc s = initial;
//	GLdouble xx, yy, zz, rad;
//	map->GetOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
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
//	map->GetOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
//	glVertex3f(xx, yy, zz-rad/2);
//	glEnd();
//}

void MapEnvironment::GetNextState(const xyLoc &currents, tDirection dir, xyLoc &news) const
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
 	mapWidth = m->GetMapWidth();
 	mapHeight = m->GetMapHeight();
	bitvec.resize(mapWidth*mapHeight);// = new BitVector(mapWidth * mapHeight);
	
	//initialize the bitvector
//	for(int i=0; i<m->GetMapWidth(); i++)
//		for(int j=0; j<m->GetMapHeight(); j++)
//			bitvec->Set(CalculateIndex(i,j), false);
}



/** Destructor for the BaseMapOccupancyInterface
* 
* @author Renee Jansen
* @date 08/22/2007
*/
BaseMapOccupancyInterface::~BaseMapOccupancyInterface()
{
//	delete bitvec;
//	bitvec = 0;
}

/** Sets the occupancy of a state.
* 
* @author Renee Jansen
* @date 08/22/2007
*
* @param s The state for which we want to set the occupancy
* @param occupied Whether or not the state is occupied
*/
void BaseMapOccupancyInterface::SetStateOccupied(const xyLoc &s, bool occupied)
{
	// Make sure the location is valid
	// unsigned, so must be greater than 0
	assert(/*(s.x>=0) &&*/ (s.x<mapWidth)/* && (s.y>=0)*/ && (s.y<mapHeight));
//	bitvec->Set(CalculateIndex(s.x,s.y), occupied);
	bitvec[CalculateIndex(s.x,s.y)] = occupied;
}

/** Returns the occupancy of a state.
* 
* @author Renee Jansen
* @date 08/22/2007
*
* @param s The state for which we want to know the occupancy information
* @return True if the state is occupied, false otherwise. 
*/
bool BaseMapOccupancyInterface::GetStateOccupied(const xyLoc &s)
{
	// unsigned, so must be greater than 0
	assert(/*s.x>=0 &&*/ s.x<=mapWidth && /*s.y>=0 && */s.y<=mapHeight);
	//return bitvec->Get(CalculateIndex(s.x,s.y));
	return bitvec[CalculateIndex(s.x,s.y)];
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
void BaseMapOccupancyInterface::MoveUnitOccupancy(const xyLoc &oldState, const xyLoc &newState)
{
	SetStateOccupied(oldState, false);
	SetStateOccupied(newState, true);
}

bool BaseMapOccupancyInterface::CanMove(const xyLoc &, const xyLoc &l2)
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
