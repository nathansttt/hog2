#include "WeightedMap2DEnvironment.h"

using namespace GraphAbstractionConstants;

/**
* Constructor for the WeightedMap2DEnvironment
*
* @param ma The map abstraction the environment uses
*/
WeightedMap2DEnvironment::WeightedMap2DEnvironment(MapAbstraction *ma)
:AbsMapEnvironment(ma)
{
	// initialize all edge counters to 0
	Graph* g = ma->GetAbstractGraph(0);
	edge_iterator ei = g->getEdgeIter();
	
	oi = new BaseMapOccupancyInterface(map);
	
	for(edge* e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		// Reset the counters
		e->SetLabelL(kForwardCount, 0);
		e->SetLabelL(kBackwardCount, 0);
	}
}

/**
* Destructor for the WeightedMap2DEnvironment
*/
WeightedMap2DEnvironment::~WeightedMap2DEnvironment()
{
	angleLookup.clear();
}

/**
* ApplyAction is called when an action is applied. 
* The current state, occupancy interface, and edge weight
* are updated if the action can be applied. 
* 
* @param s The current state
* @param dir The action taken from the current state
*/
void WeightedMap2DEnvironment::ApplyAction(xyLoc &s, tDirection dir)
{
	Vector2D angle;
	xyLoc old = s;
	switch (dir)
	{
		case kN: s.y-=1; angle.Set(0,-1); break;
		case kS: s.y+=1; angle.Set(0,1);break;
		case kE: s.x+=1; angle.Set(1,0); break;
		case kW: s.x-=1; angle.Set(-1,0);break;
		case kNW: s.y-=1; s.x-=1; angle.Set(-1,-1); break;
		case kSW: s.y+=1; s.x-=1; angle.Set(-1,1);break;
		case kNE: s.y-=1; s.x+=1; angle.Set(1,-1);  break;
		case kSE: s.y+=1; s.x+=1; angle.Set(1,1); break;
		default: break;
	}
	

	
	// May need to do different check as well oi->CanMove for example
	if (map->canStep(s.x, s.y, old.x, old.y) && !(oi->GetStateOccupied(s))) 
	{
// 		// Update the edge weight (mark the edge)
// 		node* from = ma->GetNodeFromMap(old.x, old.y);
// 		node* to = ma->GetNodeFromMap(s.x, s.y);
// 		edge* e = ma->GetAbstractGraph(0)->FindEdge(from->GetNum(),to->GetNum());
// 		
// 		if(e->getFrom() == from->GetNum())
// 			e->SetLabelL(kForwardCount, e->GetLabelL(kForwardCount) + 1);
// 		else
// 			e->SetLabelL(kBackwardCount, e->GetLabelL(kBackwardCount)+1);

		// Update the angle
		AngleUtil::AngleSearchNode sn(s,GetStateHash(s));
		if(angleLookup.find(sn) == angleLookup.end())
		{
			angleLookup[sn] = angle;
		}
		else
		{
			Vector2D old = angleLookup[sn];
			angleLookup[sn] = angle; // For now - just the new angle; no combining
		}
		
		oi->SetStateOccupied(s, false);
		oi->SetStateOccupied(old, true);
		
		return;
	}
	s = old;
}

/**
* 
* @param l1 The first location
* @param l2 The second location
* @return The weighted G-cost between l1 and l2, if there exists
* an edge between the two locations, DBL_MAX otherwise. 
*/
double WeightedMap2DEnvironment::GCost(xyLoc &l1, xyLoc &l2)
{
	//std::cout<<"l1 "<<l1<<" l2 "<<l2<<std::endl;
	double h = HCost(l1, l2);

	if (fgreater(h, ROOT_TWO))
		return DBL_MAX;
	
	tDirection dir = GetAction(l1,l2);
	
	Vector2D angle;
	switch (dir)
	{
		case kN: angle.Set(0,-1); break;
		case kS: angle.Set(0,1);break;
		case kE: angle.Set(1,0); break;
		case kW: angle.Set(-1,0);break;
		case kNW: angle.Set(-1,-1); break;
		case kSW: angle.Set(-1,1);break;
		case kNE: angle.Set(1,-1);  break;
		case kSE: angle.Set(1,1); break;
		default: break;
	}
	
	
	AngleUtil::AngleSearchNode sn(l2,GetStateHash(l2));
	
	if(angleLookup.find(sn) != angleLookup.end())
		{
			Vector2D old = angleLookup[sn];
			if(old == angle)
			{
				return h;
			}
			else
			{
				return 10 * h;
			}
		}
	
/*	if(l1 == l2)
		return 0;	
	// Get the edge between the two locations
	Graph* g = ma->GetAbstractGraph(0);
	node* from = ma->GetNodeFromMap(l1.x, l1.y);
	node* to = ma->GetNodeFromMap(l2.x, l2.y);
	edge* e = g->FindEdge(from->GetNum(),to->GetNum());
	
	double weight = 0; 
	
	// add weight -- 1 / (# units that have passed in the other direction)
	if(e->getFrom() == from->GetNum())
		{
			if(e->GetLabelL(kBackwardCount) > e->GetLabelL(kForwardCount))
			{
				weight = 1.0-(1.0 / (e->GetLabelL(kBackwardCount) - e->GetLabelL(kForwardCount)));
			}
			else
				weight = 0; 
			
		}
	else
		{
			if(e->GetLabelL(kBackwardCount) < e->GetLabelL(kForwardCount))
			{
				weight = 1.0 - (1.0 / (e->GetLabelL(kForwardCount) - e->GetLabelL(kBackwardCount)));
			}
			else
				weight = 0; 
		}	
	return h + 100*weight;*/
	return h; 
}

void WeightedMap2DEnvironment::OpenGLDraw(int window)
{	
	// Draw the map
	AbsMapEnvironment::OpenGLDraw(window);
	
	// Draw direction for all nodes
	
	Graph *g = ma->GetAbstractGraph(0);
	node_iterator ni = g->getNodeIter();
	
	xyLoc s;
	
	for(node *n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni))
	{
			
		s.x = n->GetLabelL(kFirstData);
		s.y = n->GetLabelL(kFirstData+1);
		
		AngleUtil::AngleSearchNode sn(s,GetStateHash(s));

		if(angleLookup.find(sn) != angleLookup.end())
		{
			Vector2D old = angleLookup[sn];
			
			// Reduce length to 40%
			GLdouble xx, yy, zz, rad;	
			glColor3f(1.0, 0,0);
			map->getOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
			glBegin(GL_LINE_STRIP);
			glVertex3f(xx, yy, zz-rad/2);	
			
			xx += old.x * 0.4 / map->getCoordinateScale();
			yy += old.y * 0.4 / map->getCoordinateScale();
			glVertex3f(xx, yy, zz-rad/2);	
			glEnd();
			
		}
		
	}
	
	
	// Draw all edges
//  	Graph* g = ma->GetAbstractGraph(0);
//  	
//  	edge_iterator ei = g->getEdgeIter();
//  	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
//  	{
//  		DrawEdge(window, e);
//  	}
// 	
}

void WeightedMap2DEnvironment::DrawEdge(int window, edge* e)
{
	Graph* g = ma->GetAbstractGraph(0);
	node* from = g->GetNode(e->getFrom());
	node* to = g->GetNode(e->getTo());
		
	GLdouble xx, yy, zz, rad;	
	if(e->GetLabelL(kBackwardCount) == e->GetLabelL(kForwardCount))
	{
		// whole edge should be same colour - white
		
		glColor3f(1.0, 1.0, 1.0);
		
		map->getOpenGLCoord(from->GetLabelL(kFirstData), from->GetLabelL(kFirstData+1), xx, yy, zz, rad);
		glBegin(GL_LINE_STRIP);
		glVertex3f(xx, yy, zz-rad/2);	
	
		map->getOpenGLCoord(to->GetLabelL(kFirstData), to->GetLabelL(kFirstData+1), xx, yy, zz, rad);
		glVertex3f(xx, yy, zz-rad/2);
		glEnd();
	}
	else if(e->GetLabelL(kForwardCount) > e->GetLabelL(kBackwardCount))
	{
		// white on "from" side, blue on "to" side
		glColor3f(1.0, 1.0, 1.0);
		
				map->getOpenGLCoord(from->GetLabelL(kFirstData), from->GetLabelL(kFirstData+1), xx, yy, zz, rad);
		glBegin(GL_LINE_STRIP);
		glVertex3f(xx, yy, zz-rad/2);	
	
		glColor3f(0,0,1.0);
		map->getOpenGLCoord(to->GetLabelL(kFirstData), to->GetLabelL(kFirstData+1), xx, yy, zz, rad);
		glVertex3f(xx, yy, zz-rad/2);
		glEnd();
	}
	else
	{
		// blue on "from" side, white on "to" side
		glColor3f(0,0,1.0);
				map->getOpenGLCoord(from->GetLabelL(kFirstData), from->GetLabelL(kFirstData+1), xx, yy, zz, rad);
		glBegin(GL_LINE_STRIP);
		glVertex3f(xx, yy, zz-rad/2);	
		
		
		glColor3f(1.0, 1.0, 1.0);
		map->getOpenGLCoord(to->GetLabelL(kFirstData), to->GetLabelL(kFirstData+1), xx, yy, zz, rad);
		glVertex3f(xx, yy, zz-rad/2);
		glEnd();
	}
}

void WeightedMap2DEnvironment::OpenGLDraw(int window, xyLoc& s, tDirection &dir)
{
	// Figure out which edge this corresponds to
	// Figure out which direction is preferred
	//   if none, then keep one colour
		
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
		
	Graph* g = ma->GetAbstractGraph(0);
	node* from = ma->GetNodeFromMap(initial.x, initial.y);
	node* to = ma->GetNodeFromMap(s.x, s.y);
	edge* e = g->FindEdge(from->GetNum(),to->GetNum());	
		
	DrawEdge(window, e);
	
	s = initial;
}



