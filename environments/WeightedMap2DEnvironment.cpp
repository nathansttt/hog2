#include "WeightedMap2DEnvironment.h"


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
	

	
	// May need to do different check as well oi->CanMove for example
	if (map->canStep(s.x, s.y, old.x, old.y) && !(oi->GetStateOccupied(s))) 
	{
		// Update the edge weight (mark the edge)
		node* from = ma->GetNodeFromMap(old.x, old.y);
		node* to = ma->GetNodeFromMap(s.x, s.y);
		edge* e = ma->GetAbstractGraph(0)->FindEdge(from->GetNum(),to->GetNum());
		
		if(e->getFrom() == from->GetNum())
			e->SetLabelL(kForwardCount, e->GetLabelL(kForwardCount) + 1);
		else
			e->SetLabelL(kBackwardCount, e->GetLabelL(kBackwardCount)+1);
		
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
		
	if(l1 == l2)
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
	return h + weight;
}

void WeightedMap2DEnvironment::OpenGLDraw(int window)
{	
	// Draw the map
	AbsMapEnvironment::OpenGLDraw(window);
	
	// Draw all edges
 	Graph* g = ma->GetAbstractGraph(0);
 	
 	edge_iterator ei = g->getEdgeIter();
 	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
 	{
 		DrawEdge(window, e);
 	}
	
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



