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
	//Graph* g = ma->GetAbstractGraph(0);
	//edge_iterator ei = g->getEdgeIter();
	
	oi = new BaseMapOccupancyInterface(map);
	diffWeight = 1;
	oldProportion = 0.7;
	
	useWindow = false;
	noWeighting = false;
	
	queryOldProportion = 0.9;
	updateOnQuery = false;
	
	updateSurrounding = false;
	surroundingProportion = 0.9;
	//for(edge* e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	//{
	//	// Reset the counters
	//	e->SetLabelL(kForwardCount, 0);
	//	e->SetLabelL(kBackwardCount, 0);
	//}
	
	usePerceptron = false;
	learningRate = 0.1;
}

WeightedMap2DEnvironment::WeightedMap2DEnvironment(AbsMapEnvironment *ame)
:AbsMapEnvironment(ame->GetMapAbstraction()->clone(ame->GetMapAbstraction()->GetMap()->clone()))
{
	oi = ame->GetOccupancyInfo();
	diffWeight = 1;
	oldProportion = 0.7;
	useWindow = false;
	noWeighting = false;
		
	queryOldProportion = 0.9;
	updateOnQuery = false;
	
	updateSurrounding = false;
	surroundingProportion = 0.9;
	
	usePerceptron = false;
	learningRate = 0.1;
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
	// this isn't called 
	
	std::cout<<"Weighted update action\n";
	//Vector2D angle;
	xyLoc old = s;
	switch (dir)
	{
		case kN: s.y-=1; /*angle.Set(0,-1);*/ break;
		case kS: s.y+=1; /*angle.Set(0,1);*/break;
		case kE: s.x+=1; /*angle.Set(1,0);*/ break;
		case kW: s.x-=1; /*angle.Set(-1,0);*/break;
		case kNW: s.y-=1; s.x-=1; /*angle.Set(-1,-1);*/ break;
		case kSW: s.y+=1; s.x-=1; /*angle.Set(-1,1);*/break;
		case kNE: s.y-=1; s.x+=1; /*angle.Set(1,-1);*/  break;
		case kSE: s.y+=1; s.x+=1; /*angle.Set(1,1);*/ break;
		default: break;
	}
	

	
	// May need to do different check as well oi->CanMove for example
	if (map->canStep(s.x, s.y, old.x, old.y))// && !(oi->GetStateOccupied(s)))(should be done in unitSim)
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
		
/*		double oldProportion = 0.8;
		double newProportion = 1 - oldProportion;
		
		// Update angle on new location
		AngleUtil::AngleSearchNode sn(s,GetStateHash(s));
		if(angleLookup.find(sn) == angleLookup.end())
		{
			angleLookup[sn] = angle;
		}
		else
		{
			Vector2D oldAngle = angleLookup[sn];
			angleLookup[sn] = oldProportion * oldAngle + newProportion * angle;
		}
		
		// Update angle on old location
		AngleUtil::AngleSearchNode sn2(old,GetStateHash(old));
		if(angleLookup.find(sn2) == angleLookup.end())
		{
			angleLookup[sn2] = angle;
		}
		else
		{
			Vector2D oldAngle = angleLookup[sn2];
			angleLookup[sn2] = oldProportion * oldAngle + newProportion * angle;
		}	
		
		//UnitSimulation will deal with all occupancy interface 'stuff'
		//oi->SetStateOccupied(s, false);
		//oi->SetStateOccupied(old, true);*/
		
		return;
	}
	s = old;
}

void WeightedMap2DEnvironment::UpdateAngle(xyLoc &old, xyLoc &s, double oldProp)
{
	if(old == s) 
		return;
	tDirection dir = GetAction(old, s);
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
	angle.Normalize(); //<-- otherwise diagonal ones are messed up
	double newProportion = 1 - oldProp;
		
	if(updateSurrounding)
	{
		//Update new location
		AngleUtil::AngleSearchNode sn(s,GetStateHash(s));
		if(angleLookup.find(sn) == angleLookup.end())
		{
			if(usePerceptron)
				angleLookup[sn] = learningRate * angle;		
			else
			{
				angle.Normalize();
				angleLookup[sn] = angle;
			}
		}
		else
		{
			if(usePerceptron)
			{
				Vector2D oldAngle = angleLookup[sn];
				
				float x = oldAngle.x - learningRate * (oldAngle.x - angle.x);
				float y = oldAngle.y - learningRate * (oldAngle.y - angle.y);
				
				Vector2D	newAngle(x,y);
// 				std::cout<<sqrt(x*x + y*y)<<std::endl;
				if(sqrt(x*x + y*y) > 1)
					newAngle.Normalize();
					
				angleLookup[sn] = newAngle;
			}
			else
			{
				Vector2D oldAngle = angleLookup[sn];
				
				Vector2D storeme = oldProp * oldAngle + newProportion * angle;
				storeme.Normalize();
				angleLookup[sn] = storeme;
			}
		}
		
		//Update all neighbours of new location
		std::vector<xyLoc> neighbours; 
		GetSuccessors(s, neighbours);
		
		//oldProp = surroundingProportion;
		//newProportion = 1 - oldProp;
		
		for(unsigned int i=0; i<neighbours.size(); i++)
		{
			AngleUtil::AngleSearchNode sn(neighbours[i],GetStateHash(neighbours[i]));
			if(angleLookup.find(sn) == angleLookup.end())
			{
				if(usePerceptron)
					angleLookup[sn] = surroundingProportion * angle;		
				else
				{
					angle.Normalize();
					angleLookup[sn] = angle;
				}
			}
			else
			{
				if(usePerceptron)
				{
					Vector2D oldAngle = angleLookup[sn];
					
					float x = oldAngle.x - surroundingProportion * (oldAngle.x - angle.x);
					float y = oldAngle.y - surroundingProportion * (oldAngle.y - angle.y);
				
					Vector2D	newAngle(x,y);
// 					std::cout<<sqrt(x*x + y*y)<<std::endl;
					if(sqrt(x*x + y*y) > 1)
						newAngle.Normalize();
					
					angleLookup[sn] = newAngle;
				}
				else
				{
					Vector2D oldAngle = angleLookup[sn];
					Vector2D storeme = oldProp * oldAngle + newProportion * angle;
					storeme.Normalize();
					angleLookup[sn] = storeme;
				}
			}
		}
	} // end if(updateSurrounding)
	else
	{
		// Update angle on new location
		AngleUtil::AngleSearchNode sn(s,GetStateHash(s));
		if(angleLookup.find(sn) == angleLookup.end())
		{
			if(usePerceptron)
				angleLookup[sn] = learningRate * angle;		
			else
			{
				angle.Normalize();
				angleLookup[sn] = angle;
			}
		}
		else
		{
			if(usePerceptron)
			{
				Vector2D oldAngle = angleLookup[sn];
				
				float x = oldAngle.x - learningRate * (oldAngle.x - angle.x);
				float y = oldAngle.y - learningRate * (oldAngle.y - angle.y);
				
				Vector2D	newAngle(x,y);
// 				std::cout<<sqrt(x*x + y*y)<<std::endl;
				if(sqrt(x*x + y*y) > 1)
					newAngle.Normalize();
					
				angleLookup[sn] = newAngle;
			}
			else
			{
				Vector2D oldAngle = angleLookup[sn];
				Vector2D storeme = oldProp * oldAngle + newProportion * angle;
				storeme.Normalize();
				angleLookup[sn] = storeme;
			}
		}
	
		// Update angle on old location
		AngleUtil::AngleSearchNode sn2(old,GetStateHash(old));
		if(angleLookup.find(sn2) == angleLookup.end())
		{
			if(usePerceptron)
				angleLookup[sn] = learningRate * angle;		
			else
			{
				angle.Normalize();
				angleLookup[sn] = angle;
			}
		}
		else
		{
			if(usePerceptron)
			{
				Vector2D oldAngle = angleLookup[sn];
				
				float x = oldAngle.x - learningRate * (oldAngle.x - angle.x);
				float y = oldAngle.y - learningRate * (oldAngle.y - angle.y);
				
				Vector2D	newAngle(x,y);
// 				std::cout<<sqrt(x*x + y*y)<<std::endl;
				if(sqrt(x*x + y*y) > 1)
					newAngle.Normalize();
					
				angleLookup[sn] = newAngle;
			}
			else
			{
				Vector2D oldAngle = angleLookup[sn];
				Vector2D storeme = oldProp * oldAngle + newProportion * angle;
				storeme.Normalize();
				angleLookup[sn] = storeme;
			}
		}			
	}
	return;
}


void WeightedMap2DEnvironment::UpdateAngle(xyLoc &old, xyLoc &s)
{
	UpdateAngle(old,s,oldProportion);
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
	// Update angles 
 	if(updateOnQuery)
 	{
 		UpdateAngle(l1,l2,queryOldProportion);
 	}
 	
	
	//std::cou<t<"l1 "<<l1<<" l2 "<<l2<<std::endl;
	double h = HCost(l1, l2);
	
	if(noWeighting)
		return h;
	
	// No weighting if we're using the window and we're outside the window
	if(useWindow && ( (HCost(windowCenter,l1) > windowSize) || 
							(HCost(windowCenter, l2) > windowSize)))
	{
		return h; 
	}
	
	if (fgreater(h, ROOT_TWO))
		return DBL_MAX;

	tDirection dir = GetAction(l1,l2);
	
	Vector2D angle = GetAngleFromDirection(dir);
		
	AngleUtil::AngleSearchNode sn(l1,GetStateHash(l1));
	
	double weight1 = 0; double weight2 = 0;
	if(angleLookup.find(sn) != angleLookup.end())
		{
			Vector2D old = angleLookup[sn];
			
			//double returnme = h + ( ((old.x-angle.x)*(old.x-angle.x)+(old.y-angle.y)*(old.y-angle.y)));
			
			weight1 = diffWeight * ((1 - ((old.x * angle.x)+(old.y * angle.y)))/2);
/*			if(old == angle)
			{
				return h;
			}
			else
			{
				return 10 * h;
			}*/
		}
		
		AngleUtil::AngleSearchNode sn2(l2,GetStateHash(l2));
		if(angleLookup.find(sn) != angleLookup.end())
		{
			Vector2D old = angleLookup[sn2];
			weight2 = diffWeight * ((1 - ((old.x * angle.x)+(old.y * angle.y)))/2);
		}
		
		return h + (0.5 * weight1 + 0.5 * weight2);
	
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

}

void WeightedMap2DEnvironment::OpenGLDraw(int window)
{	
	
	// Draw the map
	//AbsMapEnvironment::OpenGLDraw(window);
	
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
			//glColor3f(1.0, 0,0);
			glColor3f(0,0,0);
			map->getOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
			//glLineWidth(2);
			glBegin(GL_LINE_STRIP);
			glVertex3f(xx-old.x*rad, yy-old.y*rad, zz-rad/2);
			
			//glColor3f(0.0, 0, 0);
			glVertex3f(xx+old.x*rad, yy+old.y*rad, zz-rad/2);	
			glEnd();
			
			double percOffset=0.5;
			double lengthOffset=0.5*rad;
			glBegin(GL_LINE_STRIP);
			glVertex3f(xx+percOffset*old.x*rad-old.y*lengthOffset,
							yy+percOffset*old.y*rad+old.x*lengthOffset, zz-rad/2);
			glVertex3f(xx+old.x*rad, yy+old.y*rad, zz-rad/2);	
							
			glVertex3f(xx+percOffset*old.x*rad+old.y*lengthOffset,
							yy+percOffset*old.y*rad-old.x*lengthOffset, zz-rad/2);
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

void WeightedMap2DEnvironment::SetAngle(xyLoc &l, Vector2D angle)
{
	AngleUtil::AngleSearchNode sn(l,GetStateHash(l));
	angleLookup[sn]=angle;
}

Vector2D WeightedMap2DEnvironment::GetAngle(xyLoc &l)
{
	AngleUtil::AngleSearchNode sn(l,GetStateHash(l));
	if(angleLookup.find(sn) != angleLookup.end())
		return angleLookup[sn];
	else 
	{
		Vector2D angle(0,0);
		return angle;
	}
}

double WeightedMap2DEnvironment::ComputeArrowMetric(bool doNormalize)
{
	Graph* g = ma->GetAbstractGraph(0);
	node_iterator ni = g->getNodeIter();
	
	// Create a vector of directions so we can loop through them
	std::vector<tDirection> directions;
	directions.push_back(kN);
	directions.push_back(kNE);
	directions.push_back(kE);
	directions.push_back(kSE);
	directions.push_back(kS);
	directions.push_back(kSW);
	directions.push_back(kW);
	directions.push_back(kNW);
	
	int numDotProducts = 0;
	double totalDotProducts = 0;
	
	for(node *n=g->nodeIterNext(ni); n; n=g->nodeIterNext(ni))
	{
		
		xyLoc current;
		current.x = n->GetLabelL(kFirstData);
		current.y = n->GetLabelL(kFirstData+1);
				
		Vector2D nodeVec = GetAngle(current);
		if(nodeVec.x == 0 && nodeVec.y == 0) // check if there's no angle stored here
			continue;
					
		double bestDotProd = -1;
		tDirection bestDir;		
				
		for(unsigned int i=0; i<directions.size(); i++)
		{
			Vector2D dirVec = GetAngleFromDirection(directions[i]);
			
			double dotProd = (nodeVec.x * dirVec.x) + (nodeVec.y * dirVec.y);
			
			if(dotProd > bestDotProd)
			{
				bestDotProd = dotProd;
				bestDir = directions[i];
			}	
		}
		
		// Now that we know the closest direction, find out if there's a node there 
		xyLoc next;
		GetNextState(current, bestDir, next);
			
		if(!(next==current))
		{
			numDotProducts++;
			Vector2D nextNodeVec = GetAngle(next);
			if(doNormalize)
			{
				nodeVec.Normalize();
				nextNodeVec.Normalize();
			}
			//double dotprod = (nodeVec.x * nextNodeVec.x) + (nodeVec.y * nextNodeVec.y);
			//totalDotProducts += dotprod;
			Vector2D difference((nodeVec.x + nextNodeVec.x)/2, (nodeVec.y + nextNodeVec.y)/2);
			double addme = sqrt(difference.x * difference.x + difference.y * difference.y);
			totalDotProducts += addme;
		}	
	}
	return (totalDotProducts / (double)(numDotProducts));
}

Vector2D WeightedMap2DEnvironment::GetAngleFromDirection(tDirection dir)
{
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
	
	angle.Normalize();
	
	return angle;
}
