///*
// * $Id: LRTAStarGroup.cpp,v 1.5 2005/12/09 23:49:44 nathanst Exp $
// *
// *  LRTAStarGroup.cpp
// *  HOG
// *
// *  Created by Shanny Lu (based on Vadim Bulitko's LRTS) on 10/03/05.
// *
// */
//
//#include "searchUnit.h"
//#include "LRTAStar.h"
//#include "LRTAStarGroup.h"
//#include "math.h"
//#include "constants.h"
//
//const bool verbose = false;
//
//// The constructor
//LRTAStarGroup::LRTAStarGroup(mapProvider *mp)
//:unitGroup(mp)
//{
//	h = NULL;
//	displayMode = 0;
//	hSize = 0;
//	aMap = mp->getMapAbstraction();
//	allocateH();
//}
//
//
//// Allocate the heuristic at the base level and fill it with -1.0
//void LRTAStarGroup::allocateH() {
//	graph *g = aMap->getAbstractGraph(0);
//	unsigned size = g->getNumNodes();
//	h = new double[size];
//	
//	for (unsigned a=0; a<size; a++)
//		h[a] = -1.0;
//}
//
//
//// Make move for unit u within the LRTA* group
//tDirection LRTAStarGroup::makeMove(unit *u, mapProvider *mapP, reservationProvider *rp, simulationInfo *simInfo)
//{
//	aMap = mapP->getMapAbstraction();
//	return ((LRTAStarUnit*)u)->makeMove(mapP, rp, simInfo, this);
//}
//
//// LRTA* group is done when all units are done
//bool LRTAStarGroup::done()
//{
//	for (unsigned int x = 0; x < myUnits.size(); x++)
//		if (!myUnits[x]->done())
//			return false;
//	return true;
//}
//
//// Let all LRTA* units know that the new trial is started
//void LRTAStarGroup::startNewTrial(statCollection *stats) {
//	for (unsigned int x = 0; x < myUnits.size(); x++)
//		((LRTAStarUnit*)myUnits[x])->startNewTrial();
//}
//
//
//// Draw the information available to the unit group on the map
//void LRTAStarGroup::openGLDraw(mapProvider *mp, simulationInfo *)
//{
//	glBegin(GL_QUADS);
//	glNormal3f(0, 0, -1);
//	GLdouble coverage = 0.9;
//	aMap = mp->getMapAbstraction();
//	
//	// Preliminary computations
//	graph *g = aMap->getAbstractGraph(0);           // base level map
//	Map *map = mp->getMap();
//	
//	// Get the target for LRTA* unit #0 from the LRTA* Unit Group
//	// Eventually, we will pick up a mouse click here where the user
//	// will be able to select the target of h(*,target) to display
//	unit *target = getUnit(0)->getTarget();
//	int x_goal, y_goal;
//	target->getLocation(x_goal, y_goal);
//	int toNodeNum = map->getNodeNum(x_goal,y_goal);
//	node* to = g->getNode(toNodeNum);
//	
//	// get the heuristic value for the h
//	double hToDisplay;
//		
//	// Now, display the heuristic
//	for (int x = 0; x < map->getMapWidth(); x++)
//	{
//		for (int y = 0; y < map->getMapHeight(); y++)
//		{
//			GLdouble a, b, c, radius;
//			
//			// Is this a ground location?		
//			if (map->getTerrainType(x, y) == kGround) 
//			{
//				int fromNodeNum = map->getNodeNum(x,y);
//				node* from = g->getNode(fromNodeNum);
//				if (from == NULL)
//					continue;
//				
//				// Get h
//				hToDisplay = ((displayMode & 0x1) == 0) ? 
//					getDiffH(from,to) : getH(from,to);
//				
//				// change the color according to the heuristic here
//				recColor r = getColor(tanh(hToDisplay*0.05));
//				glColor3fv(&r.r);
//			}
//			else {
//				// black color for non-ground cells
//				glColor3f(0, 0, 0);
//				hToDisplay = 0;
//			}
//			
//			// Draw the rectangle
//			map->getOpenGLCoord(x, y, a, b, c, radius);
//			
//			// Check if we are displaying heights or not
//			if ((displayMode & 0x2) == 0) {
//				// No heights
//				glVertex3f(a+coverage*radius, b+coverage*radius, c-radius/4);
//				glVertex3f(a+coverage*radius, b-coverage*radius, c-radius/4);
//				glVertex3f(a-coverage*radius, b-coverage*radius, c-radius/4);
//				glVertex3f(a-coverage*radius, b+coverage*radius, c-radius/4);
//			}
//			else {
//				// Yes, do the height display
//				// 10 below is the height scaling coefficient
//				hToDisplay = 5*(1+tanh(hToDisplay)) + 0.25;
//				map->getOpenGLCoord(x, y, a, b, c, radius);
//				glVertex3f(a+coverage*radius, b+coverage*radius, c-radius*hToDisplay);
//				glVertex3f(a+coverage*radius, b-coverage*radius, c-radius*hToDisplay);
//				glVertex3f(a-coverage*radius, b-coverage*radius, c-radius*hToDisplay);
//				glVertex3f(a-coverage*radius, b+coverage*radius, c-radius*hToDisplay);
//			}
//			
//		}
//	}
//	
//	glEnd();
//}
//
//
//// Adapted from the by Vadim Bulitko
//// It is assumed that v is between -1.0 and 1.0
//recColor LRTAStarGroup::getColor(double v)
//{
//	recColor c = {1.0,1.0,1.0};
//	
//	if (v>0) {
//		c.b = c.g = 1 - v;
//		c.r = 1;
//	} 
//	else {
//		c.b = c.r = 1 + v;
//		c.g = 1;
//	}
//	
//	return(c);
//}
