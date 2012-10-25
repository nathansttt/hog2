///*
// *  patrolUnit.cpp
// *  hog
// *
// *  Created by Nathan Sturtevant on 3/23/06.
// *  Copyright 2006 Nathan Sturtevant, University of Alberta. All rights reserved.
// *
// * This file is part of HOG.
// *
// * HOG is free software; you can redistribute it and/or modify
// * it under the terms of the GNU General Public License as published by
// * the Free Software Foundation; either version 2 of the License, or
// * (at your option) any later version.
// * 
// * HOG is distributed in the hope that it will be useful,
// * but WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// * GNU General Public License for more details.
// * 
// * You should have received a copy of the GNU General Public License
// * along with HOG; if not, write to the Free Software
// * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
// *
// */
//
//#include "AbsMapPatrolUnit.h"
//
//using namespace GraphAbstractionConstants;
//
//AbsMapPatrolUnit::AbsMapPatrolUnit(int _x, int _y, SearchAlgorithm* alg)
//:SearchUnit(_x,_y,NULL,alg)
//{
//	std::cout<<"Being created\n";
//	xyLoc l;
//	l.x = _x;
//	l.y = _y;
//	Locs.push_back(l);
//	
//	//setObjectType(kWorldObject);
//	currTarget = -1;
//	nodesExpanded = nodesTouched = 0;
//}
//
/////** Creates a patrol unit and assigns it numPLocations random locations to patrol.
////    The last location is the unit's starting position to create a loop */
////AbsMapPatrolUnit::AbsMapPatrolUnit(int _x, int _y, int numPLocations, unitSimulation* us)
////:unit(_x, _y)
////{
////	setObjectType(kWorldObject);
////	currTarget = -1;
////	nodesExpanded = nodesTouched = 0;
////	for (int i=1; i<numPLocations; i++)
////	{
////		int xx, yy;
////		us->getRandomLocation(_x,_y,xx,yy);
////		addPatrolLocation(new unit(xx, yy));
////	}
////	addPatrolLocation(new unit(_x, _y));
////}
//
//AbsMapPatrolUnit::~AbsMapPatrolUnit()
//{
//}
//
//bool AbsMapPatrolUnit::makeMove(MapProvider *mp, reservationProvider *, AbsMapSimulationInfo *, tDirection &dir)
//{
//	std::cout<<"CurrTarget "<<currTarget<<std::endl;
//	MapAbstraction *aMap = mp->GetMapAbstraction();
//	if (moves.size() > 0)
//	{
//		dir = moves.back();
//		moves.pop_back();
//		return true;
//	}
//	
//	if (currTarget != -1)
//	{
//		goToLoc(aMap, currTarget);
//		currTarget = (currTarget+1)%Locs.size();
//		if (moves.size() > 0)
//		{
//			dir = moves.back();
//			moves.pop_back();
//			return true;
//		}
//	}
//	dir = kStay;
//	return true;
//}
//
//double AbsMapPatrolUnit::goToLoc(MapAbstraction *aMap, int which)
//{
//	std::cout<<"goToLoc\n";
//	double pathCost=-1;
//	path *p;
//	node *from, *to;
//	from = aMap->GetNodeFromMap(loc.x, loc.y); // gets my location
//	int tox=Locs[which].x;
//	int toy=Locs[which].y;
//	//Locs[which].getLocation(tox, toy);
////	printf("Patrol unit going to %d, %d\n", tox, toy);
//	to = aMap->GetNodeFromMap(tox, toy);
//	p = algorithm->GetPath(aMap, from, to);
//	nodesExpanded += algorithm->GetNodesExpanded();
//	nodesTouched += algorithm->GetNodesTouched();
//	if (p)
//	{
//		pathCost = aMap->distance(p);
//		addPathToCache(p);
//	}
//	return pathCost;
//}
//
//void AbsMapPatrolUnit::OpenGLDraw(const AbsMapEnvironment *ame, const AbsMapSimulationInfo *) const
//
//{
//	GLdouble xx, yy, zz, rad;
//	Map *map = ame->GetMap();
//	int posx = loc.x, posy = loc.y;
//	map->GetOpenGLCoord(posx, posy, xx, yy, zz, rad);
//	glColor3f(r, g, b);
//	glBegin(GL_LINE_STRIP);
////	glVertex3f(xx, yy+rad/2, zz);
//	glVertex3f(xx, yy, zz-rad/2);
//	for (int t = moves.size()-1; t >= 0; t--)
//	{
//		posx += ((moves[t]&kE)?1:0) - ((moves[t]&kW)?1:0);
//		posy += ((moves[t]&kS)?1:0) - ((moves[t]&kN)?1:0);
//		
//		map->GetOpenGLCoord(posx, posy, xx, yy, zz, rad);
//		
////		glVertex3f(xx, yy+rad/2, zz);
//		glVertex3f(xx, yy, zz-rad/2);
//	}
//	glEnd();
//	
//	map->GetOpenGLCoord(loc.x, loc.y, xx, yy, zz, rad);
//	glColor3f(r, g, b);
//	DrawSphere(xx, yy, zz, rad);
//}
//
//void AbsMapPatrolUnit::addPatrolLocation(xyLoc ru)
//{
//	
//	currTarget = 1;
//	Locs.push_back(ru);
//}
//
//xyLoc AbsMapPatrolUnit::GetGoal()
//{
//	if (currTarget == -1)
//	{
//		xyLoc l;
//		l.x = -1;
//		l.y = -1;
//		return l;
//	}
//	return Locs[currTarget];
//}
//
//void AbsMapPatrolUnit::addPathToCache(path *p)
//{
//	// we are at the last move; abort recursion
//	if ((p == NULL) || (p->next == NULL))
//		return;
//	// there is another move; add it first to cache
//	if (p->next->next)
//		addPathToCache(p->next);
//	
//	// ----- Ok, we have a path starting at (x,y) [the current location] and
//	// having at least one more state ----------------------------------------
//	
//	// Take the first move off the path and execute it
//	int result = kStay;
//	
//	// Decide on the horizontal move
//	switch ((p->n->GetLabelL(kFirstData)-p->next->n->GetLabelL(kFirstData)))
//	{
//		case -1: result = kE; break;
//		case 0: break;
//		case 1: result = kW; break;
//		default :
//			printf("SU: %s : The (x) nodes in the path are not next to each other!\n",
//						 this->GetName());
//			printf("Distance is %ld\n",
//						 p->n->GetLabelL(kFirstData)-p->next->n->GetLabelL(kFirstData));
//			std::cout << *p->n << "\n" << *p->next->n << "\n";
//			exit(10); break;
//	}
//	
//	// Tack the vertical move onto it
//	// Notice the exploit of particular encoding of kStay, kE, etc. labels
//	switch ((p->n->GetLabelL(kFirstData+1)-p->next->n->GetLabelL(kFirstData+1)))
//	{
//		case -1: result = result|kS; break;
//		case 0: break;
//		case 1: result = result|kN; break;
//		default :
//			printf("SU: %s : The (y) nodes in the path are not next to each other!\n",
//						 this->GetName());
//			printf("Distance is %ld\n",
//						 p->n->GetLabelL(kFirstData+1)-p->next->n->GetLabelL(kFirstData+1));
//			std::cout << *p->n << "\n" << *p->next->n << "\n";
//			exit(10); break;
//	}
//	moves.push_back((tDirection)result);
//}
//
//void AbsMapPatrolUnit::logStats(StatCollection *stats)
//{
//	if (((nodesExpanded == 0) && (nodesTouched != 0)) ||
//			((nodesExpanded != 0) && (nodesTouched == 0)))
//	{
//		printf("Error; somehow nodes touched/expanded are inconsistent. t:%d e:%d\n",
//					 nodesTouched, nodesExpanded);
//	}
//	// printf("SearchUnit::logStats(nodesExpanded=%d, nodesTouched=%d)\n",nodesExpanded,nodesTouched);
//	if (nodesExpanded != 0)
//		stats->AddStat("nodesExpanded", GetName(), (long)nodesExpanded);
//	if (nodesTouched != 0)
//		stats->AddStat("nodesTouched", GetName(), (long)nodesTouched);
//	nodesExpanded = nodesTouched = 0;
//}
//
//void AbsMapPatrolUnit::LogFinalStats(StatCollection *stats)
//{
//	algorithm->LogFinalStats(stats);
//}
