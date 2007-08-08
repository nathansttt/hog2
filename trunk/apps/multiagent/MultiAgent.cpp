/** A general multiagent pathfinding application. 
 * 
 *  @file multiAgent.cpp
 *  @package hog2
 *
 * This file is part of HOG2.
 *
 * HOG2 is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG2; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include "Common.h"
#include "MultiAgent.h"
#include "AStar.h"
#include "PRAStar.h"
#include "SearchUnit.h"
//#include "SharedAMapGroup.h"
#include "UnitGroup.h"
#include "MapCliqueAbstraction.h"
#include "NodeLimitAbstraction.h"
#include "MapQuadTreeAbstraction.h"
//#include "RadiusAbstraction.h"
#include "MapFlatAbstraction.h"
//#include "ClusterAbstraction.h"
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include "Plot2D.h"
#include "Map2DEnvironment.h"
#include "RandomUnits.h"

#include "AbsMapPatrolUnit.h"
#include "TemplateAStar.h"
#include "GenericSearchUnit.h"
#include "GenericPatrolUnit.h"
#include "WeightedMap2DEnvironment.h"

bool mouseTracking;
int px1, py1, px2, py2;
int absType = 0;

AbsMapEnvironment *env=0;

std::vector<UnitAbsMapSimulation *> unitSims;
//std::vector<UnitWeightedMapSimulation *> unitSims;
//unit *cameraTarget = 0;

Plotting::Plot2D *plot = 0;
Plotting::Line *distLine = 0;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv);
}


/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
void CreateSimulation(int id)
{
	Map *map;
//	if (gDefaultMap[0] == 0)
//	{
		//Empty map for now
		map = new Map(60, 60);
		//MakeMaze(map, 1);
//	}
//	else
//		map = new Map(gDefaultMap);

	unitSims.resize(id+1);
	//unitSims[id] = new EpisodicSimulation<xyLoc, tDirection, AbsMapEnvironment>(new AbsMapEnvironment(new MapQuadTreeAbstraction(map, 2)));
	//unitSims[id] = new EpisodicSimulation<xyLoc, tDirection, AbsMapEnvironment>(new AbsMapEnvironment(new NodeLimitAbstraction(map, 8)));
	//unitSims[id] = new UnitSimulation<xyLoc, tDirection, AbsMapEnvironment>(new AbsMapEnvironment(new MapCliqueAbstraction(map)));
	
	env = new AbsMapEnvironment(new MapFlatAbstraction(map));
	
	unitSims[id] = new UnitSimulation<xyLoc, tDirection, AbsMapEnvironment>(env);
	
	
	
	//unitSims[id] = new UnitSimulation<xyLoc, tDirection, AbsMapEnvironment>(new WeightedMap2DEnvironment(new MapFlatAbstraction(map)));
	
	
	//unitSims[id] = new UnitSimulation<xyLoc, tDirection, WeightedMap2DEnvironment>(new WeightedMap2DEnvironment (new MapFlatAbstraction(map)));
	
	unitSims[id]->SetStepType(kMinTime);
//	unitSim = new UnitSimulation<xyLoc, tDirection, MapEnvironment>(new MapEnvironment(map),
//																																 (OccupancyInterface<xyLoc, tDirection>*)0);
//	if (absType == 0)
//		unitSim = new unitSimulation(new MapCliqueAbstraction(map));
//	else if (absType == 1)
//		unitSim = new unitSimulation(new RadiusAbstraction(map, 1));
//	else if (absType == 2)
//		unitSim = new unitSimulation(new MapQuadTreeAbstraction(map, 2));
//	else if (absType == 3)
//		unitSim = new unitSimulation(new ClusterAbstraction(map, 8));
	
	//unitSim->setCanCrossDiagonally(true);
	//unitSim = new unitSimulation(new MapFlatAbstraction(map));
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kNoModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
//	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase //abstraction type", kAnyModifier, ']');
//	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease //abstraction type", kAnyModifier, '[');

	InstallKeyboardHandler(MyPathfindingKeyHandler, "Patrol Unit", "Deploy patrol unit that patrols between two locations on the map", kNoModifier, 'd');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
//	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a //randomly moving unit", kShiftDown, 'a');
//	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a //right-hand-rule unit", kControlDown, 1);

//	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the //default map to be loaded.");
	
	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler);
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		CreateSimulation(windowID);
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if ((windowID < unitSims.size()) && (unitSims[windowID] == 0))
		return;

	if (viewport == 0)
	{
		unitSims[windowID]->StepTime(1.0/30.0);
	}
	unitSims[windowID]->OpenGLDraw(windowID);
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	strncpy(gDefaultMap, argument[1], 1024);
	return 2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '\t':
			if (mod != kShiftDown)
				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
			else
			{
				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
			}
			break;
		case 'p': unitSims[windowID]->SetPaused(!unitSims[windowID]->GetPaused()); break;
		case 'o':
		{
			if (unitSims[windowID]->GetPaused())
			{
				unitSims[windowID]->SetPaused(false);
				unitSims[windowID]->StepTime(1.0/30.0);
				unitSims[windowID]->SetPaused(true);
			}
		}
		break;
//		case ']': absType = (absType+1)%3; break;
//		case '[': absType = (absType+4)%3; break;
//		case '{': unitSim->setPaused(true); unitSim->offsetDisplayTime(-0.5); break;
//		case '}': unitSim->offsetDisplayTime(0.5); break;
		default:
			if (unitSims[windowID])
				unitSims[windowID]->GetEnvironment()->GetMapAbstraction()->ToggleDrawAbstraction(((mod == kControlDown)?10:0)+(key-'0'));
			break;
	}
}

void MyRandomUnitKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
/*	Map *m = unitSims[windowID]->GetEnvironment()->GetMap();
	
	int x1, y1, x2, y2;
	x2 = random()%m->getMapWidth();
	y2 = random()%m->getMapHeight();
	x1 = random()%m->getMapWidth();
	y1 = random()%m->getMapHeight();
	SearchUnit *su1 = new SearchUnit(x1, y1, 0, 0);
	//SearchUnit *su2 = new SearchUnit(random()%m->getMapWidth(), random()%m->getMapHeight(), su1, new praStar());
	SearchUnit *su2 = new SearchUnit(x2, y2, su1, new aStar());
	//unitSim->AddUnit(su1);
	unitSims[windowID]->AddUnit(su2);*/
	
//	RandomerUnit *r = new RandomerUnit(random()%m->getMapWidth(), random()%m->getMapHeight());
//	int id = unitSim->AddUnit(r);
//	xyLoc loc;
//	r->GetLocation(loc);
//	printf("Added unit %d at (%d, %d)\n", id, loc.x, loc.y);

//	int x1, y1, x2, y2;
//	unit *u;
//	unitSim->getRandomLocation(x1, y1);
//	unitSim->getRandomLocation(x2, y2);
//	switch (mod)
//	{
//		case kControlDown: unitSim->addUnit(u=new rhrUnit(x1, y1)); break;
//		case kShiftDown: unitSim->addUnit(u=new randomUnit(x1, y1)); break;
//		default:
//			unit *targ;
//	unitSim->addUnit(targ = new unit(x2, y2));
//	unitSim->addUnit(u=new SearchUnit(x1, y1, targ, new praStar())); break;
//	}
//	delete plot;
//	plot = new Plotting::Plot2D();
//	delete distLine;
//	plot->AddLine(distLine = new Plotting::Line("distline"));
//	cameraTarget = u;
//	u->setSpeed(1.0/4.0);
}

void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
//	for (int x = 0; x < ((mod==kShiftDown)?(50):(1)); x++)
//	{
	//if (unitSims[windowID]->GetUnitGroup(1) == 0)
	//{
	//	unitSims[windowID]->AddUnitGroup(new unitGroup(unitSim));
		//unitSims[windowID]->setmapAbstractionDisplay(2);
	//}

	int xStart=5, yStart=5, xGoal=50, yGoal=50;
	xyLoc start;
	start.x = xStart;
	start.y = yStart;
	
	xyLoc goal;
	goal.x = xGoal;
	goal.y = yGoal;
	
	
	//aStar* a = new aStar();
	
	//std::vector<xyLoc> path;
	TemplateAStar<xyLoc, tDirection,AbsMapEnvironment>* a = new TemplateAStar<xyLoc, tDirection,AbsMapEnvironment>();
	//a->GetPath(env, start,goal, path);
	
//	GenericSearchUnit<xyLoc, tDirection, AbsMapEnvironment>* unit = new GenericSearchUnit<xyLoc, tDirection, AbsMapEnvironment>(start, goal, a);
 	GenericPatrolUnit<xyLoc, tDirection,AbsMapEnvironment> *u = new GenericPatrolUnit<xyLoc, tDirection,AbsMapEnvironment>(start, a);
// 	
 	u->AddPatrolLocation(goal);
// 	
	//AbsMapPatrolUnit* pUnit = new AbsMapPatrolUnit(xStart,yStart, a);
	//pUnit->addPatrolLocation(goal);
	
	//Unit<xyLoc, tDirection, WeightedMap2DEnvironment> *pUnit = new Unit<xyLoc, tDirection, WeightedMap2DEnvironment>;


	unitSims[windowID]->AddUnit(u);
	
	
	//pUnit->setSpeed(0.5);
	
//		int xx1, yy1, xx2, yy2;
//		unitSim->getRandomLocation(xx1, yy1);
//		unitSim->getRandomLocation(xx2, yy2);
//		
//		unit *u, *u2 = new unit(xx2, yy2, 0);
//		
//		praStar *pra = new praStar(); pra->setPartialPathLimit(4);
//		//aStar *pra = new aStar();
//		
//		unitSim->addUnit(u2);
//		u = new SearchUnit(xx1, yy1, u2, pra);
//		// just set the group of the unit, and it will share a map with those
//		// units.
//		unitSim->getUnitGroup(1)->addUnit(u);
//		unitSim->addUnit(u);
//		u->setSpeed(0.5); // time to go 1 distance						
//	}
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	return false;
//	mouseTracking = false;
//	if (button == kRightButton)
//	{
//		switch (mType)
//		{
//			case kMouseDown:
//				unitSim->GetMap()->getPointFromCoordinate(loc, px1, py1);
//				//printf("Mouse down at (%d, %d)\n", px1, py1);
//				break;
//			case kMouseDrag:
//				mouseTracking = true;
//				unitSim->GetMap()->getPointFromCoordinate(loc, px2, py2);
//				//printf("Mouse tracking at (%d, %d)\n", px2, py2);
//				break;
//			case kMouseUp:
//			{
//				if ((px1 == -1) || (px2 == -1))
//					break;
//				unitSim->GetMap()->getPointFromCoordinate(loc, px2, py2);
//				//printf("Mouse up at (%d, %d)\n", px2, py2);
//				unit *u, *u2 = new unit(px2, py2, 0);
//				//praStar *pra = new praStar(); pra->setPartialPathLimit(4);
//				aStar *pra = new aStar();
//				unitSim->addUnit(u2);
//				u = new SearchUnit(px1, py1, u2, pra);
//				unitSim->addUnit(u);
//				u->setSpeed(0.5); // time to go 1 distance						
//			}
//			break;
//		}
//		return true;
//	}
//	return false;
}
