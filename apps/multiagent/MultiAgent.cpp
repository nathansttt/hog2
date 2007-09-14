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

#include <cstdlib>

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

#include <fstream>

bool mouseTracking;
int px1, py1, px2, py2;
int absType = 0;

int envType = 0; // 0 --> AbsMapEnvironment

AbsMapEnvironment *env=0;
WeightedMap2DEnvironment *wenv = 0;

std::vector<UnitAbsMapSimulation *> unitSims;
std::vector<UnitWeightedMapSimulation *> wUnitSims;

Plotting::Plot2D *plot = 0;
Plotting::Line *distLine = 0;

char locsFile[1024];

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
	// Only show 1 copy of the map
	SetNumPorts(id, 1);
	
	Map *map;
	if (gDefaultMap[0] == 0)
	{
		map = new Map(60, 60);
	}
	else
		map = new Map(gDefaultMap);

	switch(envType)
	{
		case 0:
		{
			unitSims.resize(id+1);
			env = new AbsMapEnvironment(new MapFlatAbstraction(map));
			unitSims[id] = new UnitSimulation<xyLoc, tDirection, AbsMapEnvironment>(env);
			unitSims[id]->SetStepType(kRealTime);
			unitSims[id]->SetThinkingPenalty(0);
			break;
		}
		case 1:
		{
			wUnitSims.resize(id+1);
			wenv = new WeightedMap2DEnvironment(new MapFlatAbstraction(map));
			wUnitSims[id] = new UnitSimulation<xyLoc, tDirection, WeightedMap2DEnvironment>(wenv);	
			wUnitSims[id]->SetStepType(kRealTime);
			wUnitSims[id]->SetThinkingPenalty(0);
			break;
		}
		default:
		{
			std::cout<<"Invalid environment\n";
			break;
		}
	}	
	
	// Test patrol unit
/*	xyLoc start, goal, goalTwo, goalThree;
	start.x = 5;
	start.y = 5;
	goal.x = 10;
	goal.y = 5;
	goalTwo.x = 10;
	goalTwo.y = 10;
	goalThree.x = 5;
	goalThree.y = 10;
	
	TemplateAStar<xyLoc, tDirection,WeightedMap2DEnvironment>* a = new TemplateAStar<xyLoc, tDirection,WeightedMap2DEnvironment>();
	
	GenericPatrolUnit<xyLoc, tDirection,WeightedMap2DEnvironment> *u = new GenericPatrolUnit<xyLoc, tDirection,WeightedMap2DEnvironment>(start, a);
	
	//u->AddPatrolLocation(goal);
	u->AddPatrolLocation(goalTwo);
	//u->AddPatrolLocation(goalThree);
	u->SetNumPatrols(1);
	
	wUnitSims[id]->AddUnit(u);*/
	
	
	// TEST adding groups to units, vice versa

	
// 	std::cout<<"Create two units\n";
// 	
// 	xyLoc start,goal;
// 	start.x = 5; start.y = 5; goal.x = 7; goal.y = 7;
// 	TemplateAStar<xyLoc, tDirection,AbsMapEnvironment>* a = new TemplateAStar<xyLoc, tDirection,AbsMapEnvironment>();
// 	GenericSearchUnit<xyLoc,tDirection,AbsMapEnvironment> *su = new GenericSearchUnit<xyLoc,tDirection,AbsMapEnvironment>(start,goal,a);
// 	 
// 	std::cout<<"After creating one\n";
// 	 
// 	unitSims[id]->AddUnit(su);
// 	
// 
// 	UnitGroup<xyLoc,tDirection,AbsMapEnvironment> *g = su->GetUnitGroup();
// 	std::cout<<"The first unit belongs to "<<g<<std::endl;
// 	
// 		std::cout<<"Create two unit groups\n";
// 	UnitGroup<xyLoc,tDirection,AbsMapEnvironment> *ug = new UnitGroup<xyLoc,tDirection,AbsMapEnvironment>;
// 	std::cout<<"Adding unit group "<<unitSims[id]->AddUnitGroup(ug)
// 	<<" "<<ug<<std::endl;
// 	
// 	UnitGroup<xyLoc,tDirection,AbsMapEnvironment> *ug1 = new UnitGroup<xyLoc,tDirection,AbsMapEnvironment>;
// 	std::cout<<"Adding unit group "<<unitSims[id]->AddUnitGroup(ug1)<<" "<<ug1<<std::endl;
// 	
// 	
// 	
// 	UnitGroup<xyLoc,tDirection,AbsMapEnvironment> *g2 = su->GetUnitGroup();
// 	std::cout<<"The first unit now belongs to "<<g2<<std::endl;
// 	
// 		su->SetUnitGroup(ug1);
	
	// First experiment code - on particular map (Aug 28 2007)
	if(strcmp(gDefaultMap, "../../maps/local/test_s1_ground.map")==0)
	{
		// Set up experiment
		srand(time(0));
		// Place units on LHS
		
		std::ifstream input(locsFile);
	
		
		for(unsigned int i = 0; i<10; i++)
		{
			// STORE LOCATIONS SOMEWHERE -- TO PRINT (& do other exp)	
			int xStart, yStart, xGoal, yGoal;
				
			input>>xStart>>yStart>>xGoal>>yGoal;
				
			//std::cout<<xStart<<" "<<yStart<<" "<<xGoal<<" "<<yGoal<<std::endl;
			
			xyLoc start, goal;
			start.x = xStart;
			start.y = yStart;
			goal.x = xGoal;
			goal.y = yGoal;
				
			switch(envType)
			{
			case 0:
			{
			TemplateAStar<xyLoc, tDirection, AbsMapEnvironment> *alg = new TemplateAStar<xyLoc, tDirection, AbsMapEnvironment>();
				
			//GenericSearchUnit<xyLoc, tDirection, AbsMapEnvironment> *su = new GenericSearchUnit<xyLoc, tDirection, AbsMapEnvironment>(start,goal,alg);
				
			GenericPatrolUnit<xyLoc,tDirection,AbsMapEnvironment> *su = new GenericPatrolUnit<xyLoc,tDirection,AbsMapEnvironment>(start,alg);
			su->AddPatrolLocation(goal);
			su->SetNumPatrols(5);	
			
			su->SetSpeed(2.0);	
			su->SetColor(1, 0, 0);
			unitSims[id]->AddUnit(su);
			break;
			}
			case 1:
			{
			TemplateAStar<xyLoc, tDirection, WeightedMap2DEnvironment> *alg = new TemplateAStar<xyLoc, tDirection, WeightedMap2DEnvironment>();
				
			//GenericSearchUnit<xyLoc, tDirection, WeightedMap2DEnvironment> *su = new GenericSearchUnit<xyLoc, tDirection, WeightedMap2DEnvironment>(start,goal,alg);
			
			GenericPatrolUnit<xyLoc,tDirection,WeightedMap2DEnvironment> *su = new GenericPatrolUnit<xyLoc,tDirection,WeightedMap2DEnvironment>(start,alg);
			su->AddPatrolLocation(goal);
			su->SetNumPatrols(5);
			
			su->SetSpeed(2.0);	
			su->SetColor(1, 0, 0);

			wUnitSims[id]->AddUnit(su);
			break;
			}
			default:
			{
				std::cout<<"Invalid environment\n";
				break;
			}
			
			}	// end switch
		
		} // end for
		
		for(unsigned int i=0; i<10; i++)
		{
			int xStart, yStart, xGoal, yGoal;
				
			input>>xGoal>>yGoal>>xStart>>yStart;
				
			//std::cout<<xGoal<<" "<<yGoal<<" "<<xStart<<" "<<yStart<<std::endl;
			
			xyLoc start, goal;
			start.x = xStart;
			start.y = yStart;
			goal.x = xGoal;
			goal.y = yGoal;
			
			switch(envType)
			{
				case 0:
				{
					TemplateAStar<xyLoc, tDirection, AbsMapEnvironment> *alg = new TemplateAStar<xyLoc, tDirection, AbsMapEnvironment>();
				
					//GenericSearchUnit<xyLoc, tDirection, AbsMapEnvironment> *su = new GenericSearchUnit<xyLoc, tDirection, AbsMapEnvironment>(start,goal,alg);
				
					GenericPatrolUnit<xyLoc,tDirection,AbsMapEnvironment> *su = new GenericPatrolUnit<xyLoc,tDirection,AbsMapEnvironment>(start,alg);
					su->AddPatrolLocation(goal);
					su->SetNumPatrols(5);
					su->SetSpeed(1.0);	
					su->SetColor(0, 1, 0);

					unitSims[id]->AddUnit(su);
					break;
				}
				case 1:
				{
				TemplateAStar<xyLoc, tDirection, WeightedMap2DEnvironment> *alg = new TemplateAStar<xyLoc, tDirection, WeightedMap2DEnvironment>();
				
				//GenericSearchUnit<xyLoc, tDirection, WeightedMap2DEnvironment> *su = new GenericSearchUnit<xyLoc, tDirection, WeightedMap2DEnvironment>(start,goal,alg);
				GenericPatrolUnit<xyLoc,tDirection,WeightedMap2DEnvironment> *su = new GenericPatrolUnit<xyLoc,tDirection,WeightedMap2DEnvironment>(start,alg);
				su->AddPatrolLocation(goal);
				su->SetNumPatrols(5);
				
				su->SetSpeed(1.0);	
				su->SetColor(0, 1, 0);
									
				wUnitSims[id]->AddUnit(su);
				break;
				}
			default:
			{
				std::cout<<"Invalid environment\n";
				break;
			}	
			} // end switch
			//TemplateAStar<xyLoc, tDirection, WeightedMap2DEnvironment> *alg = new TemplateAStar<xyLoc, tDirection, WeightedMap2DEnvironment>();
// 				
			//GenericSearchUnit<xyLoc, tDirection, WeightedMap2DEnvironment> *su = new GenericSearchUnit<xyLoc, tDirection, WeightedMap2DEnvironment>(start,goal,alg);
			
			/*TemplateAStar<xyLoc, tDirection, AbsMapEnvironment> *alg = new TemplateAStar<xyLoc, tDirection, AbsMapEnvironment>();
				
			GenericSearchUnit<xyLoc, tDirection, AbsMapEnvironment> *su = new GenericSearchUnit<xyLoc, tDirection, AbsMapEnvironment>(start,goal,alg);
				
			su->SetSpeed(1.0);	
				
			unitSims[id]->AddUnit(su);*/
		
		
		}
		wUnitSims[id]->SetPaused(true);
		input.close();
		
		double timestep = 1.5;
		double time = 0.0;
		
/*		switch(envType)
 	{
		case 0:
				while(!(unitSims[id]->Done()))
		{
			unitSims[id]->StepTime(timestep);
			time += timestep;
			//std::cout<<time<<std::endl;
		}

			break;
		case 1:
				while(!(wUnitSims[id]->Done()))
		{
			wUnitSims[id]->StepTime(timestep);
			time += timestep;
			//std::cout<<time<<std::endl;
		}

			break;
		default:
			std::cout<<"Invalid environment\n";
			break;
	}	

		std::cout<<"Simulation finished after "<<time<<" seconds.\n";  
 		exit(0);*/
 	}
 	
	
	

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

	InstallKeyboardHandler(MyPatrolKeyHandler, "Patrol Unit", "Deploy patrol unit that patrols between two locations on the map", kNoModifier, 'd');
	InstallKeyboardHandler(MySearchUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
//	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a //randomly moving unit", kShiftDown, 'a');
//	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a //right-hand-rule unit", kControlDown, 1);

	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the //default map to be loaded.");
	InstallCommandLineHandler(MyCLHandler, "-environment", "-environment envType", "Selects the environment type to be used.");
	InstallCommandLineHandler(MyCLHandler, "-locs", "-locs filename", "Selects the file with unit start and end locations");
	
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
	switch(envType)
	{
		case 0:
		{
			if ((windowID < unitSims.size()) && (unitSims[windowID] == 0))
				return;

			if (viewport == 0)
			{
				//unitSims[windowID]->StepTime(1.0/30.0);
				unitSims[windowID]->StepTime(1.1);
			}
			unitSims[windowID]->OpenGLDraw(windowID);
			break;
		}
		case 1:
		{
			if ((windowID < wUnitSims.size()) && (wUnitSims[windowID] == 0))
				return;

			if (viewport == 0)
			{
				//wUnitSims[windowID]->StepTime(1.0/30.0);
				wUnitSims[windowID]->StepTime(1.1);
			}
			wUnitSims[windowID]->OpenGLDraw(windowID);		
			break;
		}
		default:
		{
			std::cout<<"Invalid environment\n";
			break;
		}	
	} // end switch	
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
				return 0;
			
	if(strcmp(argument[0],"-map")==0)
	{
		strncpy(gDefaultMap, argument[1], 1024);
	}
	else if(strcmp(argument[0],"-environment")==0)		
	{
		if(strcmp(argument[1],"AbsMapEnvironment")==0)
			envType = 0; 
		else if(strcmp(argument[1],"WeightedMap2DEnvironment")==0)
			envType = 1;
		else
		{
			std::cout<<"Invalid environment type\n";
			exit(0);
		}
	}
	else if(strcmp(argument[0],"-locs")==0)
	{
		strncpy(locsFile, argument[1],1024);
	}
	return 2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '\t':
		{
			if (mod != kShiftDown)
				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
			else
			{
				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
			}
			break;
		}
		case 'p': 
		{
			switch(envType)
			{
				case 0:
				{
					unitSims[windowID]->SetPaused(!unitSims[windowID]->GetPaused());
					break;
				}
				case 1:
				{
					wUnitSims[windowID]->SetPaused(!wUnitSims[windowID]->GetPaused());
					break;
				}
				default:
				{
					std::cout<<"Invalid environment\n";
					break;
				}
			}	
			break;
		}
		case 'o':
		{
			switch(envType)
			{
				case 0:
				{
					if (unitSims[windowID]->GetPaused())
					{
					unitSims[windowID]->SetPaused(false);
					unitSims[windowID]->StepTime(1.0/30.0);
					unitSims[windowID]->SetPaused(true);
					}
					break;
				}
				case 1:
				{
				if (wUnitSims[windowID]->GetPaused())
				{
					wUnitSims[windowID]->SetPaused(false);
					wUnitSims[windowID]->StepTime(1.0/30.0);
					wUnitSims[windowID]->SetPaused(true);
				}
					break;
				}
				default:
				{
					std::cout<<"Invalid environment\n";
					break;
				}
			}	

			
			break;
		}
		
//		case ']': absType = (absType+1)%3; break;
//		case '[': absType = (absType+4)%3; break;
//		case '{': unitSim->setPaused(true); unitSim->offsetDisplayTime(-0.5); break;
//		case '}': unitSim->offsetDisplayTime(0.5); break;
		default:
		{
			if (unitSims[windowID])
				unitSims[windowID]->GetEnvironment()->GetMapAbstraction()->ToggleDrawAbstraction(((mod == kControlDown)?10:0)+(key-'0'));
			break;
		}
	}

}

void MySearchUnitKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
// 	Map *m = unitSims[windowID]->GetEnvironment()->GetMap();
// 	xyLoc start, goal;
// 	start.x = random()%m->getMapWidth();
// 	start.y = random()%m->getMapHeight();
// 	goal.x = random()%m->getMapWidth();
// 	goal.y = random()%m->getMapHeight();
// 	
// 		double r,g,b;
// 	r = (double)rand() / RAND_MAX;
// 	g = (double)rand() / RAND_MAX;
// 	b = (double)rand() / RAND_MAX;
// 
// 	TemplateAStar<xyLoc, tDirection, WeightedMap2DEnvironment> *a = new TemplateAStar<xyLoc, tDirection, WeightedMap2DEnvironment>();
// 	
// 	GenericSearchUnit<xyLoc, tDirection, WeightedMap2DEnvironment> *u = new GenericSearchUnit<xyLoc, tDirection, WeightedMap2DEnvironment>(start,goal,a);
// 	
// 	unitSims[windowID]->AddUnit(u);
}

void MyPatrolKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
// 	int xStart=5, yStart=5, xGoal=50, yGoal=50;
// 	xyLoc start;
// 	start.x = xStart;
// 	start.y = yStart;
// 	
// 	xyLoc goal;
// 	goal.x = xGoal;
// 	goal.y = yGoal;
// 	
// 	xyLoc goalTwo;
// 	goalTwo.x = xStart;
// 	goalTwo.y = yGoal;
// 	
// 	double r,g,b;
// 	r = (double)rand() / RAND_MAX;
// 	g = (double)rand() / RAND_MAX;
// 	b = (double)rand() / RAND_MAX;
// 	
// 	//TemplateAStar<xyLoc, tDirection,AbsMapEnvironment>* a = new TemplateAStar<xyLoc, tDirection,AbsMapEnvironment>();
// 	
// 	TemplateAStar<xyLoc, tDirection, WeightedMap2DEnvironment> *a = new TemplateAStar<xyLoc, tDirection, WeightedMap2DEnvironment>();
	
	//GenericPatrolUnit<xyLoc, tDirection,AbsMapEnvironment> *u = new GenericPatrolUnit<xyLoc, tDirection,AbsMapEnvironment>(start, a,r,g,b);
//
	//GenericPatrolUnit<xyLoc, tDirection, WeightedMap2DEnvironment> *u = new GenericPatrolUnit<xyLoc, tDirection, WeightedMap2DEnvironment>(start, a,r,g,b); 	

//GenericSearchUnit<xyLoc, tDirection, WeightedMap2DEnvironment> *u = new GenericSearchUnit<xyLoc, tDirection, WeightedMap2DEnvironment>(start, goal, a); 	

//	u->AddPatrolLocation(goal);
	//u->AddPatrolLocation(goalTwo);
	
	//unitSims[windowID]->AddUnit(u);
	
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
