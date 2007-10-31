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
#include "WeightedUnitGroup.h"
#include "AbsMapPatrolUnit.h"
#include "TemplateAStar.h"
#include "GenericSearchUnit.h"
#include "GenericPatrolUnit.h"
#include "WeightedMap2DEnvironment.h"
#include "ScenarioLoader.h"

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

bool runExperiment = false;
bool weighted = false;

char locsFile[1024];
char scenFileName[1024];
char outFileName[1024];
ScenarioLoader *sl;
std::vector<char*> names;

double weight = -1; 
double radius = -1;
double proportion = -1;
int numPatrols = 5;

bool useWindow = false;
double windowSize = 0; 
bool useTrim = false;
double trimRadius = 0; 
bool useLocal = false;
double localRadius = 0; 

bool weightedAstar = false;
double astarweight = 1; 

bool noWeighting = false;

WeightedUnitGroup<xyLoc,tDirection,AbsMapEnvironment> *wug;
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
// 	Map* map;
// 	map = new Map("../../maps/local/test_s2_ground.map");
// 	map->scale(64,64);
// 	map->save("../../maps/local/test_s2_64.map");
// 	
// 	delete map;
// 	
// 	map = new Map("../../maps/local/test_s3_ground.map");
// 	
// 	map->scale(64,64);
// 	map->save("../../maps/local/test_s3_64.map");
// 	
// 	exit(0);
// 	
	if(scenFileName[0] != 0)
	{
		RunScenario(id);
		//exit(0);
	}
		
		
		
	// Only show 1 copy of the map
/*	SetNumPorts(id, 1);
	
	Map *map;
	if (gDefaultMap[0] == 0)
	{
		map = new Map(60, 60);
	}
	else
		map = new Map(gDefaultMap);

	map->setTileSet(kWinterTile);
	
//	unitSims[id]->SetPaused(true);
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
	*/
	//RunExperiment(id);
}

void RunScenario(int id)
{
	Experiment e = sl->GetNthExperiment(0);	

	char mname[1024];
	char newname[1024] = "\0";
	e.GetMapName(mname);
	Map* map = new Map(mname);
	map->setTileSet(kWinterTile);
	
	unitSims.resize(id+1);
	env = new AbsMapEnvironment(new MapFlatAbstraction(map));
	unitSims[id] = new UnitSimulation<xyLoc, tDirection, AbsMapEnvironment>(env);
	unitSims[id]->SetStepType(kRealTime);
	unitSims[id]->SetThinkingPenalty(0);
//	unitSims[id]->SetPaused(true);
	
	wug = new WeightedUnitGroup<xyLoc, tDirection, AbsMapEnvironment>();
	
	if(weighted)
	{	
		if(weight != -1)
			wug->SetWeight(weight);
		if(proportion != -1)
			wug->SetProportion(proportion);
 		if(noWeighting)
			wug->SetNoWeighting(true);
	
		unitSims[id]->AddUnitGroup(wug);
	}
	if(useWindow)
	{
		wug->SetUseWindow(true);
		wug->SetWindowSize(windowSize);
	}
	if(useLocal)
	{
		wug->UseLocalWeights(true);
		wug->SetLocalWeightRadius(localRadius); 
	}

	char* name;
	
	int numExperiments = sl->GetNumExperiments();
	int i = 0; 
 	do
 	{
 		char num[8];
 		sprintf(num,"%d",i);
 		name = new char[256];
 		strcpy(name,"PatrolUnit");
 		strcat(name, num);
 		
 		names.push_back(name);

		e = sl->GetNthExperiment(i);
		e.GetMapName(newname);
		
		if(strcmp(mname, newname) != 0)
		{
			std::cout<<"Scenario file can only use one map\n";
			exit(1);
		}
		
		xyLoc start, goal;
		start.x = e.GetStartX();
		start.y = e.GetStartY();
		goal.x = e.GetGoalX();
		goal.y = e.GetGoalY();
		
		TemplateAStar<xyLoc, tDirection, AbsMapEnvironment> *alg = new TemplateAStar<xyLoc, tDirection, AbsMapEnvironment>();
		if(radius != -1)
			alg->SetRadius(radius);
		if(weightedAstar)
			alg->SetWeight(astarweight);
			
 		GenericPatrolUnit<xyLoc,tDirection,AbsMapEnvironment> *su = new GenericPatrolUnit<xyLoc,tDirection,AbsMapEnvironment>(start,alg);
		su->AddPatrolLocation(goal);
		su->SetNumPatrols(numPatrols);	
			
		su->SetName(name);	
		su->SetSpeed(1.0);
		//if(start.x < 16)	
			su->SetColor(1, 0, 0);
		//else
		//	su->SetColor(0, 1, 0);
		if(weighted) wug->AddUnit(su);
		if(useTrim)
		{
			su->SetTrimPath(true);
			su->SetTrimWindow(trimRadius);
		}
		unitSims[id]->AddUnit(su);
 

 		i++;

 	} while ( i < numExperiments);

	// Do timing

	if(runExperiment)
		{
			if(outFileName[0]==0)
				strncpy(outFileName, "defaultOut.txt",1024);
				
			std::ofstream outfile(outFileName, std::ios::out);
			//double beginTime = unitSims[id]->GetSimulationTime();
			//std::cout<<beginTime<<std::endl;
			
			
			double timestep = 1.5;
			double time = 0.0;
		
			outfile<<"Inputfile : "<<scenFileName<<std::endl
						<<std::endl
						<<"Speed 1.0 for all units; timeStep 1.5; kRealTime"
						<<std::endl<<std::endl
						<<"Weight    : "<<weight<<std::endl
						<<"Radius    : "<<radius<<std::endl
						<<"Proportion: "<<proportion<<std::endl
						<<"NumPatrols: "<<numPatrols<<std::endl<<std::endl;
									
		
			while(!(unitSims[id]->Done()))
			{
				unitSims[id]->StepTime(timestep);
				time += timestep;
				if(((int)time%15==0) && (wug->GetMembers().size() > 0))
				{
					std::cout<<wug->ComputeArrowMetric()<<std::endl;
				}
					
					
				if(time > 100000) 
				{
					std::cout<<"Ran out of time. Details in outfile "<<outFileName<<std::endl;
 					exit(1);
				}
				//std::cout<<time<<"\r";
			}
			//double endTime = unitSims[id]->GetSimulationTime();
			//std::cout<<"Simulation time "<<endTime-beginTime<<std::endl;
			
			outfile<<"Total simulation time "<<time<<std::endl<<std::endl; 
			
			// Get the "arrow metric" from the weighted unit group
// 			if(wug->GetMembers().size() > 0)
// 			{
// 				std::cout<<wug->ComputeArrowMetric()<<std::endl;
// 			}	
		
			// Collect statistics
			unitSims[id]->ClearAllUnits();
			PrintStatistics(id,outfile);			
			exit(0); 			
		} // end if(runExperiment)
	//unitSims[id]->ClearAllUnits();
	//PrintStatistics(id);
	SetNumPorts(id,1);	

}

void PrintStatistics(int id, std::ofstream &outfile)
{
	outfile<<"Unit NodesExpanded TotalDistanceTravelled NumDirectionChanges NumDirectionChangesColl NumFailedMoves\n";
	long nodes = 0; long dirChanged = 0; long failedMoves = 0;
	long dirChangedColl = 0; 
	long tnodes = 0; long tdirChanged = 0; long tfailedMoves = 0; 
	long tdirChangedColl = 0;
	double dist = 0; 
	double tdist = 0; 
	statValue v;
	for(unsigned int i=0; i<names.size(); i++)\
	{
		if (unitSims[id]->GetStats()->lookupStat("nodesExpanded",names[i],v))
		{
			nodes = v.lval;
			tnodes += nodes;
		}
		if(unitSims[id]->GetStats()->lookupStat("distanceTravelled",names[i],v))
		{
			dist = v.fval;
			tdist += dist;
		}
		if(unitSims[id]->GetStats()->lookupStat("directionChanges",names[i],v))
		{
			dirChanged = v.lval;
			tdirChanged += dirChanged;
		}
		if(unitSims[id]->GetStats()->lookupStat("failedMoves",names[i],v))
		{
			failedMoves = v.lval;
			tfailedMoves += failedMoves;
		}
		if(unitSims[id]->GetStats()->lookupStat("directionChangesCollision",names[i],v))
		{
			dirChangedColl = v.lval;
			tdirChangedColl += dirChangedColl;
		}
		outfile<<i<<" "<<nodes<<" "<<dist<<" "<<dirChanged<<" "
				 <<dirChangedColl<<" "<<failedMoves<<std::endl;
	}	
	outfile<<std::endl<<"Total "<<tnodes<<" "<<tdist<<" "
	<<tdirChanged<<" "<<tdirChangedColl<<" "<<tfailedMoves<<std::endl<<std::endl
	<<"Average "<<((double)tnodes / (double)(names.size()))
	<<" "<<(tdist / (double)(names.size()))
	<<" "<<(tdirChanged / (double)(names.size()))
	<<" "<<(tdirChangedColl / (double)(names.size()))
	<<" "<<(tfailedMoves / (double)(names.size()))<<std::endl;
}

void RunExperiment(int id)
{
		// First experiment code - on particular map (Aug 28 2007)
	if(strcmp(gDefaultMap, "../../maps/local/test_s1_ground.map")==0)
	{
		// Set up experiment
		srand(time(0));
		// Place units on LHS
		
		std::ifstream input(locsFile);
	
		WeightedUnitGroup<xyLoc,tDirection,AbsMapEnvironment> *wug = new WeightedUnitGroup<xyLoc, tDirection, AbsMapEnvironment>();
		if(weighted &&(envType==0))
		{
			unitSims[id]->AddUnitGroup(wug);
		}
		
		for(unsigned int i = 0; i<25; i++)
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
			su->SetNumPatrols(numPatrols);	
			
			su->SetSpeed(2.0);	
			su->SetColor(1, 0, 0);
			if(weighted)
				wug->AddUnit(su);
			unitSims[id]->AddUnit(su);
			break;
			}
			case 1:
			{
			TemplateAStar<xyLoc, tDirection, WeightedMap2DEnvironment> *alg = new TemplateAStar<xyLoc, tDirection, WeightedMap2DEnvironment>();
				
			//GenericSearchUnit<xyLoc, tDirection, WeightedMap2DEnvironment> *su = new GenericSearchUnit<xyLoc, tDirection, WeightedMap2DEnvironment>(start,goal,alg);
			
			GenericPatrolUnit<xyLoc,tDirection,WeightedMap2DEnvironment> *su = new GenericPatrolUnit<xyLoc,tDirection,WeightedMap2DEnvironment>(start,alg);
			su->AddPatrolLocation(goal);
			su->SetNumPatrols(numPatrols);
			
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
		
		for(unsigned int i=0; i<25; i++)
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
					su->SetNumPatrols(numPatrols);
					su->SetSpeed(1.0);	
					su->SetColor(0, 1, 0);
					if(weighted)
						wug->AddUnit(su);
					unitSims[id]->AddUnit(su);
					break;
				}
				case 1:
				{
				TemplateAStar<xyLoc, tDirection, WeightedMap2DEnvironment> *alg = new TemplateAStar<xyLoc, tDirection, WeightedMap2DEnvironment>();
				
				//GenericSearchUnit<xyLoc, tDirection, WeightedMap2DEnvironment> *su = new GenericSearchUnit<xyLoc, tDirection, WeightedMap2DEnvironment>(start,goal,alg);
				GenericPatrolUnit<xyLoc,tDirection,WeightedMap2DEnvironment> *su = new GenericPatrolUnit<xyLoc,tDirection,WeightedMap2DEnvironment>(start,alg);
				su->AddPatrolLocation(goal);
				su->SetNumPatrols(numPatrols);
				
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
		//unitSims[id]->SetPaused(true);
		input.close();
		if(runExperiment)
		{
			double timestep = 1.5;
			double time = 0.0;
		
			switch(envType)
 			{
				case 0:
					while(!(unitSims[id]->Done()))
					{
						unitSims[id]->StepTime(timestep);
						time += timestep;
						std::cout<<time<<"\r";
					}
					break;
				case 1:
					while(!(wUnitSims[id]->Done()))
					{
					wUnitSims[id]->StepTime(timestep);
					time += timestep;
					//std::cout<<time<<"\r";
					}
					break;
				default:
					std::cout<<"Invalid environment\n";
					break;
			}	
			std::cout<<"Simulation finished after "<<time<<" seconds.\n";  
 			exit(0);
		} // end if(runExperiment)
 	} // end if certain map file
 	
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
	
	InstallKeyboardHandler(MyDisplayHandler, "Draw next environment", "Cycles through each unit's environment",kAnyModifier,'n');
//	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase //abstraction type", kAnyModifier, ']');
//	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease //abstraction type", kAnyModifier, '[');

	InstallKeyboardHandler(MyPatrolKeyHandler, "Patrol Unit", "Deploy patrol unit that patrols between two locations on the map", kNoModifier, 'd');
	InstallKeyboardHandler(MySearchUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
//	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a //randomly moving unit", kShiftDown, 'a');
//	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a //right-hand-rule unit", kControlDown, 1);

	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the //default map to be loaded.");
	InstallCommandLineHandler(MyCLHandler, "-environment", "-environment envType", "Selects the environment type to be used.");
	InstallCommandLineHandler(MyCLHandler, "-locs", "-locs filename", "Selects the file with unit start and end locations");
	
	InstallCommandLineHandler(MyCLHandler, "-exp", "-exp", "Runs the experiment");
	InstallCommandLineHandler(MyCLHandler, "-weighted", "-weighted", "Uses a WeightedUnitGroup");
	InstallCommandLineHandler(MyCLHandler, "-scen", "-scen scenarioFile", "Runs the experiments in the scenariofile");
	
	InstallCommandLineHandler(MyCLHandler, "-weight", "-weight w", "Environment is weighted by w");
	InstallCommandLineHandler(MyCLHandler, "-radius", "-radius r", "TemplateAStar only uses occupancy interface within a heuristic distance of r");
	InstallCommandLineHandler(MyCLHandler, "-prop", "-prop p", "Set the proportion of old direction to be used in the weighted environment");
	InstallCommandLineHandler(MyCLHandler, "-patrols", "-patrols p", "Set the number of times to patrol");
	InstallCommandLineHandler(MyCLHandler, "-outfile", "-outfile filename", "Set the filename for output.");
	
	InstallCommandLineHandler(MyCLHandler, "-window", "-window wSize", "Only use weights within a window of size wSize");
	InstallCommandLineHandler(MyCLHandler, "-trim", "-trim radius", "Trim the path to radius");
	InstallCommandLineHandler(MyCLHandler, "-local", "-local radius", "Each unit keeps a local copy of weight and updates within a distance of radius");
	
	InstallCommandLineHandler(MyCLHandler, "-astarweight", "-astarweight weight", "Use weighted A*");
	InstallCommandLineHandler(MyCLHandler, "-noweighting", "-noweighting", "Draw directions but don't do weighting");
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
				unitSims[windowID]->StepTime(1.5);
// 				if(unitSims[windowID]->Done())
// 					std::cout<<"DONE\n";
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
				wUnitSims[windowID]->StepTime(1.5);
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
	//if (maxNumArgs <= 1)
	//			return 0;
	//FIX THIS --> this could break if bad input is given
			
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
	else if(strcmp(argument[0],"-exp")==0)
	{
		runExperiment = true;
		return 1;
	}
	else if(strcmp(argument[0],"-weighted")==0)
	{
		weighted = true;
		return 1;
	}
	else if(strcmp(argument[0],"-scen")==0)
	{
		strncpy(scenFileName, argument[1],1024);
		sl = new ScenarioLoader(scenFileName);
		if(sl->GetNumExperiments()==0)
		{
			std::cout<<"No experiments in this scenario file or invalid file.\n";
			exit(1);
		}
	}
	else if(strcmp(argument[0],"-weight")==0)
	{
		weight = atof(argument[1]);
		if(weight<0)
		{
			std::cout<<"Invalid weight. Must be non-negative\n";
			exit(1);
		}
	}
	else if(strcmp(argument[0],"-radius")==0)
	{
		radius = atof(argument[1]);
		if(radius < 0)
		{	
			std::cout<<"Invalid radius. Must be non-negative\n";
			exit(1);
		}	
	}
	else if(strcmp(argument[0],"-prop")==0)
	{
		proportion = atof(argument[1]);
		if((proportion < 0) || (proportion > 1))
		{
			std::cout<<"Invalid proportion. Must be between 0 and 1\n";
			exit(1);
		}
	}
	else if(strcmp(argument[0],"-patrols")==0)
	{
		numPatrols = atoi(argument[1]);
	}
	else if(strcmp(argument[0],"-outfile")==0)
	{
		strncpy(outFileName, argument[1],1024);
	}
	else if(strcmp(argument[0],"-window")==0)
	{
		useWindow = true;
		windowSize = atof(argument[1]);
	}
	else if(strcmp(argument[0],"-trim")==0)
	{
		useTrim = true;
		trimRadius = atof(argument[1]);
	}
	else if(strcmp(argument[0],"-local")==0)
	{
		useLocal = true;
		localRadius = atof(argument[1]);
	}
	else if(strcmp(argument[0],"-astarweight")==0)
	{
		weightedAstar = true;
		astarweight = atof(argument[1]);
	}	
	else if(strcmp(argument[0],"-noweighting")==0)
	{
		noWeighting = true;
		weighted = true;
		return 1;
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
		case 'n':
		{
			if(useLocal)
				wug->DrawNextEnvironment();
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





// TESTING CODE - probably not useful anymore; just backup

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
	
	u->AddPatrolLocation(goal);
	u->AddPatrolLocation(goalTwo);
	u->AddPatrolLocation(goalThree);
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
	

