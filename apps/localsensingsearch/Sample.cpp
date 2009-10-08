/*
 * $Id: sample.cpp,v 1.23 2006/11/01 23:33:56 nathanst Exp $
 *
 *  sample.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Copyright 2005 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include "Common.h"
#include "Sample.h"
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include "Map2DEnvironment.h"
#include "RandomUnits.h"
#include "AStar.h"
#include "TemplateAStar.h"
#include "GraphEnvironment.h"
#include "LocalSensingUnit.h"
#include "LocalSensingUnit2.h"
#include "LRTAStarUnit.h"
#include "LSSLRTAStarUnit.h"
#include "ScenarioLoader.h"
#include "StatUtil.h"
#include "HeuristicLearningMeasure.h"

bool mouseTracking = false;
bool runningSearch1 = false;
bool runningSearch2 = false;
int px1, py1, px2, py2;
int absType = 0;
int mazeSize = 50;

std::vector<EpisodicSimulation<xyLoc, tDirection, MapEnvironment> *> unitSims;
TemplateAStar<xyLoc, tDirection, MapEnvironment> a1;
TemplateAStar<xyLoc, tDirection, MapEnvironment> a2;
MapEnvironment *ma1 = 0;
MapEnvironment *ma2 = 0;
GraphDistanceHeuristic *gdh = 0;
std::vector<xyLoc> path;
double stepsPerFrame = 1.0/30.0;
void RunScenario(char *name);
void RunScalingTest(int size);
void RunWorkMeasureTest();
void RunSTPTest();

void CreateSimulation(int id);

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
	if (gDefaultMap[0] == 0)
	{
		map = new Map(mazeSize, mazeSize);
		map->SetTerrainType(1, mazeSize-2,
							mazeSize-2, mazeSize-2, kOutOfBounds);
		map->SetTerrainType(mazeSize-2, 1,
							mazeSize-2, mazeSize-2, kOutOfBounds);

//		for (int x = 0; x < mazeSize-1; x+=2)
//		{
//			map->SetTerrainType(x, ((x/2)%2)?0:1, kOutOfBounds);
//		}
//		map->SetTerrainType(0, 2,
//							mazeSize-1, 2, kOutOfBounds);
		
		//MakeMaze(map, 1);
//		BuildRandomRoomMap(map, 16);
//		map->Scale(mazeSize, mazeSize);
	}
	else
		map = new Map(gDefaultMap);
	map->SetTileSet(kWinter);
	
	unitSims.resize(id+1);
	unitSims[id] = new EpisodicSimulation<xyLoc, tDirection, MapEnvironment>(new MapEnvironment(map, false));
	unitSims[id]->SetStepType(kRealTime);//kUniTime
	unitSims[id]->SetThinkingPenalty(0);
	unitSims[id]->GetStats()->EnablePrintOutput(true);
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
	InstallKeyboardHandler(MyDisplayHandler, "Increase Sim Speed", "Increase Sim Speed", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Decrease Sim Speed", "Decrease Sim Speed", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Clear units", "Clear all units from sim", kAnyModifier, '|');

	InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a randomly moving unit", kShiftDown, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, '1');

	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	InstallCommandLineHandler(MyCLHandler, "-size", "-size <integer>", "If size is set, we create a square maze with the x and y dimensions specified.");
	InstallCommandLineHandler(MyCLHandler, "-scenario", "-scenario <file>", "Load and run a scenario offline.");
	InstallCommandLineHandler(MyCLHandler, "-scaleTest", "-scaleTest <size>", "Run a scaling test with local minima <size>.");
	InstallCommandLineHandler(MyCLHandler, "-STPTest", "-STPTest", "Run a STP test.");
	
	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler);
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);

		delete ma1;
		ma1 = 0;
		delete ma2;
		ma2 = 0;
		delete gdh;
		gdh = 0;
		delete unitSims[windowID];
		unitSims[windowID] = 0;
		runningSearch1 = false;
		runningSearch2 = false;
		mouseTracking = false;
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		CreateSimulation(windowID);
		SetNumPorts(windowID, 1);
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (viewport == 0)
	{
		unitSims[windowID]->StepTime(stepsPerFrame);
	}
	
	if ((GetNumPorts(windowID) != 3) || (unitSims[windowID]->GetNumUnits() != 3))
		unitSims[windowID]->OpenGLDraw();
	else {
		unitSims[windowID]->GetEnvironment()->OpenGLDraw();
		unitSims[windowID]->OpenGLDraw(viewport);
	}
	
	if (mouseTracking)
	{
		glBegin(GL_LINES);
		glColor3f(1.0f, 0.0f, 0.0f);
		Map *m = unitSims[windowID]->GetEnvironment()->GetMap();
		GLdouble x, y, z, r;
		m->GetOpenGLCoord(px1, py1, x, y, z, r);
		glVertex3f(x, y, z-3*r);
		m->GetOpenGLCoord(px2, py2, x, y, z, r);
		glVertex3f(x, y, z-3*r);
		glEnd();
	}
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp( argument[0], "-map" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		strncpy(gDefaultMap, argument[1], 1024);
		return 2;
	}
	else if (strcmp( argument[0], "-size" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		mazeSize = atoi(argument[1]);
		assert( mazeSize > 0 );
		return 2;
	}
	else if (strcmp(argument[0], "-scenario") == 0)
	{
		RunScenario(argument[1]);
	}
	else if (strcmp(argument[0], "-scaleTest") == 0)
	{
		RunScalingTest(atoi(argument[1]));
	}
	else if (strcmp(argument[0], "-STPTest") == 0)
	{
		RunSTPTest();
	}
	return 2; //ignore typos
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '[': stepsPerFrame/=2.0; break; 
		case ']': stepsPerFrame*=2.0; break; 
		case '|':
			unitSims[windowID]->ClearAllUnits();
			break;
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
			else
				unitSims[windowID]->StepTime(1.0/30.0);
		}
			break;
		default:
			break;
	}
}

void MyRandomUnitKeyHandler(unsigned long windowID, tKeyboardModifier mod, char)
{
	if (mod == kShiftDown)
	{
		RunWorkMeasureTest();
		return;
	}
	Map *m = unitSims[windowID]->GetEnvironment()->GetMap();
	unitSims[windowID]->ClearAllUnits();

	int x1, y1, x2, y2;
	do {
		x2 = random()%m->GetMapWidth();
		y2 = random()%m->GetMapHeight();
		x1 = random()%m->GetMapWidth();
		y1 = random()%m->GetMapHeight();
	} while ((m->GetTerrainType(x1, y1) != kGround) || (m->GetTerrainType(x2, y2) != kGround));
	
	//xyLoc a(x1, y1), b(x2, y2);
	xyLoc a(0, 0), b(mazeSize-1, mazeSize-1);
	//xyLoc a(0, 0), b(mazeSize-1, 0);
	
	HeuristicLearningMeasure<xyLoc, tDirection, MapEnvironment> measure;
	measure.MeasureDifficultly(unitSims[windowID]->GetEnvironment(), a, b);
	
	LocalSensing::LocalSensingUnit2<xyLoc, tDirection, MapEnvironment> *u1 = new LocalSensing::LocalSensingUnit2<xyLoc, tDirection, MapEnvironment>(a, b);
	u1->SetWeight(1.0);
	u1->SetSpeed(0.02);
	unitSims[windowID]->AddUnit(u1);

	LRTAStarUnit<xyLoc, tDirection, MapEnvironment> *u2 = new LRTAStarUnit<xyLoc, tDirection, MapEnvironment>(a, b, new LRTAStar<xyLoc, tDirection, MapEnvironment>());
	u2->SetSpeed(0.02);
	unitSims[windowID]->AddUnit(u2);

	LSSLRTAStarUnit<xyLoc, tDirection, MapEnvironment> *u3 = new LSSLRTAStarUnit<xyLoc, tDirection, MapEnvironment>(a, b, new LSSLRTAStar<xyLoc, tDirection, MapEnvironment>(8));
	u3->SetSpeed(0.02);
	unitSims[windowID]->AddUnit(u3);
	
	unitSims[windowID]->GetStats()->AddFilter("trialDistanceMoved");
	unitSims[windowID]->GetStats()->AddFilter("TotalLearning");
	unitSims[windowID]->GetStats()->EnablePrintOutput(true);
	unitSims[windowID]->SetTrialLimit(50000);
}

void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	std::vector<graphState> thePath;
	MapEnvironment *env = unitSims[windowID]->GetEnvironment();
	Map *m = env->GetMap();
	Graph *g = GraphSearchConstants::GetGraph(m);

//	GraphDistanceHeuristic diffHeuristic(g);
//	diffHeuristic.UseSmartPlacement(true);
//	for (int x = 0; x < 20; x++)
//		diffHeuristic.AddHeuristic();

	GraphMapInconsistentHeuristic diffHeuristic(m, g);
	diffHeuristic.UseSmartPlacement(true);
	for (int x = 0; x < 20; x++)
		diffHeuristic.AddHeuristic();
	
	GraphEnvironment gEnv(g, &diffHeuristic);
	gEnv.SetDirected(true);

	diffHeuristic.SetMode(kRandom);
	diffHeuristic.SetNumUsedHeuristics(2);

	TemplateAStar<graphState, graphMove, GraphEnvironment> taNew;
	TemplateAStar<graphState, graphMove, GraphEnvironment> taOld;
	Timer t;
	
	for (int x = 0; x < 500; x++)
	{
		graphState s1, g1;
		do {
			s1 = g->GetRandomNode()->GetNum();
			g1 = g->GetRandomNode()->GetNum();
		} while (gEnv.HCost(s1, g1) < 100);
		double firstLength;

		taNew.SetUseBPMX(false);
		t.StartTimer();
		taNew.GetPath(&gEnv, s1, g1, thePath);
		t.EndTimer();
		printf("old: %lld nodes expanded. Path length %d / %f. Time: %f\nrate0: %lld %f\n", 
			   taNew.GetNodesExpanded(), (int)thePath.size(), gEnv.GetPathLength(thePath), t.GetElapsedTime(),
			   taNew.GetNodesExpanded(), taNew.GetNodesExpanded()/t.GetElapsedTime());
		firstLength = gEnv.GetPathLength(thePath);

		for (int y = 1; y < 5; y++)
		{
			taNew.SetUseBPMX(y);
			t.StartTimer();
			taNew.GetPath(&gEnv, s1, g1, thePath);
			t.EndTimer();
			printf("new: %lld nodes expanded. Path length %d / %f. Time: %f\nrate%d: %lld %f\n", 
				   taNew.GetNodesExpanded(), (int)thePath.size(), gEnv.GetPathLength(thePath), t.GetElapsedTime(),
				   y, taNew.GetNodesExpanded(), taNew.GetNodesExpanded()/t.GetElapsedTime());

			if (gEnv.GetPathLength(thePath) != firstLength)
				printf("\n\n\n!!!!!!!!!!!!!!!!!! IT FAILED!!!!!!!!!!!!!!!!!!!!!!!!\n\n\n\n");
		}
		printf("--\n");
	}	
	delete g;
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	mouseTracking = false;
	if (button == kRightButton)
	{
		switch (mType)
		{
			case kMouseDown:
				unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px1, py1);
				//printf("Mouse down at (%d, %d)\n", px1, py1);
				break;
			case kMouseDrag:
				mouseTracking = true;
				unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px2, py2);
				//printf("Mouse tracking at (%d, %d)\n", px2, py2);
				break;
			case kMouseUp:
			{
				if ((px1 == -1) || (px2 == -1))
					break;
				unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px2, py2);
				printf("Searching from (%d, %d) to (%d, %d)\n", px1, py1, px2, py2);
				
				if (ma1 == 0)
				{
					ma1 = new MapEnvironment(unitSims[windowID]->GetEnvironment()->GetMap());
					gdh = new GraphDistanceHeuristic(GraphSearchConstants::GetGraph(ma1->GetMap()));
					gdh->UseSmartPlacement(true);
					ma1->SetGraphHeuristic(gdh);
					for (int x = 0; x < 10; x++)
						gdh->AddHeuristic();
				}
				if (ma2 == 0)
					ma2 = new MapEnvironment(unitSims[windowID]->GetEnvironment()->GetMap());

				a1.SetStopAfterGoal(true);
				a2.SetStopAfterGoal(true);
				xyLoc s1;
				xyLoc g1;
				s1.x = px1; s1.y = py1;
				g1.x = px2; g1.y = py2;
					   
				a1.InitializeSearch(ma1, s1, g1, path);
				a2.InitializeSearch(ma2, s1, g1, path);
				runningSearch1 = true;
				runningSearch2 = true;
				
			}
			break;
		}
		return true;
	}
	return false;
}


typedef EpisodicSimulation<xyLoc, tDirection, MapEnvironment> EpSim;
void RunSingleTest(EpSim *es, const Experiment &e);

void RunScenario(char *name)
{
	ScenarioLoader *sl = new ScenarioLoader(name);
	printf("Loading map: %s\n", sl->GetNthExperiment(0).GetMapName());
	
	Map *map = new Map(sl->GetNthExperiment(0).GetMapName());
	map->Scale(sl->GetNthExperiment(0).GetXScale(),
			   sl->GetNthExperiment(0).GetYScale());
	EpSim *es = new EpSim(new MapEnvironment(map, false));
	es->SetStepType(kRealTime);
	es->SetThinkingPenalty(0);

	for (int x = 0; x < sl->GetNumExperiments(); x++)
	{
		printf("Experiment %d of %d\n", x+1, sl->GetNumExperiments());
		RunSingleTest(es, sl->GetNthExperiment(x));
	}
	exit(0);
}

void RunSingleTest(EpSim *es, const Experiment &e)
{
	es->ClearAllUnits();
	// add units
	es->GetStats()->ClearAllStats();
	es->GetStats()->AddFilter("trialDistanceMoved");
	es->GetStats()->EnablePrintOutput(false);
	xyLoc a(e.GetStartX(), e.GetStartY()), b(e.GetGoalX(), e.GetGoalY());

//	LocalSensing::LocalSensingUnit2<xyLoc, tDirection, MapEnvironment> *u1 = new LocalSensing::LocalSensingUnit2<xyLoc, tDirection, MapEnvironment>(a, b);
//	u1->SetSpeed(1.0);
//	es->AddUnit(u1); // go to goal and stop
	
	LRTAStarUnit<xyLoc, tDirection, MapEnvironment> *u1 = new LRTAStarUnit<xyLoc, tDirection, MapEnvironment>(a, b, new LRTAStar<xyLoc, tDirection, MapEnvironment>());
	u1->SetSpeed(1.0);
	es->AddUnit(u1);

	es->SetTrialLimit(100000);
	while (!es->Done())
	{
		es->StepTime(10.0);
	}
	statValue v;
	
	int which = es->GetStats()->FindNextStat("trialDistanceMoved", u1->GetName(), 0);
	es->GetStats()->LookupStat(which, v);
	//es->GetStats()->LookupStat("trialDistanceMoved", u1->GetName(), v);
	//es->GetStats()->SumStat("trialDistanceMoved", <#const char *owner#>, <#long value#>);
	printf("first\t%s\t%d\t%f\n", u1->GetName(), e.GetBucket(), v.fval);
	printf("sum\t%s\t%d\t%f\n", u1->GetName(), e.GetBucket(), SumStatEntries(es->GetStats(), "trialDistanceMoved", u1->GetName()));
}


void RunScalingTest(int size)
{
	Map *map = new Map(size, size);
	map->SetTerrainType(1, size-2,
						size-2, size-2, kOutOfBounds);
	map->SetTerrainType(size-2, 1,
						size-2, size-2, kOutOfBounds);

	EpSim *es = new EpSim(new MapEnvironment(map, false));
	es->SetStepType(kRealTime);
	es->SetThinkingPenalty(0);
	
	es->ClearAllUnits();
	// add units
	es->GetStats()->AddFilter("trialDistanceMoved");
	es->GetStats()->AddFilter("TotalLearning");
	es->GetStats()->EnablePrintOutput(false);
	xyLoc a(0, 0), b(size-1, size-1);
	
	HeuristicLearningMeasure<xyLoc, tDirection, MapEnvironment> measure;
	double requiredLearning = measure.MeasureDifficultly(es->GetEnvironment(), a, b);

	//	LocalSensing::LocalSensingUnit2<xyLoc, tDirection, MapEnvironment> *u1 = new LocalSensing::LocalSensingUnit2<xyLoc, tDirection, MapEnvironment>(a, b);
//	u1->SetSpeed(1.0);
//	es->AddUnit(u1); // go to goal and stop
	
//	LRTAStarUnit<xyLoc, tDirection, MapEnvironment> *u1 = new LRTAStarUnit<xyLoc, tDirection, MapEnvironment>(a, b, new LRTAStar<xyLoc, tDirection, MapEnvironment>());
//	u1->SetSpeed(1.0);
//	es->AddUnit(u1);

	LSSLRTAStarUnit<xyLoc, tDirection, MapEnvironment> *u1 = new LSSLRTAStarUnit<xyLoc, tDirection, MapEnvironment>(a, b, new LSSLRTAStar<xyLoc, tDirection, MapEnvironment>(100));
	u1->SetSpeed(1.0);
	es->AddUnit(u1);
	
	es->SetTrialLimit(100000);
	while (!es->Done())
	{
		es->StepTime(10.0);
	}
	statValue v;
	
	int which = es->GetStats()->FindNextStat("trialDistanceMoved", u1->GetName(), 0);
	es->GetStats()->LookupStat(which, v);
	printf("first\t%s\t%d\t%f\n", u1->GetName(), size, v.fval);
	printf("sum\t%s\t%d\t%f\n", u1->GetName(), size, SumStatEntries(es->GetStats(), "trialDistanceMoved", u1->GetName()));
	es->GetStats()->LookupStat("TotalLearning", u1->GetName(), v);
	printf("learning\t%s\t%f\t%f\t%f\n", u1->GetName(), v.fval, requiredLearning, v.fval/requiredLearning);
	exit(0);
}

#include "MNPuzzle.h"

void RunWorkMeasureTest()
{
	MNPuzzle *mnp = new MNPuzzle(4, 4);
	srandom(81);
	for (int x = 0; x < 50; x++)
	{
		HeuristicLearningMeasure<MNPuzzleState, slideDir, MNPuzzle> measure;
		MNPuzzleState s(4, 4);
		MNPuzzleState g(4, 4);
		std::vector<slideDir> acts;
		for (unsigned int y = 0; y < 100; y++)
		{
			mnp->GetActions(s, acts);
			mnp->ApplyAction(s, acts[random()%acts.size()]);
		}
		std::cout << "Start: " << s << std::endl << "Goal: " << g << std::endl;
		std::cout << "Heuristic: " << mnp->HCost(s, g) << std::endl;

		double requiredLearning = measure.MeasureDifficultly(mnp, s, g);
		std::cout << "Required learning: " << requiredLearning << std::endl;
	}
}


void RunSTPTest()
{
	typedef EpisodicSimulation<MNPuzzleState, slideDir, MNPuzzle> STPSim;
	
	MNPuzzle *mnp = new MNPuzzle(3, 3);

	srandom(81);

	STPSim *es = new STPSim(mnp);
	es->SetStepType(kRealTime);
	es->SetThinkingPenalty(0);
	
	// add units
	es->GetStats()->AddFilter("trialDistanceMoved");
	es->GetStats()->AddFilter("TotalLearning");
	es->GetStats()->EnablePrintOutput(false);
	
	for (int x = 0; x < 100; x++)
	{
		es->ClearAllUnits();

		MNPuzzleState s(3, 3);
		MNPuzzleState g(3, 3);
		std::vector<slideDir> acts;
		for (unsigned int y = 0; y < 100; y++)
		{
			mnp->GetActions(s, acts);
			mnp->ApplyAction(s, acts[random()%acts.size()]);
		}
		std::cout << "Start: " << s << std::endl << "Goal: " << g << std::endl;
		std::cout << "Heuristic: " << mnp->HCost(s, g) << std::endl;
		
		double requiredLearning;
		{
			HeuristicLearningMeasure<MNPuzzleState, slideDir, MNPuzzle> measure;
			requiredLearning = measure.MeasureDifficultly(mnp, s, g);
			std::cout << "Required learning: " << requiredLearning << std::endl;
		}
		
		//	LocalSensing::LocalSensingUnit2<xyLoc, tDirection, MapEnvironment> *u1 = new LocalSensing::LocalSensingUnit2<xyLoc, tDirection, MapEnvironment>(a, b);
		//	u1->SetSpeed(1.0);
		//	es->AddUnit(u1); // go to goal and stop
		
		LRTAStarUnit<MNPuzzleState, slideDir, MNPuzzle> *u1 = new LRTAStarUnit<MNPuzzleState, slideDir, MNPuzzle>(s, g, new LRTAStar<MNPuzzleState, slideDir, MNPuzzle>());
		u1->SetSpeed(1.0);
		es->AddUnit(u1);
		
		es->SetTrialLimit(100000);
		while (!es->Done())
		{
			es->StepTime(10.0);
		}

		statValue v;
		
		int which = es->GetStats()->FindNextStat("trialDistanceMoved", u1->GetName(), 0);
		es->GetStats()->LookupStat(which, v);
		printf("first\t%s\t%d\t%f\n", u1->GetName(), x, v.fval);
		printf("sum\t%s\t%d\t%f\n", u1->GetName(), x, SumStatEntries(es->GetStats(), "trialDistanceMoved", u1->GetName()));
		es->GetStats()->LookupStat("TotalLearning", u1->GetName(), v);
		printf("learning\t%s\t%f\t%f\t%f\n", u1->GetName(), v.fval, requiredLearning, v.fval/requiredLearning);
	}
	exit(0);
}
