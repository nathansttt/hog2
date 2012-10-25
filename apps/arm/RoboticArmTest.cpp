/*
 *  RoboticArmTest.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/15/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
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
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include <deque>
#include "IDAStar.h"
#include "TemplateAStar.h"
#include "RoboticArmTest.h"
#include "FrontierBFS.h"

#include "RoboticArm.h"


//#define RUN_RANDOM_TESTS
//#define HEURISTIC_TABLES
//#define MAX_DIST_HEUR_TABLES
#define FIXED_RANDOM_NUMBER_SEED 7


const int numArms = 4;
RoboticArm *r = 0;
armAngles config;
armAngles goal;
bool validSearch = false;
unsigned int pathLoc = 0;
std::vector<armAngles> ourPath;
TemplateAStar<armAngles, armRotations, RoboticArm> astar;
float totalTime;
ArmToArmHeuristic *aa = 0;
void TestArms();
void TestArms2(bool h);
void BuildTipTables();
void Build4ArmDH();

bool mouseTracking;
int px1, py1, px2, py2;
int absType = 0;

//std::vector<PuzzleSimulation *> unitSims;

int main(int argc, char* argv[])
{
	InstallHandlers();
	setlinebuf( stdout );
	RunHOGGUI(argc, argv);
}


/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
void CreateSimulation(int)
{
	if (r != 0)
		delete r;
	r = new RoboticArm(numArms, 1.0/(double)numArms);

	r->AddObstacle(line2d(recVec(-0.32, -0.30, 0), recVec(0.32, -0.30, 0)));
	r->AddObstacle(line2d(recVec(0.30, 0.32, 0), recVec(0.30, -0.32, 0)));
	r->AddObstacle(line2d(recVec(0.28, 0.30, 0), recVec(0.32, 0.30, 0)));
	r->AddObstacle(line2d(recVec(-0.30, -0.32, 0), recVec(-0.30, 0.32, 0)));
	r->AddObstacle(line2d(recVec(-0.28, 0.30, 0), recVec(-0.32, 0.30, 0)));

#if 0
	r->AddObstacle(line2d(recVec(0.50, 0, 0), recVec(0.52, 0.02, 0)));
	r->AddObstacle(line2d(recVec(0.52, 0.02, 0), recVec(0.54, 0, 0)));
	r->AddObstacle(line2d(recVec(0.54, 0, 0), recVec(0.52, -0.02, 0)));
	r->AddObstacle(line2d(recVec(0.52, -0.02, 0), recVec(0.50, 0, 0)));
#endif

	config.SetNumArms( numArms );
	for (int x = 0; x < numArms; x++)
		config.SetAngle( x, 512 );
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	//InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kNoModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');
	
	InstallKeyboardHandler(MyKeyHandler, "0-9", "select segment", kNoModifier, '0', '9');
	InstallKeyboardHandler(MyKeyHandler, "Rotate segment", "rotate segment CW", kNoModifier, 'a');
	InstallKeyboardHandler(MyKeyHandler, "Rotate segment", "rotate segment CCW", kNoModifier, 's');
	InstallKeyboardHandler(MyKeyHandler, "Build Heuristic", "Build differential heuristic", kNoModifier, 'b');
	InstallKeyboardHandler(MyKeyHandler, "Test Heuristic", "Build & test differential heuristic", kNoModifier, 't');
	
	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	
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
		SetNumPorts(windowID, 1);
	}
}

void MyFrameHandler(unsigned long , unsigned int , void *)
{
	static int currFrame = 0;
	currFrame++;

	r->OpenGLDraw();
	if (!validSearch)
	{
		if (ourPath.size() == 0)
			r->OpenGLDraw(config);
		else
			r->OpenGLDraw(ourPath[(pathLoc++)%ourPath.size()]);
	}
	else {
		Timer t;
		t.StartTimer();
		for (int x = 0; x < 1000; x++)
		{
			if (astar.DoSingleSearchStep(ourPath))
			{
				validSearch = false;
				if (ourPath.size() > 0)
				{
					totalTime += t.EndTimer();
					printf("Done! %lld nodes expanded; %1.1f nodes/sec\n",
						   astar.GetNodesExpanded(), (double)astar.GetNodesExpanded()/totalTime);
					config = ourPath.back();
					pathLoc = 0;
					break;
				}
			}
		}
		totalTime += t.EndTimer();
		if ((currFrame%500) == 499)
			printf("Currently generating %1.1f nodes/sec\n", (double)astar.GetNodesExpanded()/totalTime);
		armAngles next = astar.CheckNextNode();
		r->OpenGLDraw(next);
		r->OpenGLDraw(goal);
		//astar.GetPath(r, config, goal, ourPath);
	}
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	CreateSimulation(0);
	TestArms2(maxNumArgs==1);
	//Build4ArmDH();
	//BuildTipTables();
	exit(0);
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
		case 'p': //unitSims[windowID]->SetPaused(!unitSims[windowID]->GetPaused()); break;
		case 'o':
//			if (unitSims[windowID]->GetPaused())
//			{
//				unitSims[windowID]->SetPaused(false);
//				unitSims[windowID]->StepTime(1.0/30.0);
//				unitSims[windowID]->SetPaused(true);
//			}
			break;
		case ']': absType = (absType+1)%3; break;
		case '[': absType = (absType+4)%3; break;
			//		case '{': unitSim->setPaused(true); unitSim->offsetDisplayTime(-0.5); break;
			//		case '}': unitSim->offsetDisplayTime(0.5); break;
		default:
			break;
	}
}


void MyKeyHandler(unsigned long, tKeyboardModifier, char key)
{
	static int which = 0;
	if ((key >= '0') && (key <= '9'))
	{
		which = key-'0';
		return;
	}
	
	if (key == 'a')
	{
		std::vector<armRotations> actions;

		armRotations rot;
		rot.SetRotation(which, kRotateCW);
		r->GetActions(config, actions);
		for (unsigned int x = 0; x < actions.size(); x++)
			if (rot == actions[x])
			{
				r->ApplyAction(config, rot);
				ourPath.resize(0);
			}
	}
	if (key == 's')
	{
		std::vector<armRotations> actions;
		
		armRotations rot;
		rot.SetRotation(which, kRotateCCW);
		r->GetActions(config, actions);
		for (unsigned int x = 0; x < actions.size(); x++)
			if (rot == actions[x])
			{
				r->ApplyAction(config, rot);
				ourPath.resize(0);
			}
	}
	
	if (key == 't')
	{
		BuildTipTables();
		//TestArms();
		TestArms2(true);
		//TestArms2();
	}
	
	if (key == 'b')
	{
		if (aa == 0)
		{
			aa = new ArmToArmHeuristic(r, config);
			r->AddHeuristic(aa);
		}
		else
			aa->AddDiffTable();
	}
}

bool MyClickHandler(unsigned long , int x, int y, point3d loc, tButtonType whichButton, tMouseEventType mouseEvent)
{
	printf("Hit %d/%d (%f, %f)\n", x, y, loc.x, loc.y);
	if ((mouseEvent != kMouseDown) || (whichButton != kRightButton))
		return false;
	goal.SetGoal(loc.x, loc.y);
	if (aa)
	{
		const std::vector<armAngles> &pos = aa->GetTipPositions(loc.x, loc.y);
		if (pos.size() > 0)
		{
			goal = pos[0];
			validSearch = astar.InitializeSearch(r, goal, config, ourPath);
			for (unsigned int t = 1; t < pos.size(); t++)
			{
				goal = pos[t];
				astar.AddAdditionalStartState(goal);
			}
		}
	}
	else
		validSearch = astar.InitializeSearch(r, config, goal, ourPath);

	if (validSearch)
	{
		std::cout << "Starting search between: " << config << " and " << goal << std::endl;
		totalTime = 0;
	}
//	static point3d oldloc(0, 0, 0);
//
//	if (oldloc.x != 0)
//	{
//		r->AddObstacle(line2d(recVec(oldloc.x, oldloc.y, 0), recVec(loc.x, loc.y, 0)));
//		oldloc = loc;
//		return true;
//	}
//	oldloc = loc;
//	return false;

	return true;
}

#if 0
void AddToMinHeap( std::vector<armAngles> &heap, armAngles &arm,
		   float *distances )
{
	unsigned i;

	// need to increase size, although we don't actually care
	// about the new content yet
	heap.push_back( arm );
	for( i = heap.size(); i > 1; i >>= 1 ) {
		if( ArmDistance( arm, distances )
		    >= ArmDistance( heap[ ( i >> 1 ) - 1 ], distances ) ) {
			break;
		}
		heap[ i - 1 ] = heap[ ( i >> 1 ) - 1 ];
	}
	heap[ i - 1 ] = arm;
}

armAngles GetFromMinHeap( std::vector<armAngles> &heap, float *distances )
{
	unsigned c; // child index
	armAngles ret;

	ret = heap[ 1 - 1 ];

	// item heap.size() starts as the root (1) but it
	// must be pushed down if it is too large
	c = 1 << 1;
	while( c <= heap.size() - 1 ) {
		if( c + 1 <= heap.size() - 1
		    && ArmDistance( heap[ c - 1 ], distances )
		    > ArmDistance( heap[ c + 1 - 1 ], distances ) ) {
			// child c is bigger than child c+1, so use c+1
			++c;
		}
		if( ArmDistance( heap[ heap.size() - 1 ], distances )
		    <= ArmDistance( heap[ c - 1 ], distances ) ) {
			// new root <= than child, so it stops moving down
			break;
		}

		// root is pushed down past child, so child
		// item is now the parent
		heap[ ( c >> 1 ) - 1 ] = heap[ c - 1 ];

		// root has a new potential location, get new child
		c <<= 1;
	}

	// we found the child of our final location, put item
	// in its place
	heap[ ( c >> 1 ) - 1 ] = heap[ heap.size() - 1 ];
	heap.pop_back();

	return ret;
}
#endif

void TestArms()
{
	assert(aa == 0);
	aa = new ArmToArmHeuristic(r, config, true);
	r->AddHeuristic(aa);
	
	std::vector<armAngles> starts;
	std::vector<armAngles> goals;
	
	// first, get lots of good problems
	printf("Generating problems\n");
	while (starts.size() < 500)
	{
		if ((starts.size()%500) == 0)
			printf("Generating problem %d\n", (int)starts.size());
		double x, y;
		x = random()%10000;
		x = 2*x/10000-1;
		y = random()%10000;
		y = 2*y/10000-1;
		//printf("Trying config from (%f, %f) for start\n", x, y);
		const std::vector<armAngles> &pos = aa->GetTipPositions(x, y);
		if (pos.size() > 0)
		{
			starts.push_back(pos[random()%pos.size()]);
		}		
		else
			continue;

		while (1)
		{
			x = random()%10000;
			x = 2*x/10000-1;
			y = random()%10000;
			y = 2*y/10000-1;
			//printf("Trying (%f, %f) for goal\n", x, y);
			const std::vector<armAngles> &pos2 = aa->GetTipPositions(x, y);
			if (pos2.size() > 0)
			{
				armAngles theGoal;
				theGoal.SetGoal(x, y);
				goals.push_back(theGoal);
				break;
			}
		}
	}
	printf("Done generating problems\n");
	
//	if (aa == 0)
//	{
//		aa = new ArmToArmHeuristic(r, config);
//		r->AddHeuristic(aa);
//	}
//	else
//		aa->AddDiffTable();

	double fTotalTime;
	double totalNodes;
	double totalHvalue;
	
//	aa->AddDiffTable();
//	aa->AddDiffTable();
//	aa->AddDiffTable();
	for (int total = 0; total <= -1; total++)
	{
		printf("Solving with %d heuristics\n", total);
		fTotalTime = 0;
		totalNodes = 0;
		totalHvalue = 0;
		for (unsigned int x = 0; x < starts.size(); x++)
		{
			armAngles theGoal;
			double x1, y1;
			goals[x].GetGoal(x1, y1);
			const std::vector<armAngles> &pos = aa->GetTipPositions(x1, y1);
			theGoal = pos[0];
			double localHvalue = r->HCost(starts[x], theGoal);
			validSearch = astar.InitializeSearch(r, theGoal, starts[x], ourPath);
			for (unsigned int t = 1; t < pos.size(); t++)
			{
				theGoal = pos[t];
				astar.AddAdditionalStartState(theGoal);
				localHvalue = min(localHvalue, r->HCost(starts[x], theGoal));
			}
			
			Timer t;
			t.StartTimer();
//			while (!astar.DoSingleSearchStep(ourPath))
//			{}
			totalHvalue += localHvalue;
			fTotalTime += t.EndTimer();
			totalNodes += astar.GetNodesExpanded();
			printf("%d\t%lld\t%f\t%f\n", x, astar.GetNodesExpanded(), t.GetElapsedTime(), localHvalue);
		}
		fTotalTime /= starts.size();
		totalNodes /= starts.size();
		totalHvalue /= starts.size();
		printf("time\t%f\tnodes\t%f\thcost\t%f\n", fTotalTime, totalNodes, totalHvalue);
		aa->AddDiffTable();
	}

	printf("Solving with no heuristics the normal way\n");
	fTotalTime = 0;
	totalNodes = 0;
	for (unsigned int x = 0; x < starts.size(); x++)
	{
		validSearch = astar.InitializeSearch(r, starts[x], goals[x], ourPath);
		
		Timer t;
		t.StartTimer();
		while (!astar.DoSingleSearchStep(ourPath))
		{}
		fTotalTime += t.EndTimer();
		totalHvalue += r->HCost(starts[x], goals[x]);
		totalNodes += astar.GetNodesExpanded();
		printf("%d\t%lld\t%f\t%f\n", x, astar.GetNodesExpanded(), t.GetElapsedTime(), r->HCost(starts[x], goals[x]));
	}
	fTotalTime /= starts.size();
	totalNodes /= starts.size();
	totalHvalue /= starts.size();
	printf("time\t%f\tnodes\t%f\n", fTotalTime, totalNodes);
	exit(0);
}

void TestArms2(bool h)
{
	if (h) printf("using heuristic\n");
	std::vector<armAngles> starts;
	std::vector<armAngles> goals;
	std::vector<armAngles> succ;
	srandom(101);
	for (int x = 0; x < 100; x++)
	{
		std::cout << "Building start " << x << std::endl;
		armAngles a;
		a.SetNumArms(numArms);
		//for (int y = 0; y < numArms; y++)
		//	a.SetAngle( y, 512 );
		//32, 1022, 2, 1022
		if (x%1)
		{
			a.SetAngle(0, 32);
			a.SetAngle(1, 1022);
			a.SetAngle(2, 2);
			a.SetAngle(3, 1022);
		}
		else {
			//988, 328, 1022, 444
			a.SetAngle(0, 988);
			a.SetAngle(1, 328);
			a.SetAngle(2, 1022);
			a.SetAngle(3, 444);
		}
		for (int y = 0; y < 35000; y++)
		{
			r->GetSuccessors(a, succ);
			a = succ[random()%succ.size()];
		}
		starts.push_back(a);
	}
	for (int x = 0; x < 100; x++)
	{
		std::cout << "Building goal " << x << std::endl;
		armAngles a;
		a.SetNumArms(numArms);
		for (int y = 0; y < numArms; y++)
			a.SetAngle( y, 512 );
		if (numArms == 3)
		{
			for (int y = 0; y < 35000; y++)
			{
				if (x%2)
				{
					a.SetAngle(0, 694);
					a.SetAngle(1, 1022);
					a.SetAngle(2, 154);
				}
				else {
					a.SetAngle(0, 330);
				a.SetAngle(1, 2);
				a.SetAngle(2, 870);
				}
			}
		}
		else {
			for (int y = 0; y < numArms; y++)
				a.SetAngle( y, 512 );
		}
		for (int y = 0; y < 35000; y++)
		{
			r->GetSuccessors(a, succ);
			a = succ[random()%succ.size()];
		}
		goals.push_back(a);
	}
	if (h)
	{
		std::cout << "Loading heuristics" << std::endl;
		ArmToArmCompressedHeuristic *a1 = new ArmToArmCompressedHeuristic(r, "4-arm_far3.diff");
		//		ArmToArmCompressedHeuristic *a2 = new ArmToArmCompressedHeuristic(r, "4-arm_far2.diff");
		r->AddHeuristic(a1);
		//	r->AddHeuristic(a2);
	}
	std::vector<int> use;
	use.push_back(1);
	use.push_back(2);
	use.push_back(3);

	use.push_back(5);
	use.push_back(6);
	use.push_back(7);
	use.push_back(8);
	use.push_back(9);
	use.push_back(10);
	use.push_back(11);


	use.push_back(14);
	use.push_back(15);
	use.push_back(16);
	use.push_back(17);
	use.push_back(18);
	use.push_back(20);
	use.push_back(21);
	use.push_back(22);
	use.push_back(23);
	use.push_back(24);
	use.push_back(25);
	use.push_back(27);
	use.push_back(28);
	use.push_back(29);
	use.push_back(30);
	use.push_back(31);
	use.push_back(32);
	use.push_back(33);
	use.push_back(34);
	use.push_back(36);
	use.push_back(37);
	use.push_back(38);
	use.push_back(39);
	use.push_back(41);
	use.push_back(42);
	use.push_back(43);
	use.push_back(44);
	use.push_back(46);
	use.push_back(48);
	use.push_back(53);
	use.push_back(54);
	use.push_back(55);
	use.push_back(56);
	use.push_back(57);
	use.push_back(58);
	use.push_back(59);
	use.push_back(61);
	use.push_back(62);
	use.push_back(63);
	use.push_back(64);
	use.push_back(65);
	use.push_back(66);
	use.push_back(67);
	use.push_back(70);
	use.push_back(71);
	use.push_back(72);
	use.push_back(73);
	use.push_back(75);
	use.push_back(76);
	use.push_back(77);
	use.push_back(78);
	use.push_back(79);
	use.push_back(81);
	use.push_back(82);
	use.push_back(83);
	use.push_back(86);
	use.push_back(87);
	use.push_back(88);
	use.push_back(89);
	use.push_back(90);
	use.push_back(91);
	use.push_back(92);
	use.push_back(93);
	use.push_back(94);
	use.push_back(95);
	use.push_back(96);
	use.push_back(97);
	use.push_back(99);


	astar.SetUseBPMX(1);
	printf("%d starts; %d goals\n", (int)starts.size(), (int)goals.size());
	for (unsigned int t = 0; t < use.size(); t+=1)
	{
		int x = use[t];
		config = starts[x];
		goal = goals[x];
		std::cout << "Searching " << starts[x] << " to " << goals[x] << std::endl;
		astar.InitializeSearch(r, starts[x], goals[x], ourPath);
		
		Timer tmr;
		tmr.StartTimer();
		int cnt = 1;
		while (!astar.DoSingleSearchStep(ourPath))

		{ if (((++cnt%100) == 0) && (tmr.EndTimer() > 300)) break; }

		tmr.EndTimer();
//		totalHvalue += r->HCost(starts[x], goals[x]);
//		totalNodes += astar.GetNodesExpanded();
		printf("%d\t%lld\t%lld\t%f\t%f\t%1.0f\n", x, astar.GetNodesExpanded(), astar.GetUniqueNodesExpanded(),
			   tmr.GetElapsedTime(), r->HCost(starts[x], goals[x]), r->GetPathLength(ourPath));
	}
	
}

void Build4ArmDH()
{
	//	return;

	std::vector<int> reduction, offset1;
	reduction.push_back(3);reduction.push_back(3);reduction.push_back(2);reduction.push_back(2);
	offset1.push_back(0);offset1.push_back(0);offset1.push_back(0);offset1.push_back(0);
	//offset2.push_back(0);offset2.push_back(0);offset2.push_back(1);
	ArmToArmCompressedHeuristic *aah = new ArmToArmCompressedHeuristic(r, reduction, offset1);
	
	FrontierBFS<armAngles, armRotations> fbfs;
	printf("Performing frontier BFS!\n");
	std::cout << "Starting from " << config << std::endl;
//	std::vector<std::vector<armAngles> > cache;
	
	armAngles tmp = config;
	//988, 328, 1022, 444
	//32, 1022, 2, 1022
	tmp.SetAngle(0, 32); 
	tmp.SetAngle(1, 1022);
	tmp.SetAngle(2, 2);
	tmp.SetAngle(3, 1022);
	std::cout << "Adding heuristic from: " << tmp << std::endl;
	aah->BuildHeuristic(tmp);
	aah->Save("4-arm_far3.diff");
}

void WriteCache(int index, std::vector<armAngles> &values);
void BuildDHTables(std::vector<int> reduction, const char *baseName);

void BuildTipTables()
{
	if (r == 0)
		CreateSimulation(0);
	std::vector<int> reduction;

	reduction.clear();
	reduction.push_back(1);reduction.push_back(1);reduction.push_back(1);
	BuildDHTables(reduction, "3-arm_1V");

	reduction.clear();
	reduction.push_back(1);reduction.push_back(1);reduction.push_back(2);
	BuildDHTables(reduction, "3-arm_2V");
	
	reduction.clear();
	reduction.push_back(1);reduction.push_back(2);reduction.push_back(2);
	BuildDHTables(reduction, "3-arm_4V");

	reduction.clear();
	reduction.push_back(2);reduction.push_back(2);reduction.push_back(2);
	BuildDHTables(reduction, "3-arm_8V");

	reduction.clear();
	reduction.push_back(3);reduction.push_back(2);reduction.push_back(2);
	BuildDHTables(reduction, "3-arm_16V");

	reduction.clear();
	reduction.push_back(3);reduction.push_back(3);reduction.push_back(2);
	BuildDHTables(reduction, "3-arm_32V");

	reduction.clear();
	reduction.push_back(3);reduction.push_back(3);reduction.push_back(3);
	BuildDHTables(reduction, "3-arm_64V");
}

void BuildDHTables(std::vector<int> reduction, const char *baseName)
{
	std::vector<int> offset1, offset2, offset3, offset4;
	//reduction.push_back(3);reduction.push_back(2);reduction.push_back(2);
	offset1.push_back(0);offset1.push_back(0);offset1.push_back(0);
	offset2.push_back(0);offset2.push_back(0);offset2.push_back(1);
	offset3.push_back(0);offset3.push_back(1);offset3.push_back(0);
	offset4.push_back(0);offset4.push_back(1);offset4.push_back(1);

	ArmToArmCompressedHeuristic *aah = new ArmToArmCompressedHeuristic(r, reduction, offset1);
	ArmToArmCompressedHeuristic *aah1 = new ArmToArmCompressedHeuristic(r, reduction, offset2);
	ArmToArmCompressedHeuristic *aah2 = new ArmToArmCompressedHeuristic(r, reduction, offset3);
	ArmToArmCompressedHeuristic *aah3 = new ArmToArmCompressedHeuristic(r, reduction, offset4);
	Timer t;
	t.StartTimer();
	
	FrontierBFS<armAngles, armRotations> fbfs;

	std::vector<armAngles> init, far;
	armAngles tmp = config;
	init.push_back(tmp);
	std::cout << "Building heuristic 1 from: " << std::endl;
	far.push_back(aah->BuildHeuristic(init)); // initial finding of far state; far[0] = far1

	std::cout << "Building heuristic 2: " << std::endl;
	far.push_back(aah1->BuildHeuristic(far)); // first real heuristic; far[1] = far2

	std::cout << "Building heuristic 3: " << std::endl;
	far.push_back(aah2->BuildHeuristic(far)); // first real heuristic; far[1] = far2
	
	std::cout << "Building heuristic 4: " << std::endl;
	far.push_back(aah3->BuildHeuristic(far)); // first real heuristic; far[1] = far2

	char name[255];
	sprintf(name, "%s_a.diff", baseName);
	aah->Save(name);
	sprintf(name, "%s_b.diff", baseName);
	aah1->Save(name);
	sprintf(name, "%s_c.diff", baseName);
	aah2->Save(name);
	sprintf(name, "%s_d.diff", baseName);
	aah3->Save(name);
	delete aah;
	delete aah1;
	delete aah2;
	delete aah3;
}

void WriteCache(int index, std::vector<armAngles> &values)
{
	return;
	char filename[255];
	sprintf(filename, "%d.tipIndex", index);
	FILE *f = fopen(filename, "a+");
	if (!f) assert(!"Couldn't open file!");
	for (unsigned int x = 0; x < values.size(); x++)
	{
		for (unsigned int y = 0; y < values[x].GetNumArms(); y++)
			fprintf(f, "%d ", values[x].GetAngle(y));
		fprintf(f, "\n");
	}
	fclose(f);
	values.resize(0);
}
