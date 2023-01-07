/*
 *  hog2
 *
 *  Created by Nathan Sturtevant on 12/13/22.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "Common.h"
#include "Driver.h"
#include "Map2DEnvironment.h"
#include "TemplateAStar.h"
#include "ScenarioLoader.h"
#include "FPUtil.h"
#include "Graphics.h"
#include "SVGUtil.h"
#include "DSDWAStar.h"
#include "MapGenerators.h"

int stepsPerFrame = 1;
float bound = 5;
float testScale = 1.0;
void GetNextWeightRange(float &minWeight, float &maxWeight, point3d currPoint, float nextSlope);
float GetPriority(float h, float g);
float ChooseWeightForTargetPriority(point3d point, float priority, float minWeight, float maxWeight, point3d last, float &K);
bool showPlane = false;
DSDWAStar<xyLoc, tDirection, MapEnvironment> dsd;
std::vector<xyLoc> solution;
bool searchRunning = false;
MapEnvironment *me = 0;
xyLoc start, goal;

int main(int argc, char* argv[])
{
	setvbuf(stdout, NULL, _IONBF, 0);
	InstallHandlers();
	RunHOGGUI(argc, argv, 1000, 1000);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Reset lines", "Reset incremenetal lines", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Show plane", "Show gradient of plane heights", kAnyModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Faster", "Speed up search animation", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Slower", "Slow down search animation", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Policy", "Increment policy", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Problem", "Increment problem", kAnyModifier, '.');

	InstallCommandLineHandler(MyCLHandler, "-stp", "-stp problem alg weight", "Test STP <problem> <algorithm> <weight>");
	InstallCommandLineHandler(MyCLHandler, "-map", "-map <map> <scenario> alg weight", "Test grid <map> on <scenario> with <algorithm> <weight>");

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
		ReinitViewports(windowID, {-1, -1, 0, 1}, kScaleToSquare);
		AddViewport(windowID, {0, -1, 1, 1}, kScaleToSquare);
		Map *m = new Map(200, 200);
		srandom(20221228);
		//BuildRandomRoomMap(m, 30);
		//MakeRandomMap(m, 30);
		MakeMaze(m, 10);
		// default 8-connected with ROOT_TWO edge costs
		me = new MapEnvironment(m);
		dsd.policy = kWA;
		start = {1,1};
		goal = {198, 198};
		dsd.SetWeight(1.8);
		dsd.InitializeSearch(me, start, goal, solution);
		searchRunning = true;
	}

}
//#include "Plot2D.h"
struct DSDdata {
	float slope;
	float weight;
	float K;
	point3d crossPoint; // cached for simplicity
};
std::vector<DSDdata> data;
point3d zero(0, 0);
point3d origin(-1, 1);

point3d HOGToLocal(point3d p)
{
	return point3d((p.x+1)*bound/2.0f , (p.y-1)*bound/-2.0);
}

point3d LocalToHOG(point3d p)
{
	return point3d(2.0f*(p.x/bound-0.5), -2.0*(p.y/bound-0.5));
}

// 1. resize window to 1024x768
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;

	if (viewport == 0)
	{
		if (me)
			me->Draw(display);
		if (searchRunning)
		{
			for (int x = 0; x < stepsPerFrame; x++)
			{
				if (solution.size() == 0)
				{
					if (dsd.DoSingleSearchStep(solution))
						std::cout << "Expansions: " << dsd.GetNodesExpanded() << "\n";
				}
			}
			dsd.Draw(display);
		}
	}
	if (viewport == 1)
	{
		if (searchRunning)
		{
			dsd.DrawPriorityGraph(display);
		}
	}
	
	if (viewport == 3) // TODO: Add mode for exploration
	{
		if (showPlane)
		{
			//		float divisor = GetPriority(bound, bound);
			float divisor = GetPriority(1, 0);
			if (isinf(divisor))
				divisor = 4;
			const float delta = 0.015;//0.025;
			for (float x = 0; x < bound; x+=delta)
			{
				for (float y = 0; y < bound; y+=delta)
				{
					display.FillSquare(LocalToHOG({x, y}), delta/2, rgbColor::hsl(fmodf(GetPriority(x, y)/divisor, 1.0f), 1.0, 0.5));
				}
			}
		}
		
		//	point3d lastCrossPoint(1/bound, 0);
		float priority = 1;//HOGToLocal({-1, -1}).y;//2.0f/bound;
		
		// draw bounding line
		point3d bl1(priority, 0), bl2(0, priority*bound); // main suboptimality line
		point3d bl3(priority*bound, 0);//(0+2, priority*bound-2); //
		point3d bl4(priority*bound/(2*bound-1), 0);//priority*bound-(2*bound-1));
		point3d bl2a(0, priority);
		point3d bl2c(priority-priority*bound/(2*bound-1), priority*bound);
		// main priority line
		display.DrawLine(LocalToHOG(bl1), LocalToHOG(bl2), 1/100.0f, Colors::yellow);
		display.DrawLine(LocalToHOG(bl1*testScale), LocalToHOG(bl2*testScale), 1/100.0f, Colors::darkyellow);
		// 45° upper bound line
		display.DrawLine(LocalToHOG(bl2), LocalToHOG(bl3), 1/100.0f, Colors::darkgray);
		// 2w-1 upper bound line
		display.DrawLine(LocalToHOG(bl1), LocalToHOG(bl2c), 1/100.0f, Colors::darkgray);
		
		// 45° lower bound line
		display.DrawLine(LocalToHOG(bl2a), LocalToHOG(bl1), 1/100.0f, Colors::lightgray);
		// 2w-1 lower bound line
		display.DrawLine(LocalToHOG(bl2), LocalToHOG(bl4), 1/100.0f, Colors::lightgray);
		
		// Draw actual priority line across
		for (int x = 0; x < data.size(); x++)
		{
			point3d value = origin;
			
			// y = slope * x // x=1 -> y = slope; y=1 -> x = 1/slope;
			if (data[x].slope < 1)
			{
				value.x += 2;
				value.y -= 2*data[x].slope;
			}
			else {
				value.x += 2/data[x].slope;
				value.y -= 2;
			}
			display.DrawLine(origin, value, 1/200.0f, Colors::blue);
			
			point3d crossPoint1, crossPoint2;
			crossPoint1.x = priority/(data[x].K*(data[x].slope+data[x].weight));
			crossPoint1.y = crossPoint1.x*data[x].slope;
			float lastSlope = ((x==0)?(0):(data[x-1].slope));
			crossPoint2.x = priority/(data[x].K*(lastSlope+data[x].weight));
			crossPoint2.y = crossPoint2.x*lastSlope;
			display.DrawLine(LocalToHOG(crossPoint1), LocalToHOG(crossPoint2), 1/100.0f, Colors::red);
		}
		for (int x = 0; x < data.size(); x++)
			display.FillCircle(LocalToHOG(data[x].crossPoint), 0.01, Colors::darkgreen);
		
		display.DrawLine(origin, {1, 1}, 1./100.0f, Colors::white);
		display.DrawLine(origin, {-1, -1}, 1./100.0f, Colors::white);
	}
}

#include "MNPuzzle.h"
#include "STPInstances.h"
int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-stp") == 0)
	{
		assert(maxNumArgs >= 4);
		
		DSDWAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> dsd_mnp;
		std::vector<MNPuzzleState<4, 4>> path;
		MNPuzzle<4, 4> mnp;
		MNPuzzleState<4, 4> start = STP::GetKorfInstance(atoi(argument[1]));
		MNPuzzleState<4, 4> goal;
		dsd_mnp.policy = (tExpansionPriority)atoi(argument[2]);
		dsd_mnp.SetWeight(atof(argument[3]));
		printf("Solving STP Korf instance [%d of %d] using DSD weight %f\n", atoi(argument[1])+1, 100, atof(argument[3]));
		dsd_mnp.GetPath(&mnp, start, goal, path);
		printf("STP %d ALG %d weight %1.2f Nodes %llu path %lu\n", atoi(argument[1]), atoi(argument[2]), atof(argument[3]), dsd_mnp.GetNodesExpanded(), path.size());
		exit(0);
	}
	else if (strcmp(argument[0], "-map") == 0)
	{
		assert(maxNumArgs >= 5);
		me = new MapEnvironment(new Map(argument[1]));
		ScenarioLoader sl(argument[2]);
		for (int x = 0; x < sl.GetNumExperiments(); x++)
		{
			Experiment exp = sl.GetNthExperiment(x);
			start.x = exp.GetStartX();
			start.y = exp.GetStartY();
			goal.x = exp.GetGoalX();
			goal.y = exp.GetGoalY();
			dsd.policy = (tExpansionPriority)atoi(argument[3]);
			dsd.SetWeight(atof(argument[4]));
			dsd.GetPath(me, start, goal, solution);
			printf("MAP %s #%d %1.2f ALG %d weight %1.2f Nodes %llu path %f\n", argument[1], x, exp.GetDistance(), atoi(argument[3]), atof(argument[4]), dsd.GetNodesExpanded(), me->GetPathLength(solution));
		}
		exit(0);
	}
	return 0;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 'r': data.resize(0); break;
		case 'p': showPlane = !showPlane; break;
		case '[': stepsPerFrame = stepsPerFrame/2; break;
		case ']': stepsPerFrame = stepsPerFrame*2; break;
		case '}':
			dsd.policy = (tExpansionPriority)((dsd.policy+1)%kDSDPolicyCount);
			printf("Policy: %d\n", dsd.policy);
			dsd.InitializeSearch(me, start, goal, solution);
			searchRunning = true;
			break;
		case '.':
			do {
				start.x = random()%me->GetMap()->GetMapWidth();
				start.y = random()%me->GetMap()->GetMapHeight();
			} while (me->GetMap()->GetTerrainType(start.x, start.y) != kGround);
			do {
				goal.x = random()%me->GetMap()->GetMapWidth();
				goal.y = random()%me->GetMap()->GetMapHeight();
			} while (me->GetMap()->GetTerrainType(goal.x, goal.y) != kGround);
			dsd.InitializeSearch(me, start, goal, solution);
			searchRunning = true;
			break;
		default:
			break;
	}
}

void SetNextPriority(float h, float g, float target) // const Graphics::point &loc
{
	float slope = g/h;
	if (h > 0 && g > 0)
	{
		if (data.size() == 0 || data.back().slope < slope)
		{
			//std::cout << "Virtual hit of " << loc << " slope " << slope << "\n";
			float minWeight, maxWeight;
			if (data.size() > 0)
				GetNextWeightRange(minWeight, maxWeight, data.back().crossPoint, slope);
			else
				GetNextWeightRange(minWeight, maxWeight, {1, 0}, slope);
						
			float K;
			// K (g + [w] * h) = 1 at previous point
			point3d last;
			if (data.size() == 0)
			{
				last = point3d(1, 0);
			}
			else {
				last = data.back().crossPoint;
			}
			// returns nextWeight and K
			float nextWeight = ChooseWeightForTargetPriority({h, g}, target, minWeight, maxWeight, last, K);
			
			// our cross point of next slope
			point3d crossPoint1;
			crossPoint1.x = 1.0f/(K*(slope+nextWeight));
			crossPoint1.y = crossPoint1.x*slope;
			
			data.push_back({slope, nextWeight, K, crossPoint1});
			
//			std::cout << "Cross Priorities: ";
//			for (const auto &i : data)
//			{
//				std::cout << i.crossPoint << ": ";
//				std::cout << GetPriority(i.crossPoint.x, i.crossPoint.y) << " ";
//			}
//			std::cout << "\n";
		}
	}
}

bool MyClickHandler(unsigned long windowID, int viewport, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
//	return false;
//	static point3d startLoc;
//	loc.x = (loc.x+1)/2;
//	loc.y = 1-(loc.y+1)/2;
//	loc *= 2;
	loc = HOGToLocal(loc);
	if (mType == kMouseDown)
	{
		switch (button)
		{
			case kRightButton: //printf("Right button\n"); break;
			{
				std::cout << "Priority of " << loc << " is " << GetPriority(loc.x, loc.y) << "\n";
			}
				break;
			case kLeftButton: //printf("Left button\n");
				
				if (viewport == 3) // TODO: add interactive mode back in later
					SetNextPriority(loc.x, loc.y, testScale); // h and g
				break;
			case kMiddleButton: printf("Middle button\n"); break;
			case kNoButton: break;
		}
	}
	if ((button == kMiddleButton) && (mType == kMouseDown))
	{}
	if (button == kRightButton)
	{}
	return false;
}

float GetPriority(float h, float g)
{
	if (data.size() == 0)
		return INFINITY;
	float slope = g/h;
	if (fgreater(slope, data.back().slope))
		return INFINITY;
	// range includes low but not high
//	int low = 0, high = data.size();
	// dumb/slow but correct
	for (int x = 0; x < data.size(); x++)
		if (flesseq(slope, data[x].slope))
			return data[x].K*(g + data[x].weight * h);
//	while (true)
//	{
//		// f = K * ( g + w_i * h )
//		if (low >= high-1)
//		{
//			return data[low].K*(g + data[low].weight * h);
//		}
//		int mid = (low+high)/2;
//		if (data[mid].slope > slope)
//		{
//			high = mid+1;
//		}
//		else {
//			low = mid+1;
//		}
//	}
	return INFINITY;
//	return data[low].K*(g + data[low].weight * h);
}

/**
 * Given the slope of the next bounding line, give the possbile range of weights that can be used in the priority function
 *
 * \param minWeight (returned) The minimum weight that can be used without going under the lower limit
 * \param maxWeight (returned) The maximum weight that can be used without going over the upper limit
 * \param currPoint The point on the previous bounding line with priorirty 1.0
 * \param nextSlope The slope of the next bounding line
 **/
void GetNextWeightRange(float &minWeight, float &maxWeight, point3d currPoint, float nextSlope)
{
	// defaults
	minWeight = 1;
	maxWeight = 2*bound-1;

	// 0. next slope line is y = slope*x
	// 1. cannot go over the [y = -x + w] line
	// slope*x = -x + w; slope*x + x = w; x = w/(slope+1)
	// y/slope = w-y; y = slope * w - slope *y; y*(1+slope) = slope*w; y = slope*w/(1+slope)
	point3d upperPoint(bound/(nextSlope+1),
					   nextSlope*bound/(1+nextSlope));
//	std::cout << "Upper point: " << upperPoint << "\n";
	// 2. cannot go under the [y = -(2w-1)x + w] line
	// slope*x = -(2w-1)x + w; x*(slope + (2w-1)) = w
	// y/slope = (w-y)/(2w-1)
	// y (2w-1) = slope * w - slope*y
	// y (slope + 2w-1) = slope*w
	point3d lowerPoint(bound/(nextSlope+2*bound-1),
					   nextSlope*bound / (nextSlope + 2*bound-1));
	// get (negative) slopes to upper and lower points
	minWeight = std::max(minWeight, (lowerPoint.y-currPoint.y)/(currPoint.x-lowerPoint.x));
	if (upperPoint.x < currPoint.x)
		maxWeight = std::min(maxWeight, (upperPoint.y-currPoint.y)/(currPoint.x-upperPoint.x));
//	printf("Weight needs to be [%f, %f]\n", minWeight, maxWeight);
}

/*
 * Given location, and a range of priorities, try to give the point the exact desired priority
 * Returned priority will always be in the minWeight/maxWeight range
 */
float ChooseWeightForTargetPriority(point3d loc, float priority, float minWeight, float maxWeight, point3d last, float &K)
{
	float weight;
	// New: Last point is the priority 1 crossing point.
	// 1. Find the point on the previous line with priority {priority}
	point3d projectedPoint = last*priority;
	// 2. FYI: loc is the point on the new line that we want to have the desired priority
	// 3. Find the slope between the last point and our new point.
	//    Which is delta y / delta x
	if (flesseq(loc.y, projectedPoint.y))
	{
		printf("Ill defined case (new y < old y); defaulting to min\n");
		K = 1/(last.y+minWeight*last.x);
		return minWeight;
	}
	if (fgreatereq(loc.x, projectedPoint.x))
	{
		printf("Ill defined case (new x > old x); defaulting to max\n");
		K = 1/(last.y+maxWeight*last.x);
		return maxWeight;
	}
	
	// Then try to extend that point to the new point giving it the desired priority
	weight = (loc.y-projectedPoint.y)/(projectedPoint.x-loc.x);
	// and bound by min/max weight
	weight = std::max(std::min(weight, maxWeight), minWeight);
	K = 1/(last.y+weight*last.x);
	return weight;

/*
	// Old approach. (Keeping to verify should be same for priority=1)
    // Only difference is in checking for ill defined cases
	// Requires that loc be above and to the left of last
	if (loc.x > last.x)
	{
		K = 1/(last.y+maxWeight*last.x);
		return maxWeight;
	}
	if (loc.y < last.y)
	{
		K = 1/(last.y+minWeight*last.x);
		return minWeight;
	}
	// Old: Find the crossing point of two equations of the form:
	// f = K * (y +  w*x);
	// K = 1/(last.y+nextWeight*last.x);
	// priority = K*(loc.y + weight * loc.x);
	// priority = (loc.y + weight * loc.x)/(last.y+weight*last.x);
	// priority*last.y+priority*weight*last.x = loc.y + weight * loc.x;
	// priority*weight*last.x-weight * loc.x = loc.y - priority*last.y
	// weight*(priority*last.x-loc.x) = loc.y - priority*last.y
	weight = (loc.y - priority*last.y)/(priority*last.x-loc.x);
	weight = std::max(std::min(weight, maxWeight), minWeight);
	K = 1/(last.y+weight*last.x);
	return weight;
*/
 }

