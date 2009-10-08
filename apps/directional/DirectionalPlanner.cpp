/*
 *  DirectionalPlanner.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/11/09.
 *  Copyright 2009 NS Software. All rights reserved.
 *
 */

#include "DirectionalPlanner.h"

void DirectionalPlanner::GetPath(Directional2DEnvironment *env,
								 const xySpeedHeading &from, const xySpeedHeading &to,
								 std::vector<xySpeedHeading> &path)
{
	// 1. Find abstract path between from and two
	Graph *g = mapAbs->GetAbstractGraph(1);
	GraphAbstractionHeuristic gah(mapAbs, 1);
	delete age;
	age = new AbstractionGraphEnvironment(mapAbs, 1, &gah);
	graphState start = mapAbs->GetParent(mapAbs->GetNodeFromMap(from.x, from.y))->GetNum();
	graphState end = mapAbs->GetParent(mapAbs->GetNodeFromMap(to.x, to.y))->GetNum();
	
	//std::vector<graphState> absPath;
	abstractSearcher.GetPath(age, start, end, absPath);

	// 2. Convert to direction and find short-term path
	int index = refineLength;
	xySpeedHeading newTo;
	if ((int)absPath.size() > refineLength)
	{
		int x, y;
		mapAbs->GetTileFromNode(g->GetNode(absPath[index]), x, y);
		newTo.x = x;
		newTo.y = y;
		newTo.speed = 0;
		newTo.rotation = 0;

//		int x1, y1;
//		int angles = env->GetNumAngles();
//		mapAbs->GetTileFromNode(g->GetNode(absPath[std::min(index+3, (int)absPath.size()-1)]), x1, y1);
//		double angle = fabs(y1-y)/fabs(x1-x);
//		angle = atan(angle);
//		angle = angles*angle/TWOPI;
//		if (x1 == x)
//			angle = angles/4;
////		printf("Initial angle is %f, (%1.1f, %1.1f) -> (%d, %d) -> (%d, %d)\n", angle,
////			   from.x, from.y, x, y, x1, y1);
//		if ((x1>x) && (y1>y))
//		{}
//		else if ((x1<x) && (y1>y))
//		{ angle = angles/2-angle; }
//		else if ((x1>=x) && (y1<y))
//		{ angle = angles-angle; }
//		else if ((x1<x) && (y1<=y))
//		{ angle = angles/2+angle; }
////		printf("Setting intermediate angle to %f\n", angle);
//		newTo.rotation = ((int)angle)%angles;

//		myGoalTest mgt;
//		mgt.mapAbs = mapAbs;
//		mgt.absState =  g->GetNode(absPath[index]);
//		env->SetGoalTest(&mgt);
	}
	else {
		env->SetGoalTest(0);
		newTo = to;
		index = -1;
	}
	lowLevelSearcher.SetWeight(1.2);
	lowLevelSearcher.GetPath(env, from, newTo, path);
//	if (index != -1)
//		path.resize(path.size() - path.size()/4);
}

void DirectionalPlanner::OpenGLDraw() const
{
//	mapAbs->OpenGLDraw();
	/*abstractSearcher.OpenGLDraw();*/
//	lowLevelSearcher.OpenGLDraw(); 
	for (unsigned int x = 0; x < absPath.size(); x++)
	{
		//printf("Drawing abstract path %d\n", x);
		graphState s = absPath[x];
		age->OpenGLDraw(s);
	}
}
