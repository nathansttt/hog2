//
//  JPS.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 12/28/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#include "JPS.h"

JPS::JPS()
{
	
}

void JPS::GetPath(MapEnvironment *env, const xyLoc &from, const xyLoc &to, std::vector<xyLoc> &path)
{
	openClosedList.Reset();
	xyLocParent f;
	f.loc = from;
	f.parent = 0xFF;
	openClosedList.AddOpenNode(f, env->GetStateHash(from), 0, env->HCost(from));
	path.resize(0);
	
	while (openClosedList.OpenSize() > 0)
	{
		uint64_t next = openClosedList.Close();
		xyLocParent nextState = openClosedList.Lookat(next).data;

		// if found goal
		// return path
		
		successors.resize(0);
		GetJPSSuccessors(nextState);

		// foreach successor
		{
			// if on closed, ignore
			// if on open, update
			// else add to open
		}
	}
	// path not found
}

void JPS::GetPath(MapEnvironment *env, const xyLoc &from, const xyLoc &to, std::vector<tDirection> &path)
{
	
}

void JPS::GetJPSSuccessors(xyLocParent s)
{
	
}

uint64_t JPS::GetNodesExpanded() const
{
	return 0;
}

uint64_t JPS::GetNodesTouched() const
{
	return 0;
}

void JPS::LogFinalStats(StatCollection *stats)
{
	
}

void JPS::OpenGLDraw() const {}
void JPS::OpenGLDraw(const MapEnvironment *env) const {}
