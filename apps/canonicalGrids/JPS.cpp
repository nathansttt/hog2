//
//  JPS.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 12/28/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#include "JPS.h"
#include <algorithm>

JPS::JPS()
{
	env = 0;
	weight = 1.0;
}

bool JPS::InitializeSearch(MapEnvironment *env, const xyLoc& from, const xyLoc& to, std::vector<xyLoc> &thePath)
{
	nodesExpanded = nodesTouched = 0;
	this->env = env;
	this->to = to;
	openClosedList.Reset();
	xyLocParent f;
	f.loc = from;
	f.parent = 0xFF;
	openClosedList.AddOpenNode(f, env->GetStateHash(from), 0, weight*env->HCost(from, to));
	if (from == to)
		return false;
	return true;
}

bool JPS::DoSingleSearchStep(std::vector<xyLoc> &thePath)
{
	if (openClosedList.OpenSize() > 0)
	{
		nodesExpanded++;
		uint64_t next = openClosedList.Close();
		xyLocParent nextState = openClosedList.Lookat(next).data;
		//std::cout << nextState.loc << "\n";
		
		// if found goal
		if (nextState.loc == to)
		{
			//printf("Found optimal path cost %1.6f\n", openClosedList.Lookat(next).g);
			thePath.resize(0);
			ExtractPathToStartFromID(next, thePath);
			reverse(thePath.begin(), thePath.end());

			return true;
			// return path
		}
		
		successors.resize(0);
		GetJPSSuccessors(nextState, to);
		
		for (const auto &s : successors)
		{
			uint64_t theID;
			uint64_t hash = env->GetStateHash(s.s.loc);
			dataLocation l = openClosedList.Lookup(hash, theID);
			switch (l)
			{
				case kClosedList:
				{
					break;
				}
				case kNotFound:
				{
					openClosedList.AddOpenNode(s.s,
											   hash,
											   openClosedList.Lookup(next).g+s.cost,
											   weight*env->HCost(s.s.loc, to),
											   next);
					break;
				}
				case kOpenList:
				{
					if (openClosedList.Lookup(next).g+s.cost < openClosedList.Lookup(theID).g)
					{
						openClosedList.Lookup(theID).parentID = next;
						openClosedList.Lookup(theID).g = openClosedList.Lookup(next).g+s.cost;
						openClosedList.Lookup(theID).data.parent = s.s.parent;
						openClosedList.KeyChanged(theID);
					}
					break;
				}
			}
		}
	}
	else {
		return true;
	}
	// path not found
	return false;
}

void JPS::GetPath(MapEnvironment *env, const xyLoc &from, const xyLoc &to, std::vector<xyLoc> &path)
{
	InitializeSearch(env, from, to, path);
	while (DoSingleSearchStep(path) == false)
	{}
}

void JPS::GetPath(MapEnvironment *env, const xyLoc &from, const xyLoc &to, std::vector<tDirection> &path)
{
	
}

void JPS::GetJPSSuccessors(const xyLocParent &s, const xyLoc &goal)
{
	// write this and return g-cost too
	GetJPSSuccessors(s.loc.x, s.loc.y, s.parent, goal, 0);
}

bool JPS::Passable(int x, int y)
{
	return env->GetMap()->GetTerrainType(x, y) == kGround;
}

void JPS::GetJPSSuccessors(int x, int y, uint8_t parent, const xyLoc &goal, double cost)
{
	if (goal.x == x && goal.y == y)
	{
		successors.push_back(jpsSuccessor(x, y, 0, cost));
		return;
	}
	nodesTouched++;
	int w = env->GetMap()->GetMapWidth();
	int h = env->GetMap()->GetMapHeight();
	bool n1 = false, s1 = false, e1 = false, w1 = false;
	if (parent&kN) // action that got me here
	{
		if (y != 0 && Passable(x, (y-1)))
		{
			bool a = false, b = false;
			if (x != 0 && !Passable(x-1, (y)) && Passable(x-1, (y-1)))
				a = true;
			if (x != w-1 && !Passable(x+1, (y)) && Passable(x+1, (y-1)))
				b = true;
			
			if (a && b)
			{
				successors.push_back(jpsSuccessor(x, y-1, tDirection(kNW|kNE), cost+1));
			}
			else if (a)
			{
				successors.push_back(jpsSuccessor(x, y-1, tDirection(kNW), cost+1));
			}
			else if (b)
			{
				successors.push_back(jpsSuccessor(x, y-1, tDirection(kNE), cost+1));
			}
			else {
				GetJPSSuccessors(x, y-1, kN, goal, cost+1);
			}
			n1 = true;
		}
	}
	if (parent&kW) // action that got me here
	{
		if (x != 0 && Passable(x-1, (y)))
		{
			bool a = false, b = false;
			if (y != 0 && !Passable(x, (y-1)) && Passable(x-1, (y-1)))
				a = true;
			if (y != h-1 && !Passable(x, (y+1)) && Passable(x-1, (y+1)))
				b = true;
			
			if (a && b)
			{
				successors.push_back(jpsSuccessor(x-1, y, tDirection(kNW|kSW), cost+1));
			}
			else if (a)
			{
				successors.push_back(jpsSuccessor(x-1, y, tDirection(kNW), cost+1));
			}
			else if (b)
			{
				//neighbors.push_back(xyLoc(x-1, y, kSW));
				successors.push_back(jpsSuccessor(x-1, y, tDirection(kSW), cost+1));
			}
			else {
				GetJPSSuccessors(x-1, y, kW, goal, cost+1);
			}
			e1 = true;
		}
	}
	if (parent&kS) // action that got me here
	{
		if (y != h-1 && Passable(x, (y+1)))
		{
			bool a = false, b = false;
			if (x != 0 && !Passable(x-1, (y)) && Passable(x-1, (y+1)))
				a = true;
			if (x != w-1 && !Passable(x+1, (y)) && Passable(x+1, (y+1)))
				b = true;
			
			if (a && b)
			{
				successors.push_back(jpsSuccessor(x, y+1, tDirection(kSE|kSW), cost+1));
			}
			else if (a)
			{
				successors.push_back(jpsSuccessor(x, y+1, tDirection(kSW), cost+1));
			}
			else if (b)
			{
				successors.push_back(jpsSuccessor(x, y+1, tDirection(kSE), cost+1));
			}
			else {
				GetJPSSuccessors(x, y+1, kS, goal, cost+1);
			}
			s1 = true;
		}
	}
	if (parent&kE) // action that got me here
	{
		if (x != w-1 && Passable(x+1, (y)))
		{
			bool a = false, b = false;
			
			if (y != 0 && !Passable(x, (y-1)) && Passable(x+1, (y-1)))
				a = true;
			if (y != h-1 && !Passable(x, (y+1)) && Passable(x+1, (y+1)))
				b = true;
			
			if (a && b)
			{
				successors.push_back(jpsSuccessor(x+1, y, tDirection(kNE|kSE), cost+1));
			}
			else if (a)
			{
				successors.push_back(jpsSuccessor(x+1, y, tDirection(kNE), cost+1));
			}
			else if (b)
			{
				successors.push_back(jpsSuccessor(x+1, y, tDirection(kSE), cost+1));
			}
			else {
				GetJPSSuccessors(x+1, y, kE, goal, cost+1);
			}
			w1 = true;
		}
	}
	if (parent&kNW)
	{
		if (x != 0 && y != 0 && Passable(x-1, (y-1)) && n1 && e1)
		{
			GetJPSSuccessors(x-1, y-1, kNW, goal, cost+ROOT_TWO);
			//neighbors.push_back(xyLoc(x-1, y-1, kNW));
		}
	}
	if (parent&kNE)
	{
		if (x != w-1 && y != 0 && Passable(x+1, (y-1)) && n1 && w1)
		{
			GetJPSSuccessors(x+1, y-1, kNE, goal, cost+ROOT_TWO);
			//neighbors.push_back(xyLoc(x+1, y-1, kNE));
		}
	}
	if (parent&kSW)
	{
		if (x != 0 && y != h-1 && Passable(x-1, (y+1)) && s1 && e1)
		{
			GetJPSSuccessors(x-1, y+1, kSW, goal, cost+ROOT_TWO);
			//neighbors.push_back(xyLoc(x-1, y+1, kSW));
		}
	}
	if (parent&kSE)
	{
		if (x != w-1 && y != h-1 && Passable(x+1, (y+1)) && s1 && w1)
		{
			GetJPSSuccessors(x+1, y+1, kSE, goal, cost+ROOT_TWO);
			//neighbors.push_back(xyLoc(x+1, y+1, kSE));
		}
	}
}

void JPS::ExtractPathToStartFromID(uint64_t node, std::vector<xyLoc> &thePath)
{
	do {
		thePath.push_back(openClosedList.Lookup(node).data.loc);
		node = openClosedList.Lookup(node).parentID;
	} while (openClosedList.Lookup(node).parentID != node);
	thePath.push_back(openClosedList.Lookup(node).data.loc);
}

uint64_t JPS::GetNodesExpanded() const
{
	return nodesExpanded;
}

uint64_t JPS::GetNodesTouched() const
{
	return nodesTouched;
}

void JPS::LogFinalStats(StatCollection *stats)
{
	
}

void JPS::OpenGLDraw() const
{
	double transparency = 1.0;
	if (openClosedList.size() == 0)
		return;
	uint64_t top = -1;
	
	if (openClosedList.OpenSize() > 0)
	{
		top = openClosedList.Peek();
	}
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const AStarOpenClosedData<xyLocParent> &data = openClosedList.Lookat(x);
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->OpenGLDraw(data.data.loc);
		}
		if ((data.where == kOpenList) && (data.reopened))
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->OpenGLDraw(data.data.loc);
		}
		else if (data.where == kOpenList)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->OpenGLDraw(data.data.loc);
		}
		else if ((data.where == kClosedList) && (data.reopened))
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			env->OpenGLDraw(data.data.loc);
		}
		else if (data.where == kClosedList)
		{
			env->SetColor(1.0, 0.0, 0.0, transparency);
			env->OpenGLDraw(data.data.loc);
		}
	}
}

void JPS::OpenGLDraw(const MapEnvironment *env) const {}
