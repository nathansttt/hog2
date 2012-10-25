//
//  CBSUnits.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/6/12.
//  Copyright (c) 2012 University of Denver. All rights reserved.
//

#include "CBSUnits.h"

bool CBSUnit::MakeMove(MapEnvironment *me, OccupancyInterface<xyLoc,tDirection> *, SimulationInfo<xyLoc,tDirection,MapEnvironment> *, tDirection& a)
{
	if (myPath.size() > 1)
	{
		a = me->GetAction(myPath[myPath.size()-1], myPath[myPath.size()-2]);
		myPath.pop_back();
		return true;
	}
	return false;
}
	
void CBSUnit::SetPath(std::vector<xyLoc> &p)
{
	myPath = p;
	std::reverse(myPath.begin(), myPath.end());
}

void CBSUnit::OpenGLDraw(const MapEnvironment *me, const SimulationInfo<xyLoc,tDirection,MapEnvironment> *) const
{
	GLfloat r, g, b;
	GetColor(r, g, b);
	me->SetColor(r, g, b);
	me->OpenGLDraw(current);
	me->GLDrawPath(myPath);
}

//struct CBSTreeNode {
//	std::vector< std::vector<xyLoc> > paths;
//	constraint c;
//	unsigned int parent;
//	bool closed;
//};

//	bool planFinished;
//	Map2DConstrainedEnvironment *m2e;
//	std::vector<CBSTreeNode> tree;

CBSGroup::CBSGroup(MapEnvironment *me)
{
	this->me = me;
	m2e = new Map2DConstrainedEnvironment(me->GetMap());
	planFinished = false;
	time = 0;
	tree.resize(1);
	tree[0].parent = 0;
	bestNode = 0;
	overlay = new MapOverlay(me->GetMap());
	overlay->SetTransparentValue(-1);
	overlay->SetColorMap(1);
	for (int x = 0; x < me->GetMap()->GetMapWidth(); x++)
	{
		for (int y = 0; y < me->GetMap()->GetMapHeight(); y++)
		{
			if (me->GetMap()->GetTerrainType(x, y) != kGround)
				overlay->SetOverlayValue(x, y, -1);
		}
	}
}

bool CBSGroup::MakeMove(Unit<xyLoc, tDirection, MapEnvironment> *u, MapEnvironment *e, SimulationInfo<xyLoc,tDirection,MapEnvironment> *si, tDirection& a)
{
	if (planFinished && si->GetSimulationTime() > time)
	{
		return u->MakeMove(e, 0, si, a);
	}
	if (si->GetSimulationTime() == time)
	{
		return false;
	}
	else {
		time = si->GetSimulationTime();
		// expand 1 CBS node
		ExpandOneCBSNode();
	}
	return false;
}

void CBSGroup::ExpandOneCBSNode()
{
	conflict c1, c2;
	bool found = FindFirstConflict(bestNode, c1, c2);
	if (!found)
	{
		planFinished = true;
		for (unsigned int x = 0; x < tree[bestNode].paths.size(); x++)
		{
			CBSUnit *unit = (CBSUnit*)GetMember(x);
			unit->SetPath(tree[bestNode].paths[x]);
		}
		return;
	}
	if (overlay)
	{
		overlay->SetOverlayValue(c1.c.loc.l.x, c1.c.loc.l.y, overlay->GetOverlayValue(c1.c.loc.l.x, c1.c.loc.l.y)+1);
		overlay->SetOverlayValue(c2.c.loc.l.x, c2.c.loc.l.y, overlay->GetOverlayValue(c2.c.loc.l.x, c2.c.loc.l.y)+1);
	}
	
	unsigned long last = tree.size();
	tree.resize(last+2);
	
	tree[last] = tree[bestNode];
	tree[last].con = c1;
	tree[last].parent = bestNode;

	tree[last+1] = tree[bestNode];
	tree[last+1].con = c2;
	tree[last+1].parent = bestNode;
	
	tree[bestNode].closed = true;
	
	Replan(last);
	Replan(last+1);
	
	double bestCost = 100000000;
	for (unsigned int x = 0; x < tree.size(); x++)
	{
		double cost = 0;
		for (int y = 0; y < tree[x].paths.size(); y++)
			cost += me->GetPathLength(tree[x].paths[y]);
		if (cost < bestCost && tree[x].closed == false)
		{
			bestNode = x;
			bestCost = cost;
		}
	}
	std::cout << "New best node " << bestNode << std::endl;
}

void CBSGroup::UpdateLocation(Unit<xyLoc, tDirection, MapEnvironment> *u, MapEnvironment *e, xyLoc &loc, bool success, SimulationInfo<xyLoc,tDirection,MapEnvironment> *si)
{
	u->UpdateLocation(e, loc, success, si);
}

void CBSGroup::AddUnit(Unit<xyLoc, tDirection, MapEnvironment> *u)
{
	UnitGroup::AddUnit(u);

	CBSUnit *c = (CBSUnit*)u;
	m2e->ClearConstraints();
	xytLoc start, goal;
	c->GetStart(start.l);
	c->GetGoal(goal.l);
	start.t = 0;
	goal.t = 0;
	tree[0].paths.resize(GetNumMembers());
	astar.GetPath(m2e, start, goal, thePath);
	for (unsigned int x = 0; x < thePath.size(); x++)
	{
		tree[0].paths.back().push_back(thePath[x].l);
	}
}

void CBSGroup::Replan(int location)
{
	int theUnit = tree[location].con.unit1;
	m2e->ClearConstraints();
	int tempLocation = location;
	while (tempLocation != 0)
	{
		if (theUnit == tree[tempLocation].con.unit1)
			m2e->AddConstraint(tree[tempLocation].con.c);
		tempLocation = tree[tempLocation].parent;
		// todo find constarints on agents goal here
	}

	CBSUnit *c = (CBSUnit*)GetMember(theUnit);
	xytLoc start, goal;
	c->GetStart(start.l);
	c->GetGoal(goal.l);
	start.t = 0;
	goal.t = 0; // TODO: find the true goal time

	astar.GetPath(m2e, start, goal, thePath);
	tree[location].paths[theUnit].resize(0);
	for (unsigned int x = 0; x < thePath.size(); x++)
	{
		tree[location].paths[theUnit].push_back(thePath[x].l);
	}
}

bool CBSGroup::FindFirstConflict(int location, conflict &c1, conflict &c2)
{
	for (int x = 0; x < GetNumMembers(); x++)
	{
		for (int y = x+1; y < GetNumMembers(); y++)
		{
			int maxLength = (tree[location].paths[x].size() > tree[location].paths[y].size())?tree[location].paths[x].size():tree[location].paths[y].size();
			for (int z = 0; z+1 < maxLength; z++)
			{
				int a1time = min(z, tree[location].paths[x].size()-1);
				int a2time = min(z, tree[location].paths[y].size()-1);
				if (tree[location].paths[x][a1time] == tree[location].paths[y][a2time])
				{
					c1.unit1 = x;
					c2.unit1 = y;
					c1.c.loc.l = tree[location].paths[x][a1time];
					c2.c.loc.l = tree[location].paths[y][a2time];
					c1.c.loc.t = z;
					c2.c.loc.t = z;
					c1.c.dir = kTeleport;
					c2.c.dir = kTeleport;
					std::cout << "State conflict found between " << x << " and " << y << " at " << c1.c.loc.l << std::endl;
					return true;
				}
				if (z >= tree[location].paths[x].size() ||
					z >= tree[location].paths[y].size())
					continue;
				
				if (tree[location].paths[x][z] == tree[location].paths[y][z+1] &&
					tree[location].paths[x][z+1] == tree[location].paths[y][z])
				{
					c1.unit1 = x;
					c2.unit1 = y;
					c1.c.loc.l = tree[location].paths[x][z];
					c2.c.loc.l = tree[location].paths[y][z];
					c1.c.loc.t = z;
					c2.c.loc.t = z;

					c1.c.dir = me->GetAction(tree[location].paths[x][z], tree[location].paths[y][z]);
					c2.c.dir = me->GetAction(tree[location].paths[y][z], tree[location].paths[x][z]);
					
					std::cout << "Edge conflict found between " << x << " and " << y << " at " << c1.c.loc.l << " and " << c2.c.loc.l << std::endl;

					return true;
				}
			}
		}
	}
	
	return false;
}

void CBSGroup::OpenGLDraw(const MapEnvironment *me, const SimulationInfo<xyLoc,tDirection,MapEnvironment> *)  const
{
	GLfloat r, g, b;

//	int tempLocation = bestNode;
//	while (tempLocation != 0)
//	{
//		CBSUnit *unit = (CBSUnit*)GetMember(tree[tempLocation].con.unit1);
//		unit->GetColor(r, g, b);
//		m2e->SetColor(r, g, b, 0.5);
//		m2e->OpenGLDraw(tree[tempLocation].con.c.loc);
//
//		tempLocation = tree[tempLocation].parent;
//	}

	glLineWidth(2.0);
	for (unsigned int x = 0; x < tree[bestNode].paths.size(); x++)
	{
		CBSUnit *unit = (CBSUnit*)GetMember(x);
		unit->GetColor(r, g, b);
		m2e->SetColor(r, g, b);
		for (unsigned int y = 0; y+1 < tree[bestNode].paths[x].size(); y++)
		{
			xytLoc a(tree[bestNode].paths[x][y], y);
			xytLoc b(tree[bestNode].paths[x][y+1], y+1);
			m2e->GLDrawLine(a, b);
		}
		//me->GLDrawPath(tree[bestNode].paths[x]);
	}
	glLineWidth(1.0);
	if (overlay)
		overlay->OpenGLDraw();
}

