//
//  CBSUnits.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/6/12.
//  Copyright (c) 2012 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__CBSUnits__
#define __hog2_glut__CBSUnits__

#include <iostream>
#include "Unit.h"
#include "UnitGroup.h"
#include "Map2DEnvironment.h"
#include "Map2DConstrainedEnvironment.h"
#include "TemplateAStar.h"
#include "MapOverlay.h"

class CBSUnit : public Unit<xyLoc, tDirection, MapEnvironment> {
public:
	CBSUnit(const xyLoc &s, const xyLoc &g)
	:start(s), goal(g), current(s) {}
	const char *GetName() { return "CBSUnit"; }
	bool MakeMove(MapEnvironment *, OccupancyInterface<xyLoc,tDirection> *, SimulationInfo<xyLoc,tDirection,MapEnvironment> *, tDirection& a);
	void UpdateLocation(MapEnvironment *, xyLoc &newLoc, bool success, SimulationInfo<xyLoc,tDirection,MapEnvironment> *)
	{ if (success) current = newLoc; else assert(!"CBS Unit: Movement failed"); }
	
	void GetLocation(xyLoc &l) { l = current; }
	void OpenGLDraw(const MapEnvironment *, const SimulationInfo<xyLoc,tDirection,MapEnvironment> *) const;
	void GetGoal(xyLoc &s) { s = goal; }
	void GetStart(xyLoc &s) { s = start; }
	void SetPath(std::vector<xyLoc> &p);
private:
	xyLoc start, goal, current;
	std::vector<xyLoc> myPath;
};

struct conflict {
	constraint c;
	int unit1;
};

struct CBSTreeNode {
	CBSTreeNode() { closed = false; }
	
	std::vector< std::vector<xyLoc> > paths;
	conflict con;
	unsigned int parent;
	bool closed;
};

class CBSGroup : public UnitGroup<xyLoc, tDirection, MapEnvironment>
{
public:
	CBSGroup(MapEnvironment *me);
	bool MakeMove(Unit<xyLoc, tDirection, MapEnvironment> *u, MapEnvironment *e, SimulationInfo<xyLoc,tDirection,MapEnvironment> *si, tDirection& a);
	void UpdateLocation(Unit<xyLoc, tDirection, MapEnvironment> *u, MapEnvironment *e, xyLoc &loc, bool success, SimulationInfo<xyLoc,tDirection,MapEnvironment> *si);
	void AddUnit(Unit<xyLoc, tDirection, MapEnvironment> *u);
	void OpenGLDraw(const MapEnvironment *, const SimulationInfo<xyLoc,tDirection,MapEnvironment> *)  const;
private:
	void ExpandOneCBSNode();
	void Replan(int location);
	bool FindFirstConflict(int location, conflict &c1, conflict &c2);
	
	bool planFinished;
	Map2DConstrainedEnvironment *m2e;
	MapEnvironment *me;
	std::vector<CBSTreeNode> tree;
	std::vector<xytLoc> thePath;
	TemplateAStar<xytLoc, tDirection, Map2DConstrainedEnvironment> astar;
	double time;
	unsigned int bestNode;
	MapOverlay *overlay;
};


#endif /* defined(__hog2_glut__CBSUnits__) */
