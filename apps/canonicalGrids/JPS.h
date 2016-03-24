//
//  JPS.hpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 12/28/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#ifndef JPS_h
#define JPS_h

#include <stdio.h>
#include "GenericSearchAlgorithm.h"
#include "Map2DEnvironment.h"
#include "AStarOpenClosed.h"
// For comparing items in open/closed
#include "TemplateAStar.h"

struct xyLocParent
{
	xyLoc loc;
	uint8_t parent;
};

struct jpsSuccessor
{
	jpsSuccessor() {}
	jpsSuccessor(int x, int y, uint8_t parent, double cost)
	{ s.loc.x = x; s.loc.y = y; s.parent = parent; this->cost = cost; }
	xyLocParent s;
	double cost;
};

class JPS : public GenericSearchAlgorithm<xyLoc, tDirection, MapEnvironment>
{
public:
	JPS();
	void GetPath(MapEnvironment *env, const xyLoc &from, const xyLoc &to, std::vector<xyLoc> &path);
	void GetPath(MapEnvironment *env, const xyLoc &from, const xyLoc &to, std::vector<tDirection> &path);
	
	bool InitializeSearch(MapEnvironment *env, const xyLoc& from, const xyLoc& to, std::vector<xyLoc> &thePath);
	bool DoSingleSearchStep(std::vector<xyLoc> &thePath);
	
	const char *GetName() { return "JPS"; }
	uint64_t GetNodesExpanded() const;
	uint64_t GetNodesTouched() const;
	uint64_t GetNumOpenItems() const { return openClosedList.OpenSize(); }
	void SetWeight(double val) { weight = val; }
	void SetJumpLimit(uint32_t val) { jumpLimit = val; }
	void LogFinalStats(StatCollection *stats);
	void OpenGLDraw() const;
	std::string SVGDraw();
	void OpenGLDraw(const MapEnvironment *env) const;
private:
	void GetJPSSuccessors(const xyLocParent &s, const xyLoc &goal);
	void GetJPSSuccessors(int x, int y, uint8_t parent, const xyLoc &goal, double cost);
	bool Passable(int x, int y);
	void ExtractPathToStartFromID(uint64_t node, std::vector<xyLoc> &thePath);
	AStarOpenClosed<xyLocParent, AStarCompare<xyLocParent> > openClosedList;
	std::vector<jpsSuccessor> successors;
	MapEnvironment *env;
	xyLoc to;
	uint64_t nodesExpanded, nodesTouched;
	double weight;
	uint32_t jumpLimit;
};

#endif /* JPS_h */
