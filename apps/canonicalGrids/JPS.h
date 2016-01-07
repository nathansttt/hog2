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

class JPS : public GenericSearchAlgorithm<xyLoc, tDirection, MapEnvironment>
{
public:
	JPS();
	void GetPath(MapEnvironment *env, const xyLoc &from, const xyLoc &to, std::vector<xyLoc> &path);
	void GetPath(MapEnvironment *env, const xyLoc &from, const xyLoc &to, std::vector<tDirection> &path);
	const char *GetName() { return "JPS"; }
	uint64_t GetNodesExpanded() const;
	uint64_t GetNodesTouched() const;
	void LogFinalStats(StatCollection *stats);
	void OpenGLDraw() const;
	void OpenGLDraw(const MapEnvironment *env) const;
private:
	void GetJPSSuccessors(xyLocParent s);
	AStarOpenClosed<xyLocParent, AStarCompare<xyLocParent> > openClosedList;
	std::vector<xyLocParent> successors;
};

#endif /* JPS_h */
