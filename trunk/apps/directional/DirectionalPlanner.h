/*
 *  DirectionalPlanner.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/11/09.
 *  Copyright 2009 NS Software. All rights reserved.
 *
 */

#include "GenericSearchAlgorithm.h"
#include "MapAbstraction.h"
#include "TemplateAStar.h"
#include "Directional2DEnvironment.h"
#include "GraphEnvironment.h"

class myGoalTest : public GoalTester {
public:
	virtual bool goalTest(const xySpeedHeading &i1) const
	{
		node *n1 = mapAbs->GetNodeFromMap(i1.x, i1.y);
		return (mapAbs->GetParent(n1) == absState);
	}

	MapAbstraction *mapAbs;
	node *absState;
};


class DirectionalPlanner : public GenericSearchAlgorithm<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment>
{
public:
	DirectionalPlanner(MapAbstraction *abst, int refinement = 3) :mapAbs(abst) { age = 0; refineLength = refinement; };
	virtual ~DirectionalPlanner() { delete age; };
	virtual void GetPath(Directional2DEnvironment *env, const xySpeedHeading &from, const xySpeedHeading &to, std::vector<xySpeedHeading> &path);
	virtual void GetPath(Directional2DEnvironment *, const xySpeedHeading &, const xySpeedHeading &, std::vector<deltaSpeedHeading> &) { return; }
	virtual const char *GetName() { return "DirAbstractPlanner"; }
	virtual uint64_t GetNodesExpanded() const { return lowLevelSearcher.GetNodesExpanded();/* + abstractSearcher.GetNodesExpanded();*/ }
	virtual uint64_t GetNodesTouched() const { return lowLevelSearcher.GetNodesTouched();/* + abstractSearcher.GetNodesTouched(); */}
	virtual void LogFinalStats(StatCollection *) { return; }
	virtual void OpenGLDraw() const;
private:
	TemplateAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> lowLevelSearcher;
	TemplateAStar<graphState, graphMove, GraphEnvironment> abstractSearcher;
	MapAbstraction *mapAbs;
	AbstractionGraphEnvironment *age;
	std::vector<graphState> absPath;
	int refineLength;
};
