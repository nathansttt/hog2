/**
* @file AbsMapPatrolUnit.h
* @package HOG2
* 
* @author Nathan Sturtevant
* @date Modified for HOG2 on June 18, 2007
*
*/

#ifndef ABSMAPPATROLUNIT_H
#define ABSMAPPATROLUNIT_H

#include "SearchUnit.h"
#include "SearchAlgorithm.h"
#include "GLUtil.h"

class AbsMapPatrolUnit : public SearchUnit {
public:
	AbsMapPatrolUnit(int x, int y, SearchAlgorithm* alg);

	virtual const char *GetName() { return "AbsMapPatrolUnit"; }
	
	virtual bool done() { return false; }
	
	virtual tDirection makeMove(MapProvider *, reservationProvider *, SimulationInfo *simInfo);
	virtual tDirection MakeMove(AbsMapEnvironment *ame, BaseMapOccupancyInterface *, SimulationInfo *si){ std::cout<<"HERE\n"; return makeMove(ame->GetMapAbstraction(), 0, si); }
	//void OpenGLDraw(int window, MapProvider *, SimulationInfo *);
	virtual void OpenGLDraw(int window, AbsMapEnvironment *, SimulationInfo *);
	void addPatrolLocation(xyLoc);
	xyLoc GetGoal();
		
	void UpdateLocation(AbsMapEnvironment *, xyLoc &l, bool success, SimulationInfo *si) { updateLocation(l.x, l.y, success, si); }	
	void updateLocation(int _x, int _y, bool worked, SimulationInfo *)
	{ loc.x = _x; loc.y = _y; if (!worked) { moves.resize(0); if (currTarget != -1) currTarget = 0; } }
	void logStats(StatCollection *stats);
		void LogFinalStats(StatCollection *stats);
private:
	double goToLoc(MapAbstraction *aMap, int which);
	void addPathToCache(path *p);
	//std::vector<tDirection> moves;
	std::vector<xyLoc> Locs;
//	aStar a;
//	praStar a;
	//SearchAlgorithm *algorithm;
	
	int currTarget;
	int nodesExpanded;
	int nodesTouched;
};

#endif
