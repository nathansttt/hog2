/*
 * $Id: unitSimulation.h,v 1.26 2006/11/01 23:28:21 nathanst Exp $
 *
 *  Hierarchical Open Graph File
 *
 *  Created by Nathan Sturtevant on 9/30/04.
 *  Copyright 2004 Nathan Sturtevant. All rights reserved.
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifndef UNITSIMULATION_H
#define UNITSIMULATION_H

#include <vector>
#include <queue>

#include "bitVector.h"
#include "constants.h"
#include "map.h"
#include "mapAbstraction.h"
#include "reservationProvider.h"
#include "mapProvider.h"
#include "statCollection.h"
#ifdef OS_MAC
#include <Carbon/Carbon.h>
#undef check
#endif

class unit;
class unitGroup;

enum {
	kUnitSimulationMap = 0
};

class timeStep {
public:
	timeStep(int _x, int _y, double time)
	:x(_x), y(_y), startTime(time) {}
	timeStep() { x = -1; y = -1; startTime = 0; }
  int x, y;
  double startTime;
};

/**
 * Private per-unit unitSimulation data.
 */

class unitInfo {
public:
	unitInfo() :actionHistory(0) {}
	unit *agent;
	tDirection lastMove;
	bool blocking;
	int startx, starty;
	int currx, curry;
	double thinkTime, moveDist;
	double nextTime;
	bool ignoreOnTarget;
	unsigned int historyIndex;
	std::vector<timeStep> actionHistory;
};

class unitInfoCompare {
public:
	bool operator()(const unitInfo* u1, const unitInfo* u2);
};

class simulationInfo {
public:
	virtual ~simulationInfo() {};
	virtual double getSimulationTime() = 0;
};

/**
 * The basic simulation class for the world.
 */

class unitSimulation : public mapProvider, reservationProvider, public simulationInfo {
public:
	unitSimulation(mapAbstraction *, bool keepStats = false);
	virtual ~unitSimulation();
	
	bool saveHistory(char *, bool includeMap = true);
	bool loadHistory(char *);
	
	virtual void addUnit(unit *);
	virtual void addUnit(unit *, bool block);
	virtual void addUnitGroup(unitGroup *);
	unitGroup *getUnitGroup(int which);
	unit *getUnit(int which);
	unit *findUnit(int x, int y);
	bool setIgnoreOnTarget(unit*,bool);
								 
	virtual void advanceTime(double amount);
	void setRealTime(bool);
	bool getRealTime() { return realTime; }
	inline void setLockstepTime(bool b) { lockstepTime = b; }
	bool getLockstepTime() { return lockstepTime; }
	/** Set if the simulation is asynchronous. */
	void setAsynchronous() { asynch = true; }
	void setSynchronous() { asynch = false; }
	void setSimulationPaused(bool val) { pause = val; }
	bool getSimulationPaused() { return pause; }
	/** return the current inside the simulation */
	double getSimulationTime() { return currTime; }
	/** return the current time being drawn */
	double getDisplayTime() { return viewTime; }
	void setDisplayTime(double val);
	void offsetDisplayTime(double val);
	void openGLDraw();
	void print(bool forceOutput = true);
	virtual bool done();
	/** reservationProvider interface */
	inline bool nodeOccupied(node *currNode)
	{ if (currNode->getLabelL(kAbstractionLevel) != 0) return false;
		return tileOccupied((unsigned int)currNode->getLabelL(kFirstData),
		                    (unsigned int)currNode->getLabelL(kFirstData+1)); }
	inline bool tileOccupied(int x, int y) { return bv->get(y*map_width+x); }
	/* temporal reservations not supported by unitSimulation */
	virtual bool canMove(node *, node *, double, unit *) { return true;}
	virtual bool reserveMove(node *, node *, double, unit *) { return true; }
	virtual bool clearMove(node *, node *, double, unit *) { return true; }
	virtual void clearAllReservations() {}
	
	void clearAllUnits();
	/** turns unit blocking on and off */
	void setUseBlocking(bool val) { blocking = val; }
	void setmapAbstractionDisplay(int _whichMap=kUnitSimulationMap);
	/** Return which map is being currently displayed. */
	int getDisplayMapNumber() { return which_map; }

	/** Returns the underlying map. */
	Map *getMap() { return map; }
	/** Returns the abstract map from the simulation. */
	mapAbstraction *getMapAbstraction();
	
	/** Returns the nth groups abstract map. (0 is the actual map of the world.) */
	mapAbstraction *getMapAbstraction(int _which);
	/** Returns the abstract map currently being displayed. */
	mapAbstraction *getMapAbstractionDisplay();
	/** Cycle which abstract map should be displayed. */
	void cyclemapAbstractionDisplay();
	
	/** setPenalty for thinking. Sets the multiplier used to penalize thinking time. */
	void setPenalty(double pen) { penalty = pen; }
	/** getPenalty for thinking. Gets the multiplier used to penalize thinking time. */
	double getPenalty() { return penalty; }
	/** Toggle open GL display */
	void toggleNoOpenGLDraw() { noOpenGLDraw = !noOpenGLDraw; }
	/** set chance for move failing. This is the chance that a move will just fail. */
	void setMoveStochasticity(double _stochasticity) { stochasticity = _stochasticity; }
	double getMoveStochasticity() { return stochasticity; }
	//void setLogFile(FILE *);
	statCollection *getStats() { return &stats; }
	void printCollectedStats(bool v) { stats.enablePrintOutput(v); }
	void getRandomLocation(int &x, int &y, tTerrain terrain = kGround);
	void getRandomLocation(int x1, int y1, int &x2, int &y2, tTerrain terrain = kGround);
	void getRandomLocations(int &x1, int &y1, int &x2, int &y2, tTerrain terrain = kGround);

	bool canCrossDiagonally() { return (!disallowDiagonalCrossingMoves); }
	void setCanCrossDiagonally(bool cross) { disallowDiagonalCrossingMoves = !cross; }
	
protected:
	unitInfo *findUnit(unit *);	
	virtual void doPreTimestepCalc();
	virtual void doTimestepCalc();
	virtual void doPostTimestepCalc();
	void stepUnitTime(unitInfo *);
	void setAgentLocation(unitInfo *, bool success = false, bool timer = false);
	void updateMap();
	bool updatemapAbstraction();
	void drawBlockedSquare(int x, int y);
// 	void startTimer();
// 	double endTimer();
	bool findUnitDisplayTime(unitInfo *ui);
	
	Map *map;
	//FILE *LOGFILE;
	mapAbstraction *aMap;
	bitVector *bv;
	int which_map;						// the number of the group to display info for
	int map_width, map_height, map_revision;
	std::vector<unitInfo *> units;
	std::vector<unitInfo *> displayUnits;
	std::vector<unitGroup *> unitGroups;
	std::priority_queue<const unitInfo*, std::vector<unitInfo *>, unitInfoCompare> moveQ;
	double currTime, viewTime;
	bool asynch;
	bool blocking;		// this is the default for all units added
	bool realTime;
	bool pause;
	bool lockstepTime;      // Finn/Wes - individual unit times will be update exactly according to the amount specified to advanceTime
	double penalty;
	double stochasticity;
	bool unitsMoved;
	bool disallowDiagonalCrossingMoves;
	double currDist;        // total distance moved so far
	bool noOpenGLDraw;		// turns display on/off
	bool keepHistory; // keep action history
	
	statCollection stats;
};

#define LOCAL_PATH

#endif
