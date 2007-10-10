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
#include "Unit.h"
#include "ReservationProvider.h"
#include "Timer.h"
#include "FPUtil.h"
#include "StatCollection.h"
#include "SearchEnvironment.h"
#include "OccupancyInterface.h"

class SimulationInfo {
public:
	virtual ~SimulationInfo() {}
	virtual double GetSimulationTime() = 0;
};

/**
 * Private per-unit unitSimulation data.
 */

template <class state, class action>
class TimeStep {
public:
	TimeStep(state s, double time)
	:m_state(s), startTime(time) {}
	TimeStep() { startTime = 0; }
  state m_state;
  double startTime;
};

template <class state, class action, class environment>
class Unit;

template <class state, class action, class environment>
class UnitGroup;


template <class state, class action, class environment>
class UnitInfo {
public:
	UnitInfo() :stateHistory(0) {}
	Unit<state, action, environment> *agent;
	action lastMove;
	state startState;
	state currentState;
//	double thinkTime, moveDist;
	double nextTime;
	unsigned int historyIndex;
	double totalDistance;
	double totalThinking;
	std::vector<TimeStep<state, action> > stateHistory;
};

//template<class state, class action>
//class UnitInfoCompare {
//public:
//	bool operator()(const UnitInfo<state, action> *u1, const UnitInfo<state, action> *u2);
//};


// kLockStep - each unit goes exactly to the next time
// kRealTime - each unit goes up thinkingTime*thinkingPenalty + movement time*speed
// kMinTime - each unit goes up at least step time, but can go longer
//            that is, max(next time, thinkingTime*thinkingPenalty + movement time*speed)
enum tTimestep {
	kLockStep, kRealTime, kMinTime
};

/**
 * The basic simulation class for the world.
 */
template<class state, class action, class environment>
class UnitSimulation : public SimulationInfo {
public:
	UnitSimulation(environment *se);

	virtual int AddUnit(Unit<state, action, environment> *u);
	Unit<state, action, environment> *GetUnit(unsigned int which);

	int AddUnitGroup(UnitGroup<state, action, environment> *ug);
	UnitGroup<state, action, environment> *GetUnitGroup(unsigned int which);
	void ClearAllUnits();
	
	environment *GetEnvironment() { return env; }
		
	void StepTime(double);
	double GetSimulationTime() { return currTime; }
	void SetStepType(tTimestep step) { stepType = step; }
	tTimestep GetStepType() { return stepType; }
	void SetPaused(bool val) { paused = val; }
	bool GetPaused() { return paused; }

	bool Done();

	/** setPenalty for thinking. Sets the multiplier used to penalize thinking time. */
	void SetThinkingPenalty(double pen) { penalty = pen; }
	/** getPenalty for thinking. Gets the multiplier used to penalize thinking time. */
	double GetThinkingPenalty() { return penalty; }

	virtual void OpenGLDraw(int window);
	
	StatCollection* GetStats() { return &stats; }
protected:
	void StepUnitTime(UnitInfo<state, action, environment> *ui, double timeStep);
	bool MakeUnitMove(UnitInfo<state, action, environment> *theUnit, action where, double &moveCost);

	virtual void DoPreTimestepCalc();
	virtual void DoTimestepCalc(double amount);
	virtual void DoPostTimestepCalc();
	
	double penalty;
	bool paused;
	std::vector<UnitInfo<state, action, environment> *> units;
	std::vector<UnitGroup<state, action, environment> *> unitGroups;
	environment *env;
	double currTime, viewTime;
	tTimestep stepType;
	StatCollection stats;
};

template<class state, class action, class environment>
UnitSimulation<state, action, environment>::UnitSimulation(environment *se)
{
	env = se;
	stepType = kRealTime;
	currTime = 0.0;
	paused = false;
	unitGroups.push_back(new UnitGroup<state, action, environment>);
	// allocate default unit group!(?)
}

template<class state, class action, class environment>
int UnitSimulation<state, action, environment>::AddUnit(Unit<state, action, environment> *u)
{
	UnitInfo<state, action, environment> *ui = new UnitInfo<state, action, environment>();
	ui->agent = u;
	u->GetLocation(ui->startState);
	ui->currentState = ui->startState;
	if (ui->agent->GetUnitGroup() == 0)
	{
		ui->agent->SetUnitGroup(unitGroups[0]);
	}
	ui->agent->GetUnitGroup()->UpdateLocation(ui->agent, env, ui->currentState, true, this);
	ui->nextTime = currTime;
	ui->totalThinking = 0.0;
	ui->totalDistance = 0.0;
	ui->historyIndex = 0;
	
	if (0)//(keepHistory)
	{
		TimeStep<state, action> ts(ui->startState, ui->nextTime);
		ui->stateHistory.push_back(ts);		
	}
	units.push_back(ui);
	return units.size()-1;
}

template<class state, class action, class environment>
Unit<state, action, environment> *UnitSimulation<state, action, environment>::GetUnit(unsigned int which)
{
	if (which < units.size())
		return units[which]->agent;
	return 0;
}

template<class state, class action, class environment>
int UnitSimulation<state, action, environment>::AddUnitGroup(UnitGroup<state, action, environment> *ug)
{
	unitGroups.push_back(ug);
	return unitGroups.size()-1;
}

template<class state, class action, class environment>
UnitGroup<state, action, environment> *UnitSimulation<state, action, environment>::GetUnitGroup(unsigned int which)
{
	if (which < unitGroups.size())
		return units[which];
	return 0;
}

template<class state, class action, class environment>
void UnitSimulation<state, action, environment>::ClearAllUnits()
{
	while (units.size() > 0)
	{
		UnitInfo<state, action, environment> *ui = units.back();
		ui->agent->LogFinalStats(&stats);
		units.pop_back();
		delete ui->agent;
		delete ui;
	}
	while (unitGroups.size() > 0)
	{
		//unitGroups.back()->LogFinalStats(&stats);
		delete unitGroups.back();
		unitGroups.pop_back();
	}
	//unitGroups.push_back(new unitGroup(this));
	viewTime = 0;
	currTime = 0;
}

template<class state, class action, class environment>
bool UnitSimulation<state,action,environment>::Done()
{
	// assumes that all units belong to a unit group
	for(unsigned int i=0; i<unitGroups.size(); i++)
	{
		if(!unitGroups[i]->Done())
			return false;
	}
	return true;
}

//template<class state, class action, class environment>
//double UnitSimulation<state, action, environment>::SetUnitLocation(UnitInfo<state, action, environment> *ui,
//																																	 bool success, bool usetimer)
//{
//	Timer t;
//	if (usetimer)
//	{
//		t.startTimer();
//	}
//	if (ui->agent->getUnitGroup() == 0)
//	{
//		ui->agent->UpdateLocation(ui->currentState, success);
//	}
//	else {
//		u->agent->getUnitGroup()->updateLocation(ui->agent, ui->currentState, success);
//	}
//	if (usetimer)
//	{
//		return t.endTimer();
//	}
//	return 0;
//}

template<class state, class action, class environment>
void UnitSimulation<state, action, environment>::StepTime(double timeStep)
{
	//std::cout<<"StepTime\n";
	if (paused)
	{
		return;
	}
	DoPreTimestepCalc();
	stats.AddStat("simulationTime", "UnitSimulation", currTime);
	currTime += timeStep;
	DoTimestepCalc(timeStep);
	DoPostTimestepCalc();
	
	//std::cout<<"currTime "<<currTime<<std::endl;
	//if(Done()) std::cout<<"DONE!!!\n";
}

template<class state, class action, class environment>
void UnitSimulation<state, action, environment>::DoPreTimestepCalc()
{
}

template<class state, class action, class environment>
void UnitSimulation<state, action, environment>::DoTimestepCalc(double timeStep)
{
	for (unsigned int x = 0; x < units.size(); x++)
		StepUnitTime(units[x], timeStep);
}

template<class state, class action, class environment>
void UnitSimulation<state, action, environment>::DoPostTimestepCalc()
{
}

/**
* step time for a single unit.
 *
 * This function takes care of all the simulation details for moving a
 * single unit, doing timing, etc. When overloading advanceTime, this
 * function can be called for each unit that moves.
 */
template<class state, class action, class environment>
void UnitSimulation<state, action, environment>::StepUnitTime(UnitInfo<state, action, environment> *theUnit, double timeStep)
{
	if (currTime < theUnit->nextTime)
	{
		return;
	}

	double moveThinking, locThinking, moveTime;
	action where;
	Timer t;
	Unit<state, action, environment>* u = theUnit->agent;
	
	t.startTimer();
	// need to do if/then check - makemove ok or not? need to stay where you are? 
	if(u->GetUnitGroup()->MakeMove(u, env, this,where))
	{
		moveThinking = t.endTimer();
		theUnit->totalThinking += moveThinking;
		theUnit->lastMove = where;
		stats.AddStat("MakeMoveThinkingTime", u->GetName(), moveThinking);
	
		bool success = MakeUnitMove(theUnit, where, moveTime);

		t.startTimer();
		u->GetUnitGroup()->UpdateLocation(theUnit->agent, env, theUnit->currentState, success, this);
		locThinking = t.endTimer();
		theUnit->totalThinking += locThinking;
		stats.AddStat("UpdateLocationThinkingTime", u->GetName(), locThinking);

		switch (stepType)
		{
			case kLockStep:
				theUnit->nextTime = currTime + timeStep;
				break;
			case kRealTime:
				theUnit->nextTime += (locThinking+moveThinking)*penalty + moveTime;
				break;
			case kMinTime:
				theUnit->nextTime += min((locThinking+moveThinking)*penalty + moveTime,	 timeStep);
			break;
		}
		//u->GetUnitGroup()->logStats(&stats);
		//u->logStats(&stats);
	}
	else // stay where you are
	{
		theUnit->nextTime += theUnit->agent->GetSpeed();
		theUnit->agent->GetUnitGroup()->UpdateLocation(theUnit->agent, env, theUnit->currentState, true, this);	
	}

}

template<class state, class action, class environment>
bool UnitSimulation<state, action, environment>::MakeUnitMove(UnitInfo<state, action, environment> *theUnit, action where, double &moveCost)
{
	bool success = false;
	moveCost = 0;
	state oldState = theUnit->currentState;
	env->ApplyAction(theUnit->currentState, where);
	OccupancyInterface<state, action> *envInfo = env->GetOccupancyInfo();
	
	if ((!envInfo) || (envInfo->CanMove(oldState, theUnit->currentState)))
	{	
		success = true;
		moveCost = env->GCost(oldState, theUnit->currentState)*theUnit->agent->GetSpeed();
		theUnit->totalDistance += env->GCost(oldState, theUnit->currentState);
		
		if (envInfo)
		{
// update here!
			envInfo->MoveUnitOccupancy(oldState, theUnit->currentState);
			//envInfo->SetStateOccupied(oldState, false);
			//envInfo->SetStateOccupied(theUnit->currentState, true);
		}
	}
	else {
		if(!envInfo)
			success = true;
		else
			success = false;
			
		theUnit->currentState = oldState;
	}
	
	return success;
}

template<class state, class action, class environment>
void UnitSimulation<state, action, environment>::OpenGLDraw(int window)
{
	env->OpenGLDraw(window);
	for (unsigned int x = 0; x < units.size(); x++)
	{
		units[x]->agent->OpenGLDraw(window, env, this);
	}
	
	for (unsigned int x = 0; x <unitGroups.size(); x++)
		unitGroups[x]->OpenGLDraw(window,env,this);
}

//
//
//class unitSimulation : public MapProvider, reservationProvider, public SimulationInfo {
//public:
//	unitSimulation(MapAbstraction *, bool keepStats = false);
//	virtual ~unitSimulation();
//	
//	bool saveHistory(char *, bool includeMap = true);
//	bool loadHistory(char *);
//	
//	virtual void addUnit(unit *);
//	virtual void addUnit(unit *, bool block);
//	virtual void addUnitGroup(unitGroup *);
//
//	unitGroup *getUnitGroup(int which);
//	unit *getUnit(int which);
//	unit *findUnit(int x, int y);
//
//	void clearAllUnits();
//
//	virtual void advanceTime(double amount);
//	void setSimulationPaused(bool val) { pause = val; }
//	bool getSimulationPaused() { return pause; }
//	/** return the current inside the simulation */
//	double getSimulationTime() { return currTime; }
//	/** return the current time being drawn */
//	double getDisplayTime() { return viewTime; }
//	void setDisplayTime(double val);
//	void offsetDisplayTime(double val);
//	
//	
//	// simplify timing modes
//	void setRealTime(bool);
//	bool getRealTime() { return realTime; }
//	inline void setLockstepTime(bool b) { lockstepTime = b; }
//	bool getLockstepTime() { return lockstepTime; }
//	/** Set if the simulation is asynchronous. */
//	void setAsynchronous() { asynch = true; }
//	void setSynchronous() { asynch = false; }
//
//
//	void OpenGLDraw();
//	void print(bool forceOutput = true);
//	virtual bool done();
//
//	/** turns unit blocking on and off */
//	void setUseBlocking(bool val) { blocking = val; }
//	
//	/** setPenalty for thinking. Sets the multiplier used to penalize thinking time. */
//	void setPenalty(double pen) { penalty = pen; }
//	/** getPenalty for thinking. Gets the multiplier used to penalize thinking time. */
//	double getPenalty() { return penalty; }
//
//	
//	//////////////////////////////////////////////////
//	// this may move to the environment -- have to check
//	/** reservationProvider interface */
//	inline bool nodeOccupied(node *currNode)
//	{ if (currNode->GetLabelL(kAbstractionLevel) != 0) return false;
//		return tileOccupied((unsigned int)currNode->GetLabelL(kFirstData),
//		                    (unsigned int)currNode->GetLabelL(kFirstData+1)); }
//	inline bool tileOccupied(int x, int y) { return bv->get(y*map_width+x); }
//	/* temporal reservations not supported by unitSimulation */
//	virtual bool canMove(node *, node *, double, unit *) { return true;}
//	virtual bool reserveMove(node *, node *, double, unit *) { return true; }
//	virtual bool clearMove(node *, node *, double, unit *) { return true; }
//	virtual void clearAllReservations() {}
//
//	
//	/** Toggle open GL display */
//	void toggleNoOpenGLDraw() { noOpenGLDraw = !noOpenGLDraw; }
//	//void setLogFile(FILE *);
//	
//	StatCollection *getStats() { return &stats; }
//	void printCollectedStats(bool v) { stats.enablePrintOutput(v); }
//
//
//	//////////////////////////////////////////////////
//	// environment variables, don't belong in unit simulation!
//	void setmapAbstractionDisplay(int _whichMap=kUnitSimulationMap);
//	/** Return which map is being currently displayed. */
//	int getDisplayMapNumber() { return which_map; }
//	
//	/** Returns the underlying map. */
//	Map *GetMap() { return map; }
//	/** Returns the abstract map from the simulation. */
//	MapAbstraction *GetMapAbstraction();
//	/** set chance for move failing. This is the chance that a move will just fail. */
//	/** Returns the nth groups abstract map. (0 is the actual map of the world.) */
//	MapAbstraction *GetMapAbstraction(int _which);
//	/** Returns the abstract map currently being displayed. */
//	MapAbstraction *getMapAbstractionDisplay();
//	/** Cycle which abstract map should be displayed. */
//	void cyclemapAbstractionDisplay();
//	void setMoveStochasticity(double _stochasticity) { stochasticity = _stochasticity; }
//	double getMoveStochasticity() { return stochasticity; }
//	bool canCrossDiagonally() { return (!disallowDiagonalCrossingMoves); }
//	void setCanCrossDiagonally(bool cross) { disallowDiagonalCrossingMoves = !cross; }
//	void getRandomLocation(int &x, int &y, tTerrain terrain = kGround);
//	void getRandomLocation(int x1, int y1, int &x2, int &y2, tTerrain terrain = kGround);
//	void getRandomLocations(int &x1, int &y1, int &x2, int &y2, tTerrain terrain = kGround);
//	
//	//////////////////////////////////////////////////
//	// ???
//	bool setIgnoreOnTarget(unit*,bool);
//
//protected:
//	unitInfo *findUnit(unit *);	
//	virtual void doPreTimestepCalc();
//	virtual void doTimestepCalc();
//	virtual void doPostTimestepCalc();
//	void stepUnitTime(unitInfo *);
//	void setAgentLocation(unitInfo *, bool success = false, bool timer = false);
//	void updateMap();
//	bool updatemapAbstraction();
//	void drawBlockedSquare(int x, int y);
//// 	void startTimer();
//// 	double endTimer();
//	bool findUnitDisplayTime(unitInfo *ui);
//	
//	Map *map;
//	//FILE *LOGFILE;
//	MapAbstraction *aMap;
//	bitVector *bv;
//	int which_map;						// the number of the group to display info for
//	int map_width, map_height, map_revision;
//	std::vector<unitInfo *> units;
//	std::vector<unitInfo *> displayUnits;
//	std::vector<unitGroup *> unitGroups;
//	std::priority_queue<const unitInfo*, std::vector<unitInfo *>, unitInfoCompare> moveQ;
//	double currTime, viewTime;
//	bool asynch;
//	bool blocking;		// this is the default for all units added
//	bool realTime;
//	bool pause;
//	bool lockstepTime;      // Finn/Wes - individual unit times will be update exactly according to the amount specified to advanceTime
//	double penalty;
//	double stochasticity;
//	bool unitsMoved;
//	bool disallowDiagonalCrossingMoves;
//	double currDist;        // total distance moved so far
//	bool noOpenGLDraw;		// turns display on/off
//	bool keepHistory; // keep action history
//	
//	StatCollection stats;
//};
//
//#define LOCAL_PATH

#endif

//
//struct xyLoc {
//	uint16_t x;
//	uint16_t y;
//};
//
//class MapEnvironment : public SearchEnvironment<xyLoc, tDirection>
//{
//public:
//	MapEnvironment(Map *m);
//private:
//	Map *m;
//};
//
//class AbstractMapEnvironment : public SearchEnvironment<node *, edge *>
//{
//public:
//	AbstractMapEnvironment(MapAbstraction *);
//private:
//	MapAbstraction *ma;
//};
//
//class AbstractMapUnit : public Unit<node *, edge *>
//{
//}
