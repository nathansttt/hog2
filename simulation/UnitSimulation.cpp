/*
 * $Id: unitSimulation.cpp,v 1.50 2006/11/23 16:47:07 bulitko Exp $
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

//#include "UnitSimulation.h"
//#include "Unit.h"
//#include "UnitGroup.h"
//#include "FPUtil.h"
//#include "Timer.h"
//
//const bool verbose = false;
//
////bool unitInfoCompare::operator()(const unitInfo* u1, const unitInfo* u2)
////{
////	//	printf("Comparing %1.2f to %1.2f\n", u1->nextTime, u2->nextTime);
////	return (fgreater(u1->nextTime,  u2->nextTime));
////}
//
///** construct a unit simulation.
//*
//* construct a unit simulation around a given map.
//*/
//unitSimulation::unitSimulation(MapAbstraction *_aMap, bool keepStats)
//:map_revision(-1)
//{                                                                                
//	aMap = _aMap;
//	map = aMap->GetMap();
//	which_map = kUnitSimulationMap;
//	currTime = 0;
//	map_height = map->getMapHeight();
//	map_width = map->getMapWidth();
//	bv = new bitVector(map_height*map_width);
//	map_revision = map->getRevision();
//	//aMap = 0;
//	penalty = 1.0;
//	asynch = true;
//	blocking = true;
//	realTime = true;
//	pause = false;
//	noOpenGLDraw = false;
//	stochasticity = 0;
//	unitGroups.push_back(new unitGroup(this));
//	unitsMoved = true;
//	disallowDiagonalCrossingMoves = false;
//	lockstepTime = false;
//	keepHistory = keepStats;
//}
//
///** delete a unit simulation.
//*
//* The unit simulation will delete all units (display and otherwise) as well
//* as all groups that have been added to the simulation.
//*/
//unitSimulation::~unitSimulation()
//{
//	delete aMap;
//	aMap = 0;
//	delete bv;
//	bv = 0;
//	// the units display list is tied to this map/simulation size,
//	// so we should clear it whe the simulation is destroyed
//	unit::clearDisplayList();
//	while (units.size() > 0)
//	{
//		unitInfo *ui = units.back();
//		units.pop_back();
//		
//		ui->agent->LogFinalStats(&stats);
//		
//		delete ui->agent;
//		delete ui;
//	}
//	while (displayUnits.size() > 0)
//	{
//		unitInfo *ui = displayUnits.back();
//		displayUnits.pop_back();
//		delete ui->agent;
//		delete ui;
//	}
//	while (unitGroups.size() > 0)
//	{
//		unitGroups.back()->LogFinalStats(&stats);
//		delete unitGroups.back();
//		unitGroups.pop_back();
//	}
//}
//
///** Save the history of actions that have run in this simulation.
//*
//* Returns true if the history was saved properly. The simluation must
//* be initialized with the keepStats value if the history is to be saved.
//* If includeMap is true, the map will always be included in the save file.
//* Otherwise it will only be included if a valid path isn't available.
//*/
//const float VERSION = 1.0;
//bool unitSimulation::saveHistory(char *fname, bool includeMap)
//{
//	FILE *f = fopen(fname, "w+");
//	if (f == 0)
//		return false;
//	fprintf(f, "VERSION %1.2f\n", VERSION);
//	//fprintf(f, "MAP %s (%dx%d)\n", );
//	fprintf(f, "UNITS %d\n", (int)units.size());
//	fprintf(f, "DISPLAY UNITS %d\n", (int)displayUnits.size());
//	for (unsigned int x = 0; x < units.size(); x++)
//	{
//		float r, g, b;
//		units[x]->agent->getColor(r, g, b);
//		fprintf(f, "UNIT %d ACTIONS %d COLOR %f %f %f\n",
//						x, (int)units[x]->actionHistory.size(), r, g, b);
//		fwrite(&units[x]->actionHistory[0], sizeof(timeStep), 
//					 units[x]->actionHistory.size(), f);
//		fprintf(f, "\n");
//	}
//	for (unsigned int x = 0; x < displayUnits.size(); x++)
//	{
//		float r, g, b;
//		displayUnits[x]->agent->getColor(r, g, b);
//		fprintf(f, "DUNIT %d ACTIONS %d COLOR %f %f %f\n",
//						x, (int)displayUnits[x]->actionHistory.size(), r, g, b);
//		fwrite(&displayUnits[x]->actionHistory[0], sizeof(timeStep), 
//					 displayUnits[x]->actionHistory.size(), f);
//		fprintf(f, "\n");
//	}
//	if ((!includeMap) && (map->getMapName()))
//	{
//		fprintf(f, "MAP %s", map->getMapName());
//	}
//	else {
//		fprintf(f, "MAP INCLUDED\n");
//		map->save(f);
//	}
//	fclose(f);
//	return true;
//}
//
///** Load a history file into the simulation.
//*
//* Returns true if the history loads. Loading a history causes all
//* units currently in the simulation to be removed and turns the history
//* on.
//*
//*/
//bool unitSimulation::loadHistory(char *fname)
//{
//	FILE *f = fopen(fname, "r");
//	if (f == 0)
//		return false;
//	clearAllUnits();
//	float version;
//	fscanf(f, "VERSION %f\n", &version);
//	if (!fequal(VERSION, version))
//	{
//		printf("Invalid version %1.2f in file %s\n", version, fname);
//		return false;
//	}
//	int numUnits, numDisplay;
//	fscanf(f, "UNITS %d\n", &numUnits);
//	fscanf(f, "DISPLAY UNITS %d\n", &numDisplay);
//	for (int x = 0; x < numUnits; x++)
//	{
//		float r, g, b;
//		int numbActions;
//		int sanity;
//		fscanf(f, "UNIT %d ACTIONS %d COLOR %f %f %f\n",
//					 &sanity, &numbActions, &r, &g, &b);
//		assert(sanity == x);
//		unitInfo *ui = new unitInfo();
//		ui->actionHistory.resize(numbActions);
//		fread(&ui->actionHistory[0], sizeof(timeStep), 
//					ui->actionHistory.size(), f);
//		ui->agent = new unit(ui->actionHistory[numbActions-1].x,
//												 ui->actionHistory[numbActions-1].y,
//												 r, g, b);
//		ui->agent->setObjectType(kWorldObject);
//		ui->startx = ui->actionHistory[0].x;
//		ui->starty = ui->actionHistory[0].y;
//		ui->currx = ui->actionHistory[numbActions-1].x;
//		ui->curry = ui->actionHistory[numbActions-1].y;
//		if (fgreater(ui->actionHistory[numbActions-1].startTime, currTime))
//			currTime = ui->actionHistory[numbActions-1].startTime;
//		fscanf(f, "\n");
//		units.push_back(ui);
//	}
//	for (int x = 0; x < numDisplay; x++)
//	{
//		float r, g, b;
//		int numbActions;
//		int sanity;
//		fscanf(f, "DUNIT %d ACTIONS %d COLOR %f %f %f\n",
//					 &sanity, &numbActions, &r, &g, &b);
//		assert(sanity == x);
//		unitInfo *ui = new unitInfo();
//		ui->actionHistory.resize(numbActions);
//		fread(&ui->actionHistory[0], sizeof(timeStep), 
//					ui->actionHistory.size(), f);
//		ui->agent = new unit(ui->actionHistory[numbActions-1].x,
//												 ui->actionHistory[numbActions-1].y,
//												 r, g, b);
//		ui->startx = ui->actionHistory[0].x;
//		ui->starty = ui->actionHistory[0].y;
//		ui->currx = ui->actionHistory[numbActions-1].x;
//		ui->curry = ui->actionHistory[numbActions-1].y;
//		if (fgreater(ui->actionHistory[numbActions-1].startTime, currTime))
//			currTime = ui->actionHistory[numbActions-1].startTime;
//		fscanf(f, "\n");
//		displayUnits.push_back(ui);
//	}
//	char where[128];
//	fscanf(f, "MAP %s\n", where);
//	//delete aMap;
//	MapAbstraction *tmp = aMap;
//	if (strcmp(where, "INCLUDED") == 0)
//	{
//		map = new Map(f);
//	}
//	else {
//		map = new Map(where);
//	}
//	//map->setDrawLandWhite(tmp->GetMap()->getDrawLandWhite());
//	map->setTileSet(tmp->GetMap()->getTileSet());
//	aMap = aMap->clone(map);
//	delete tmp;
//	// create map abstraction?!?
//	fclose(f);
//	viewTime = currTime;
//	setSimulationPaused(true);
//	return true;
//}
//
//
///** add a unit to the simulation.
//*
//* Adds a unit to the simulation. If the unit's current x/y location is valid,
//* the unit will be placed there. Otherwise it will be added to a random location
//* on the map. If a unit is part of a group, it should be added to that group
//* before being added to the simulation.
//*/
//void unitSimulation::addUnit(unit *u)
//{
//	//u->setUnitSimulation(this);
//	unitInfo *ui = new unitInfo();
//	ui->lastMove = kStay;
//	ui->agent = u;
//	ui->blocking = blocking;
//	ui->ignoreOnTarget = false;
//	u->getLocation(ui->currx, ui->curry);
//	
//	if ((ui->currx < 0) || (ui->currx >= map_width) ||
//			(ui->curry < 0) || (ui->curry >= map_height) ||
//			(bv->get(ui->curry*map_width+ui->currx)))
//	{
//		if (verbose)
//			printf("Warning -- unit in illegal location; resetting unit to random location\n");
//		getRandomLocation(ui->currx, ui->curry);
//	}
//	else if (verbose)
//		printf("Adding unit at (%d, %d)\n", ui->currx, ui->curry);
//	
//	if (u->getObjectType() != kDisplayOnly)
//	{
//		if (blocking)
//			bv->set(ui->curry*map_width+ui->currx, 1);
//	}
//	ui->startx = ui->currx;
//	ui->starty = ui->curry;
//	setAgentLocation(ui);
//	ui->nextTime = currTime;
//	ui->thinkTime = 0.0;
//	//ui->memory = 0.0;
//	ui->moveDist = 0.0;
//	//	ui->firstMoveThinkTime = -1.0;
//	ui->historyIndex = 0;
//	
//	if (u->getObjectType() == kDisplayOnly)
//	{
//		displayUnits.push_back(ui);
//	}
//	else {
//		if (u->getUnitGroup() == 0)
//		{
//			u->setUnitGroup(unitGroups[0]);
//			unitGroups[0]->addUnit(u);
//		}
//		units.push_back(ui);
//		// add unit to global move queue
//		moveQ.push(ui);
//	}
//	if (keepHistory)
//	{
//		timeStep ts(ui->startx, ui->starty, ui->nextTime);
//		ui->actionHistory.push_back(ts);		
//	}
//	// printf("Added unit -- # of units is %d\n", (int)units.size());
//}
//
///** add a unit to the simulation.
//*
//* Adds a unit to the simulation. You can optionally set whether this unit
//* blocks the places where it stands, which can differ from the simulation default.
//* If the unit's current x/y location is valid,
//* the unit will be placed there. Otherwise it will be added to a random location
//* on the map. If a unit is part of a group, it should be added to that group
//* before being added to the simulation.
//*/
//void unitSimulation::addUnit(unit *u, bool _blocking)
//{
//	//u->setUnitSimulation(this);
//	unitInfo *ui = new unitInfo();
//	ui->lastMove = kStay;
//	ui->agent = u;
//	ui->blocking = _blocking;
//	ui->ignoreOnTarget = false;
//	u->getLocation(ui->currx, ui->curry);
//	
//	if ((ui->currx < 0) || (ui->currx >= map_width) ||
//			(ui->curry < 0) || (ui->curry >= map_height) ||
//			(bv->get(ui->curry*map_width+ui->currx)))
//	{
//		getRandomLocation(ui->currx, ui->curry);
//		if (verbose)
//			printf("Warning -- unit in illegal location; resetting unit to random location\n");
//	}
//	else if (verbose)
//		printf("Adding unit at (%d, %d)\n", ui->currx, ui->curry);
//	
//	if (u->getObjectType() != kDisplayOnly)
//	{
//		if (ui->blocking)
//			bv->set(ui->curry*map_width+ui->currx, 1);
//	}
//	ui->startx = ui->currx;
//	ui->starty = ui->curry;
//	setAgentLocation(ui, true);
//	ui->nextTime = currTime;
//	ui->thinkTime = 0.0;
//	//ui->memory = 0.0;
//	ui->moveDist = 0.0;
//	//ui->firstMoveThinkTime = -1.0;
//	ui->historyIndex = 0;
//	
//	if (u->getObjectType() == kDisplayOnly)
//	{
//		displayUnits.push_back(ui);
//	}
//	else {
//		if (keepHistory)
//		{
//			timeStep ts(ui->startx, ui->starty, ui->nextTime);
//			ui->actionHistory.push_back(ts);		
//		}
//		units.push_back(ui);
//		if (u->getUnitGroup() == 0)
//			u->setUnitGroup(unitGroups[0]);
//		// add unit to global move queue
//		moveQ.push(ui);
//	}
//}
//
//unitInfo *unitSimulation::findUnit(unit *u)
//{
//	for (unsigned int x = 0; x < units.size(); x++)
//		if (units[x]->agent == u)
//			return units[x];
//	for (unsigned int x = 0; x < displayUnits.size(); x++)
//		if (displayUnits[x]->agent == u)
//			return displayUnits[x];
//	return 0;
//}
//
///**
//* Find a unit in the world.
// * Return the unit on location x, y, if there is one. Otherwise returns NULL.
// */
//unit *unitSimulation::findUnit(int x, int y)
//{
//	for (unsigned int t = 0; t < units.size(); t++)
//		if ((units[t]->currx == x) && (units[t]->curry == y))
//			return units[t]->agent;
//	for (unsigned int t = 0; t < displayUnits.size(); t++)
//		if ((displayUnits[t]->currx == x) && (displayUnits[t]->curry == y))
//			return displayUnits[t]->agent;
//	
//	return 0;
//}
//
///**
//* clear all units in simulation.
// *
// * removes and deletes all units & unit groups from the simulation.
// */
//void unitSimulation::clearAllUnits()
//{
//	which_map = kUnitSimulationMap;
//	bv->clear();
//	unit::clearDisplayList();
//	while (!moveQ.Empty())
//		moveQ.pop();
//	while (units.size() > 0)
//	{
//		unitInfo *ui = units.back();
//		ui->agent->LogFinalStats(&stats);
//		units.pop_back();
//		delete ui->agent;
//		delete ui;
//	}
//	while (displayUnits.size() > 0)
//	{
//		unitInfo *ui = displayUnits.back();
//		displayUnits.pop_back();
//		delete ui->agent;
//		delete ui;
//	}
//	while (unitGroups.size() > 0)
//	{
//		unitGroups.back()->LogFinalStats(&stats);
//		delete unitGroups.back();
//		unitGroups.pop_back();
//	}
//	unitGroups.push_back(new unitGroup(this));
//	viewTime = 0;
//	currTime = 0;
//}
//
///**
//* Add unit group to simulation.
// *
// * Adds a unit group to the simulation. Unit groups are in simulation mostly
// * for memory management and display purposes.
// */
//void unitSimulation::addUnitGroup(unitGroup *ug)
//{
//	//updateMap();
//	//ug->setUnitSimulation(this, map);
//	unitGroups.push_back(ug);
//}
//
///**
//* Get the nth unit group.
// */
//unitGroup *unitSimulation::getUnitGroup(int which)
//{
//	if ((which < 0) || ((unsigned int)which >= unitGroups.size()))
//		return NULL;
//	return unitGroups[which];
//}
//
///**
//* Get the nth unit
// */
//unit *unitSimulation::getUnit(int which)
//{
//	if ((which < 0) || ((unsigned int)which >= units.size()))
//		return NULL;
//	return units[which]->agent;
//}
//
//
///**
//* Set ignoreOnTatget status of nth unit
// */
//bool unitSimulation::setIgnoreOnTarget(unit* u, bool x)
//{
//	unitInfo* ui = findUnit(u);
//	if (ui == NULL) return false;
//	ui->ignoreOnTarget = x;
//	return true;
//}
//
//
///**
//* Set which map is returned from GetMapAbstraction.
// *
// * if you want to view the map of one of the groups instead of
// * the actual world, you can use thsi function to choose.
// * The default value is the actual world map. Pass 1..n to select the 1st to nth
// * unit group's map.
// */
//void unitSimulation::setmapAbstractionDisplay(int _whichMap)
//{
//	which_map = _whichMap;
//	if (which_map == kUnitSimulationMap)
//		return;
//	if ((which_map-1 >= (int)unitGroups.size()) || (which_map-1 < 0))
//		which_map = kUnitSimulationMap;
//}
//
///**
//* cycle the map returned from GetMapAbstraction.
// *
// * selects the next possibile abstract map to be returned from GetMapAbstraction.
// */
//void unitSimulation::cyclemapAbstractionDisplay()
//{
//	setmapAbstractionDisplay(which_map+1);
//}
//
///**
//* Determines whether the simulation strives to run in "real" time or
// * be more accurate.
// *
// * In real-time mode each unit get a maximum of 1 move each time step,
// * and the current time for any unit is set to the current time after any time
// * step, so that things run more in real-time.
// *
// * In non-real-time mode the simulation allows units to run until they have
// * used up all the time they are alloted. This may run as much as nx slower than
// * real time mode, if all units are using up all their time.
// *
// * If a unit wants to stay, they forfeit the rest of their turn time. This has
// * implications when units are chasing each other around, but really helps when
// * units are sitting around doing nothing.
// */
//void unitSimulation::setRealTime(bool _realTime)
//{
//	realTime = _realTime;
//}
//
///** Set the time currently displayed
//*
//* When a history is kept, this allows you to shift the display to any time.
//* Values below 0 will be truncated to 0. Values above the current simulation time
//* will be set to the current simulation time. Ignored if history isn't being
//* kept.
//*/
//void unitSimulation::setDisplayTime(double val)
//{
//	if (!keepHistory)
//		return;
//	if (fless(val, 0))
//		viewTime = 0.0;
//	else if (fgreater(val, currTime))
//		viewTime = currTime;
//	else
//		viewTime = val;
//}
//
///** Offset the display time
//*
//* When a history is kept, this allows you to offset the displayed time.
//* New values for the view time below 0 will be truncated to 0. Values above
//* the current simulation time will be set to the current simulation time.
//*/
//void unitSimulation::offsetDisplayTime(double val)
//{
//	if (!keepHistory)
//		return;
//	setDisplayTime(viewTime+val);
//}
//
//
///**
//* advance time in the simulation.
// *
// * This is the basic function for stepping time in the simulation.
// * The simulation will advance all units by this amount
// * (seconds -- although really unitless)
// * If we are in real-time mode, each unit will get one move per this time step.
// * Otherwise, we will step units iteratively until they have all moved as often
// * as they can in this amount of time. Once a unit returns kStay they are giving
// * up any movement until this time step is finished.
// */
//void unitSimulation::advanceTime(double amount)
//{
//	if (pause)
//		return;
//	
//	stats.AddStat("simulationTime", "unitSimulation", currTime);
//	doPreTimestepCalc();
//	currTime += amount;
//	doTimestepCalc();
//	doPostTimestepCalc();
//	viewTime = currTime;
//}
//
//void unitSimulation::doPreTimestepCalc()
//{
//	if (units.size() > 0)
//		unitsMoved = false;
//	updateMap();
//}
//
//void unitSimulation::doTimestepCalc()
//{
//	double finalTime = currTime;
//	
//	if (verbose)
//		printf("Running simulation step at time %1.2f\n", currTime);
//	
//	for (unsigned int t = 0; t < unitGroups.size(); t++)
//	{
//		// for the moment this isn't timed, but it should be, and the time spent
//		// should be charged across all units in the group
//		unitGroups[t]->think(this);
//	}
//	
//	if (lockstepTime)
//	{
//		for (unsigned int t = 0; t < units.size(); t++)
//		{
//			stepUnitTime(units[t]);
//			units[t]->nextTime = currTime;
//		}
//		return;
//	}
//	
//	if (moveQ.Empty())
//		return;
//	
//	unitInfo *next = moveQ.top();
//	while (fless(next->nextTime, finalTime))
//	{
//		moveQ.pop();
//		
//		// this the actual time when the unit moves
//		currTime = next->nextTime;
//		if (verbose) printf("Moving %s at time %1.2f\n", next->agent->GetName(), currTime);
//		stepUnitTime(next);
//		
//		if ((fless(next->nextTime, finalTime)) && (realTime))
//			next->nextTime = finalTime;
//		
//		moveQ.push(next);
//		next = moveQ.top();
//	}
//	
//	// jump to appropriate time for next unit, if needed
//	if ((!realTime) && (next))
//		currTime = next->nextTime;
//	else
//		currTime = finalTime;
//	
//	stats.AddStat("simulationTime", "unitSimulation", currTime);
//}
//
//void unitSimulation::doPostTimestepCalc()
//{
//}
//
///**
//* step time for a single unit.
// *
// * This function takes care of all the simulation details for moving a
// * single unit, doing timing, etc. When overloading advanceTime, this
// * function can be called for each unit that moves.
// */
//void unitSimulation::stepUnitTime(unitInfo *theUnit)
//{
//	double thinkingCost;
//	tDirection where;
//	unit* u = theUnit->agent;
//	
//	if (currTime < theUnit->nextTime)
//	{
//		return;
//	}
//	
//	//	if (verbose) printf("[%d] is moving\n", t);
//	
//	//	if (blocking && (bv->get(theUnit->curry*map_width+theUnit->currx) != 1))
//	//		printf("Unit [%d] should be blocked and isn't\n", t);
//	
//	// Start the timer
//	Timer t;
//	t.startTimer();
//	
//	// Is this unit a part of a group or freelance?
//	if (u->getUnitGroup() != NULL)
//	{
//		where = u->getUnitGroup()->makeMove(u, this, this, this);
//		theUnit->lastMove = where;
//		// group memory usage should be calculated separately from each unit
//		//theUnit->memory = u->getMemoryUsage();
//	}
//	else {
//		where = u->makeMove(this, this, this);
//		theUnit->lastMove = where;
//		//theUnit->memory = u->getMemoryUsage();
//	}
//	
//	thinkingCost = t.endTimer();
//	theUnit->thinkTime += thinkingCost;
//	stats.AddStat("makeMoveThinkingTime", u->GetName(), thinkingCost);
//	
//	if (asynch)
//		theUnit->nextTime += unitSimulation::penalty*thinkingCost;
//	
//	//	if (fless(theUnit->firstMoveThinkTime,0.0)) {
//	//		theUnit->firstMoveThinkTime = thinkingCost;
//	//		if (verbose) 
//	//			printf("Unit %s made 1st move in %f ms\n",theUnit->agent->GetName(),thinkingCost);
//	//	}
//	
//	// The unit wants to stay put
//	if (where == kStay)
//	{
//		theUnit->nextTime += theUnit->agent->getSpeed();
//		setAgentLocation(theUnit, true, true); // move was successful; update time
//																					 // printf("unit %s wants to stay\n",u->GetName());
//		if (keepHistory)
//		{
//			timeStep ts(theUnit->currx, theUnit->curry, theUnit->nextTime);
//			theUnit->actionHistory.push_back(ts);		
//		}
//		u->logStats(&stats);
//		u->getUnitGroup()->logStats(&stats);
//		return;
//	}
//	
//	// The unit is teleporting
//	if (where == kTeleport)
//	{
//		int n1, n2;
//		u->getLocation(n1, n2);
//		if (!bv->get(n2*map_width+n1))
//		{
//			if (theUnit->blocking) 
//				bv->set(theUnit->curry*map_width+theUnit->currx, 0);
//			
//			theUnit->currx = n1;
//			theUnit->curry = n2;
//			setAgentLocation(theUnit, true, true);  // move was successful; update time
//			if (keepHistory)
//			{
//				timeStep ts(theUnit->currx, theUnit->curry, theUnit->nextTime);
//				theUnit->actionHistory.push_back(ts);		
//			}
//			
//			if (theUnit->blocking) 
//				bv->set(theUnit->curry*map_width+theUnit->currx, 1);
//		}
//		unitsMoved = true;
//		
//		u->logStats(&stats);
//		u->getUnitGroup()->logStats(&stats);
//		return;
//	}
//	
//	
//	// The unit is moving
//	int newx = theUnit->currx + ((where&kE)?1:0) - ((where&kW)?1:0);
//	int newy = theUnit->curry + ((where&kS)?1:0) - ((where&kN)?1:0);
//	
//	// Check if the move is valid
//	//if ((map->getTerrainType(newx, newy)>>terrainBits) == (kGround>>terrainBits))
//	//if (map->canStep(theUnit->currx, theUnit->curry, newx, newy))
//	if (aMap->GetAbstractGraph(0)->FindEdge(map->getNodeNum(theUnit->currx, theUnit->curry),
//																					map->getNodeNum(newx, newy)))
//	{
//		if ((stochasticity > 0) && ((random()%1023)/1024.0 < stochasticity))
//		{
//			if (verbose) 
//				printf("HAHA; move failed for %s because of stochastic environment\n",
//							 u->GetName());
//			setAgentLocation(theUnit, false, true); // move was not successful; update time
//			if (keepHistory)
//			{
//				timeStep ts(theUnit->currx, theUnit->curry, theUnit->nextTime);
//				theUnit->actionHistory.push_back(ts);		
//			}
//			unitsMoved = true;
//		}
//		// if the place we want to move is occupied, we can't go there
//		else if (bv->get(newy*map_width+newx))
//		{
//			if (verbose)
//				printf("Can't move unit %s from (%d, %d) onto other unit (%d, %d); time %1.2f\n",
//							 u->GetName(),theUnit->currx, theUnit->curry, newx, newy, currTime);
//			
//			// if not asynch then this is the same as a stay move: you have to sit and think
//			if (!asynch)
//				theUnit->nextTime += theUnit->agent->getSpeed();
//
//			setAgentLocation(theUnit, false, true); // move was not successful; update time
//			if (keepHistory)
//			{
//				timeStep ts(theUnit->currx, theUnit->curry, theUnit->nextTime);
//				theUnit->actionHistory.push_back(ts);		
//			}
//		}
//		// disallow diagonal moves if either side is blocked
//		else if (disallowDiagonalCrossingMoves &&
//						 ((newx != theUnit->currx) && (newy != theUnit->curry)) &&
//						 ((bv->get(newy*map_width+theUnit->currx) ||
//							 bv->get(theUnit->curry*map_width+newx))))
//		{
//			if (verbose)
//				printf("Can't move unit %s from (%d, %d) diagonally past other unit to (%d, %d); time %1.2f\n",
//							 u->GetName(),theUnit->currx, theUnit->curry, newx, newy, currTime);
//			
//			setAgentLocation(theUnit, false, true); // move was not successful; update time
//			if (keepHistory)
//			{
//				timeStep ts(theUnit->currx, theUnit->curry, theUnit->nextTime);
//				theUnit->actionHistory.push_back(ts);		
//			}
//		}
//		else {
//			double movementCost = 1.0;
//			// diagonal movement
//			if ((newx != theUnit->currx) && (newy != theUnit->curry))
//				movementCost = ROOT_TWO;
//			
//			theUnit->moveDist += movementCost;
//			theUnit->nextTime += movementCost*u->getSpeed();
//			stats.SumStat("distanceMoved", theUnit->agent->GetName(), (double)movementCost);
//			
//			if (theUnit->blocking) 
//				bv->set(theUnit->curry*map_width+theUnit->currx, 0);
//			
//			theUnit->currx = newx;
//			theUnit->curry = newy;
//			setAgentLocation(theUnit, true, true);  // move was successful; update time
//			if (keepHistory)
//			{
//				timeStep ts(theUnit->currx, theUnit->curry, theUnit->nextTime);
//				theUnit->actionHistory.push_back(ts);		
//			}
//			unitsMoved = true;
//			
//			if (theUnit->blocking) 
//				bv->set(theUnit->curry*map_width+theUnit->currx, 1);
//		}
//	}
//	else {
//		if (verbose) 
//			printf("Can't move onto bad terrain\n");
//		// printf("unit %s wants to move into a wall\n",u->GetName());
//		
//		setAgentLocation(theUnit, false, true);  // move was not successful; update time
//		if (keepHistory)
//		{
//			timeStep ts(theUnit->currx, theUnit->curry, theUnit->nextTime);
//			theUnit->actionHistory.push_back(ts);		
//		}
//	}
//	
//	u->logStats(&stats);
//	u->getUnitGroup()->logStats(&stats);
//}
//
///**
//* done returns true when all units/groups say they are done.
// *
// * For episodic tasks we may want to run a set number of times, or until learning
// * converges, etc. This is a way of detecting this so we can stop the simulation or
// * start a new episode.
// */
//bool unitSimulation::done()
//{
//	bool isDone = true;
//	for (unsigned int t = 0; (t < units.size())&&(isDone); t++)
//	{
//		if (units[t]->agent->getUnitGroup() == 0)
//			isDone = units[t]->agent->done();
//		else
//			isDone = units[t]->agent->getUnitGroup()->done();
//	}
//	return isDone;
//	//	return (!unitsMoved);
//}
//
//// void unitSimulation::startTimer()
//// {
//// #ifdef OS_MAC
////   startTime = UpTime();
//// #else
////   // startTime = clock();
//// 	CycleCounter c;
//// 	startTime = c.count();
//// #endif
//// }
//
//// #ifdef linux
//
//// float getCPUSpeed()
//// {
//// 	FILE *f;
//
//// 	static float answer = -1;
//
//// 	if (answer != -1)
//// 		return answer;
//
//// 	f = fopen("/proc/cpuinfo", "r");
//// 	if (f)
//// 	{
//// 		while (!feof(f))
//// 		{
//// 			char entry[1024];
//// 			char temp[1024];
//// 			fgets(entry, 1024, f);
//// 			if (strstr(entry, "cpu MHz"))
//// 			{
//// 				//                              cpu MHz         : 997.399
//// 				float answer;
//// 				sscanf(entry, "%[^0-9:] : %f", temp, &answer);
//// 				//printf("Read CPU speed: %1.2f\n", answer);
//// 				fclose(f);
//// 				return answer;
//// 			}
//// 		}
//// 		fclose(f);
//// 	}
//// 	return 0;
//// }
//
//// #endif
//
//// double unitSimulation::endTimer()
//// {
//// #ifdef OS_MAC
////   AbsoluteTime stopTime = UpTime();
////   Nanoseconds diff = AbsoluteDeltaToNanoseconds(stopTime, startTime);
////   uint64_t nanosecs = UnsignedWideToUInt64(diff);
////   //cout << nanosecs << " ns elapsed (" << (double)nanosecs/1000000.0 << " ms)" << endl;
////   return (double)(nanosecs/1000000000.0);
//// #else
////   CycleCounter c;
////   double diffTime = (double)(c.count() - startTime);
////   const static double ClocksPerSecond = getCPUSpeed() * 1000000.0;
////   return  diffTime / ClocksPerSecond;
//// #endif
//// }
//
//void unitSimulation::setAgentLocation(unitInfo *u, bool success, bool timer)
//{
//	Timer t;
//	if (timer)
//	{
//		t.startTimer();
//	}
//	if (u->agent->getUnitGroup() == 0)
//	{
//		u->agent->updateLocation(u->currx, u->curry, success, this);
//		//u->memory = u->agent->getMemoryUsage();
//	}
//	else {
//		u->agent->getUnitGroup()->updateLocation(u->agent, this, u->currx, u->curry, success, this);
//		//u->memory = u->agent->getUnitGroup()->getMemoryUsage(); 
//	}
//	if (timer)
//	{
//		double thinkingCost = t.endTimer();
//		u->thinkTime += thinkingCost;
//		stats.AddStat("setLocationThinkingTime", u->agent->GetName(), thinkingCost);
//		if (asynch)
//			u->nextTime += unitSimulation::penalty*thinkingCost;
//		
//		//		if (fless(u->firstMoveThinkTime,0.0)) {
//		//			u->firstMoveThinkTime = thinkingCost;
//		//			if (verbose) printf("Unit %s made 1st move in %f ms\n",u->agent->GetName(),thinkingCost);
//		//		}
//	}
//}
//
//
//void unitSimulation::updateMap()
//{
//	//  if (aMap->GetMap()->getRevision() != map_revision)
//	//	{
//	//    bv->clear();
//	//    map_revision = map->getRevision();
//	//
//	//    for (unsigned int t = 0; t < displayUnits.size(); t++) {
//	//			getRandomLocation(displayUnits[t]->currx, displayUnits[t]->curry);
//	//			setAgentLocation(displayUnits[t]);
//	//    }
//	//    for (unsigned int t = 0; t < units.size(); t++) {
//	//			getRandomLocation(units[t]->currx, units[t]->curry);
//	//			units[t]->startx = units[t]->currx;
//	//			units[t]->starty = units[t]->curry;
//	//			setAgentLocation(units[t]);
//	//			
//	//      if (units[t]->agent->getObjectType() != kDisplayOnly) {
//	//				if (units[t]->blocking) 
//	//					bv->set(units[t]->curry*map_width+units[t]->currx, 1);
//	//      }
//	//    }
//	//  }
//}
//
//bool unitSimulation::updatemapAbstraction()
//{
//	//	if (!aMap || (map->getRevision() != map_revision))
//	//	{
//	////		delete aMap;
//	//		if (aMap == 0)
//	//			aMap = new MapAbstraction(map);
//	//		// printf("Rebuilt abstract map!\n");
//	//		return true;
//	//	}
//	return false;
//}
//
//MapAbstraction *unitSimulation::GetMapAbstraction()
//{
//	updateMap();
//	updatemapAbstraction();
//	return aMap;
//}
//
//MapAbstraction *unitSimulation::GetMapAbstraction(int _which)
//{
//	if (_which == kUnitSimulationMap)
//	{
//		updateMap();
//		updatemapAbstraction();
//		return aMap;
//	}
//	if ((_which < 1) || (_which > (int)unitGroups.size()))
//		return 0;
//	return unitGroups[_which-1]->GetMapAbstraction();
//}
//
//MapAbstraction *unitSimulation::getMapAbstractionDisplay()
//{
//	if (which_map == kUnitSimulationMap)
//	{
//		updateMap();
//		updatemapAbstraction();
//		return aMap;
//	}
//	return unitGroups[which_map-1]->GetMapAbstraction();
//}
//
//
//void unitSimulation::getRandomLocation(int &x, int &y, tTerrain terrain)
//{
//	do {
//		x = random()%map_width;
//		y = random()%map_height;
//	} while ((bv->get(y*map_width+x)) ||
//					 (map->getTerrainType(x, y) != terrain));
//}
//
///**
//* Get random location which is guaranteed to be Pathable and which is not
// * occupied by any other objects in the world so far.
// */
//void unitSimulation::getRandomLocations(int &x1, int &y1, int &x2, int &y2, tTerrain terrain)
//{
//	do {
//		x1 = random()%map_width;
//		y1 = random()%map_height;
//		x2 = random()%map_width;
//		y2 = random()%map_height;
//	} while ((map->getTerrainType(x2, y2) != terrain) ||
//					 (map->getTerrainType(x1, y1) != terrain) ||
//					 (!aMap->Pathable(aMap->GetNodeFromMap(x1, y1), aMap->GetNodeFromMap(x2, y2))) ||
//					 findUnit(x2, y2) || findUnit(x1, y1));
//}
//
///**
//* Get random location which is guaranteed to be Pathable from the given location and which is not
// * occupied by any other objects in the world so far.
// */
//void unitSimulation::getRandomLocation(int x1, int y1, int &x2, int &y2, tTerrain terrain)
//{
//	do {
//		x2 = random()%map_width;
//		y2 = random()%map_height;
//	} while ((map->getTerrainType(x2, y2) != terrain) ||
//					 (map->getTerrainType(x1, y1) != terrain) ||
//					 (!aMap->Pathable(aMap->GetNodeFromMap(x1, y1), aMap->GetNodeFromMap(x2, y2))) ||
//					 findUnit(x2, y2) || findUnit(x1, y1));
//}
//
//
//void unitSimulation::OpenGLDraw()
//{
//	if (noOpenGLDraw)
//		return;
//	
//	if (fequal(currTime, viewTime))
//	{
//		for (unsigned int t = 0; t < unitGroups.size(); t++)
//		{
//			if (which_map-1 == (int)t)
//			{
//				unitGroups[t]->OpenGLDraw(this, this);
//			}
//		}
//		for (unsigned int t = 0; t < displayUnits.size(); t++)
//		{
//			displayUnits[t]->agent->OpenGLDraw(this, this);
//		}
//		for (unsigned int t = 0; t < units.size(); t++)
//		{
//			units[t]->agent->OpenGLDraw(this, this);
//		}
//	}
//	else {
//		for (unsigned int t = 0; t < displayUnits.size(); t++)
//		{
//			printf("Handing unit %d\n", t);
//			if (findUnitDisplayTime(displayUnits[t]))
//			{
//				GLfloat r, g, b;
//				GLdouble x, y, z, rad;
//				displayUnits[t]->agent->getColor(r, g, b);
//				glColor3f(r, g, b);
//				printf("Getting location -- (%d, %d)\n", displayUnits[t]->actionHistory[displayUnits[t]->historyIndex].x,
//							 displayUnits[t]->actionHistory[displayUnits[t]->historyIndex].y);
//				map->getOpenGLCoord(displayUnits[t]->actionHistory[displayUnits[t]->historyIndex].x,
//														displayUnits[t]->actionHistory[displayUnits[t]->historyIndex].y,
//														x, y, z, rad);
//				DrawPyramid(x, y, z, rad, rad);
//			}
//		}
//		for (unsigned int t = 0; t < units.size(); t++)
//		{
//			if (findUnitDisplayTime(units[t]))
//			{
//				GLfloat r, g, b;
//				GLdouble x, y, z, rad;
//				units[t]->agent->getColor(r, g, b);
//				glColor3f(r, g, b);
//				map->getOpenGLCoord(units[t]->actionHistory[units[t]->historyIndex].x,
//														units[t]->actionHistory[units[t]->historyIndex].y,
//														x, y, z, rad);
//				drawBox(x, y, z, rad);
//			}
//		}
//	}
//	// test code for drawing the bitmap of blocked squares
//	//	if ((map_height == map->getMapHeight()) && (map_width == map->getMapWidth()))
//	//	{
//	//		for (int xx = 0; xx < map_width; xx++)
//	//			for (int yy = 0; yy < map_height; yy++)
//	//			{
//	//				if (bv->get(yy*map_width+xx))
//	//				{
//	//					drawBlockedSquare(xx, yy);
//	//				}
//	//			}
//	//	}
//}
//
//bool unitSimulation::findUnitDisplayTime(unitInfo *ui)
//{
//	if (fgreater(ui->actionHistory[0].startTime, viewTime))
//		return false;
//	for (ui->historyIndex = 0; ui->historyIndex < ui->actionHistory.size()-1; ui->historyIndex++)
//	{
//		if ((!fgreater(ui->actionHistory[ui->historyIndex].startTime, viewTime)) &&
//				(fgreater(ui->actionHistory[ui->historyIndex+1].startTime, viewTime)))
//		{
//			return true;
//		}
//	}
//	// set to last value
//	ui->historyIndex = ui->actionHistory.size()-1;
//	return true;
//}
//
//void unitSimulation::print(bool forceOutput)
//{
//	static int counter = 0;
//	int scale = (int)(map->getMapHeight()/100)+1;
//	counter++;
//	if (forceOutput)
//		printf("%c[2J", 27);//, 1, 32);		
//		if ((counter%10 != 0) && (!forceOutput))
//			return;
//		if ((counter%10000 == 0) || (forceOutput))
//			map->print(scale);
//		for (unsigned int t = 0; t < displayUnits.size(); t++)
//		{
//			printf("%c[%d;%dm", 27, 1, 32);
//			printf("%c[%d;%dfg", 27, (displayUnits[t]->curry)/(2*scale), (displayUnits[t]->currx+1)/scale);
//		}
//		for (unsigned int t = 0; t < units.size(); t++)
//		{
//			printf("%c[%d;%dm", 27, 1, 31);
//			printf("%c[%d;%df%d", 27, (units[t]->curry)/(2*scale), (units[t]->currx+1)/scale, t);
//		}
//		// reset color to black
//		printf("%c[%d;%dm", 27, 0, 0);
//		// go to bottom of map, in case we do other printing
//		printf("%c[%d;%df", 27, (int)map->getMapHeight()/(2*scale)+1, 1);
//}
//
//void unitSimulation::drawBlockedSquare(int x, int y)
//{
//	GLdouble xx, yy, zz, rad;
//	map->getOpenGLCoord(x, y, xx, yy, zz, rad);
//	glColor4f(.5, .5, .5, .5);
//	
//	glBegin(GL_QUAD_STRIP);
//	glVertex3f(xx-rad, yy, zz-rad);
//	glVertex3f(xx-rad, yy+rad, zz-rad);
//	
//	glVertex3f(xx+rad, yy, zz-rad);
//	glVertex3f(xx+rad, yy+rad, zz-rad);
//	
//	glVertex3f(xx+rad, yy, zz+rad);
//	glVertex3f(xx+rad, yy+rad, zz+rad);
//	
//	glVertex3f(xx-rad, yy, zz+rad);
//	glVertex3f(xx-rad, yy+rad, zz+rad);
//	
//	glVertex3f(xx-rad, yy, zz-rad);
//	glVertex3f(xx-rad, yy+rad, zz-rad);
//	
//	glEnd();
//	
//	glBegin(GL_QUADS);
//	glVertex3f(xx-rad, yy+rad, zz-rad);
//	glVertex3f(xx+rad, yy+rad, zz-rad);
//	glVertex3f(xx+rad, yy+rad, zz+rad);
//	glVertex3f(xx-rad, yy+rad, zz+rad);
//	glEnd();
//}
//
