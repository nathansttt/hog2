/*
 * $Id: sharedAMapGroup.h,v 1.9 2006/09/18 06:19:31 nathanst Exp $
 *
 *  sharedAMapGroup.h
 *  HOG
 *
 *  Created by Nathan Sturtevant on 12/16/04.
 *  Copyright 2004 University of Alberta. All rights reserved.
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


#include "unitGroup.h"

#ifndef SHAREDAMAPGROUP_H
#define SHAREDAMAPGROUP_H

/**
 * A group which incrementally builds a map of the world as the units in the group
 * explore the graph.
 */
class sharedAMapGroup : public unitGroup, public MapProvider {
public:
	sharedAMapGroup(MapProvider *);
	~sharedAMapGroup();
	virtual tDirection makeMove(unit *u, MapProvider *mp, reservationProvider *rp, simulationInfo *simInfo);
	virtual void OpenGLDraw(MapProvider *, simulationInfo *);
	virtual Map *getMap();
	virtual mapAbstraction *getMapAbstraction();
	virtual int getNewTileCount() { return newTileCountPerTrial; }
	
	/** reset the location of a given unit */
	virtual void updateLocation(unit *, MapProvider *m, int _x, int _y, bool, simulationInfo *);
	/** Is the group done with their exploration? */
	virtual bool done();
	/** Lets the unit group do what it needs to reset a trial */
	void startNewTrial(StatCollection *stats);
	void logStats(StatCollection *stats);
	
	void setVisibilityRadius(int _visibility);
	int getVisibilityRadius();
	bool explored(int x, int y);
	bool explored(unsigned int _node);
	//void printRoundStats(unit *, FILE *f);

        int getNewTileCountPerStep() { return newTileCount; }

        bool seenBefore(int x, int y) { return (seen->get(y * map->getMapWidth() + x)); }
	
protected:
	//void setUnitSimulation(unitSimulation *_us, Map *m);

	mapAbstraction *aMap;
	Map *map;
	bitVector *seen;
	int visRadius;
	bool sawNewLand;
	int newTileCount;
	int newTileCountPerTrial;
};

#endif
