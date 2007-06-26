/**
* A 2D map environment with edge costs weighted according to the 
* number of times a unit has passed over each edge. 
* 
* @file WeightedMap2DEnvironment.h
* @package hog2
* @brief A 2D map environment with edge costs weighted according to the number of times a unit has passed over each edge. 
* @author Renee Jansen
* @date 06/20/2007
*
* This file is part of HOG2.
* HOG : http://www.cs.ualberta.ca/~nathanst/hog.html
* HOG2: http://code.google.com/p/hog2/
*
* HOG2 is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
* 
* HOG2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with HOG2; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef WEIGHTEDMAP2DENVIRONMENT_H
#define WEIGHTEDMAP2DENVIRONMENT_H

#include "Map2DEnvironment.h" 
#include "BitVector.h"

//template <class state, class action>
class MapOccupancyInterface //: public OccupancyInterface
{
public:	
	MapOccupancyInterface(Map *m);
	~MapOccupancyInterface();
	//virtual void SetStateOccupied(state, bool) = 0;
	void SetStateOccupied(xyLoc, bool);
	//virtual bool GetStateOccupied(state) = 0;
	bool GetStateOccupied(xyLoc);
	//virtual bool CanMove(state, state);
	//virtual bool CanMove(state, state, double, uint32_t ID) = 0;
	//virtual bool ReserveMove(state, state, double, uint32_t ID) = 0;
	//virtual bool ClearMove(state, state, double, uint32_t ID) = 0;
	//virtual bool ClearStates() = 0;

	
private:
	bitVector *bitvec; /// For each map position, set (1) if occupied
	long mapWidth; /// Used to compute index into bitvector
	long mapHeight; /// used to compute index into bitvector
	
	long CalculateIndex(uint16_t x, uint16_t y);
	

};



class WeightedMap2DEnvironment : public AbsMapEnvironment 
{
public:
	WeightedMap2DEnvironment(MapAbstraction *ma);
	virtual ~WeightedMap2DEnvironment();
	void ApplyAction(xyLoc &s, tDirection dir);
	virtual double GCost(xyLoc &node1, xyLoc &node2);

private:
	MapOccupancyInterface* MOI;
};

typedef UnitSimulation<xyLoc, tDirection, WeightedMap2DEnvironment> UnitWeightedMapSimulation;

#endif

