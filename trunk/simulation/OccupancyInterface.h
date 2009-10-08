/**
* @file OccupancyInterface.h
* @package hog2
* @brief Interface for OccupancyInterface
* @author Renee Jansen
* @date 8/15/07
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

// This interface has been moved into its own file so it can more easily
// be included in other files. 

// Moved from ReservationProvider.h

#ifndef OCCUPANCYINTERFACE_H
#define OCCUPANCYINTERFACE_H	

template <class state, class action>
class OccupancyInterface {
public:
	virtual ~OccupancyInterface() {}
	virtual void SetStateOccupied(const state&, bool) = 0;
	virtual bool GetStateOccupied(const state&) = 0;
	virtual void MoveUnitOccupancy(const state &, const state&) = 0;
	virtual bool CanMove(const state&, const state&) = 0;
	//virtual bool CanMove(state, state, double, uint32_t ID) = 0;
	//virtual bool ReserveMove(state, state, double, uint32_t ID) = 0;
	//virtual bool ClearMove(state, state, double, uint32_t ID) = 0;
	//virtual bool ClearStates() = 0;
};

#endif
