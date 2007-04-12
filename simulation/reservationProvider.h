/*
 * $Id: reservationProvider.h,v 1.4 2006/09/18 06:20:50 nathanst Exp $
 *
 *  reservationProvider.h
 *  HOG
 *
 *  Created by Nathan Sturtevant on 5/13/05.
 *  Copyright 2005 Nathan Sturtevant, University of Alberta. All rights reserved.
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

#ifndef RESERVATIONPROVIDER_H
#define RESERVATIONPROVIDER_H

#include "graph.h"
class unit;

class reservationProvider {
public:
	virtual ~reservationProvider() {};
	//virtual bool tileOccupied(int x, int y) = 0;	
	virtual bool nodeOccupied(node *) = 0;

	virtual bool canMove(node *from, node *to, double startTime, unit *) = 0;
	virtual bool reserveMove(node *from, node *to, double startTime, unit *) = 0;
	virtual bool clearMove(node *from, node *to, double startTime, unit *) = 0;
	virtual void clearAllReservations() = 0;
};

#include "unit.h"

#endif
