/*
 * $Id: humanUnit.h,v 1.3 2006/09/18 06:19:31 nathanst Exp $
 *
 *  Hierarchical Open Graph File
 *
 *  Created by Nathan Sturtevant on 10/4/04.
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

#ifndef HUMANUNIT_H
#define HUMANUNIT_H

#include "unit.h"

class humanUnit : public unit {
public:
  humanUnit(int x, int y, unit *target);
	tDirection makeMove(mapProvider *mp, reservationProvider *rp, simulationInfo *simInfo);
  //tDirection makeMove(mapProvider *, reservationProvider *, simulationInfo *simInfo); // this is where the World says you are
  //	double getSpeed() { return .25; }
	void setNextDirection(tDirection dir) { nextDir = dir; }
private:
	tDirection nextDir;
};
	
#endif
