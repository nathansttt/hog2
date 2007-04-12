/*
 * $Id: humanUnit.cpp,v 1.4 2006/10/18 23:52:25 nathanst Exp $
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
 */

#include "humanUnit.h"

humanUnit::humanUnit(int _x, int _y, unit *_target)
:unit(_x, _y, _target)
{
	unitType = kWorldObject;
  r = 0.0; g = 0.0; b = 1.0;
}

tDirection humanUnit::makeMove(mapProvider *, reservationProvider *, simulationInfo *)
{
  b = 1-b;
  tDirection res = nextDir;
	// uncomment this line if you don't want units to keep moving
	//nextDir = kStay;
  return (tDirection)res;
}
