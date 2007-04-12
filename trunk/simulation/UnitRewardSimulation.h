/*
 * $Id: unitRewardSimulation.h,v 1.7 2006/10/20 19:01:44 nathanst Exp $
 *
 *  unitRewardSimulation.h
 *  HOG
 *
 *  Created by Nathan Sturtevant on 2/16/05.
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

#include "unitSimulation.h"

#ifndef UNITREWARDSIMULATION_H
#define UNITREWARDSIMULATION_H

/**
 * A simple simulation in which units can receive reward for interacting with other
 * units.
 * 
 * This class should not be used. If needed please inform me (nathanst) and I will
 * show you how to implement the same thing using a unit group. Otherwise this
 * class introduces cyclic dependencies.
 */

//class unitRewardSimulation : public unitSimulation {
//public:
//	unitRewardSimulation(mapAbstraction *m, bool keepStats = false);
//	~unitRewardSimulation();
//private:
//	void doTimestepCalc();
//};

#endif
