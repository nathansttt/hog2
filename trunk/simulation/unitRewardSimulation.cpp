/*
 * $Id: unitRewardSimulation.cpp,v 1.7 2006/10/20 19:01:44 nathanst Exp $
 *
 *  unitRewardSimulation.cpp
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

//#include "unitRewardSimulation.h"
//#include "rewardUnit.h"
//
//
//static const int verbose = 0;
//
//unitRewardSimulation::unitRewardSimulation(mapAbstraction *_m, bool keepStats)
//:unitSimulation(_m, keepStats)
//{
//}
//
//unitRewardSimulation::~unitRewardSimulation()
//{
//}
//
//void unitRewardSimulation::doTimestepCalc()
//{
//	bool allUpToDate;
//	unit *mainActor = 0;
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
//	do {
//		allUpToDate = true;
//		for (unsigned int t = 0; t < units.size(); t++)
//		{
//			if (units[t]->agent->getObjectType() == kWorldObject)
//				mainActor = units[t]->agent;
//			
//			stepUnitTime(units[t]);
//			
//			if (units[t]->nextTime < currTime)
//			{
//				if (realTime)
//					units[t]->nextTime = currTime;
//				else
//					allUpToDate = false;
//			}
//		}
//		if (mainActor == 0)
//			continue;
//		// check to see about giving out rewards!
//		for (unsigned int t = 0; t < units.size(); t++)
//		{
//			if (units[t]->agent->getObjectType() == kIncidentalUnit)
//			{
//				int x1, x2, y1, y2;
//				units[t]->agent->getLocation(x1, y1);
//				mainActor->getLocation(x2, y2);
//				if ((x1 == x2) && (y1 == y2))
//					((rewardUnit*)mainActor)->receiveReward(((rewardUnit*)units[t]->agent)->sendReward());
//			}
//		}
//	} while (!allUpToDate);	
//}
