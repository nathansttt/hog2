/*
 * $Id: rewardUnit.h,v 1.5 2006/09/18 06:19:31 nathanst Exp $
 *
 *  rewardUnit.h
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


#include "unit.h"
#include "aStar.h"

#ifndef REWARDUNIT_H
#define REWARDUNIT_H

class rewardUnit : public unit {
public:
	rewardUnit(int x, int y);
	virtual const char *getName() { return "rewardUnit"; }
	virtual double sendReward() { return 1.0; }
	virtual void receiveReward(double) {}
	void openGLDraw(mapProvider *, simulationInfo *);
};

class rewardSeekingUnit : public rewardUnit {
public:
	rewardSeekingUnit(int x, int y);
	virtual const char *getName() { return "rewardUnit"; }
	virtual void receiveReward(double);
	virtual tDirection makeMove(mapProvider *, reservationProvider *, simulationInfo *simInfo);
	void openGLDraw(mapProvider *, simulationInfo *);
	void addRewardLocation(rewardUnit *);
private:
	double goToRewardLoc(mapAbstraction *aMap, int which);
	void addPathToCache(path *p);
	std::vector<tDirection> moves;
	std::vector<rewardUnit *> rewardLocs;
	aStar a;
};

#endif
