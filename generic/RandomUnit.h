/*
 *  RandomUnit.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/14/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "Unit.h"

#ifndef RANDOMUNIT_H
#define RANDOMUNIT_H

template <class state, class action, class environment>
class RandomUnit : public Unit<state, action, environment>
{
public:
	RandomUnit(state startLoc) :loc(startLoc) {}
	virtual ~RandomUnit() {}
	virtual const char *GetName() { return "Random Unit"; }
	virtual action MakeMove(environment *env, OccupancyInterface<state, action> *, SimulationInfo *)
	{
		std::vector<action> acts;
		env->GetActions(loc, acts);
		return acts[random()%acts.size()];
	}
	virtual void UpdateLocation(environment *, state &newloc, bool success, SimulationInfo *)
	{
		if (success)
			loc = newloc;
	}
	virtual void GetLocation(state &l)
	{ l = loc; }
	virtual void OpenGLDraw(int window, environment *env, SimulationInfo *)
	{ env->OpenGLDraw(window, loc); }
	virtual void GetGoal(state &s)
	{ s = loc; }
private:
	state loc;
};

#endif

