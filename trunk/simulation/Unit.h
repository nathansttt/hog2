/*
 * $Id: unit.h,v 1.21 2007/02/22 01:50:51 bulitko Exp $
 *
 *  Hierarchical Open Graph File
 *
 *  Created by Nathan Sturtevant on 9/28/04.
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

#ifndef UNITS_H
#define UNITS_H

#include "UnitGroup.h"
#include "UnitSimulation.h"
#include "OccupancyInterface.h"

template<class state, class action, class environment>
class SimulationInfo;

template <class state, class action, class environment>
class Unit {
public:
	//	Unit(state s, Unit<state, action, env> *target);
	Unit() :speed(0), group(0) { SetColor(1.0, 0.0, 0.0); priority = 999; }
	virtual ~Unit() { SetUnitGroup(0); }
	virtual const char *GetName() = 0;
	virtual bool MakeMove(environment *, OccupancyInterface<state,action> *, SimulationInfo<state,action,environment> *, action& a) = 0;
	virtual void UpdateLocation(environment *, state &, bool success, SimulationInfo<state,action,environment> *) = 0;
	virtual void GetLocation(state &) = 0;
	virtual void OpenGLDraw(const environment *, const SimulationInfo<state,action,environment> *) const = 0;
	virtual void GetGoal(state &s) = 0;
	virtual bool Done() { return true;} 

	virtual double GetSpeed() { return speed; }
	void SetSpeed(double s) { speed = s; }

	/** log an stats that may have been computed during the last run */
	virtual void LogStats(StatCollection *) {}
	/** log any final one-time stats before a simulation is ended */
	virtual void LogFinalStats(StatCollection *) {}
	
	virtual void SetColor(GLfloat _r, GLfloat _g, GLfloat _b) { r=_r; g=_g; b=_b; }
	virtual void GetColor(GLfloat& _r, GLfloat& _g, GLfloat& _b) const { _r=r; _g=g; _b=b; }
	
	UnitGroup<state, action, environment> *GetUnitGroup() { return group; }
	void SetUnitGroup(UnitGroup<state, action, environment> *_group)
		{
			// If we're already set to the given group then do nothing
			if (_group == group)
				return;
			
			UnitGroup<state, action, environment> *tmp = group;
			
			group = 0; 
			if(tmp)
				tmp->RemoveUnit(this);
			
			// Set the back pointer
			group = _group;
			
			if(_group != 0)
				_group->AddUnit(this);
			
			// OLD CODE
/*			// If we had a group before then move
			if (tmp != 0)
			{
				tmp->RemoveUnit(this);
				if (_group)
					_group->AddUnit(this);
			} */ 
		}

	virtual unsigned int GetNum() { return unitid; }
	virtual void SetNum( unsigned int num ) { unitid = num; return; }
	virtual void StartNewTrial(StatCollection *) {}

	virtual int GetPriority() { return priority; }
	virtual void SetPriority(int val) { priority = val; }
private:
	double speed;
	int priority;
	UnitGroup<state, action, environment> *group;
	GLfloat r, g, b;
	unsigned int unitid;
};


#endif
