
// Most of this code has been copied from CRSimulation.h
// note: this is customization to DSCREnvironment

#ifndef DSCRSIMULATION_H
#define DSCRSIMULATION_H

#include <vector>
#include <queue>
#include "Unit.h"
#include "Timer.h"
#include "FPUtil.h"
#include "StatCollection.h"
#include "SearchEnvironment.h"
#include "OccupancyInterface.h"
#include "SimulationInfo.h"
#include "../DSCREnvironment.h"


template <class state, class action, class environment>
class DSCRUnitInfo {
public:
	DSCRUnitInfo() {};
	Unit<state, action, environment> *agent;
	action lastMove;
	state startState;
	state currentState;
	double nextTime;
	double totalDistance;
	double totalThinking;
};


/**
 * A simulation of the game on cops and robber
 * Note: Please add a robber before you use anything else of the simulation (this simulation
 * is only designed for one robber)
 */
template<class state, class action, class environment>
class DSCRSimulation {
public:

	typedef typename DSCREnvironment<state,action>::CRState CRState;

	/*! creates the simulation by specifying the environment */
	DSCRSimulation(environment *env, bool playerscanpause = false, unsigned int cop_speed = 1, bool simultaneous = false );
	~DSCRSimulation();

	/*! adds the robber to the simulation */
	unsigned int AddRobber( Unit<state,action,environment> *unit, double timeOffset = 0. );
	/*! add cops to the simulation */
	unsigned int AddCop( Unit<state, action, environment> *unit, double timeOffset = 0. );

	/*!
		Use this functionality if you want to have the copgroup make moves and such.
		This can be used to coordinate the cops and to spread information amongst them
		(as to where the actual current position of each cop is etc etc)
	*/
	void AddCopGroup( UnitGroup<state,action,environment> *copgroup );

	Unit<state, action, environment> *GetUnit( unsigned int which );
	UnitGroup<state,action,environment> *GetCopGroup() { return copgroup; };

	/*! deletes all the cops of the simulation (and it's copgroup) */
	void ClearAllUnits();

	environment *GetEnvironment() { return env; }

	void StepTime(double);
	double GetSimulationTime() { return currTime; }
	double GetTimeToNextStep();
	void DoNextStep() { StepTime( GetTimeToNextStep() ); };

	void SetPaused(bool val) { paused = val; }
	bool GetPaused() { return paused; }

	/*! simulation is over when either all the agents are done
	 * or the robber is caught */
	bool Done();

	void OpenGLDraw(int window);

	SimulationInfo<state,action,environment>* GetSimulationInfo() {
		return &sinfo;
	};

protected:
	void StepUnitTime( unsigned int index, double timeStep);
	bool MakeUnitMove( unsigned int index, std::vector<action> &next_actions, double &moveCost);

	// if index == -1 then update all information, otherwise just that particular unit
	void UpdatePublicInfo( int index = -1 );

	// variables
	environment *env;
	bool simultaneous; // whether the simulation is simultaneous or alternating
	double currTime;
	bool paused;
	UnitGroup<state,action,environment> *copgroup;
	std::vector<DSCRUnitInfo<state, action, environment> *> units;
	DSCREnvironment<state,action> *dscrenvironment;
	CRState worldstate;
	SimulationInfo<state,action,environment> sinfo;
};



typedef DSCRSimulation<xyLoc, tDirection, AbsMapEnvironment> DSCRAbsMapSimulation;







/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/

template<class state, class action, class environment>
DSCRSimulation<state, action, environment>::DSCRSimulation(environment *_env,
	bool playerscanpass, unsigned int cop_speed, bool _simultaneous ):
	env(_env), simultaneous(_simultaneous), currTime(0.), paused(false), copgroup(NULL),
	dscrenvironment( new DSCREnvironment<state,action>(env,playerscanpass,cop_speed) )
{ };

template<class state, class action, class environment>
DSCRSimulation<state, action, environment>::~DSCRSimulation() {
	// deletes all the cops
	ClearAllUnits();
	// deletes the Cop Robber environment
	delete dscrenvironment;
	// deletes the submitted environment
	//delete env;
}


template<class state,class action,class environment>
unsigned int DSCRSimulation<state,action,environment>::AddRobber( Unit<state,action,environment> *unit, double timeOffset ) {
	DSCRUnitInfo<state,action,environment> *ui = new DSCRUnitInfo<state,action,environment>();
	ui->agent = unit;
	unit->GetLocation( ui->startState );
	ui->currentState = ui->startState;

	SimulationInfo<state,action,environment> *s = new SimulationInfo<state,action,environment>( &sinfo );
	ui->agent->UpdateLocation( env, ui->currentState, true, s );
	delete s;

	ui->nextTime = currTime + timeOffset;
	ui->totalThinking = 0.;
	ui->totalDistance = 0.;

	units.insert( units.begin(), ui ); // put the robber on top of the vector
	worldstate.insert( worldstate.begin(), ui->currentState );
	ui->agent->SetNum( 0 );

	UpdatePublicInfo();
	return 0;
};


template<class state, class action, class environment>
unsigned int DSCRSimulation<state,action,environment>::AddCop( Unit<state,action,environment> *unit, double timeOffset ) {
	DSCRUnitInfo<state,action,environment> *ui = new DSCRUnitInfo<state,action,environment>();
	ui->agent = unit;
	unit->GetLocation( ui->startState );
	ui->currentState = ui->startState;

	SimulationInfo<state,action,environment> *s = new SimulationInfo<state,action,environment>( &sinfo );
	if( copgroup ) {
		// if there is a copgroup set we'll update its position through the copgroup
		copgroup->UpdateLocation( ui->agent, env, ui->currentState, true, s );
	} else {
		ui->agent->UpdateLocation( env, ui->currentState, true, s );
	}
	delete s;

	ui->nextTime = currTime + timeOffset;
	ui->totalThinking = 0.;
	ui->totalDistance = 0.;

	units.push_back( ui );
	worldstate.push_back( ui->currentState );
	ui->agent->SetNum( units.size() - 1 );

	UpdatePublicInfo();
	return( units.size()-1 );
}


template<class state,class action, class environment>
void DSCRSimulation<state,action,environment>::AddCopGroup( UnitGroup<state,action,environment> *_copgroup ) {
	if( copgroup )
		fprintf( stderr, "Warning: Added a group for the cops twice.\n" );
	copgroup = _copgroup;
	UpdatePublicInfo();
	return;
}


template<class state, class action, class environment>
Unit<state,action,environment>* DSCRSimulation<state,action,environment>::GetUnit( unsigned int which ) {
	assert( which < units.size() );
	return units[which];
}


template<class state, class action, class environment>
void DSCRSimulation<state, action, environment>::ClearAllUnits()
{
	while( units.size() > 0 ) {
		DSCRUnitInfo<state,action,environment> *ui = units.back();
		units.pop_back();
		delete ui->agent;
		delete ui;
	}

	UpdatePublicInfo();
	currTime = 0.; // reset simulation time
}


template<class state, class action, class environment>
bool DSCRSimulation<state,action,environment>::Done()
{

	// check if robber is caught
	if( dscrenvironment->GoalTest( worldstate ) )
		return true;

	// check if robber is done
	if( !units[0]->agent->Done() )
		return false;

	// check if cops are done
	if( copgroup )
		return( copgroup->Done() );
	else {
		for( unsigned int i = 1; i < units.size(); i++ ) {
			if( !units[i]->agent->Done() )
				return false;
		}
	}

	return true;
}



template<class state, class action, class environment>
double DSCRSimulation<state,action,environment>::GetTimeToNextStep() {
	double minimum = DBL_MAX;

	for( unsigned int i = 0; i < units.size(); i++ ) {
		if( minimum > units[i]->nextTime - currTime )
			minimum = units[i]->nextTime - currTime;
	}
	if( minimum < 1./30. )
		fprintf( stderr, "Warning: GetTimeToNextStep sank under 1./30. (%g).\n", minimum );
	return minimum;
}



template<class state, class action, class environment>
void DSCRSimulation<state, action, environment>::StepTime(double timeStep)
{
	if (paused) return;
	if( Done() ) {
		fprintf( stderr, "Warning: simulation is over but StepTime was requested.\n" );
		return;
	}

	currTime += timeStep;
	for( unsigned int i = 1; i < units.size(); i++ ) {
		StepUnitTime( i, timeStep );
		if( !simultaneous ) UpdatePublicInfo( i ); // update our public information record
	}
	if( !Done() ) {
		StepUnitTime( 0, timeStep );
		if( !simultaneous ) UpdatePublicInfo( 0 );
	} else {
		// tell the robber that he's done
		SimulationInfo<state,action,environment> *s = new SimulationInfo<state,action,environment>( &sinfo );
		units[0]->agent->UpdateLocation( env, units[0]->currentState, false, s );
		delete s;
	}
	if( simultaneous ) UpdatePublicInfo();
	
	return;
}



template<class state, class action, class environment>
void DSCRSimulation<state, action, environment>::StepUnitTime( unsigned int index, double timeStep)
{
	DSCRUnitInfo<state,action,environment> *theUnit = units[index];

	// if this unit is not scheduled to move, it doesn't
	if (currTime < theUnit->nextTime) return;

	action next_action;
	std::vector<action> next_actions;
	Unit<state, action, environment>* u = theUnit->agent;


	// make a copy of the current simulation info
	// this is done for security reasons
	SimulationInfo<state,action,environment> *s = new SimulationInfo<state,action,environment>(&sinfo);


	// call the agent to make it's move
	// note: cops are called multiple times to make their subsequent moves (according to cop_speed)
	if( index > 0 && copgroup ) {
		// cop in a cop group
		for( unsigned int i = 0; i < dscrenvironment->GetCopSpeed(); i++ ) {
			if( ! copgroup->MakeMove( u, env, s, next_action ) ) break;
			next_actions.push_back( next_action );
		}
	} else if( index > 0 ) {
		// seperate cop
		for( unsigned int i = 0; i < dscrenvironment->GetCopSpeed(); i++ ) {
			if( ! u->MakeMove( env, env->GetOccupancyInfo(), s, next_action ) ) break;
			next_actions.push_back( next_action );
		}
	}	else {
		// robber
		if( u->MakeMove( env, env->GetOccupancyInfo(), s, next_action ) )
			next_actions.push_back( next_action );
	}


	// check whether there is actually anything to do
	bool success = true;
	double moveTime = 1.;
	if( next_actions.size() > 0 ) {
		success = MakeUnitMove( index, next_actions, moveTime);
	}
	// notice the agent of his location update
	if( index && copgroup ) {
		copgroup->UpdateLocation( u, env, theUnit->currentState, success, s );
	} else {
		u->UpdateLocation( env, theUnit->currentState, success, s );
	}

	// delete the copy of public unit information
	delete s;

	// next time the agent is to move again
	// this can be made arbitrarily complex
	theUnit->nextTime += moveTime;

	return;
}

template<class state, class action, class environment>
bool DSCRSimulation<state, action, environment>::MakeUnitMove( unsigned int index, std::vector<action> &next_actions, double &moveCost)
{
	DSCRUnitInfo<state,action,environment> *theUnit = units[index];

	bool success = true;
	state current_state = worldstate[index];
	action last_move = next_actions[0];

	// compute the result state from the actions
	// i.e. execute his chosen path
	for( typename std::vector<action>::iterator it = next_actions.begin(); it != next_actions.end(); it++ ) {
		state temp_state = current_state;
		env->ApplyAction( temp_state, *it );

		if( index > 0 ) {
			// if the agent is a cop, check whether he intersects with another cop
			for( unsigned int i = 1; i < worldstate.size(); i++ ) {
				if( worldstate[i] == temp_state ) {
					success = false;
					break;
				}
			}
			if( !success ) break;
		}
		current_state = temp_state;
		last_move = *it;
	}

	if( index > 0 ) {
		moveCost = dscrenvironment->CopGCost( worldstate[index], current_state );
	} else {
		moveCost = dscrenvironment->RobberGCost( worldstate[index], current_state );
	}

	// update the world state
	worldstate[index] = current_state;
	theUnit->currentState = current_state;
	theUnit->lastMove = last_move;

	return success;
}

/* ATTENTION!!!! This function gives a way the current real information,
 * this is a security issue but for performance we still did it!
*/
template<class state, class action, class environment>
void DSCRSimulation<state, action, environment>::OpenGLDraw(int window)
{
	env->OpenGLDraw(window);

//	UpdatePublicInfo();
	SimulationInfo<state,action,environment> *s = new SimulationInfo<state,action,environment>( &sinfo );

	for (unsigned int i = 0; i < units.size(); i++)
		units[i]->agent->OpenGLDraw( window, env, s );

	if( copgroup )
		copgroup->OpenGLDraw( window, env, s );

	delete s;
	return;
}


template<class state, class action, class environment>
void DSCRSimulation<state,action,environment>::UpdatePublicInfo( int index ) {
	unsigned int i;

	if( index >= 0 && (unsigned int)index < units.size() ) {

		state goal;
		units[index]->agent->GetGoal( goal );
		sinfo.unitinfos[index] = PublicUnitInfo<state,action,environment>(
			units[index]->agent->GetName(),
			units[index]->startState, worldstate[index],
			units[index]->lastMove, units[index]->nextTime, units[index]->agent->GetSpeed(),
			units[index]->agent->Done(), goal
		);

	} else {
		// update all information
		sinfo.unitinfos.clear();
		state goal;
		for( i = 0; i < units.size(); i++ ) {
			units[i]->agent->GetGoal( goal );

			sinfo.unitinfos.push_back( PublicUnitInfo<state,action,environment>(
				units[i]->agent->GetName(),
				units[i]->startState, worldstate[i],
				units[i]->lastMove, units[i]->nextTime, units[i]->agent->GetSpeed(),
				units[i]->agent->Done(), goal
				) );
		}

		sinfo.unitsingroups.clear();
		if( copgroup ) {
			std::vector<Unit<state,action,environment> *> mems = copgroup->GetMembers();
			std::vector<unsigned int> isingroup;
			for( i = 0; i < mems.size(); i++ ) {
				isingroup.push_back( mems[i]->GetNum() );
			}
			sinfo.unitsingroups.push_back( isingroup );
		}
	}

	sinfo.currentTime = currTime;
	return;
}


#endif
