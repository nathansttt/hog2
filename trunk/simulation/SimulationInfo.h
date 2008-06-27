#include <vector>
#include "UnitSimulation.h"

#ifndef SIMULATIONINFO_H
#define SIMULATIONINFO_H


template<class state, class action, class environment>
class PublicUnitInfo;

/*! public information that will be given to every unit */
template<class state, class action, class environment>
class SimulationInfo {

	public:
	SimulationInfo( std::vector<PublicUnitInfo<state,action,environment> > unitinfos, std::vector<std::vector<unsigned int> > unitsingroups, double currTime );

	// copy operator
	SimulationInfo( SimulationInfo<state,action,environment> *sim );
	SimulationInfo();

	virtual ~SimulationInfo() { unitinfos.clear(); }
	virtual double GetSimulationTime() { return currentTime; };

	virtual unsigned int GetNumUnits() { return unitinfos.size(); };
	virtual PublicUnitInfo<state,action,environment>* GetPublicUnitInfo( unsigned int unitnum ) { return &(unitinfos[unitnum]); }

	virtual unsigned int GetNumUnitGroups() { return unitsingroups.size(); };
	virtual std::vector<unsigned int> GetUnitGroupMembers( unsigned int unitgroupnum ) {
		return unitsingroups[unitgroupnum];
	}

	std::vector<PublicUnitInfo<state,action,environment> > unitinfos;
	std::vector<std::vector<unsigned int> > unitsingroups;
	double currentTime;
};

template<class state, class action, class environment>
class PublicUnitInfo {
	public:

	PublicUnitInfo( const char *_name, state _startState, state _currentState,
		action _lastMove, double _nextTime, double _speed, bool _done, state _goal )
		:
		name(_name),
		startState(_startState), currentState(_currentState), lastMove(_lastMove),
		nextTime(_nextTime), speed(_speed), done(_done), goal(_goal)
		{ };

	const char *name;
	state startState;
	state currentState;
	action lastMove;

	double nextTime;
	double speed;
	bool done;
	state goal;
};

template<class state, class action, class environment>
SimulationInfo<state,action,environment>::SimulationInfo():
	currentTime(0.)
{ };

template<class state, class action, class environment>
SimulationInfo<state,action,environment>::SimulationInfo( std::vector<PublicUnitInfo<state,action,environment> > _unitinfos, std::vector<std::vector<unsigned int> > _unitsingroups, double _currTime ):
	unitinfos(_unitinfos), unitsingroups(_unitsingroups), currentTime(_currTime)
{ };

template<class state, class action, class environment>
SimulationInfo<state,action,environment>::SimulationInfo( SimulationInfo<state,action,environment> *sim ) {
	unitinfos = sim->unitinfos;
	unitsingroups = sim->unitsingroups;
	currentTime = sim->currentTime;
};


#endif
