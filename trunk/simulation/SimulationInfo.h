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
	//SimulationInfo( std::vector<PublicUnitInfo<state,action,environment> > unitinfos, std::vector<std::vector<unsigned int> > unitsingroups, double currTime );
	// copy operator
	//SimulationInfo( SimulationInfo<state,action,environment> *sim );
	//SimulationInfo();

	virtual ~SimulationInfo() { /*unitinfos.clear();*/ }
	virtual double GetSimulationTime() const = 0;

	virtual unsigned int GetNumUnits() const = 0;
	virtual void GetPublicUnitInfo( unsigned int unitnum, PublicUnitInfo<state,action,environment> & ) const = 0;
	//{ return &(unitinfos[unitnum]); }

	virtual unsigned int GetNumUnitGroups() const = 0;
	virtual unsigned int GetCurrentUnit() const = 0;

	virtual StatCollection *GetStats() = 0;
//	virtual std::vector<unsigned int> GetUnitGroupMembers( unsigned int unitgroupnum ) {
//		return unitsingroups[unitgroupnum];
//	}

//	std::vector<PublicUnitInfo<state,action,environment> > unitinfos;
//	std::vector<std::vector<unsigned int> > unitsingroups;
//	double currentTime;
};

template<class state, class action, class environment>
class PublicUnitInfo {
public:
	PublicUnitInfo() {}
	PublicUnitInfo(state &_startState, state &_currentState, state &_lastState,
				   action _lastMove, double _lastTime, double _nextTime)
	:startState(_startState), currentState(_currentState), lastState(_lastState),
	lastMove(_lastMove), lastTime(_lastTime), nextTime(_nextTime)
	{ }
	void init(state &_startState, state &_currentState, state &_lastState,
			  action _lastMove, double _lastTime, double _nextTime)
	{
		startState = _startState;
		currentState = _currentState;
		lastState = _lastState;
		lastMove = _lastMove;
		lastTime = _lastTime;
		nextTime = _nextTime;
	}	
	
	state startState;
	state currentState;
	state lastState;
	action lastMove;

	double lastTime;
	double nextTime;
};
//
//template<class state, class action, class environment>
//SimulationInfo<state,action,environment>::SimulationInfo():
//	currentTime(0.)
//{ };
//
//template<class state, class action, class environment>
//SimulationInfo<state,action,environment>::SimulationInfo( std::vector<PublicUnitInfo<state,action,environment> > _unitinfos, std::vector<std::vector<unsigned int> > _unitsingroups, double _currTime ):
//	unitinfos(_unitinfos), unitsingroups(_unitsingroups), currentTime(_currTime)
//{ };
//
//template<class state, class action, class environment>
//SimulationInfo<state,action,environment>::SimulationInfo( SimulationInfo<state,action,environment> *sim ) {
//	unitinfos = sim->unitinfos;
//	unitsingroups = sim->unitsingroups;
//	currentTime = sim->currentTime;
//};


#endif
