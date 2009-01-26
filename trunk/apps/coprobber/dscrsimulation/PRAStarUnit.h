#include "Unit.h"
#include "Map2DEnvironment.h"
#include "GraphEnvironment.h"
#include "MapAbstraction.h"
#include "PRAStar.h"

#ifndef PRASTARUNIT_H
#define PRASTARUNIT_H

/*!
	Provides a unit that does PRA* and follows the path
*/
template<class state,class action,class environment>
class PraStarUnit: public Unit<state,action,environment> {

	public:
	PraStarUnit( MapAbstraction *abs, state initial_state, unsigned int cop_speed );
	PraStarUnit( MapAbstraction *abs, state initial_state, unsigned int robberunit, unsigned int cop_speed );
	~PraStarUnit();

	// sets the unit number of the robber that we want to run to
	void SetRobberUnit( unsigned int _robberunit ) { robberunit = _robberunit; };

	// returns PraStarUnit
	const char *GetName();

	bool MakeMove( environment *env, OccupancyInterface<state,action> *, SimulationInfo<state,action,environment> *sinfo, action &a );
	void UpdateLocation( environment *env, state &s, bool success, SimulationInfo<state,action,environment> *sinfo );
	void GetLocation( state &s ) { s = current_pos; };
	void OpenGLDraw( int window, environment *env, SimulationInfo<state,action,environment>* );
	void GetGoal( state &s ) { s = current_pos; };
	bool Done() { return done; };

	void SetColor( GLfloat _r, GLfloat _g, GLfloat _b ) {
		r = _r; g = _g; b = _b;
	};

	protected:

	// gives back kStay for MapEnvironment and (current_pos,current_pos) for GraphEnvironment
	action nomove();

	void GetPraStarPath( environment *env, state robber_pos );

	GLfloat r, g, b;
	MapAbstraction *abs;
	state current_pos;
	unsigned int robberunit, cop_speed;
	bool done;
	praStar *pra;

	private:

	std::vector<state> pathcache;
};


/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/
template<class state,class action,class environment>
PraStarUnit<state,action,environment>::PraStarUnit( MapAbstraction *_abs, state initial_state, unsigned int _cop_speed ):
	abs(_abs), current_pos(initial_state), robberunit(0), cop_speed(_cop_speed),
	done(false), pra( new praStar() )
{ };

template<class state,class action,class environment>
PraStarUnit<state,action,environment>::PraStarUnit( MapAbstraction *_abs, state initial_state,
	unsigned int _copunit, unsigned int _cop_speed ):
	abs(_abs), current_pos(initial_state), robberunit(_copunit), cop_speed(_cop_speed),
	done(false), pra( new praStar() )
{ };

template<class state,class action,class environment>
PraStarUnit<state,action,environment>::~PraStarUnit() {
	delete pra;
};


template<class state,class action,class environment>
const char* PraStarUnit<state,action,environment>::GetName() {
	return "PraStarUnit";
};

template<class state,class action,class environment>
bool PraStarUnit<state,action,environment>::MakeMove( environment *env, OccupancyInterface<state,action> *, SimulationInfo<state,action,environment> *sinfo, action &a ) {

	state robber_pos = sinfo->GetPublicUnitInfo( robberunit )->currentState;

	if( robber_pos == current_pos ) {
		done = true;
		a = nomove();
		return false;
	}

	if( pathcache.size() == 0 )
		GetPraStarPath( env, robber_pos );

	if( pathcache.size() > 0 ) {
		a = env->GetAction( current_pos, pathcache[0] );
		current_pos = pathcache[0]; // update my current position for me
		pathcache.erase( pathcache.begin() );

		if( robber_pos == current_pos ) done = true;
		return true;
	} else {
		a = nomove();
		return false;
	}
};

template<class state,class action,class environment>
void PraStarUnit<state,action,environment>::UpdateLocation( environment *, state &s, bool, SimulationInfo<state,action,environment> *sinfo ) {
	current_pos = s;
};


// \see PraStarUnit.cpp for implementation
template<>
void PraStarUnit<xyLoc,tDirection,MapEnvironment>::OpenGLDraw( int window, MapEnvironment *env, SimulationInfo<xyLoc,tDirection,MapEnvironment>* );
template<>
void PraStarUnit<xyLoc,tDirection,AbsMapEnvironment>::OpenGLDraw( int window, AbsMapEnvironment *env, SimulationInfo<xyLoc,tDirection,AbsMapEnvironment>* );
template<>
void PraStarUnit<graphState,graphMove,GraphEnvironment>::OpenGLDraw( int window, GraphEnvironment *env, SimulationInfo<graphState,graphMove,GraphEnvironment>* );
template<>
void PraStarUnit<graphState,graphMove,AbstractionGraphEnvironment>::OpenGLDraw( int window, AbstractionGraphEnvironment *env, SimulationInfo<graphState,graphMove,AbstractionGraphEnvironment>* );
template<>
tDirection PraStarUnit<xyLoc,tDirection,MapEnvironment>::nomove();
template<>
tDirection PraStarUnit<xyLoc,tDirection,AbsMapEnvironment>::nomove();
template<>
graphMove PraStarUnit<graphState,graphMove,GraphEnvironment>::nomove();
template<>
graphMove PraStarUnit<graphState,graphMove,AbstractionGraphEnvironment>::nomove();
template<>
void PraStarUnit<xyLoc,tDirection,MapEnvironment>::GetPraStarPath( MapEnvironment *env, xyLoc robber_pos );
template<>
void PraStarUnit<xyLoc,tDirection,AbsMapEnvironment>::GetPraStarPath( AbsMapEnvironment *env, xyLoc robber_pos );
template<>
void PraStarUnit<graphState,graphMove,GraphEnvironment>::GetPraStarPath( GraphEnvironment *env, graphState robber_pos );
template<>
void PraStarUnit<graphState,graphMove,AbstractionGraphEnvironment>::GetPraStarPath( AbstractionGraphEnvironment *env, graphState robber_pos );


#endif
