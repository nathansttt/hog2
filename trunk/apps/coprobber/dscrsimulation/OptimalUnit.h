#include "Unit.h"
#include "Map2DEnvironment.h"
#include "GraphEnvironment.h"
#include "../DSDijkstra_MemOptim.h"

#ifndef OPTIMALUNIT_H
#define OPTIMALUNIT_H

/*!
	Provides a unit that moves due to minimax value of the game tree
	(Nash equilibrium player)

	note: this unit solves the entire state space first before acting.
	This shouldn't be necessary but due to laziness I implemented it like that.
*/
class OptimalUnit: public Unit<graphState,graphMove,AbstractionGraphEnvironment> {

	public:
	OptimalUnit( AbstractionGraphEnvironment *env, graphState initial_state, unsigned int cop_speed );
	OptimalUnit( AbstractionGraphEnvironment *env, graphState initial_state, unsigned int copunit, unsigned int cop_speed );
	~OptimalUnit();

	// sets the unit number of the cop we want to run away from
	void SetCopUnit( unsigned int _copunit ) { copunit = _copunit; };

	// returns OptimalUnit
	const char *GetName();

	bool MakeMove( AbstractionGraphEnvironment *env, OccupancyInterface<graphState,graphMove> *, SimulationInfo<graphState,graphMove,AbstractionGraphEnvironment> *sinfo, graphMove &a );
	void UpdateLocation( AbstractionGraphEnvironment *env, graphState &s, bool success, SimulationInfo<graphState,graphMove,AbstractionGraphEnvironment> *sinfo );
	void GetLocation( graphState &s ) { s = current_pos; };
	void OpenGLDraw( int window, AbstractionGraphEnvironment *env, SimulationInfo<graphState,graphMove,AbstractionGraphEnvironment>* );
	void GetGoal( graphState &s ) { s = current_pos; };
	bool Done() { return done; };

	void SetColor( GLfloat _r, GLfloat _g, GLfloat _b ) {
		r = _r; g = _g; b = _b;
	};

	protected:

	DSDijkstra_MemOptim *dsdijkstra;
	GLfloat r, g, b;
	graphState current_pos;
	unsigned int copunit, cop_speed;
	bool done;
};

#endif
