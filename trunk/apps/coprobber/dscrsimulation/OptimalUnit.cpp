#include "OptimalUnit.h"
#include "GLUtil.h"

/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/
OptimalUnit::OptimalUnit( AbstractionGraphEnvironment *env, graphState initial_state, unsigned int _cop_speed ):
	dsdijkstra( new DSDijkstra_MemOptim( env, _cop_speed ) ),
	current_pos(initial_state), copunit(0), cop_speed(_cop_speed), done(false)
{
	fprintf( stdout, "solving entire state space... " );
	fflush( stdout );
	dsdijkstra->dsdijkstra();
	fprintf( stdout, "done\n" );
	fflush( stdout );
};

OptimalUnit::OptimalUnit( AbstractionGraphEnvironment *env, graphState initial_state, unsigned int _copunit, unsigned int _cop_speed ):
	dsdijkstra( new DSDijkstra_MemOptim( env, _cop_speed ) ),
	current_pos(initial_state), copunit(_copunit), cop_speed(_cop_speed), done(false)
{
	fprintf( stdout, "solving entire state space... " );
	fflush( stdout );
	dsdijkstra->dsdijkstra();
	fprintf( stdout, "done\n" );
	fflush( stdout );
};

OptimalUnit::~OptimalUnit() {
	delete dsdijkstra;
};


const char* OptimalUnit::GetName() {
	char *str; str = (char*) malloc( sizeof(char) * 20 );
	sprintf( str, "OptimalUnit" );
	return str;
};

bool OptimalUnit::MakeMove( AbstractionGraphEnvironment *env, OccupancyInterface<graphState,graphMove> *, SimulationInfo<graphState,graphMove,AbstractionGraphEnvironment> *sinfo, graphMove &a ) {

	// if the cop that we have to run away from is not set yet
	if( copunit == 0 || done ) {
		a = env->GetAction( current_pos, current_pos );
		return false;
	}

	DSDijkstra_MemOptim::CRState mypos;
	mypos.push_back( current_pos );
	mypos.push_back( sinfo->GetPublicUnitInfo( copunit )->currentState );
	graphState next_state = dsdijkstra->MakeMove( mypos, false );
	a = env->GetAction( current_pos, next_state );
	return true;
};

void OptimalUnit::UpdateLocation( AbstractionGraphEnvironment *, graphState &s, bool, SimulationInfo<graphState,graphMove,AbstractionGraphEnvironment> *sinfo ) {
	current_pos = s;
	if( copunit > 0 && sinfo->GetPublicUnitInfo( copunit )->currentState == current_pos )
		done = true;
};



void OptimalUnit::OpenGLDraw( int, AbstractionGraphEnvironment *env, SimulationInfo<graphState,graphMove,AbstractionGraphEnvironment>* ) {
	node *n  = env->GetGraph()->GetNode( current_pos );
	GLdouble x, y, z, rad = 0.01;// env->scale()/2.;
	x = n->GetLabelF(GraphAbstractionConstants::kXCoordinate);
	y = n->GetLabelF(GraphAbstractionConstants::kYCoordinate);
	z = n->GetLabelF(GraphAbstractionConstants::kZCoordinate);
	if( done )
		glColor3d( 0., 1., 0. ); // turn green when done
	else
		glColor3f( r, g, b );
	DrawSphere( x, y, z, rad );

	return;
};
