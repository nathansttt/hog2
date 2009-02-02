#include "CopRobber.h"
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <sys/times.h>
#include <string.h>
#include <time.h>
#include "Minimax.h"
#include "Minimax_optimized.h"
#include "MapCliqueAbstraction.h"
#include "LoadedCliqueAbstraction.h"
#include "MultilevelCopRobberGame.h"
#include "RobberUnit.h"
#include "MySearchUnit.h"
#include "CRSimulation.h"
#include "PRAStar.h"
//#include "TIDAStar.h"
#include "TIDAStar_optimized.h"
#include "IPNSearch.h"
#include "IPNTTables.h"
//#include "IPNTTables_optimized.h"
#include "Dijkstra.h"
#include "MaximumNormGraphMapHeuristic.h"
#include "MaximumNormAbstractGraphMapHeuristic.h"
//#include "MinimaxAStar.h"
#include "MinimaxAStar_optimized.h"
#include "TwoPlayerDijkstra.h"
//#include "DSCREnvironment.h"
#include "DSTPDijkstra.h"
#include "DSDijkstra.h"
#include "DSRMAStar.h"
#include "DSDijkstra_MemOptim.h"
#include "DSCover.h"
#include "DSHeuristicGreedy.h"
#include "DSMinimax.h"
#include "DSDAM.h"
#include "DSDATPDijkstra.h"
#include "DSRandomBeacons.h"
#include "DSPRAStarCop.h"
#include "dscrsimulation/DSCRSimulation.h"
#include "dscrsimulation/TrailMaxUnit.h"
#include "dscrsimulation/PRAStarMapUnit.h"
#include "dscrsimulation/PRAStarGraphUnit.h"


//std::vector<CRAbsMapSimulation *> unitSims;
int mazeSize = 10;
DSCRSimulation<graphState,graphMove,AbstractionGraphEnvironment> *simulation;
time_t last_simulation_update;

/*------------------------------------------------------------------------------
| Main
------------------------------------------------------------------------------*/
int main(int argc, char* argv[])
{


	// parse command line parameters
	if( argc < 2 ) {
		printf( "usage: coprobber <algorithm/test> [algorithm/test params]\n" );
		printf( "available algorithms/tests:\n" );
		printf( "testing             - test code of lots of stuff\n" );
		printf( "dijkstra            - Dijkstra for the entire state space\n" );
		printf( "rma                 - RMA* (for one cop, lookup code for multiple cops version)\n" );
		printf( "minimax             - Minimax (one cop)\n" );
		printf( "minimax_optimized   - Minimax optimized (one cop)\n" );
		printf( "tida                - TIDA*\n" );
		printf( "ipn                 - IPN search without transposition tables\n" );
		printf( "ipnttables          - IPN with transposition tables\n" );
		printf( "tpdijkstra          - Two player dijkstra\n" );
		printf( "dstpdijkstra        - different speed two player dijkstra\n" );
		printf( "dsdijkstra          - different speed dijkstra\n" );
		printf( "dsdijkstra_memoptim - optimized different speed dijkstra\n" );
		printf( "dsrma               - different speed RMA*\n" );
		printf( "dscover             - different speed Cover heuristic\n" );
		printf( "dsheuristicgreedy   - different speed distance heuristic greedy algorithm\n" );
		printf( "dsminimax           - different speed minimax\n" );
		printf( "dsdam               - different speed dynamic abstract minimax\n" );
		printf( "dsdatpdijkstra      - different speed dynamic abstract two player dijkstra\n" );
		printf( "dsrandombeacons     - different speed random beacons\n" );
		printf( "\n" );
		printf( "testpoints          - generation of problem sets\n" );
		printf( "experiment_optimal  - experiment for \"Optimal Solutions to MTS\"\n" );
		printf( "experiment_allstate - Dijkstra vs. Markov\n" );
		printf( "experiment_graphs   - computation of some statistics on graphs\n" );
		printf( "website             - website interface\n" );
		printf( "experiment_suboptimal - experiment for \"Suboptimal Solutions to MTS\"\n" );
		return(1);
	}

	if( strcmp( argv[1], "testing" ) == 0 ) {
		compute_testing( argc, argv );
	}
	else if( strcmp( argv[1], "dijkstra" ) == 0 ) {
		compute_dijkstra( argc, argv );
	}
	else if( strcmp( argv[1], "rma" ) == 0 ) {
		compute_rma( argc, argv );
	}
	else if( strcmp( argv[1], "minimax" ) == 0 ) {
		compute_minimax( argc, argv );
	}
	else if( strcmp( argv[1], "minimax_optimized" ) == 0 ) {
		compute_minimax_optimized( argc, argv );
	}
	else if( strcmp( argv[1], "tida" ) == 0 ) {
		compute_tida( argc, argv );
	}
	else if( strcmp( argv[1], "ipn" ) == 0 ) {
		compute_ipn( argc, argv );
	}
	else if( strcmp( argv[1], "ipnttables" ) == 0 ) {
		compute_ipnttables( argc, argv );
	}
	else if( strcmp( argv[1], "tpdijkstra" ) == 0 ) {
		compute_tpdijkstra( argc, argv );
	}
	else if( strcmp( argv[1], "dstpdijkstra" ) == 0 ) {
		compute_dstpdijkstra( argc, argv );
	}
	else if( strcmp( argv[1], "dsdijkstra" ) == 0 ) {
		compute_dsdijkstra( argc, argv );
	}
	else if( strcmp( argv[1], "dsdijkstra_memoptim" ) == 0 ) {
		compute_dsdijkstra_memoptim( argc, argv );
	}
	else if( strcmp( argv[1], "dsrma" ) == 0 ) {
		compute_dsrma( argc, argv );
	}
	else if( strcmp( argv[1], "dscover" ) == 0 ) {
		compute_dscover( argc, argv );
	}
	else if( strcmp( argv[1], "dsheuristicgreedy" ) == 0 ) {
		compute_dsheuristicgreedy( argc, argv );
	}
	else if( strcmp( argv[1], "dsminimax" ) == 0 ) {
		compute_dsminimax( argc, argv );
	}
	else if( strcmp( argv[1], "dsdam" ) == 0 ) {
		compute_dsdam( argc, argv );
	}
	else if( strcmp( argv[1], "dsdatpdijkstra" ) == 0 ) {
		compute_dsdatpdijkstra( argc, argv );
	}
	else if( strcmp( argv[1], "dsrandombeacons" ) == 0 ) {
		compute_dsrandombeacons( argc, argv );
	}
	else if( strcmp( argv[1], "testpoints" ) == 0 ) {
		compute_testpoints( argc, argv );
	}
	else if( strcmp( argv[1], "experiment_optimal" ) == 0 ) {
		compute_experiment_optimal( argc, argv );
	}
	else if( strcmp( argv[1], "experiment_allstate" ) == 0 ) {
		compute_experiment_allstate( argc, argv );
	}
	else if( strcmp( argv[1], "experiment_graphs" ) == 0 ) {
		compute_experiment_graphs( argc, argv );
	}
	else if( strcmp( argv[1], "website" ) == 0 ) {
		compute_website_interface( argc, argv );
	}
	else if( strcmp( argv[1], "experiment_suboptimal" ) == 0 ) {
		compute_experiment_suboptimal( argc, argv );
	}
	else {
		printf( "ERROR: could not parse your algorithm/test parameter\n" );
		return 1;
	}

	return 0;
}




void output_syntax() {
	printf( "syntax: -c <num_cops> -g <graph>\n" );
	printf( "where <graph> is: <num_vertices> <num_edges>\n" );
	printf( "<edge1_vertice1> <edge1_vertice2> [<edge2_vertice1> <edge2_vertice2>...]\n" );
	return;
}

/*------------------------------------------------------------------------------
| Implementation of all algorithms
------------------------------------------------------------------------------*/
void compute_testing( int argc, char* argv[] ) {
/*
	// VISUALIZATION
	InstallHandlers();
	RunHOGGUI( argc, argv );
*/
/*
	// VALUE ITERATION WITH MULTIPLE LEVELS
	Map *m = new Map( "../../maps/local/test_coprobber_1.map" );

	unsigned int num_levels = 1;
	unsigned int num_iterations = 0;
	double precision = 0.1;
	double gamma = 1.0;

	MapCliqueAbstraction *mca = new MapCliqueAbstraction( m );
	MultilevelCopRobberGame* mcrgame = new MultilevelCopRobberGame( mca, 1, false, true );
	double **V;
	unsigned int *iter;
	mcrgame->GetExpectedStateRewards( 0, gamma, 0.01, precision, V, iter, num_levels, num_iterations );

	for( unsigned int i = 0; i < num_levels; i++ )
		delete[] V[i];

	delete[] V;
	delete[] iter;
	delete mcrgame;
	delete mca;
*/

/*
	// A NORMAL VALUE ITERATION WITHOUT LEVELS
	Graph *g = mca->GetAbstractGraph( 0 );
	MultilevelGraphHeuristic *gh = new MultilevelGraphHeuristic( mca, 0 );
	GraphEnvironment *genv = new GraphEnvironment( g, gh );
	CopRobberGame *game = new CopRobberGame( genv, 1, true, true );

	double *V = NULL;
	unsigned int iter = 0;

	game->GetExpectedStateRewardsSimultaneousActionGame( 0, 0.9, 0.01, precision, V, iter, num_iterations );
	char filename[250];
	sprintf( filename, "er_solution_level0.stat" );
	game->WriteExpectedRewardToDisc( filename, 0, V );

	delete[] V;
	delete game;
	delete genv;
	delete gh;
	delete mca;
*/


/*
	// OUTPUT OF NODE NUMBERS FOR THE MAP
	Graph *g = mca->GetAbstractGraph( 0 );
	node *n = NULL;
	recVec rv;
	int x, y;
	for( unsigned int i = 0; i < g->GetNumNodes(); i++ ) {
		n = g->GetNode( i );
		mca->GetTileFromNode( n, x, y );
		printf( "node %u: %d %d\n", i, x, y );
	}
*/

/*
	// CONVERGENCE/SPEED MEASUREMENTS FOR VALUE ITERATION
	int i, num_cops;
	unsigned int iter = 0;
	double precision, epsilon, *V = NULL, gamma;
	struct tms start, end;
	FILE *fhandler;
	bool canpass;

	fhandler = fopen( "stats.out", "w" );
	gamma = 0.9;
	epsilon = 0.01;
	precision = 10.0;
	num_cops = 1;
	canpass = true;

	fprintf( fhandler, "number of cops = %d\n", num_cops );
	fprintf( fhandler, "agents can pass = %d\n", canpass );
	fprintf( fhandler, "epsilon = %g\n", epsilon );
	fprintf( fhandler, "precision = %g\n", precision );
	fprintf( fhandler, "gamma = %g\n", gamma );
	fprintf( fhandler, "mapsize | number of nodes | number of states | " );
	fprintf( fhandler, "computation time (in s) | iterations in value iteration" );
	fprintf( fhandler, "\n\n" );

	fprintf( stdout, "number of cops = %d\n", num_cops );
	fprintf( stdout, "agents can pass = %d\n", canpass );
	fprintf( stdout, "epsilon = %g\n", epsilon );
	fprintf( stdout, "precision = %g\n", precision );
	fprintf( stdout, "gamma = %g\n", gamma );
	fprintf( stdout, "mapsize | number of nodes | number of states | " );
	fprintf( stdout, "computation time (in s) | iterations in value iteration" );
	fprintf( stdout, "\n\n" );

	for( i = 2; i <= 40; i++ ) {
		m = new Map( i, i );
		MakeMaze( m, 1 );

		// MDP learning code
		Graph *g = GraphSearchConstants::GetGraph( m );
		GraphMapHeuristic *gmh = new GraphMapHeuristic( m, g );
		GraphEnvironment *genv = new GraphEnvironment( g, gmh );

		// play the game with one robber
		CopRobberGame *game = new CopRobberGame( genv, num_cops, true, canpass );

		fprintf( stdout, "%d %d %u ", i, g->GetNumNodes(), game->GetNumStates() );
		fflush( stdout );

		times( &start );
		game->GetExpectedStateRewardsSimultaneousActionGame( 0, gamma, epsilon, precision, V, iter );
		times( &end );

		// write states down
		fprintf( fhandler, "%d %d %u %7.3f %u\n", i, g->GetNumNodes(),
			game->GetNumStates(),
			(double) ((end.tms_utime-start.tms_utime+end.tms_stime-start.tms_stime)/100.0),
			iter );
		fprintf( stdout, "%7.3f %u\n",
			(double) ((end.tms_utime-start.tms_utime+end.tms_stime-start.tms_stime)/100.0),
			iter );
		fflush( fhandler );
		fflush( stdout );

		// CLEANUP
		delete [] V;
		delete game;
		delete genv;
		delete gmh;
		delete g;
		delete m;
	}

	fclose( fhandler );
*/

/*
	// PRA* test code
	Map *m = new Map( "../../maps/bgmaps/AR0700SR.map" );
	MapCliqueAbstraction *mclab = new MapCliqueAbstraction( m );
	praStar *pra = new praStar();
	//pra->setPartialPathLimit( 10 );

	node *from = mclab->GetAbstractGraph(0)->GetNode(0);
	node *to   = mclab->GetAbstractGraph(0)->GetNode( mclab->GetAbstractGraph(0)->GetNumNodes() - 1 );

	path *p = pra->GetPath( mclab, from, to );
	path *ptemp = p;

	printf( "path: " );
	while( ptemp ) {
		printf( "%d ", ptemp->n->GetNum() );
		ptemp = ptemp->next;
	}
	printf( "\n" );

	delete ptemp;
	delete p;
	delete mclab;
*/


/*
	// test the heuristic for abstraction environments
	Map *m = new Map( "../../maps/local/test_coprobber_1.map" );
	MapCliqueAbstraction *mclab = new MapCliqueAbstraction( m );
	Graph *g = mclab->GetAbstractGraph( 0 );
	printf( "graph at level 0:\n" );
	int num = g->GetNumNodes();
	for( int i = 0; i < num; i++ ) {
		node *n = g->GetNode( i );
		printf( "i=%d : x@%g y@%g\n", i, n->GetLabelF(GraphAbstractionConstants::kXCoordinate) *m->getCoordinateScale(), n->GetLabelF(GraphAbstractionConstants::kYCoordinate) * m->getCoordinateScale() );
	}
	printf( "graph at level 1:\n" );
	g = mclab->GetAbstractGraph( 1 );
	num = g->GetNumNodes();
	for( int i = 0; i < num; i++ ) {
		node *n = g->GetNode( i );
		printf( "i=%d : x@%g y@%g\n", i, n->GetLabelF(GraphAbstractionConstants::kXCoordinate) *m->getCoordinateScale(), n->GetLabelF(GraphAbstractionConstants::kYCoordinate) * m->getCoordinateScale() );
	}

	g = mclab->GetAbstractGraph( 1 );
	MaximumNormAbstractGraphMapHeuristic *h = new MaximumNormAbstractGraphMapHeuristic( g, m );
	graphState f = 3, t = 4;
	printf( "h(0,1)@level 0 = %g\n", h->HCost( f, t ) );
*/

	InstallHandlers();
	RunHOGGUI( argc, argv );

	return;
}


// Dijkstra for the entire state space
void compute_dijkstra( int argc, char* argv[] ) {
	xyLoc pos_cop, pos_robber;
	Map *m;
	int max_depth;

	parseCommandLineParameters( argc, argv, m, pos_cop, pos_robber, max_depth );
	//printf( "map: %s\n", m->getMapName() );

	Graph *g = GraphSearchConstants::GetGraph( m );
	GraphMapHeuristic *gh = new GraphMapHeuristic( m, g );
	GraphEnvironment *env = new GraphEnvironment( g, gh );
	env->SetDirected( true ); // because the GetGraph routine
	// adds edges for both directions
	//FILE *fhandler = fopen( "testgraph", "r" );
	//Graph *g = readGraph( fhandler );
	//fclose( fhandler );
	//GraphEnvironment *env = new GraphEnvironment( g, NULL );
	//env->SetDirected( true );

	Dijkstra *d = new Dijkstra( env, 1, true );
	d->dijkstra();
	d->WriteValuesToDisk( "dijkstra.dat" );

	delete d;
	delete env;
	//delete gh;
	delete g;
	//delete m;
}


void compute_rma( int argc, char* argv[] ) {
	/*
	// Minimax A*
	xyLoc pos_cop, pos_robber;
	Map *m = NULL;
	Graph *g = NULL;
	MaximumNormGraphMapHeuristic *gh = NULL;
	GraphEnvironment *env = NULL;
	int curr, x, y;

	MinimaxAStar::CRState s;
	curr = 1;
	while( curr < argc ) {
		if( strcmp( argv[curr], "-map" ) == 0 ) {
			m = new Map( argv[curr+1] );
			g = GraphSearchConstants::GetGraph( m );
			gh = new MaximumNormGraphMapHeuristic( g );
			env = new GraphEnvironment( g, gh );
			env->SetDirected( true ); // because the GetGraph routine
	// adds edges for both directions
		}
		if( strcmp( argv[curr], "-pc" ) == 0 ) {
			sscanf( argv[curr+1], "%d,%d", &x, &y );
			printf( "x = %d, y = %d\n", x, y );
			s.push_back( m->getNodeNum( x, y ) );
		}
		if( strcmp( argv[curr], "-pr" ) == 0 ) {
			sscanf( argv[curr+1], "%d,%d", &x, &y );
			printf( "x = %d, y = %d\n", x, y );
			s.push_back( m->getNodeNum( x, y ) );
		}

		curr += 2;
	}
	printf( "map: %s\n", m->getMapName() );
	printf( "robber position: %lu\n", s[0] );
	printf( "cops positions: " );
	for( unsigned int i = 1; i < s.size(); i++ )
		printf( "%lu ", s[i] );
	printf( "\n" );

	MinimaxAStar *astar = new MinimaxAStar( env, 2, true );

	double result = astar->astar( s, true, 1. );
	fprintf( stdout, "result: %f\n", result );

	delete astar;
	delete env;
	delete gh;
	delete g;
	delete m;
*/


/*
	// code to compute for large mazes
	Map *m = new Map( 64, 64 );
	MakeMaze( m );
	m->scale( 180, 180 );
//	m->save( "testmap.map" );
	Graph *g = GraphSearchConstants::GetGraph( m );
	MapEnvironment *env = new MapEnvironment( m );
	clock_t clock_start, clock_end;
	node *n = g->GetRandomNode();
	xyLoc pos_robber( n->GetLabelL( GraphSearchConstants::kMapX ), n->GetLabelL( GraphSearchConstants::kMapY ) );
	std::vector<node*> *reachableNodes = g->getReachableNodes( n );
	n = (*reachableNodes)[random()%reachableNodes->size()];
	xyLoc pos_cop( n->GetLabelL( GraphSearchConstants::kMapX ), n->GetLabelL( GraphSearchConstants::kMapY ) );
*/


	// Optimized Minimax A*
	xyLoc pos_cop, pos_robber;
	Map *m;
	int max_depth;

	parseCommandLineParameters( argc, argv, m, pos_cop, pos_robber, max_depth );
	printf( "map: %s\n", m->getMapName() );
	printf( "cop position: %d,%d\n", pos_cop.x, pos_cop.y );
	printf( "robber position: %d,%d\n", pos_robber.x, pos_robber.y );

	MapEnvironment *env = new MapEnvironment( m );
	MinimaxAStar<xyLoc,tDirection,MapEnvironment> *astar =
		new MinimaxAStar<xyLoc,tDirection,MapEnvironment>( env, 1, true );
//	astar->set_useHeuristic( false );
//	printf( "Computing perfect distance metric ... " ); fflush( stdout );
//	astar->set_usePerfectDistanceHeuristic( true );
//	printf( "done.\n" ); fflush( stdout );
	std::vector<xyLoc> s;
	s.push_back( pos_robber );
	s.push_back( pos_cop );

	double       result        = astar->astar( s, true );
	unsigned int nodesExpanded = astar->nodesExpanded;
	unsigned int nodesTouched  = astar->nodesTouched;

	fprintf( stdout, "result: %f\n", result );
	fprintf( stdout, "nodesExpanded: %u\n", nodesExpanded );
	fprintf( stdout, "nodesTouched: %u\n", nodesTouched );

	delete astar;
	delete env;
}


// NORMAL MINIMAX
void compute_minimax( int argc, char* argv[] ) {
	Map *m;
	xyLoc pos_cop, pos_robber;
	int max_depth;
	double minimax;
	std::vector<Minimax<xyLoc,tDirection,MapEnvironment>::CRState> path;

	parseCommandLineParameters( argc, argv, m, pos_cop, pos_robber, max_depth );
	printf( "map: %s\n", m->getMapName() );
	printf( "cop position: %d,%d\n", pos_cop.x, pos_cop.y );
	printf( "robber position: %d,%d\n", pos_robber.x, pos_robber.y );

	MapEnvironment *env = new MapEnvironment( m );

	Minimax<xyLoc,tDirection,MapEnvironment> *minclass = new Minimax<xyLoc,tDirection,MapEnvironment>( env, true );

	std::vector<xyLoc> pos;
	pos.push_back( pos_robber ); pos.push_back( pos_cop );

	minimax = minclass->minimax( pos, path, true, max_depth );

	printf( "result: %f\n", minimax );
	printf( "reached level %d in the tree\n", minclass->max_depth_reached );
	fprintf( stdout, "solution is:\n" );
	for( unsigned int i = 0; i < path.size(); i++ ) {
		fprintf( stdout, "(%u,%u) (%u,%u)\n", path[i][0].x, path[i][0].y, path[i][1].x, path[i][1].y );
		if( (i-1) % 2 ) fprintf( stdout, "----\n" );
	}

	delete minclass;
	delete env;
}


// optimized Minimax
void compute_minimax_optimized( int argc, char* argv[] ) {
	Map *m;
	xyLoc pos_cop, pos_robber;
	int max_depth;
	std::vector<Minimax<xyLoc,tDirection,MapEnvironment>::CRState> path;

	parseCommandLineParameters( argc, argv, m, pos_cop, pos_robber, max_depth );
	printf( "map: %s\n", m->getMapName() );
	printf( "cop position: %d,%d\n", pos_cop.x, pos_cop.y );
	printf( "robber position: %d,%d\n", pos_robber.x, pos_robber.y );

	MapEnvironment *env = new MapEnvironment( m );
	MinimaxOptimized<xyLoc,tDirection,MapEnvironment> *minclass =
		new MinimaxOptimized<xyLoc,tDirection,MapEnvironment>( env, true );
	std::vector<xyLoc> pos;
	pos.push_back( pos_robber ); pos.push_back( pos_cop );
	
	double result = minclass->minimax( pos, true, max_depth );
	printf( "result: %f\n", result );
	delete minclass;
	delete env;
}


// TIDA*
void compute_tida( int argc, char* argv[] ) {
/*
	Map *m;
	xyLoc pos_cop, pos_robber;
	int max_recursion_level;

	parseCommandLineParameters( argc, argv, m, pos_cop, pos_robber, max_recursion_level );
	printf( "map: %s\n", m->getMapName() );
	printf( "cop position: %d,%d\n", pos_cop.x, pos_cop.y );
	printf( "robber position: %d,%d\n", pos_robber.x, pos_robber.y );

	MapEnvironment *env = new MapEnvironment( m );

	TIDAStar<xyLoc,tDirection,MapEnvironment> *tidastar = new TIDAStar<xyLoc,tDirection,MapEnvironment>( env, true );

	std::vector<xyLoc> pos;
	pos.push_back( pos_robber ); pos.push_back( pos_cop );
	std::vector<TIDAStar<xyLoc,tDirection,MapEnvironment>::CRState> path;

	double minimax = tidastar->tida( pos, max_recursion_level, path, 1. );

	fprintf( stdout, "tida result: %f\n", minimax );
	fprintf( stdout, "maximal depth of the computation tree: %u\n", tidastar->maxDepthReached );
	fprintf( stdout, "solution is:\n" );
	for( unsigned int i = 0; i < path.size(); i++ ) {
		fprintf( stdout, "(%u,%u) (%u,%u)\n", path[i][0].x, path[i][0].y, path[i][1].x, path[i][1].y );
		if( (i-1) % 2 ) fprintf( stdout, "----\n" );
	}
	fprintf( stdout, "nodes expanded: %u ( ", tidastar->nodesExpanded );
	for( unsigned int i = 0; i < tidastar->iteration_nodesExpanded.size(); i++ )
		fprintf( stdout, "%u ", tidastar->iteration_nodesExpanded[i] );
	fprintf( stdout, ")\n" );
	fprintf( stdout, "nodes touched: %u ( ", tidastar->nodesTouched );
	for( unsigned int i = 0; i < tidastar->iteration_nodesTouched.size(); i++ )
		fprintf( stdout, "%u ", tidastar->iteration_nodesTouched[i] );
	fprintf( stdout, ")\n" );

	delete tidastar;
	delete env;
*/


	// optimized TIDA*
	Map *m;
	xyLoc pos_cop, pos_robber;
	int max_recursion_level;

	parseCommandLineParameters( argc, argv, m, pos_cop, pos_robber, max_recursion_level );
	printf( "map: %s\n", m->getMapName() );
	printf( "cop position: %d,%d\n", pos_cop.x, pos_cop.y );
	printf( "robber position: %d,%d\n", pos_robber.x, pos_robber.y );

	MapEnvironment *env = new MapEnvironment( m );

	TIDAStar<xyLoc,tDirection,MapEnvironment> *tidastar = new TIDAStar<xyLoc,tDirection,MapEnvironment>( env, true );

	std::vector<xyLoc> pos;
	pos.push_back( pos_robber ); pos.push_back( pos_cop );

	double minimax = tidastar->tida( pos );
	minimax = tidastar->tida( pos );

	fprintf( stdout, "tida result: %f\n", minimax );
	fprintf( stdout, "nodes expanded: %u ( ", tidastar->nodesExpanded );
	for( unsigned int i = 0; i < tidastar->iteration_nodesExpanded.size(); i++ )
		fprintf( stdout, "%u ", tidastar->iteration_nodesExpanded[i] );
	fprintf( stdout, ")\n" );
	fprintf( stdout, "nodes touched: %u ( ", tidastar->nodesTouched );
	for( unsigned int i = 0; i < tidastar->iteration_nodesTouched.size(); i++ )
		fprintf( stdout, "%u ", tidastar->iteration_nodesTouched[i] );
	fprintf( stdout, ")\n" );

	delete tidastar;
	delete env;
}


// IPN-Search
void compute_ipn( int argc, char* argv[] ) {
	Map *m;
	xyLoc pos_cop, pos_robber;
	int max_recursion_level;

	parseCommandLineParameters( argc, argv, m, pos_cop, pos_robber, max_recursion_level );
	printf( "map: %s\n", m->getMapName() );
	printf( "cop position: %d,%d\n", pos_cop.x, pos_cop.y );
	printf( "robber position: %d,%d\n", pos_robber.x, pos_robber.y );

	MapEnvironment *env = new MapEnvironment( m );

	IPNSearch<xyLoc,tDirection,MapEnvironment> *ipns = new IPNSearch<xyLoc,tDirection,MapEnvironment>( env, true );

	std::vector<xyLoc> pos;
	pos.push_back( pos_robber ); pos.push_back( pos_cop );

	double minimax = ipns->ipn( pos, true );

	fprintf( stdout, "ipn search result: %f\n", minimax );

	delete ipns;
	delete env;
}


// IPN-Search with transposition tables
void compute_ipnttables( int argc, char* argv[] ) {
	Map *m;
	xyLoc pos_cop, pos_robber;
	int max_recursion_level;

	parseCommandLineParameters( argc, argv, m, pos_cop, pos_robber, max_recursion_level );
	printf( "map: %s\n", m->getMapName() );
	printf( "cop position: %d,%d\n", pos_cop.x, pos_cop.y );
	printf( "robber position: %d,%d\n", pos_robber.x, pos_robber.y );

	MapEnvironment *env = new MapEnvironment( m );

	IPNTTables<xyLoc,tDirection,MapEnvironment> *ipntt = new IPNTTables<xyLoc,tDirection,MapEnvironment>( env, true );

	std::vector<xyLoc> pos;
	pos.push_back( pos_robber ); pos.push_back( pos_cop );

	double minimax = ipntt->ipn( pos, true );

	fprintf( stdout, "ipn search result: %f\n", minimax );
	fprintf( stdout, "nodes expanded: %u ( ", ipntt->nodesExpanded );
	for( unsigned int i = 0; i < ipntt->iteration_nodesExpanded.size(); i++ )
		fprintf( stdout, "%u ", ipntt->iteration_nodesExpanded[i] );
	fprintf( stdout, ")\n" );
	fprintf( stdout, "nodes touched: %u ( ", ipntt->nodesTouched );
	for( unsigned int i = 0; i < ipntt->iteration_nodesTouched.size(); i++ )
		fprintf( stdout, "%u ", ipntt->iteration_nodesTouched[i] );
	fprintf( stdout, ")\n" );
	fprintf( stdout, "nodes updated: %u ( ", ipntt->nodesUpdated );
	for( unsigned int i = 0; i < ipntt->iteration_nodesUpdated.size(); i++ )
		fprintf( stdout, "%u ", ipntt->iteration_nodesUpdated[i] );
	fprintf( stdout, ")\n" );

	delete ipntt;
	delete env;
}


// Two Player Dijkstra
void compute_tpdijkstra( int argc, char* argv[] ) {
	Map *m;
	xyLoc pos_cop, pos_robber;
	int max_recursion_level;

	parseCommandLineParameters( argc, argv, m, pos_cop, pos_robber, max_recursion_level );
	printf( "map: %s\n", m->getMapName() );
	printf( "cop position: %d,%d\n", pos_cop.x, pos_cop.y );
	printf( "robber position: %d,%d\n", pos_robber.x, pos_robber.y );

	MapEnvironment *env = new MapEnvironment( m );
	TwoPlayerDijkstra<xyLoc,tDirection,MapEnvironment> *tpd = new TwoPlayerDijkstra<xyLoc,tDirection,MapEnvironment>( env, true );

	double result = tpd->tpdijkstra( pos_robber, pos_cop );
	printf( "result: %f\n", result );
	printf( "nodes expanded: %u\n", tpd->nodesExpanded );
	printf( "nodes touched: %u\n", tpd->nodesTouched );

	delete tpd;
	delete env;
}


// Two Player Dijkstra with move generation and possibly fast cop
// DSTPDijkstra
void compute_dstpdijkstra( int argc, char* argv[] ) {
	Map *m;
	xyLoc pos_cop, pos_robber;
	int max_recursion_level;
	std::vector<xyLoc> path;

	parseCommandLineParameters( argc, argv, m, pos_cop, pos_robber, max_recursion_level );
	printf( "map: %s\n", m->getMapName() );
	printf( "cop position: %d,%d\n", pos_cop.x, pos_cop.y );
	printf( "robber position: %d,%d\n", pos_robber.x, pos_robber.y );

	MapEnvironment *env = new MapEnvironment( m );
	DSTPDijkstra<xyLoc,tDirection> *dstpdijkstra = new DSTPDijkstra<xyLoc,tDirection>( env, 2 );

	// attention: robber moves first here
	double result = dstpdijkstra->dstpdijkstra( pos_robber, pos_cop, false, path );
	printf( "result: %f\n", result );
	printf( "nodes expanded: %u\n", dstpdijkstra->nodesExpanded );
	printf( "nodes touched: %u\n", dstpdijkstra->nodesTouched );
	printf( "path (%u): ", (unsigned int)path.size() );
	for( std::vector<xyLoc>::iterator it = path.begin(); it != path.end(); it++ )
		printf( "(%u,%u) ", it->x, it->y );
	printf( "\n" );

	delete dstpdijkstra;
	delete env;
}


// Dijkstra with different speeds for the entire state space
// DSDijkstra
void compute_dsdijkstra( int argc, char* argv[] ) {
	xyLoc pos_cop, pos_robber;
	Map *m;
	int max_depth;

	parseCommandLineParameters( argc, argv, m, pos_cop, pos_robber, max_depth );
	printf( "map: %s\n", m->getMapName() );

	MapEnvironment *env = new MapEnvironment( m );

	DSDijkstra<xyLoc,tDirection,MapEnvironment> *dsdijkstra = new DSDijkstra<xyLoc,tDirection,MapEnvironment>( env, 1 );

	dsdijkstra->dsdijkstra();
	dsdijkstra->WriteValuesToDisk( "dsdijkstra.dat" );

	delete dsdijkstra;
	delete env;
}


// Dijkstra with different speeds for the entire state space
// DSDijkstra_MemOptim
void compute_dsdijkstra_memoptim( int argc, char* argv[] ) {
	xyLoc pos_cop, pos_robber;
	Map *m;
	unsigned int cop_speed = 2;
	int max_depth;

	parseCommandLineParameters( argc, argv, m, pos_cop, pos_robber, max_depth );
	printf( "map: %s\n", m->getMapName() );

	Graph *g = GraphSearchConstants::GetGraph( m );
	GraphEnvironment *env = new GraphEnvironment( g, NULL );
	env->SetDirected( true );

	DSDijkstra_MemOptim *dsdijkstra = new DSDijkstra_MemOptim( env, cop_speed );

	dsdijkstra->dsdijkstra();

	dsdijkstra->WriteValuesToDisk( "dsdijkstra.dat" );
	printf( "nodes expanded: %u\n", dsdijkstra->nodesExpanded );
	printf( "nodes touched: %u\n", dsdijkstra->nodesTouched );

	delete dsdijkstra;
	delete env;
	delete g;
	delete m;
}



// RMA* with possibly faster cop => DSRMAStar
void compute_dsrma( int argc, char* argv[] ) {
	xyLoc pos_cop, pos_robber;
	Map *m;
	int max_depth;

	parseCommandLineParameters( argc, argv, m, pos_cop, pos_robber, max_depth );
	printf( "map: %s\n", m->getMapName() );
	printf( "cop position: %d,%d\n", pos_cop.x, pos_cop.y );
	printf( "robber position: %d,%d\n", pos_robber.x, pos_robber.y );
	MapEnvironment *env = new MapEnvironment( m );

	DSRMAStar<xyLoc,tDirection,MapEnvironment> *dsrmastar =
		new DSRMAStar<xyLoc,tDirection,MapEnvironment>( env, 1 );
	std::vector<xyLoc> s;
	s.push_back( pos_robber );
	s.push_back( pos_cop );

	double       result        = dsrmastar->rmastar( s, true );
	unsigned int nodesExpanded = dsrmastar->nodesExpanded;
	unsigned int nodesTouched  = dsrmastar->nodesTouched;

	fprintf( stdout, "result: %f\n", result );
	fprintf( stdout, "nodesExpanded: %u\n", nodesExpanded );
	fprintf( stdout, "nodesTouched: %u\n", nodesTouched );

	delete dsrmastar;
	delete env;
}


// DSCover
// Cover for one cop and one robber with DS move generation
void compute_dscover( int argc, char* argv[] ) {
	Map *m;
	xyLoc pos_cop, pos_robber;
	int max_recursion_level;
	bool who;

	parseCommandLineParameters( argc, argv, m, pos_cop, pos_robber, max_recursion_level );
	printf( "map: %s\n", m->getMapName() );
	printf( "cop position: %d,%d\n", pos_cop.x, pos_cop.y );
	printf( "robber position: %d,%d\n", pos_robber.x, pos_robber.y );

	Graph *g = GraphSearchConstants::GetGraph( m );
	GraphEnvironment *env = new GraphEnvironment( g, NULL );
	env->SetDirected( true );

	DSCover<graphState,graphMove> *dscover = new DSCover<graphState,graphMove>( env, 1 );

	unsigned int c = dscover->cover( m->getNodeNum( pos_robber.x, pos_robber.y ),
		m->getNodeNum( pos_cop.x, pos_cop.y ), true, who );

	if( who )
		printf( "cop" );
	else
		printf( "robber" );
	printf( " cover: %u\n", c );
	printf( "nodes expanded: %u\n", dscover->nodesExpanded );
	printf( "nodes touched: %u\n", dscover->nodesTouched );

	graphState s;
	s = dscover->MakeMove( m->getNodeNum(pos_robber.x,pos_robber.y), m->getNodeNum(pos_cop.x,pos_cop.y), false, g->GetNumNodes() );
	printf( "next move for the robber: %lu\n", s );
	s = dscover->MakeMove( m->getNodeNum(pos_robber.x,pos_robber.y), m->getNodeNum(pos_cop.x,pos_cop.y), true, g->GetNumNodes() );
	printf( "next move for the cop: %lu\n", s );

	delete dscover;
	delete env;
	delete g;
	delete m;
}


// DSHeuristicGreedy
void compute_dsheuristicgreedy( int argc, char* argv[] ) {
	Map *m;
	xyLoc pc, pr;
	graphState pos_cop, pos_robber;
	int max_recursion_level;

	parseCommandLineParameters( argc, argv, m, pc, pr, max_recursion_level );
	printf( "map: %s\n", m->getMapName() );
	printf( "cop position: %d,%d\n", pc.x, pc.y );
	printf( "robber position: %d,%d\n", pr.x, pr.y );

	Graph *g = GraphSearchConstants::GetGraph( m );
	MaximumNormGraphMapHeuristic *gh = new MaximumNormGraphMapHeuristic( g );
	GraphEnvironment *env = new GraphEnvironment( g, gh );
	env->SetDirected( true );

	DSHeuristicGreedy<graphState,graphMove> *dshg = new DSHeuristicGreedy<graphState,graphMove>( env, true, 1 );

	pos_robber = m->getNodeNum( pr.x, pr.y );
	pos_cop    = m->getNodeNum( pc.x, pc.y );
	printf( "next move for the robber: %lu\n", dshg->MakeMove( pos_robber, pos_cop, false ) );
	printf( "next move for the cop: %lu\n", dshg->MakeMove( pos_robber, pos_cop, true ) );

	delete dshg;
	delete env;
	delete gh;
	delete g;
	delete m;
}



// DSMinimax
void compute_dsminimax( int argc, char* argv[] ) {
	Map *m;
	xyLoc pc, pr;
	graphState pos_cop, pos_robber;
	std::vector<graphState> path;
	int depth;

	parseCommandLineParameters( argc, argv, m, pc, pr, depth );
	printf( "map: %s\n", m->getMapName() );
	printf( "cop position: %d,%d\n", pc.x, pc.y );
	printf( "robber position: %d,%d\n", pr.x, pr.y );
	printf( "computation depth: %d\n", depth );

	MapCliqueAbstraction *mclab = new MapCliqueAbstraction( m );
	Graph *g = mclab->GetAbstractGraph( 0 );
//	Graph *g = GraphSearchConstants::GetGraph( m );
//	MaximumNormGraphMapHeuristic *gh = new MaximumNormGraphMapHeuristic( g );
	MaximumNormAbstractGraphMapHeuristic *gh = new MaximumNormAbstractGraphMapHeuristic( g, m );
	GraphEnvironment *env = new GraphEnvironment( g, gh );
//	env->SetDirected( true );

	DSMinimax<graphState,graphMove> *dsminimax = new DSMinimax<graphState,graphMove>( env, true, 2 );

	pos_robber = m->getNodeNum( pr.x, pr.y );
	pos_cop    = m->getNodeNum( pc.x, pc.y );
	double result = dsminimax->minimax( pos_robber, pos_cop, path, false, depth );

	printf( "result of the computation: %g\n", result );
	printf( "nodes expanded: %u\n", dsminimax->nodesExpanded );
	printf( "nodes touched: %u\n", dsminimax->nodesTouched );
	printf( "moves: " );
	for( std::vector<graphState>::iterator it = path.begin(); it != path.end(); it++ ) {
		printf( "=> %lu ", *it );
	}
	printf( "\n" );

	delete dsminimax;
	delete env;
	delete gh;
	delete g;
	delete m;
}


// DSDAM (DS dynamic abstract minimax)
void compute_dsdam( int argc, char* argv[] ) {
	Map *m;
	xyLoc pc, pr;
	int depth;

	parseCommandLineParameters( argc, argv, m, pc, pr, depth );
	MapCliqueAbstraction *mclab = new MapCliqueAbstraction( m );
	printf( "map: %s\n", m->getMapName() );
	printf( "cop position: %d,%d (%d)\n", pc.x, pc.y, mclab->GetNodeFromMap( pc.x, pc.y )->GetNum() );
	printf( "robber position: %d,%d (%d)\n", pr.x, pr.y, mclab->GetNodeFromMap( pr.x, pr.y )->GetNum() );
	printf( "computation depth: %d\n", depth );

	//printf( "\n\nGraphs:\n" );
	//writeGraph( stdout, mclab->GetAbstractGraph(0) );
	//writeGraph( stdout, mclab->GetAbstractGraph(1) );

	DSDAM *dsdam = new DSDAM( mclab, true, 2, true );
	std::vector<node*> path;
	dsdam->dam( mclab->GetNodeFromMap( pr.x, pr.y ), mclab->GetNodeFromMap( pc.x, pc.y ), path, false, depth );
	printf( "path: " );
	for( std::vector<node*>::iterator it = path.begin(); it != path.end(); it++ ) {
		printf( "%u (%d) ", (*it)->GetNum(), (*it)->getUniqueID() );
	}
	printf( "\n" );
	printf( "nodesExpanded: %u\n", dsdam->nodesExpanded );
	printf( "nodesTouched: %u\n", dsdam->nodesTouched );

	delete dsdam;
	delete mclab;
}


// DSDATPDijkstra (DS dynamic abstract two player dijkstra)
void compute_dsdatpdijkstra( int argc, char* argv[] ) {
	Map *m;
	xyLoc pc, pr;
	int minimum_escape_length;

	parseCommandLineParameters( argc, argv, m, pc, pr, minimum_escape_length );
	MapCliqueAbstraction *mclab = new MapCliqueAbstraction( m );
	printf( "map: %s\n", m->getMapName() );
	printf( "cop position: %d,%d (%d)\n", pc.x, pc.y, mclab->GetNodeFromMap( pc.x, pc.y )->GetNum() );
	printf( "robber position: %d,%d (%d)\n", pr.x, pr.y, mclab->GetNodeFromMap( pr.x, pr.y )->GetNum() );
	printf( "minimum escape length: %d\n", minimum_escape_length );

	DSDATPDijkstra *dsdatpdijkstra = new DSDATPDijkstra( mclab, 2, true );
	std::vector<node*> path;
	dsdatpdijkstra->datpdijkstra( mclab->GetNodeFromMap( pr.x, pr.y ), mclab->GetNodeFromMap( pc.x, pc.y ), path, false, minimum_escape_length );
	printf( "path: " );
	for( std::vector<node*>::iterator it = path.begin(); it != path.end(); it++ ) {
		printf( "%u ", (*it)->GetNum() );
	}
	printf( "\n" );
	printf( "nodesExpanded: %u\n", dsdatpdijkstra->myNodesExpanded );
	printf( "nodesTouched: %u\n", dsdatpdijkstra->myNodesTouched );

	delete dsdatpdijkstra;
	delete mclab;
}


// DSRandomBeacons
void compute_dsrandombeacons( int argc, char* argv[] ) {
	Map *m;
	xyLoc pc, pr;
	graphState pos_cop, pos_robber;
	int max_recursion_level;

	parseCommandLineParameters( argc, argv, m, pc, pr, max_recursion_level );
	printf( "map: %s\n", m->getMapName() );
	printf( "cop position: %d,%d\n", pc.x, pc.y );
	printf( "robber position: %d,%d\n", pr.x, pr.y );

	MapCliqueAbstraction *mclab = new MapCliqueAbstraction( m );
	DSRandomBeacons *dsrandb = new DSRandomBeacons( mclab, true, 2 );

	pos_robber = m->getNodeNum( pr.x, pr.y );
	pos_cop    = m->getNodeNum( pc.x, pc.y );
	std::vector<graphState> path;
	dsrandb->GetPath( pos_robber, pos_cop, 40, path );

	printf( "path: " );
	for( std::vector<graphState>::iterator it = path.begin(); it != path.end(); it++ ) {
		printf( "%lu ", *it );
	}
	printf( "\n" );

	delete dsrandb;
	delete mclab;
}

/*------------------------------------------------------------------------------
| Implementation of tests
------------------------------------------------------------------------------*/

// problem set generation
void compute_testpoints( int argc, char* argv[] ) {
	char map_file[20];
	char problem_file[20] = "problem_set5.dat";
	Map *m;
	time_t t; time( &t ); srandom( (unsigned int) t );
	unsigned int num;
	int j;
	int i;
	// sets for experiment_optimality:
	// problem set 1: i = 1:15, scaled to size 15
	// problem set 2: i = 6:20, scaled to size 20
	// problem set 3: i = 16:35, scaled to size 40
	// problem set 4: i = 20:39, scaled to size 60
	// problem set 5: i = 30:49, scaled to size 80
	// note: experiment_suboptimality used /maps/bgmaps
	FILE *fhandler = fopen( problem_file, "w" );
//	FILE *file_with_maps = fopen( argv[1], "r" );
//	while( !feof( file_with_maps ) ) {
	for( i = 30; i <= 49; i++ ) {
		m = new Map( i, i );
		MakeMaze( m );
		m->scale( 80, 80 );
		sprintf( map_file, "problem_set5_map%d.map", i );
		m->save( map_file );
//		fscanf( file_with_maps, "%s\n", map_file );
//		fprintf( fhandler, "%s\n", map_file );
//		m = new Map( map_file );
		MapEnvironment *env = new MapEnvironment( m );
		Graph *g = GraphSearchConstants::GetGraph( m );
		for( j = 0; j < 1000; ) {
			// generate random position for the robber
			num = (unsigned int)floor(
				(double)random()/(double)RAND_MAX * (double)g->GetNumNodes());
			unsigned int rx = g->GetNode(num)->GetLabelL(GraphSearchConstants::kMapX);
			unsigned int ry = g->GetNode(num)->GetLabelL(GraphSearchConstants::kMapY);

			// generate random position for the cop
			// within the nodes that can be reached from the robber ;-)
			std::vector<node*>* reachable_nodes = g->getReachableNodes( g->GetNode( num ) );
			num = (unsigned int)floor(
				(double)random()/(double)RAND_MAX * (double)(reachable_nodes->size()-1) );
			unsigned int cx = (*reachable_nodes)[num]->GetLabelL(GraphSearchConstants::kMapX);
			unsigned int cy = (*reachable_nodes)[num]->GetLabelL(GraphSearchConstants::kMapY);
			delete reachable_nodes;

//			TIDAStar<xyLoc,tDirection,MapEnvironment> *tidastar =
//				new TIDAStar<xyLoc,tDirection,MapEnvironment>( env, true );
//			std::vector<xyLoc> pos;
//			pos.push_back( xyLoc(rx,ry) ); pos.push_back( xyLoc(cx,cy) );
//			double result = tidastar->tida( pos );
//			delete tidastar;

//			if( result <= 35. ) {
//				fprintf( stdout, "%f\n", result );
				// print out the starting position for the robber
				fprintf( fhandler, "(%u,%u) ", rx, ry );
				// print out the starting position for the cop
				fprintf( fhandler, "(%u,%u) ", cx, cy );
				// print out the map that we are in
				fprintf( fhandler, "%s\n", map_file );
				j++;
//			}
		}
	
		delete env;
		//delete m;
	}
	fclose( fhandler );
//	fclose( file_with_maps );
}




// TESTs for "Optimal solutions for Moving Target Search"
void compute_experiment_optimal( int argc, char* argv[] ) {
	char map_file[100], old_map_file[100];
	strcpy( map_file, "" );
	FILE *problem_file, *result_file, *tida_file_handler = NULL;
	clock_t clock_start, clock_end;
	clock_start = clock_end = clock();
	unsigned int rx,ry,cx,cy;
	double result = 0;
	unsigned int nodesExpanded = 0, nodesTouched = 0;
	Map *m = NULL; MapEnvironment *env = NULL;

	if( argc < 5 ) {
		printf( "Syntax: <problem set file> <algorithm> <result file>\n" );
		printf( "where <algorithm> = tida|rma|ipn|minimax|tpdijkstra\n" );
		exit(1);
	}

	problem_file = fopen( argv[2], "r" );
	result_file  = fopen( argv[4], "w" );

	// in case we want to compute with minimax
	// we have to get the correct values from a file (hello to TIDA*)
	if( strcmp( argv[3], "minimax" ) == 0 ) {
			char s[100];
			sprintf( s, "tida_%s", argv[2] );
			tida_file_handler = fopen( s, "r" );
			if( tida_file_handler == NULL ) {
				fprintf( stderr, "ERROR: could not find tida value file\n" );
				exit(1);
			}
	}


	TIDAStar<xyLoc,tDirection,MapEnvironment> *tidastar = NULL;
	IPNTTables<xyLoc,tDirection,MapEnvironment> *ipntt = NULL;
	MinimaxAStar<xyLoc,tDirection,MapEnvironment> *astar = NULL;
	MinimaxAStar<xyLoc,tDirection,MapEnvironment> *astar_dijkstra = NULL;
	MinimaxAStar<xyLoc,tDirection,MapEnvironment> *astar_perfecth = NULL;
	MinimaxOptimized<xyLoc,tDirection,MapEnvironment> *minclass = NULL;
	TwoPlayerDijkstra<xyLoc,tDirection,MapEnvironment> *tpd = NULL;

	if( !feof( problem_file ) ) {
		fscanf( problem_file, "(%u,%u) (%u,%u) %s\n", &rx,&ry,&cx,&cy,map_file );
		strcpy( old_map_file, map_file );
		m = new Map( map_file );
		env = new MapEnvironment( m );

		tidastar       = new TIDAStar<xyLoc,tDirection,MapEnvironment>( env, true );
		ipntt          = new IPNTTables<xyLoc,tDirection,MapEnvironment>( env, true );
		astar          = new MinimaxAStar<xyLoc,tDirection,MapEnvironment>( env, 1, true );
		astar_dijkstra = new MinimaxAStar<xyLoc,tDirection,MapEnvironment>( env, 1, true );
		astar_dijkstra->set_useHeuristic( false );
		astar_perfecth = new MinimaxAStar<xyLoc,tDirection,MapEnvironment>( env, 1, true );
		astar_perfecth->set_usePerfectDistanceHeuristic( true );
		minclass       = new MinimaxOptimized<xyLoc,tDirection,MapEnvironment>( env, true );
		tpd            = new TwoPlayerDijkstra<xyLoc,tDirection,MapEnvironment>( env, true );
	}

	
	// for all the problems in the problem set file
	while( true && strcmp( map_file, "" ) != 0 ) {

		if( strcmp( argv[3], "tida" ) == 0 ) {
			// if we want to test TIDA*
			std::vector<xyLoc> pos;
			pos.push_back( xyLoc(rx,ry) ); pos.push_back( xyLoc(cx,cy) );

			clock_start   = clock();
			result        = tidastar->tida( pos );
			clock_end     = clock();
			nodesExpanded = tidastar->nodesExpanded;
			nodesTouched  = tidastar->nodesTouched;
		}

		if( strcmp( argv[3], "ipn" ) == 0 ) {
			std::vector<xyLoc> pos;
			pos.push_back( xyLoc(rx,ry) ); pos.push_back( xyLoc(cx,cy) );

			clock_start   = clock();
			result        = ipntt->ipn( pos, true );
			clock_end     = clock();
			nodesExpanded = ipntt->nodesExpanded;
			nodesTouched  = ipntt->nodesTouched;
		}

		if( strcmp( argv[3], "rma" ) == 0 ) {
			std::vector<xyLoc> s;
			s.push_back( xyLoc( rx, ry ) );
			s.push_back( xyLoc( cx, cy ) );

			clock_start   = clock();
			result        = astar->astar( s, true );
			clock_end     = clock();
			nodesExpanded = astar->nodesExpanded;
			nodesTouched  = astar->nodesTouched;
		}

		if( strcmp( argv[3], "rma_dijkstra" ) == 0 ) {
			std::vector<xyLoc> s;
			s.push_back( xyLoc( rx, ry ) );
			s.push_back( xyLoc( cx, cy ) );

			clock_start   = clock();
			result        = astar_dijkstra->astar( s, true );
			clock_end     = clock();
			nodesExpanded = astar_dijkstra->nodesExpanded;
			nodesTouched  = astar_dijkstra->nodesTouched;
		}

		if( strcmp( argv[3], "rma_perfect" ) == 0 ) {
			std::vector<xyLoc> s;
			s.push_back( xyLoc( rx, ry ) );
			s.push_back( xyLoc( cx, cy ) );

			clock_start   = clock();
			result        = astar_perfecth->astar( s, true );
			clock_end     = clock();
			nodesExpanded = astar_perfecth->nodesExpanded;
			nodesTouched  = astar_perfecth->nodesTouched;
		}

		if( strcmp( argv[3], "minimax" ) == 0 ) {
			double tida_value;
			fscanf( tida_file_handler, "(%*u,%*u) (%*u,%*u) %lf %*u %*u %*u %*s\n",
				&tida_value );

			std::vector<xyLoc> pos;
			pos.push_back( xyLoc(rx,ry) ); pos.push_back( xyLoc(cx,cy) );

			clock_start   = clock();
			result        = minclass->minimax( pos, true, (int)floor(tida_value) );
			clock_end     = clock();
			nodesExpanded = minclass->nodesExpanded;
			nodesTouched  = minclass->nodesTouched;
		}

		if( strcmp( argv[3], "tpdijkstra" ) == 0 ) {
			clock_start   = clock();
			result        = tpd->tpdijkstra( xyLoc( rx, ry ), xyLoc( cx, cy ) );
			clock_end     = clock();
			nodesExpanded = tpd->nodesExpanded;
			nodesTouched  = tpd->nodesTouched;
		}

		// write out the statistics
		fprintf( result_file, "(%u,%u) (%u,%u) %7.4f %lu %u %u %s\n",
			rx,ry,cx,cy,result,(clock_end-clock_start)/1000,
			nodesExpanded,nodesTouched,map_file );
		fflush( result_file );

		if( feof( problem_file ) ) {
			// cleanup
			delete tidastar;
			delete ipntt;
			delete astar;
			delete astar_dijkstra;
			delete astar_perfecth;
			delete minclass;
			delete tpd;
			delete env;
			break;
		} else {
			fscanf( problem_file, "(%u,%u) (%u,%u) %s\n", &rx,&ry,&cx,&cy,map_file );
			if( strcmp( map_file, old_map_file) != 0 ) {
				// next map is different, thus delete everything
				delete tidastar;
				delete ipntt;
				delete astar;
				delete astar_dijkstra;
				delete astar_perfecth;
				delete minclass;
				delete tpd;
				delete env;

				// create the new classes
				strcpy( old_map_file, map_file );
				m = new Map( map_file );
				env = new MapEnvironment( m );
				tidastar       = new TIDAStar<xyLoc,tDirection,MapEnvironment>( env, true );
				ipntt          = new IPNTTables<xyLoc,tDirection,MapEnvironment>( env, true );
				astar          = new MinimaxAStar<xyLoc,tDirection,MapEnvironment>( env, 1, true );
				astar_dijkstra = new MinimaxAStar<xyLoc,tDirection,MapEnvironment>( env, 1, true );
				astar_dijkstra->set_useHeuristic( false );
				astar_perfecth = new MinimaxAStar<xyLoc,tDirection,MapEnvironment>( env, 1, true );
				astar_perfecth->set_usePerfectDistanceHeuristic( true );
				minclass       = new MinimaxOptimized<xyLoc,tDirection,MapEnvironment>( env, true );
				tpd            = new TwoPlayerDijkstra<xyLoc,tDirection,MapEnvironment>( env, true );
			}
		}
	}
	fclose( problem_file );
	fclose( result_file );
	fprintf( stdout, "Done.\n" );
}




// test of all-state algorithms
void compute_experiment_allstate( int argc, char* argv[] ) {
	char map_file[100];
	FILE *problem_file, *result_file;
	clock_t clock_start, clock_end;
	clock_start = clock_end = clock();
	Map *m;

	if( argc < 4 ) {
		printf( "Syntax: <problem set file> <algorithm> <result file>\n" );
		printf( "where <algorithm> = dijkstra|markov\n" );
		exit(1);
	}

	problem_file = fopen( argv[1], "r" );
	result_file  = fopen( argv[3], "w" );

	// for all the problems in the problem set file
	while( !feof( problem_file ) ) {
		fscanf( problem_file, "%s\n", map_file );
		m = new Map( map_file );
		Graph *g = GraphSearchConstants::GetGraph( m );
		GraphMapHeuristic *gh = new GraphMapHeuristic( m, g );
		GraphEnvironment *env = new GraphEnvironment( g, gh );
		env->SetDirected( true );

		if( strcmp( argv[2], "dijkstra" ) == 0 ) {
			env->SetDirected( true );

			Dijkstra *d = new Dijkstra( env, 1, true );
			clock_start = clock();
			d->dijkstra();
			clock_end = clock();

			delete d;
		}

		if( strcmp( argv[2], "markov" ) == 0 ) {

			double precision = 0.1;
			double gamma = 1.0;
			double *V = NULL;
			unsigned int iter = 0;

			CopRobberGame *game = new CopRobberGame( env, 1, false, true );

			clock_start = clock();
			game->GetExpectedStateRewards( 0, gamma, 0.01, precision, V, iter );
			clock_end = clock();

			delete[] V;
			delete game;
		}

		delete env;
		delete gh;
		delete g;
		delete m;
		// write out the statistics
		fprintf( result_file, "%lu %s\n",(clock_end-clock_start)/1000,map_file );
		fflush( result_file );
	}
	fclose( result_file );
	fclose( problem_file );
}



// code for testing various sets of graphs
void compute_experiment_graphs( int argc, char* argv[] ) {

	for( int i = 4; i <= 9; i++ ) {
//	char s[10]; sprintf( s, "graph%dc.out", i );
//	char s[10]; sprintf( s, "graph%dcopwin.out", i );
	char s[10]; sprintf( s, "planar_conn.%dcopwin.out", i );

		FILE *fhandler = fopen( s, "r" );
//	FILE *fhandler = fopen( "graph2c.out", "r" );
//	FILE *fcopwin  = fopen( "planar_out", "w" );
	int num_cop_win = 0;
	double maximum_st = 0.;
	double maximum_ratio = 1.;

	while( !feof( fhandler ) ) {
		// read the next graph from the file
		Graph *g = readGraph( fhandler );
		if( g == NULL ) break;
		bool is_cop_win = true;
		std::vector<double>::iterator it;

		// determine whether this graph is cop win or not
		GraphEnvironment *env = new GraphEnvironment( g, NULL );
		Dijkstra *d = new Dijkstra( env, 1, true );
		d->dijkstra();
		TwoPlayerDijkstra<graphState,graphMove,GraphEnvironment> *tpdijkstra = new TwoPlayerDijkstra<graphState,graphMove,GraphEnvironment>( env, true );

		for( it = d->min_cost.begin(); it != d->min_cost.end(); it++ ) {
			if( *it == DBL_MAX ) {
				is_cop_win = false;
				break;
			}
		}

		if( is_cop_win ) {
//			writeGraph( fcopwin, g );
			num_cop_win++;

			// determine maximum search time
			// determine whether the optimal moves for the robber have a move in max_p_r min_p_c V(pr,pc)
			Dijkstra::CRState pos;
			pos.push_back( 0 ); pos.push_back( 0 );
			double maximum_ratio_on_graph = 1.;
			int maximum_ratio_on_graph_c1 = 0;
			int maximum_ratio_on_graph_r = 0;
			double search_time = DBL_MAX;
			for( int c1 = 0; c1 < g->GetNumNodes(); c1++ ) {
				graphState c1_state = c1;
				pos[1] = c1_state;
				double search_time_max = 0.;
				for( int r = 0; r < g->GetNumNodes(); r++ ) {

					// if we are not in a terminal position
					if( r != c1 ) {
						graphState r_state = r;
						pos[0] = r_state;
						search_time_max = max( search_time_max, d->min_cost[d->crg->GetNumberByState(pos)] );

						double temp2 = tpdijkstra->tpdijkstra( r_state, c1_state );
						double temp = d->min_cost[d->crg->GetNumberByState(pos)]/temp2; ///tpdijkstra->tpdijkstra( r_state, c1_state );
						if( temp > maximum_ratio_on_graph ) {
//							printf( "temp = %f r = %d c1 = %d, tpdijkstra = %f\n", temp, r, c1, temp2 );
							maximum_ratio_on_graph = temp;
							maximum_ratio_on_graph_r  = r;
							maximum_ratio_on_graph_c1 = c1;
						}

						// generate all neighbors
						//std::vector<graphState> neighbors;
						//std::vector<graphState>::iterator it;
						//env->GetSuccessors( r_state, neighbors );
						//neighbors.push_back( r_state );
						//
						//double neighbors_max_value = DBL_MIN, neighbors_max_max_min_value = DBL_MIN;
						//std::vector<double> neighbors_values;
						//std::vector<double> neighbors_max_min_values;
						//double temp;
						//for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
						//	pos[0] = *it;
						//	temp = d->min_cost[d->crg->GetNumberByState(pos)];
						//	neighbors_values.push_back( temp );
						//	neighbors_max_value = max( neighbors_max_value, temp );
						//	temp = tpdijkstra->tpdijkstra( *it, c1_state );
						//	neighbors_max_min_values.push_back( temp );
						//	neighbors_max_max_min_value = max( neighbors_max_max_min_value, temp );
						//}
						//
						//bool there_is_an_optimal_move_that_is_in_max_min = false;
						//for( unsigned int i = 0; i < neighbors.size(); i++ ) {
						//	if( neighbors_values[i] == neighbors_max_value &&
						//	    neighbors_max_min_values[i] == neighbors_max_max_min_value ) {
						//		there_is_an_optimal_move_that_is_in_max_min = true;
						//		break;
						//	}
						//}
						//
						//if( !there_is_an_optimal_move_that_is_in_max_min ) {
						//	// report this graph!!!
						//	writeGraph( stdout, g );
						//	printf( "position: r = %d, c1 = %d\n", r, c1 );
						//}
					}
				}
				search_time = min( search_time, search_time_max );
			}

			maximum_st = max( maximum_st, search_time );
			if( maximum_ratio < maximum_ratio_on_graph ) {
				writeGraph( stdout, g );
				printf( "ratio is %f for (%d,%d)\n", maximum_ratio_on_graph, maximum_ratio_on_graph_r, maximum_ratio_on_graph_c1 );
				maximum_ratio = maximum_ratio_on_graph;
			}
		}

		delete tpdijkstra;
		delete d;
		delete env;
		delete g;
	}
	fclose( fhandler );
//	fclose( fcopwin );
	printf( "Number of cop-win graphs: %d\n", num_cop_win );
	printf( "Maximal time to capture: %f\n", maximum_st );
	printf( "Maximal ratio: %f\n", maximum_ratio );
	}
}




// code for Cops and Robber website interface
void compute_website_interface( int argc, char* argv[] ) {
	if( argc < 7 ) {
		output_syntax();
		exit(1);
	}
	int num_cops;
	if( strcmp( argv[1], "-c" ) == 0 ) {
		num_cops = atoi( argv[3] );
	} else {
		output_syntax();
		exit(1);
	}

	double search_time = DBL_MAX;
	Graph *g = readGraphFromCommandLine( 4, argc, argv );
	
	if( g == NULL ) exit(1);
	std::vector<double>::iterator it;

	// determine whether this graph is cop win or not
	GraphEnvironment *env = new GraphEnvironment( g, NULL );
	Dijkstra *d = new Dijkstra( env, num_cops, true );
	d->dijkstra();

	// determine maximum search time
	Dijkstra::CRState pos, optimal_init_pos;
	pos.assign( num_cops + 1, 0 );
	optimal_init_pos = pos;

	double temp;
	// for all possible cop initial setups
	unsigned int num_player_states[num_cops], player_states[num_cops];
	for( int i = 0; i < num_cops; i++ ) num_player_states[i] = g->GetNumNodes();
	for( unsigned long hash = 0; hash < pow( g->GetNumNodes(), num_cops ); hash++ ) {
		dehash_permutation( hash, num_cops, num_player_states, player_states );
		for( int i = 0; i < num_cops; i++ ) pos[i+1] = player_states[i];

		// for all possible robber initial setups
		double search_time_max = 0.;
		int max_init_pos = 0;
		for( int r = 0; r < g->GetNumNodes(); r++ ) {
			pos[0] = r;
			temp = d->min_cost[d->crg->GetNumberByState(pos)];
			if( search_time_max < temp ) {
				search_time_max = temp;
				max_init_pos = r;
			}

			// output of the current state
			printf( "%d ", r );
			for( int i = 0; i < num_cops; i++ )
				printf( "%u ", player_states[i] );
			if( temp == DBL_MAX )
				printf( "infty\n" );
			else {
				if( temp > 0. ) temp = (temp + 1.)/2.;
				printf( "%0.f\n", temp );
			}
		}

		if( search_time > search_time_max ) {
			search_time = search_time_max;
			optimal_init_pos = pos;
			optimal_init_pos[0] = max_init_pos;
		}
	}

	if( search_time == DBL_MAX )
		printf( "0 infty\n" );
	else {
		// print out the optimal initial position
		for( unsigned int i = 0; i < optimal_init_pos.size(); i++ ) {
			printf( "%lu ", optimal_init_pos[i] );
		}
		printf( "\n" );

		// print out whether the graph is cop win or not and the search time
		if( search_time > 0. ) search_time = (search_time + 1.)/2.;
		printf( "1 %.0f\n", search_time );
	}

}




bool find_algorithm( std::vector<const char*> list, const char* alg ) {
	for( unsigned int i = 0; i < list.size(); i++ ) {
		if( strcmp( list[i], alg ) == 0 ) return true;
	}
	return false;
};




// Code for "Suboptimal Solutions to MTS"
void compute_experiment_suboptimal( int argc, char* argv[] ) {
	// CONFIG PARAMETER
	unsigned int cop_speed = 2;

	// variables
	FILE *fhandler = NULL, *foutput = NULL;
	char map_file[20];
	bool compute_optimal_solution = false;

	// parameter input
	if( argc < 4 ) {
		printf( "syntax: -r <robber_algorithms> -c <cop_algorithms> -p <problem_file> -o <output_file>\n" );
		printf( "<robber_algorithms>:\n" );
		printf( "  optimal         - optimal robber (the entire state space is solved first)\n" );
		printf( "  cover           - redefined version of cover\n" );
		printf( "  minimax <depth> - minimax up to given depth with distance metric evaluation\n" );
		printf( "  dam <depth>     - dynamic abstract minimax to a given depth\n" );
		printf( "  greedy          - heuristic hill climbing due to distance metric\n" );
		printf( "  randombeacons <num_beacons> <k_min> <k_max>\n" );
		printf( "                  - randombeacons(k) with k=k_min,...,k_max\n" );
		printf( "  trailmax <k_min> <k_max>\n" );
		printf( "                  - TrailMax(k) with k=k_min,...,k_max\n" );
		printf( "  datrailmax <k_min> <k_max> <l>\n" );
		printf( "                  - Dynamic Abstract TrailMax with k=k_min,...,k_max and given l\n" );
		printf( "<cop_algorithms>:\n" );
		printf( "  optimal - optimal cop (entire state space is solved)\n" );
		printf( "  pra     - cop follows half the PRA* path before recomputing\n" );
		exit(1);
	}
	// command line parsing
	int param_num = 2;
	int problem_file_argv_num = 2;
	std::vector<const char*> robber_algorithms;
	std::vector<std::vector<int> > robber_int_params;
	std::vector<std::vector<double> > robber_double_params;
	std::vector<const char*> cop_algorithms;
	while( param_num < argc ) {
		if( strcmp( argv[param_num], "-r" ) == 0 ) {
			param_num++;
			// parse for all robber algorithms
			while( param_num < argc && strstr( argv[param_num], "-" ) == NULL ) {
				if(      strcmp( argv[param_num], "optimal" ) == 0 ) {
					robber_algorithms.push_back( "optimal" );
					robber_int_params.push_back( std::vector<int>() );
					robber_double_params.push_back( std::vector<double>() );
					compute_optimal_solution = true;
				}
				else if( strcmp( argv[param_num], "cover" ) == 0 ) {
					robber_algorithms.push_back( "cover" );
					robber_int_params.push_back( std::vector<int>() );
					robber_double_params.push_back( std::vector<double>() );
				}
				else if( strcmp( argv[param_num], "minimax" ) == 0 ) {
					robber_algorithms.push_back( "minimax" );
					robber_int_params.push_back( std::vector<int>() );
					robber_double_params.push_back( std::vector<double>( 1, atof( argv[param_num+1] ) ) );
					param_num++;
				}
				else if( strcmp( argv[param_num], "dam" ) == 0 ) {
					robber_algorithms.push_back( "dam" );
					robber_int_params.push_back( std::vector<int>() );
					robber_double_params.push_back( std::vector<double>( 1, atof( argv[param_num+1] ) ) );
					param_num++;
				}
				else if( strcmp( argv[param_num], "greedy" ) == 0 ) {
					robber_algorithms.push_back( "greedy" );
					robber_int_params.push_back( std::vector<int>() );
					robber_double_params.push_back( std::vector<double>() );
				}
				else if( strcmp( argv[param_num], "randombeacons" ) == 0 ) {
					robber_algorithms.push_back( "randombeacons" );
					std::vector<int> myparams;
					myparams.push_back( atoi( argv[param_num+1] ) );
					myparams.push_back( atoi( argv[param_num+2] ) );
					myparams.push_back( atoi( argv[param_num+3] ) );
					robber_int_params.push_back( myparams );
					robber_double_params.push_back( std::vector<double>() );
					param_num += 3;
				}
				else if( strcmp( argv[param_num], "trailmax" ) == 0 ) {
					robber_algorithms.push_back( "trailmax" );
					std::vector<int> myparams;
					myparams.push_back( atoi( argv[param_num+1] ) );
					myparams.push_back( atoi( argv[param_num+2] ) );
					robber_int_params.push_back( myparams );
					robber_double_params.push_back( std::vector<double>() );
					param_num += 2;
				}
				else if( strcmp( argv[param_num], "datrailmax" ) == 0 ) {
					robber_algorithms.push_back( "datrailmax" );
					std::vector<int> myparams;
					myparams.push_back( atoi( argv[param_num+1] ) );
					myparams.push_back( atoi( argv[param_num+2] ) );
					myparams.push_back( atoi( argv[param_num+3] ) );
					robber_int_params.push_back( myparams );
					robber_double_params.push_back( std::vector<double>() );
					param_num += 3;
				}
				else fprintf( stderr, "Warning: unsupported robber algorithm %s\n", argv[param_num] );

				param_num++;
			}
			if( robber_algorithms.size() == 0 ) {
				fprintf( stderr, "ERROR: no robber algorithms submitted\n" );
				exit(1);
			}
		}
		else if( strcmp( argv[param_num], "-c" ) == 0 ) {
			param_num++;
			while( param_num < argc && strstr( argv[param_num], "-" ) == NULL ) {
				if(      strcmp( argv[param_num], "optimal" ) == 0 ) {
					cop_algorithms.push_back( "optimal" );
					compute_optimal_solution = true;
				}
				else if( strcmp( argv[param_num], "pra"     ) == 0 ) cop_algorithms.push_back( "pra" );
				else fprintf( stderr, "Warning: unsupported cop algorithm %s\n", argv[param_num] );
				param_num++;
			}
			if( cop_algorithms.size() == 0 ) {
				fprintf( stderr, "ERROR: no cop algorithms submitted\n" );
				exit(1);
			}
		}
		else if( strcmp( argv[param_num], "-p" ) == 0 ) {
			param_num++;
			fhandler = fopen( argv[param_num], "r" );
			if( fhandler == NULL ) {
				fprintf( stderr, "ERROR: could not open problem file\n" );
				exit(1);
			}
			problem_file_argv_num = param_num;
			param_num++;
		}
		else if( strcmp( argv[param_num], "-o" ) == 0 ) {
			param_num++;
			foutput = fopen( argv[param_num], "w" );
			if( foutput == NULL ) {
				fprintf( stderr, "ERROR: could not open output file\n" );
				exit(1);
			}
			param_num++;
		}
		else {
			fprintf( stderr, "Warning: could not parse parameter: %s\n", argv[param_num] );
			param_num++;
		}
	}



	// syntax explanation output into output file
	fprintf( foutput, "problem file: %s\n", argv[problem_file_argv_num] );
	fprintf( foutput, "cop algorithms: " );
	for( unsigned int i = 0; i < cop_algorithms.size(); i++ )
		fprintf( foutput, "%s ", cop_algorithms[i] );
	fprintf( foutput, "\nrobber algorithms:\n" );
	for( unsigned int i = 0; i < robber_algorithms.size(); i++ ) {
		fprintf( foutput, "  %s( ", robber_algorithms[i] );
		for( unsigned int j = 0; j < robber_int_params[i].size(); j++ )
			fprintf( foutput, "(int)%d ", robber_int_params[i][j] );
		for( unsigned int j = 0; j < robber_double_params[i].size(); j++ )
			fprintf( foutput, "(double)%g ", robber_double_params[i][j] );
		fprintf( foutput, ")\n" );
	}
	fprintf( foutput, "cop speed: %u\n", cop_speed );
	fprintf( foutput, "columns:\n  1: robber_pos\n  2: cop_pos\n" );
	param_num = 3;
	if( compute_optimal_solution ) {
		fprintf( foutput, "  %d: optimal value of game\n", param_num );
		param_num++;
	}
	for( unsigned int i = 0; i < cop_algorithms.size(); i++ ) {
		for( unsigned int j = 0; j < robber_algorithms.size(); j++ ) {

			if( strcmp( cop_algorithms[i], "optimal" ) == 0 && strcmp( robber_algorithms[j], "optimal" ) == 0 ) continue;

			int new_param_num = param_num + 6;
			if( strcmp( robber_algorithms[j], "randombeacons" ) == 0 )
				new_param_num = param_num + 6 * ( robber_int_params[j][2] - robber_int_params[j][1] + 1 );
			if( strcmp( robber_algorithms[j], "trailmax" ) == 0 )
				new_param_num = param_num + 6 * ( robber_int_params[j][1] - robber_int_params[j][0] + 1 );
			if( strcmp( robber_algorithms[j], "datrailmax" ) == 0 )
				new_param_num = param_num + 6 * ( robber_int_params[j][1] - robber_int_params[j][0] + 1 );
			if( strcmp( robber_algorithms[j], "optimal" ) == 0 )
				new_param_num = param_num + 1;

			fprintf( foutput, "  %d-%d: %s vs %s <format>\n", param_num, new_param_num-1, robber_algorithms[j], cop_algorithms[i] );
			param_num = new_param_num;
		}
	}
	fprintf( foutput, "where format is <sol num avg div nE nT>\n" );
	fprintf( foutput, "      sol = solution length against optimal cop\n" );
	fprintf( foutput, "      num = number of computations needed\n" );
	fprintf( foutput, "      avg = average computation time in ms for each such computation\n" );
	fprintf( foutput, "      div = standard diviation in ms for all computations\n" );
	fprintf( foutput, "      nE  = average number of nodes expanded in every computation\n" );
	fprintf( foutput, "      nT  = average number of nodes touched in every computation\n" );
	fprintf( foutput, "--------------------------------------------------------------------------------\n\n" );


	// begin of computation code
	while( !feof( fhandler ) ) {

		// read first map of the problem file
		fscanf( fhandler, "%s\n", map_file );
		fprintf( foutput, "%s\n", map_file );
		Map *m = new Map( map_file );
		fprintf( stdout, "map file: %s\n", m->getMapName() );

		MapCliqueAbstraction *mclab = new MapCliqueAbstraction( m );
		Graph *g = mclab->GetAbstractGraph( 0 );
		MaximumNormAbstractGraphMapHeuristic *gh = new MaximumNormAbstractGraphMapHeuristic( g, m );
		GraphEnvironment *env = new GraphEnvironment( g, gh );

		// cop objects
		DSDijkstra_MemOptim *dsdijkstra = NULL;
		DSPRAStarCop *pracop = NULL;

		// Solve the entire state space if needed
		if( compute_optimal_solution ) {
			dsdijkstra = new DSDijkstra_MemOptim( env, cop_speed );
			dsdijkstra->dsdijkstra();
			fprintf( stdout, "dijkstra done.\n" ); fflush( stdout );
		}
		// PRA* cop
		if( find_algorithm( cop_algorithms, "pra" ) ) pracop = new DSPRAStarCop( mclab, cop_speed );


		// robber objects
		DSCover<graphState,graphMove> *dscover = NULL;
		DSMinimax<graphState,graphMove> *dsminimax = NULL;
		DSDAM *dsdam = NULL;
		DSHeuristicGreedy<graphState,graphMove> *dsheuristic = NULL;
		DSRandomBeacons *dsrandomb = NULL;
		DSTPDijkstra<graphState,graphMove> *dstp = NULL; // trailmax
		DSDATPDijkstra *dsdatp = NULL; // datrailmax

		// initialize robber objects
		if( find_algorithm( robber_algorithms, "cover"      ) ) dscover     = new DSCover<graphState,graphMove>( env, cop_speed );
		if( find_algorithm( robber_algorithms, "minimax"    ) ) dsminimax   = new DSMinimax<graphState,graphMove>( env, true, cop_speed );
		if( find_algorithm( robber_algorithms, "dam"        ) ) dsdam       = new DSDAM( mclab, true, cop_speed, true );
		if( find_algorithm( robber_algorithms, "greedy"     ) ) dsheuristic = new DSHeuristicGreedy<graphState,graphMove>( env, true, cop_speed );
		if( find_algorithm( robber_algorithms, "trailmax"   ) ) dstp        = new DSTPDijkstra<graphState,graphMove>( env, cop_speed );
		if( find_algorithm( robber_algorithms, "datrailmax" ) ) dsdatp      = new DSDATPDijkstra( mclab, cop_speed, true );
		if( find_algorithm( robber_algorithms, "randombeacons" ) ) dsrandomb = new DSRandomBeacons( mclab, true, cop_speed );


		// there is always 1000 problems for a map -> see problem set generation
		for( int i = 0; i < 1000; i++ ) {
			unsigned int rx, ry, cx, cy;
			fscanf( fhandler, "(%d,%d) (%d,%d)\n", &rx, &ry, &cx, &cy );
			fprintf( foutput, "(%u,%u) (%u,%u)", rx, ry, cx, cy );

			// build the actual position data structure
			std::vector<graphState> pos;
			pos.push_back( m->getNodeNum( rx, ry ) );
			pos.push_back( m->getNodeNum( cx, cy ) );

			// now find out the optimal value
			if( compute_optimal_solution )
				fprintf( foutput, " %g", dsdijkstra->Value( pos, true ) );

			// for each cop algorithm
			for( unsigned int cop_alg = 0; cop_alg < cop_algorithms.size(); cop_alg++ ) {
				// for each robber algorithm
				for( unsigned int robber_alg = 0; robber_alg < robber_algorithms.size(); robber_alg++ ) {

					// we do not play optimal against optimal (that doesn't make sense) ;-)
					if( strcmp( cop_algorithms[cop_alg],       "optimal" ) == 0 &&
					    strcmp( robber_algorithms[robber_alg], "optimal" ) == 0 )
						continue;

					// determine the parameter range
					int min_param_range = 0, max_param_range = 0;
					if( strcmp( robber_algorithms[robber_alg], "randombeacons" ) == 0 ) {
						min_param_range = robber_int_params[robber_alg][1];
						max_param_range = robber_int_params[robber_alg][2];
					}
					if( strcmp( robber_algorithms[robber_alg], "trailmax" ) == 0 ) {
						min_param_range = robber_int_params[robber_alg][0];
						max_param_range = robber_int_params[robber_alg][1];
					}
					if( strcmp( robber_algorithms[robber_alg], "datrailmax" ) == 0 ) {
						min_param_range = robber_int_params[robber_alg][0];
						max_param_range = robber_int_params[robber_alg][1];
					}

					for( int current_param = min_param_range; current_param <= max_param_range; current_param++ ) {

						// reset the initial position
						pos[0] = m->getNodeNum( rx, ry );
						pos[1] = m->getNodeNum( cx, cy );
						// variables to keep track of statistics
						double value = 0.;
						unsigned long calculations = 0, timer_average = 0, timer_stddiviation = 0;
						clock_t clock_start, clock_end;
						clock_start = clock_end = clock();
						unsigned int nodesExpanded = 0, nodesTouched = 0;
						std::vector<graphState> mygraphstatepath;
						std::vector<node*> mynodepath;
						unsigned int counter = 0;
						while( true ) {

							// cop's move
							if( strcmp( cop_algorithms[cop_alg], "optimal" ) == 0 ) pos[1] = dsdijkstra->MakeMove( pos, true );
							if( strcmp( cop_algorithms[cop_alg], "pra"     ) == 0 ) pos[1] = pracop->MakeMove( pos[0], pos[1] );
							value += 1.;
							if( pos[0] == pos[1] ) break;

							// robber's move
							if( strcmp( robber_algorithms[robber_alg], "optimal" ) == 0 ) {
								pos[0] = dsdijkstra->MakeMove( pos, false );
							} else {
								// if the current robber strategy is not "optimal"
								// cover
								if( strcmp( robber_algorithms[robber_alg], "cover" ) == 0 ) {
									clock_start = clock();
									pos[0] = dscover->MakeMove( pos[0], pos[1], false, g->GetNumNodes() );
									clock_end   = clock();
									nodesExpanded += dscover->nodesExpanded;
									nodesTouched += dscover->nodesTouched;
									timer_average      += (clock_end-clock_start)/1000;
									timer_stddiviation += (clock_end-clock_start)/1000 * (clock_end-clock_start)/1000;
									calculations++;
								}
								// minimax
								if( strcmp( robber_algorithms[robber_alg], "minimax" ) == 0 ) {
									clock_start = clock();
									pos[0] = dsminimax->MakeMove( pos[0], pos[1], false, robber_double_params[robber_alg][0] );
									clock_end   = clock();
									nodesExpanded += dsminimax->nodesExpanded;
									nodesTouched  += dsminimax->nodesTouched;
									timer_average      += (clock_end-clock_start)/1000;
									timer_stddiviation += (clock_end-clock_start)/1000 * (clock_end-clock_start)/1000;
									calculations++;
								}
								// dam
								if( strcmp( robber_algorithms[robber_alg], "dam" ) == 0 ) {
									node *r = g->GetNode( pos[0] );
									node *c = g->GetNode( pos[1] );
									clock_start = clock();
									r = dsdam->MakeMove( r, c, false, robber_double_params[robber_alg][0] );
									clock_end   = clock();
									pos[0] = r->GetNum();
									nodesExpanded += dsdam->nodesExpanded;
									nodesTouched  += dsdam->nodesTouched;
									timer_average      += (clock_end-clock_start)/1000;
									timer_stddiviation += (clock_end-clock_start)/1000 * (clock_end-clock_start)/1000;
									calculations++;
								}
								// greedy
								if( strcmp( robber_algorithms[robber_alg], "greedy" ) == 0 ) {
									clock_start = clock();
									pos[0] = dsheuristic->MakeMove( pos[0], pos[1], false );
									clock_end   = clock();
									nodesExpanded += dsheuristic->nodesExpanded;
									nodesTouched  += dsheuristic->nodesTouched;
									timer_average      += (clock_end-clock_start)/1000;
									timer_stddiviation += (clock_end-clock_start)/1000 * (clock_end-clock_start)/1000;
									calculations++;
								}
								// Random Beacons
								if( strcmp( robber_algorithms[robber_alg], "randombeacons" ) == 0 ) {
									// if new path has to be computed, do so
									if( counter == (unsigned int)current_param || counter >= mygraphstatepath.size() ) {
										clock_start = clock();
										dsrandomb->GetPath( pos[0], pos[1], robber_int_params[robber_alg][0], mygraphstatepath );
										clock_end   = clock();
										timer_average      += (clock_end-clock_start)/1000;
										timer_stddiviation += (clock_end-clock_start)/1000 * (clock_end-clock_start)/1000;
										nodesExpanded += dsrandomb->nodesExpanded;
										nodesTouched  += dsrandomb->nodesTouched;
										calculations++;
										// reset counter
										counter = 0;
									}
									pos[0] = mygraphstatepath[counter];
									counter++;
								}
								// TrailMax
								if( strcmp( robber_algorithms[robber_alg], "trailmax" ) == 0 ) {
									if( counter == (unsigned int)current_param || counter >= mygraphstatepath.size() ) {
										clock_start = clock();
										dstp->dstpdijkstra( pos[0], pos[1], false, mygraphstatepath );
										clock_end   = clock();
										nodesExpanded += dstp->nodesExpanded;
										nodesTouched  += dstp->nodesTouched;
										timer_average      += (clock_end-clock_start)/1000;
										timer_stddiviation += (clock_end-clock_start)/1000 * (clock_end-clock_start)/1000;
										calculations++;
										// reset counter
										counter = 0;
									}
									pos[0] = mygraphstatepath[counter];
									counter++;
								}
								// DATrailMax
								if( strcmp( robber_algorithms[robber_alg], "datrailmax" ) == 0 ) {
									node *r = g->GetNode( pos[0] );
									node *c = g->GetNode( pos[1] );
									if( counter == (unsigned int)current_param || counter >= mynodepath.size() ) {
										clock_start = clock();
										dsdatp->datpdijkstra( r, c, mynodepath, false, robber_int_params[robber_alg][2] );
										clock_end   = clock();
										timer_average      += (clock_end-clock_start)/1000;
										timer_stddiviation += (clock_end-clock_start)/1000 * (clock_end-clock_start)/1000;
										nodesExpanded += dsdatp->myNodesExpanded;
										nodesTouched  += dsdatp->myNodesTouched;
										calculations++;
										// reset counter
										counter = 0;
									}
									pos[0] = mynodepath[counter]->GetNum();
									counter++;
								}



								value += 1.;
								if( pos[0] == pos[1] ) break;
							}
						}
						
						if( strcmp( robber_algorithms[robber_alg], "optimal" ) == 0 ) {
							fprintf( foutput, " %g", value );
						} else {
							double expected_value = (double)timer_average/(double)calculations;
							double std_diviation  = sqrt( (double)timer_stddiviation/(double)calculations
							                        - expected_value*expected_value );
							double expected_nodesExpanded = (double)nodesExpanded/(double)calculations;
							double expected_nodesTouched  = (double)nodesTouched /(double)calculations;
							fprintf( foutput, " %g %lu %g %g %g %g", value, calculations, expected_value, std_diviation, expected_nodesExpanded, expected_nodesTouched );
							fflush( foutput );
						}
					} // parameter range
				} // robber algorithms
			} // cop algorithms

			
			// finish the line, go to next problem instance
			fprintf( foutput, "\n" );

		}

		// delete robber objects
		if( dsrandomb   != NULL ) delete dsrandomb;
		if( dsdatp      != NULL ) delete dsdatp;
		if( dstp        != NULL ) delete dstp;
		if( dsheuristic != NULL ) delete dsheuristic;
		if( dsdam       != NULL ) delete dsdam;
		if( dsminimax   != NULL ) delete dsminimax;
		if( dscover     != NULL ) delete dscover;
		// delete cop objects
		if( pracop     != NULL ) delete pracop;
		if( dsdijkstra != NULL ) delete dsdijkstra;
		// delete environments
		delete env;
		delete gh;
		delete mclab;
	}

	fclose( fhandler );
	fclose( foutput );
}


/*------------------------------------------------------------------------------
| Other stuff
------------------------------------------------------------------------------*/

void CreateSimulation( int id ) {
//	Map *map = new Map( "../../maps/local/test_coprobber_1.map" );
/*
	if( gDefaultMap[0] == 0 ) {
		map = new Map( mazeSize, mazeSize );
		MakeMaze( map, 1 );
	} else
		map = new Map( gDefaultMap );

	MapCliqueAbstraction *mca = new MapCliqueAbstraction( map );
	AbsMapEnvironment *absenv = new AbsMapEnvironment( mca );


	// the Robber
	MultilevelCopRobberGame* mcrgame = new MultilevelCopRobberGame( mca, 1, true, true );
	RobberUnit *ru = new RobberUnit( xyLoc(5,10), mcrgame, "data/markov_level0_map1.stat" );
	ru->SetSpeed( 1. );
	
	unitSims.resize( id + 1 );
	unitSims[id] = new CRSimulation<xyLoc, tDirection, AbsMapEnvironment>( absenv, ru, 1., true );

	// we want to play on a graph with homogen edge costs, and we do this by simply
	// letting every unit move after the same time
	unitSims[id]->SetStepType( kUniTime );

	// and here come the cops
	MySearchUnit *c1 = new MySearchUnit( 1, 10, ru->GetNum(), new praStar() );
	c1->SetSpeed( 1. );
	unitSims[id]->AddUnit( c1, 1. );
//	MySearchUnit *c2 = new MySearchUnit( 1, 8, ru->GetNum(), new praStar() );
//	c2->SetSpeed( 1. );
//	unitSims[id]->AddUnit( c2, 0.5 );

	std::vector<unsigned int> copids;
	copids.push_back( c1->GetNum() );
//	copids.push_back( c2->GetNum() );
	ru->SetCopUnits( copids );

	sleep( 5 );
*/


	FILE *fhandler = fopen( "graph", "r" );
	Graph *g = readGraph( fhandler, true );
	printf( "num nodes: %d\n", g->GetNumNodes() );
	printf( "num edges: %d\n", g->GetNumEdges() );
	LoadedCliqueAbstraction *cliqueabs = new LoadedCliqueAbstraction( g );
	AbstractionGraphEnvironment *env = new AbstractionGraphEnvironment( cliqueabs, 0, NULL );

	simulation = new DSCRSimulation<graphState,graphMove,AbstractionGraphEnvironment>( env, true, 2, false );
	simulation->SetPaused( true );
	TrailMaxUnit<graphState,graphMove,AbstractionGraphEnvironment> *runit =
		new TrailMaxUnit<graphState,graphMove,AbstractionGraphEnvironment>( 21796, 1, 2 );
	runit->SetColor( 1., 0., 0. );
	unsigned int robberunitnumber = simulation->AddRobber( runit, 1. );
	PraStarGraphUnit<graphState,graphMove,AbstractionGraphEnvironment> *cunit =
		new PraStarGraphUnit<graphState,graphMove,AbstractionGraphEnvironment>( cliqueabs, 21989, 2 );
	cunit->SetColor( 0., 0., 1. );
	unsigned int copunitnumber = simulation->AddCop( cunit, 1. );

	runit->SetCopUnit( copunitnumber );
	cunit->SetRobberUnit( robberunitnumber );

	// add the other cops to the simulation
	cunit = new PraStarGraphUnit<graphState,graphMove,AbstractionGraphEnvironment>( cliqueabs, 16179, 2 );
	cunit->SetColor( 0., 0., 1. );
	simulation->AddCop( cunit, 1. );
	cunit = new PraStarGraphUnit<graphState,graphMove,AbstractionGraphEnvironment>( cliqueabs, 1913, 2 );
	cunit->SetColor( 0., 0., 1. );
	simulation->AddCop( cunit, 1. );
	cunit = new PraStarGraphUnit<graphState,graphMove,AbstractionGraphEnvironment>( cliqueabs, 139, 2 );
	cunit->SetColor( 0., 0., 1. );
	simulation->AddCop( cunit, 1. );
	cunit = new PraStarGraphUnit<graphState,graphMove,AbstractionGraphEnvironment>( cliqueabs, 1896, 2 );
	cunit->SetColor( 0., 0., 1. );
	simulation->AddCop( cunit, 1. );
	cunit = new PraStarGraphUnit<graphState,graphMove,AbstractionGraphEnvironment>( cliqueabs, 2706, 2 );
	cunit->SetColor( 0., 0., 1. );
	simulation->AddCop( cunit, 1. );
	cunit = new PraStarGraphUnit<graphState,graphMove,AbstractionGraphEnvironment>( cliqueabs, 10415, 2 );
	cunit->SetColor( 0., 0., 1. );
	simulation->AddCop( cunit, 1. );
	cunit = new PraStarGraphUnit<graphState,graphMove,AbstractionGraphEnvironment>( cliqueabs, 26047, 2 );
	cunit->SetColor( 0., 0., 1. );
	simulation->AddCop( cunit, 1. );
	cunit = new PraStarGraphUnit<graphState,graphMove,AbstractionGraphEnvironment>( cliqueabs, 1104, 2 );
	cunit->SetColor( 0., 0., 1. );
	simulation->AddCop( cunit, 1. );


/*
	Map *map;
	if( gDefaultMap[0] == 0 ) {
		map = new Map( mazeSize, mazeSize );
		MakeMaze( map, 1 );
	} else
		map = new Map( gDefaultMap );
	MapCliqueAbstraction *mca = new MapCliqueAbstraction( map );
	AbstractionGraphEnvironment *env = new AbstractionGraphEnvironment( mca, 0, new MaximumNormAbstractGraphMapHeuristic( mca->GetAbstractGraph(0), map ) );

	// global variable, see above
	simulation = new DSCRSimulation<graphState,graphMove,AbstractionGraphEnvironment>( env, true, 2, false );

	TrailMaxUnit<graphState,graphMove,AbstractionGraphEnvironment> *runit =
		new TrailMaxUnit<graphState,graphMove,AbstractionGraphEnvironment>( mca->GetNodeFromMap(41,59)->GetNum(), 1, 2 );
	runit->SetColor( 1., 0., 0. );
	unsigned int robberunitnumber = simulation->AddRobber( runit, 1. );

	PraStarMapUnit<graphState,graphMove,AbstractionGraphEnvironment> *cunit =
		new PraStarMapUnit<graphState,graphMove,AbstractionGraphEnvironment>( mca, mca->GetNodeFromMap(49,27)->GetNum(), 2 );
	cunit->SetColor( 0., 0., 1. );
	unsigned int copunitnumber = simulation->AddCop( cunit, 1. );

	runit->SetCopUnit( copunitnumber );
	cunit->SetRobberUnit( robberunitnumber );

	sleep( 5 );
*/

	return;
}

void InstallHandlers() {
	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	InstallCommandLineHandler(MyCLHandler, "-size", "-size integer", "If size is set, we create a square maze with the x and y dimensions specified.");
	InstallWindowHandler(MyWindowHandler);
	InstallKeyboardHandler(MyDisplayHandler, "pause", "pause/unpause simulation", kAnyModifier, 'p');
	InstallMouseClickHandler(MyClickHandler);
	return;
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType) {
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		SetNumPorts( windowID, 1 );
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		CreateSimulation(windowID);
		time(&last_simulation_update);
	}
	return;
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *) {
/*
	if ((windowID < unitSims.size()) && (unitSims[windowID] == 0))
		return;
	if( viewport == 0 ) {
		sleep( 1 );
		unitSims[windowID]->OpenGLDraw(windowID);
		unitSims[windowID]->StepTime( 1. );
//		unitSims[windowID]->StepTime( unitSims[windowID]->GetTimeToNextStep() );
		printf( "simulation time is: %g\n", unitSims[windowID]->GetSimulationTime() );
		printf( "----------\n" );
	}
*/
/*
	graphenv->OpenGLDraw( windowID );
	graphState s = 32511;
	graphenv->OpenGLDraw( windowID, s );
*/
	if( viewport == 0 ) {
		simulation->OpenGLDraw( windowID );
		time_t temptime; time(&temptime);
		if( last_simulation_update < temptime ) {
			if( !simulation->Done() && !simulation->GetPaused() ) {
				simulation->StepTime( 1. );
				printf( "simulation time is: %g\n", simulation->GetSimulationTime() );
				printf( "---------------\n" );
			}
			time(&last_simulation_update);
		}
	}
	return;
}

int MyCLHandler(char *argument[], int maxNumArgs) {
	if( strcmp( argument[0], "-map" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		strncpy(gDefaultMap, argument[1], 1024);
		return 2;
	}
	else if( strcmp( argument[0], "-size" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		mazeSize = atoi(argument[1]);
		assert( mazeSize > 0 );
		printf("mazeSize = %d\n", mazeSize);
		return 2;
	}
	return 2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key) {
	switch( key ) {
		case 'p': // pause/unpause simulation
			simulation->SetPaused( !simulation->GetPaused() );
			break;
	}
	return;
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType) {
	return false;
}



void parseCommandLineParameters( int argc, char* argv[], Map* &m, xyLoc &pos_cop, xyLoc &pos_robber, int &max_recursion_level ) {
	int curr = 2;
	int x,y;

	if( argc < 7 ) {
		fprintf( stderr, "Syntax: followme -map <map> -pc <pos_cop> -pr <pos_robber> -rlevel <max_recursion_level>\n" );
		fprintf( stderr, "where <pos_cop>/<pos_robber> is of the form x,y\n" );
		exit(1);
	}

	while( curr < argc ) {
		if( strcmp( argv[curr], "-map" ) == 0 ) {
			m = new Map( argv[curr+1] );
		}
		if( strcmp( argv[curr], "-pc" ) == 0 ) {
			sscanf( argv[curr+1], "%d,%d", &x, &y );
			pos_cop = xyLoc( x, y );
		}
		if( strcmp( argv[curr], "-pr" ) == 0 ) {
			sscanf( argv[curr+1], "%d,%d", &x, &y );
			pos_robber = xyLoc( x, y );
		}
		if( strcmp( argv[curr], "-rlevel" ) == 0 ) {
			sscanf( argv[curr+1], "%d", &max_recursion_level );
		}
			
		curr += 2;
	}
	return;
}

Graph *readGraphFromCommandLine( int offset, int argc, char* argv[] ) {
	if( strcmp( argv[offset], "-g" ) != 0 ) {
		output_syntax();
		return NULL;
	}
	offset++;
	int num_vertices = atoi( argv[offset] ); //, num_edges = atoi( argv[offset+1] );
	offset += 2;

	// create all the vertices
	Graph *g = new Graph();
	for( int i = 0; i < num_vertices; i++ ) {
		char s[30]; sprintf( s, "%d", i );
		node *n = new node( s );
		g->AddNode( n );
	}

	while( offset < argc ) {
		int v1 = atoi( argv[offset] ), v2 = atoi( argv[offset+1] );
		edge *e = new edge( v1, v2, 1. );
		g->AddEdge(e);		
		offset += 2;
	}

	return g;
}

// reads a graph from an open file
Graph* readGraph( FILE *fhandler, bool input_with_vertice_coordinates ) {

	if( feof( fhandler ) )
		return NULL;

	Graph *g = new Graph();
	int num_vertices, num_edges;
	if( !fscanf( fhandler, "%d %d\n", &num_vertices, &num_edges ) ) return NULL;

	for( int i = 0; i < num_vertices; i++ ) {
		char s[30]; sprintf( s, "%d", i );
		node *n = new node( s );
		if( input_with_vertice_coordinates ) {
			double x,y;
			fscanf( fhandler, "(%lf,%lf) ", &x, &y );
			n->SetLabelL( GraphAbstractionConstants::kAbstractionLevel, 0 );
			n->SetLabelL( GraphAbstractionConstants::kNumAbstractedNodes, 1 );
			n->SetLabelL( GraphAbstractionConstants::kParent, -1 );
			n->SetLabelF( GraphAbstractionConstants::kXCoordinate, x );
			n->SetLabelF( GraphAbstractionConstants::kYCoordinate, y );
			n->SetLabelF( GraphAbstractionConstants::kZCoordinate, 0. );
			n->SetLabelL( GraphAbstractionConstants::kNodeBlocked, 0 );
		}
		g->AddNode( n );
	}

	if( input_with_vertice_coordinates ) fscanf( fhandler, "\n" );

	for( int i = 0; i < num_edges; i++ ) {
		int v1, v2;
		fscanf( fhandler, "%d %d", &v1, &v2 );
		if( input_with_vertice_coordinates ) {
			double d1 = g->GetNode(v1)->GetLabelF(GraphAbstractionConstants::kXCoordinate) - g->GetNode(v2)->GetLabelF(GraphAbstractionConstants::kXCoordinate);
			double d2 = g->GetNode(v1)->GetLabelF(GraphAbstractionConstants::kYCoordinate) - g->GetNode(v2)->GetLabelF(GraphAbstractionConstants::kYCoordinate);
			g->AddEdge( new edge( v1, v2, sqrt( d1*d1 + d2*d2 ) ) );
		} else
			g->AddEdge( new edge( v1, v2, 1. ) );
	}
	fscanf( fhandler, "\n" );

	return g;
}

void writeGraph( FILE *fhandler, Graph *g ) {
	fprintf( fhandler, "%d %d\n", g->GetNumNodes(), g->GetNumEdges() );
	edge_iterator eit = g->getEdgeIter();
	edge *e = g->edgeIterNext( eit );

	while( e != NULL ) {
		fprintf( fhandler, "%u %u  ", e->getFrom(), e->getTo() );
		e = g->edgeIterNext( eit );
	}
	fprintf( fhandler, "\n" );	
}
