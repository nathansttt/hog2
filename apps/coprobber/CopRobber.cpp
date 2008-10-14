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
#include "MyGraphMapHeuristic.h"
//#include "MinimaxAStar.h"
#include "MinimaxAStar_optimized.h"


std::vector<CRAbsMapSimulation *> unitSims;
int mazeSize = 10;

/*------------------------------------------------------------------------------
| Main
------------------------------------------------------------------------------*/
int main(int argc, char* argv[])
{
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
	// Dijkstra for the entire state space
	xyLoc pos_cop, pos_robber;
	Map *m;
	int max_depth;

	parseCommandLineParameters( argc, argv, m, pos_cop, pos_robber, max_depth );
	printf( "map: %s\n", m->getMapName() );
	printf( "cop position: %d,%d\n", pos_cop.x, pos_cop.y );
	printf( "robber position: %d,%d\n", pos_robber.x, pos_robber.y );

	Graph *g = GraphSearchConstants::GetGraph( m );
	GraphMapHeuristic *gh = new GraphMapHeuristic( m, g );
	GraphEnvironment *env = new GraphEnvironment( g, gh );
	env->SetDirected( true ); // because the GetGraph routine
	// adds edges for both directions

	Dijkstra *d = new Dijkstra( env, 2, true );
	d->dijkstra();
	d->WriteValuesToDisk( "dijkstra.dat" );

	delete d;
	delete env;
	delete gh;
	delete g;
	delete m;
*/


/*
// Minimax A*
	xyLoc pos_cop, pos_robber;
	Map *m;
	int max_depth;

	parseCommandLineParameters( argc, argv, m, pos_cop, pos_robber, max_depth );
	printf( "map: %s\n", m->getMapName() );
	printf( "cop position: %d,%d\n", pos_cop.x, pos_cop.y );
	printf( "robber position: %d,%d\n", pos_robber.x, pos_robber.y );

	Graph *g = GraphSearchConstants::GetGraph( m );
	MyGraphMapHeuristic *gh = new MyGraphMapHeuristic( m, g );
	GraphEnvironment *env = new GraphEnvironment( g, gh );
	env->SetDirected( true ); // because the GetGraph routine
	// adds edges for both directions

	MinimaxAStar *astar = new MinimaxAStar( env, 1, true );
	MinimaxAStar::CRState s;
	s.push_back( m->getNodeNum( pos_robber.x, pos_robber.y ) );
	s.push_back( m->getNodeNum( pos_cop.x, pos_cop.y ) );
	fprintf( stdout, "%u %u\n", m->getNodeNum( pos_robber.x, pos_robber.y ), m->getNodeNum( pos_cop.x, pos_cop.y ) );

	double result = astar->astar( s, true, 1. );
	fprintf( stdout, "result: %f\n", result );

	delete astar;
	delete env;
	delete gh;
	delete g;
	delete m;
*/


/*
	// NORMAL MINIMAX
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
*/

/*
	// optimized Minimax
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
*/


/*
	// TIDA*
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



/*
	// IPN-Search
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
*/


/*
	// IPN-Search with transposition tables
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
*/


/*
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
*/

/*
// problem set generation
	char map_file[20];
	char problem_file[20] = "problem_set6.dat";
	Map *m;
	time_t t; time( &t ); srandom( (unsigned int) t );
	unsigned int num;
	int i,j;
	// problem set 1: i = 6:20, scaled to size 20
	// problem set 2: i = 16:35, scaled to size 40
	// problem set 3: i = 20:39, scaled to size 60
	// problem set 4: i = 30:49, scaled to size 80
	// problem set 1
	FILE *fhandler = fopen( problem_file, "w" );
	FILE *file_with_maps = fopen( argv[1], "r" );
	while( !feof( file_with_maps ) ) {
//	for( i = 1; i <= 15; i++ ) {
//		m = new Map( i, i );
//		MakeMaze( m );
//		m->scale( 15,15 );
//		sprintf( map_file, "problem_set5_map%d.map", i );
//		m->save( map_file );
		fscanf( file_with_maps, "%s\n", map_file );
		m = new Map( map_file );
		MapEnvironment *env = new MapEnvironment( m );
		Graph *g = GraphSearchConstants::GetGraph( m );
		for( j = 0; j < 50;  ) {
			// generate random position for the robber
			num = (unsigned int)floor(
				(double)random()/(double)RAND_MAX * (double)g->GetNumNodes());
			unsigned int rx = g->GetNode(num)->GetLabelL(GraphSearchConstants::kMapX);
			unsigned int ry = g->GetNode(num)->GetLabelL(GraphSearchConstants::kMapY);
			// generate random position for the cop
			num = (unsigned int)floor(
				(double)random()/(double)RAND_MAX * (double)g->GetNumNodes());
			unsigned int cx = g->GetNode(num)->GetLabelL(GraphSearchConstants::kMapX);
			unsigned int cy = g->GetNode(num)->GetLabelL(GraphSearchConstants::kMapY);

			TIDAStar<xyLoc,tDirection,MapEnvironment> *tidastar =
				new TIDAStar<xyLoc,tDirection,MapEnvironment>( env, true );
			std::vector<xyLoc> pos;
			pos.push_back( xyLoc(rx,ry) ); pos.push_back( xyLoc(cx,cy) );
			double result = tidastar->tida( pos );
			delete tidastar;

			if( result <= 35. ) {
				fprintf( stdout, "%f\n", result );
				// print out the starting position for the robber
				fprintf( fhandler, "(%u,%u) ", rx, ry );
				// print out the starting position for the cop
				fprintf( fhandler, "(%u,%u) ", cx, cy );
				// print out the map that we are in
				fprintf( fhandler, "%s\n", map_file );
				j++;
			}
		}
	
		delete env;
		//delete m;
	}
	fclose( fhandler );
	fclose( file_with_maps );
*/

/*
	// TESTs for "Optimal solutions for Moving Target Search"
	char map_file[100];
	FILE *problem_file, *result_file, *tida_file_handler = NULL;
	clock_t clock_start, clock_end;
	clock_start = clock_end = clock();
	unsigned int rx,ry,cx,cy;
	double result = 0;
	unsigned int nodesExpanded = 0, nodesTouched = 0;
	Map *m;

	if( argc < 4 ) {
		printf( "Syntax: <problem set file> <algorithm> <result file>\n" );
		printf( "where <algorithm> = tida|rma|ipn|minimax\n" );
		exit(1);
	}

	problem_file = fopen( argv[1], "r" );
	result_file  = fopen( argv[3], "w" );

	// in case we want to compute with minimax
	// we have to get the correct values from a file (hello to TIDA*)
	if( strcmp( argv[2], "minimax" ) == 0 ) {
			char s[40];
			sprintf( s, "tida_%s", argv[1] );
			tida_file_handler = fopen( s, "r" );
			if( tida_file_handler == NULL ) {
				fprintf( stderr, "ERROR: could not find tida value file\n" );
				exit(1);
			}
	}

	// for all the problems in the problem set file
	while( !feof( problem_file ) ) {
		fscanf( problem_file, "(%u,%u) (%u,%u) %s\n", &rx,&ry,&cx,&cy,map_file );
		m = new Map( map_file );
		MapEnvironment *env = new MapEnvironment( m );

		if( strcmp( argv[2], "tida" ) == 0 ) {
			// if we want to test TIDA*
			TIDAStar<xyLoc,tDirection,MapEnvironment> *tidastar =
				new TIDAStar<xyLoc,tDirection,MapEnvironment>( env, true );

			std::vector<xyLoc> pos;
			pos.push_back( xyLoc(rx,ry) ); pos.push_back( xyLoc(cx,cy) );

			clock_start   = clock();
			result        = tidastar->tida( pos );
			clock_end     = clock();
			nodesExpanded = tidastar->nodesExpanded;
			nodesTouched  = tidastar->nodesTouched;

			delete tidastar;
		}

		if( strcmp( argv[2], "ipn" ) == 0 ) {
			IPNTTables<xyLoc,tDirection,MapEnvironment> *ipntt =
				new IPNTTables<xyLoc,tDirection,MapEnvironment>( env, true );

			std::vector<xyLoc> pos;
			pos.push_back( xyLoc(rx,ry) ); pos.push_back( xyLoc(cx,cy) );

			clock_start   = clock();
			result        = ipntt->ipn( pos, true );
			clock_end     = clock();
			nodesExpanded = ipntt->nodesExpanded;
			nodesTouched  = ipntt->nodesTouched;

			delete ipntt;
		}

		if( strcmp( argv[2], "rma" ) == 0 ) {
			MinimaxAStar<xyLoc,tDirection,MapEnvironment> *astar =
				new MinimaxAStar<xyLoc,tDirection,MapEnvironment>( env, 1, true );
			std::vector<xyLoc> s;
			s.push_back( xyLoc( rx, ry ) );
			s.push_back( xyLoc( cx, cy ) );

			clock_start   = clock();
			result        = astar->astar( s, true );
			clock_end     = clock();
			nodesExpanded = astar->nodesExpanded;
			nodesTouched  = astar->nodesTouched;

			delete astar;
		}

		if( strcmp( argv[2], "minimax" ) == 0 ) {

			double tida_value;
			fscanf( tida_file_handler, "(%*u,%*u) (%*u,%*u) %lf %*u %*u %*u %*s\n",
				&tida_value );

			MinimaxOptimized<xyLoc,tDirection,MapEnvironment> *minclass =
				new MinimaxOptimized<xyLoc,tDirection,MapEnvironment>( env, true );
			std::vector<xyLoc> pos;
			pos.push_back( xyLoc(rx,ry) ); pos.push_back( xyLoc(cx,cy) );

			clock_start   = clock();
			result        = minclass->minimax( pos, true, (int)floor(tida_value) );
			clock_end     = clock();
			nodesExpanded = minclass->nodesExpanded;
			nodesTouched  = minclass->nodesTouched;

			delete minclass;
		}

		// cleanup
		delete env;

		// write out the statistics
		fprintf( result_file, "(%u,%u) (%u,%u) %7.f %lu %u %u %s\n",
			rx,ry,cx,cy,result,(clock_end-clock_start)/1000,
			nodesExpanded,nodesTouched,map_file );
		fflush( result_file );
		//delete m;
	}
	fclose( problem_file );
	fclose( result_file );
	fprintf( stdout, "Done.\n" );
*/

	// test of all-state algorithms
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

	return 0;
}


void CreateSimulation( int id ) {
	Map *map = new Map( "../../maps/local/test_coprobber_1.map" );
/*
	if( gDefaultMap[0] == 0 ) {
		map = new Map( mazeSize, mazeSize );
		MakeMaze( map, 1 );
	} else
		map = new Map( gDefaultMap );
*/

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

	return;
}

void InstallHandlers() {
	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	InstallCommandLineHandler(MyCLHandler, "-size", "-size integer", "If size is set, we create a square maze with the x and y dimensions specified.");
	InstallWindowHandler(MyWindowHandler);
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
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		CreateSimulation(windowID);
	}
	return;
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *) {
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

}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType) {
	return false;
}



void parseCommandLineParameters( int argc, char* argv[], Map* &m, xyLoc &pos_cop, xyLoc &pos_robber, int &max_recursion_level ) {
	int curr = 1;
	int x,y;

	if( argc < 6 ) {
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
