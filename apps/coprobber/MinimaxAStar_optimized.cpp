#include "MinimaxAStar_optimized.h"


template<>
void MinimaxAStar<graphState,graphMove,GraphEnvironment>::push_end_states_on_queue( CRState &goal_pos, bool &goal_minFirst ) {
	QueueEntry qe;
	CRState pos;
	std::vector<graphState> neighbors;
	node_iterator nit;
	Graph *g = env->GetGraph();
	node *n;
	unsigned int i;

	// sanity check: is the queue empty or are there rests?
	assert( queue.empty() );

	// loop through the graph
	nit = g->getNodeIter();
	n = g->nodeIterNext( nit );
	while( n != NULL ) {

		nodesExpanded++;nodesTouched++;

		pos.clear();
		for( i = 0; i < 2; i++ ) {
			pos.push_back( n->GetNum() );
		}

		min_cost[pos] = 0.;
		max_cost[pos] = 0.;

		// now push all the states on the queue that are one step away
		// from the goal
		//
		// if cop moves last
		neighbors.clear();
		env->GetSuccessors( pos[1], neighbors );
		qe.pos = pos;
		qe.minFirst = true;
		for( std::vector<graphState>::iterator it = neighbors.begin(); it != neighbors.end(); it++ ) {
				nodesTouched++;

				qe.pos[1] = *it;
				qe.gvalue   = 1.; //MinGCost( qe.pos, pos );
				qe.fvalue   = qe.gvalue + HCost( goal_pos, goal_minFirst, qe.pos, qe.minFirst );
				queue.push( qe );
		}

		n = g->nodeIterNext( nit );
	}
	
};



template<>
void MinimaxAStar<xyLoc,tDirection,MapEnvironment>::push_end_states_on_queue( CRState &goal_pos, bool &goal_minFirst ) {
	QueueEntry qe;
	CRState pos;
	std::vector<xyLoc> neighbors;
	Map *m = env->GetMap();

	// sanity check: is the queue empty or are there rests?
	assert( queue.empty() );

	for( long width = 0; width < m->getMapWidth(); width++ ) {
		for( long height = 0; height < m->getMapHeight(); height++ ) {

			if( m->getTerrainType( width, height ) != kGround ) continue;

			nodesExpanded++;nodesTouched++;

			pos.clear(); pos.push_back( xyLoc( width, height ) ); pos.push_back( xyLoc( width, height ) );

			min_cost[pos] = 0.;
			max_cost[pos] = 0.;

			// now push all the states on the queue that are one step away
			// from the goal
			//
			// if cop moves last
			neighbors.clear();
			env->GetSuccessors( pos[1], neighbors );
			qe.pos = pos;
			qe.minFirst = true;
			for( std::vector<xyLoc>::iterator it = neighbors.begin(); it != neighbors.end(); it++ ) {
					nodesTouched++;

					qe.pos[1] = *it;
					qe.gvalue   = 1.; //MinGCost( qe.pos, pos );
					qe.fvalue   = qe.gvalue + HCost( goal_pos, goal_minFirst, qe.pos, qe.minFirst );
					queue.push( qe );
			}

		}
	}
	
};


