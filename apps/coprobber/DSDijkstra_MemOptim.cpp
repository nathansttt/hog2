#define __STDC_LIMIT_MACROS
#include <stdint.h>
#include "DSDijkstra_MemOptim.h"

/*------------------------------------------------------------------------------
| Hashing
------------------------------------------------------------------------------*/
// squeze the entire joint state into a 32bit integer
uint32_t DSDijkstra_MemOptim::CRHash_MemOptim( CRState &s ) {
	return( s[0] * numnodes + s[1] );
//	return( ( (uint32_t)s[0] )<<16 | (((uint32_t)s[1])<<16)>>16 );
};

// get the joint state back from the hash
void DSDijkstra_MemOptim::MemOptim_Hash_To_CRState( uint32_t &hash, DSDijkstra_MemOptim::CRState &s ) {
	s.push_back( (graphState) (hash/numnodes) );
	s.push_back( (graphState) (hash%numnodes) );
//	s.push_back( (graphState) (hash>>16) );
//	s.push_back( (graphState) ( (hash<<16)>>16 ) );
	return;
};


/*------------------------------------------------------------------------------
| DSDijkstra Memory Optimized - Implementation
------------------------------------------------------------------------------*/
DSDijkstra_MemOptim::DSDijkstra_MemOptim( GraphEnvironment *_env, unsigned int cop_speed ):
	dscrenv( new DSCREnvironment<graphState,graphMove>( _env, true, cop_speed ) ),
	env(_env)
{
	numnodes = env->GetGraph()->GetNumNodes();
	// this is just to keep errors in computation out
	if( numnodes*numnodes > UINT32_MAX ) {
		fprintf( stderr, "ERROR: More nodes in graph than can be encoded in hash\n" );
		exit(1);
	}
};

DSDijkstra_MemOptim::~DSDijkstra_MemOptim() {
	delete dscrenv;
};


float DSDijkstra_MemOptim::compute_target_value( CRState &s ) {
	float result = 0.;

	CRState temp = s;
	float tempvalue;
	std::vector<graphState> myneighbors;
	dscrenv->GetRobberSuccessors( temp, myneighbors );

	// now, for all successor states
	for( std::vector<graphState>::iterator it = myneighbors.begin();
	     it != myneighbors.end(); it++ ) {
	
		// build the state
		temp[0] = *it;

		tempvalue = dscrenv->RobberGCost( s, temp ) + min_cost[ CRHash_MemOptim( temp ) ];
		if( tempvalue > result ) result = tempvalue;

		if( result == FLT_MAX ) return FLT_MAX;
	}
	return result;
};


void DSDijkstra_MemOptim::dsdijkstra() {
	QueueEntry qe, qtemp;
	std::vector<graphState>::iterator it;
	CRState pos, postemp;

	// failproof
	assert( queue.empty() );

	// assign space neccessary
	min_cost.assign( numnodes*numnodes, FLT_MAX );
	max_cost.assign( numnodes*numnodes, FLT_MAX );

	push_end_states_on_queue();

	while( !queue.empty() ) {

		// get the element from the queue
		qe = queue.top(); queue.pop();
		pos.clear(); MemOptim_Hash_To_CRState( qe.pos_hash, pos );

		// verbose
		//fprintf( stdout, "minFirst = %d, pos = (%u,%u)(%u,%u), value = %f\n", qe.minFirst, qe.pos[0].x, qe.pos[0].y, qe.pos[1].x, qe.pos[1].y, qe.value );

		if( qe.minFirst ) {

			if( min_cost[qe.pos_hash] == FLT_MAX ) {

				min_cost[qe.pos_hash] = qe.value;

				std::vector<graphState> myneighbors;
				dscrenv->GetRobberSuccessors( pos, myneighbors, false );

				// now, for all successor states
				for( it = myneighbors.begin(); it != myneighbors.end(); it++ ) {
					// build the state
					pos[0] = *it;
					qe.pos_hash = CRHash_MemOptim( pos );

					// check whether its value is not yet set
					if( max_cost[qe.pos_hash] == FLT_MAX ) {
						qe.value = compute_target_value( pos );
						if( qe.value != FLT_MAX ) {
							qe.minFirst = false;
							//printf( "pushed %d (%u,%u)(%u,%u) on queue with %g\n", qe.minFirst,
							//	qe.pos[0].x, qe.pos[0].y, qe.pos[1].x, qe.pos[1].y, qe.value );
							queue.push( qe );
						}
					}
				}
			}

		} else {

			if( max_cost[qe.pos_hash] == FLT_MAX ) {

				max_cost[qe.pos_hash] = qe.value;
				postemp = pos;

				// get neighbors
				std::vector<graphState> myneighbors;
				dscrenv->GetCopSuccessors( postemp, myneighbors );

				for( it = myneighbors.begin(); it != myneighbors.end(); it++ ) {
					
					// build the next state
					postemp[1] = *it;
					qtemp.pos_hash = CRHash_MemOptim( postemp );

					// check again whether already set or not
					if( min_cost[ qtemp.pos_hash ] == FLT_MAX ) {
						qtemp.minFirst = true;
						qtemp.value    = qe.value + dscrenv->CopGCost( postemp, pos );
						//printf( "pushed %d (%u,%u)(%u,%u) on queue with %g\n", qtemp.minFirst,
						//	qtemp.pos[0].x, qtemp.pos[0].y, qtemp.pos[1].x, qtemp.pos[1].y, qtemp.value );
						queue.push( qtemp );
					}
				}
			}
		}

	}

}


float DSDijkstra_MemOptim::Value( CRState &pos, bool minFirst ) {
	if( minFirst ) {
		return min_cost[CRHash_MemOptim( pos )];
	} else {
		return max_cost[CRHash_MemOptim( pos )];
	}
};


graphState DSDijkstra_MemOptim::MakeMove( CRState &pos, bool minFirst ) {

	CRState temppos = pos;
	std::vector<graphState> neighbors;
	float temp;
	graphState result = minFirst?temppos[1]:temppos[0];
	std::vector<graphState>::iterator it;

	// get the available moves
	if( minFirst ) {
		float value = FLT_MAX;
		dscrenv->GetCopSuccessors( temppos, neighbors );

		for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
			temppos[1] = *it;
			temp = max_cost[ CRHash_MemOptim( temppos ) ];
			if( value >= temp ) {
				value = temp;
				result = *it;
			}
		}
	} else {
		float value = FLT_MIN;
		dscrenv->GetRobberSuccessors( temppos, neighbors );

		for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
			temppos[0] = *it;
			temp = min_cost[ CRHash_MemOptim( temppos ) ];
			if( value <= temp ) {
				value = temp;
				result = *it;
			}
		}
	}

	return result;
};


void DSDijkstra_MemOptim::push_end_states_on_queue() {
	QueueEntry qe;
	CRState pos, postemp;
	std::vector<graphState> neighbors;
	node_iterator nit;
	Graph *g = env->GetGraph();
	node *n;

	// sanity check: is the queue empty or are there rests?
	assert( queue.empty() );

	// loop through the graph
	nit = g->getNodeIter();
	n = g->nodeIterNext( nit );
	while( n != NULL ) {

		nodesExpanded++;nodesTouched++;

		pos.clear();
		pos.push_back( n->GetNum() );
		pos.push_back( n->GetNum() );

		min_cost[CRHash_MemOptim( pos )] = 0.;
		max_cost[CRHash_MemOptim( pos )] = 0.;

		// now push all the states on the queue that are one step away
		// from the goal
		//
		// if cop moves last
		neighbors.clear();
		dscrenv->GetCopSuccessors( pos, neighbors );
		postemp = pos;
		qe.minFirst = true;
		for( std::vector<graphState>::iterator it = neighbors.begin(); it != neighbors.end(); it++ ) {
				nodesTouched++;

				postemp[1]  = *it;
				qe.pos_hash = CRHash_MemOptim( postemp );
				qe.value    = dscrenv->CopGCost( postemp, pos );
				queue.push( qe );
		}

		n = g->nodeIterNext( nit );
	}
	
};


void DSDijkstra_MemOptim::WriteValuesToDisk( const char* filename ) {
	FILE *fhandler;

	CRState pos;

	fhandler = fopen( filename, "w" );
	fprintf( fhandler, "states in space: %lu\n", min_cost.size() );
	fprintf( fhandler, "expected rewards if cop moves first:\n" );
	fprintf( fhandler, "\n\n" );
	for( unsigned int i = 0; i < min_cost.size(); i++ ) {
		pos.clear();
		MemOptim_Hash_To_CRState( i, pos );
		fprintf( fhandler, "%lu %lu %g\n", pos[0], pos[1], min_cost[i] );
	}
	fclose( fhandler );
	return;
};
