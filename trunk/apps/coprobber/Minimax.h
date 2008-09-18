#include <ext/hash_set>
#include <map>
#include <vector>
#include <math.h>
#include "Map2DEnvironment.h"
#include "MultiAgentEnvironment.h"


#ifndef MINIMAX_H
#define MINIMAX_H

// hash function definition
template<class state>
uint64_t CRHash( const std::vector<state> &pos );

/*!
	two players minimax implementation
*/
template<class state, class action, class environment>
class Minimax {

	public:

	// type definitions
	typedef typename MultiAgentEnvironment<state,action>::MAState CRState;

	// SearchCache for the path from the root to the current node
	struct CRStateHash {
		size_t operator()( CRState *s ) const {
			return CRHash<state>( *s );
		}
	};
	struct CRStateEqual {
		bool operator()( const CRState *c1, const CRState *c2 ) const {
			return ( *c1 == *c2 );
		}
	};
	typedef __gnu_cxx::hash_set<CRState*, CRStateHash, CRStateEqual> SearchCache;

	// transposition tables
	class TPEntry {
		public:
		TPEntry( CRState _pos, double _value = 0. ): pos(_pos), value(_value) {};
		TPEntry( CRState _pos, std::vector<CRState> _path, double _value = 0. ): pos(_pos),path(_path),value(_value) {};
		CRState pos; // the node that has to be stored
		std::vector<CRState> path; // path from the leafs to this node
		double value;
	};
	struct TPEntryHash {
		size_t operator()( const TPEntry &e ) const {
			return CRHash<state>( e.pos );
		}
	};
	struct TPEntryEqual {
		bool operator()( const TPEntry &e1, const TPEntry &e2 ) const {
			return( e1.pos == e2.pos );
		}
	};

	typedef __gnu_cxx::hash_set<TPEntry, TPEntryHash, TPEntryEqual> TranspositionTable;
	typedef std::map<double, TranspositionTable> GTTables;
	typedef std::pair<double, TranspositionTable> GTTables_Pair;



	// constructor
	Minimax( environment *_env, bool _canPause ):
		max_depth_reached(0), env(_env), canPause(_canPause)
		{};

	// functions
//	double iterative_minimax( CRState pos, std::vector<CRState> &path, bool minFirst = true );

	double minimax( CRState pos, std::vector<CRState> &path, bool minFirst = true, int max_depth = 100 );

	// gets updated after each call to minimax (or minimax_update)
	int max_depth_reached;

	protected:

	double minimax_help( CRState pos, std::vector<CRState> &path, bool minFirst, int depth, double alpha, double beta );

	// evaluation function for the leafs
	double EvalFunc( CRState &pos, bool minsTurn = true );
	// consistent heuristic function that provides a lower bound on the solution
	double MinHCost( CRState &pos, bool minsTurn = true );
	double MinGCost( CRState &pos1, CRState &pos2 );
	bool GoalTest( CRState &pos );
	double TerminalCost( CRState &pos );


	// variables
	environment *env;
	bool canPause;
	// caching the path from the root to the current node (during the computation)
	SearchCache min_scache, max_scache;

	// transposition table
	GTTables min_gttables, max_gttables;
};






/*------------------------------------------------------------------------------
| MiniMax Implementation
------------------------------------------------------------------------------*/
/*
template<class state, class action, class environment>
double Minimax<state,action,environment>::iterative_minimax( CRState pos, std::vector<CRState> &path, bool minFirst ) {

	double result = 0.;
	int bound = 0;
	while( true ) {
		result = minimax_update( pos, path, minFirst, bound );
		if( result < (double)bound ) break;
		bound++;
	}
	return result;
}
*/


template<class state,class action,class environment>
double Minimax<state,action,environment>::minimax( CRState pos, std::vector<CRState> &path, bool minFirst, int max_depth ) {

	double result;

	// during the run of minimax_help, the minimal depth is stored in max_depth_reached
	// and there we substract it from max_depth later to get the actual depth we reached
	max_depth_reached = max_depth;

	// just to make sure
	min_scache.clear();
	max_scache.clear();

	// just to make sure
	min_gttables.clear();
	max_gttables.clear();

	result = minimax_help( pos, path, minFirst, max_depth, DBL_MIN, DBL_MAX );

	// the temporary search cache should be empty
	assert( min_scache.empty() );
	assert( max_scache.empty() );

	// cleanup
	min_gttables.clear();
	max_gttables.clear();

	max_depth_reached = max_depth - max_depth_reached;

	// reverse the path
	std::reverse( path.begin(), path.end() );

	return result;
}


// alpha = minimum score that the maximum player is assured of
// beta  = maximum score that the minimum player is assured of
template<class state,class action,class environment>
double Minimax<state,action,environment>::minimax_help( CRState pos, std::vector<CRState> &path, bool minFirst, int depth, double alpha, double beta ) {

//	fprintf( stdout, "considered position (%u,%u) (%u,%u) for player ", pos[0].x, pos[0].y, pos[1].x, pos[1].y );
//	fprintf( stdout, "%s\n", (minFirst?"min":"max") );

	// store the minimal depth reached
	if( depth < max_depth_reached )
		max_depth_reached = depth;

	// clear the path
	path.clear();

	// if we reached a leaf node
	if( GoalTest( pos ) ) {
		path.push_back( pos );
		return TerminalCost( pos );
	}

	// determine whether we yet encountered this position on the
	// path from the root to this node
	// technically we need this to make the game tree finite
	if( minFirst ) {
		if( min_scache.find( &pos ) != min_scache.end() )
			return DBL_MAX;
	} else {
		if( max_scache.find( &pos ) != max_scache.end() )
			return DBL_MAX;
	}

	// in case of a consistent heuristic we can also prune
	// because we are then guaranteed not to reach the terminals anymore
	if( beta <= MinHCost( pos, minFirst ) )
		return DBL_MAX;

	// if we reached the bottom of the computation tree
	if( depth <= 0 ) {
		path.push_back( pos );
		return MinHCost( pos, minFirst );
//		return EvalFunc( pos, minFirst );
	}

	// transposition table lookup
	TPEntry mytpentry( pos );
	typename GTTables::iterator gttit;
	typename TranspositionTable::iterator tit;
	if( minFirst ) {
		gttit = min_gttables.find( depth );
		if( gttit != min_gttables.end() ) {
			// if a transposition table for this gCost was found
			tit = gttit->second.find( mytpentry );
			if( tit != gttit->second.end() ) {
				path = tit->path;
				return tit->value;
			}
		} else {
			// else create a new transposition table for this gCost
			GTTables_Pair mypair;
			mypair.first = depth;
			min_gttables.insert( mypair );
		}
	} else {
		gttit = max_gttables.find( depth );
		if( gttit != max_gttables.end() ) {
			tit = gttit->second.find( mytpentry );
			if( tit != gttit->second.end() ) {
				path = tit->path;
				return tit->value;
			}
		} else {
			GTTables_Pair mypair;
			mypair.first = depth;
			max_gttables.insert( mypair );
		}
	}


	// push the current node on the path cache
	if( minFirst )
		min_scache.insert( &pos );
	else
		max_scache.insert( &pos );

	// variable declarations
	std::vector<state> next_mystates;
	CRState child;
	double child_value, pathcost;
	std::vector<CRState> mypath;

	// generate the next moves/children
	int myid = minFirst?1:0;
	// compute recursively for all the children
	env->GetSuccessors( pos[myid], next_mystates );
	if( canPause ) next_mystates.push_back( pos[myid] );

	// for all possible next moves
	while( !next_mystates.empty() ) {

		child = pos;
		child[myid] = next_mystates.back();
		next_mystates.pop_back();

		pathcost    = MinGCost( pos, child );
		child_value = pathcost + minimax_help( child, mypath, !minFirst, depth - 1, alpha - pathcost, beta - pathcost );

		if( minFirst ) {
			// min player updates his score
			if( child_value < beta ) {
				beta = child_value;
				path = mypath;
				path.push_back( pos );
			}
		} else {
			// max player updates his score
			if( child_value > alpha ) {
				alpha = child_value;
				path = mypath;
				path.push_back( pos );
			}
		}

		// alpha-beta pruning
		if( beta <= alpha ) {
			break;
		}

	}

	// cleanup
	// update the list of moves from the root to this nodes parent
	if( minFirst )
		min_scache.erase( &pos );
	else
		max_scache.erase( &pos );

	// store the current value
	mytpentry.path = path;
	if( minFirst ) {
		mytpentry.value = beta;
		min_gttables.find( depth )->second.insert( mytpentry );
	} else {
		mytpentry.value = alpha;
		max_gttables.find( depth )->second.insert( mytpentry );
	}

	return ( (minFirst)?beta:alpha );
}


template<class state, class action, class environment>
double Minimax<state,action,environment>::EvalFunc( CRState &pos, bool minsTurn ) {
	return 0.;
//	return MinHCost( pos, minsTurn );
}

template<class state, class action, class environment>
double Minimax<state,action,environment>::MinHCost( CRState &pos, bool minsTurn ) {
	if( canPause )
		return ( 2. * env->HCost( pos[1], pos[0] ) - (minsTurn?MinGCost(pos,pos):0.) );
	else
		// distance from cop to the robber
		return env->HCost( pos[1], pos[0] );
}

// specification for state=xyLoc
template<>
double Minimax<xyLoc,tDirection,MapEnvironment>::MinHCost( CRState &pos, bool minsTurn ) {
	double dist;
	if( abs(pos[1].x - pos[0].x) < abs(pos[1].y - pos[0].y) )
		dist = abs(pos[1].y - pos[0].y);
	else
		dist = abs(pos[1].x - pos[0].x);

	if( canPause )
		return( 2. * dist - (minsTurn?MinGCost(pos,pos):0.) );
	else
		return dist;
}


template<class state, class action, class environment>
double Minimax<state,action,environment>::MinGCost( CRState&, CRState& ) {
	return 1.;
}

template<class state, class action, class environment>
bool Minimax<state,action,environment>::GoalTest( CRState &pos ) {
	return( pos[0] == pos[1] );
}

template<class state, class action, class environment>
double Minimax<state,action,environment>::TerminalCost( CRState& ) {
	return 0.;
}

/* doesn't work because we do not have an object env
template<class state>
uint64_t CRHash<state>( CRState &pos ) {
	return (env->GetStateHash( pos[0] )<<32 | (uint32_t)env->GetStateHash( pos[1] ) );
}
*/

// specification for state=xyLoc
template<>
uint64_t CRHash<xyLoc>( const std::vector<xyLoc> &pos ) {
	return( ((uint64_t)pos[0].x)<<48 | (((uint64_t)pos[0].y)<<48)>>16 |
		(((uint64_t)pos[1].x)<<48)>>32 | (((uint64_t)pos[1].y)<<48)>>48 );
}


#endif
