#include <vector>
#include "SearchEnvironment.h"
#include "MultiAgentEnvironment.h"

#ifndef TIDASTAR_H
#define TIDASTAR_H

/*!
	two players IDA* implementation
	min player (cop) plays first by default (parameter minFirst in tida)
	pos is a tuple of locations of robber (pos[0]) and cop (pos[1])
*/
template<class state, class action, class environment>
class TIDAStar {

	public:

	typedef typename MultiAgentEnvironment<state,action>::MAState CRState;

	TIDAStar( environment *_env, bool _canPause ):
		maxDepthReached(0),
		env(_env), canPause(_canPause) {};

	double tida( CRState &pos, unsigned int maxDepth, std::vector<CRState> &path, double weight = 1., bool minFirst = true );

	// when TIDA* is run it updates this value
	unsigned int maxDepthReached;

	protected:

	double tida_update( CRState &pos, double gCost, double bound, unsigned int maxDepth, double weight, bool minFirst, std::vector<CRState> &path );

	double MinHCost( CRState &pos, bool minsTurn = true );
	double MinGCost( CRState &pos1, CRState &pos2 );
	bool GoalTest( CRState &pos );
	double TerminalCost( CRState &pos );

	environment *env;
	bool canPause;
	unsigned int nodesExpanded, nodesTouched;

};




/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/

// public functions

template<class state, class action, class environment>
double TIDAStar<state,action,environment>::tida( CRState &pos, unsigned int maxDepth, std::vector<CRState> &path, double weight, bool minFirst ) {
	double b, c = weight * MinHCost( pos, minFirst );

	maxDepthReached = maxDepth;

	do {
		b = c;
//		fprintf( stdout, "set bound to b = %f\n", b );
		c = tida_update( pos, 0., b, maxDepth, weight, minFirst, path );
		// verbose
//		fprintf( stdout, "solution: " );
//		std::reverse( path.begin(), path.end() );
//		for( unsigned int i = 0; i < path.size(); i++ ) {
//			fprintf( stdout, "(%u,%u)(%u,%u) => ", path[i][0].x, path[i][0].y, path[i][1].x, path[i][1].y );
//		}
//		fprintf( stdout, "\n" );
	} while( c > b ); // until c <= b

	// turn the path around to have the path from the root to the leafs
	std::reverse( path.begin(), path.end() );

	maxDepthReached = maxDepth - maxDepthReached;

	return c;
}


template<class state, class action, class environment>
double TIDAStar<state,action,environment>::tida_update( CRState &pos, double gCost, double bound, unsigned int maxDepth, double weight, bool minFirst, std::vector<CRState> &path )
{
	// keep track of the minimal depth parameter that we encounter
	if( maxDepth < maxDepthReached ) maxDepthReached = maxDepth;
	path.clear();

	// verbose output
//	fprintf( stdout, "Considering position (%u,%u) (%u,%u) %d\n", pos[0].x, pos[0].y, pos[1].x, pos[1].y, minFirst );

	if( GoalTest( pos ) ) {
		path.push_back( pos ); // keep track of our terminal position
		return TerminalCost( pos );
	}
	if( bound < weight * MinHCost( pos, minFirst ) + gCost ) {
		return (weight * MinHCost( pos, minFirst ));
	}
//	if( maxDepth <= 0 ) {
//		fprintf( stdout, "=> %f\n", bound-gCost );
//		return bound - gCost;
//	}

	double result, temp, c;
	std::vector<state> neighbors;
	CRState neighbor;
	std::vector<CRState> childpath;

	// get the successor states
	int myid = minFirst?1:0; // am I cop or robber (robber/max is at 0, cop/min at 1)
	env->GetSuccessors( pos[myid], neighbors );
	if( canPause ) neighbors.push_back( pos[myid] );

	// in case we are the cop/min player
	if( minFirst ) {
		result = DBL_MAX;

		for( unsigned int i = 0; i < neighbors.size(); i++ ) {
			neighbor = pos;
			neighbor[myid] = neighbors[i];
			c = MinGCost( pos, neighbor );
			temp = c + tida_update( neighbor, gCost + c, bound, maxDepth - 1, weight, !minFirst, childpath );

			// result = min( temp, result )
			if( temp == result && path.empty() && !childpath.empty() ) {
				path = childpath; path.push_back( pos );
			}
			if( temp < result ) {
				result = temp;
				if( childpath.empty() )
					path.clear();
				else {
					path = childpath; path.push_back( pos );
				}
			}

			// alpha pruning
			// in case you do not care about the solution path but only the
			// solution length, you can prune with "<=", otherwise "<"
//			if( result + gCost <= bound )
			if( result + gCost < bound )
				break;
		}
	// in case we are the robber/max player
	} else {
		result = DBL_MIN;

		for( unsigned int i = 0; i < neighbors.size(); i++ ) {
			neighbor = pos;
			neighbor[myid] = neighbors[i];
			c = MinGCost( pos, neighbor );
			temp = c + tida_update( neighbor, gCost + c, bound, maxDepth - 1, weight, !minFirst, childpath );

			// result = max( temp, result )
			if( temp == result && path.empty() && !childpath.empty() ) {
				path = childpath; path.push_back( pos );
			}
			if( temp > result ) {
				result = temp;
				if( childpath.empty() )
					path.clear();
				else {
					path = childpath; path.push_back( pos );
				}
			}

			// beta pruning
			if( result + gCost > bound ) {
				// if you do not want to have any pseudo solutions before
				// the last iteration uncomment this to really prune non
				// solution paths
//				path.clear();
				break;
			}
		}
	}

	return result;
}



// protected functions


template<class state, class action, class environment>
double TIDAStar<state,action,environment>::MinHCost( CRState &pos, bool minsTurn ) {
	if( canPause )
		return ( 2. * env->HCost( pos[1], pos[0] ) - (minsTurn?MinGCost(pos,pos):0.) );
	else
		// distance from cop to the robber
		return env->HCost( pos[1], pos[0] );
}

// specification for state=xyLoc
template<>
double TIDAStar<xyLoc,tDirection,MapEnvironment>::MinHCost( CRState &pos, bool minsTurn ) {

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
double TIDAStar<state,action,environment>::MinGCost( CRState&, CRState& ) {
	return 1.;
}

template<class state, class action, class environment>
bool TIDAStar<state,action,environment>::GoalTest( CRState &pos ) {
	return( pos[0] == pos[1] );
}

template<class state, class action, class environment>
double TIDAStar<state,action,environment>::TerminalCost( CRState& ) {
	return 0.;
}

#endif
