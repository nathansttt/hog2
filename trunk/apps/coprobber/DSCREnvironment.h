/*------------------------------------------------------------------------------
| Different Speed Cop Robber Environment
|
| Alternative implementation of the cop robber game
| with a possibly faster cop
------------------------------------------------------------------------------*/

#ifndef DSCRENVIRONMENT_H
#define DSCRENVIRONMENT_H

#include <vector>
#include <queue>
#include <ext/hash_set>
#include "MultiAgentEnvironment.h"
#include "MyHash.h"

/*
	A one cop and one robber implementation, alternating action game.
*/
template<class state, class action>
class DSCREnvironment {

	public:

	// types
	typedef typename MultiAgentEnvironment<state,action>::MAState CRState;

	// constructor and destructor
	DSCREnvironment(
		SearchEnvironment<state,action> *env,
		bool playerscanpass = true, unsigned int cop_speed = 1 );
	virtual ~DSCREnvironment();

	// set the playerscanpass variable
	virtual void SetPlayersCanPass( bool _playerscanpass ) { playerscanpass = _playerscanpass; };
	virtual bool GetPlayersCanpass() { return playerscanpass; };

	// speed management
	virtual void SetCopSpeed( unsigned int speed ) { cop_speed = speed; };
	virtual unsigned int GetCopSpeed() { return cop_speed; };

	// move generation
	virtual void GetRobberSuccessors( CRState &s, std::vector<state> &neighbors, bool take_cop_into_account = true );
	virtual void GetRobberSuccessors( state &s, std::vector<state> &neighbors );
	virtual void GetCopSuccessors( CRState &s, std::vector<state> &neighbors );
	virtual void GetCopSuccessors( state &s,   std::vector<state> &neighbors );

	// termination criteria
	virtual bool GoalTest( CRState &s );
	virtual double TerminalCost( CRState &s );

	// transition costs
	virtual double RobberGCost( CRState &s, state &n );
	virtual double RobberGCost( CRState &s, CRState &n ) { return RobberGCost( s, n[0] ); };
	virtual double CopGCost( CRState &s, state &n );
	virtual double CopGCost( CRState &s, CRState &n ) { return CopGCost( s, n[1] ); };

	// heuristc distance between two states in the graph/map
	// assumes, that all transition costs are 1
	// note: if used with graphs, use MyGraphHeuristic within the GraphEnvironment
	// if used with maps, we implemented a version ourselfs
	virtual double HCost( state &s1, state &s2 );


	protected:

	// dijkstra algorithm for successor generation for the cops
	// note that we should actually give back trajectories of the cops and then
	// detect collision in the cops trajectories...
	virtual void Dijkstra( state &s, unsigned int &steps, std::vector<state> &neighbors );

	// variables
	SearchEnvironment<state,action> *env;
	bool playerscanpass;
	unsigned int cop_speed;

	private:
	struct MyStateHash {
		size_t operator() ( const state &s ) const {
			return StateHash<state>( s );
		}
	};

	class QueueEntry {
		public:
		QueueEntry() {};
		QueueEntry( state _pos, unsigned int _distance ):pos(_pos),distance(_distance) {};
		state pos;
		unsigned int distance;
	};

	struct QueueEntryCompare {
		bool operator() ( const QueueEntry &q1, const QueueEntry &q2 ) const {
			return( q1.distance > q2.distance );
		}
	};

	typedef std::priority_queue< QueueEntry, std::vector<QueueEntry>, QueueEntryCompare > OpenList;
	typedef __gnu_cxx::hash_set< state, MyStateHash > ClosedList;
	OpenList openlist;
	ClosedList closedlist;

};

/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/
template<class state, class action>
DSCREnvironment<state,action>::DSCREnvironment( SearchEnvironment<state,action> *_env, bool _playerscanpass, unsigned int _cop_speed ):
	env(_env),playerscanpass(_playerscanpass),cop_speed(_cop_speed)
{
};

template<class state, class action>
DSCREnvironment<state,action>::~DSCREnvironment() {};

template<class state, class action>
void DSCREnvironment<state,action>::GetRobberSuccessors( CRState &s, std::vector<state> &neighbors, bool take_cop_into_account ) {
	neighbors.clear();
	if( take_cop_into_account ) {
		if( s[0] == s[1] ) return; // if the cop is on the robbers position, the robber cannot move
	}
	// otherwise generate all successors for s
	GetRobberSuccessors( s[0], neighbors );
	return;
};

template<class state, class action>
void DSCREnvironment<state,action>::GetRobberSuccessors( state &s, std::vector<state> &neighbors ) {
	neighbors.clear();
	env->GetSuccessors( s, neighbors );
	if( playerscanpass ) neighbors.push_back( s );
	return;
};

template<class state, class action>
void DSCREnvironment<state,action>::GetCopSuccessors( CRState &s, std::vector<state> &neighbors ) {
	GetCopSuccessors( s[1], neighbors );
	return;
};

template<class state, class action>
void DSCREnvironment<state,action>::GetCopSuccessors( state &s, std::vector<state> &neighbors ) {
	Dijkstra( s, cop_speed, neighbors );
	return;
};

template<class state, class action>
bool DSCREnvironment<state,action>::GoalTest( CRState &s ) {
	return( s[0] == s[1] );
};

template<class state, class action>
double DSCREnvironment<state,action>::TerminalCost( CRState& ) {
	return 0.;
};

template<class state, class action>
double DSCREnvironment<state,action>::RobberGCost( CRState&, state& ) {
	return 1.;
};

template<class state, class action>
double DSCREnvironment<state,action>::CopGCost( CRState&, state& ) {
	return 1.;
};

// \see DSCREnvironment.h
template<>
double DSCREnvironment<xyLoc,tDirection>::HCost( xyLoc &s1, xyLoc &s2 );
template<>
double DSCREnvironment<graphState,graphMove>::HCost( graphState &s1, graphState &s2 );

template<class state, class action>
void DSCREnvironment<state,action>::Dijkstra( state &s, unsigned int &steps, std::vector<state> &neighbors ) {
	neighbors.clear();
	openlist.push( QueueEntry( s, 0 ) );

	std::vector<state> tempneighbors;
	typename std::vector<state>::iterator it;
	QueueEntry qe;
	while( !openlist.empty() ) {

		qe = openlist.top();
		openlist.pop();

		if( closedlist.find( qe.pos ) == closedlist.end() ) {
			if( playerscanpass || !(qe.pos == s) )
				neighbors.push_back( qe.pos );
			closedlist.insert( qe.pos );

			// if there can be neighbors from this node
			if( qe.distance < steps ) {
				env->GetSuccessors( qe.pos, tempneighbors );
				for( it = tempneighbors.begin(); it != tempneighbors.end(); it++ ) {
					if( closedlist.find( *it ) == closedlist.end() ) {
						// if not yet in list push it on the open queue
						openlist.push( QueueEntry( *it, qe.distance + 1 ) );
					}
				}
				tempneighbors.clear();
			}
		}
	}

	// cleanup
	// openlist = OpenList(); // shouldn't be neccessary
	closedlist.clear();

	return;
}

#endif
