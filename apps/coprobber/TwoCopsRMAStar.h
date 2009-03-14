#include <vector>
#include <set>
#include <queue>
#include <ext/hash_map>
#include "GraphEnvironment.h"

#ifndef TWOCOPSRMASTAR_H
#define TWOCOPSRMASTAR_H

/*
	RMA* Implementation for one robber and two cops

	note: this implementation is designed for two cops only
	  and does a few optimizations to compute much faster!

	note: agents can always pass their turns

	note: This implementation only works with graphs instead of any
		kind of environments. This is due to the hash functions.
*/
class TwoCopsRMAStar {

	public:

	// constructor
	TwoCopsRMAStar( GraphEnvironment *env );
	//~TwoCopsDijkstra();

	unsigned int rmastar( graphState &r, graphState &c1, graphState &c2, bool minFirst );

	void set_useHeuristic( bool _useHeuristic ) { useHeuristic = _useHeuristic; };
	void set_usePerfectDistanceHeuristic( bool usePerfectDistanceHeuristic );


	unsigned int nodesExpanded, nodesTouched;

	protected:
	
	// for internal use within hashing and move generation functions
	typedef uint32_t Position;
	typedef graphState CRState[3];

	// variables
	GraphEnvironment *env;
	unsigned int numnodes;

	CRState current_goal;
	bool current_goal_minFirst;

	// copied from TwoCopsDijkstra
	Position CRHash_MemOptim( CRState &s );
	void MemOptim_Hash_To_CRState( Position &hash, CRState &s );

	// neighbor generation for the cops or the robber (depending on minFirst)
	// we're giving back a set here because we want uniqueness of moves
	void GetNeighbors( Position &pos, bool minFirst, std::set<Position> &neighbors );

	// functions for dijkstra computation
	unsigned int compute_target_value( Position &s );
	void push_end_states_on_queue();

	unsigned int HCost( Position &pos, bool &minFirst );

	// queue
	class QueueEntry {
		public:
		QueueEntry() {};
		QueueEntry( Position _pos, bool mf, unsigned int gv, unsigned int fv ):
			pos( _pos ), minFirst( mf ), gvalue( gv ), fvalue( fv ) {};

		Position pos;
		bool minFirst;
		unsigned int gvalue, fvalue;
	};
	struct QueueEntryCompare {
		bool operator() ( const QueueEntry &q1, const QueueEntry &q2 ) const {
			return( q1.fvalue > q2.fvalue );
		}
	};
	typedef std::priority_queue<QueueEntry, std::deque<QueueEntry>, QueueEntryCompare> MyPriorityQueue;
	MyPriorityQueue queue;

	// closed lists
	typedef __gnu_cxx::hash_map<Position,unsigned int> MyClosedList;
	MyClosedList min_cost, max_cost;

	void clear_cache();


	bool useHeuristic;
	bool usePerfectDistanceHeuristic;
	std::vector<std::vector<double> > distance_heuristic;

};


#endif
