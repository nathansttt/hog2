#include <vector>
#include <ext/hash_set>
#include <map>
#include "SearchEnvironment.h"
#include "MultiAgentEnvironment.h"
#include "Minimax.h" // for the function CRHash
#include "IPNSearch.h" // for function uintplus


#ifndef IPNTTABLES_H
#define IPNTTABLES_H

/*
	Implementation of proof number search with (consistent) heuristic
	enhancement and iterative increase of the bound to be proofed
	(inspired by TIDA*).
	This implementation differs from IPNSearch.h in so far that it tries
	to incorporate transposition tables
*/
template<class state, class action, class environment>
class IPNTTables {

	public:

	typedef typename MultiAgentEnvironment<state,action>::MAState CRState;

	// transposition tables
	class TPEntry {
		public:
		TPEntry( CRState &_pos ): pos(_pos), iteration_of_last_update(0) {};
		TPEntry( CRState &_pos, unsigned int pn, unsigned int dn, bool mf = true, double v = 0., double b = 0. ):
			pos(_pos), minFirst(mf), proof_number(pn), disproof_number(dn), value(v), bound(b),
			iteration_of_last_update(0) {};

		CRState pos;
		bool minFirst;
		unsigned int proof_number, disproof_number;
		double value, bound;
		std::vector<TPEntry*> childs;
		unsigned int iteration_of_last_update;
	};
	struct TPEntryHash {
		size_t operator()( const TPEntry *e ) const {
			return CRHash<state>( e->pos );
		}
	};
	struct TPEntryEqual {
		bool operator()( const TPEntry *e1, const TPEntry *e2 ) const {
			return( e1->pos == e2->pos );
		}
	};

	typedef __gnu_cxx::hash_set<TPEntry*, TPEntryHash, TPEntryEqual> TranspositionTable;
	typedef std::map<double, TranspositionTable> GTTables;
	typedef std::pair<double, TranspositionTable> GTTables_Pair;



	// constructor
	IPNTTables( environment *_env, bool _canPause ):
		env(_env), canPause(_canPause) {};

	double ipn( CRState &pos, bool minFirst = true );

	unsigned int nodesExpanded, nodesTouched, nodesUpdated;
	std::vector<unsigned int> iteration_nodesExpanded, iteration_nodesTouched, iteration_nodesUpdated;


	protected:

	// iterates until the root node is proved
	double ipn_update( CRState &pos, bool minFirst, double bound );
	// creates a node and registeres it in the transposition tables
	TPEntry* ipn_create_node( CRState &pos, bool minFirst, double bound );
	// updates the values/numbers of a parent due to the values of his childs
	void ipn_update_node( TPEntry *n, unsigned int &iteration );
	// gives back whether in the subtree under n there was an update needed (and done)
	// w.r.t. the expanded node expanded_node in iteration iteration
	bool ipn_update_branch( TPEntry *n, TPEntry* &expanded_node, unsigned int &iteration );
	// the main procedure of ipn
	void ipn_expand( TPEntry *n, unsigned int &iteration, TPEntry* &expanded_node );

	double MinHCost( CRState &pos, bool minsTurn = true );
	double MinGCost( CRState &pos1, CRState &pos2 );
	bool GoalTest( CRState &pos );
	double TerminalCost( CRState &pos );

	environment *env;
	bool canPause;

	// transposition tables
	GTTables min_gttables, max_gttables;
};




/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/

template<class state,class action,class environment>
double IPNTTables<state,action,environment>::ipn( CRState &pos, bool minFirst ) {

	// we have to make sure that we do not submit a terminal node to the algorithm
	if( GoalTest( pos ) ) return TerminalCost( pos );

	double b, c = MinHCost( pos, minFirst );

	unsigned int sumNodesExpanded = 0, sumNodesTouched = 0, sumNodesUpdated = 0;

	do {
		nodesExpanded = 0; nodesTouched = 0; nodesUpdated = 0;
		b = c;
//		fprintf( stdout, "bound set to %f\n", b );
		c = ipn_update( pos, minFirst, b );
		iteration_nodesExpanded.push_back( nodesExpanded );
		iteration_nodesTouched.push_back( nodesTouched );
		iteration_nodesUpdated.push_back( nodesUpdated );
		sumNodesExpanded += nodesExpanded;
		sumNodesTouched  += nodesTouched;
		sumNodesUpdated  += nodesUpdated;
	} while( c > b );

	nodesExpanded = sumNodesExpanded;
	nodesTouched  = sumNodesTouched;
	nodesUpdated  = sumNodesUpdated;

	return c;
}

// protected functions

template<class state,class action, class environment>
double IPNTTables<state,action,environment>::ipn_update( CRState &pos, bool minFirst, double bound ) {

	unsigned int iteration = 0;

	// create the root search node
	TPEntry *temptpentry, *root = new TPEntry( pos, 1, 1, true, bound, bound );

	while( root->proof_number != 0 && root->disproof_number != 0 ) {
		iteration++;
		// verbose
		// fprintf( stdout, "running from the root (to prove: %u, to disprove: %u, value: %f)\n",
		// 	root->proof_number, root->disproof_number, root->value );
		ipn_expand( root, iteration, temptpentry );
		// verbose
		// fprintf( stdout, "expanded node (%u,%u)(%u,%u), player %d, bound %f\n",
		//	temptpentry->pos[0].x, temptpentry->pos[0].y, temptpentry->pos[1].x, temptpentry->pos[1].y,
		//	temptpentry->minFirst, temptpentry->bound );
	}

	// verbose output
	if( root->proof_number == 0 )
		fprintf( stdout, "Root has been proved with bound %f and value %f\n", bound, root->value );
	else //if( root.disproof_number == 0 )
		fprintf( stdout, "Root has been disproved with bound %f and value %f\n", bound, root->value );

	// TODO: cleanup such that the objects behind the pointers in the transposition tables are really deleted
	typename GTTables::iterator gttit;
	typename TranspositionTable::iterator tit;
	for( gttit = min_gttables.begin(); gttit != min_gttables.end(); gttit++ ) {
		while( !gttit->second.empty() ) {
			tit = gttit->second.begin();
			gttit->second.erase( tit );
			delete (*tit);
		}
	}
	for( gttit = max_gttables.begin(); gttit != max_gttables.end(); gttit++ ) {
		while( !gttit->second.empty() ) {
			tit = gttit->second.begin();
			gttit->second.erase( tit );
			delete (*tit);
		}
	}
	min_gttables.clear();
	max_gttables.clear();

	double result = root->value;
	delete root;
	return result;
}




template<class state, class action, class environment>
typename IPNTTables<state,action,environment>::TPEntry* IPNTTables<state,action,environment>::ipn_create_node( CRState &pos, bool minFirst, double bound ) {

//	verbose
//	fprintf( stdout, "creating node for position (%u,%u)(%u,%u) player %d\n", pos[0].x, pos[0].y, pos[1].x, pos[1].y, minFirst );

	nodesTouched++;

	// transposition table lookup

	// allocation
	TPEntry *mytpentry = new TPEntry( pos );
	typename GTTables::iterator gttit;
	typename TranspositionTable::iterator tit;
	std::pair<typename GTTables::iterator, bool> insert_return;
	GTTables *current_gttables = minFirst?&min_gttables:&max_gttables;

	// find the transposition table for our rest bound
	gttit = current_gttables->find( bound );
	if( gttit != current_gttables->end() ) {
		// in case we found one: search for our position
		tit = gttit->second.find( mytpentry );
		if( tit != gttit->second.end() ) {
			delete mytpentry; // cleanup
			return (*tit); // return a pointer on the TPEntry
		}
	} else {
		// in case there isn't a transposition table for our bound yet
		// create one:
		GTTables_Pair mypair;
		mypair.first = bound;
		insert_return = current_gttables->insert( mypair );
		// sanity check: are we creating the transposition tables?
		assert( insert_return.second );
		gttit = insert_return.first;
	}

	// if we didn't find the position in the transposition table then
	// create a new entry

	if( GoalTest( pos ) ) {
		// if we are at a terminal state
		if( TerminalCost( pos ) > bound ) {
			// disproved
			mytpentry->proof_number    = UINT_MAX;
			mytpentry->disproof_number = 0;
			mytpentry->value           = TerminalCost( pos );
		} else {
			// proved
			mytpentry->proof_number    = 0;
			mytpentry->disproof_number = UINT_MAX;
			mytpentry->value           = TerminalCost( pos );
		}
	} else {
		// if we are at a non terminal
		if( bound < MinHCost( pos, minFirst ) ) {
			// if there is no possibility to prove this bound in this node, h-prune
			mytpentry->proof_number    = UINT_MAX;
			mytpentry->disproof_number = 0;
			mytpentry->value           = MinHCost( pos, minFirst );
		} else {
			// in every other case
			mytpentry->proof_number    = 1;
			mytpentry->disproof_number = 1;
			mytpentry->value           = bound;
		}
	}
	mytpentry->minFirst = minFirst;
	mytpentry->bound    = bound;

	// push the entry onto the transposition tables
	gttit->second.insert( mytpentry );
	return mytpentry;
}





template<class state, class action, class environment>
void IPNTTables<state,action,environment>::ipn_update_node( TPEntry *n, unsigned int &iteration ) {

	nodesUpdated++;

	double child_value;
	if( n->minFirst ) {
		// if we are in a min node
		n->proof_number = UINT_MAX;
		n->disproof_number = 0;
		n->value = DBL_MAX;
		for( typename std::vector<TPEntry*>::iterator iti = n->childs.begin(); iti != n->childs.end(); iti++ ) {
			// proof_number = min( child proof numbers )
			if( n->proof_number > (*iti)->proof_number ) n->proof_number = (*iti)->proof_number;
			// disproof_number = sum( child disproof numbers )
			n->disproof_number = uintplus( n->disproof_number, (*iti)->disproof_number );

			// value = min( childs values )
			child_value = ( n->bound - (*iti)->bound ) + (*iti)->value;
			if( n->value > child_value ) n->value = child_value;
		}
	} else {
		// if we are in a max node
		n->proof_number = 0;
		n->disproof_number = UINT_MAX;
		n->value = DBL_MIN;
		for( typename std::vector<TPEntry*>::iterator iti = n->childs.begin(); iti != n->childs.end(); iti++ ) {
			// proof number = sum( child proof numbers )
			n->proof_number = uintplus( n->proof_number, (*iti)->proof_number );
			// disproof number = min( child disproof numbers )
			if( n->disproof_number > (*iti)->disproof_number ) n->disproof_number = (*iti)->disproof_number;

			// value = max( child values )
			child_value = (n->bound - (*iti)->bound) + (*iti)->value;
			if( n->value < child_value ) n->value = child_value;
		}
	}
	n->iteration_of_last_update = iteration;
}



template<class state, class action, class environment>
bool IPNTTables<state,action,environment>::ipn_update_branch( TPEntry *n, TPEntry* &expanded_node, unsigned int &iteration ) {

	nodesTouched++;

	// h-prune, if we are too far down in the tree and the expanded_node is above us
	// should work fine with non-negative edge costs
	if( n->bound < expanded_node->bound )
		return false;

	// if this node has been updated throughout this iteration
	if( n->iteration_of_last_update == iteration ) return true;
	// the following is not needed anymore since expanded_node->iteration_of_last_update == iteration
	// if( n == expanded_node ) return true;

	// if this node is proved yet there is no update needed
	if( n->proof_number == 0 || n->disproof_number == 0 )
		return false;

	bool update_needed = false;
	for( typename std::vector<TPEntry*>::iterator iti = n->childs.begin(); iti != n->childs.end(); iti++ ) {
		update_needed |= ipn_update_branch( *iti, expanded_node, iteration );
	}

	if( update_needed )
		ipn_update_node( n, iteration );
	else
		n->iteration_of_last_update = iteration;

	return update_needed;
}


template<class state, class action, class environment>
void IPNTTables<state,action,environment>::ipn_expand( TPEntry *n, unsigned int &iteration, TPEntry* &expanded_node ) {

	if( n->childs.empty() ) {
		// if this node doesn't have any children yet then we create them
		nodesExpanded++; nodesTouched++;

		// Sanity check: do we expand terminal nodes? This should never happen!
		assert( !GoalTest( n->pos ) );

		// get the next positions for the current player
		int myid = n->minFirst?1:0;
		std::vector<state> neighbors;
		env->GetSuccessors( n->pos[myid], neighbors );
		if( canPause ) neighbors.push_back( n->pos[myid] );

		// generate all children
		CRState child_pos;
		for( unsigned int i = 0; i < neighbors.size(); i++ ) {
			// generate child position
			child_pos = n->pos;
			child_pos[myid] = neighbors[i];

			n->childs.push_back( ipn_create_node( child_pos, !n->minFirst, n->bound - MinGCost( n->pos, child_pos ) ) );
		}

		// Sanity check: if this fails it would mean we are in a node that has no
		// children but is not a terminal node
		assert( n->childs.size() > 0 );

		// since we have expanded this node, set it to the expanded node
		expanded_node = n;

		// update my own values
		ipn_update_node( n, iteration );

	} else {

		nodesTouched++;

		typename std::vector<TPEntry*>::iterator iti, itj;
		if( n->minFirst ) {
			for( iti = n->childs.begin(); iti != n->childs.end(); iti++ ) {
				if( (*iti)->proof_number == n->proof_number ) break;
			}
		} else {
			for( iti = n->childs.begin(); iti != n->childs.end(); iti++ ) {
				if( (*iti)->disproof_number == n->disproof_number ) break;
			}
		}

		// expand to the next child
		assert( iti != n->childs.end() );
		ipn_expand( *iti, iteration, expanded_node );

		// this is the point where the values are coming back from the computed subtree
		// now update all the other branches with respect to the expanded_node
		for( itj = n->childs.begin(); itj != n->childs.end(); itj++ ) {
			if( itj != iti )
				ipn_update_branch( *itj, expanded_node, iteration );
		}

		ipn_update_node( n, iteration );
	}
	return;
}


template<class state, class action, class environment>
double IPNTTables<state,action,environment>::MinHCost( CRState &pos, bool minsTurn ) {
	if( canPause )
		return ( 2. * env->HCost( pos[1], pos[0] ) - (minsTurn?MinGCost(pos,pos):0.) );
	else
		// distance from cop to the robber
		return env->HCost( pos[1], pos[0] );
}

// specification for state=xyLoc
template<>
double IPNTTables<xyLoc,tDirection,MapEnvironment>::MinHCost( CRState &pos, bool minsTurn ) {

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
double IPNTTables<state,action,environment>::MinGCost( CRState&, CRState& ) {
	return 1.;
}

template<class state, class action, class environment>
bool IPNTTables<state,action,environment>::GoalTest( CRState &pos ) {
	return( pos[0] == pos[1] );
}

template<class state, class action, class environment>
double IPNTTables<state,action,environment>::TerminalCost( CRState& ) {
	return 0.;
};

#endif
