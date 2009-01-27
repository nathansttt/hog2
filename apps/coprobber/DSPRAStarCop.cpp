#include "DSPRAStarCop.h"

DSPRAStarCop::DSPRAStarCop( GraphAbstraction *_graphabs, unsigned int _cop_speed ):
	graphabs(_graphabs), cop_speed(_cop_speed), g(graphabs->GetAbstractGraph(0)),
	pra(new praStar())
{ };

DSPRAStarCop::~DSPRAStarCop() {
	delete pra;
};

graphState DSPRAStarCop::MakeMove( graphState &robber, graphState &cop ) {

	nodesExpanded = 0; nodesTouched = 0;

	if( robber == cop ) return robber;

	if( pathcache.size() == 0 ) {
		// compute new pathcache

		node *r = g->GetNode( robber );
		node *c = g->GetNode( cop );

		path *prapath  = pra->GetPath( graphabs, c, r );
		nodesExpanded  = pra->nodesExpanded;
		nodesTouched   = pra->nodesTouched;
		path *temppath = prapath;

		unsigned int steps = prapath->length() / 2 / cop_speed;
		if( steps == 0 ) steps = 1; // make at least one step

		for( unsigned int i = 0; i < steps; i++ ) {
			for( unsigned int j = 0; j < cop_speed; j++ ) {
				if( temppath->next != NULL )
					temppath = temppath->next;
				else
					break;
			}
			pathcache.push_back( temppath->n->GetNum() );
			if( temppath->next == NULL ) break;
		}

		// cleanup
		delete prapath;
		// done computing new pathcache
	}

	if( pathcache.size() > 0 ) {
		graphState result = pathcache[0];
		pathcache.erase( pathcache.begin() );
		return result;
	} else {
		// if we cannot make a move, we'll stay where we are
		return cop;
	}
};
