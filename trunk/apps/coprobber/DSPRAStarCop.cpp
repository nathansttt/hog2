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

	node *r = g->GetNode( robber );
	node *c = g->GetNode( cop );

	path *prapath  = pra->GetPath( graphabs, c, r );
	nodesExpanded  = pra->nodesExpanded;
	nodesTouched   = pra->nodesTouched;
	path *temppath = prapath;

	for( unsigned int i = 0; i < cop_speed; i++ ) {
		if( temppath->next != NULL )
			temppath = temppath->next;
		else
			break;
	}

	graphState result = temppath->n->GetNum();
	delete prapath;

	return result;
};
