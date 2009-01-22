#include "DSPRAStarCop.h"

DSPRAStarCop::DSPRAStarCop( GraphAbstraction *_graphabs, unsigned int _cop_speed ):
	graphabs(_graphabs), cop_speed(_cop_speed), g(graphabs->GetAbstractGraph(0)),
	pra(new praStar())
{ };

DSPRAStarCop::~DSPRAStarCop() {
	delete pra;
};

graphState DSPRAStarCop::MakeMove( graphState &robber, graphState &cop ) {

	node *r = g->GetNode( robber );
	node *c = g->GetNode( cop );

	path *prapath  = pra->GetPath( graphabs, c, r );
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
