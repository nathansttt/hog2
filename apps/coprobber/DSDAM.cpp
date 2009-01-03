#include "DSDAM.h"

/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/

DSDAM::DSDAM( GraphAbstraction *_gabs, bool _canPause, unsigned int _cop_speed, bool _useAbstraction ):
	gabs(_gabs), canPause(_canPause), cop_speed(_cop_speed), useAbstraction(_useAbstraction), pra( new praStar() )
{
	// create the minimax objects
	for( unsigned int level = 0; level < gabs->getNumAbstractGraphs(); level++ ) {
		graphmapheuristics.push_back( new MaximumNormGraphMapHeuristic( gabs->GetAbstractGraph( level ) ) );
		graphenvironments.push_back( new GraphEnvironment( gabs->GetAbstractGraph( level ), graphmapheuristics[level] ) );
		dsminimax.push_back( new DSMinimax<graphState,graphMove>( graphenvironments[level], canPause, cop_speed ) );

		// note that we do NOT use path costs in higher levels, the tree is only generated
		// and the leafs are evaluated, no edge costs are used! And yes, this kind of makes sense ;-)
		dsminimax[level]->useEdgeCosts( false );
	}
	// use edge costs on the lowest level
	dsminimax[0]->useEdgeCosts( true );

};

DSDAM::~DSDAM() {
	while( !dsminimax.empty() ) {
		std::vector<DSMinimax<graphState,graphMove>*>::iterator it = dsminimax.begin();
		DSMinimax<graphState,graphMove> *temp = *it;
		dsminimax.erase( it );
		delete temp;
	}
	while( !graphenvironments.empty() ) {
		std::vector<GraphEnvironment*>::iterator it = graphenvironments.begin();
		GraphEnvironment *temp = *it;
		graphenvironments.erase( it );
		delete temp;
	}
	while( !graphmapheuristics.empty() ) {
		std::vector<MaximumNormGraphMapHeuristic*>::iterator it = graphmapheuristics.begin();
		MaximumNormGraphMapHeuristic *temp = *it;
		graphmapheuristics.erase( it );
		delete temp;
	}
	delete pra;
//	delete gabs;
};

void DSDAM::dam( node* pos_robber, node* pos_cop, std::vector<node*> &resultpath, bool minFirst, double depth ) {
	resultpath.clear();

	// find the joint hierarchy
	std::vector<node*> robberChain, copChain;
	gabs->GetNumAbstractGraphs( pos_robber, pos_cop, robberChain, copChain );

	// determine the level where we start planning
	// make a sanity check (to ensure the start level actually makes sense)
	// if there should not be used any abstraction then set the start level to 0
	unsigned int start = robberChain.size() / 2;
	assert( robberChain.size() <= gabs->getNumAbstractGraphs() );
	assert( robberChain.size() == copChain.size() );
	if( !useAbstraction ) start = 0;

	node* target = NULL;

	// now start planning beginning at the highest level until we find a possible solution
	for( int level = start; level >= 0; level-- ) {
		std::vector<graphState> numberpath;
		graphState probber = (graphState) robberChain[level]->GetNum();
		graphState pcop    = (graphState) copChain[level]->GetNum();
		double temp = dsminimax[level]->minimax( probber, pcop, numberpath, minFirst, depth );
		if( fgreater(temp,0.) ) {
			// the robber can escape on this level

			// verbose
			//printf( "found solution on level %d with %g\n", level, temp );

			assert( numberpath.size() > 0 );

			// verbose
			//printf( "path: " );
			//for( unsigned int i = 0; i < numberpath.size(); i++ ) {
			//	printf( "%lu(%d) ", numberpath[i], gabs->GetAbstractGraph(level)->GetNode(numberpath[i])->getUniqueID() );
			//}
			//printf( "\n" );

			// select my last move in the chain of actions
			if( numberpath.size() == 1 || numberpath.size()%2 ) //== 1
				target = gabs->GetAbstractGraph( level )->GetNode( numberpath[numberpath.size()-1] );
			else
				target = gabs->GetAbstractGraph( level )->GetNode( numberpath[numberpath.size()-2] );
			break;
		}
		// else continue to the next level
	}

	if( target != NULL ) {
		// select a target on the lowest level of abstraction
		target = gabs->GetRandomGroundNodeFromNode( target );
		//printf( "going for node %d\n", target->GetNum() );
	} else {
		fprintf( stderr, "Warning: strangely could not select a target\n" );
		fprintf( stderr, "Warning: selecting first node in the map as a target\n" );
		target = gabs->GetAbstractGraph( 0 )->GetNode( 0 );
	}

	path *p, *ptemp;
	if( minFirst )
		p = pra->GetPath( gabs, pos_cop, target );
	else
		p = pra->GetPath( gabs, pos_robber, target );

	// convert p to a vector
	ptemp = p;
	if( minFirst ) {
		// since PRA* gives us back a path we only have to choose
		// the positions that he can get to in cop_speed
		// and the last position he wants to get to
		// note: that we run towards the selected position
		// as fast as possible, this might actually not be optimal
		unsigned int step = 0;
		while( ptemp ) {
			if( step % cop_speed == 0 || ptemp->next == NULL ) {
				resultpath.push_back( ptemp->n );
			}
			ptemp = ptemp->next;
			step++;
		}

	} else {
		// the robber does every step
		while( ptemp ) {
			resultpath.push_back( ptemp->n );
			ptemp = ptemp->next;
		}
	}

	// cleanup
	delete ptemp;
	delete p;

	return;
};
