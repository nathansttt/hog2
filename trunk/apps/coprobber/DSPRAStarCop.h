#include "PRAStar.h"
#include "GraphAbstraction.h"
#include "GraphEnvironment.h"

#ifndef DSPRASTARCOP_H
#define DSPRASTARCOP_H

/*
	Cop move policy. The cop chooses a move due to PRA* and
	follows it for cop_speed steps
*/
class DSPRAStarCop {

	public:

	// constructor and destructor
	DSPRAStarCop( GraphAbstraction *graphabs, unsigned int cop_speed = 1 );
	~DSPRAStarCop();

	graphState MakeMove( graphState &robber, graphState &cop );

	unsigned int nodesExpanded;
	unsigned int nodesTouched;

	protected:

	GraphAbstraction *graphabs;
	unsigned int cop_speed;
	Graph *g;
	praStar *pra;

};

#endif
