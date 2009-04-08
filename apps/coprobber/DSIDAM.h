#include <vector>
#include "MapAbstraction.h"
#include "DSMinimax.h"
#include "PRAStar.h"
#include "MaximumNormAbstractGraphMapHeuristic.h"

#ifndef DSIDAM_H
#define DSIDAM_H

/*
	improved dynamic abstract minimax (IDAM)
	code functions with DSCREnvironment, thus possibly faster cop
*/

class DSIDAM:
	praStar // because we want to use the refinement routines from PRA*
{

	public:

	// constructor
	DSIDAM( MapAbstraction *gabs, bool canPause = true, unsigned int cop_speed = 1, bool useAbstraction = true );
	~DSIDAM();

	void dam( node* pos_robber, node* pos_cop, std::vector<node*> &path, bool minFirst = true, double depth = 5., double start_level_fraction = 0.5 );

	node* MakeMove( node* pos_robber, node* pos_cop, bool minFirst = true, double depth = 5., double start_level_fraction = 0.5 );

	// statistic variables that get resetted everytime dam(...) is called
	unsigned int myNodesExpanded, myNodesTouched;

	protected:

	MapAbstraction *gabs;
	bool canPause;
	unsigned int cop_speed;
	bool useAbstraction;

	// cache the minimax objects and graph environments for each level of abstraction
	std::vector<DSMinimax<graphState,graphMove>* > dsminimax;
	std::vector<GraphEnvironment*> graphenvironments;
	std::vector<MaximumNormAbstractGraphMapHeuristic*> graphmapheuristics;
};

#endif
