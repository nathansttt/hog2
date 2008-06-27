#include "MultilevelMarkovGame.h"
#include "GraphAbstraction.h"
#include "GraphEnvironment.h"
#include "CopRobberGame.h"

#ifndef MULTILEVELCOPROBBERGAME_H
#define MULTILEVELCOPROBBERGAME_H

/*------------------------------------------------------------------------------
| MultilevelGraphHeuristic
------------------------------------------------------------------------------*/
class MultilevelGraphHeuristic: public GraphHeuristic {
	public:

	MultilevelGraphHeuristic( GraphAbstraction *_gabstraction, unsigned int _level ):
		gabstraction(_gabstraction), level(_level) {};
	virtual ~MultilevelGraphHeuristic() {};
	virtual double HCost( graphState &state1, graphState &state2 ) {
		return( gabstraction->h( gabstraction->GetAbstractGraph(level)->GetNode(state1), gabstraction->GetAbstractGraph(level)->GetNode(state2) ) );
	};

	private:
	GraphAbstraction *gabstraction;
	unsigned int level;
};


/*------------------------------------------------------------------------------
| MultilevelCopRobberGame
------------------------------------------------------------------------------*/
class MultilevelCopRobberGame:
	public MultilevelMarkovGame<graphState,graphMove>
{
	public:

	typedef CopRobberGame::CRState CRState;

	MultilevelCopRobberGame( GraphAbstraction *gabstraction, unsigned int num_cops, bool simultaneous, bool playerscanpass = false );

	virtual ~MultilevelCopRobberGame();

	virtual unsigned int NumLevels();

	virtual MarkovGame<graphState,graphMove>* GetMarkovGame( unsigned int level );
	// the following functions assume that there can still be a parent/child
	// so do not mess up with it by using an slevel that is too high/low!
	virtual CRState GetParent( CRState s, unsigned int slevel );
	virtual unsigned int GetNumChildren( CRState s, unsigned int slevel );
	virtual CRState GetNthChild( CRState s, unsigned int slevel, unsigned int n );

	protected:
		GraphAbstraction *gabstraction;
		unsigned int num_cops;
		bool playerscanpass;

	private:
		// these variables are just holds of pointers to avoid memory leaks
		CopRobberGame **crgames;
		MultilevelGraphHeuristic **graphheuristics;
		GraphEnvironment **graphenvironments;
};

#endif
