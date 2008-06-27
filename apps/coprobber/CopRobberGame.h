#include "GraphEnvironment.h"
#include "Map2DEnvironment.h"
#include "CopRobberEnvironment.h"
#include "MarkovGame.h"

#ifndef COPROBBERGAME_H
#define COPROBBERGAME_H

/*!
	The Cop vs. Robber game: This is what we are talking of!

	\see CopRobberEnvironment.h
*/
class CopRobberGame:
	public CopRobberEnvironment<graphState,graphMove>,
	public MarkovGame<graphState, graphMove>
{

	public:

	// types CRState, CRMove and CRAction are defined in
	// CopRobberEnvironment.h


	CopRobberGame( GraphEnvironment *genv, unsigned int num_cops, bool simultaneous = false, bool playerscanpass = false );

	virtual ~CopRobberGame();

	virtual unsigned int GetNumPlayers() const;

	virtual void GetPossiblePlayerActions( unsigned int player, CRState s, std::vector<CRAction> &actions );
	virtual void GetPossibleOpponentActions( unsigned int excluded_player, CRState s, std::vector<CRMove> &actions );

	virtual double GetReward( unsigned int player, CRState s, std::vector<CRAction> act );

	virtual unsigned int GetNumStates() const;

	virtual CRState GetStateByNumber( unsigned int num );
	virtual unsigned int GetNumberByState( CRState s );



	virtual void WriteExpectedRewardToDisc( const char* filename, unsigned int player, double *V );
	virtual void ReadExpectedRewardFromDisc( const char* filename, unsigned int &player, double *&V );


	protected:
	unsigned int num_cops;
	GraphEnvironment *genv;
};


#endif
