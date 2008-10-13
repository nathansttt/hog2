#include "Minimax.h"


// specification for state=xyLoc
template<>
double Minimax<xyLoc,tDirection,MapEnvironment>::MinHCost( CRState &pos, bool minsTurn ) {
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


// specification for state=xyLoc
template<>
uint64_t CRHash<xyLoc>( const std::vector<xyLoc> &pos ) {
	return( ((uint64_t)pos[0].x)<<48 | (((uint64_t)pos[0].y)<<48)>>16 |
		(((uint64_t)pos[1].x)<<48)>>32 | (((uint64_t)pos[1].y)<<48)>>48 );
}

