#include "DSCREnvironment.h"

template<>
double DSCREnvironment<xyLoc,tDirection>::HCost( xyLoc &s1, xyLoc &s2 ) {

	// we can just take the max since all edge costs are 1
	// (including the diagonals)
	return( max(abs(s1.x-s2.x),abs(s1.y-s2.y)) / (double)cop_speed );
};

template<>
double DSCREnvironment<graphState,graphMove>::HCost( graphState &s1, graphState &s2 ) {

	return ceil( env->HCost( s1, s2 ) / (double)cop_speed );
};
