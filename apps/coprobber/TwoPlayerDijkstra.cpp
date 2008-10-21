#include "TwoPlayerDijkstra.h"

template<>
uint64_t StateHash<xyLoc>( xyLoc s ) {
	return( ((uint64_t)s.x)<<32 | (((uint64_t)s.y)<<32)>>32 );
}


template<>
double TwoPlayerDijkstra<xyLoc, tDirection, MapEnvironment>::GCost( xyLoc &s1, xyLoc &s2 ) {
/*
	if( abs(s1.x-s2.x)==1 && abs(s1.y-s2.y)==1 ) return 1.5;
*/
	return 1.;
}

