#include "statehash.h"
#include "GraphEnvironment.h"
#include "Map2DEnvironment.h"

template<>
uint64_t sfbds_state_hash<graphState>( graphState s1, graphState s2 ) {

	// sort the two states
	if( s1 > s2 ) {
		graphState temp = s1;
		s1 = s2;
		s2 = temp;
	}

	// take the last 32 bits of state 1 and the last 32 bits of state 2
	#ifdef SFBDS_LITTLE_ENDIAN
		return ( (s1 >> 32) | ( (s2 >> 32) << 32 ) );
	#else
		return ( (s1 << 32) | ( (s2 << 32) >> 32 ) );
	#endif
};

template<>
uint64_t sfbds_state_hash<xyLoc>( xyLoc s1, xyLoc s2 ) {

	// sort the states in x and alternatively in y
	if( s1.x > s2.x || (s1.x == s2.x && s1.y > s2.y) ) {
		xyLoc temp = s1; s1 = s2; s2 = temp;
	}

	// take the last 16 bits of s1.x and s1.y
	// plus the last 16 bits of s2.x and s2.y
	#ifdef SFBDS_LITTLE_ENDIAN
		return(
			((uint64_t)s1.x)>>48 |
			((uint64_t)s1.y)>>32 |
			((uint64_t)s2.x)>>16 |
			((uint64_t)s2.y) );
	#else
		return(
			((uint64_t)s1.x)<<48 |
			((uint64_t)s1.y)<<32 |
			((uint64_t)s2.x)<<16 |
			((uint64_t)s2.y) );
	#endif
};
