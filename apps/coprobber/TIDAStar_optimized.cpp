#include "TIDAStar_optimized.h"

// specification for state=xyLoc
/*
template<>
double TIDAStar<xyLoc,tDirection,MapEnvironment>::MinHCost( CRState &pos, bool minsTurn ) {
	if( GoalTest( pos ) ) return TerminalCost( pos );
//	if( pos[0] == pos[1] ) return 0.;

	double dist;
	dist = max( abs(pos[1].x - pos[0].x), abs(pos[1].y - pos[0].y) );

	if( canPause )
		return( 2. * dist - (minsTurn?MinGCost(pos,pos):0.) );
	else
		return dist;
}
*/

template<>
double TIDAStar<xyLoc, tDirection, MapEnvironment>::MinHCost( CRState &pos, bool minsTurn ) {
	if( GoalTest( pos ) ) return TerminalCost( pos );

	int a = abs(pos[1].x-pos[0].x);
	int b = abs(pos[1].y-pos[0].y);
	int turns   = (a>b)?a:b;
	double dist = (a>b)?(b*1.5+a-b):(a*1.5+b-a);

	if( canPause )
		return ( dist + MinGCost( pos, pos ) * (a - (minsTurn?1:0)) );
	else
		return dist;
}



template<>
double TIDAStar<xyLoc, tDirection, MapEnvironment>::MinGCost( CRState &p1, CRState &p2 ) {
	if( abs(p1[0].x-p2[0].x)==1 && abs(p1[0].y-p2[0].y)==1 ) return 1.5;
	if( abs(p1[1].x-p2[1].x)==1 && abs(p1[1].y-p2[1].y)==1 ) return 1.5;
	return 1.;
}
