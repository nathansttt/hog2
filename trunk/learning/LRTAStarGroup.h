///*
// * $Id: LRTAStarGroup.h,v 1.4 2006/03/27 02:27:55 bulitko Exp $
// *
// *  LRTAStarGroup.h
// *  HOG
// *
// *  Created by Shanny (based on Vadim Bulitko's LRTS) on 10/03/05.
// *  Copyright 2004 University of Alberta. All rights reserved.
// *
// */
//
//#include "unitGroup.h"
//#include "LRTAStarUnit.h"
//
//#ifndef LRTSTARGROUP_H
//#define LRTSTARGROUP_H
//
//class LRTAStarGroup : public UnitGroup {
//public:
//	LRTAStarGroup(mapProvider *);
//	~LRTAStarGroup() { if (h != NULL) delete [] h; }
//	tDirection makeMove(unit *u, mapProvider *, reservationProvider *, simulationInfo *simInfo);
//	void openGLDraw(mapProvider *, simulationInfo *);
//	bool done();
//	void startNewTrial(statCollection *stats);
//	void cycleDisplayMode(void) { displayMode++; }
//	void increaseDisplayALevel(void) { ; }
//	void decreaseDisplayALevel(void) { ; }
//	unsigned memorySize(void) { return hSize*sizeof(double); }
//	double getMemoryUsage(void) { return (double) hSize; }
//	
//	// Return the h value and, if needed, cache it
//	double getH(node *from, node *to) {
//		double value = h[from->getNum()];
//		return fless(value,0.0) ? aMap->h(from, to) : value;
//	}
//
//	// Return the difference between the current and the initial value of the array
//	double getDiffH(node *from, node *to) {
//		double value = h[from->getNum()];
//		return fless(value,0.0) ? 0.0 : value - aMap->h(from,to);
//	}
//
//	// Set the h value
//	void setH(node *from, node *to, double newValue) {
//		if (fless(h[from->getNum()],0.0))
//			hSize++;
//		h[from->getNum()] = newValue;
//	}
//	
//private:
//	friend class LRTAStar;
//	
//	void allocateH();
//
//	recColor getColor(double v);
//	
//	// Data shared by LRTA units
//	double *h;						// the shared h heuristic at the base level
//	unsigned hSize;					// number of h values cached
//	short int displayMode;			// defines the display mode for the heuristic
//	mapAbstraction* aMap;			// our abstracted map
//};
//
//#endif
