//
//  PDBHeuristic.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/19/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//

#ifndef hog2_glut_PDBHeuristic_h
#define hog2_glut_PDBHeuristic_h

template <class state>
class PDBHeuristic : public Heuristic<state> {
public:
	virtual ~Heuristic() {}
	virtual double HCost(const state &a, const state &b) = 0;
};


#endif
