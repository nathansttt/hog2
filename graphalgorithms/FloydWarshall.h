/*
 *  FloydWarshall.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 12/7/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef FLOYDWARSHALL_H
#define FLOYDWARSHALL_H

#include "Graph.h"
#include <vector>

void FloydWarshall(Graph *g, std::vector<std::vector<double> > &lengths);

#endif
