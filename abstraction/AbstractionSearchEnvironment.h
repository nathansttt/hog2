/*
 *  AbstractionSearchEnvironment.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 4/5/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef ABSTRACTIONSEARCHENVIRONMENT_H
#define ABSTRACTIONSEARCHENVIRONMENT_H

#include "SearchEnvironment.h"
#include "graphAbstraction.h"

class AbstractionSearchEnvironment : public SearchEnvironment
{
public:
	AbstractionSearchEnvironment(graphAbstraction *_ga, int _level)
	:ga(_ga), level(_level) {  }
	~AbstractionSearchEnvironment() {}
	void getNeighbors(uint32_t nodeID, std::vector<uint32_t> &neighbors);
	double heuristic(uint32_t node1, uint32_t node2);
	double gcost(uint32_t node1, uint32_t node2);
private:
		graphAbstraction *ga;
	int level;
};

#endif
