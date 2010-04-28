/*
 *  FunctionAproximator.cpp
 *  games
 *
 *  Created by Adam White on Mon Apr 04 2005.
 *  Copyright (c) 2005 __MyCompanyName__. All rights reserved.
 *
 */

#include "FunctionApproximator.h"

double FunctionApproximator::getLearnRate()
{
	return rate;
}

void FunctionApproximator::setLearnRate(double _rate)
{
	rate = _rate;
}
