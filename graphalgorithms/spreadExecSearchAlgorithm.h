/*
 * $Id: spreadExecSearchAlgorithm.h,v 1.3 2006/09/18 06:19:31 nathanst Exp $
 *
 *  spreadExecSearchAlgorithm.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 6/27/05.
 *  Copyright 2005 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include "searchAlgorithm.h"

#ifndef SPREADEXECSEARCHALGORITHM_H
#define SPREADEXECSEARCHALGORITHM_H

class spreadExecSearchAlgorithm : public searchAlgorithm {
public:
	spreadExecSearchAlgorithm() { }
	virtual const char *getName() { return "unnamed"; }
	virtual path *getPath(graphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);

	virtual void setTargets(graphAbstraction *_aMap, node *s, node *e, reservationProvider *_rp = 0)
	{ rp = _rp; start = s; end = e; aMap = _aMap; }
	/** how many times do we have to "think" to find the solution, return -1 if unknown */
	virtual int getNumThinkSteps() = 0; 
	/** do next processing for path, returns avaliability of path moves */
	virtual path *think() = 0; 
protected:
	node *start, *end;
	reservationProvider *rp;
	graphAbstraction *aMap;
};

#endif
