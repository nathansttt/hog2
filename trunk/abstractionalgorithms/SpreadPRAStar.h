/*
 * $Id: spreadPRAStar.h,v 1.3 2006/09/18 06:19:31 nathanst Exp $
 *
 *  spreadPRAStar.h
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
#include "heap.h"
#include "corridorAStar.h"

#ifndef SPREADPRASTAR_H
#define SPREADPRASTAR_H

#include "spreadExecSearchAlgorithm.h"

class spreadPRAStar : public spreadExecSearchAlgorithm {
public:
	spreadPRAStar();
	virtual ~spreadPRAStar() {}
	void setPartialPathLimit(int limit)
	{ partialLimit = limit; sprintf(algName, "SpreadPRA*(%d)", partialLimit); }
	
	virtual path *getPath(graphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);
	void setTargets(graphAbstraction *, node *, node *, reservationProvider *);
	int getNumThinkSteps(); 
	path *think(); 
private:
	void setupSearch(graphAbstraction *aMap,
									 std::vector<node *> &fromChain, node *from,
									 std::vector<node *> &toChain, node *to);
	path *buildNextAbstractPath(graphAbstraction *, path *lastPath,
															std::vector<node *> &fromChain,
															std::vector<node *> &toChain,
															reservationProvider *);
	path *trimPath(path *lastPath, node *origDest);

	char algName[30];
	int partialLimit;
	bool expandSearchRadius;
	path *lastPath;
	std::vector<node *> startChain, endChain;
	corridorAStar cAStar;
};

#endif
