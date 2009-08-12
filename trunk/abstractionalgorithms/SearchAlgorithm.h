/*
 * $Id: SearchAlgorithm.h,v 1.8 2006/10/18 23:52:25 nathanst Exp $
 *
 *  Hierarchical Open Graph File
 *
 *  Created by Nathan Sturtevant on 9/28/04.
 *  Copyright 2004 Nathan Sturtevant. All rights reserved.
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

#ifndef SEARCHALGORITHM_H
#define SEARCHALGORITHM_H

#include "Graph.h"
#include "Path.h"
#include "MapAbstraction.h"
#include "UnitSimulation.h"
#include "ReservationProvider.h"

/**
 * A generic algorithm which can be used for pathfinding.
 */

class SearchAlgorithm {
public:
	SearchAlgorithm() { nodesExpanded = nodesTouched = 0; }
	virtual ~SearchAlgorithm() {}
	virtual const char *GetName() = 0;
	virtual path *GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0) = 0;
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	virtual void LogFinalStats(StatCollection *) {}

	//protected:
	uint32_t nodesExpanded;
	uint32_t nodesTouched;
};

extern void DoRandomPath(GraphAbstraction *aMap, SearchAlgorithm *sa, bool repeat = false);

#endif
