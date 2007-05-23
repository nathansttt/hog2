/*
 * $Id: corridorAStar.h,v 1.6 2006/09/18 06:19:31 nathanst Exp $
 *
 *  corridorAStar.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 6/22/05.
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

#ifndef CORRIDORASTAR_H
#define CORRIDORASTAR_H

#include "SearchAlgorithm.h"
#include "Graph.h"
#include "Heap.h"

/** Corridor AStar builds a a* path between two nodes, restricting itself to
a particular corridor, if defined. The corridor must be set before every search
if it is to be used properly. After each GetPath call the corridor is reset. If
no corridor is defined, it will explore all nodes.
*/

class corridorAStar : public SearchAlgorithm {
public:
	corridorAStar();
	virtual ~corridorAStar() {}
	path *GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);
	/** get the best path from FROM to TO. Use hGoal as the heuristic goal. If TO is not on
		* the same level as from path will be returned that ends inside the child of TO. */
	path *getBestPath(GraphAbstraction *aMap, node *from, node *to, node *hGoal, reservationProvider *rp = 0);
	/** get the best path from aFROM to aTO. Use an insertion edge cost from the original from/to. */
	path *getBestPath(GraphAbstraction *aMap, node *afrom, node *ato, node *from, node *to, reservationProvider *rp = 0);
	void setCorridor(const std::vector<node *> *);
	virtual const char *GetName() { return "corridorAStar"; }
private:
		void relaxEdge(Heap *nodeHeap, Graph *g, GraphAbstraction *aMap,
									 edge *e, node *from, node *to, node *dest);
	void relaxFirstEdge(Heap *nodeHeap, Graph *g, GraphAbstraction *aMap,
											edge *e, node *from, node *afrom, node *ato, node *dest);

	void relaxFinalEdge(Heap *nodeHeap, Graph *g, GraphAbstraction *aMap,
											edge *e, node *from, node *to, node *realDest);
	path *extractBestPath(Graph *g, unsigned int current);
	const std::vector<node *> *corridor;
	std::vector<node *> emptyCorridor;
};

#endif
