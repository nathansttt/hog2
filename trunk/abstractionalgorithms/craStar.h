/*
 * $Id: craStar.h,v 1.4 2006/09/18 06:19:31 nathanst Exp $
 *
 *  craStar.h
 *  original: craStar.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 6/23/05.
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

#ifndef CRASTAR_H
#define CRASTAR_H

#include "searchAlgorithm.h"
#include "heap.h"
#include "corridorAStar.h"

/**
* The pra* search algorithm which does partial pathfinding using abstraction.
 */

class craStar : public searchAlgorithm {
	
public:
	// From HPA* original code - smoothwizard.h
	typedef enum {
		NORTH,
		EAST,
		SOUTH,
		WEST,
		NE,
		SE,
		SW,
		NW} Direction;

	typedef enum {
		BEGIN,
		END,
		TWO_BACK} SmoothType;


	craStar();
	virtual ~craStar() {}
	virtual path *getPath(graphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);
	virtual const char *getName()
	{ return algName; } 
	void setPartialPathLimit(int limit)
	{ partialLimit = limit; sprintf(algName,"CRA*(%d)", partialLimit); }
	int getPartialPathLimit() { return partialLimit; }

	/**
	 * Set whether the path will be smoothed or not. Default is TRUE. 
	 */ 
	void setSmoothing(bool smooth) {smoothing = smooth;}

	/**
	 * Set the smoothing type.Default is BEGIN
	 *
	 * Whenever we update the path with a better subpath, 
	 * if s = BEGIN, smoothing is restarted from the beginning of this subpath
	 * if s = END, smoothing is restarted from the end of the new subpath
	 * if s = TWO_BACK, we go back two from the end of the new subpath
	 */
	void setSmoothType(SmoothType s) { smType = s; }

	/**
	 * set the abstract level 
	 *
	 * if the level is smaller than 1, the abstract level is chosen
	 * dynamically.
	 */
	void setAbstractLevel(int level) { absLevel = level; }

protected:
	void setupSearch(graphAbstraction *aMap,
			 std::vector<node *> &fromChain, node *from,
			 std::vector<node *> &toChain, node *to);

	void findGoalNode(graphAbstraction* aMap,node* n, std::vector<node *> &toChain);
	path *buildNextAbstractPath(graphAbstraction *, path *lastPath,
				    std::vector<node *> &fromChain,
				    std::vector<node *> &toChain,
				    reservationProvider *);


	path *trimPath(path *lastPath, node *origDest);
	path* buildAbstractPath(graphAbstraction *aMap, 
													std::vector<node *> &fromChain,
													std::vector<node *> &toChain,
													reservationProvider *rp);
	path* doRefinement(graphAbstraction *aMap, path* absPath, 
										 std::vector<node*> &fromChain, 
										 std::vector<node*> &toChain);

	node* getNextNode(graphAbstraction *aMap, node* currentLow, path* returnPath, path* apath, graph* g, int abstractLevel);

	int partialLimit;
	bool expandSearchRadius;
	corridorAStar cAStar;
  char algName[30];

	path* smoothPath(graphAbstraction* m,path* p);

	std::vector<node*> lookup;
	path* nextPathNode(graphAbstraction* m,node* n, int dir);
	node* getNextNode(mapAbstraction* m,int x, int y, int dir);
	bool nextInLookup(int last, int curr, std::vector<node*> lookup);
	int backTwoNodes(int i, std::vector<node*> lookup);

	void findMinMax(path* p);

	SmoothType smType;
	bool smoothing;
	int absLevel;

	int minx;
	int maxx;
	int miny;
	int maxy;
};


#endif
