/*
 * $Id: praStar2.h,v 1.6 2006/09/18 06:19:31 nathanst Exp $
 *
 *  praStar2.h
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
 *  Skipping added by Renee Jansen on 5/15/06.
 */

#ifndef PRASTAR2_H
#define PRASTAR2_H

#include "searchAlgorithm.h"
#include "heap.h"
#include "corridorAStar.h"

/**
 * The pra* search algorithm which does partial pathfinding using abstraction.
 */

class praStar2 : public searchAlgorithm {
	
public:
	praStar2();
	virtual ~praStar2() {}
	virtual path *getPath(graphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);
	virtual const char *getName()
	{ return algName; } 
	void setPartialPathLimit(int limit)
	{ partialLimit = limit; sprintf(algName,"PRA2*(%d)", partialLimit); }
	int getPartialPathLimit() { return partialLimit; }
	void setEnhancedAbstractPathing(bool enhance) { enhancedAbstractPathing = enhance; }
  
  /** Set whether we want to expand the corridor to search in. 
   * Set whether we want to expand the corridor from just the parents along the path to these parents and 
   * their neighbors
   */ 
  void setExpandSearchRadius(bool _expandSearchRadius) { expandSearchRadius = _expandSearchRadius; }

  /** Set whether we want to start planning in the middle between base and top level of abstraction.
   * If true, sets the top level for pathfinding to be the level in the middle between the top level and the base 
   * level of abstraction. If false, top level for pathfinding is the topmost abstraction if fixedPlanLevel=-1, and 
   * the fixedPlanLevel otherwise.
   */ 
  void setPlanFromMiddle(bool _planFromMiddle) { planFromMiddle = _planFromMiddle; } 
  
  /** Set the level of abstraction to start pathfinding at.
   * Setting the fixed plan level to -1 will do dynamic level selection if setPlanFromMiddle is true, and it 
   * will start at the highest possible level if setPlanFromMiddle is false. Default is -1.
   */
  void setFixedPlanLevel(int p) { fixedPlanLevel = p; }

  /** Set how many abstraction layers should be skipped at each step.
   * A skip of -1 will only do pathfinding at the top level of abstraction and at the base level. A skip of 0 is 
   * the default, which will do pathfinding at each level.
   */
  void setSkipParameter(int _skip) { skip = _skip; }

  int getTopLevel() { return topLevel; }
  int getNumLevelsUsed() { return numLevels; }

protected:
	void setupSearch(graphAbstraction *aMap,
			 std::vector<node *> &fromChain, node *from,
			 std::vector<node *> &toChain, node *to);
	path *buildNextAbstractPath(graphAbstraction *, path *lastPath,
				    std::vector<node *> &fromChain,
				    std::vector<node *> &toChain,
				    reservationProvider *);
	path *trimPath(path *lastPath, node *origDest);

  void selectTopAbstractionLevel(graphAbstraction * aMap,
				 std::vector<node *> &fromChain,
				 std::vector<node *> & toChain);

	int partialLimit;
	bool enhancedAbstractPathing;
	bool expandSearchRadius;
	corridorAStar cAStar;
  char algName[30];

  bool planFromMiddle;
  int fixedPlanLevel;
  int skip; 
  int topLevel; //the highest level at which an abstract path is found
  int numLevels; // the number of levels at which an abstract path is found
};


#endif
