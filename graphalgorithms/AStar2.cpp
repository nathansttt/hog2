/*
 * $Id: aStar2.cpp,v 1.3 2006/10/18 23:52:25 nathanst Exp $
 *
 *  Hierarchical Open Graph File
 *
 *  Created by Nathan Sturtevant on 9/29/04.
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

#include "aStar2.h"

path *aStar2::getPath(graphAbstraction *aMap, node *from, node *to, reservationProvider *)
{
	if ((from == 0) || (to == 0) || (from == to))
		return 0;
  path *lastPath = 0;
  map = aMap;
	
  std::vector<unsigned int> eligibleNodeParents;
	
  lastPath = getAbstractPath(map->getAbstractGraph(from->getLabelL(kAbstractionLevel)),
														 from->getNum(), to->getLabelL(kParent),
														 eligibleNodeParents,
														 kTemporaryLabel,
														 to->getNum());
  return lastPath;
}
	
