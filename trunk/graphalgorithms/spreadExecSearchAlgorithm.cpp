/*
 * $Id: spreadExecSearchAlgorithm.cpp,v 1.3 2006/10/18 23:52:25 nathanst Exp $
 *
 *  spreadExecSearchAlgorithm.cpp
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

#include "spreadExecSearchAlgorithm.h"

path *spreadExecSearchAlgorithm::getPath(graphAbstraction *_aMap, node *from, node *to, reservationProvider *_rp)
{
	setTargets(_aMap, from, to, _rp);
	path *p;
	while ((p = think()) == 0) {}
	return p;
}

