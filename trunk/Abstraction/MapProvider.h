/*
 * $Id: mapProvider.h,v 1.5 2006/09/18 06:22:14 nathanst Exp $
 *
 *  mapProvider.h
 *  HOG
 *
 *  Created by Nathan Sturtevant on 5/10/05.
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

#ifndef MAPPROVIDERINTERFACE_H
#define MAPPROVIDERINTERFACE_H

#include "map.h"
#include "mapAbstraction.h"

/**
 * An interface for any class that can provide a map & abstract map, as well as a
 * heuristic for that map.
 */

class mapProvider {
public:
	virtual ~mapProvider() {};
	virtual Map *getMap() = 0;
	virtual mapAbstraction *getMapAbstraction() = 0;
};

#endif
