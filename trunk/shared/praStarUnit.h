/*
 * $Id: praStarUnit.h,v 1.5 2006/09/18 06:19:31 nathanst Exp $
 *
 *  praStarUnit.h
 *  HOG
 *
 *  Created by Nathan Sturtevant on 1/16/05.
 *  Copyright 2005 Nathan Sturtevant. All rights reserved.
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

#include "praStar.h"

#ifndef PRASTARUNIT_H
#define PRASTARUNIT_H

#include "searchUnit.h"
#include "constants.h"

/**
 * A unit which caches path information to speed pra*
 */

class praStarUnit : public searchUnit {
public:
	praStarUnit(int _x, int _y, unit *_target, praStar *_alg);
//	praStarUnit(int _x, int _y, int _r, int _g, int _b, unit *_target, praStar *_alg);
	~praStarUnit() { delete cache; }
	const char *getName() { sprintf(name, "c%s", algorithm->getName()); return name; }
	tDirection makeMove(mapProvider *, reservationProvider *, simulationInfo *simInfo); 
private:
	path *cache;
	praStar *algorithm;
char name[32];
};

#endif
