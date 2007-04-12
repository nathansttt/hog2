/*
 * $Id: statUtil.h,v 1.5 2006/11/29 01:22:40 bulitko Exp $
 *
 *  statUtil.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 11/9/05.
 *  Copyright 2005 University of Alberta. All rights reserved.
 *
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
 */

#include "statCollection.h"

void setupAverageRatio(statCollection *stats, char *stat1, char *stat2);
void measureAverageRatio(statCollection *stats);

double sumStatEntries(statCollection *stats, const char *category, const char *owner);
double averageStatEntries(statCollection *stats, const char *category, const char *owner);
double stdevStatEntries(statCollection *stats, const char *category, const char *owner);
double maxStatEntries(statCollection *stats, const char *category, const char *owner);
long unsigned countStatEntries(statCollection *stats, const char *category, const char *owner);
