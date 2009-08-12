/*
 * $Id: path.cpp,v 1.2 2006/11/01 19:00:06 nathanst Exp $
 *
 *  Hierarchical Open Graph File
 *
 *  Created by Vadim Bulitko on 11/16/04.
 *  Copyright 2004 Vadim Bulitko. All rights reserved.
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

#include "Path.h"

// Returns the length of the path -- number of steps
unsigned path::length()
{
	return (n == NULL) ? 0 : ((next == NULL) ? 1 : (1+next->length()));
}

// Return the cummulative distance along a path
//double path::distance(GraphAbstraction* aMap)
//{
//	// check of the path is empty or has only one node
//	if ((n == NULL) || (next == NULL))
//		return 0.0;
//
//	// Otherwise, iterate through the path
//	return aMap->h(n,next->n) + next->distance(aMap);
//}

// Return the neighbor complexity of a path
// That is the cummulative degree of all vertices except the last one
unsigned path::degree()
{
	// check of the path is empty or has only one node
	if ((n == NULL) || (next == NULL))
		return 0;
	
	// Otherwise, iterate through the rest of the path
	return n->GetNumEdges() + next->degree();
}

// Print the path
void path::Print(bool beginning)
{
	if (beginning)
		printf("[");
	
	if (n != NULL)
		printf("%d",n->GetNum());
	else
		printf("NULL");
	
	if (next != NULL) {
		printf(",");
		next->Print(false);
	}
	else
		printf("]");
}

path *path::reverse()
{
	if (next == 0)
		return this;
	path *tmp = next->reverse();
	next->next = this;
	next = 0;
	return tmp;
}
