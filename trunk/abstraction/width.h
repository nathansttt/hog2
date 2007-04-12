/**
 * $Id: width.h,v 1.2 2006/09/18 06:22:14 nathanst Exp $
 *
 * HOG File
 *
 * Helper functions for determining edge widths and node widths.
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

#ifndef WIDTH_H
#define WIDTH_H

#include "graph.h"

// THESE MACROS ARE EVIL (side-effects)
//#define MIN(A,B) (((A) < (B)) ? (A) : (B))
//#define MAX(A,B) (((A) > (B)) ? (A) : (B))

node* findNodeAt(int x, int y, graph* g);
edge* findEdgeBetween(int from, int to, graph* g);
float findMin(node* n);
float findMax(node* n);
float findMaxAbstracted(node* x, graph* g);
float findMinAbstractedNode(node* x, graph* g);
float minSpanningTree(node* n, graph* g);
bool edgeInVector(edge* e, std::vector<edge*> edges);
void sortEdgeWidths(std::vector<edge*>* edges);
bool hasCycle(edge* e, std::vector<int> nodes);

#endif
