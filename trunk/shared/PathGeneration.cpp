/*
 * $Id: pathGeneration.cpp,v 1.4 2006/10/24 18:18:31 nathanst Exp $
 *
 *  pathGeneration.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 11/30/05.
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

#include "pathGeneration.h"
#include "map.h"
#include "aStar.h"
#include "mapFlatAbstraction.h"

// outputs algorithm/path length/time per step
void generatePaths(char *_map, int mapSizeX, int mapSizeY, int numBuckets, int bucketSize, int pathsPerBucket)
{
//	int numBuckets = (int)(mapSize/4.0);
//	const int pathsPerBucket = 10;
	
	int pathsLeft = numBuckets*pathsPerBucket;
	int iterations = 0;
	int maxIterations = 10*numBuckets*pathsPerBucket;
	Map *map = new Map(_map);
	if ((mapSizeX != -1) && (mapSizeY != -1))
		map->scale(mapSizeX, mapSizeY);
	else {
		mapSizeX = map->getMapWidth();
		mapSizeY = map->getMapHeight();
	}
		
	mapFlatAbstraction *absMap = new mapFlatAbstraction(map);
	graph *g = absMap->getAbstractGraph(0);

	node *r1, *r2;
	//  10 maps/bigMaps/600.map  48 151  55 152 7.4142135624
	// bucket, map, sizex, sizey, startx, starty, endx, endy, a* length
	printf("# bucket\tmap\tsizex\tsizey\tfromx\tfromy\ttox\ttoy\tA*len\n");
	std::vector<int> buckets(numBuckets);
	for (int x = 0; x < numBuckets; x++)
		buckets[x] = 0;
	while ((pathsLeft > 0) && (iterations < maxIterations))
	{
		iterations++;
		// try to find two valid points
		do {
			r1 = g->getRandomNode();
			r2 = g->getRandomNode();
		} while (!absMap->pathable(r1, r2) || (r1 == r2) ||
						 ((iterations > maxIterations/2) && (absMap->h(r1, r2) > numBuckets*bucketSize)));
		
		aStar a;

		path *p;
		p = a.getPath(absMap, r1, r2);
		
		int cnt = 0;
		double length = 0;
		for (path *q = p; q; q = q->next)
		{
			if (q && q->next)
			{
				double t1, t2;
				t1 = q->n->getLabelL(kFirstData)-q->next->n->getLabelL(kFirstData);
				t2 = q->n->getLabelL(kFirstData+1)-q->next->n->getLabelL(kFirstData+1);
				length += sqrt(t1*t1+t2*t2);
			}
			cnt++;
		}
		if ((length > numBuckets*bucketSize) ||
				((iterations > maxIterations/2) && (buckets[(int)length/bucketSize] == pathsPerBucket)))
		{
			while (p && p->next)
			{
				double t1, t2;
				t1 = p->n->getLabelL(kFirstData)-p->next->n->getLabelL(kFirstData);
				t2 = p->n->getLabelL(kFirstData+1)-p->next->n->getLabelL(kFirstData+1);
				length -= sqrt(t1*t1+t2*t2);
				path *t = p;
				p = p->next;
				t->next = 0; delete t;
				if (p->next == 0)
				{
					delete p;
					p = 0;
				}
				if (p == 0)
					break;
				if (buckets[(int)length/numBuckets] < pathsPerBucket)
					break;
			}
			if (p && p->next)
			{
				r1 = p->n;
				for (path *q = p; q; q = q->next)
				{
					r2 = q->n;
				}
				delete p;

				p = a.getPath(absMap, r1, r2);

				length = 0; cnt = 0;
				for (path *q = p; q; q = q->next)
				{
					if (q && q->next)
					{
						double t1, t2;
						t1 = q->n->getLabelL(kFirstData)-q->next->n->getLabelL(kFirstData);
						t2 = q->n->getLabelL(kFirstData+1)-q->next->n->getLabelL(kFirstData+1);
						length += sqrt(t1*t1+t2*t2);
					}
					cnt++;
				}
			}
		}
		if (p == 0)
			continue;
		if ((length/bucketSize < numBuckets) && (buckets[(int)length/bucketSize] < pathsPerBucket))
		{
			buckets[(int)length/bucketSize]++;
			pathsLeft--;
		}
		else {
			continue;
		}

		int x1, x2, y1, y2;
		x1 = r1->getLabelL(kFirstData); y1 = r1->getLabelL(kFirstData+1);
		x2 = r2->getLabelL(kFirstData); y2 = r2->getLabelL(kFirstData+1);
		printf("%d\t%s\t%d\t%d\t%d\t%d\t%d\t%d\t%1.2f\n",
					 (int)(length/bucketSize), _map, mapSizeX, mapSizeY, x1, y1, x2, y2, length);
		delete p;
	}
	delete absMap;
}
