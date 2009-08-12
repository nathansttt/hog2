/*
 * $Id: SearchAlgorithm.cpp,v 1.5 2006/10/24 18:18:45 nathanst Exp $
 *
 *  Hierarchical Open Graph File
 *
 *  Created by Nathan Sturtevant on 9/28/04.
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

//#ifdef OS_MAC
//#include <CoreServices/CoreServices.h>
//#endif

#include "SearchAlgorithm.h"
#include <sys/time.h>
#include <sys/resource.h>
#include <stdint.h>

using namespace GraphAbstractionConstants;
using namespace std;

static const int verbose = 0;

void DoRandomPath(GraphAbstraction *aMap, SearchAlgorithm *sa, bool repeat)
{
	static double lastLength, lastTime;
	static node *r1 = 0, *r2 = 0;
	Graph *g = aMap->GetAbstractGraph(0);
	//if (verbose) cout << "Clearing marked nodes" << endl;
	//aMap->ClearMarkedNodes();
	
	//	r1 = ((MapAbstraction*)aMap)->GetNodeFromMap(257, 449);
	//	r2 = ((MapAbstraction*)aMap)->GetNodeFromMap(319, 458);
	if ((!repeat) || (r1 == 0) || (r2 == 0))
	{
		lastLength = 0;
		lastTime = 1;
		do {
			//      do {
			r1 = g->GetRandomNode();
			//      } while (aMap->GetMap()->GetTerrainType((long)r1->GetLabelL(kFirstData), (long)r1->GetLabelL(kFirstData+1)) == kOutOfBounds);
			//      do {
			r2 = g->GetRandomNode();
			//      } while (aMap->GetMap()->GetTerrainType((long)r2->GetLabelL(kFirstData), (long)r2->GetLabelL(kFirstData+1)) == kOutOfBounds);
		} while (!aMap->Pathable(r1, r2));
	}
	
	if (verbose)
	{
		cout << "Attempting path between nodes:" << endl;
		cout << (*r1) << endl << (*r2) << endl;
	}
	
	//	while (r1 != r2)
	//	{
	//		r1 = getPathStep(r1, r2);
	//		if (verbose)
	//			cout << "Stepping to " << (*r1) << endl;
	//	}
	
	// ignoring return value! Leaking memory!
	
#ifdef OS_MAC
	AbsoluteTime startTime = UpTime();
#else
	clock_t startTime, endTime;
	long double duration;
	startTime = clock();
	
#endif
	
	
	path *p;
	
	p = sa->GetPath(aMap, r1, r2);
	//	if (optimal)
	//		//p = getLibraryPath(r1, r2);
	//		p = GetPath(r1, r2);
	//	else
	//		//p = getLibraryPath(r1, r2);
	//		p = getApproximatePath(r1, r2);
	//	//		while (r1 != r2)
	//	//			r1 = getPathStep(r1, r2);
	
	
#ifdef OS_MAC
	AbsoluteTime stopTime = UpTime();
	Nanoseconds diff = AbsoluteDeltaToNanoseconds(stopTime, startTime);
	uint64_t nanosecs = UnsignedWideToUInt64(diff);
	//cout << nanosecs << " ns elapsed (" << (double)nanosecs/1000000.0 << " ms)" << endl;
#else
	endTime = clock();
	duration=(long double)(endTime-startTime)/CLOCKS_PER_SEC;
	//cout << duration << " seconds elapsed" << endl;
#endif
	
	int cnt = 0;
	double length = 0;
	for (path *q = p; q; q = q->next)
	{
		if (q && q->next)
		{
			double t1, t2;
			t1 = q->n->GetLabelL(kFirstData)-q->next->n->GetLabelL(kFirstData);
			t2 = q->n->GetLabelL(kFirstData+1)-q->next->n->GetLabelL(kFirstData+1);
			length += sqrt(t1*t1+t2*t2);
		}
		cnt++;
	}
	
#ifdef OS_MAC
	cout << "Steps: " << cnt << ", len: " << length << ", time: " << (double)nanosecs/1000000.0
		;//<< ",  time/step: " << (double)nanosecs/(1000*cnt) << ", time/unit: " << (double)nanosecs/(1000*length);
		cout << "ms, h() = " << aMap->h(r1, r2) << ", nodes: " << sa->GetNodesExpanded() << endl;
		
		//cout << "DATA\t" << nanosecs << "\t" << length << endl;
		if (!repeat)
		{
			lastLength = length;
			lastTime = (double)nanosecs;
		} else {
			cout << "Comparison: " << lastLength/length << "x longer; but " << (double)nanosecs/lastTime << "x faster." << endl;
		}
#endif	
		
		//aMap->clearDisplayLists();
}
