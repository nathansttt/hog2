//
//  BidirPancake.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/7/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#include "BidirPancake.h"
#include "PancakePuzzle.h"
#include "TemplateAStar.h"
#include "NBS.h"
#include "IDAStar.h"
#include "MM.h"
#include "BSStar.h"

const int S = 10; // must be factor of sizes below

void TestPancakeTR();
void TestPancakeRandom();

void TestPancake()
{
	TestPancakeRandom();
}

void TestPancakeTR()
{
	// multiples of 5
	int arrangement[] = {0,2,4,1,3,5,7,9,6,8,10,12,14,11,13,15,17,19,16,18,20,22,24,21,23,25,27,29,26,28,30,32,34,31,33,35,37,39,36,38,40,42,44,41,43,45,47,49,46,48,50,52,54,51,53,55,57,59,56,58,60,62,64,61,63,65,67,69,66,68,70,72,74,71,73,75,77,79,76,78,80,82,84,81,83,85,87,89,86,88,90,92,94,91,93,95,97,99,96,98,};
	// multiples of 9
//	const int arrangement[] = {0,4,7,2,5,8,3,6,1,9,13,16,11,14,17,12,15,10,18,22,25,20,23,26,21,24,19,27,31,34,29,32,35,30,33,28,36,40,43,38,41,44,39,42,37,45,49,52,47,50,53,48,51,46,54,58,61,56,59,62,57,60,55,63,67,70,65,68,71,66,69,64,72,76,79,74,77,80,75,78,73,81,85,88,83,86,89,84,87,82,90,94,97,92,95,98,93,96,91};

	for (int gap = 0; gap < 10; gap++)
	{
		
		PancakePuzzleState<S> start;
		PancakePuzzleState<S> goal;
		PancakePuzzle<S> pancake(gap);
		PancakePuzzle<S> pancake2(gap);
		
		NBS<PancakePuzzleState<S>, PancakePuzzleAction, PancakePuzzle<S>> nbs;
		MM<PancakePuzzleState<S>, PancakePuzzleAction, PancakePuzzle<S>> mm;
		TemplateAStar<PancakePuzzleState<S>, PancakePuzzleAction, PancakePuzzle<S>> astar;
		IDAStar<PancakePuzzleState<S>, PancakePuzzleAction, false> idastar;
		
		std::vector<PancakePuzzleState<S>> nbsPath;
		std::vector<PancakePuzzleState<S>> astarPath;
		std::vector<PancakePuzzleState<S>> mmPath;
		std::vector<PancakePuzzleAction> idaPath;
		Timer t1, t2, t3, t4;
		
		
		goal.Reset();
		for (int x = 0; x < S; x++)
			start.puzzle[x] = arrangement[x];
		t1.StartTimer();
		astar.GetPath(&pancake, start, goal, astarPath);
		t1.EndTimer();
		uint64_t necessary = 0;
		double solutionCost = pancake.GetPathLength(astarPath);
		for (unsigned int x = 0; x < astar.GetNumItems(); x++)
		{
			const auto &item = astar.GetItem(x);
			if ((item.where == kClosedList) && (item.g+item.h < solutionCost))
				necessary++;
		}
		printf("A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", pancake.GetPathLength(astarPath),
			   astar.GetNodesExpanded(), necessary, t1.GetElapsedTime());
		
		goal.Reset();
		for (int x = 0; x < S; x++)
			start.puzzle[x] = arrangement[x];
		t2.StartTimer();
		nbs.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsPath);
		t2.EndTimer();
		printf("NBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", pancake.GetPathLength(nbsPath),
			   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t2.GetElapsedTime());
		
		goal.Reset();
		for (int x = 0; x < S; x++)
			start.puzzle[x] = arrangement[x];
		t3.StartTimer();
		idastar.GetPath(&pancake, start, goal, idaPath);
		t3.EndTimer();
		printf("IDA* found path length %ld; %llu expanded; %llu generated; %1.2fs elapsed\n", idaPath.size(),
			   idastar.GetNodesExpanded(), idastar.GetNodesTouched(), t3.GetElapsedTime());
		
		
		goal.Reset();
		for (int x = 0; x < S; x++)
			start.puzzle[x] = arrangement[x];
		t4.StartTimer();
		mm.GetPath(&pancake, start, goal, &pancake, &pancake2, mmPath);
		t4.EndTimer();
		printf("MM found path length %1.0f; %llu expanded; %1.2fs elapsed\n", pancake.GetPathLength(mmPath),
			   mm.GetNodesExpanded(), t4.GetElapsedTime());
		
		printf("Problem & IDA* & & A* & & MM & & NBS* & \\\\\n");
		printf("%d G-%d & %llu & %1.2fs & %llu & %1.2fs & %llu & %1.2fs & %llu & %1.2fs \\\\ \n", S, gap,
			   idastar.GetNodesExpanded(), t3.GetElapsedTime(),
			   astar.GetNodesExpanded(), t1.GetElapsedTime(),
			   mm.GetNodesExpanded(), t4.GetElapsedTime(),
			   nbs.GetNodesExpanded(), t2.GetElapsedTime());
	}
	exit(0);
}

const int N = 16;
void TestPancakeRandom()
{
	for (int gap = 0; gap < 4; gap++)
	{
		srandom(2017218);
		PancakePuzzleState<N> start;
		PancakePuzzleState<N> original;
		PancakePuzzleState<N> goal;
		PancakePuzzle<N> pancake(gap);
		PancakePuzzle<N> pancake2(gap);
		
		
		std::vector<PancakePuzzleState<N>> nbsPath;
		std::vector<PancakePuzzleState<N>> bsPath;
		std::vector<PancakePuzzleState<N>> astarPath;
		std::vector<PancakePuzzleState<N>> mmPath;
		std::vector<PancakePuzzleAction> idaPath;
		Timer t1, t2, t3, t4, t5;
		
		for (int count = 0; count < 50; count++)
		{
			srandom(random());
			
			goal.Reset();
			original.Reset();
			for (int x = 0; x < N; x++)
				std::swap(original.puzzle[x], original.puzzle[x+random()%(N-x)]);
			
			printf("Problem %d of %d\n", count+1, 50);
			std::cout << original << "\n";
			
			// A*
			{
				TemplateAStar<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> astar;
				start = original;
				t1.StartTimer();
				astar.GetPath(&pancake, start, goal, astarPath);
				t1.EndTimer();
				printf("GAP-%d A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap, pancake.GetPathLength(astarPath),
					   astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
			}
			
			// NBS
			{
				NBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> nbs;
				goal.Reset();
				start = original;
				t2.StartTimer();
				nbs.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsPath);
				t2.EndTimer();
				printf("GAP-%d NBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap, pancake.GetPathLength(nbsPath),
					   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t2.GetElapsedTime());
			}
			
			// BS*
			{
				BSStar<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> bs;
				goal.Reset();
				start = original;
				t2.StartTimer();
				bs.GetPath(&pancake, start, goal, &pancake, &pancake2, bsPath);
				t2.EndTimer();
				printf("GAP-%d BS* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap, pancake.GetPathLength(bsPath),
					   bs.GetNodesExpanded(), bs.GetNecessaryExpansions(), t2.GetElapsedTime());
			}
			
			// IDA*
			{
				IDAStar<PancakePuzzleState<N>, PancakePuzzleAction, false> idastar;
				goal.Reset();
				start = original;
				t3.StartTimer();
				idastar.GetPath(&pancake, start, goal, idaPath);
				t3.EndTimer();
				printf("GAP-%d IDA* found path length %ld; %llu expanded; %llu generated; %1.2fs elapsed\n", gap, idaPath.size(),
					   idastar.GetNodesExpanded(), idastar.GetNodesTouched(), t3.GetElapsedTime());
			}
			
			// MM
			{
				MM<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> mm;
				goal.Reset();
				start = original;
				t4.StartTimer();
				mm.GetPath(&pancake, start, goal, &pancake, &pancake2, mmPath);
				t4.EndTimer();
				printf("GAP-%d MM found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap, pancake.GetPathLength(mmPath),
					   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), t1.GetElapsedTime());
			}

			// MM0
			if (gap == 3)
			{
				MM<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> mm;
				ZeroHeuristic<PancakePuzzleState<N>> z;
				goal.Reset();
				start = original;
				t4.StartTimer();
				mm.GetPath(&pancake, start, goal, &z, &z, mmPath);
				t4.EndTimer();
				printf("GAP-%d MM0 found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake.GetPathLength(mmPath),
					   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), t1.GetElapsedTime());
			}

		}
	}

}
