//
//  BidirSTP.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/30/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#include "BidirSTP.h"
#include "MNPuzzle.h"
#include "NBS.h"
#include "IDAStar.h"
#include "MM.h"
#include "BSStar.h"
#include "TemplateAStar.h"
#include "WeightedVertexGraph.h"
#include "STPInstances.h"

void TestSTP(int algorithm)
{
	NBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> nbs;
	MM<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> mm;
	BSStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> bs;
	TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> astar;
	MNPuzzle<4,4> mnp;
	
	for (int x = 0; x < 100; x++) // 547 to 540
	{
		
		MNPuzzleState<4, 4> start, goal;
		printf("Problem %d of %d\n", x+1, 100);
		
		std::vector<MNPuzzleState<4,4>> nbsPath;
		std::vector<MNPuzzleState<4,4>> astarPath;
		Timer t1, t2;
		
		if (algorithm == -1) // Optimal Analysis
		{
			start = STP::GetKorfInstance(x);
			goal.Reset();

			std::string t = "/Users/nathanst/bidir/stp/stp_";
			t += std::to_string(x+1);
			
			BidirectionalProblemAnalyzer<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> p(start, goal, &mnp, &mnp, &mnp);
//			p.drawFullGraph = true;
//			p.drawProblemInstance = false;
//			p.drawMinimumVC = true;
//			p.drawAllG = false;
//			p.drawStatistics = false;
//			p.SaveSVG((t+"-full.svg").c_str());
//			p.drawFullGraph = false;
//			p.drawProblemInstance = false;
//			p.drawAllG = true;
//			p.drawStatistics = false;
//			p.SaveSVG((t+"-min.svg").c_str());
//			printf("Forward: %d\n", p.GetForwardWork());
//			printf("Backward: %d\n", p.GetBackwardWork());
//			printf("Minimum: %d\n", p.GetMinWork());
//			int maxg = p.GetNumGCosts();
//			p.SaveSVG((t+"-shrunk.svg").c_str(), (maxg+11)/12);

		}

		if (algorithm == 0) // A*
		{
			goal.Reset();
			start = STP::GetKorfInstance(x);
			t1.StartTimer();
			astar.GetPath(&mnp, start, goal, astarPath);
			t1.EndTimer();
			printf("A* found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(astarPath),
				   astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), astar.GetNodesTouched(), t1.GetElapsedTime());
		}
		if (algorithm == 1) // BS*
		{
			goal.Reset();
			start = STP::GetKorfInstance(x);
			t2.StartTimer();
			bs.GetPath(&mnp, start, goal, &mnp, &mnp, nbsPath);
			t2.EndTimer();
			printf("BS* found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(nbsPath),
				   bs.GetNodesExpanded(), bs.GetNecessaryExpansions(), bs.GetNodesTouched(), t2.GetElapsedTime());
		}
		if (algorithm == 2) // MM
		{
			goal.Reset();
			start = STP::GetKorfInstance(x);
			t2.StartTimer();
			mm.GetPath(&mnp, start, goal, &mnp, &mnp, nbsPath);
			t2.EndTimer();
			printf("MM found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(nbsPath),
				   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), mm.GetNodesTouched(), t2.GetElapsedTime());
		}
		if (algorithm == 3) // NBS
		{
			goal.Reset();
			start = STP::GetKorfInstance(x);
			t2.StartTimer();
			nbs.GetPath(&mnp, start, goal, &mnp, &mnp, nbsPath);
			t2.EndTimer();
			printf("NBS found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(nbsPath),
				   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), nbs.GetNodesTouched(), t2.GetElapsedTime());
		}
		if (algorithm == 4) // MM0
		{
			ZeroHeuristic<MNPuzzleState<4,4>> z;
			goal.Reset();
			start = STP::GetKorfInstance(x);
			t2.StartTimer();
			mm.GetPath(&mnp, start, goal, &z, &z, nbsPath);
			t2.EndTimer();
			printf("MM found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(nbsPath),
				   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), mm.GetNodesTouched(), t2.GetElapsedTime());
		}
		
//		
//		std::cout << astar.GetNodesExpanded() << "\t" << nbs.GetNodesExpanded() << "\t";
//		std::cout << t1.GetElapsedTime() << "\t" <<  t2.GetElapsedTime() << "\n";
	}
	exit(0);
}

void TestSTPFull()
{
	NBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> nbs;
	MM<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> mm;
	MNPuzzle<4,4> mnp;
	IDAStar<MNPuzzleState<4,4>, slideDir> ida;
	TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> astar;

	for (int x = 0; x < 100; x++) // 547 to 540
	{
		MNPuzzleState<4, 4> start, goal;
		printf("Problem %d of %d\n", x+1, 100);
		
		std::vector<slideDir> idaPath;
		std::vector<MNPuzzleState<4,4>> nbsPath;
		std::vector<MNPuzzleState<4,4>> astarPath;
		std::vector<MNPuzzleState<4,4>> mmPath;
		Timer t1, t2, t3, t4;
		
		goal.Reset();
		start = STP::GetKorfInstance(x);
		t1.StartTimer();
		ida.GetPath(&mnp, start, goal, idaPath);
		t1.EndTimer();
		printf("IDA* found path length %ld; %llu expanded; %1.2fs elapsed\n", idaPath.size(),  ida.GetNodesExpanded(), t1.GetElapsedTime());

		goal.Reset();
		start = STP::GetKorfInstance(x);
		t2.StartTimer();
		astar.GetPath(&mnp, start, goal, astarPath);
		t2.EndTimer();
		printf("A* found path length %ld; %llu expanded; %1.2fs elapsed\n", astarPath.size()-1,  astar.GetNodesExpanded(), t2.GetElapsedTime());

		goal.Reset();
		start = STP::GetKorfInstance(x);
		t3.StartTimer();
		nbs.GetPath(&mnp, start, goal, &mnp, &mnp, nbsPath);
		t3.EndTimer();
		printf("NBS found path length %ld; %llu expanded; %1.2fs elapsed\n", nbsPath.size()-1,  nbs.GetNodesExpanded(), t3.GetElapsedTime());

		goal.Reset();
		start = STP::GetKorfInstance(x);
		t4.StartTimer();
		mm.GetPath(&mnp, start, goal, &mnp, &mnp, mmPath);
		t4.EndTimer();
		printf("MM found path length %ld; %llu expanded; %1.2fs elapsed\n", mmPath.size()-1,  mm.GetNodesExpanded(), t3.GetElapsedTime());


		std::cout << ida.GetNodesExpanded() << "\t" <<  astar.GetNodesExpanded() << "\t" << nbs.GetNodesExpanded() << "\t";
		std::cout << t1.GetElapsedTime() << "\t" <<  t2.GetElapsedTime() << "\t" << t3.GetElapsedTime() << "\n";
		
		//if (!fequal)
		if (nbsPath.size() != idaPath.size()+1)
		{
			std::cout << "error solution cost:\t expected cost\n";
			std::cout << nbsPath.size() << "\t" << idaPath.size() << "\n";
//			double d;
//			for (auto x : correctPath)
//			{
//				astar.GetClosedListGCost(x, d);
//				auto t = nbs.GetNodeForwardLocation(x);
//				auto u = nbs.GetNodeBackwardLocation(x);
//				std::cout << x << " is on " << t << " and " << u << "\n";
//				std::cout << "True g: " << d;
//				if (t != kUnseen)
//					std::cout << " forward g: " << nbs.GetNodeForwardG(x);
//				if (u != kUnseen)
//					std::cout << " backward g: " << nbs.GetNodeBackwardG(x);
//				std::cout << "\n";
//			}
			exit(0);
		}
		
	}
	exit(0);
}
