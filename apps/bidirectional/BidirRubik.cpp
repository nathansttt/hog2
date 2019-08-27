//
//  BidirRubik.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/15/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#include "BidirRubik.h"
#include "RubiksCube.h"
#include "RubiksInstances.h"
#include "TemplateAStar.h"
#include "NBS.h"
#include "MM.h"
#include "BSStar.h"
#include "WeightedVertexGraph.h"

enum heuristicType {
	kNone,
	k444,
	kSmall,
	k888,
	k1997,
	k839,
	k8210,
	kCustom
};

const char *hprefix = "/Users/nathanst/Data/pdb/";

void BuildHeuristics(RubiksState goal, Heuristic<RubiksState> &result, heuristicType h)
{
	RubiksCube cube;
	std::vector<int> blank;
	
	switch (h)
	{
		case kNone:
		{
			ZeroHeuristic<RubiksState> *zero = new ZeroHeuristic<RubiksState>();
			result.lookups.push_back({kLeafNode, 0, 0});
			result.heuristics.push_back(zero);
			break;
		}
		case k444:
		{
			std::vector<int> edges1 = {1, 3, 8, 9}; // first 4
			std::vector<int> edges2 = {0, 2, 4, 5}; // first 4
			std::vector<int> corners = {0, 1, 2, 3}; // first 4
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
			if (!pdb1->Load(hprefix))
			{
				pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb1->Save(hprefix);
			}
			if (!pdb2->Load(hprefix))
			{
				pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb2->Save(hprefix);
			}
			if (!pdb3->Load(hprefix))
			{
				pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb3->Save(hprefix);
			}
			result.lookups.push_back({kMaxNode, 1, 3});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			break;
		}
		case kSmall:
		{
			assert(!"PDB not being saved!");
			std::vector<int> edges1 = {0, 1, 2, 4, 6};
			std::vector<int> edges2 = {3, 5};
			std::vector<int> edges3 = {7, 8, 9, 10, 11};
			std::vector<int> corners1 = {0, 1, 2, 3, 4, 5};
			std::vector<int> corners2 = {2, 3, 4, 5, 6, 7};
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, edges3, blank);
			RubikPDB *pdb4 = new RubikPDB(&cube, goal, blank, corners1);
			RubikPDB *pdb5 = new RubikPDB(&cube, goal, blank, corners2);
			pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
			pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
			pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
			pdb4->BuildPDB(goal, std::thread::hardware_concurrency());
			pdb5->BuildPDB(goal, std::thread::hardware_concurrency());
			result.lookups.push_back({kMaxNode, 1, 5});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.lookups.push_back({kLeafNode, 3, 0});
			result.lookups.push_back({kLeafNode, 4, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			result.heuristics.push_back(pdb4);
			result.heuristics.push_back(pdb5);
			break;
		}
		case k1997:
		{
			std::vector<int> edges1 = {1, 3, 8, 9, 10, 11};
			std::vector<int> edges2 = {0, 2, 4, 5, 6, 7};
			std::vector<int> corners = {0, 1, 2, 3, 4, 5, 6, 7};
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
			
			if (!pdb1->Load(hprefix))
			{
				pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb1->Save(hprefix);
			}
			else {
				printf("Loaded previous heuristic\n");
			}
			if (!pdb2->Load(hprefix))
			{
				pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb2->Save(hprefix);
			}
			else {
				printf("Loaded previous heuristic\n");
			}
			if (!pdb3->Load(hprefix))
			{
				pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb3->Save(hprefix);
			}
			else {
				printf("Loaded previous heuristic\n");
			}
			result.lookups.push_back({kMaxNode, 1, 3});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			break;
		}
		case k888:
		{
			std::vector<int> edges1 = {0, 1, 2, 3, 4, 5, 6, 7};
			std::vector<int> edges2 = {1, 3, 5, 7, 8, 9, 10, 11};
			std::vector<int> corners = {0, 1, 2, 3, 4, 5, 6, 7}; // first 4
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
			if (!pdb1->Load(hprefix))
			{
				pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb1->Save(hprefix);
			}
			if (!pdb2->Load(hprefix))
			{
				pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb2->Save(hprefix);
			}
			if (!pdb3->Load(hprefix))
			{
				pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb3->Save(hprefix);
			}
			result.lookups.push_back({kMaxNode, 1, 3});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			break;
		}
		case k839:
		{
			std::vector<int> edges1 = {0, 1, 2, 3, 4, 5, 6, 7, 8};
			std::vector<int> edges2 = {9, 10, 11};
			std::vector<int> corners = {0, 1, 2, 3, 4, 5, 6, 7};
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
			if (!pdb1->Load(hprefix))
			{
				pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb1->Save(hprefix);
			}
			if (!pdb2->Load(hprefix))
			{
				pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb2->Save(hprefix);
			}
			if (!pdb3->Load(hprefix))
			{
				pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb3->Save(hprefix);
			}
			result.lookups.push_back({kMaxNode, 1, 3});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			break;
		}
		case k8210:
		{
			std::vector<int> edges1 = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
			std::vector<int> edges2 = {10, 11};
			std::vector<int> corners = {0, 1, 2, 3, 4, 5, 6, 7};
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
			if (!pdb1->Load(hprefix))
			{
				pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb1->Save(hprefix);
			}
			if (!pdb2->Load(hprefix))
			{
				pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb2->Save(hprefix);
			}
			if (!pdb3->Load(hprefix))
			{
				pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb3->Save(hprefix);
			}
			result.lookups.push_back({kMaxNode, 1, 3});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			break;
		}
		case kCustom:
		{
			std::vector<int> edges1 = {0, 1, 2, 3};
			std::vector<int> edges2 = {4, 5, 6, 7};
			std::vector<int> edges3 = {8, 9};
			std::vector<int> edges4 = {10, 11};
			std::vector<int> corners = {0, 1, 2, 3, 4, 5, 6, 7};
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
			RubikPDB *pdb4 = new RubikPDB(&cube, goal, blank, corners);
			RubikPDB *pdb5 = new RubikPDB(&cube, goal, blank, corners);
			if (!pdb1->Load(hprefix))
			{
				pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb1->Save(hprefix);
			}
			if (!pdb2->Load(hprefix))
			{
				pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb2->Save(hprefix);
			}
			if (!pdb3->Load(hprefix))
			{
				pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb3->Save(hprefix);
			}
			if (!pdb4->Load(hprefix))
			{
				pdb4->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb4->Save(hprefix);
			}
			if (!pdb5->Load(hprefix))
			{
				pdb5->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb5->Save(hprefix);
			}
			result.lookups.push_back({kMaxNode, 1, 5});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.lookups.push_back({kLeafNode, 3, 0});
			result.lookups.push_back({kLeafNode, 4, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			result.heuristics.push_back(pdb4);
			result.heuristics.push_back(pdb5);
			break;
		}
	}
}

void TestRubik(int algorithm)
{
	const int walkLength = 14;
	NBS<RubiksState, RubiksAction, RubiksCube> nbs;
	MM<RubiksState, RubiksAction, RubiksCube> mm;
	BSStar<RubiksState, RubiksAction, RubiksCube> bs;
	TemplateAStar<RubiksState, RubiksAction, RubiksCube> astar;
	RubiksCube cube;
	RubiksState start, goal;
	
	Heuristic<RubiksState> h_f;
	Heuristic<RubiksState> h_b;

	BuildHeuristics(goal, h_f, kCustom);
	h_b = h_f;
	for (int x = 0; x < h_b.heuristics.size(); x++)
	{
		h_b.heuristics[x] = new RubikArbitraryGoalPDB((RubikPDB*)h_b.heuristics[x]);
	}

	
	
	for (int x = 0; x < 100; x++) // 547 to 540
	{
		printf("Problem %d of %d\n", x+1, 100);
	
		RubiksCubeInstances::GetRandomN(start, walkLength, x);
		goal.Reset();

		std::vector<RubiksState> nbsPath;
		std::vector<RubiksState> astarPath;
		Timer t1, t2;
		
		if (algorithm == -1 || algorithm == 10) // Optimal Analysis
		{
			std::string t = hprefix;
			t += "RC_w"+std::to_string(walkLength)+"_"+std::to_string(x+1)+".svg";
			
			BidirectionalProblemAnalyzer<RubiksState, RubiksAction, RubiksCube> p(start, goal, &cube, &h_f, &h_b);
			p.drawProblemInstance = false;
			p.drawAllG = true;
			p.flipBackwardsGCost = true;
			//			p.drawFullGraph = true;
			//			p.drawMinimumVC = true;
			//			p.drawStatistics = false;
			//			p.SaveSVG((t+"-full.svg").c_str());
			//			p.drawFullGraph = false;
			//			p.drawProblemInstance = false;
			//			p.drawAllG = true;
			// p.drawStatistics = false;
			//			p.SaveSVG((t+"-min.svg").c_str());
			//			printf("Forward: %d\n", p.GetForwardWork());
			//			printf("Backward: %d\n", p.GetBackwardWork());
			//			printf("Minimum: %d\n", p.GetMinWork());
			//			int maxg = p.GetNumGCosts();
			//p.SaveSVG((t+"-shrunk.svg").c_str(), (maxg+11)/12);
			p.SaveSVG(t.c_str());
			
		}
		
		if (algorithm == 0 || algorithm == 10) // A*
		{
			goal.Reset();
			RubiksCubeInstances::GetRandomN(start, walkLength, x);
			t1.StartTimer();
			astar.SetHeuristic(&h_f);
			astar.GetPath(&cube, start, goal, astarPath);
			t1.EndTimer();
			printf("A* found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", cube.GetPathLength(astarPath),
				   astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), astar.GetNodesTouched(), t1.GetElapsedTime());
		}
		if (algorithm == 1) // BS*
		{
			goal.Reset();
			RubiksCubeInstances::GetRandomN(start, walkLength, x);
			t2.StartTimer();
			bs.GetPath(&cube, start, goal, &cube, &cube, nbsPath);
			t2.EndTimer();
			printf("BS* found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", cube.GetPathLength(nbsPath),
				   bs.GetNodesExpanded(), bs.GetNecessaryExpansions(), bs.GetNodesTouched(), t2.GetElapsedTime());
		}
		if (algorithm == 2) // MM
		{
			goal.Reset();
			RubiksCubeInstances::GetRandomN(start, walkLength, x);
			t2.StartTimer();
			mm.GetPath(&cube, start, goal, &h_f, &h_b, nbsPath);
			t2.EndTimer();
			printf("MM found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", cube.GetPathLength(nbsPath),
				   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), mm.GetNodesTouched(), t2.GetElapsedTime());
		}
		if (algorithm == 3||algorithm == 10) // NBS
		{
			goal.Reset();
			RubiksCubeInstances::GetRandomN(start, walkLength, x);
			t2.StartTimer();
			nbs.GetPath(&cube, start, goal, &h_f, &h_b, nbsPath);
			t2.EndTimer();
			printf("NBS found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", cube.GetPathLength(nbsPath),
				   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), nbs.GetNodesTouched(), t2.GetElapsedTime());
		}
		if (algorithm == 4) // MM0
		{
			ZeroHeuristic<RubiksState> z;
			goal.Reset();
			RubiksCubeInstances::GetRandomN(start, walkLength, x);
			t2.StartTimer();
			mm.GetPath(&cube, start, goal, &z, &z, nbsPath);
			t2.EndTimer();
			printf("MM found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", cube.GetPathLength(nbsPath),
				   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), mm.GetNodesTouched(), t2.GetElapsedTime());
		}
		
	}
	exit(0);
}
