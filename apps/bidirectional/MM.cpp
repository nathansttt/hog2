//
//  MM.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/31/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//

#include "MM.h"
#include "IDAStar.h"
#include "Timer.h"
#include <string>

NAMESPACE_OPEN(MM)

//const int fileBuckets = 8; // must be at least 8
const uint64_t bucketBits = 3;
const int bucketMask = 0x7;

int bestSolution;
int currentC;
int minGForward;
int minGBackward;
int minFForward;
int minFBackward;
uint64_t expanded;
std::vector<uint64_t> gDistForward;
std::vector<uint64_t> gDistBackward;

void GetBucketAndData(const RubiksState &s, int &bucket, uint64_t &data);
void GetState(RubiksState &s, int bucket, uint64_t data);
int GetBucket(const RubiksState &s);

void BuildHeuristics(RubiksState start, RubiksState goal, Heuristic<RubiksState> &result);
RubiksCube cube;

Heuristic<RubiksState> forward;
Heuristic<RubiksState> reverse;

enum  tSearchDirection {
	kForward,
	kBackward,
};

// This tells us the open list buckets
struct openData {
	tSearchDirection dir;  // at most 2 bits
	uint8_t priority;      // at most 6 bits
	uint8_t gcost;         // at most 4 bits
	uint8_t hcost;         // at most 4 bits
	uint8_t bucket;        // at most (3) bits
};

static bool operator==(const openData &a, const openData &b)
{
	return (a.dir == b.dir && a.priority == b.priority && a.gcost == b.gcost && a.hcost == b.hcost && a.bucket == b.bucket);
}

static std::ostream &operator<<(std::ostream &out, const openData &d)
{
	out << "[" << ((d.dir==kForward)?"forward":"backward") << ", p:" << +d.priority << ", g:" << +d.gcost << ", h:" << +d.hcost;
	out << ", b:" << +d.bucket << ")";
	return out;
}

struct openDataHash
{
	std::size_t operator()(const openData & x) const
	{
		return (x.dir)|(x.priority<<2)|(x.gcost<<8)|(x.hcost<<12)|(x.bucket<<16);
	}
};

struct openList {
	//openData data;
	std::unordered_map<RubiksState, bool> states;
};

std::unordered_map<RubiksState, bool> closed;
std::unordered_map<openData, openList, openDataHash> open;

std::string GetOpenName(const openData &d)
{
	std::string s;
	if (d.dir == kForward)
		s += "forward-";
	else
		s += "backward-";
	s += std::to_string(d.priority);
	s += "-";
	s += std::to_string(d.gcost);
	s += "-";
	s += std::to_string(d.hcost);
	s += "-";
	s += std::to_string(d.bucket);
	s += ".open";
	return s;
}

openData GetBestFile()
{
	minGForward = 100;
	minGBackward = 100;
	minFForward = 100;
	minFBackward = 100;
	// actually do priority here
	openData best =(open.begin())->first;
	//return (open.begin())->first;
	for (const auto &s : open)
	{
		if (s.first.dir == kForward && s.first.gcost < minGForward)
			minGForward = s.first.gcost;
		else if (s.first.dir == kBackward && s.first.gcost < minGBackward)
			minGBackward = s.first.gcost;
		
		if (s.first.dir == kForward && s.first.gcost+s.first.hcost < minFForward)
			minFForward = s.first.gcost+s.first.hcost;
		else if (s.first.dir == kBackward && s.first.gcost+s.first.hcost < minFBackward)
			minFBackward = s.first.gcost+s.first.hcost;

		if (s.first.priority < best.priority)
			best = s.first;
		else if (s.first.priority == best.priority)
		{
			if (s.first.gcost < best.gcost)
				best = s.first;
			else if (s.first.gcost == best.gcost && s.first.dir == kForward)
				best = s.first;
		}
	}
	return best;
}

void AddStateToQueue(const RubiksState &start, tSearchDirection dir, int cost)
{
	int bucket;
	uint64_t data;
	GetBucketAndData(start, bucket, data);
	openData d;
	d.dir = dir;
	d.gcost = cost;
	d.hcost = (dir==kForward)?forward.HCost(start, start):reverse.HCost(start, start);
	d.bucket = bucket;
	d.priority = std::max(d.gcost+d.hcost, d.gcost*2);

	// Add to open list
	open[d].states[start] = true;

	// Check for reversed duplicates
	for (const auto &d : open)
	{
		if (d.first.dir != dir && d.second.states.find(start) != d.second.states.end())
		{
			printf("Found solution cost %d+%d=%d\n", cost, d.first.gcost, cost + d.first.gcost);
			bestSolution = std::min(cost + d.first.gcost, bestSolution);
			printf("Current best solution: %d\n", bestSolution);
		}
	}
	
//	FILE *f = fopen(GetOpenName(d).c_str(), "a");
//	assert(f != 0);
//	fwrite(&data, 1, sizeof(uint64_t), f);
//	fclose(f);
	
	// TODO: Add d to open list (if not already present)
	
}

void ReadFile(const openData &d, std::unordered_map<RubiksState, bool> &map)
{
	// 1. open file for d;
	FILE *f = fopen(GetOpenName(d).c_str(), "r");
	assert(f != 0);
	uint64_t data;
	RubiksState s;
	while (fread(&data, 1, sizeof(uint64_t), f) == 1)
	{
		GetState(s, d.bucket, data);
		map[s] = true;
	}
	fclose(f);
	// keep file for duplicate detection purposes
	//remove(GetOpenName(d).c_str());
}

void ExpandNextFile()
{
	// 1. Get next expansion target
	openData d = GetBestFile();
	std::cout << "Next: " << d << "\n";
	currentC = d.priority;
	
	if (bestSolution <= std::max(currentC, std::max(minFForward, std::max(minFBackward, minGBackward+minGForward+1))))
	{
		printf("Done!\n");
		printf("%llu nodes expanded\n", expanded);
		printf("Forward Distribution:\n");
		for (int x = 0; x < gDistForward.size(); x++)
			if (gDistForward[x] != 0)
				printf("%d\t%llu\n", x, gDistForward[x]);
		printf("Backward Distribution:\n");
		for (int x = 0; x < gDistBackward.size(); x++)
			if (gDistBackward[x] != 0)
				printf("%d\t%llu\n", x, gDistBackward[x]);
		exit(0);
	}
	
	// 3. expand all states & write out successors
	RubiksState tmp;
	for (auto &values : open[d].states)
	{
		expanded++;
		if (d.dir == kForward)
			gDistForward[d.gcost]++;
		else
			gDistBackward[d.gcost]++;
		for (int x = 0; x < 18; x++)
		{
			// copying 2 64-bit values is faster than undoing a move
			tmp = values.first;
			cube.ApplyAction(tmp, x);
			if (closed.find(tmp) == closed.end())
			{
				AddStateToQueue(tmp, d.dir, d.gcost+1);
			}
		}

		// 4. do possible check for solution
		closed[values.first] = true;
	}
	open.erase(open.find(d));
	
}

void BuildHeuristics(RubiksState start, RubiksState goal, Heuristic<RubiksState> &result)
{
	RubiksCube cube;
	std::vector<int> edges1 = {1, 3, 8, 9, 10, 11}; // first 4
	std::vector<int> edges2 = {0, 2, 4, 5, 6, 7}; // first 4
	std::vector<int> corners = {0, 1, 2, 3, 4, 5, 6, 7}; // first 4
	std::vector<int> blank;
	RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
	RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
	RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
	pdb1->BuildPDB(goal, 0, std::thread::hardware_concurrency());
	pdb2->BuildPDB(goal, 0, std::thread::hardware_concurrency());
	pdb3->BuildPDB(goal, 0, std::thread::hardware_concurrency());
	result.lookups.push_back({kMaxNode, 1, 3});
	result.lookups.push_back({kLeafNode, 0, 0});
	result.lookups.push_back({kLeafNode, 1, 0});
	result.lookups.push_back({kLeafNode, 2, 0});
	result.heuristics.push_back(pdb1);
	result.heuristics.push_back(pdb2);
	result.heuristics.push_back(pdb3);
}

int GetBucket(const RubiksState &s)
{
	uint64_t ehash = RubikEdgePDB::GetStateHash(s.edge);
	return ehash&bucketMask;
}

void GetBucketAndData(const RubiksState &s, int &bucket, uint64_t &data)
{
	uint64_t ehash = RubikEdgePDB::GetStateHash(s.edge);
	uint64_t chash = RubikCornerPDB::GetStateHash(s.corner);
	bucket = ehash&bucketMask;
	data = (ehash>>bucketBits)*RubikCornerPDB::GetStateSpaceSize()+chash;
}

void GetState(RubiksState &s, int bucket, uint64_t data)
{
	RubikCornerPDB::GetStateFromHash(s.corner, data%RubikCornerPDB::GetStateSpaceSize());
	RubikEdgePDB::GetStateFromHash(s.edge, bucket|((data/RubikCornerPDB::GetStateSpaceSize())<<bucketBits));
}

#pragma mark Main Code

void MM(RubiksState &start, RubiksState &goal)
{
	bestSolution = 100;
	gDistBackward.resize(12);
	gDistForward.resize(12);
	expanded = 0;
	BuildHeuristics(start, goal, forward);
	BuildHeuristics(goal, start, reverse);

//	printf("---IDA*---\n");
//	std::vector<RubiksAction> path;
//	Timer t;
//	t.StartTimer();
//	cube.SetPruneSuccessors(true);
//	IDAStar<RubiksState, RubiksAction> ida;
//	ida.SetHeuristic(&forward);
//	ida.GetPath(&cube, start, goal, path);
//	t.EndTimer();
//	printf("%1.5fs elapsed\n", t.GetElapsedTime());
//	printf("%llu nodes expanded (%1.3f nodes/sec)\n", ida.GetNodesExpanded(),
//		   ida.GetNodesExpanded()/t.GetElapsedTime());
//	printf("%llu nodes generated (%1.3f nodes/sec)\n", ida.GetNodesTouched(),
//		   ida.GetNodesTouched()/t.GetElapsedTime());
//	cube.SetPruneSuccessors(false);
//	printf("Solution cost: %d\n", path.size());
	
	printf("---MM*---\n");
	AddStateToQueue(start, kForward, 0);
	AddStateToQueue(goal, kBackward, 0);
	
	while (!open.empty())
	{
		ExpandNextFile();
	}
}

NAMESPACE_CLOSE(MM)