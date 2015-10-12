//
//  MM.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/31/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//

#include "MMPancake.h"
#include "TemplateAStar.h"
#include "Timer.h"
#include "PermutationPDB.h"
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <iomanip>

NAMESPACE_OPEN(MMPancake)

//const int fileBuckets = 8; // must be at least 8
//const uint64_t bucketBits = 3;
//const int bucketMask = 0x7;
const int fileBuckets = 32; // must be at least 8
const uint64_t bucketBits = 5;
const int bucketMask = 0x1F;

bool finished = false;

int bestSolution;
int currentC;
int minGForward;
int minGBackward;
int minFForward;
int minFBackward;
uint64_t expanded;
std::vector<uint64_t> gDistForward;
std::vector<uint64_t> gDistBackward;
std::mutex printLock;
std::mutex countLock;
std::mutex openLock;

void GetBucketAndData(const PancakePuzzleState &s, int &bucket, uint64_t &data);
void GetState(PancakePuzzleState &s, int bucket, uint64_t data);
int GetBucket(const PancakePuzzleState &s);

void BuildHeuristics(int count, PancakePuzzleState start, PancakePuzzleState goal, Heuristic<PancakePuzzleState> &result);
PancakePuzzle pancake(10);

Heuristic<PancakePuzzleState> forward;
Heuristic<PancakePuzzleState> reverse;

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
	out << ", b:" << +d.bucket << "]";
	return out;
}

struct openDataHash
{
	std::size_t operator()(const openData & x) const
	{
		return (x.dir)|(x.priority<<2)|(x.gcost<<8)|(x.hcost<<12)|(x.bucket<<20);
	}
};

struct openList {
	//openList() :f(0) {}
	std::unordered_set<uint64_t> states;
	//FILE *f;
};

struct closedData {
	tSearchDirection dir;
	uint8_t depth;
	uint8_t bucket;
};

struct closedList {
	//closedList() :f(0) {}
	std::unordered_set<uint64_t> states;
	//FILE *f;
};

static bool operator==(const closedData &a, const closedData &b)
{
	return (a.dir == b.dir && a.depth == b.depth && a.bucket == b.bucket);
}

struct closedDataHash
{
	std::size_t operator()(const closedData & x) const
	{
		return (x.dir)|(x.bucket<<2)|(x.bucket<<7);
	}
};

//std::vector<std::unordered_set<uint64_t>> closed(fileBuckets);
std::unordered_map<closedData, closedList, closedDataHash> closed;
std::unordered_map<openData, openList, openDataHash> open;

//std::string GetClosedName(closedData d)
//{
//	std::string s;
//	if (d.dir == kForward)
//	{
//		s += prefix1;
//		s += "forward-";
//	}
//	else {
//		s += prefix2;
//		s += "backward-";
//	}
//	s += std::to_string(d.bucket);
//	s += "-";
//	s += std::to_string(d.depth);
//	s += ".closed";
//	return s;
//}
//
//std::string GetOpenName(const openData &d)
//{
//	std::string s;
//	if (d.dir == kForward)
//	{
//		s += prefix1;
//		s += "forward-";
//	}
//	else {
//		s += prefix2;
//		s += "backward-";
//	}
//	s += std::to_string(d.priority);
//	s += "-";
//	s += std::to_string(d.gcost);
//	s += "-";
//	s += std::to_string(d.hcost);
////	s += "-";
////	s += std::to_string(d.hcost2);
//	s += "-";
//	s += std::to_string(d.bucket);
//	s += ".open";
//	return s;
//}

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
		{
			best = s.first;
		}
		else if (s.first.priority == best.priority)
		{
			if (s.first.gcost < best.gcost)
				best = s.first;
			else if (s.first.gcost == best.gcost)
			{
				if (s.first.dir == best.dir)
				{
					if (best.bucket > s.first.bucket)
						best = s.first;
				}
				else if (s.first.dir == kForward)
					best = s.first;
			}
		}
	}
	return best;
}

void GetOpenData(const PancakePuzzleState &start, tSearchDirection dir, int cost,
				 openData &d, uint64_t &data)
{
	int bucket;
	//uint64_t data;
	GetBucketAndData(start, bucket, data);
	//openData d;
	d.dir = dir;
	d.gcost = cost;
	d.hcost = (dir==kForward)?forward.HCost(start, start):reverse.HCost(start, start);
	//d.hcost2 = (dir==kForward)?reverse.HCost(start, start):forward.HCost(start, start);
	d.bucket = bucket;
	//d.priority = d.gcost+d.hcost;
	d.priority = std::max(d.gcost+d.hcost, d.gcost*2);
}

void AddStatesToQueue(const openData &d, uint64_t *data, size_t count)
{
	openLock.lock();
	for (size_t x = 0; x < count; x++)
		open[d].states.insert(data[x]);
	openLock.unlock();
}

void AddStateToQueue(openData &d, uint64_t data)
{
	AddStatesToQueue(d, &data, 1);
//	if (open.find(d) == open.end())
//	{
//		openLock.lock();
//		open[d].f = fopen(GetOpenName(d).c_str(), "w+b");
//		if (open[d].f == 0)
//		{
//			printf("Error opening %s; Aborting!\n", GetOpenName(d).c_str());
//			exit(0);
//		}
//		openLock.unlock();
//	}
//	if (open[d].f == 0)
//	{
//		printf("Error - file is null!\n");
//		exit(0);
//	}
//	fwrite(&data, sizeof(data), 1, open[d].f);
}

void AddStateToQueue(const PancakePuzzleState &start, tSearchDirection dir, int cost)
{
	openData d;
	uint64_t rank;
	GetOpenData(start, dir, cost, d, rank);
	AddStateToQueue(d, rank);
}

//void ReadFile(const openData &d, std::unordered_map<PancakePuzzleState, bool> &map)
//{
//	// 1. open file for d;
//	FILE *f = fopen(GetOpenName(d).c_str(), "r");
//	assert(f != 0);
//	uint64_t data;
//	PancakePuzzleState s;
//	while (fread(&data, 1, sizeof(uint64_t), f) == 1)
//	{
//		GetState(s, d.bucket, data);
//		map[s] = true;
//	}
//	fclose(f);
//	// keep file for duplicate detection purposes
//	//remove(GetOpenName(d).c_str());
//}

bool CanTerminateSearch()
{
	int val;
	if (bestSolution <= (val = std::max(currentC, std::max(minFForward, std::max(minFBackward, minGBackward+minGForward+1)))))
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
		finished = true;
		if (val == currentC)
			printf("-Triggered by current priority\n");
		if (val == minFForward)
			printf("-Triggered by f in the forward direction\n");
		if (val == minFBackward)
			printf("-Triggered by f in the backward direction\n");
		if (val == minGBackward+minGForward+1)
			printf("-Triggered by gforward+gbackward+1\n");
		return true;
	}
	return false;
}

void CheckSolution(std::unordered_map<openData, openList, openDataHash> currentOpen, openData d,
				   const std::unordered_set<uint64_t> &states)
{
	for (const auto &s : currentOpen)
	{
		// Opposite direction, same bucket AND could be a solution (g+g >= C)
		if (s.first.dir != d.dir && s.first.bucket == d.bucket &&
			d.gcost + s.first.gcost >= currentC)// && d.hcost2 == s.first.hcost)
		{
			for (auto &tmp : s.second.states)
			{
				if (states.find(tmp) != states.end())
				{
					printLock.lock();
					//s.second.states.find(data) != s.second.states.end()
					printf("\nFound solution cost %d+%d=%d\n", d.gcost, s.first.gcost, d.gcost + s.first.gcost);
					bestSolution = std::min(d.gcost + s.first.gcost, bestSolution);
					printf("Current best solution: %d\n", bestSolution);
					printLock.unlock();
					
					if (CanTerminateSearch())
						return;
				}
			}
		}
	}
}

void ReadBucket(std::unordered_set<uint64_t> &states, openData d)
{
	states = open[d].states;
//	const size_t bufferSize = 128;
//	uint64_t buffer[bufferSize];
//	rewind(open[d].f);
//	size_t numRead;
//	do {
//		numRead = fread(buffer, sizeof(uint64_t), bufferSize, open[d].f);
//		for (int x = 0; x < numRead; x++)
//			states.insert(buffer[x]);
//	} while (numRead == bufferSize);
//	fclose(open[d].f);
//	remove(GetOpenName(d).c_str());
//	open[d].f = 0;
}

void RemoveDuplicates(std::unordered_set<uint64_t> &states, openData d)
{
	for (int depth = d.gcost-2; depth < d.gcost; depth++)
	{
		closedData c;
		c.bucket = d.bucket;
		c.depth = depth;
		c.dir = d.dir;
		
		closedList &cd = closed[c];
		
		for (const auto &tmp : cd.states)
		{
			auto iter = states.find(tmp);
			if (iter != states.end())
				states.erase(iter);
		}
	}
}

void WriteToClosed(std::unordered_set<uint64_t> &states, openData d)
{
	closedData c;
	c.bucket = d.bucket;
	c.depth = d.gcost;
	c.dir = d.dir;

	closedList &cd = closed[c];
	for (const auto &i : states)
	{
		cd.states.insert(i);
	}
}

void ParallelExpandBucket(openData d, const std::unordered_set<uint64_t> &states, int myThread, int totalThreads)
{
	const int cacheSize = 1024;
	std::unordered_map<openData, std::vector<uint64_t>, openDataHash> cache;
	PancakePuzzleState tmp;
	uint64_t localExpanded = 0;
	int count = 0;
	for (const auto &values : states)
	{
		if (finished)
			break;

		count++;
		if (myThread != (count%totalThreads))
			continue;
		
		localExpanded++;
		for (int x = 2; x <= 10; x++)
		{
			GetState(tmp, d.bucket, values);
			// copying 2 64-bit values is faster than undoing a move
			pancake.ApplyAction(tmp, x);
			openData newData;
			uint64_t newRank;
			GetOpenData(tmp, d.dir, d.gcost+1, newData, newRank);
			
			std::vector<uint64_t> &c = cache[newData];
			c.push_back(newRank);
			if (c.size() > cacheSize)
			{
				AddStatesToQueue(newData, &c[0], c.size());
				c.clear();
			}
		}
	}
	for (auto &i : cache)
	{
		AddStatesToQueue(i.first, &(i.second[0]), i.second.size());
	}
	
	countLock.lock();
	expanded += localExpanded;
	if (d.dir == kForward)
		gDistForward[d.gcost]+=localExpanded;
	else
		gDistBackward[d.gcost]+=localExpanded;
	countLock.unlock();
}

void ExpandNextFile()
{
	// 1. Get next expansion target
	openData d = GetBestFile();
	currentC = d.priority;

	if (CanTerminateSearch())
		return;

	Timer timer;
	timer.StartTimer();
	
	std::unordered_set<uint64_t> states;
	ReadBucket(states, d);
	RemoveDuplicates(states, d);
	WriteToClosed(states, d);
	timer.EndTimer();
	
	printLock.lock();
	std::cout << "Next: " << d << " (" << states.size() << " entries) [" << timer.GetElapsedTime() << "s reading/dd] ";
	printLock.unlock();

	timer.StartTimer();
	// Read in opposite buckets to check for solutions in parallel to expanding this bucket
	openLock.lock();
	std::thread t(CheckSolution, open, d, std::ref(states));
	openLock.unlock();
	
	// 3. expand all states in current bucket & write out successors
	const int numThreads = 1;//std::thread::hardware_concurrency();
	std::vector<std::thread *> threads;
	for (int x = 0; x < numThreads; x++)
		threads.push_back(new std::thread(ParallelExpandBucket, d, std::ref(states), x, numThreads));
	for (int x = 0; x < threads.size(); x++)
	{
		threads[x]->join();
		delete threads[x];
	}
	open.erase(open.find(d));
	timer.EndTimer();
	printLock.lock();
	std::cout << "[" << timer.GetElapsedTime() << "s expanding]+";
	timer.StartTimer();
	printLock.unlock();
	t.join();

	printLock.lock();
	timer.EndTimer();
	std::cout << "[" << timer.GetElapsedTime() << "s]\n";
	printLock.unlock();
}

void BuildHeuristics(int count, PancakePuzzleState start, PancakePuzzleState goal, Heuristic<PancakePuzzleState> &result)
{
	PancakePuzzle pancake(10);
	std::vector<int> pdb;
	for (int x = 0; x < count; x++)
		pdb.push_back(x);

	PermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> *pdb1;
	pdb1 = new PermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle>(&pancake, goal, pdb);
	pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
	result.lookups.push_back({kLeafNode, 0, 0});
	result.heuristics.push_back(pdb1);
}

int GetBucket(const PancakePuzzleState &s)
{
	return 0;
}

void GetBucketAndData(const PancakePuzzleState &s, int &bucket, uint64_t &data)
{
	bucket = 0;
	data = pancake.GetStateHash(s);
}

void GetState(PancakePuzzleState &s, int bucket, uint64_t data)
{
#pragma warning Size not being set correctly
	s.puzzle.resize(10);
	pancake.GetStateFromHash(s, data);
}

#pragma mark Main Code

void GetInstance(int which, PancakePuzzleState &s)
{
	int states[309][10] =
	{{0, 1, 2, 3, 4, 5, 6, 7, 9, 8},
		{0, 1, 2, 3, 4, 5, 6, 9, 8, 7},
		{0, 1, 2, 3, 4, 5, 9, 8, 7, 6},
		{0, 1, 2, 3, 4, 9, 8, 7, 6, 5},
		{0, 1, 2, 3, 9, 8, 7, 6, 5, 4},
		{0, 1, 2, 9, 8, 7, 6, 5, 4, 3},
		{0, 1, 9, 8, 7, 6, 5, 4, 3, 2},
		{0, 9, 8, 7, 6, 5, 4, 3, 2, 1},
		{9, 8, 7, 6, 5, 4, 3, 2, 1, 0},
		{0, 1, 2, 3, 4, 5, 6, 8, 9, 7},
		{0, 1, 2, 3, 4, 5, 8, 9, 7, 6},
		{0, 1, 2, 3, 4, 8, 9, 7, 6, 5},
		{0, 1, 2, 3, 8, 9, 7, 6, 5, 4},
		{0, 1, 2, 8, 9, 7, 6, 5, 4, 3},
		{0, 1, 8, 9, 7, 6, 5, 4, 3, 2},
		{0, 8, 9, 7, 6, 5, 4, 3, 2, 1},
		{8, 9, 7, 6, 5, 4, 3, 2, 1, 0},
		{0, 1, 2, 3, 4, 5, 6, 9, 7, 8},
		{0, 1, 2, 3, 4, 5, 7, 8, 9, 6},
		{0, 1, 2, 3, 4, 7, 8, 9, 6, 5},
		{0, 1, 2, 3, 7, 8, 9, 6, 5, 4},
		{0, 1, 2, 7, 8, 9, 6, 5, 4, 3},
		{0, 1, 7, 8, 9, 6, 5, 4, 3, 2},
		{0, 7, 8, 9, 6, 5, 4, 3, 2, 1},
		{7, 8, 9, 6, 5, 4, 3, 2, 1, 0},
		{0, 1, 2, 3, 4, 5, 9, 8, 6, 7},
		{0, 1, 2, 3, 4, 5, 9, 6, 7, 8},
		{0, 1, 2, 3, 4, 6, 7, 8, 9, 5},
		{0, 1, 2, 3, 6, 7, 8, 9, 5, 4},
		{0, 1, 2, 6, 7, 8, 9, 5, 4, 3},
		{0, 1, 6, 7, 8, 9, 5, 4, 3, 2},
		{0, 6, 7, 8, 9, 5, 4, 3, 2, 1},
		{6, 7, 8, 9, 5, 4, 3, 2, 1, 0},
		{0, 1, 2, 3, 4, 9, 8, 7, 5, 6},
		{0, 1, 2, 3, 4, 9, 8, 5, 6, 7},
		{0, 1, 2, 3, 4, 9, 5, 6, 7, 8},
		{0, 1, 2, 3, 5, 6, 7, 8, 9, 4},
		{0, 1, 2, 5, 6, 7, 8, 9, 4, 3},
		{0, 1, 5, 6, 7, 8, 9, 4, 3, 2},
		{0, 1, 2, 3, 4, 5, 6, 8, 7, 9},
		{0, 1, 2, 3, 4, 5, 7, 9, 8, 6},
		{0, 1, 2, 3, 4, 7, 9, 8, 6, 5},
		{0, 1, 2, 3, 7, 9, 8, 6, 5, 4},
		{0, 1, 2, 7, 9, 8, 6, 5, 4, 3},
		{0, 1, 7, 9, 8, 6, 5, 4, 3, 2},
		{0, 7, 9, 8, 6, 5, 4, 3, 2, 1},
		{7, 9, 8, 6, 5, 4, 3, 2, 1, 0},
		{0, 1, 2, 3, 4, 5, 8, 9, 6, 7},
		{0, 1, 2, 3, 4, 5, 8, 6, 7, 9},
		{0, 1, 2, 3, 4, 6, 7, 9, 8, 5},
		{0, 1, 2, 3, 6, 7, 9, 8, 5, 4},
		{0, 1, 2, 6, 7, 9, 8, 5, 4, 3},
		{0, 1, 6, 7, 9, 8, 5, 4, 3, 2},
		{0, 6, 7, 9, 8, 5, 4, 3, 2, 1},
		{6, 7, 9, 8, 5, 4, 3, 2, 1, 0},
		{0, 1, 2, 3, 4, 8, 9, 7, 5, 6},
		{0, 1, 2, 3, 4, 8, 9, 5, 6, 7},
		{0, 1, 2, 3, 4, 8, 5, 6, 7, 9},
		{0, 1, 2, 3, 5, 6, 7, 9, 8, 4},
		{0, 1, 2, 5, 6, 7, 9, 8, 4, 3},
		{0, 1, 5, 6, 7, 9, 8, 4, 3, 2},
		{0, 5, 6, 7, 9, 8, 4, 3, 2, 1},
		{5, 6, 7, 9, 8, 4, 3, 2, 1, 0},
		{0, 1, 2, 3, 8, 9, 7, 6, 4, 5},
		{0, 1, 2, 3, 8, 9, 7, 4, 5, 6},
		{0, 1, 2, 3, 8, 9, 4, 5, 6, 7},
		{0, 1, 2, 3, 8, 4, 5, 6, 7, 9},
		{0, 1, 2, 4, 5, 6, 7, 9, 8, 3},
		{0, 1, 4, 5, 6, 7, 9, 8, 3, 2},
		{0, 1, 2, 3, 4, 5, 9, 7, 8, 6},
		{0, 1, 2, 3, 4, 9, 7, 8, 6, 5},
		{0, 1, 2, 3, 9, 7, 8, 6, 5, 4},
		{0, 1, 2, 9, 7, 8, 6, 5, 4, 3},
		{0, 1, 9, 7, 8, 6, 5, 4, 3, 2},
		{0, 9, 7, 8, 6, 5, 4, 3, 2, 1},
		{9, 7, 8, 6, 5, 4, 3, 2, 1, 0},
		{0, 1, 2, 3, 4, 5, 7, 9, 6, 8},
		{0, 1, 2, 3, 4, 6, 8, 9, 7, 5},
		{0, 1, 2, 3, 6, 8, 9, 7, 5, 4},
		{0, 1, 2, 6, 8, 9, 7, 5, 4, 3},
		{0, 1, 6, 8, 9, 7, 5, 4, 3, 2},
		{0, 6, 8, 9, 7, 5, 4, 3, 2, 1},
		{6, 8, 9, 7, 5, 4, 3, 2, 1, 0},
		{0, 1, 2, 3, 4, 7, 9, 8, 5, 6},
		{0, 1, 2, 3, 4, 7, 9, 5, 6, 8},
		{0, 1, 2, 3, 4, 7, 5, 6, 8, 9},
		{0, 1, 2, 3, 5, 6, 8, 9, 7, 4},
		{0, 1, 2, 5, 6, 8, 9, 7, 4, 3},
		{0, 1, 5, 6, 8, 9, 7, 4, 3, 2},
		{0, 5, 6, 8, 9, 7, 4, 3, 2, 1},
		{5, 6, 8, 9, 7, 4, 3, 2, 1, 0},
		{0, 1, 2, 3, 7, 9, 8, 6, 4, 5},
		{0, 1, 2, 3, 7, 9, 8, 4, 5, 6},
		{0, 1, 2, 3, 7, 9, 4, 5, 6, 8},
		{0, 1, 2, 3, 7, 4, 5, 6, 8, 9},
		{0, 1, 2, 4, 5, 6, 8, 9, 7, 3},
		{0, 1, 4, 5, 6, 8, 9, 7, 3, 2},
		{0, 4, 5, 6, 8, 9, 7, 3, 2, 1},
		{4, 5, 6, 8, 9, 7, 3, 2, 1, 0},
		{0, 1, 2, 3, 4, 6, 8, 7, 9, 5},
		{0, 1, 2, 3, 6, 8, 7, 9, 5, 4},
		{0, 1, 2, 6, 8, 7, 9, 5, 4, 3},
		{0, 1, 6, 8, 7, 9, 5, 4, 3, 2},
		{0, 6, 8, 7, 9, 5, 4, 3, 2, 1},
		{6, 8, 7, 9, 5, 4, 3, 2, 1, 0},
		{0, 1, 2, 3, 5, 6, 8, 7, 9, 4},
		{0, 1, 2, 5, 6, 8, 7, 9, 4, 3},
		{0, 1, 5, 6, 8, 7, 9, 4, 3, 2},
		{0, 5, 6, 8, 7, 9, 4, 3, 2, 1},
		{5, 6, 8, 7, 9, 4, 3, 2, 1, 0},
		{0, 1, 2, 3, 9, 7, 8, 6, 4, 5},
		{0, 1, 2, 4, 5, 6, 8, 7, 9, 3},
		{0, 1, 4, 5, 6, 8, 7, 9, 3, 2},
		{0, 4, 5, 6, 8, 7, 9, 3, 2, 1},
		{4, 5, 6, 8, 7, 9, 3, 2, 1, 0},
		{0, 1, 2, 9, 7, 8, 6, 5, 3, 4},
		{0, 1, 2, 9, 7, 8, 6, 3, 4, 5},
		{0, 1, 3, 4, 5, 6, 8, 7, 9, 2},
		{0, 3, 4, 5, 6, 8, 7, 9, 2, 1},
		{3, 4, 5, 6, 8, 7, 9, 2, 1, 0},
		{0, 1, 9, 7, 8, 6, 5, 4, 2, 3},
		{0, 1, 9, 7, 8, 6, 5, 2, 3, 4},
		{0, 1, 9, 7, 8, 6, 2, 3, 4, 5},
		{0, 2, 3, 4, 5, 6, 8, 7, 9, 1},
		{2, 3, 4, 5, 6, 8, 7, 9, 1, 0},
		{0, 9, 7, 8, 6, 5, 4, 3, 1, 2},
		{0, 9, 7, 8, 6, 5, 4, 1, 2, 3},
		{0, 9, 7, 8, 6, 5, 1, 2, 3, 4},
		{0, 9, 7, 8, 6, 1, 2, 3, 4, 5},
		{0, 1, 2, 3, 5, 9, 7, 8, 6, 4},
		{0, 1, 2, 5, 9, 7, 8, 6, 4, 3},
		{0, 1, 5, 9, 7, 8, 6, 4, 3, 2},
		{0, 5, 9, 7, 8, 6, 4, 3, 2, 1},
		{5, 9, 7, 8, 6, 4, 3, 2, 1, 0},
		{0, 1, 2, 3, 6, 8, 7, 9, 4, 5},
		{0, 1, 2, 4, 5, 9, 7, 8, 6, 3},
		{0, 1, 4, 5, 9, 7, 8, 6, 3, 2},
		{0, 4, 5, 9, 7, 8, 6, 3, 2, 1},
		{4, 5, 9, 7, 8, 6, 3, 2, 1, 0},
		{0, 1, 2, 6, 8, 7, 9, 5, 3, 4},
		{0, 1, 2, 6, 8, 7, 9, 3, 4, 5},
		{0, 1, 3, 4, 5, 9, 7, 8, 6, 2},
		{0, 3, 4, 5, 9, 7, 8, 6, 2, 1},
		{3, 4, 5, 9, 7, 8, 6, 2, 1, 0},
		{0, 1, 6, 8, 7, 9, 5, 4, 2, 3},
		{0, 1, 6, 8, 7, 9, 5, 2, 3, 4},
		{0, 1, 6, 8, 7, 9, 2, 3, 4, 5},
		{0, 2, 3, 4, 5, 9, 7, 8, 6, 1},
		{2, 3, 4, 5, 9, 7, 8, 6, 1, 0},
		{0, 6, 8, 7, 9, 5, 4, 3, 1, 2},
		{0, 6, 8, 7, 9, 5, 4, 1, 2, 3},
		{0, 6, 8, 7, 9, 5, 1, 2, 3, 4},
		{0, 6, 8, 7, 9, 1, 2, 3, 4, 5},
		{1, 2, 3, 4, 5, 9, 7, 8, 6, 0},
		{6, 8, 7, 9, 5, 4, 3, 2, 0, 1},
		{6, 8, 7, 9, 5, 4, 3, 0, 1, 2},
		{6, 8, 7, 9, 5, 4, 0, 1, 2, 3},
		{6, 8, 7, 9, 5, 0, 1, 2, 3, 4},
		{6, 8, 7, 9, 0, 1, 2, 3, 4, 5},
		{0, 1, 2, 4, 6, 8, 7, 9, 5, 3},
		{0, 1, 4, 6, 8, 7, 9, 5, 3, 2},
		{0, 4, 6, 8, 7, 9, 5, 3, 2, 1},
		{4, 6, 8, 7, 9, 5, 3, 2, 1, 0},
		{0, 1, 2, 5, 9, 7, 8, 6, 3, 4},
		{0, 1, 2, 5, 3, 4, 6, 8, 7, 9},
		{0, 1, 3, 4, 6, 8, 7, 9, 5, 2},
		{0, 3, 4, 6, 8, 7, 9, 5, 2, 1},
		{3, 4, 6, 8, 7, 9, 5, 2, 1, 0},
		{0, 1, 5, 9, 7, 8, 6, 4, 2, 3},
		{0, 1, 5, 9, 7, 8, 6, 2, 3, 4},
		{0, 1, 5, 2, 3, 4, 6, 8, 7, 9},
		{0, 2, 3, 4, 6, 8, 7, 9, 5, 1},
		{2, 3, 4, 6, 8, 7, 9, 5, 1, 0},
		{0, 5, 9, 7, 8, 6, 4, 3, 1, 2},
		{0, 5, 9, 7, 8, 6, 4, 1, 2, 3},
		{0, 5, 9, 7, 8, 6, 1, 2, 3, 4},
		{0, 5, 1, 2, 3, 4, 6, 8, 7, 9},
		{1, 2, 3, 4, 6, 8, 7, 9, 5, 0},
		{5, 9, 7, 8, 6, 4, 3, 2, 0, 1},
		{5, 9, 7, 8, 6, 4, 3, 0, 1, 2},
		{5, 9, 7, 8, 6, 4, 0, 1, 2, 3},
		{5, 9, 7, 8, 6, 0, 1, 2, 3, 4},
		{5, 0, 1, 2, 3, 4, 6, 8, 7, 9},
		{0, 1, 2, 5, 4, 9, 7, 8, 6, 3},
		{0, 1, 5, 4, 9, 7, 8, 6, 3, 2},
		{0, 5, 4, 9, 7, 8, 6, 3, 2, 1},
		{5, 4, 9, 7, 8, 6, 3, 2, 1, 0},
		{0, 1, 2, 4, 5, 3, 6, 8, 7, 9},
		{0, 1, 2, 4, 3, 6, 8, 7, 9, 5},
		{0, 1, 3, 5, 9, 7, 8, 6, 4, 2},
		{0, 3, 5, 9, 7, 8, 6, 4, 2, 1},
		{3, 5, 9, 7, 8, 6, 4, 2, 1, 0},
		{0, 1, 4, 6, 8, 7, 9, 5, 2, 3},
		{0, 1, 4, 6, 8, 7, 9, 2, 3, 5},
		{0, 1, 4, 2, 3, 5, 9, 7, 8, 6},
		{0, 2, 3, 5, 9, 7, 8, 6, 4, 1},
		{2, 3, 5, 9, 7, 8, 6, 4, 1, 0},
		{0, 4, 6, 8, 7, 9, 5, 3, 1, 2},
		{0, 4, 6, 8, 7, 9, 5, 1, 2, 3},
		{0, 4, 6, 8, 7, 9, 1, 2, 3, 5},
		{0, 4, 1, 2, 3, 5, 9, 7, 8, 6},
		{1, 2, 3, 5, 9, 7, 8, 6, 4, 0},
		{4, 6, 8, 7, 9, 5, 3, 2, 0, 1},
		{4, 6, 8, 7, 9, 5, 3, 0, 1, 2},
		{4, 6, 8, 7, 9, 5, 0, 1, 2, 3},
		{4, 6, 8, 7, 9, 0, 1, 2, 3, 5},
		{4, 0, 1, 2, 3, 5, 9, 7, 8, 6},
		{0, 1, 4, 3, 6, 8, 7, 9, 5, 2},
		{0, 4, 3, 6, 8, 7, 9, 5, 2, 1},
		{4, 3, 6, 8, 7, 9, 5, 2, 1, 0},
		{0, 9, 7, 8, 6, 4, 3, 5, 2, 1},
		{9, 7, 8, 6, 4, 3, 5, 2, 1, 0},
		{0, 1, 3, 4, 2, 5, 9, 7, 8, 6},
		{0, 1, 3, 2, 5, 9, 7, 8, 6, 4},
		{0, 2, 5, 9, 7, 8, 6, 4, 3, 1},
		{2, 5, 9, 7, 8, 6, 4, 3, 1, 0},
		{0, 3, 4, 6, 8, 7, 9, 5, 1, 2},
		{0, 3, 4, 6, 8, 7, 9, 1, 2, 5},
		{0, 3, 4, 1, 2, 5, 9, 7, 8, 6},
		{0, 1, 3, 5, 2, 4, 6, 8, 7, 9},
		{0, 2, 4, 6, 8, 7, 9, 5, 3, 1},
		{2, 4, 6, 8, 7, 9, 5, 3, 1, 0},
		{0, 3, 5, 9, 7, 8, 6, 4, 1, 2},
		{0, 3, 5, 9, 7, 8, 6, 1, 2, 4},
		{0, 3, 5, 1, 2, 4, 6, 8, 7, 9},
		{0, 3, 1, 2, 4, 6, 8, 7, 9, 5},
		{1, 2, 4, 6, 8, 7, 9, 5, 3, 0},
		{3, 5, 9, 7, 8, 6, 4, 2, 0, 1},
		{3, 5, 9, 7, 8, 6, 4, 0, 1, 2},
		{3, 5, 9, 7, 8, 6, 0, 1, 2, 4},
		{3, 5, 0, 1, 2, 4, 6, 8, 7, 9},
		{3, 0, 1, 2, 4, 6, 8, 7, 9, 5},
		{0, 3, 2, 5, 9, 7, 8, 6, 4, 1},
		{3, 2, 5, 9, 7, 8, 6, 4, 1, 0},
		{0, 5, 3, 2, 9, 7, 8, 6, 4, 1},
		{5, 3, 2, 9, 7, 8, 6, 4, 1, 0},
		{6, 8, 7, 9, 5, 3, 2, 4, 1, 0},
		{0, 2, 3, 5, 1, 4, 6, 8, 7, 9},
		{0, 2, 3, 1, 4, 6, 8, 7, 9, 5},
		{0, 2, 1, 4, 6, 8, 7, 9, 5, 3},
		{1, 4, 6, 8, 7, 9, 5, 3, 2, 0},
		{2, 3, 5, 9, 7, 8, 6, 4, 0, 1},
		{2, 3, 5, 9, 7, 8, 6, 0, 1, 4},
		{2, 3, 5, 0, 1, 4, 6, 8, 7, 9},
		{2, 3, 0, 1, 4, 6, 8, 7, 9, 5},
		{2, 0, 1, 4, 6, 8, 7, 9, 5, 3},
		{0, 4, 6, 8, 7, 9, 5, 2, 1, 3},
		{0, 4, 2, 1, 3, 5, 9, 7, 8, 6},
		{0, 2, 1, 3, 5, 9, 7, 8, 6, 4},
		{9, 7, 8, 6, 4, 2, 5, 3, 1, 0},
		{0, 2, 4, 1, 3, 5, 9, 7, 8, 6},
		{1, 3, 5, 9, 7, 8, 6, 4, 2, 0},
		{2, 4, 6, 8, 7, 9, 5, 3, 0, 1},
		{2, 4, 6, 8, 7, 9, 5, 0, 1, 3},
		{2, 4, 6, 8, 7, 9, 0, 1, 3, 5},
		{2, 4, 0, 1, 3, 5, 9, 7, 8, 6},
		{2, 0, 1, 3, 5, 9, 7, 8, 6, 4},
		{2, 1, 4, 6, 8, 7, 9, 5, 3, 0},
		{4, 2, 1, 6, 8, 7, 9, 5, 3, 0},
		{1, 2, 4, 0, 3, 5, 9, 7, 8, 6},
		{1, 2, 0, 3, 5, 9, 7, 8, 6, 4},
		{1, 0, 3, 5, 9, 7, 8, 6, 4, 2},
		{3, 5, 9, 7, 8, 6, 4, 1, 0, 2},
		{3, 5, 9, 7, 8, 6, 1, 0, 2, 4},
		{3, 5, 1, 0, 2, 4, 6, 8, 7, 9},
		{3, 1, 0, 2, 4, 6, 8, 7, 9, 5},
		{1, 0, 2, 4, 6, 8, 7, 9, 5, 3},
		{3, 5, 9, 7, 8, 6, 4, 0, 2, 1},
		{3, 5, 9, 7, 8, 6, 0, 1, 4, 2},
		{3, 5, 9, 7, 8, 6, 0, 4, 2, 1},
		{3, 5, 0, 9, 7, 8, 6, 4, 2, 1},
		{3, 0, 5, 9, 7, 8, 6, 4, 2, 1},
		{1, 4, 6, 8, 7, 9, 5, 2, 3, 0},
		{3, 2, 5, 9, 7, 8, 6, 0, 1, 4},
		{1, 4, 6, 8, 7, 9, 2, 3, 5, 0},
		{3, 5, 9, 7, 8, 6, 4, 1, 2, 0},
		{1, 4, 6, 8, 7, 9, 5, 3, 0, 2},
		{1, 4, 6, 8, 7, 9, 5, 0, 2, 3},
		{1, 4, 6, 8, 7, 9, 0, 2, 3, 5},
		{1, 3, 5, 0, 2, 4, 6, 8, 7, 9},
		{1, 3, 0, 2, 4, 6, 8, 7, 9, 5},
		{3, 1, 2, 0, 4, 6, 8, 7, 9, 5},
		{3, 5, 9, 7, 8, 6, 0, 2, 4, 1},
		{1, 4, 6, 8, 7, 9, 5, 2, 0, 3},
		{1, 4, 2, 0, 3, 5, 9, 7, 8, 6},
		{2, 0, 3, 5, 1, 4, 6, 8, 7, 9},
		{2, 0, 3, 1, 4, 6, 8, 7, 9, 5},
		{4, 0, 2, 1, 3, 5, 9, 7, 8, 6},
		{2, 0, 4, 6, 8, 7, 9, 5, 1, 3},
		{2, 0, 4, 1, 3, 5, 9, 7, 8, 6},
		{3, 1, 5, 9, 7, 8, 6, 4, 0, 2},
		{2, 5, 3, 1, 9, 7, 8, 6, 4, 0},
		{5, 3, 1, 9, 7, 8, 6, 4, 0, 2},
		{1, 3, 5, 2, 0, 4, 6, 8, 7, 9},
		{4, 1, 6, 8, 7, 9, 5, 0, 3, 2},
		{2, 5, 9, 7, 8, 6, 3, 0, 4, 1},
		{1, 4, 0, 3, 6, 8, 7, 9, 5, 2},
		{3, 1, 4, 0, 2, 5, 9, 7, 8, 6},
		{2, 5, 1, 3, 0, 4, 6, 8, 7, 9},
		{4, 1, 2, 5, 0, 3, 6, 8, 7, 9},
		{2, 5, 9, 7, 8, 6, 0, 3, 1, 4},
		{2, 5, 0, 3, 1, 4, 6, 8, 7, 9},
		{4, 1, 3, 0, 2, 5, 9, 7, 8, 6},
		{4, 6, 8, 7, 9, 0, 3, 1, 5, 2},
		{1, 3, 0, 4, 2, 5, 9, 7, 8, 6},
		{3, 1, 5, 2, 0, 9, 7, 8, 6, 4},
		{3, 1, 6, 8, 7, 9, 5, 2, 0, 4},
		{3, 0, 2, 5, 1, 4, 6, 8, 7, 9},
		{1, 3, 2, 4, 0, 5, 9, 7, 8, 6}};
	for (int x = 0; x < 10; x++)
	{
		s.puzzle[x] = 9-states[which][9-x];
	}
}

void MM()
{
	PancakePuzzleState start(10);
	PancakePuzzleState goal(10);
	
	expanded = 0;
	BuildHeuristics(3, start, goal, forward);
	BuildHeuristics(3, goal, start, reverse);

	std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
	std::cout << std::setprecision(2);
	
	for (int instance = 69; instance < 309; instance++)
	{
		expanded = 0;
		gDistBackward.clear();
		gDistForward.clear();
		gDistBackward.resize(12);
		gDistForward.resize(12);

		bestSolution = 100;
		finished = false;
		printf("Problem %d\n", instance);
		GetInstance(instance, start);
		goal.Reset();
		std::cout << "Start: " << start << std::endl;
		std::cout << "Goal: " << goal << std::endl;
		printf("---A*---\n");
		std::vector<PancakePuzzleState> path;
		Timer t;
		t.StartTimer();
		TemplateAStar<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> astar;
		astar.SetHeuristic(&forward);
		astar.GetPath(&pancake, start, goal, path);
		t.EndTimer();
		printf("%1.5fs elapsed\n", t.GetElapsedTime());
		printf("%llu nodes expanded (%1.3f nodes/sec)\n", astar.GetNodesExpanded(),
			   astar.GetNodesExpanded()/t.GetElapsedTime());
		printf("%llu nodes generated (%1.3f nodes/sec)\n", astar.GetNodesTouched(),
			   astar.GetNodesTouched()/t.GetElapsedTime());
		printf("Solution cost: %1.1f\n", pancake.GetPathLength(path));
		
		t.StartTimer();
		GetInstance(instance, start);
		goal.Reset();
		std::cout << "Start: " << start << std::endl;
		std::cout << "Goal: " << goal << std::endl;
		printf("---MM*---\n");
		AddStateToQueue(start, kForward, 0);
		AddStateToQueue(goal, kBackward, 0);
		while (!open.empty() && !finished)
		{
			ExpandNextFile();
		}
		t.EndTimer();
		printf("Solution cost: %d\n", bestSolution);
		printf("%1.2fs elapsed\n", t.GetElapsedTime());
		closed.clear();
		open.clear();

		printf("[final] %d %1.0f Astar: %llu MM: %llu\n", bestSolution, pancake.GetPathLength(path), astar.GetNodesExpanded(), expanded);
	}
}

NAMESPACE_CLOSE(MMPancake)
