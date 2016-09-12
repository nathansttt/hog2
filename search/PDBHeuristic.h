//
//  PDBHeuristic.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/19/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//

#ifndef hog2_glut_PDBHeuristic_h
#define hog2_glut_PDBHeuristic_h

#include <cassert>
#include <thread>
#include <string>
#include "Heuristic.h"
#include "SharedQueue.h"
#include "NBitArray.h"
#include "Timer.h"
#include "RangeCompression.h"

enum PDBLookupType {
	kPlain,
	kDivCompress,
//	kFractionalCompress,
//	kFractionalModCompress,
	kModCompress,
	kValueCompress,
	kDivPlusValueCompress,
	kDivPlusDeltaCompress, // two lookups with the same index, one is div, one is delta
	kDefaultHeuristic
};

const int coarseSize = 1024;
const int maxThreads = 32; // TODO: This isn't enforced in a static assert

template <class abstractState, class abstractAction, class abstractEnvironment, class state = abstractState, uint64_t pdbBits = 8>
class PDBHeuristic : public Heuristic<state> {
public:
	PDBHeuristic(abstractEnvironment *e) :type(kPlain), env(e)
	{ goalSet = false; }
	virtual ~PDBHeuristic() {}

	void SetGoal(const state &goal)
	{	GetStateFromPDBHash(this->GetAbstractHash(goal), goalState); goalSet = true; }

	virtual double HCost(const state &a, const state &b) const;

	virtual uint64_t GetPDBSize() const = 0;

	virtual uint64_t GetPDBHash(const abstractState &s, int threadID = 0) const = 0;
	virtual uint64_t GetAbstractHash(const state &s, int threadID = 0) const = 0;
	virtual void GetStateFromPDBHash(uint64_t hash, abstractState &s, int threadID = 0) const = 0;
	virtual state GetStateFromAbstractState(abstractState &s) const = 0;

	virtual bool Load(const char *prefix) = 0;
	virtual void Save(const char *prefix) = 0;
	virtual bool Load(FILE *f);
	virtual void Save(FILE *f);
	virtual std::string GetFileName(const char *prefix) = 0;
	
	void BuildPDB(const state &goal, int numThreads)
	{ BuildPDBForwardBackward(goal, numThreads); }
	void BuildPDBForward(const state &goal, int numThreads);
	void BuildPDBBackward(const state &goal, int numThreads);
	void BuildPDBForwardBackward(const state &goal, int numThreads);

	void BuildAdditivePDB(state &goal, const char *pdb_filename, int numThreads);

	void DivCompress(int factor, bool print_histogram);
	void ModCompress(int factor, bool print_histogram);
	void ModCompress(uint64_t newEntries, bool print_histogram);
	void ZeroLowValues(int limit)
	{ for (uint64_t s = 0; s < PDB.Size(); s++)
		if (PDB.Get(s) < limit) PDB.Set(s, 0); }
	
	void DeltaCompress(Heuristic<state> *h, state goal, bool print_histogram);
	
	void FractionalDivCompress(uint64_t count, bool print_histogram);
	void FractionalModCompress(uint64_t factor, bool print_histogram);
	void ValueCompress(int maxValue, bool print_histogram);
	void ValueCompress(std::vector<int> cutoffs, bool print_histogram);
	void ValueRangeCompress(int numBits, bool print_histogram);
	void CustomValueRangeCompress(std::vector<uint64_t> dist, int numBits, bool print_histogram);

	void ValueRangeCompress(PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, 5> *, bool print_histogram);
	void ValueRangeCompress(PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, 4> *, bool print_histogram);
	void ValueRangeCompress(PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, 3> *, bool print_histogram);
	void ValueRangeCompress(PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, 2> *, bool print_histogram);
	void ValueRangeCompress(PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, 1> *, bool print_histogram);
	
	double PrintHistogram();
	double GetAverageValue();
	void GetHistogram(std::vector<uint64_t> &histogram);
//protected:
//	friend class PDBHeuristic<abstractState, abstractAction, abstractEnvironment, abstractState, 4>;
//	friend class PDBHeuristic<abstractState, abstractAction, abstractEnvironment, abstractState, 3>;
//	friend class PDBHeuristic<abstractState, abstractAction, abstractEnvironment, abstractState, 2>;
//	friend class PDBHeuristic<abstractState, abstractAction, abstractEnvironment, abstractState, 1>;

	// holds a Pattern Databases
	NBitArray<pdbBits> PDB;
	int vrcValues[1<<pdbBits];
	PDBLookupType type;
	uint64_t compressionValue;

	abstractEnvironment *env;
	abstractState goalState;
private:
	bool goalSet;
	void ForwardThreadWorker(int threadNum, int depth,
							 NBitArray<pdbBits> &DB,
							 std::vector<bool> &coarse,
							 SharedQueue<std::pair<uint64_t, uint64_t> > *work,
							 SharedQueue<uint64_t> *results,
							 std::mutex *lock);
	void BackwardThreadWorker(int threadNum, int depth,
							  NBitArray<pdbBits> &DB,
							  std::vector<bool> &coarse,
							  SharedQueue<std::pair<uint64_t, uint64_t> > *work,
							  SharedQueue<uint64_t> *results,
							  std::mutex *lock);
	void ForwardBackwardThreadWorker(int threadNum, int depth, bool forward,
									 NBitArray<pdbBits> &DB,
									 std::vector<bool> &coarseOpen,
									 std::vector<bool> &coarseClosed,
									 SharedQueue<std::pair<uint64_t, uint64_t> > *work,
									 SharedQueue<uint64_t> *results,
									 std::mutex *lock);
};

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
double PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::HCost(const state &a, const state &b) const
{
	switch (type)
	{
		case kPlain:
		{
			return PDB.Get(GetAbstractHash(a)); //PDB[GetPDBHash(a)];
		}
		case kDivCompress:
		{
			return PDB.Get(GetAbstractHash(a)/compressionValue);
		}
		case kModCompress:
		{
			return PDB.Get(GetAbstractHash(a)%compressionValue);
		}
		case kValueCompress:
		{
			return vrcValues[PDB.Get(GetAbstractHash(a))]; //PDB[GetPDBHash(a)];
		}
		case kDivPlusValueCompress:
		{
			return vrcValues[PDB.Get(GetAbstractHash(a)/compressionValue)];
		}
		default:
			assert(!"Not implemented");
	}
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::BuildPDBForward(const state &goal, int numThreads)
{
	assert(goalSet);
	SharedQueue<std::pair<uint64_t, uint64_t> > workQueue(numThreads*20);
	SharedQueue<uint64_t> resultQueue;
	std::mutex lock;
	
	uint64_t COUNT = GetPDBSize();
	PDB.Resize(COUNT);
	PDB.FillMax();
	
	// with weights we have to store the lowest weight stored to make sure
	// we don't skip regions
	std::vector<bool> coarseOpenCurr((COUNT+coarseSize-1)/coarseSize);
	std::vector<bool> coarseOpenNext((COUNT+coarseSize-1)/coarseSize);
	
	uint64_t entries = 1;
	std::cout << "Num Entries: " << COUNT << std::endl;
	std::cout << "Goal State: " << goalState << std::endl;
	//std::cout << "State Hash of Goal: " << GetStateHash(goal) << std::endl;
	std::cout << "PDB Hash of Goal: " << GetPDBHash(goalState) << std::endl;
	
	std::deque<state> q_curr, q_next;
	std::vector<state> children;
	
	Timer t;
	t.StartTimer();
	PDB.Set(GetPDBHash(goalState), 0);

	coarseOpenCurr[GetPDBHash(goalState)/coarseSize] = true;
	int depth = 0;
	uint64_t newEntries;
	std::vector<std::thread*> threads(numThreads);
	printf("Creating %d threads\n", numThreads);
	do {
		newEntries = 0;
		Timer s;
		s.StartTimer();
		for (int x = 0; x < numThreads; x++)
		{
			threads[x] = new std::thread(&PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ForwardThreadWorker,
										 this,
										 x, depth, std::ref(PDB), std::ref(coarseOpenNext),
										 &workQueue, &resultQueue, &lock);
		}
		
		for (uint64_t x = 0; x < COUNT; x+=coarseSize)
		{
			if (coarseOpenCurr[x/coarseSize])
			{
				workQueue.WaitAdd({x, std::min(COUNT, x+coarseSize)});
			}
			coarseOpenCurr[x/coarseSize] = false;
		}
		for (int x = 0; x < numThreads; x++)
		{
			workQueue.WaitAdd({0,0});
		}
		for (int x = 0; x < numThreads; x++)
		{
			threads[x]->join();
			delete threads[x];
			threads[x] = 0;
		}
		// read out node counts
		uint64_t total = 0;
		{
			uint64_t val;
			while (resultQueue.Remove(val))
			{
				total+=val;
			}
		}
		
		entries += total;//newEntries;
		printf("Depth %d complete; %1.2fs elapsed. %llu new states written; %llu of %llu total\n",
			   depth, s.EndTimer(), total, entries, COUNT);
		depth++;
		coarseOpenCurr.swap(coarseOpenNext);
	} while (entries != COUNT);
	
	printf("%1.2fs elapsed\n", t.EndTimer());
	if (entries != COUNT)
	{
		printf("Entries: %llu; count: %llu\n", entries, COUNT);
		assert(entries == COUNT);
	}
	PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::BuildPDBBackward(const state &goal, int numThreads)
{
	assert(goalSet);
	SharedQueue<std::pair<uint64_t, uint64_t> > workQueue(numThreads*20);
	SharedQueue<uint64_t> resultQueue;
	std::mutex lock;
	
	uint64_t COUNT = GetPDBSize();
	PDB.Resize(COUNT);
	PDB.FillMax();
	
	// with weights we have to store the lowest weight stored to make sure
	// we don't skip regions
	std::vector<bool> coarseClosed((COUNT+coarseSize-1)/coarseSize);
	
	uint64_t entries = 1;
	std::cout << "Num Entries: " << COUNT << std::endl;
	std::cout << "Goal State: " << goalState << std::endl;
	//std::cout << "State Hash of Goal: " << GetStateHash(goal) << std::endl;
	std::cout << "PDB Hash of Goal: " << GetPDBHash(goalState) << std::endl;
	
	std::deque<state> q_curr, q_next;
	std::vector<state> children;
	
	Timer t;
	t.StartTimer();
	PDB.Set(GetPDBHash(goalState), 0);
	
	int depth = 0;
	uint64_t newEntries;
	std::vector<std::thread*> threads(numThreads);
	printf("Creating %d threads\n", numThreads);
	do {
		newEntries = 0;
		Timer s;
		s.StartTimer();
		for (int x = 0; x < numThreads; x++)
		{
			threads[x] = new std::thread(&PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::BackwardThreadWorker,
										 this,
										 x, depth, std::ref(PDB), std::ref(coarseClosed),
										 &workQueue, &resultQueue, &lock);
		}
		for (uint64_t x = 0; x < COUNT; x+=coarseSize)
		{
			if (coarseClosed[x/coarseSize] == false)
			{
				workQueue.WaitAdd({x, std::min(COUNT, x+coarseSize)});
			}
		}
		for (int x = 0; x < numThreads; x++)
		{
			workQueue.WaitAdd({0,0});
		}
		for (int x = 0; x < numThreads; x++)
		{
			threads[x]->join();
			delete threads[x];
			threads[x] = 0;
		}
		// read out node counts
		uint64_t total = 0;
		{
			uint64_t val;
			while (resultQueue.Remove(val))
			{
				total+=val;
			}
		}
		
		entries += total;//newEntries;
		printf("Depth %d complete; %1.2fs elapsed. %llu new states written; %llu of %llu total\n",
			   depth, s.EndTimer(), total, entries, COUNT);
		depth++;
	} while (entries != COUNT);
	
	printf("%1.2fs elapsed\n", t.EndTimer());
	if (entries != COUNT)
	{
		printf("Entries: %llu; count: %llu\n", entries, COUNT);
		assert(entries == COUNT);
	}
	PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::BuildPDBForwardBackward(const state &goal, int numThreads)
{
	assert(goalSet);
	SharedQueue<std::pair<uint64_t, uint64_t> > workQueue(numThreads*20);
	SharedQueue<uint64_t> resultQueue;
	std::mutex lock;
	
	uint64_t COUNT = GetPDBSize();
	PDB.Resize(COUNT);
	PDB.FillMax();
	
	// with weights we have to store the lowest weight stored to make sure
	// we don't skip regions
	std::vector<bool> coarseClosed((COUNT+coarseSize-1)/coarseSize);
	std::vector<bool> coarseOpenCurr((COUNT+coarseSize-1)/coarseSize);
	std::vector<bool> coarseOpenNext((COUNT+coarseSize-1)/coarseSize);
	
	uint64_t entries = 1;
	std::cout << "Num Entries: " << COUNT << std::endl;
	std::cout << "Goal State: " << goalState << std::endl;
	//std::cout << "State Hash of Goal: " << GetStateHash(goal) << std::endl;
	std::cout << "PDB Hash of Goal: " << GetPDBHash(goalState) << std::endl;
	
	std::deque<state> q_curr, q_next;
	std::vector<state> children;
	std::vector<uint64_t> distribution;

	Timer t;
	t.StartTimer();
	PDB.Set(GetPDBHash(goalState), 0);
	coarseOpenCurr[GetPDBHash(goalState)/coarseSize] = true;
	distribution.push_back(1);
	
	int depth = 0;
	uint64_t newEntries;
	bool searchForward = true;
	std::vector<std::thread*> threads(numThreads);
	printf("Creating %d threads\n", numThreads);
	do {
		newEntries = 0;
		Timer s;
		s.StartTimer();
		for (int x = 0; x < numThreads; x++)
		{
			threads[x] = new std::thread(&PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ForwardBackwardThreadWorker,
										 this,
										 x, depth, searchForward,
										 std::ref(PDB), std::ref(coarseOpenNext), std::ref(coarseClosed),
										 &workQueue, &resultQueue, &lock);
		}
		if (searchForward)
		{
			for (uint64_t x = 0; x < COUNT; x+=coarseSize)
			{
				if (coarseOpenCurr[x/coarseSize])
				{
					workQueue.WaitAdd({x, std::min(COUNT, x+coarseSize)});
				}
				coarseOpenCurr[x/coarseSize] = false;
			}
		}
		else {
			for (uint64_t x = 0; x < COUNT; x+=coarseSize)
			{
				if (coarseClosed[x/coarseSize] == false)
				{
					workQueue.WaitAdd({x, std::min(COUNT, x+coarseSize)});
				}
			}
		}

		for (int x = 0; x < numThreads; x++)
		{
			workQueue.WaitAdd({0,0});
		}
		for (int x = 0; x < numThreads; x++)
		{
			threads[x]->join();
			delete threads[x];
			threads[x] = 0;
		}
		// read out node counts
		uint64_t total = 0;
		{
			uint64_t val;
			while (resultQueue.Remove(val))
			{
				total+=val;
			}
		}
		entries += total;//newEntries;
		distribution.push_back(total);
		printf("Depth %d complete; %1.2fs elapsed. %llu new states written; %llu of %llu total [%s]\n",
			   depth, s.EndTimer(), total, entries, COUNT, searchForward?"forward":"backward");
		if (double(total)*double(total)*0.4 > double(COUNT-entries)*double(distribution[distribution.size()-2]))// || depth == 8)
			searchForward = false;
		if (COUNT-entries <= total) // If we wrote more entries than there are left, switch directions
			searchForward = false;
		depth++;
		coarseOpenCurr.swap(coarseOpenNext);
	} while (entries != COUNT);
	
	printf("%1.2fs elapsed\n", t.EndTimer());
	if (entries != COUNT)
	{
		printf("Entries: %llu; count: %llu\n", entries, COUNT);
		assert(entries == COUNT);
	}
	PrintHistogram();
}


template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ForwardThreadWorker(int threadNum, int depth,
																	 NBitArray<pdbBits> &DB,
																	 std::vector<bool> &coarse,
																	 SharedQueue<std::pair<uint64_t, uint64_t> > *work,
																	 SharedQueue<uint64_t> *results,
																	 std::mutex *lock)
{
//	lock->lock();
//	printf("Thread %d online\n", threadNum);
//	lock->unlock();

	std::pair<uint64_t, uint64_t> p;
	uint64_t start, end;
	std::vector<abstractAction> acts;
	abstractState s(goalState), t(goalState);
	uint64_t count = 0;
	
	struct writeInfo {
		uint64_t rank;
		int newGCost;
	};
	std::vector<writeInfo> cache;
	while (true)
	{
		work->WaitRemove(p);
		if (p.first == p.second)
		{
			break;
		}
		start = p.first;
		end = p.second;
		//int nextDepth = 255;
		for (uint64_t x = start; x < end; x++)
		{
			int stateDepth = DB.Get(x);
			if (stateDepth == depth)
			{
				GetStateFromPDBHash(x, s, threadNum);
				//std::cout << "Expanding[r][" << stateDepth << "]: " << s << std::endl;
				env->GetActions(s, acts);
				for (int y = 0; y < acts.size(); y++)
				{
					env->GetNextState(s, acts[y], t);
					assert(env->InvertAction(acts[y]) == true);
					//virtual bool InvertAction(action &a) const = 0;
					
					uint64_t nextRank = GetPDBHash(t, threadNum);
					int newCost = stateDepth+(env->GCost(t, acts[y]));
					cache.push_back({nextRank, newCost});
				}
			}
		}
		// write out everything
		lock->lock();
		for (auto d : cache)
		{
			if (d.newGCost < DB.Get(d.rank)) // shorter path
			{
				count++;
				coarse[d.rank/coarseSize] = true;
				DB.Set(d.rank, d.newGCost);
			}
		}
		lock->unlock();
		cache.resize(0);
	}
	results->Add(count);
//	lock->lock();
//	printf("Thread %d offline\n", threadNum);
//	lock->unlock();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::BackwardThreadWorker(int threadNum, int depth,
																											NBitArray<pdbBits> &DB,
																											std::vector<bool> &coarse,
																											SharedQueue<std::pair<uint64_t, uint64_t> > *work,
																											SharedQueue<uint64_t> *results,
																											std::mutex *lock)
{
	std::pair<uint64_t, uint64_t> p;
	uint64_t start, end;
	std::vector<abstractAction> acts;
	abstractState s(goalState), t(goalState);
	uint64_t count = 0;
	int blankEntries = 0;
	struct writeInfo {
		uint64_t rank;
		int newGCost;
	};
	std::vector<writeInfo> cache;
	while (true)
	{
		work->WaitRemove(p);
		if (p.first == p.second)
		{
			break;
		}
		start = p.first;
		end = p.second;
		//int nextDepth = 255;
		blankEntries = 0;
		for (uint64_t x = start; x < end; x++)
		{
			int stateDepth = DB.Get(x);
			if (stateDepth == ((1<<pdbBits)-1))//depth) // pdbBits
			{
				blankEntries++;
				GetStateFromPDBHash(x, s, threadNum);
				//std::cout << "Expanding[r][" << stateDepth << "]: " << s << std::endl;
				env->GetActions(s, acts);
				for (int y = 0; y < acts.size(); y++)
				{
					env->GetNextState(s, acts[y], t);
					//assert(env->InvertAction(acts[y]) == true);
					//virtual bool InvertAction(action &a) const = 0;

					uint64_t nextRank = GetPDBHash(t, threadNum);
					if (DB.Get(nextRank) == depth)
					{
						int newCost = depth+(env->GCost(t, acts[y]));
						cache.push_back({x, newCost});
						blankEntries--;
						break;
					}
				}
			}
		}
		// write out everything
		if (cache.size() > 0)
		{
			//printf("%d items to write\n", cache.size());
			lock->lock();
			if (blankEntries == 0)
				coarse[start/coarseSize] = true; // closed
			for (auto d : cache)
			{
				if (d.newGCost < DB.Get(d.rank)) // shorter path
				{
					count++;
					DB.Set(d.rank, d.newGCost);
				}
			}
			lock->unlock();
		}
		cache.resize(0);
	}
	results->Add(count);
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ForwardBackwardThreadWorker(int threadNum, int depth, bool forward,
																												   NBitArray<pdbBits> &DB,
																												   std::vector<bool> &coarseOpen,
																												   std::vector<bool> &coarseClosed,
																												   SharedQueue<std::pair<uint64_t, uint64_t> > *work,
																												   SharedQueue<uint64_t> *results,
																												   std::mutex *lock)
{
	std::pair<uint64_t, uint64_t> p;
	uint64_t start, end;
	std::vector<abstractAction> acts;
	abstractState s(goalState), t(goalState);
	uint64_t count = 0;
	int blankEntries = 0;
	struct writeInfo {
		uint64_t rank;
		int newGCost;
	};
	std::vector<writeInfo> cache;
	if (forward)
	{
		bool allEntriesWritten;
		while (true)
		{
			work->WaitRemove(p);
			if (p.first == p.second)
			{
				break;
			}
			start = p.first;
			end = p.second;

			allEntriesWritten = true;
			for (uint64_t x = start; x < end; x++)
			{
				int stateDepth = DB.Get(x);
				if (stateDepth > depth)
					allEntriesWritten = false;
				if (stateDepth == depth)
				{
					GetStateFromPDBHash(x, s, threadNum);
					//std::cout << "Expanding[r][" << stateDepth << "]: " << s << std::endl;
					env->GetActions(s, acts);
					for (int y = 0; y < acts.size(); y++)
					{
						env->GetNextState(s, acts[y], t);
						assert(env->InvertAction(acts[y]) == true);
						//virtual bool InvertAction(action &a) const = 0;
						
						uint64_t nextRank = GetPDBHash(t, threadNum);
						int newCost = stateDepth+(env->GCost(t, acts[y]));
						cache.push_back({nextRank, newCost});
					}
				}
			}
			// write out everything
			lock->lock();
			if (allEntriesWritten)
				coarseClosed[start/coarseSize] = true;
			for (auto d : cache)
			{
				if (d.newGCost < DB.Get(d.rank)) // shorter path
				{
					count++;
					coarseOpen[d.rank/coarseSize] = true;
					DB.Set(d.rank, d.newGCost);
				}
			}
			lock->unlock();
			cache.resize(0);
		}
	}
	else {
		while (true)
		{
			work->WaitRemove(p);
			if (p.first == p.second)
			{
				break;
			}
			start = p.first;
			end = p.second;
			//int nextDepth = 255;
			blankEntries = 0;
			for (uint64_t x = start; x < end; x++)
			{
				int stateDepth = DB.Get(x);
				if (stateDepth == ((1<<pdbBits)-1))//depth) // pdbBits
				{
					blankEntries++;
					GetStateFromPDBHash(x, s, threadNum);
					//std::cout << "Expanding[r][" << stateDepth << "]: " << s << std::endl;
					env->GetActions(s, acts);
					for (int y = 0; y < acts.size(); y++)
					{
						env->GetNextState(s, acts[y], t);
						//assert(env->InvertAction(acts[y]) == true);
						//virtual bool InvertAction(action &a) const = 0;
						
						uint64_t nextRank = GetPDBHash(t, threadNum);
						if (DB.Get(nextRank) == depth)
						{
							int newCost = depth+(env->GCost(t, acts[y]));
							cache.push_back({x, newCost});
							blankEntries--;
							break;
						}
					}
				}
			}
			// write out everything
			if (cache.size() > 0)
			{
				//printf("%d items to write\n", cache.size());
				lock->lock();
				if (blankEntries == 0)
					coarseClosed[start/coarseSize] = true; // closed
				for (auto d : cache)
				{
					if (d.newGCost < DB.Get(d.rank)) // shorter path
					{
						count++;
						DB.Set(d.rank, d.newGCost);
					}
				}
				lock->unlock();
			}
			cache.resize(0);
		}
	}
	results->Add(count);
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::BuildAdditivePDB(state &goal, const char *pdb_filename, int numThreads)
{
	assert(!"Not currently implemented.");
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::DivCompress(int factor, bool print_histogram)
{
	type = kDivCompress;
	compressionValue = factor;
	NBitArray<pdbBits> copy(PDB);
	PDB.Resize((PDB.Size()+compressionValue-1)/compressionValue);
	PDB.FillMax();
	for (uint64_t x = 0; x < copy.Size(); x++)
	{
		uint64_t newIndex = x/compressionValue;
		PDB.Set(newIndex, std::min(copy.Get(x), PDB.Get(newIndex)));
	}
	if (print_histogram)
		PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ModCompress(int factor, bool print_histogram)
{
	ModCompress((PDB.Size()+factor-1)/factor, print_histogram);
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ModCompress(uint64_t newEntries, bool print_histogram)
{
	type = kModCompress;
	compressionValue = newEntries;
	NBitArray<pdbBits> copy(PDB);
	PDB.Resize(compressionValue);
	PDB.FillMax();
	for (uint64_t x = 0; x < copy.Size(); x++)
	{
		uint64_t newIndex = x%compressionValue;
		PDB.Set(newIndex, std::min(copy.Get(x), PDB.Get(newIndex)));
	}
	if (print_histogram)
		PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::DeltaCompress(Heuristic<state> *h, state goal, bool print_histogram)
{
	abstractState s;
	state s1;
	for (int x = 0; x < PDB.Size(); x++)
	{
		GetStateFromPDBHash(x, s);
		s1 = GetStateFromAbstractState(s);
		PDB.Set(x, PDB.Get(x) - h->HCost(s1, goal));
	}
	if (print_histogram)
		PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::FractionalDivCompress(uint64_t count, bool print_histogram)
{
	assert(!"Not currently implemented.");
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::FractionalModCompress(uint64_t factor, bool print_histogram)
{
	assert(!"Not currently implemented.");
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ValueCompress(int maxValue, bool print_histogram)
{
	assert(!"Not currently implemented.");
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ValueCompress(std::vector<int> cutoffs, bool print_histogram)
{
	std::vector<uint64_t> dist;
	if (print_histogram)
	{
		printf("Setting boundaries [%d values]: ", cutoffs.size());
		for (int x = 0; x < cutoffs.size(); x++)
			printf("%d ", cutoffs[x]);
		printf("\n");
	}
	cutoffs.push_back(256);
	
	for (uint64_t x = 0; x < PDB.Size(); x++)
	{
		for (int y = 0; y < cutoffs.size(); y++)
		{
			if (PDB.Get(x) >= cutoffs[y] && PDB.Get(x) < cutoffs[y+1])
			{
				//printf("%d -> %d\n", PDB[whichPDB][x], cutoffs[y]);
				PDB.Set(x, cutoffs[y]);
				break;
			}
		}
	}
	if (print_histogram)
		PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::CustomValueRangeCompress(std::vector<uint64_t> dist, int numBits, bool print_histogram)
{
	std::vector<int> cutoffs;
	GetOptimizedBoundaries(dist, 1<<numBits, cutoffs);
	if (print_histogram)
	{
		printf("Setting boundaries [%d values]: ", (1<<numBits));
		for (int x = 0; x < cutoffs.size(); x++)
			printf("%d ", cutoffs[x]);
		printf("\n");
	}
	cutoffs.push_back(256);
	
	for (uint64_t x = 0; x < PDB.Size(); x++)
	{
		for (int y = 0; y < cutoffs.size(); y++)
		{
			if (PDB.Get(x) >= cutoffs[y] && PDB.Get(x) < cutoffs[y+1])
			{
				//printf("%d -> %d\n", PDB[whichPDB][x], cutoffs[y]);
				PDB.Set(x, cutoffs[y]);
				break;
			}
		}
	}
	if (print_histogram)
		PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ValueRangeCompress(int numBits, bool print_histogram)
{
	std::vector<uint64_t> dist;
	std::vector<int> cutoffs;
	GetHistogram(dist);
	GetOptimizedBoundaries(dist, 1<<numBits, cutoffs);
	if (print_histogram)
	{
		printf("Setting boundaries [%d values]: ", (1<<numBits));
		for (int x = 0; x < cutoffs.size(); x++)
			printf("%d ", cutoffs[x]);
		printf("\n");
	}
	cutoffs.push_back(256);
	
	for (uint64_t x = 0; x < PDB.Size(); x++)
	{
		for (int y = 0; y < cutoffs.size(); y++)
		{
			if (PDB.Get(x) >= cutoffs[y] && PDB.Get(x) < cutoffs[y+1])
			{
				//printf("%d -> %d\n", PDB[whichPDB][x], cutoffs[y]);
				PDB.Set(x, cutoffs[y]);
				break;
			}
		}
	}
	if (print_histogram)
		PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ValueRangeCompress(
																										  PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, 5> *newPDB,
																										  bool print_histogram)
{
	std::vector<uint64_t> dist;
	std::vector<int> cutoffs;
	GetHistogram(dist);
	GetOptimizedBoundaries(dist, 1<<5, cutoffs);
	if (print_histogram)
	{
		printf("Setting boundaries [%d values]: ", (1<<4));
		for (int x = 0; x < cutoffs.size(); x++)
			printf("%d ", cutoffs[x]);
		printf("\n");
	}
	if (type == kPlain)
		newPDB->type = kValueCompress;
	else if (type == kDivCompress)
	{
		newPDB->type = kDivPlusValueCompress;
		newPDB->compressionValue = compressionValue;
	}
	else {
		printf("Unknown PDB type: %d\n", type);
	}
	for (int x = 0; x < cutoffs.size(); x++)
		newPDB->vrcValues[x] = cutoffs[x];
	newPDB->PDB.Resize(PDB.Size());
	cutoffs.push_back(256);
	for (uint64_t x = 0; x < PDB.Size(); x++)
	{
		for (int y = 0; y < cutoffs.size(); y++)
		{
			if (PDB.Get(x) >= cutoffs[y] && PDB.Get(x) < cutoffs[y+1])
			{
				//printf("%d -> %d\n", PDB[whichPDB][x], cutoffs[y]);
				newPDB->PDB.Set(x, y);
				break;
			}
		}
	}
	if (print_histogram)
		newPDB->PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ValueRangeCompress(
												PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, 4> *newPDB,
																										   bool print_histogram)
{
	std::vector<uint64_t> dist;
	std::vector<int> cutoffs;
	GetHistogram(dist);
	GetOptimizedBoundaries(dist, 1<<4, cutoffs);
	if (print_histogram)
	{
		printf("Setting boundaries [%d values]: ", (1<<4));
		for (int x = 0; x < cutoffs.size(); x++)
			printf("%d ", cutoffs[x]);
		printf("\n");
	}
	if (type == kPlain)
		newPDB->type = kValueCompress;
	else if (type == kDivCompress)
	{
		newPDB->type = kDivPlusValueCompress;
		newPDB->compressionValue = compressionValue;
	}
	else {
		printf("Unknown PDB type: %d\n", type);
	}
	for (int x = 0; x < cutoffs.size(); x++)
		newPDB->vrcValues[x] = cutoffs[x];
	newPDB->PDB.Resize(PDB.Size());
	cutoffs.push_back(256);
	for (uint64_t x = 0; x < PDB.Size(); x++)
	{
		for (int y = 0; y < cutoffs.size(); y++)
		{
			if (PDB.Get(x) >= cutoffs[y] && PDB.Get(x) < cutoffs[y+1])
			{
				//printf("%d -> %d\n", PDB[whichPDB][x], cutoffs[y]);
				newPDB->PDB.Set(x, y);
				break;
			}
		}
	}
	if (print_histogram)
		newPDB->PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ValueRangeCompress(PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, 3> *newPDB, bool print_histogram)
{
	std::vector<uint64_t> dist;
	std::vector<int> cutoffs;
	GetHistogram(dist);
	GetOptimizedBoundaries(dist, 1<<3, cutoffs);
	if (print_histogram)
	{
		printf("Setting boundaries [%d values]: ", (1<<4));
		for (int x = 0; x < cutoffs.size(); x++)
			printf("%d ", cutoffs[x]);
		printf("\n");
	}
	if (type == kPlain)
		newPDB->type = kValueCompress;
	else if (type == kDivCompress)
	{
		newPDB->type = kDivPlusValueCompress;
		newPDB->compressionValue = compressionValue;
	}
	else {
		printf("Unknown PDB type: %d\n", type);
	}

	for (int x = 0; x < cutoffs.size(); x++)
		newPDB->vrcValues[x] = cutoffs[x];
	newPDB->PDB.Resize(PDB.Size());
	cutoffs.push_back(256);
	for (uint64_t x = 0; x < PDB.Size(); x++)
	{
		for (int y = 0; y < cutoffs.size(); y++)
		{
			if (PDB.Get(x) >= cutoffs[y] && PDB.Get(x) < cutoffs[y+1])
			{
				//printf("%d -> %d\n", PDB[whichPDB][x], cutoffs[y]);
				newPDB->PDB.Set(x, y);
				break;
			}
		}
	}
	if (print_histogram)
		newPDB->PrintHistogram();

}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ValueRangeCompress(PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, 2> *newPDB, bool print_histogram)
{
	std::vector<uint64_t> dist;
	std::vector<int> cutoffs;
	GetHistogram(dist);
	GetOptimizedBoundaries(dist, 1<<2, cutoffs);
	if (print_histogram)
	{
		printf("Setting boundaries [%d values]: ", (1<<4));
		for (int x = 0; x < cutoffs.size(); x++)
			printf("%d ", cutoffs[x]);
		printf("\n");
	}
	if (type == kPlain)
		newPDB->type = kValueCompress;
	else if (type == kDivCompress)
	{
		newPDB->type = kDivPlusValueCompress;
		newPDB->compressionValue = compressionValue;
	}
	else {
		printf("Unknown PDB type: %d\n", type);
	}

	for (int x = 0; x < cutoffs.size(); x++)
		newPDB->vrcValues[x] = cutoffs[x];
	newPDB->PDB.Resize(PDB.Size());
	cutoffs.push_back(256);
	for (uint64_t x = 0; x < PDB.Size(); x++)
	{
		for (int y = 0; y < cutoffs.size(); y++)
		{
			if (PDB.Get(x) >= cutoffs[y] && PDB.Get(x) < cutoffs[y+1])
			{
				//printf("%d -> %d\n", PDB[whichPDB][x], cutoffs[y]);
				newPDB->PDB.Set(x, y);
				break;
			}
		}
	}
	if (print_histogram)
		newPDB->PrintHistogram();
	
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ValueRangeCompress(PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, 1> *newPDB, bool print_histogram)
{
	std::vector<uint64_t> dist;
	std::vector<int> cutoffs;
	GetHistogram(dist);
	GetOptimizedBoundaries(dist, 1<<1, cutoffs);
	if (print_histogram)
	{
		printf("Setting boundaries [%d values]: ", (1<<4));
		for (int x = 0; x < cutoffs.size(); x++)
			printf("%d ", cutoffs[x]);
		printf("\n");
	}
	if (type == kPlain)
		newPDB->type = kValueCompress;
	else if (type == kDivCompress)
	{
		newPDB->type = kDivPlusValueCompress;
		newPDB->compressionValue = compressionValue;
	}
	else {
		printf("Unknown PDB type: %d\n", type);
	}

	for (int x = 0; x < cutoffs.size(); x++)
		newPDB->vrcValues[x] = cutoffs[x];
	newPDB->PDB.Resize(PDB.Size());
	cutoffs.push_back(256);
	for (uint64_t x = 0; x < PDB.Size(); x++)
	{
		for (int y = 0; y < cutoffs.size(); y++)
		{
			if (PDB.Get(x) >= cutoffs[y] && PDB.Get(x) < cutoffs[y+1])
			{
				//printf("%d -> %d\n", PDB[whichPDB][x], cutoffs[y]);
				newPDB->PDB.Set(x, y);
				break;
			}
		}
	}
	if (print_histogram)
		newPDB->PrintHistogram();
	
}


template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
bool PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::Load(FILE *f)
{
	if (fread(&type, sizeof(type), 1, f) != 1)
		return false;
	if (fread(&goalState, sizeof(goalState), 1, f) != 1)
		return false;
	return PDB.Read(f);
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::Save(FILE *f)
{
	fwrite(&type, sizeof(type), 1, f);
	fwrite(&goalState, sizeof(goalState), 1, f);
	PDB.Write(f);
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::GetHistogram(std::vector<uint64_t> &histogram)
{
	histogram.resize(0);
	for (uint64_t x = 0; x < PDB.Size(); x++)
	{
		if (PDB.Get(x)+1 > histogram.size())
		{
			histogram.resize(PDB.Get(x)+1);
		}
		histogram[PDB.Get(x)]++;
	}
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
double PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::PrintHistogram()
{
	int factor = 1;
	if (type == kDivCompress || type == kDivPlusValueCompress)
		factor = compressionValue;
	double average = 0;
	std::vector<uint64_t> histogram;
	if (type == kValueCompress || type == kDivPlusValueCompress)
	{
		for (uint64_t x = 0; x < PDB.Size(); x++)
		{
			if (vrcValues[PDB.Get(x)]+1 > histogram.size())
			{
				histogram.resize(vrcValues[PDB.Get(x)]+1);
			}
			histogram[vrcValues[PDB.Get(x)]]+=factor;
			average += vrcValues[PDB.Get(x)];
		}
	}
	else {
		for (uint64_t x = 0; x < PDB.Size(); x++)
		{
			if (PDB.Get(x)+1 > histogram.size())
			{
				histogram.resize(PDB.Get(x)+1);
			}
			histogram[PDB.Get(x)]+=factor;
			average += PDB.Get(x);
		}
	}
	for (int x = 0; x < histogram.size(); x++)
	{
		if (histogram[x] > 0)
			printf("%d: %llu\n", x, histogram[x]);
	}
	printf("Average: %f; count: %llu\n", average/PDB.Size(), PDB.Size());
	return average/PDB.Size();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
double PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::GetAverageValue()
{
	double average = 0;
	for (uint64_t x = 0; x < PDB.Size(); x++)
	{
		average += PDB.Get(x);
	}
	return average/PDB.Size();
}

#endif
