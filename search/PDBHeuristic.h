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

enum PDBLookupType {
	kPlain,
	kMinCompress,
	kFractionalCompress,
	kFractionalModCompress,
	kModCompress,
	kValueCompress,
	kDivPlusDeltaCompress, // two lookups with the same index, one is div, one is delta
	kDefaultHeuristic
};

template <class state, class action, class environment, uint64_t pdbBits = 8>
class PDBHeuristic : public Heuristic<state> {
public:
	PDBHeuristic(environment *e) :type(kPlain), env(e) {}
	virtual ~PDBHeuristic() {}
	virtual double HCost(const state &a, const state &b) const;

//	virtual uint64_t GetStateHash(const state &s) const = 0;
//	virtual void GetStateFromHash(state &s, uint64_t hash) const = 0;
	virtual uint64_t GetPDBSize() const = 0;
	virtual uint64_t GetPDBHash(const state &s, int threadID = 0) const = 0;
	virtual void GetStateFromPDBHash(uint64_t hash, state &s, int threadID = 0) const = 0;

	virtual bool Load(const char *prefix) = 0;
	virtual void Save(const char *prefix) = 0;
	virtual bool Load(FILE *f);
	virtual void Save(FILE *f);
	virtual std::string GetFileName(const char *prefix) = 0;
	
	void BuildPDB(const state &goal, const char *pdb_filename, int numThreads);
	void BuildAdditivePDB(state &goal, const char *pdb_filename, int numThreads);

	void DivCompress(int factor, bool print_histogram);
	void ModCompressPDB(uint64_t newEntries, bool print_histogram);
	void FractionalDivCompress(uint64_t count, bool print_histogram);
	void FractionalModCompress(uint64_t factor, bool print_histogram);
	void ValueCompress(int maxValue, bool print_histogram);
	void ValueCompress(std::vector<int> cutoffs, bool print_histogram);
	void ValueRangeCompress(int numBits, bool print_histogram);
	void DeltaCompress(Heuristic<state> *h, state goal, bool print_histogram);
protected:
	// holds a Pattern Databases
	NBitArray<pdbBits> PDB;
	//std::vector<uint8_t> PDB;
	PDBLookupType type;
	const int coarseSize = 1024;
	environment *env;
	state goalState;
private:
	void ThreadWorker(int threadNum, int depth,
					  NBitArray<pdbBits> &DB,
					  //std::vector<uint8_t> &DB,
					  SharedQueue<std::pair<uint64_t, uint64_t> > *work,
					  SharedQueue<uint64_t> *results,
					  std::mutex *lock);
};

template <class state, class action, class environment, uint64_t pdbBits>
double PDBHeuristic<state, action, environment, pdbBits>::HCost(const state &a, const state &b) const
{
	switch (type)
	{
		case kPlain: return PDB.Get(GetPDBHash(a)); //PDB[GetPDBHash(a)];
		default:
			assert(!"Not implemented");
	}
}

template <class state, class action, class environment, uint64_t pdbBits>
void PDBHeuristic<state, action, environment, pdbBits>::BuildPDB(const state &goal, const char *pdb_filename, int numThreads)
{
	goalState = goal;
	SharedQueue<std::pair<uint64_t, uint64_t> > workQueue;
	SharedQueue<uint64_t> resultQueue;
	std::mutex lock;
	
	uint64_t COUNT = GetPDBSize();//nUpperk(start.puzzle.size(), start.puzzle.size() - distinct.size());
	//std::vector<uint8_t> &DB = PDB;
	PDB.Resize(COUNT);
	PDB.FillMax();
	//std::fill(DB.begin(), DB.end(), 255);
	
	// with weights we have to store the lowest weight stored to make sure
	// we don't skip regions
	std::vector<uint8_t> coarseOpen((COUNT+coarseSize-1)/coarseSize);
	std::fill(coarseOpen.begin(), coarseOpen.end(), 255);
	
	uint64_t entries = 0;
	std::cout << "Num Entries: " << COUNT << std::endl;
	std::cout << "Goal State: " << goal << std::endl;
	//std::cout << "State Hash of Goal: " << GetStateHash(goal) << std::endl;
	std::cout << "PDB Hash of Goal: " << GetPDBHash(goal) << std::endl;
	
	std::deque<state> q_curr, q_next;
	std::vector<state> children;
	
	std::cout << "Abstract Goal State: " << goal << std::endl;
	std::cout << "Abstract PDB Hash of Goal: " << GetPDBHash(goal) << std::endl;
	Timer t;
	t.StartTimer();
	//	q_curr.push_back(goal);
	//DB[GetPDBHash(goal)] = 0;
	PDB.Set(GetPDBHash(goal), 0);

	coarseOpen[GetPDBHash(goal)/coarseSize] = 0;
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
			threads[x] = new std::thread(&PDBHeuristic<state, action, environment, pdbBits>::ThreadWorker,
										 this,
										 x, depth, std::ref(PDB),// &coarseOpen,
										 &workQueue, &resultQueue, &lock);
		}
		
		for (uint64_t x = 0; x < COUNT; x+=coarseSize)
		{
			bool submit = false;
			for (uint64_t t = x; t < std::min(COUNT, x+coarseSize); t++)
			{
				if (PDB.Get(t) == depth)
				{
					submit = true;
					//newEntries++;
				}
			}
			if (submit)
			{
				while (workQueue.size() > 10*numThreads)
				{ std::this_thread::sleep_for(std::chrono::microseconds(10)); }
				workQueue.Add({x, std::min(COUNT, x+coarseSize)});
			}
		}
		for (int x = 0; x < numThreads; x++)
		{
			workQueue.Add({0,0});
		}
		for (int x = 0; x < numThreads; x++)
		{
			threads[x]->join();
			delete threads[x];
			threads[x] = 0;
		}
		// read out node counts
		while (true)
		{
			uint64_t val;
			if (resultQueue.Remove(val))
			{
				//newEntries+=val;
			}
			else {
				break;
			}
		}
		
		newEntries = 0;
		for (uint64_t x = 0; x < COUNT; x++)
		{
			if (PDB.Get(x) == depth)
			{
				newEntries++;
			}
		}
		entries += newEntries;
		printf("Depth %d complete; %1.2fs elapsed. %llu new states seen; %llu of %llu total\n",
			   depth, s.EndTimer(), newEntries, entries, COUNT);
		depth++;
		//		depth = coarseOpen[0];
		//		for (int x = 1; x < coarseOpen.size(); x++)
		//			depth = min(depth, coarseOpen[x]);
		//		if (depth == 255) // no new entries!
		//			break;
	} while (entries != COUNT);
	
	printf("%1.2fs elapsed\n", t.EndTimer());
	if (entries != COUNT)
	{
		printf("Entries: %llu; count: %llu\n", entries, COUNT);
		assert(entries == COUNT);
	}
	
	if (pdb_filename != 0)
	{
		//TODO fix the output of PDBs
//		FILE *f = fopen(pdb_filename, "w");
		//	int num = distinct.size();
//		WritePDBHeader(f);
		//	assert(fwrite(&num, sizeof(num), 1, f) == 1);
		//	assert(fwrite(&distinct[0], sizeof(distinct[0]), distinct.size(), f) == distinct.size());
		//PDB.Write(pdb_filename);
//		assert(fwrite(&DB[0], sizeof(uint8_t), COUNT, f) == COUNT);
//		fclose(f);
	}
	
//	PDB.push_back(DB); // increase the number of regular PDBs being stored
//	PDB_distincts.push_back(distinct); // stores distinct
	//PrintPDBHistogram();

}

template <class state, class action, class environment, uint64_t pdbBits>
void PDBHeuristic<state, action, environment, pdbBits>::ThreadWorker(int threadNum, int depth,
																	 NBitArray<pdbBits> &DB,
																	 //std::vector<uint8_t> &DB,
																	 SharedQueue<std::pair<uint64_t, uint64_t> > *work,
																	 SharedQueue<uint64_t> *results,
																	 std::mutex *lock)
{
	std::pair<uint64_t, uint64_t> p;
	uint64_t start, end;
	std::vector<action> acts;
	state s, t;
	uint64_t count = 0;
	
	struct writeInfo {
		uint64_t rank;
		int newGCost;
	};
	std::vector<writeInfo> cache;
	while (true)
	{
		if (work->Remove(p) == false)
		{
			std::this_thread::sleep_for(std::chrono::microseconds(10));
			continue;
		}
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
				count++;
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
		do {
			// write out everything
			lock->lock();
			for (auto d : cache)
			{
				if (d.newGCost < DB.Get(d.rank)) // shorter path
				{
					//DB[d.rank] = d.newGCost;
					DB.Set(d.rank, d.newGCost);
				}
			}
			lock->unlock();
			cache.resize(0);
		} while (cache.size() != 0);
	}
	results->Add(count);
}

template <class state, class action, class environment, uint64_t pdbBits>
void PDBHeuristic<state, action, environment, pdbBits>::BuildAdditivePDB(state &goal, const char *pdb_filename, int numThreads)
{
	assert(!"Not currently implemented.");
}

template <class state, class action, class environment, uint64_t pdbBits>
void PDBHeuristic<state, action, environment, pdbBits>::DivCompress(int factor, bool print_histogram)
{
	assert(!"Not currently implemented.");
}

template <class state, class action, class environment, uint64_t pdbBits>
void PDBHeuristic<state, action, environment, pdbBits>::ModCompressPDB(uint64_t newEntries, bool print_histogram)
{
	assert(!"Not currently implemented.");
}

template <class state, class action, class environment, uint64_t pdbBits>
void PDBHeuristic<state, action, environment, pdbBits>::FractionalDivCompress(uint64_t count, bool print_histogram)
{
	assert(!"Not currently implemented.");
}

template <class state, class action, class environment, uint64_t pdbBits>
void PDBHeuristic<state, action, environment, pdbBits>::FractionalModCompress(uint64_t factor, bool print_histogram)
{
	assert(!"Not currently implemented.");
}

template <class state, class action, class environment, uint64_t pdbBits>
void PDBHeuristic<state, action, environment, pdbBits>::ValueCompress(int maxValue, bool print_histogram)
{
	assert(!"Not currently implemented.");
}

template <class state, class action, class environment, uint64_t pdbBits>
void PDBHeuristic<state, action, environment, pdbBits>::ValueCompress(std::vector<int> cutoffs, bool print_histogram)
{
	assert(!"Not currently implemented.");
}

template <class state, class action, class environment, uint64_t pdbBits>
void PDBHeuristic<state, action, environment, pdbBits>::ValueRangeCompress(int numBits, bool print_histogram)
{
	assert(!"Not currently implemented.");
}

template <class state, class action, class environment, uint64_t pdbBits>
void PDBHeuristic<state, action, environment, pdbBits>::DeltaCompress(Heuristic<state> *h, state goal, bool print_histogram)
{
	assert(!"Not currently implemented.");
}

template <class state, class action, class environment, uint64_t pdbBits>
bool PDBHeuristic<state, action, environment, pdbBits>::Load(FILE *f)
{
	if (fread(&type, sizeof(type), 1, f) != 1)
		return false;
	if (fread(&goalState, sizeof(goalState), 1, f) != 1)
		return false;
	return PDB.Read(f);
}

template <class state, class action, class environment, uint64_t pdbBits>
void PDBHeuristic<state, action, environment, pdbBits>::Save(FILE *f)
{
	fwrite(&type, sizeof(type), 1, f);
	fwrite(&goalState, sizeof(goalState), 1, f);
	PDB.Write(f);
}

#endif
