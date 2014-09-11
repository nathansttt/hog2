#include "SearchEnvironment.h"
#include <assert.h>
#include <deque>
#include <iostream>
#include <fstream>
#include <map>
#include <stdint.h>
#include <cstdlib>
#include <cstdio>
#include "Timer.h"
#include <pthread.h>

#ifndef PERMPUZZ_H
#define PERMPUZZ_H

template <class state, class action>
class PermutationPuzzleEnvironment;

enum PDBTreeNodeType {
	kMaxNode,
	kAddNode,
	kLeafNode
};

struct PDBTreeNode
{
	PDBTreeNodeType t;
	uint8_t numChildren;
	uint8_t firstChildID;
	uint8_t PDBID;
};

const uint64_t kDone = -1;

template <class state, class action>
struct ThreadData
{
	// initialize data structure with (1) database pointer (2) depth (3) pointer to queue
	std::vector<uint64_t> &DB;
	int depth;
	std::vector<uint64_t> &workQueue;
	pthread_mutex_t &queueLock;
	pthread_mutex_t &writeLock;
	PermutationPuzzleEnvironment<state, action> *env;
};

/**
 Note, assumes that state has a public vector<int> called puzzle in which the
 permutation is held.
 **/
template <class state, class action>
class PermutationPuzzleEnvironment : public SearchEnvironment<state, action>
{
public:
	PermutationPuzzleEnvironment():maxItem(0), maxPattern(0) {}
	/**
	 Returns the value of n! / k!
	 **/
	uint64_t nUpperk(int n, int k) const;
	
	/**
	 Builds caches for nUpperk
	 **/
	void buildCaches() const;

	/**
	 Returns the Hash Value of the given state using the given set of distinct items
	 **/
	virtual uint64_t GetPDBHash(const state &s, const std::vector<int> &distinct) const;
	
	/**
	 Loads the Regular PDB into memory
	 **/
	void Load_Regular_PDB(const char *fname, state &goal, bool print_histogram);
	
	/**
	 Performs a PDB lookup for the given state (additive or max is automatic)
	 **/
	double PDB_Lookup(const state &s);

	/**
	 Builds a regular PDB given the file name of the file to write the PDB to, and a list of distinct tiles
	 **/
	void Build_PDB(state &start, const std::vector<int> &distinct, const char *pdb_filename, bool additive);

	
	/**
	 Builds a regular PDB given the file name of the file to write the PDB to, and a list of distinct tiles
	 **/
	void Build_Regular_PDB(state &start, const std::vector<int> &distinct, const char *pdb_filename);
	
	/**
	 Builds a regular PDB given the file name of the file to write the PDB to, and a list of distinct tiles
	 **/
	void Build_Additive_PDB(state &start, const std::vector<int> &distinct, const char *pdb_filename, bool blank = false);

	/**
	 Loads a regular PDB given the file name of the file to read the PDB from, and a list of distinct tiles
	 **/
	void Load_Additive_PDB(const state &goal, const char *pdb_filename);

	
	/**
	 Returns a hash value for a permutation puzzle
	 **/
	virtual uint64_t GetStateHash(const state &s) const;
	
	/**
	 Constructs a state from a hash value
	 **/
	virtual void GetStateFromHash(state &s, uint64_t hash);
	
	/**
	 Returns the val! if it can fit in an uint64_t, otherwise returns the max
	 possible value minus 1
	 **/
	uint64_t Factorial(int val) const;
	
	/**
	 Constructs a random permutation and returns it.
	 **/
	static std::vector<int> Get_Random_Permutation(unsigned size);
	
	double HCost(const state &s);
private:
	double HCost(const state &s, int treeNode);
public:
	/**
	 Checks that the given state is a valid state for this domain. Note, is
	 not a check of solvability, just legality (ie. has the right size, dimensions).
	 Note, this method should be a quick check that can be used for debugging
	 purposes and so should not call Check_Permutation which is O(state size)
	 **/
	virtual bool State_Check(const state &to_check) = 0;
	
	/**
	 Ensures that the state contains a valid permutation. That is, if the state is of size
	 n, each of the integers from 0 to n-1 occurs exactly once.
	 **/
	static bool Check_Permutation(const std::vector<int> &to_check);
	
	bool Validate_Problems(std::vector<state> &puzzles)
	{
		for (unsigned i = 0; i < puzzles.size(); i++)
		{
			if (!State_Check(puzzles[i]) || !Check_Permutation(puzzles[i].puzzle))
			{
				std::cerr << puzzles[i] << '\n';
				std::cerr << "Invalid Puzzle\n";
			}
		}
		return true;
	}
	
	/**
	 Outputs the set of puzzles in puzzle_vector to standard output. The output is of the
	 form "I_0 I_1 ... I_(MN - 1)" where I_K is the element in the Kth position of the
	 puzzle. If the write_puzz_num flag is set as true, the first item of the output is
	 the puzzle number. Puzzles are checked for validity before they are outputted.
	 True is return if all the puzzles are valid, otherwise false is returned and
	 the puzzles are not outputted.
	 **/
	static bool Output_Puzzles(std::vector<state> &puzzle_vector, bool write_puzz_num);
	
	static bool Read_In_Permutations(const char *filename, unsigned size, unsigned max_puzzles, std::vector<std::vector<int> > &permutations, bool puzz_num_start);
	
	void CreateThreads(int count, int depth);
	void SendWorkToThread(uint64_t index);
	void DoneWithThreads();
	void WaitForThreads();

	bool additive;
	int maxItem, maxPattern;
	// holds a set of Pattern Databases which can be maxed over later
	std::vector<std::vector<uint8_t> > PDB;
	// holds the set of distinct items used to build the associated PDB (and therefore needed for hashing)
	std::vector<std::vector<int> > PDB_distincts;
	std::vector<PDBTreeNode> lookups;
	std::vector<int> histogram;

	std::vector<pthread_t> threads;
	pthread_mutex_t queueLock;
	pthread_mutex_t writeLock;
	std::vector<uint64_t> workQueue;
	mutable std::vector<std::vector<uint64_t> > factorialCache;
};

template <class state, class action>
std::vector<int> PermutationPuzzleEnvironment<state, action>::Get_Random_Permutation(unsigned size)
{	std::vector<int> permutation;
	
	for (unsigned i = 0; i < size; i++)
	{
		permutation.push_back(i);
	}
	int index = 0;
	int temp;
	
	// randomizes elements in the permutation
	while(size > 1)
	{		index = rand() % size;
		temp = permutation[size - 1];
		permutation[size - 1] = permutation[index];
		permutation[index] = temp;
		
		size--;
	}
	
	return permutation;
}

template <class state, class action>
void PermutationPuzzleEnvironment<state, action>::GetStateFromHash(state &s, uint64_t hash)
{
	std::vector<int> puzzle = s.puzzle;
	uint64_t hashVal = hash;
	
	int numEntriesLeft = 1;
	for (int x = s.puzzle.size()-1; x >= 0; x--)
	{
		puzzle[x] = hashVal%numEntriesLeft;
		hashVal /= numEntriesLeft;
		numEntriesLeft++;
		for (int y = x+1; y < (int) s.puzzle.size(); y++)
		{
			if (puzzle[y] >= puzzle[x])
				puzzle[y]++;
		}
	}
	
	s.puzzle = puzzle;
}

template <class state, class action>
uint64_t PermutationPuzzleEnvironment<state, action>::GetStateHash(const state &s) const
{
	std::vector<int> puzzle = s.puzzle;
	uint64_t hashVal = 0;
	int numEntriesLeft = s.puzzle.size();
	for (unsigned int x = 0; x < s.puzzle.size(); x++)
	{
		hashVal += puzzle[x]*Factorial(numEntriesLeft-1);
		numEntriesLeft--;
		for (unsigned y = x; y < puzzle.size(); y++)
		{
			if (puzzle[y] > puzzle[x])
				puzzle[y]--;
		}
	}
	return hashVal;
}

template <class state, class action>
uint64_t PermutationPuzzleEnvironment<state, action>::Factorial(int val) const
{
	static uint64_t table[21] =
	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
	if (val > 20)
		return (uint64_t)-1;
	return table[val];
}

template <class state, class action>
void PermutationPuzzleEnvironment<state, action>::buildCaches() const
{
	factorialCache.resize(maxItem+1);
	for (int n = 0; n < factorialCache.size(); n++)
	{
		factorialCache[n].resize(maxItem-maxPattern+1);
		for (int k = 0; k < factorialCache[n].size(); k++)
		{
			uint64_t value = 1;
			for (int i = n; i > k; i--)
			{
				value *= i;
			}
			factorialCache[n][k] = value;

		}
	}
}


template <class state, class action>
uint64_t PermutationPuzzleEnvironment<state, action>::nUpperk(int n, int k) const
{
	assert(factorialCache.size() > n);
	assert(factorialCache[n].size() > k);
	return factorialCache[n][k];

//	static std::vector<std::vector<uint64_t> > cache;
//	if (n >= cache.size())
//		cache.resize(n+1);
//	if (k >= cache[n].size())
//		cache[n].resize(k+1);
//	if (cache[n][k] == 0)
//	{
//		uint64_t value = 1;
//		assert(n >= 0 && k >= 0);
//		
//		for (int i = n; i > k; i--)
//		{
//			value *= i;
//		}
//	
//		cache[n][k] = value;
//		return value;
//	}
//	return cache[n][k];
}

// TODO Change to Myrvold and Ruskey ranking function
template <class state, class action>
uint64_t PermutationPuzzleEnvironment<state, action>::GetPDBHash(const state &s, const std::vector<int> &distinct) const
{
	static std::vector<int> locs;
	static std::vector<int> dual;
	locs.resize(distinct.size()); // vector for distinct item locations
	dual.resize(s.puzzle.size()); // vector for distinct item locations
	
	// find item locations
	for (unsigned int x = 0; x < s.puzzle.size(); x++)
	{
		dual[s.puzzle[x]] = x;
	}
	for (int x = 0; x < distinct.size(); x++)
	{
		locs[x] = dual[distinct[x]];
	}
	
	uint64_t hashVal = 0;
	int numEntriesLeft = s.puzzle.size();
	
	for (unsigned int x = 0; x < locs.size(); x++)
	{
		hashVal += locs[x]*nUpperk(numEntriesLeft-1, s.puzzle.size()-distinct.size());
		numEntriesLeft--;
		
		// decrement locations of remaining items
		for (unsigned y = x; y < locs.size(); y++)
		{
			if (locs[y] > locs[x])
				locs[y]--;
		}
	}
	return hashVal;
}

template <class state, class action>
void PermutationPuzzleEnvironment<state, action>::Load_Regular_PDB(const char *fname, state &goal, bool print_histogram)
{
	additive = false;
	PDB.resize(PDB.size()+1); // increase the number of regular PDBs being stored
	printf("Loading PDB '%s'\n", fname);
	std::vector<int> distinct;
	
	FILE *f;
	f = fopen(fname, "r");
	if (f == 0)
	{
		printf("Failed to open pdb '%s'\n", fname);
		exit(0);
	}
	
	int num_distinct;
	assert(fread(&num_distinct, sizeof(num_distinct), 1, f) == 1);
	distinct.resize(num_distinct);
	assert(fread(&distinct[0], sizeof(distinct[0]), distinct.size(), f) == distinct.size());
	
	maxItem = max(maxItem,goal.puzzle.size());
	maxPattern = max(maxPattern, distinct.size());
	buildCaches();
	
	uint64_t COUNT = nUpperk(goal.puzzle.size(), goal.puzzle.size() - distinct.size());
	PDB.back().resize(COUNT);
	
	size_t index;
	if ((index = fread(&PDB.back()[0], sizeof(uint8_t), COUNT, f)) != COUNT)
	{
		printf("Error; did not correctly read %lu entries from PDB (%lu instead)\n", COUNT, index);
		exit(0);
	}
	fclose(f);
	
	PDB_distincts.push_back(distinct); // stores distinct
	
	if (1)//print_histogram)
	{
		uint8_t maxval = 0;
		std::vector<int> values(256);
		// performs histogram count
		for (unsigned int x = 0; x < COUNT; x++)
		{
			values[(PDB.back()[x])]++;
			maxval = max(maxval, PDB.back()[x]);
		}
		// outputs histogram of heuristic value counts
		histogram.resize(max(histogram.size(),maxval+1));
		for (int x = 0; x <= maxval; x++)
		{
			printf("%d:\t%d\n", x, values[x]);
		}
	}
}


template <class state, class action>
double PermutationPuzzleEnvironment<state, action>::PDB_Lookup(const state &s)
{
	if (!additive)
	{
		double val = 0;
		for (unsigned int x = 0; x < PDB.size(); x++)
		{
			uint64_t index = GetPDBHash(s, PDB_distincts[x]);
			histogram[PDB[x][index]]++;
			val = std::max(val, (double)PDB[x][index]);
		}
		return val;
	}
	else {
		double val = 0;
		uint8_t tmp;
		for (unsigned int x = 0; x < PDB.size(); x++)
		{
			uint64_t index = GetPDBHash(s, PDB_distincts[x]);
			histogram[PDB[x][index]]++;
			tmp = PDB[x][index];
			if (tmp > 4) tmp = 4;
			val += (double)tmp;
		}
		return val;
	}
}

template <class state, class action>
void *ThreadedWorker(void *arg)
{
//	ThreadData<state, action> *d = (ThreadData<state, action>*)arg;
//	std::vector<uint64_t> myQueue;
//	state s;
//	std::vector<state> children;
//	while (true)
//	{
//		pthread_mutex_lock(&d->queueLock);
//		for (int x = 0; x < 10 && d->workQueue.size() > 0; x++)
//		{
//			myQueue.push_back(d->workQueue.back());
//			d->workQueue.pop_back();
//		}
//		pthread_mutex_unlock(&d->queueLock);
//
//		while (myQueue.size() > 0)
//		{
//			// expand all states
//			uint64_t next = myQueue.back();
//			myQueue.pop_back();
//			
//			d->env->GetStateFromHash(s, next);
//			for (unsigned int x = 0; x < children.size(); x++)
//			{
//				if (DB[GetPDBHash(children[x], distinct)] == 255)
//				{
//					int hval = depth-this->HCost(children[x]);
//					assert(hval >= 0);
//					DB[GetPDBHash(children[x], distinct)] = hval;
//
//					q_next.push_back(children[x]);
//					entries++;
//					if ((entries % 100000) == 0)
//					{
//						std::cout << entries << std::endl;
//					}
//				}
//			}
//
//			// store neighbors
//			
//			// write all neighbors & update coarse open next
//			
//		}
//	}
}

template <class state, class action>
void PermutationPuzzleEnvironment<state, action>::CreateThreads(int count, int depth)
{
//	queueLock = PTHREAD_MUTEX_INITIALIZER;
//	writeLock = PTHREAD_MUTEX_INITIALIZER;
//
//	// initialize data structure with (1) database pointer (2) depth (3) pointer to queue
//	for (int x = 0; x < count; x++)
//	{
//		pthread_create(&threads[x], NULL, ThreadedWorker<state, action>, (void *)x);
//	}
//
}

template <class state, class action>
void PermutationPuzzleEnvironment<state, action>::SendWorkToThread(uint64_t index)
{
	pthread_mutex_lock(&queueLock);
	workQueue.push_back(index);
	pthread_mutex_unlock(&queueLock);
}

template <class state, class action>
void PermutationPuzzleEnvironment<state, action>::DoneWithThreads()
{
//	pthread_mutex_lock(&queueLock);
//	for (int x = 0; x < numThreads; x++)
//		workQueue.push_back(kDone);
//	pthread_mutex_unlock(&queueLock);
}


template <class state, class action>
void PermutationPuzzleEnvironment<state, action>::WaitForThreads()
{
	for (int x = 0; x < threads.size(); x++)
	{
		int result = pthread_join(threads[x], NULL);
		if (result != 0)
		{
			printf("Unknown error joining with thread %d\n", x);
		}
	}
}


template <class state, class action>
void PermutationPuzzleEnvironment<state, action>::Build_PDB(state &start, const std::vector<int> &distinct, const char *pdb_filename, bool additive)
{
//	maxItem = max(maxItem,start.puzzle.size());
//	maxPattern = max(maxPattern, distinct.size());
//	buildCaches();
//	
//	uint64_t COUNT = nUpperk(start.puzzle.size(), start.puzzle.size() - distinct.size());
//	std::vector<uint8_t> DB(COUNT);
//	
//	for (unsigned int x = 0; x < DB.size(); x++)
//		DB[x] = 255;
//	
//	std::vector<bool> coarseOpenCurr((COUNT+1023)/1024);
//	std::vector<bool> coarseOpenNext((COUNT+1023)/1024);
//	
//	uint64_t entries = 0;
//	std::cout << "Num Entries: " << COUNT << std::endl;
//	std::cout << "Goal State: " << start << std::endl;
//	std::cout << "State Hash of Goal: " << GetStateHash(start) << std::endl;
//	std::cout << "PDB Hash of Goal: " << GetPDBHash(start, distinct) << std::endl;
//	
//	std::deque<state> q_curr, q_next;
//	std::vector<state> children;
//	
//	for (unsigned i = 0; i < start.puzzle.size(); i++)
//	{
//		bool is_distinct = false;
//		for (unsigned j = 0; j < distinct.size(); j++)
//		{
//			if (start.puzzle[i] == distinct[j])
//			{
//				is_distinct = true;
//				break;
//			}
//		}
//		
//		if (!is_distinct)
//		{
//			start.puzzle[i] = -1;
//		}
//	}
//	
//	std::cout << "Abstract Goal State: " << start << std::endl;
//	std::cout << "Abstract PDB Hash of Goal: " << GetPDBHash(start, distinct) << std::endl;
//	Timer t;
//	t.StartTimer();
//	q_curr.push_back(start);
//	DB[GetPDBHash(start, distinct)] = 0;
//	coarseOpenCurr[GetPDBHash(start, distinct)/1024] = 1;
//	entries++;
//	int depth = 0;
//	do {
//		CreateThreads(count, depth+1);
//		for (uint64_t x = 0; x < COUNT; x++)
//		{
//			if (coarseOpenCurr[x/1024] == false)
//			{
//				x += 1023;
//				continue;
//			}
//			if (DB[x] == depth)
//				SendWorkToThread(x);
//		}
//		DoneWithThreads();
//		WaitForThreads();
//		coarseOpenCurr.swap(coarseOpenNext);
//		depth++;
//	} while (newEntries > 0);
//	
//	printf("%1.2fs elapsed\n", t.EndTimer());
//	if (entries != COUNT)
//	{
//		printf("Entries: %llu; count: %llu\n", entries, COUNT);
//		assert(entries == COUNT);
//	}
//	
//	//TODO fix the output of PDBs
//	FILE *f = fopen(pdb_filename, "w");
//	int num = distinct.size();
//	assert(fwrite(&num, sizeof(num), 1, f) == 1);
//	assert(fwrite(&distinct[0], sizeof(distinct[0]), distinct.size(), f) == distinct.size());
//	assert(fwrite(&DB[0], sizeof(uint8_t), COUNT, f) == COUNT);
//	fclose(f);
//
//	PDB.push_back(DB); // increase the number of regular PDBs being stored
//	PDB_distincts.push_back(distinct); // stores distinct
}

template <class state, class action>
void PermutationPuzzleEnvironment<state, action>::Build_Regular_PDB(state &start, const std::vector<int> &distinct, const char *pdb_filename)
{
	maxItem = max(maxItem,start.puzzle.size());
	maxPattern = max(maxPattern, distinct.size());
	buildCaches();

	uint64_t COUNT = nUpperk(start.puzzle.size(), start.puzzle.size() - distinct.size());
	std::vector<uint8_t> DB(COUNT);

	for (unsigned int x = 0; x < DB.size(); x++)
		DB[x] = 255;
	
	uint64_t entries = 0;
	std::cout << "Num Entries: " << COUNT << std::endl;
	std::cout << "Goal State: " << start << std::endl;
	std::cout << "State Hash of Goal: " << GetStateHash(start) << std::endl;
	std::cout << "PDB Hash of Goal: " << GetPDBHash(start, distinct) << std::endl;
	
	std::deque<state> q_curr, q_next;
	std::vector<state> children;
	
	for (unsigned i = 0; i < start.puzzle.size(); i++)
	{
		bool is_distinct = false;
		for (unsigned j = 0; j < distinct.size(); j++)
		{
			if (start.puzzle[i] == distinct[j])
			{
				is_distinct = true;
				break;
			}
		}
		
		if (!is_distinct)
		{
			start.puzzle[i] = -1;
		}
	}
	
	std::cout << "Abstract Goal State: " << start << std::endl;
	std::cout << "Abstract PDB Hash of Goal: " << GetPDBHash(start, distinct) << std::endl;
	Timer t;
	t.StartTimer();
	q_curr.push_back(start);
	DB[GetPDBHash(start, distinct)] = 0;
	entries++;
	int depth = 1;
	std::vector<uint64_t> depths(2);
	do {
		while (!q_curr.empty())
		{
			state next = q_curr.front();
			q_curr.pop_front();
			children.clear();
			this->GetSuccessors(next, children);
			for (unsigned int x = 0; x < children.size(); x++)
			{
				if (DB[GetPDBHash(children[x], distinct)] == 255)
				{
					int hval = depth-this->HCost(children[x]);
					assert(hval >= 0);
					DB[GetPDBHash(children[x], distinct)] = hval;
					depths[hval]++;

					q_next.push_back(children[x]);
					entries++;
					if ((entries % 100000) == 0)
					{
						std::cout << entries << std::endl;
					}
//					std::cout << children[x] << std::endl;
					/* For Debugging
					 if (entries < 20)
					 {
					 std::cout << children[x] << ": " << (unsigned) DB[GetPDBHash(children[x], distinct)] << "\n";
					 }*/
				}
			}
		}
		depth++;
		depths.resize(depth+1);
		q_curr.swap(q_next);
	} while (!q_curr.empty());
	
	printf("%1.2fs elapsed\n", t.EndTimer());
	if (entries != COUNT)
	{
		printf("Entries: %llu; count: %llu\n", entries, COUNT);
		assert(entries == COUNT);
	}
	
	//TODO fix the output of PDBs
	FILE *f = fopen(pdb_filename, "w");
	int num = distinct.size();
	assert(fwrite(&num, sizeof(num), 1, f) == 1);
	assert(fwrite(&distinct[0], sizeof(distinct[0]), distinct.size(), f) == distinct.size());
	assert(fwrite(&DB[0], sizeof(uint8_t), COUNT, f) == COUNT);
	//fprintf(f, "%ld\n", distinct.size()); // first element is the number of distinct pieces
//	for (unsigned i = 0; i < distinct.size(); i++)
//	{
//		fprintf(f, "%d\n", distinct[i]); //first few lines are these distinct elements
//	}
//	for (unsigned i = 0; i < COUNT; i++)
//	{
//		fprintf(f, "%d\n", DB[i]);
//	}
	
	fclose(f);
	
	printf("Wrote %lld entries to '%s'\n", entries, pdb_filename);
	for (int x = 0; x < depths.size(); x++)
		printf("%d %lld\n", x, depths[x]);
	
	PDB.push_back(DB); // increase the number of regular PDBs being stored
	PDB_distincts.push_back(distinct); // stores distinct
}

// The distinct states should not include blanks
template <class state, class action>
void PermutationPuzzleEnvironment<state, action>::Build_Additive_PDB(state &start, const std::vector<int> &distinct, const char *pdb_filename, bool blank)
{
	maxItem = max(maxItem,start.puzzle.size());
	maxPattern = max(maxPattern, distinct.size());
	buildCaches();

	uint64_t COUNT = nUpperk(start.puzzle.size(), start.puzzle.size() - distinct.size());
	uint64_t closedSize = nUpperk(start.puzzle.size(), start.puzzle.size() - distinct.size() - 1);
	std::vector<int> closedPattern(distinct);
	std::vector<uint8_t> DB(COUNT);
	std::vector<bool> closed(closedSize);
	
	if (blank)
		closedPattern.push_back(0);
	
	for (unsigned int x = 0; x < DB.size(); x++)
		DB[x] = 255;
	
	uint64_t entries = 0;
	std::cout << "Num Entries: " << COUNT << std::endl;
	std::cout << "Closed list entries: " << closedSize << std::endl;
	std::cout << "Goal State: " << start << std::endl;
	std::cout << "State Hash of Goal: " << GetStateHash(start) << std::endl;
	std::cout << "PDB Hash of Goal: " << GetPDBHash(start, distinct) << std::endl;
	
	std::deque<state> q_curr, q_next;
	std::vector<state> children;
	
	for (unsigned i = 0; i < start.puzzle.size(); i++)
	{
		bool is_distinct = false;
		for (unsigned j = 0; j < distinct.size(); j++)
		{
			if (start.puzzle[i] == distinct[j])
			{
				is_distinct = true;
				break;
			}
		}
		
		if (!is_distinct)
		{
			if (start.puzzle[i] != 0 || !blank)
				start.puzzle[i] = -1;
		}
	}
	
	std::cout << "Abstract Goal State: " << start << std::endl;
	std::cout << "Abstract PDB Hash of Goal: " << GetPDBHash(start, distinct) << std::endl;
	
 	q_curr.push_back(start);
	DB[GetPDBHash(start, distinct)] = 0;
	entries++;
	int depth = 1;
	std::vector<uint64_t> depths(2);
	depths[0]++;
	do {
		while (!q_curr.empty())
		{
			state next = q_curr.front();
			q_curr.pop_front();
			
			//printf("%llu\n", GetPDBHash(next, distinct));
			assert(DB[GetPDBHash(next, distinct)] != 255);

			uint64_t closedHash = GetPDBHash(next, closedPattern);
			if (closed[closedHash]) // closed
			{
				continue;
			}
			else {
				closed[closedHash] = true;
			}
			
			children.resize(0);
			this->GetSuccessors(next, children);
			
			for (unsigned int x = 0; x < children.size(); x++)
			{
				if (children[x] == next)
				{
					if (closed[GetPDBHash(children[x], closedPattern)] == false)
					{
						q_curr.push_back(children[x]);
					}
				}
				else {
					uint64_t pdbHash = GetPDBHash(children[x], distinct);
					if (DB[pdbHash] == 255)
					{
						int hval = 0;//this->HCost(children[x]);//*2-40;
//						if (0 == hval%2) // even
//						{
//							hval=hval*2-40;
//						}
//						else {
//							hval=hval*2-41;
//						}
						//if (hval > 12) hval = 12;
						hval = depth-hval;
						assert(hval >= 0);
						DB[pdbHash] = hval;
						if (hval >= depths.size())
							depths.resize(hval+1);
						depths[hval]++;
						entries++;

						// updates on how much built
						if ((entries % 100000) == 0)
						{
							std::cout << entries << std::endl;
						}
					}
					if (closed[GetPDBHash(children[x], closedPattern)] == false)
					{
						q_next.push_back(children[x]);
					}
				}
			}
		}
		depth++;
		//depths.resize(depth+1);
		q_curr.swap(q_next);
	} while (!q_curr.empty());
	
	assert(entries == COUNT);
	
	//TODO fix the output of PDBs
	FILE *f = fopen(pdb_filename, "w");
	fprintf(f, "%ld\n", distinct.size()); // first element is the number of distinct pieces
	for (unsigned i = 0; i < distinct.size(); i++)
	{
		fprintf(f, "%d\n", distinct[i]); //first few lines are these distinct elements
	}
	assert(fwrite(&DB[0], sizeof(uint8_t), COUNT, f) == COUNT);
//	for (unsigned i = 0; i < COUNT; i++)
//	{
//		fprintf(f, "%d\n", DB[i]);
//	}
	
	fclose(f);
	printf("%lld entries\n", entries);
	for (int x = 0; x < depths.size(); x++)
		printf("%d %lld\n", x, depths[x]);
}

template <class state, class action>
void PermutationPuzzleEnvironment<state, action>::Load_Additive_PDB(const state &goal, const char *pdb_filename)
{
	additive = true;
	
	PDB.resize(PDB.size()+1); // increase the number of regular PDBs being stored
	
	std::vector<int> distinct;
	
	FILE *f;
	f = fopen(pdb_filename, "r");
	if (f == 0)
	{
		printf("Failed to open pdb '%s'\n", pdb_filename);
		exit(0);
	}
	
	unsigned num_distinct;
	if (fscanf(f, "%d\n", &num_distinct) != 1)
	{
		printf("Failure reading tile count from PDB\n");
		exit(0);
	}
	
	for (unsigned i = 0; i < num_distinct; i++)
	{
		int tmp;
		if (fscanf(f, "%d\n", &tmp) != 1)
		{
			printf("Failure reading tile from PDB\n");
			exit(0);
		}

		distinct.push_back(tmp);
	}
	
	maxItem = max(maxItem,goal.puzzle.size());
	maxPattern = max(maxPattern, distinct.size());
	buildCaches();

	uint64_t COUNT = nUpperk(goal.puzzle.size(), goal.puzzle.size() - distinct.size());
	PDB.back().resize(COUNT);

	size_t index;
	if ((index = fread(&PDB.back()[0], sizeof(uint8_t), COUNT, f)) != COUNT)
	{
		printf("Error; did not correctly read %lu entries from PDB (%lu instead)\n", COUNT, index);
		exit(0);
	}
	fclose(f);

	PDB_distincts.push_back(distinct); // stores distinct
	
	if (1)//print_histogram)
	{
		uint8_t maxval = 0;
		std::vector<int> values(256);
		// performs histogram count
		for (unsigned int x = 0; x < COUNT; x++)
		{
			values[(PDB.back()[x])]++;
			maxval = max(maxval, PDB.back()[x]);
		}
		// outputs histogram of heuristic value counts
		histogram.resize(max(histogram.size(),maxval+1));
		for (int x = 0; x <= maxval; x++)
		{
			printf("%d:\t%d\n", x, values[x]);
		}
	}

}

template <class state, class action>
double PermutationPuzzleEnvironment<state, action>::HCost(const state &s)
{
	if (lookups.size() == 0)
		return 0;
	return HCost(s, 0);
}

template <class state, class action>
double PermutationPuzzleEnvironment<state, action>::HCost(const state &s, int treeNode)
{
	double hval = 0;
	switch (lookups[treeNode].t)
	{
		case kMaxNode:
		{
			for (int x = 0; x < lookups[treeNode].numChildren; x++)
			{
				hval = max(hval, HCost(s, lookups[treeNode].firstChildID+x));
			}
		} break;
		case kAddNode:
		{
			for (int x = 0; x < lookups[treeNode].numChildren; x++)
			{
				hval += HCost(s, lookups[treeNode].firstChildID+x);
			}
		} break;
		default:
		{
			uint64_t index = GetPDBHash(s, PDB_distincts[lookups[treeNode].PDBID]);
			hval = PDB[lookups[treeNode].PDBID][index];
		} break;
			
	}
	return hval;
}

template <class state, class action>
bool PermutationPuzzleEnvironment<state, action>::Check_Permutation(const std::vector<int> &to_check)
{
	unsigned size = to_check.size();
	
	bool in_puzzle[size];
	
	for (unsigned i = 0; i < size; i++)
	{
		in_puzzle[i] = false;
	}
	
	for (unsigned i = 0; i < size; i++)
	{
		in_puzzle[to_check[i]] = true;
	}
	
	for (unsigned i = 0; i < size; i++)
	{
		if (!in_puzzle[i])
			return false;
	}
	
	return true;
}

template <class state, class action>
bool PermutationPuzzleEnvironment<state, action>::Output_Puzzles(std::vector<state> &puzzle_vector, bool write_puzz_num)
{
	// check validity of puzzles
	for (unsigned i = 0; i < puzzle_vector.size(); i++)
	{
		/**
		 Make sure all puzzles are for this environment
		 **/
		if (!Check_Permutation(puzzle_vector[i].puzzle))
		{
			std::cerr << "Invalid Puzzle: " << puzzle_vector[i] << '\n';
			return false;
		}
	}
	
	for (unsigned i = 0; i < puzzle_vector.size(); i++)
	{
		if (write_puzz_num)
		{
			printf("%u ", i + 1);
		}
		printf("%d", puzzle_vector[i].puzzle[0]);
		
		for (unsigned j = 1; j < puzzle_vector[i].puzzle.size(); j++)
		{
			printf(" %d", puzzle_vector[i].puzzle[j]);
		}
		printf("\n");
	}
	return true;
}

// TODO clean up using split
template <class state, class action>
bool PermutationPuzzleEnvironment<state, action>::Read_In_Permutations(const char *filename, unsigned size, unsigned max_puzzles, std::vector<std::vector<int> > &permutations, bool puzz_num_start)
{
	std::ifstream ifs(filename, std::ios::in);
	
	if (ifs.fail())
	{
		fprintf(stderr, "File Reading Failed\n");
		return false;
	}
	
	std::string s, temp;
	
	std::vector<int> puzz_ints;
	
	bool first = true;
	unsigned puzz_count = 0;
	
	while(!ifs.eof() && (puzz_count < max_puzzles || max_puzzles==0) )
	{
		puzz_ints.clear();
		
		getline(ifs, s);
		// indicates are starting new permutation
		first = true;
		for (unsigned int i = 0; i < s.length(); i++)
		{
			if (s.at(i) == ' ' || s.at(i) == '\t')
			{
				if (temp.length() > 0)
				{
					if (puzz_num_start && first)
					{
						temp.clear();
						first = false;
					}
					else
					{
						puzz_ints.push_back(atoi(temp.c_str()));
						temp.clear();
					}
				}
			}
			else
			{
				temp.push_back(s.at(i));
			}
		}
		
		
		if (temp.length() > 0)
		{
			puzz_ints.push_back(atoi(temp.c_str()));
			temp = "";
		}
		
		// ensure is proper permutation, otherwise don't add it to the list
		if (puzz_ints.size() == size && Check_Permutation(puzz_ints))
		{
			puzz_count++;
			permutations.push_back(puzz_ints);
		}
		
	}
	
	ifs.close();
	
	return true;
}

#endif // PERMPUZZ_H
