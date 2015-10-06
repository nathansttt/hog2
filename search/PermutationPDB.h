//
//  PermutationPDB.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/27/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//

#ifndef hog2_glut_PermutationPDB_h
#define hog2_glut_PermutationPDB_h

#include "PDBHeuristic.h"
const int maxThreads = 32;

template <class state, class action, class environment>
class PermutationPDB : public PDBHeuristic<state, action, environment> {
public:
	PermutationPDB(environment *e, const state &s, std::vector<int> distincts);
	virtual uint64_t GetStateHash(const state &s) const;
	virtual void GetStateFromHash(state &s, uint64_t hash) const;
	virtual uint64_t GetPDBSize() const;
	virtual uint64_t GetPDBHash(const state &s, int threadID = 0) const;
	virtual void GetStateFromPDBHash(uint64_t hash, state &s, int threadID = 0) const;

	bool Load(FILE *f);
	void Save(FILE *f);
	bool Load(const char *prefix);
	void Save(const char *prefix);
	std::string GetFileName(const char *prefix);
private:
	uint64_t Factorial(int val) const;
	uint64_t FactorialUpperK(int n, int k) const;
	std::vector<int> distinct;
	size_t puzzleSize;
	uint64_t pdbSize;
	
	// cache for computing ranking/unranking
	mutable std::vector<std::vector<int> > dual;
	mutable std::vector<std::vector<int> > locs;
};

template <class state, class action, class environment>
PermutationPDB<state, action, environment>::PermutationPDB(environment *e, const state &s, std::vector<int> distincts)
:PDBHeuristic<state, action, environment>(e), distinct(distincts), puzzleSize(s.puzzle.size()), dual(maxThreads), locs(maxThreads)
{
	pdbSize = 1;
	for (int x = (int)s.puzzle.size(); x > s.puzzle.size()-distincts.size(); x--)
	{
		pdbSize *= x;
	}
}

template <class state, class action, class environment>
uint64_t PermutationPDB<state, action, environment>::GetStateHash(const state &s) const
{
	std::vector<int> puzzle = s.puzzle;
	uint64_t hashVal = 0;
	int numEntriesLeft = (int)s.puzzle.size();
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

template <class state, class action, class environment>
void PermutationPDB<state, action, environment>::GetStateFromHash(state &s, uint64_t hash) const
{
	// WARNING: I removed a temporary cache here that didn't seem necessary
	uint64_t hashVal = hash;
	s.puzzle.resize(puzzleSize);
	
	int numEntriesLeft = 1;
	for (int x = (int)s.puzzle.size()-1; x >= 0; x--)
	{
		s.puzzle[x] = hashVal%numEntriesLeft;
		hashVal /= numEntriesLeft;
		numEntriesLeft++;
		for (int y = x+1; y < (int) s.puzzle.size(); y++)
		{
			if (s.puzzle[y] >= s.puzzle[x])
				s.puzzle[y]++;
		}
	}
}

template <class state, class action, class environment>
uint64_t PermutationPDB<state, action, environment>::GetPDBSize() const
{
	return pdbSize;
}

template <class state, class action, class environment>
uint64_t PermutationPDB<state, action, environment>::GetPDBHash(const state &s, int threadID) const
{
	// TODO: test definition
	locs[threadID].resize(distinct.size()); // vector for distinct item locations
	dual[threadID].resize(s.puzzle.size()); // vector for distinct item locations
	
	// find item locations
	for (unsigned int x = 0; x < s.puzzle.size(); x++)
	{
		if (s.puzzle[x] != -1)
			dual[threadID][s.puzzle[x]] = x;
	}
	for (int x = 0; x < distinct.size(); x++)
	{
		locs[threadID][x] = dual[threadID][distinct[x]];
	}
	
	uint64_t hashVal = 0;
	int numEntriesLeft = (int)s.puzzle.size();
	
	for (unsigned int x = 0; x < locs[threadID].size(); x++)
	{
		hashVal += locs[threadID][x]*FactorialUpperK(numEntriesLeft-1, s.puzzle.size()-distinct.size());
		numEntriesLeft--;
		
		// decrement locations of remaining items
		for (unsigned y = x; y < locs[threadID].size(); y++)
		{
			if (locs[threadID][y] > locs[threadID][x])
				locs[threadID][y]--;
		}
	}
	return hashVal;
}

template <class state, class action, class environment>
void PermutationPDB<state, action, environment>::GetStateFromPDBHash(uint64_t hash, state &s, int threadID) const
{
	uint64_t hashVal = hash;
	dual[threadID].resize(distinct.size());
	
	int numEntriesLeft = puzzleSize-distinct.size()+1;
	for (int x = distinct.size()-1; x >= 0; x--)
	{
		dual[threadID][x] = hashVal%numEntriesLeft;
		hashVal /= numEntriesLeft;
		numEntriesLeft++;
		for (int y = x+1; y < distinct.size(); y++)
		{
			if (dual[threadID][y] >= dual[threadID][x])
				dual[threadID][y]++;
		}
	}
	s.puzzle.resize(puzzleSize);
	std::fill(s.puzzle.begin(), s.puzzle.end(), -1);
	for (int x = 0; x < dual[threadID].size(); x++)
		s.puzzle[dual[threadID][x]] = distinct[x];
}

template <class state, class action, class environment>
uint64_t PermutationPDB<state, action, environment>::Factorial(int val) const
{
	static uint64_t table[21] =
	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
	if (val > 20)
		return (uint64_t)-1;
	return table[val];
}

template <class state, class action, class environment>
std::string PermutationPDB<state, action, environment>::GetFileName(const char *prefix)
{
	std::string fileName;
	fileName += prefix;
	// For unix systems, the prefix should always end in a trailing slash
	if (fileName.back() != '/')
		fileName+='/';
	fileName += PDBHeuristic<state, action, environment>::env->GetName();
	fileName += "-";
	for (auto x : PDBHeuristic<state, action, environment>::goalState.puzzle)
	{
		fileName += std::to_string(x);
		fileName += ":";
	}
	fileName.pop_back(); // remove colon
	fileName += "-";
	for (auto x : distinct)
	{
		fileName += std::to_string(x);
		fileName += ":";
	}
	fileName.pop_back(); // remove colon
	fileName += ".pdb";
	
	return fileName;
}

template <class state, class action, class environment>
bool PermutationPDB<state, action, environment>::Load(const char *prefix)
{
	assert(false);
}

template <class state, class action, class environment>
void PermutationPDB<state, action, environment>::Save(const char *prefix)
{
	assert(false);
	FILE *f = fopen(GetFileName().c_str(), "w+");
	PDBHeuristic<state, action, environment>::PDB.Write(f);
	fclose(f);
}

//template <class state, class action, class environment>
//void PermutationPDB<state, action, environment>::WritePDBHeader(FILE *f) const
//{
//	int num = (int)distinct.size();
//	size_t written;
//	written = fwrite(&num, sizeof(num), 1, f);
//	assert(written == 1);
//	written = fwrite(&distinct[0], sizeof(distinct[0]), distinct.size(), f);
//	assert(written == distinct.size());
//}
//
//template <class state, class action, class environment>
//void PermutationPDB<state, action, environment>::ReadPDBHeader(FILE *f) const
//{
//}

template <class state, class action, class environment>
uint64_t PermutationPDB<state, action, environment>::FactorialUpperK(int n, int k) const
{
	uint64_t value = 1;
	assert(n >= 0 && k >= 0);
	
	for (int i = n; i > k; i--)
	{
		value *= i;
	}
	
	return value;

}

#endif
