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

/**
 * This class does the basic permutation calculation with a regular N^2 permutation
 * computation.
 */
template <class state, class action, class environment>
class PermutationPDB : public PDBHeuristic<state, action, environment, state> {
public:
	PermutationPDB(environment *e, const state &s, std::vector<int> distincts);

	virtual uint64_t GetPDBSize() const;

	virtual uint64_t GetPDBHash(const state &s, int threadID = 0) const;
	virtual void GetStateFromPDBHash(uint64_t hash, state &s, int threadID = 0) const;
	virtual uint64_t GetAbstractHash(const state &s, int threadID = 0) const { return GetPDBHash(s); }
	virtual state GetStateFromAbstractState(state &s) const { return s; }

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
	state example;
	// cache for computing ranking/unranking
	mutable std::vector<std::vector<int> > dualCache;
	mutable std::vector<std::vector<int> > locsCache;
};

template <class state, class action, class environment>
PermutationPDB<state, action, environment>::PermutationPDB(environment *e, const state &s, std::vector<int> distincts)
:PDBHeuristic<state, action, environment, state>(e), distinct(distincts), puzzleSize(s.puzzle.size()),
dualCache(maxThreads), locsCache(maxThreads), example(s)
{
	pdbSize = 1;
	for (int x = (int)s.puzzle.size(); x > s.puzzle.size()-distincts.size(); x--)
	{
		pdbSize *= x;
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
	std::vector<int> &locs = locsCache[threadID];
	std::vector<int> &dual = dualCache[threadID];
	// TODO: test definition
	locs.resize(distinct.size()); // vector for distinct item locations
	dual.resize(s.puzzle.size()); // vector for distinct item locations
	
	// find item locations
	for (unsigned int x = 0; x < s.puzzle.size(); x++)
	{
		if (s.puzzle[x] != -1)
			dual[s.puzzle[x]] = x;
	}
	for (int x = 0; x < distinct.size(); x++)
	{
		locs[x] = dual[distinct[x]];
	}
	
	uint64_t hashVal = 0;
	int numEntriesLeft = (int)s.puzzle.size();
	
	for (unsigned int x = 0; x < locs.size(); x++)
	{
		hashVal += locs[x]*FactorialUpperK(numEntriesLeft-1, s.puzzle.size()-distinct.size());
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

template <class state, class action, class environment>
void PermutationPDB<state, action, environment>::GetStateFromPDBHash(uint64_t hash, state &s, int threadID) const
{
	uint64_t hashVal = hash;
	std::vector<int> &dual = dualCache[threadID];

	dual.resize(distinct.size());
	
	int numEntriesLeft = puzzleSize-distinct.size()+1;
	for (int x = distinct.size()-1; x >= 0; x--)
	{
		dual[x] = hashVal%numEntriesLeft;
		hashVal /= numEntriesLeft;
		numEntriesLeft++;
		for (int y = x+1; y < distinct.size(); y++)
		{
			if (dual[y] >= dual[x])
				dual[y]++;
		}
	}
	s.puzzle.resize(puzzleSize);
	std::fill(s.puzzle.begin(), s.puzzle.end(), -1);
	for (int x = 0; x < dual.size(); x++)
		s.puzzle[dual[x]] = distinct[x];
	s.FinishUnranking(example);
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
	for (int x = 0; x < PDBHeuristic<state, action, environment>::goalState.puzzle.size(); x++)
	{
		fileName += std::to_string(PDBHeuristic<state, action, environment>::goalState.puzzle[x]);
		fileName += ";";
	}
	fileName.pop_back(); // remove colon
	fileName += "-";
	for (int x = 0; x < distinct.size(); x++)
	{
		fileName += std::to_string(distinct[x]);
		fileName += ";";
	}
	fileName.pop_back(); // remove colon
	fileName += "-lex.pdb";
	
	return fileName;
}

template <class state, class action, class environment>
bool PermutationPDB<state, action, environment>::Load(const char *prefix)
{
	assert(false);
	return false;
}

template <class state, class action, class environment>
void PermutationPDB<state, action, environment>::Save(const char *prefix)
{
	assert(false);
	FILE *f = fopen(GetFileName(prefix).c_str(), "w+");
	PDBHeuristic<state, action, environment>::PDB.Write(f);
	fclose(f);
}

template <class state, class action, class environment>
bool PermutationPDB<state, action, environment>::Load(FILE *f)
{
	assert(false);
	return false;
}

template <class state, class action, class environment>
void PermutationPDB<state, action, environment>::Save(FILE *f)
{
	assert(false);
}

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
