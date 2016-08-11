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
	PermutationPDB(environment *e, const state &s, const std::vector<int> &distincts);

	virtual uint64_t GetPDBSize() const;

	virtual uint64_t GetPDBHash(const state &s, int threadID = 0) const = 0;
	virtual void GetStateFromPDBHash(uint64_t hash, state &s, int threadID = 0) const = 0;
	virtual uint64_t GetAbstractHash(const state &s, int threadID = 0) const = 0;
	virtual state GetStateFromAbstractState(state &s) const = 0;

	bool Load(FILE *f);
	void Save(FILE *f);
	bool Load(const char *prefix);
	void Save(const char *prefix);
	virtual std::string GetFileName(const char *prefix);
private:
	uint64_t Factorial(int val) const;
	uint64_t FactorialUpperK(int n, int k) const;

protected:
	std::vector<int> distinct;
	size_t puzzleSize;
	uint64_t pdbSize;
	state example;
};

template <class state, class action, class environment>
PermutationPDB<state, action, environment>::PermutationPDB(environment *e, const state &s, const std::vector<int> &distincts)
:PDBHeuristic<state, action, environment, state>(e), distinct(distincts), puzzleSize(s.size()), example(s)
{
	pdbSize = 1;
	for (int x = (int)s.size(); x > s.size()-distincts.size(); x--)
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
std::string PermutationPDB<state, action, environment>::GetFileName(const char *prefix)
{
	std::string fileName;
	fileName += prefix;
	// For unix systems, the prefix should always end in a trailing slash
	if (fileName.back() != '/' && prefix[0] != 0)
		fileName+='/';
	fileName += PDBHeuristic<state, action, environment>::env->GetName();
	fileName += "-";
	for (int x = 0; x < PDBHeuristic<state, action, environment>::goalState.size(); x++)
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
	
	return fileName;
}

template <class state, class action, class environment>
bool PermutationPDB<state, action, environment>::Load(const char *prefix)
{
	FILE *f = fopen(GetFileName(prefix).c_str(), "r+");
	if (f == 0)
		return false;
	bool result = Load(f);
	fclose(f);
	return result;
}

template <class state, class action, class environment>
void PermutationPDB<state, action, environment>::Save(const char *prefix)
{
	FILE *f = fopen(GetFileName(prefix).c_str(), "w+");
	if (f == 0)
	{
		fprintf(stderr, "Error saving");
		return;
	}
	Save(f);
	fclose(f);
}

template <class state, class action, class environment>
bool PermutationPDB<state, action, environment>::Load(FILE *f)
{
	if (PDBHeuristic<state, action, environment>::Load(f) != true)
	{
		return false;
	}
	if (fread(&puzzleSize, sizeof(puzzleSize), 1, f) != 1)
		return false;
	if (fread(&pdbSize, sizeof(pdbSize), 1, f) != 1)
		return false;
	if (fread(&example, sizeof(example), 1, f) != 1)
		return false;
	size_t distinctSize = distinct.size();
	if (fread(&distinctSize, sizeof(distinctSize), 1, f) != 1)
		return false;
	distinct.resize(distinctSize);
	if (fread(&distinct[0], sizeof(distinct[0]), distinct.size(), f) != distinctSize)
		return false;
	return true;
}

template <class state, class action, class environment>
void PermutationPDB<state, action, environment>::Save(FILE *f)
{
	PDBHeuristic<state, action, environment>::Save(f);
	fwrite(&puzzleSize, sizeof(puzzleSize), 1, f);
	fwrite(&pdbSize, sizeof(pdbSize), 1, f);
	fwrite(&example, sizeof(example), 1, f);
	size_t distinctSize = distinct.size();
	fwrite(&distinctSize, sizeof(distinctSize), 1, f);
	fwrite(&distinct[0], sizeof(distinct[0]), distinct.size(), f);
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
