#include "SearchEnvironment.h"
#include <assert.h>
#include <deque>

#ifndef PERMPUZZ_H
#define PERMPUZZ_H

template <class state, class action>
class PermutationPuzzleEnvironment : public SearchEnvironment<state, action>
{
public:
	/**
	Returns the value of n! / k!
	**/
	uint64_t nUpperk(int n, int k);

	/**
	Returns the Hash Value of the given state using the given set of distinct items
	**/
	virtual uint64_t GetPDBHash(state &s, const std::vector<int> &distinct);

	/**
	Loads the Regular PDB into memory
	**/
	void Load_Regular_PDB(char *fname, state &goal, const std::vector<int> &distinct);

	/**
	Performs a regular PDB lookup for the given state
	**/
	double Regular_PDB_Lookup(state &state);

	/**
	Builds a regular PDB given the file name of the file to write the PDB to, and a list of distinct
	tiles
	**/
	void Build_Regular_PDB(state &start, const std::vector<int> &distinct, const char *pdb_filename);

	// holds a set of Pattern Databases which can be maxed over later
	std::vector<std::vector<uint8_t> > PDB;
	// holds the set of distinct items used to build the associated PDB (and therefore needed for hashing)
	std::vector<std::vector<int> > PDB_distincts;
};

template <class state, class action>
uint64_t PermutationPuzzleEnvironment<state, action>::nUpperk(int n, int k) {
	uint64_t value = 1;
	assert(n >= 0 && k >= 0);

	for(int i = n; i > k; i--) {
		value *= i;
	}

	return value;
}

// TODO Change to Myrvold and Ruskey ranking function
template <class state, class action>
uint64_t PermutationPuzzleEnvironment<state, action>::GetPDBHash(state &s, const std::vector<int> &distinct) {
	std::vector<int> locs;
	locs.resize(distinct.size()); // vector for distinct item locations

	// find item locations
	for (unsigned int x = 0; x < s.puzzle.size(); x++)
	{
		for (unsigned int y = 0; y < distinct.size(); y++)
		{
			if (s.puzzle[x] == distinct[y])
				locs[y] = x;
		}
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
void PermutationPuzzleEnvironment<state, action>::Load_Regular_PDB(char *fname, state &goal, const std::vector<int> &distinct)
{
	std::vector<int> values(256); // number of abstract states with that heuristic value

	// number of abstract states
	uint64_t COUNT = nUpperk(goal.puzzle.size(), goal.puzzle.size() - distinct.size());

	PDB.resize(PDB.size()+1); // increase the number of regular PDBs being stored
	PDB.back().resize(COUNT);
	FILE *f = fopen(fname, "r");

	// reads in PDB
	if (f)
	{
		fread(&(PDB.back()[0]), sizeof(uint8_t), COUNT, f);
		fclose(f);
	}
	PDB_distincts.push_back(distinct); // stores distinct

	// performs histogram count
	for (unsigned int x = 0; x < COUNT; x++)
	{
		values[(PDB.back()[x])]++;
	}

	// outputs histogram of heuristic value counts
	for (int x = 0; x < 256; x++)
		printf("%d:\t%d\n", x, values[x]);
}

template <class state, class action>
double PermutationPuzzleEnvironment<state, action>::Regular_PDB_Lookup(state &s)
{
	double val = 0;
	for (unsigned int x = 0; x < PDB.size(); x++)
	{
		uint64_t index = GetPDBHash(s, PDB_distincts[x]);
		val = std::max(val, (double)PDB[x][index]);
	}
	return val;
}

template <class state, class action>
void PermutationPuzzleEnvironment<state, action>::Build_Regular_PDB(state &start, const std::vector<int> &distinct, const char *pdb_filename) {

	uint64_t COUNT = nUpperk(start.puzzle.size(), start.puzzle.size() - distinct.size());
	std::vector<uint8_t> DB(COUNT);

	for (unsigned int x = 0; x < DB.size(); x++)
		DB[x] = 255;

	uint64_t entries = 0;
	std::cout << COUNT << std::endl;
	std::cout << start << std::endl;
	std::cout << GetStateHash(start) << std::endl;
	std::cout << GetPDBHash(start, distinct) << std::endl;

	std::deque<state> q;
	std::vector<state> children;

	q.push_back(start);
	DB[GetPDBHash(start, distinct)] = 0;
	entries++;

	while (!q.empty()) {
		// updates on how much built
		if ((entries % 10000) == 0)
			std::cout << entries << std::endl;
		state next = q.front();
		q.pop_front();
		children.clear();
		GetSuccessors(next, children);
		for (unsigned int x = 0; x < children.size(); x++)
		{
			if (DB[GetPDBHash(children[x], distinct)] == 255)
			{

				DB[GetPDBHash(children[x], distinct)] = DB[GetPDBHash(next, distinct)]+1;
				q.push_back(children[x]);
				entries++;
			}
		}
	}

	assert(entries == COUNT);

	FILE *f = fopen(pdb_filename, "w");
	for(unsigned i = 0; i < COUNT; i++) {
		fprintf(f, "%d\n", DB[i]);
	}

	fclose(f);
	/*
	if (f)
	{
		fwrite(&(DB[0]), sizeof(uint8_t), COUNT, f);
		fclose(f);
	}*/
}

#endif
