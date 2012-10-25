#include "SearchEnvironment.h"
#include <assert.h>
#include <deque>
#include <iostream>
#include <fstream>
#include <map>
#include <stdint.h>
#include <cstdlib>

#ifndef PERMPUZZ_H
#define PERMPUZZ_H

/**
Note, assumes that state has a public vector<int> called puzzle in which the
permutation is held.
**/
template <class state, class action>
class PermutationPuzzleEnvironment : public SearchEnvironment<state, action>
{
public:
	/**
	Returns the value of n! / k!
	**/
	uint64_t nUpperk(int n, int k) const;

	/**
	Returns the Hash Value of the given state using the given set of distinct items
	**/
	virtual uint64_t GetPDBHash(const state &s, const std::vector<int> &distinct) const;

	/**
	Loads the Regular PDB into memory
	**/
	void Load_Regular_PDB(const char *fname, state &goal, bool print_histogram);

	/**
	Performs a regular PDB lookup for the given state
	**/
	double Regular_PDB_Lookup(const state &s);

	/**
	Builds a regular PDB given the file name of the file to write the PDB to, and a list of distinct
	tiles
	**/
	void Build_Regular_PDB(state &start, const std::vector<int> &distinct, const char *pdb_filename);

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

	bool Validate_Problems(std::vector<state> &puzzles){
		for(unsigned i = 0; i < puzzles.size(); i++) {
			if(!State_Check(puzzles[i]) || !Check_Permutation(puzzles[i].puzzle)) {
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

	// holds a set of Pattern Databases which can be maxed over later
	std::vector<std::vector<uint8_t> > PDB;
	// holds the set of distinct items used to build the associated PDB (and therefore needed for hashing)
	std::vector<std::vector<int> > PDB_distincts;

};

template <class state, class action>
std::vector<int> PermutationPuzzleEnvironment<state, action>::Get_Random_Permutation(unsigned size) {
	std::vector<int> permutation;

	for(unsigned i = 0; i < size; i++)
		permutation.push_back(i);
	int index = 0;
	int temp;

	// randomizes elements in the permutation
	while(size > 1) {
		index = rand() % size;
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
uint64_t PermutationPuzzleEnvironment<state, action>::nUpperk(int n, int k) const {
	uint64_t value = 1;
	assert(n >= 0 && k >= 0);

	for(int i = n; i > k; i--) {
		value *= i;
	}

	return value;
}

// TODO Change to Myrvold and Ruskey ranking function
template <class state, class action>
uint64_t PermutationPuzzleEnvironment<state, action>::GetPDBHash(const state &s, const std::vector<int> &distinct) const {
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
void PermutationPuzzleEnvironment<state, action>::Load_Regular_PDB(const char *fname, state &goal, bool print_histogram)
{
	std::vector<int> values(256); // number of abstract states with that heuristic value

	// number of abstract states
	uint64_t COUNT;

	PDB.resize(PDB.size()+1); // increase the number of regular PDBs being stored

	std::vector<int> distinct;

	std::ifstream ifs(fname, std::ios::in);

	if(ifs.fail()) {
		fprintf(stderr, "File Reading Failed\n");
		exit(1);
	}

	if(ifs.eof()) {
		fprintf(stderr, "File Reading Failed\n");
		exit(1);
	}

	std::string s;
	getline(ifs, s);
	unsigned num_distinct = atoi(s.c_str());

	for(unsigned i = 0; i < num_distinct; i++) {
		if(ifs.eof()) {
			fprintf(stderr, "File Reading Failed\n");
			exit(1);
		}
		getline(ifs, s);
		distinct.push_back(atoi(s.c_str()));
	}

	COUNT = nUpperk(goal.puzzle.size(), goal.puzzle.size() - distinct.size());
	PDB.back().resize(COUNT);
	uint64_t index = 0;
	while(index < COUNT && !ifs.eof()) {
		getline(ifs, s);
		PDB.back()[index] = atoi(s.c_str());
		index++;
	}

	if(index < COUNT) {
		fprintf(stderr, "File Is Not Proper PDB\n");
		exit(1);
	}
	PDB_distincts.push_back(distinct); // stores distinct

	if(print_histogram) {
		// performs histogram count
		for (unsigned int x = 0; x < COUNT; x++) {
			values[(PDB.back()[x])]++;
		}
		// outputs histogram of heuristic value counts
		for (int x = 0; x < 256; x++)
			printf("%d:\t%d\n", x, values[x]);
	}

	this->StoreGoal(goal);
}

template <class state, class action>
double PermutationPuzzleEnvironment<state, action>::Regular_PDB_Lookup(const state &s)
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
	std::cout << "Num Entries: " << COUNT << std::endl;
	std::cout << "Goal State: " << start << std::endl;
	std::cout << "State Hash of Goal: " << GetStateHash(start) << std::endl;
	std::cout << "PDB Hash of Goal: " << GetPDBHash(start, distinct) << std::endl;

	std::deque<state> q;
	std::vector<state> children;

	for(unsigned i = 0; i < start.puzzle.size(); i++) {
		bool is_distinct = false;
		for(unsigned j = 0; j < distinct.size(); j++) {
			if(start.puzzle[i] == distinct[j]) {
				is_distinct = true;
				break;
			}
		}

		if(!is_distinct) {
			start.puzzle[i] = -1;
		}
	}

	std::cout << "Abstract Goal State: " << start << std::endl;
	std::cout << "Abstract PDB Hash of Goal: " << GetPDBHash(start, distinct) << std::endl;

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
		this->GetSuccessors(next, children);
		for (unsigned int x = 0; x < children.size(); x++)
		{
			if (DB[GetPDBHash(children[x], distinct)] == 255)
			{

				DB[GetPDBHash(children[x], distinct)] = DB[GetPDBHash(next, distinct)]+1;
				q.push_back(children[x]);
				entries++;

				/* For Debugging
				if(entries < 20) {
					std::cout << children[x] << ": " << (unsigned) DB[GetPDBHash(children[x], distinct)] << "\n";
				}*/
			}
		}
	}

	assert(entries == COUNT);

	//TODO fix the output of PDBs
	FILE *f = fopen(pdb_filename, "w");
	fprintf(f, "%ld\n", distinct.size()); // first element is the number of distinct pieces
	for(unsigned i = 0; i < distinct.size(); i++) {
		fprintf(f, "%d\n", distinct[i]); //first few lines are these distinct elements
	}
	for(unsigned i = 0; i < COUNT; i++) {
		fprintf(f, "%d\n", DB[i]);
	}

	fclose(f);
}

template <class state, class action>
bool PermutationPuzzleEnvironment<state, action>::Check_Permutation(const std::vector<int> &to_check) {
	unsigned size = to_check.size();

	bool in_puzzle[size];

	for(unsigned i = 0; i < size; i++) {
		in_puzzle[i] = false;
	}

	for(unsigned i = 0; i < size; i++) {
		in_puzzle[to_check[i]] = true;
	}

	for(unsigned i = 0; i < size; i++) {
		if(!in_puzzle[i])
			return false;
	}

	return true;
}

template <class state, class action>
bool PermutationPuzzleEnvironment<state, action>::Output_Puzzles(std::vector<state> &puzzle_vector, bool write_puzz_num) {
	// check validity of puzzles
	for(unsigned i = 0; i < puzzle_vector.size(); i++) {

		/**
		Make sure all puzzles are for this environment
		**/
		if(!Check_Permutation(puzzle_vector[i].puzzle)) {
			std::cerr << "Invalid Puzzle: " << puzzle_vector[i] << '\n';
			return false;
		}
	}

	for(unsigned i = 0; i < puzzle_vector.size(); i++) {
		if(write_puzz_num) {
			printf("%u ", i + 1);
		}
		printf("%d", puzzle_vector[i].puzzle[0]);

		for(unsigned j = 1; j < puzzle_vector[i].puzzle.size(); j++) {
			printf(" %d", puzzle_vector[i].puzzle[j]);
		}
		printf("\n");
	}
	return true;
}

// TODO clean up using split
template <class state, class action>
bool PermutationPuzzleEnvironment<state, action>::Read_In_Permutations(const char *filename, unsigned size, unsigned max_puzzles, std::vector<std::vector<int> > &permutations, bool puzz_num_start) {

	std::ifstream ifs(filename, std::ios::in);

	if(ifs.fail()) {
		fprintf(stderr, "File Reading Failed\n");
		return false;
	}

	std::string s, temp;

	std::vector<int> puzz_ints;

	bool first = true;
	unsigned puzz_count = 0;

	while(!ifs.eof() && (puzz_count < max_puzzles || max_puzzles==0) ) {
		puzz_ints.clear();

		getline(ifs, s);
		// indicates are starting new permutation
		first = true;
		for(unsigned int i = 0; i < s.length(); i++) {
			if(s.at(i) == ' ' || s.at(i) == '\t') {
				if(temp.length() > 0) {
					if(puzz_num_start && first) {
						temp.clear();
						first = false;
					}
					else {
						puzz_ints.push_back(atoi(temp.c_str()));
						temp.clear();
					}
				}
			}
			else {
				temp.push_back(s.at(i));
			}
		}


		if(temp.length() > 0) {

			puzz_ints.push_back(atoi(temp.c_str()));
			temp = "";
		}

		// ensure is proper permutation, otherwise don't add it to the list
		if(puzz_ints.size() == size && Check_Permutation(puzz_ints)) {
			puzz_count++;
			permutations.push_back(puzz_ints);
		}

	}

	ifs.close();

	return true;
}
#endif
