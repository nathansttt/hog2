//
//  MR1PermutationPDB.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/14/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#ifndef MR1PermutationPDB_h
#define MR1PermutationPDB_h

#include "PDBHeuristic.h"
const int maxThreads = 32;

//Later we will add this paper:
//http://aaaipress.org/Papers/Workshops/2008/WS-08-10/WS08-10-004.pdf

/**
 * This class uses the first of two Myrvold-Russkey ranking functions
 * which run in linear time, but does not produce a lexicographical ranking.
 * Note that it produces the dual of their result, as this is necessary for
 * the partial ranking.
 *
 * For more details see:
 * http://webhome.cs.uvic.ca/~ruskey/Publications/RankPerm/MyrvoldRuskey.pdf
 */
template <class state, class action, class environment>
class MR1PermutationPDB : public PDBHeuristic<state, action, environment> {
public:
	MR1PermutationPDB(environment *e, const state &s, std::vector<int> distincts);
	//	virtual uint64_t GetStateHash(const state &s) const;
	//	virtual void GetStateFromHash(state &s, uint64_t hash) const;
	
	virtual uint64_t GetPDBSize() const;
	
	virtual uint64_t GetPDBHash(const state &s, int threadID = 0) const;
	virtual void GetStateFromPDBHash(uint64_t hash, state &s, int threadID = 0) const;
	
	bool Load(FILE *f);
	void Save(FILE *f);
	bool Load(const char *prefix);
	void Save(const char *prefix);
	std::string GetFileName(const char *prefix);
private:
	std::vector<int> distinct;
	size_t puzzleSize;
	uint64_t pdbSize;
	
	// cache for computing ranking/unranking
	mutable std::vector<std::vector<int> > dualCache;
	mutable std::vector<std::vector<int> > locsCache;
	mutable std::vector<std::vector<int> > valueStack;
};

template <class state, class action, class environment>
MR1PermutationPDB<state, action, environment>::MR1PermutationPDB(environment *e, const state &s, std::vector<int> distincts)
:PDBHeuristic<state, action, environment>(e), distinct(distincts), puzzleSize(s.puzzle.size()), dualCache(maxThreads), locsCache(maxThreads), valueStack(maxThreads)
{
	pdbSize = 1;
	for (int x = (int)s.puzzle.size(); x > s.puzzle.size()-distincts.size(); x--)
	{
		pdbSize *= x;
	}
}

template <class state, class action, class environment>
uint64_t MR1PermutationPDB<state, action, environment>::GetPDBSize() const
{
	return pdbSize;
}

inline void swap(int &a, int &b)
{
	int tmp = a;
	a = b;
	b = tmp;
}

template <class state, class action, class environment>
uint64_t MR1PermutationPDB<state, action, environment>::GetPDBHash(const state &s, int threadID) const
{
	std::vector<int> &locs = locsCache[threadID];
	std::vector<int> &dual = dualCache[threadID];
	std::vector<int> &values = valueStack[threadID];
	locs.resize(s.puzzle.size()); // vector for distinct item locations
	dual.resize(s.puzzle.size()); // vector for distinct item locations
	values.resize(0);
	memset(&locs[0], 0xFF, locs.size()*sizeof(locs[0]));
	memset(&dual[0], 0xFF, dual.size()*sizeof(dual[0]));
	// find current duals
	for (unsigned int x = 0; x < s.puzzle.size(); x++)
	{
		if (s.puzzle[x] != -1)
			dual[s.puzzle[x]] = x;
	}
	// get locs by converting from the distinct array
	for (int x = 0; x < distinct.size(); x++)
	{
		locs[s.puzzle.size()-x-1] = dual[distinct[distinct.size()-x-1]];
		dual[distinct[distinct.size()-x-1]] = -1;
	}
	// get new duals for the actual locs (after conversion)
	for (int x = s.puzzle.size()-distinct.size(); x < s.puzzle.size(); x++)
	{
		dual[locs[x]] = x;
	}
	
	//int ss[12];
	//int ssLoc = 0;
	
	size_t last = s.puzzle.size()-distinct.size();
	for (size_t i = s.puzzle.size(); i > last; i--)
	{
		values.push_back(locs[i-1]); //val = locs[i-1];//get(perm, i-1);
		
		if (dual[i-1] != -1)
		{
			swap(locs[i-1], locs[dual[i-1]]);
			swap(dual[values.back()], dual[i-1]);
		}
		
		//		swap(perm, i-1, get(dual, i-1));
		//		swap(dual, s, i-1);
	}
	uint64_t result = 0;
	int cnt = last+1;
	while (values.size() > 0)
	{
		result *= cnt;
		result += values.back();
		values.pop_back();
		cnt++;
	}
	return result;
}

template <class state, class action, class environment>
void MR1PermutationPDB<state, action, environment>::GetStateFromPDBHash(uint64_t hash, state &s, int threadID) const
{
	std::vector<int> &dual = dualCache[threadID];
	dual.resize(s.puzzle.size()); // vector for distinct item locations
	for (int x = 0; x < dual.size(); x++)
		dual[x] = x;
	
	size_t last = (s.puzzle.size()-distinct.size());
//	for (size_t x = 0; x < s.puzzle.size(); x++)
//		s.puzzle[x] = -1;
	memset(&s.puzzle[0], 0xFF, s.puzzle.size()*sizeof(s.puzzle[0]));
	
	for (size_t i = s.puzzle.size(); i > last; i--)
	{
		swap(dual[hash%i], dual[i-1]);
		hash = hash/i;
		s.puzzle[dual[i-1]] = distinct[i-last-1];
	}
}

template <class state, class action, class environment>
std::string MR1PermutationPDB<state, action, environment>::GetFileName(const char *prefix)
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
		fileName += ";";
	}
	fileName.pop_back(); // remove colon
	fileName += "-";
	for (auto x : distinct)
	{
		fileName += std::to_string(x);
		fileName += ";";
	}
	fileName.pop_back(); // remove colon
	fileName += "-MR.pdb";
	
	return fileName;
}

template <class state, class action, class environment>
bool MR1PermutationPDB<state, action, environment>::Load(const char *prefix)
{
	assert(false);
	return false;
}

template <class state, class action, class environment>
void MR1PermutationPDB<state, action, environment>::Save(const char *prefix)
{
	assert(false);
	FILE *f = fopen(GetFileName(prefix).c_str(), "w+");
	PDBHeuristic<state, action, environment>::PDB.Write(f);
	fclose(f);
}

template <class state, class action, class environment>
bool MR1PermutationPDB<state, action, environment>::Load(FILE *f)
{
	assert(false);
	return false;
}

template <class state, class action, class environment>
void MR1PermutationPDB<state, action, environment>::Save(FILE *f)
{
	assert(false);
}

#endif /* MR1PermutationPDB_h */
