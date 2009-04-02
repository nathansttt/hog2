/*
 *  TopSpin.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/1/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

//#include <ext/hash_map>
#include "GraphEnvironment.h"

//#include "ts.h"
#ifndef TOPSPIN_H
#define TOPSPIN_H

class TopSpin;

class TopSpinGraphHeuristic : public GraphHeuristic {
public:
	TopSpinGraphHeuristic(int psize, int spin, int pdb);
	TopSpinGraphHeuristic();
	Graph *GetGraph() { return 0; }
	void SetState(TopSpin *tss) { ts = tss; }
	virtual ~TopSpinGraphHeuristic() { }
	virtual double HCost(graphState &state1, graphState &state2);

	static int THmode;
private:
	TopSpin *ts;
	int pdb;
	std::vector<uint8_t> DB;
};

class TopSpinGraphData {
public:
	bool expanded;
	unsigned long hashKey;
	std::vector<int> config;
	//int zeroLoc; // get rid of this & just normalize
};

class TopSpin : public GraphEnvironment {
public:
	TopSpin(int m, int k, TopSpinGraphHeuristic *tsh);
	TopSpin(int m, int k);
	~TopSpin();
	void GetSuccessors(graphState &stateID, std::vector<graphState> &neighbors) const;
	void GetActions(graphState &stateID, std::vector<graphMove> &actions) const;
	bool GoalTest(graphState &state, graphState &goal);
	graphState Dual(graphState s);
	graphState GetState(const std::vector<int> &configuration) const;
	graphState GetState(const std::vector<int> &configuration, int zeroLoc) const;
	std::vector<int> &GetState(graphState g) const;
	
	uint64_t GetStateHash(graphState &state) const;
	uint64_t GetStateHash(std::vector<int> &config) const;
	uint64_t GetStateHash(int *config, int config_size) const;
	uint64_t GetPDBHash(const std::vector<int> &config, int pdb_size) const;
	uint64_t GetPDBHash(const graphState &state, int pdb_size) const;
	uint64_t GetPDBSize(int puzzle_size, int pdb_size) const;
private:
	typedef __gnu_cxx::hash_map<uint64_t, unsigned long, Hash64> TopSpinHashTable;

	void ExpandNode(graphState &stateID) const;
	void Flip(std::vector<int> &arrangement, int index, int radius) const;
	// these store the actual graph of the state space
	mutable Graph *g2;
	mutable TopSpinHashTable hashTable;
	mutable std::vector<TopSpinGraphData> data;
	int length, flipSize;
};

#endif
