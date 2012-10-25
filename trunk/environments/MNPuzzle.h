/*
 *  MNPuzzle.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/9/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef MNPUZZLE_H
#define MNPUZZLE_H

#include <stdint.h>
#include <iostream>
#include "SearchEnvironment.h"
#include "PermutationPuzzleEnvironment.h"
#include "UnitSimulation.h"
#include "GraphEnvironment.h"
#include "Graph.h"
#include <sstream>

class MNPuzzleState {
public:
	MNPuzzleState() { width = height = -1; }
	MNPuzzleState(unsigned int _width, unsigned int _height)
	:width(_width), height(_height)
  {
		puzzle.resize(width*height);
		for (unsigned int x = 0; x < puzzle.size(); x++)
			puzzle[x] = x;
		blank = 0;
	}
	unsigned int width, height;
	unsigned int blank;
	std::vector<int> puzzle;
};

/**
 * Note, direction "kLeft" indicates that the blank is being moved to the left.
 * That is, a tile is being slid right into the blank position. The other
 * actions are defined similarly.
 */
enum slideDir {
	kLeft, kUp, kDown, kRight
};

static std::ostream& operator <<(std::ostream & out, const MNPuzzleState &loc)
{
	out << "(" << loc.width << "x" << loc.height << ")";
	for (unsigned int x = 0; x < loc.puzzle.size(); x++)
		out << loc.puzzle[x] << " ";
	return out;
}

static std::ostream& operator <<(std::ostream & out, const slideDir &loc)
{
	switch (loc)
	{
		case kLeft: out << "Left"; break;
		case kRight: out << "Right"; break;
		case kUp: out << "Up"; break;
		case kDown: out << "Down"; break;
	}
	return out;
}


static bool operator==(const MNPuzzleState &l1, const MNPuzzleState &l2)
{
	if (l1.width != l2.width)
		return false;
	if (l1.height != l2.height)
		return false;
	for (unsigned int x = 0; x < l1.puzzle.size(); x++)
		if (l1.puzzle[x] != l2.puzzle[x])
			return false;
	return true;
}

class MNPuzzle : public PermutationPuzzleEnvironment<MNPuzzleState, slideDir> {
public:
	MNPuzzle(unsigned int width, unsigned int height);
	MNPuzzle(unsigned int width, unsigned int height, const std::vector<slideDir> op_order); // used to set action order

	~MNPuzzle();
	void GetSuccessors(const MNPuzzleState &stateID, std::vector<MNPuzzleState> &neighbors) const;
	void GetActions(const MNPuzzleState &stateID, std::vector<slideDir> &actions) const;
	slideDir GetAction(const MNPuzzleState &s1, const MNPuzzleState &s2) const;
	void ApplyAction(MNPuzzleState &s, slideDir a) const;
	bool InvertAction(slideDir &a) const;
	static unsigned GetParity(MNPuzzleState &state);

	OccupancyInterface<MNPuzzleState, slideDir> *GetOccupancyInfo() { return 0; }
	double HCost(const MNPuzzleState &state1, const MNPuzzleState &state2);
	double HCost(const MNPuzzleState &state1);

	double GCost(const MNPuzzleState &state1, const MNPuzzleState &state2);
	double GCost(const MNPuzzleState &, const slideDir &) { return 1.0; }
	bool GoalTest(const MNPuzzleState &state, const MNPuzzleState &goal);

	bool GoalTest(const MNPuzzleState &s);

	void LoadPDB(char *fname, const std::vector<int> &tiles, bool additive);

	uint64_t GetActionHash(slideDir act) const;
	void OpenGLDraw() const;
	void OpenGLDraw(const MNPuzzleState &s) const;
	void OpenGLDraw(const MNPuzzleState &l1, const MNPuzzleState &l2, float v) const;
	void OpenGLDraw(const MNPuzzleState &, const slideDir &) const { /* currently not drawing moves */ }
	void StoreGoal(MNPuzzleState &); // stores the locations for the given goal state

	/** Returns stored goal state if it is stored.**/
	MNPuzzleState Get_Goal(){
		if(!goal_stored) {
			fprintf(stderr, "ERROR: Call to Get_Goal when no goal stored\n");
			exit(1);
		}
		return goal;
	}

	virtual const std::string GetName();

	void ClearGoal(); // clears the current stored information of the goal

	bool IsGoalStored(){return goal_stored;} // returns if a goal is stored or not
	Graph *GetGraph();

	/**
	Changes the ordering of operators to the new inputted order
	**/
	void Change_Op_Order(const std::vector<slideDir> op_order);

	/**
	Creates num_puzzles random MN puzzles of the specified size and stores them in
	puzzle-vector. All random puzzles are unique and solvable for the standard goal
	of 0, 1, 2, ..., MN - 1. The method assumes that num_puzzles is no bigger than
	the total number of solvable problems for that size.
	**/
	static void Create_Random_MN_Puzzles(MNPuzzleState &goal, std::vector<MNPuzzleState> &puzzle_vector, unsigned num_puzzles);

	/**
	Reads in the the desired number of puzzles from the given filename with the
	given dimensions and stores them in puzzle_vector. Only the first max_puzzles
	are stored. first_counter should be set to true if the first element on every
	row is the index of the puzzle. The input format is the entries in each of
	the puzzle positions separated by a space, with the optional index entry
	described above.
	**/
	static int read_in_mn_puzzles(const char *filename, bool first_counter, unsigned num_cols, unsigned num_rows, unsigned max_puzzles, std::vector<MNPuzzleState> &puzzle_vector);

	/**
	Returns a possible ordering of the operators. The orders are in a "lexicographic"
	with the original ordering being Up, Left, Right, Down. This is therefore the order
	returned with a call of order_num=0. This initial ordering is that used by Korf in
	his original IDA* experiments. The ordering originally used in HOG is returned with a
	call of order_num=15.
	**/
	static std::vector<slideDir> Get_Op_Order_From_Hash(int order_num);
	std::vector<slideDir> Get_Op_Order(){return ops_in_order;}

	static MNPuzzleState Generate_Random_Puzzle(unsigned num_cols, unsigned num_rows);
	virtual void GetStateFromHash(MNPuzzleState &s, uint64_t hash);

	
	bool State_Check(const MNPuzzleState &to_check);

	unsigned Get_Num_Of_Columns(){return width;}
	unsigned Get_Num_Of_Rows(){return height;}

	void Set_Use_Manhattan_Heuristic(bool to_use){use_manhattan = to_use;}


private:
	double DoPDBLookup(const MNPuzzleState &state);
	std::vector<std::vector<uint8_t> > PDB;
	std::vector<std::vector<int> > PDBkey;
	unsigned int width, height;
	std::vector<std::vector<slideDir> > operators; // stores the operators applicable at each blank position
	std::vector<slideDir> ops_in_order;
	bool goal_stored; // whether a goal is stored or not
	bool use_manhattan;

	// stores the heuristic value of each tile-position pair indexed by the tile value (0th index is empty)
	unsigned **h_increment;
	MNPuzzleState goal;
};

class GraphPuzzleDistanceHeuristic : public GraphDistanceHeuristic {
public:
	GraphPuzzleDistanceHeuristic(MNPuzzle &mnp, Graph *graph, int count);
	double HCost(const graphState &state1, const graphState &state2);
private:
	MNPuzzle puzzle;
};

typedef UnitSimulation<MNPuzzleState, slideDir, MNPuzzle> PuzzleSimulation;

#endif
