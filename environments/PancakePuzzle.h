#ifndef PANCAKE_H
#define PANCAKE_H

#include <stdint.h>
#include <iostream>
#include "SearchEnvironment.h"
#include "PermutationPuzzleEnvironment.h"
#include <sstream>

typedef unsigned PancakePuzzleAction;

class PancakePuzzleState {
public:
	PancakePuzzleState() { puzzle.clear(); }

	PancakePuzzleState(unsigned int puzzle_size) {
		puzzle.resize(puzzle_size);
		Reset();
	}
	void Reset()
	{
		for (unsigned int x = 0; x < puzzle.size(); x++)
			puzzle[x] = x;
	}
	std::vector<int> puzzle;
};

static std::ostream& operator <<(std::ostream & out, const PancakePuzzleState &loc)
{
	for (unsigned int x = 0; x < loc.puzzle.size(); x++)
		out << loc.puzzle[x] << " ";
	return out;
}

static bool operator==(const PancakePuzzleState &l1, const PancakePuzzleState &l2)
{
	if (l1.puzzle.size() != l2.puzzle.size())
		return false;

	for (unsigned int x = 0; x < l1.puzzle.size(); x++)
		if (l1.puzzle[x] != l2.puzzle[x])
			return false;
	return true;
}

class PancakePuzzle : public PermutationPuzzleEnvironment<PancakePuzzleState, PancakePuzzleAction> {
public:
	PancakePuzzle(unsigned s);
	PancakePuzzle(unsigned size, const std::vector<unsigned> op_order); // used to set action order

	~PancakePuzzle();
	void GetSuccessors(const PancakePuzzleState &state, std::vector<PancakePuzzleState> &neighbors) const;
	void GetActions(const PancakePuzzleState &state, std::vector<unsigned> &actions) const;
	PancakePuzzleAction GetAction(const PancakePuzzleState &s1, const PancakePuzzleState &s2) const;
	void ApplyAction(PancakePuzzleState &s, PancakePuzzleAction a) const;
	bool InvertAction(PancakePuzzleAction &a) const;

	double HCost(const PancakePuzzleState &state1, const PancakePuzzleState &state2);
	double DefaultH(const PancakePuzzleState &state1);
	double DefaultH(const PancakePuzzleState &state1, std::vector<int> &goal_locs);
	double HCost(const PancakePuzzleState &state1);

	double GCost(const PancakePuzzleState &, const PancakePuzzleState &) {return 1.0;}
	double GCost(const PancakePuzzleState &, const PancakePuzzleAction &) { return 1.0; }

	bool GoalTest(const PancakePuzzleState &state, const PancakePuzzleState &goal);

	bool GoalTest(const PancakePuzzleState &s);

	uint64_t GetActionHash(PancakePuzzleAction act) const;
	void StoreGoal(PancakePuzzleState &); // stores the locations for the given goal state

	virtual const std::string GetName();
	std::vector<PancakePuzzleAction> Get_Op_Order(){return operators;}

	/** Returns stored goal state if it is stored.**/
	PancakePuzzleState Get_Goal(){
		if(!goal_stored) {
			fprintf(stderr, "ERROR: Call to Get_Goal when no goal stored\n");
			exit(1);
		}
		return goal;
	}

	void ClearGoal(){} // clears the current stored information of the goal

	bool IsGoalStored(){return goal_stored;} // returns if a goal is stored or not

	/**
	Changes the ordering of operators to the new inputted order
	**/
	void Change_Op_Order(const std::vector<PancakePuzzleAction> op_order);

	// currently not drawing anything
	void OpenGLDraw() const{}
	void OpenGLDraw(const PancakePuzzleState &) const;
	void OpenGLDraw(const PancakePuzzleState &, const PancakePuzzleAction &) const {}
	void OpenGLDraw(const PancakePuzzleState&, const PancakePuzzleState&, float) const {}

	/**
	**/
	static void Create_Random_Pancake_Puzzles(std::vector<PancakePuzzleState> &puzzle_vector, unsigned size, unsigned num_puzzles);

	static int read_in_pancake_puzzles(const char *filename, bool first_counter, unsigned size, unsigned max_puzzles, std::vector<PancakePuzzleState> &puzzle_vector);

	bool State_Check(const PancakePuzzleState &to_check) {
		if(to_check.puzzle.size() != size)
			return false;

		return true;
	}

	bool Path_Check(PancakePuzzleState start, PancakePuzzleState goal, std::vector<PancakePuzzleAction> &actions);

	/**
	Returns a possible ordering of the operators. The orders are in a "lexicographic"
	with the original ordering being 2, 3, ..., num_pancakes. This is therefore the order
	returned with a call of order_num=0. The default ordering used when a PancakePuzzle
	environment is created is num_pancakes, ..., 2 which is returned with a call of
	num_pancakes! -1.
	**/
	static std::vector<PancakePuzzleAction> Get_Puzzle_Order(int64_t order_num, unsigned num_pancakes);

	void Set_Use_Memory_Free_Heuristic(bool to_use){use_memory_free = to_use;}
	void Set_Use_Dual_Lookup( bool to_use ) { use_dual_lookup = to_use; };

private:

	std::vector<PancakePuzzleAction> operators;
	bool goal_stored; // whether a goal is stored or not
	bool use_memory_free;
	bool use_dual_lookup;

	PancakePuzzleState goal;
	std::vector<int> goal_locations;
	unsigned size;
};

//typedef UnitSimulation<PancakePuzzleState, unsigned, Pancake> PancakeSimulation;
#endif
