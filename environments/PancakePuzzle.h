#ifndef PANCAKE_H
#define PANCAKE_H

#include <stdint.h>
#include <iostream>
#include "SearchEnvironment.h"
#include "PermutationPuzzleEnvironment.h"

class PancakePuzzleState {
public:
	PancakePuzzleState() { puzzle.clear(); }

	PancakePuzzleState(unsigned int puzzle_size) {
		puzzle.resize(puzzle_size);
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

class PancakePuzzle : public PermutationPuzzleEnvironment<PancakePuzzleState, unsigned> {
public:
	PancakePuzzle(unsigned s);
	PancakePuzzle(unsigned size, const std::vector<unsigned> op_order); // used to set action order

	~PancakePuzzle();
	void GetSuccessors(PancakePuzzleState &state, std::vector<PancakePuzzleState> &neighbors) const;
	void GetActions(PancakePuzzleState &state, std::vector<unsigned> &actions) const;
	unsigned GetAction(PancakePuzzleState &s1, PancakePuzzleState &s2) const;
	void ApplyAction(PancakePuzzleState &s, unsigned a) const;
	bool InvertAction(unsigned &a) const;

	double HCost(PancakePuzzleState &state1, PancakePuzzleState &state2);
	double HCost(PancakePuzzleState &state1);

	double GCost(PancakePuzzleState &state1, PancakePuzzleState &state2) {return 2.0;}
	double GCost(PancakePuzzleState &state1, unsigned &act) { return 1.0; }
	bool GoalTest(PancakePuzzleState &state, PancakePuzzleState &goal);

	bool GoalTest(PancakePuzzleState &s);

	uint64_t GetActionHash(unsigned act) const;
	void StoreGoal(PancakePuzzleState &); // stores the locations for the given goal state
	void ClearGoal(){} // clears the current stored information of the goal

	bool IsGoalStored(){return goal_stored;} // returns if a goal is stored or not

	/**
	Changes the ordering of operators to the new inputted order
	**/
	void Change_Op_Order(const std::vector<unsigned> op_order);

	// currently not drawing anything
	void OpenGLDraw() const{}
	void OpenGLDraw(const PancakePuzzleState &s) const {}
	void OpenGLDraw(const PancakePuzzleState &, const unsigned &) const {}

	/**
	**/
	static void Create_Random_Pancake_Puzzles(std::vector<PancakePuzzleState> &puzzle_vector, unsigned size, unsigned num_puzzles);

	static int read_in_pancake_puzzles(const char *filename, bool first_counter, unsigned size, unsigned max_puzzles, std::vector<PancakePuzzleState> &puzzle_vector);

	bool State_Check(const PancakePuzzleState &to_check) {
		if(to_check.puzzle.size() != size)
			return false;

		return true;
	}

	bool Path_Check(PancakePuzzleState start, PancakePuzzleState goal, std::vector<unsigned> &actions);

	/**
	Returns a possible ordering of the operators. The orders are in a "lexicographic"
	with the original ordering being 2, 3, ..., num_pancakes. This is therefore the order
	returned with a call of order_num=0. The default ordering used when a PancakePuzzle
	environment is created is num_pancakes, ..., 2 which is returned with a call of
	num_pancakes! -1.
	**/
	static std::vector<unsigned> Get_Puzzle_Order(int64_t order_num, unsigned num_pancakes);

private:

	std::vector<unsigned> operators;
	bool goal_stored; // whether a goal is stored or not

	PancakePuzzleState goal;
	unsigned size;
};

//typedef UnitSimulation<PancakePuzzleState, unsigned, Pancake> PancakeSimulation;
#endif
