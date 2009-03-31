#include "PancakePuzzle.h"

PancakePuzzle::PancakePuzzle(unsigned s) {
	assert(s >= 2);
	size = s;

	// assign the default operator ordering
	for(unsigned i = s; i >= 2; i--)
		operators.push_back(i);

	goal_stored = false;
}

PancakePuzzle::PancakePuzzle(unsigned s, const std::vector<unsigned> op_order) {
	size = s;

	Change_Op_Order(op_order);

	goal_stored = false;
}

PancakePuzzle::~PancakePuzzle()
{
	ClearGoal();
}

void PancakePuzzle::GetSuccessors(PancakePuzzleState &parent,
                             std::vector<PancakePuzzleState> &children)
{
	children.resize(0);

	// all operators are applicable in all states
	for (unsigned i = 0; i < operators.size(); i++)
	{
		children.push_back(parent); // adds a copy of the state to the stack
		ApplyAction(children.back(), operators[i]);
	}
}

void PancakePuzzle::GetActions(PancakePuzzleState &state, std::vector<unsigned> &actions)
{
	actions.resize(0);

	// all operators are applicable in all states
	for (unsigned i = 0; i < operators.size(); i++)
	{
		actions.push_back(operators[i]);
	}
}

unsigned PancakePuzzle::GetAction(PancakePuzzleState &parent, PancakePuzzleState &child)
{
	unsigned current_action;
	bool are_equal = false;

	assert(child.puzzle.size() == size);
	assert(parent.puzzle.size() == size);
	for(unsigned i = 0; i < operators.size(); i++) {
		current_action = operators[i];
		ApplyAction(parent, current_action);

		if(parent == child)
			are_equal = true;

		InvertAction(current_action);
		ApplyAction(parent, current_action);

		if(are_equal)
			return operators[i];
	}
	fprintf(stderr, "ERROR: GetAction called with non-adjacent states\n");
	exit(1);
	return 0;
}

void PancakePuzzle::ApplyAction(PancakePuzzleState &s, unsigned action)
{
	assert(s.puzzle.size() == size);
	assert(action > 1 && action <= size);

	int upper = 0;
	int lower = action - 1;
	int temp;
	// performs pancake flipping
	while(upper < lower) {
		temp = s.puzzle[upper];
		s.puzzle[upper] = s.puzzle[lower];
		s.puzzle[lower] = temp;
		upper++;
		lower--;
	}
}

bool PancakePuzzle::InvertAction(unsigned &a)
{
	// ever action is self-inverse
	assert(a > 1 && a <= size);
	return true;
}

double PancakePuzzle::HCost(PancakePuzzleState &state) {
	assert(goal_stored);
	assert(state.puzzle.size() == size);

	return 0.0;
}

double PancakePuzzle::HCost(PancakePuzzleState &state1, PancakePuzzleState &state2)
{
	assert(state1.puzzle.size() == size);
	assert(state2.puzzle.size() == size);

	return 0.0;
}

bool PancakePuzzle::GoalTest(PancakePuzzleState &state, PancakePuzzleState &goal)
{
	return (state == goal);
}

bool PancakePuzzle::GoalTest(PancakePuzzleState &s) {
	assert(goal_stored);
	assert(s.puzzle.size() == size);
	for(unsigned i = 0; i < s.puzzle.size(); i++) {
		if(s.puzzle[i] != goal.puzzle[i])
			return false;
	}
	return true;
}

uint64_t PancakePuzzle::GetActionHash(unsigned act)
{
	return (uint64_t) act;
}

void PancakePuzzle::StoreGoal(PancakePuzzleState &g) {
	assert(g.puzzle.size() == size);

	goal = g;
}

void PancakePuzzle::Change_Op_Order(const std::vector<unsigned> op_order) {
	operators.clear();

	if(op_order.size() != size - 1) {
		fprintf(stderr, "ERROR: Not enough operators in operator sequence for construction of PancakePuzzle\n");
		exit(1);
	}

	bool all_ops[op_order.size()];

	for(unsigned i = 0; i < size; i++) {
		all_ops[i] = false;
	}

	for(unsigned i = 0; i < op_order.size(); i++) {
		if(op_order[i] < 2 || op_order[i] > size) {
			fprintf(stderr, "ERROR: Invalid operator included in construction of PancakePuzzle\n");
			exit(1);
		}
		all_ops[op_order[i] - 2] = true;
	}

	for(unsigned i = 0; i < op_order.size(); i++) {
		if(!all_ops[i]) {
			fprintf(stderr, "ERROR: Missing operator %u in construction of PancakePuzzle\n", i+2);
			exit(1);
		}
	}
	// assign the default operator ordering
	for(unsigned i = 0; i < op_order.size(); i++)
		operators.push_back(op_order[i]);
}
