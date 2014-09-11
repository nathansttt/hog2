#include "PancakePuzzle.h"
#include <cstdlib>

PancakePuzzle::PancakePuzzle(unsigned s) {
	assert(s >= 2);
	size = s;

	// assign the default operator ordering
	for(unsigned i = s; i >= 2; i--)
		operators.push_back(i);

	goal_stored = false;
	use_memory_free = true;
	use_dual_lookup = true;
}

PancakePuzzle::PancakePuzzle(unsigned s, const std::vector<unsigned> op_order) {
	size = s;

	Change_Op_Order(op_order);

	goal_stored = false;
	use_memory_free = false;
	use_dual_lookup = true;
}

PancakePuzzle::~PancakePuzzle()
{
	ClearGoal();
}

const std::string PancakePuzzle::GetName(){
	std::stringstream name;
	name << size;
	name << " Pancake Puzzle";

	if(PDB_distincts.size() > 0) {
		name << ", PDBS:";
		for(unsigned i = 0; i < PDB_distincts.size(); i++) {
			name << " <";
			for(unsigned j = 0; j < PDB_distincts[i].size() - 1; j++) {
				name << PDB_distincts[i][j];
				name << ", ";
			}
			name << PDB_distincts[i].back();
			name << ">";
		}
		if(use_memory_free)
		name << ", Memory-Free Heuristic";
	}
	else if(use_memory_free){
		name << ", Memory-Free Heuristic";
	}
	else {
		name << ",No Heuristic";
	}

	name << ", Op Order: ";
	for(unsigned op_num = 0; op_num < operators.size() - 1; op_num++){
		name << operators[op_num];
		name << ", ";
	}
	name << operators.back();
	return name.str();
}

void PancakePuzzle::GetSuccessors(const PancakePuzzleState &parent,
                             std::vector<PancakePuzzleState> &children) const
{
	children.resize(0);

	// all operators are applicable in all states
	for (unsigned i = 0; i < operators.size(); i++)
	{
		children.push_back(parent); // adds a copy of the state to the stack
		ApplyAction(children.back(), operators[i]);
	}
}

void PancakePuzzle::GetActions(const PancakePuzzleState &, std::vector<unsigned> &actions) const
{
	actions.resize(0);

	// all operators are applicable in all states
	for (unsigned i = 0; i < operators.size(); i++)
	{
		actions.push_back(operators[i]);
	}
}

unsigned PancakePuzzle::GetAction(const PancakePuzzleState &parent, const PancakePuzzleState &child) const
{
	unsigned current_action;
	bool are_equal = false;

	assert(child.puzzle.size() == size);
	assert(parent.puzzle.size() == size);
	PancakePuzzleState parentCopy = parent;
	for(unsigned i = 0; i < operators.size(); i++) {
		current_action = operators[i];
		ApplyAction(parentCopy, current_action);
		if(parentCopy == child)
			are_equal = true;
		InvertAction(current_action);
		ApplyAction(parentCopy, current_action);

		if(are_equal)
			return operators[i];
	}
	fprintf(stderr, "ERROR: GetAction called with non-adjacent states\n");
	exit(1);
	return 0;
}

void PancakePuzzle::ApplyAction(PancakePuzzleState &s, unsigned action) const
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

bool PancakePuzzle::InvertAction(unsigned &a) const
{
	// ever action is self-inverse
	assert(a > 1 && a <= size);
	return true;
}

double PancakePuzzle::HCost(const PancakePuzzleState &state) {
	if(!goal_stored) {
		fprintf(stderr, "ERROR: HCost called with a single state and goal is not stored.\n");
		exit(1);
	}
	if(state.puzzle.size() != size) {
		fprintf(stderr, "ERROR: HCost called with a single state with wrong size.\n");
		exit(1);
	}
	double h_cost = 0;

	// use PDB heuristic
	if(PDB.size() > 0) {
		if(PDB.size() != PDB_distincts.size()) {
			fprintf(stderr, "ERROR: HCost called with a single state, no use of memory free heuristic, and invalid setup of pattern databases.\n");
			exit(1);
		}
		h_cost = std::max(PDB_Lookup(state), h_cost);
	}

	// use memory-free heuristic
	if(use_memory_free) {
		h_cost =  std::max(Memory_Free_HCost(state, goal_locations), h_cost);
	}
	// if no heuristic
	else if(PDB.size()==0) {
		if(goal == state)
			return 0;
		else
			return 1;
	}

	return h_cost;
}

double PancakePuzzle::HCost(const PancakePuzzleState &state, const PancakePuzzleState &goal_state)
{
	if(state.puzzle.size() != size) {
		fprintf(stderr, "ERROR: HCost called with state with wrong size.\n");
		exit(1);
	}
	if(goal_state.puzzle.size() != size){
		fprintf(stderr, "ERROR: HCost called with goal with wrong size.\n");
		exit(1);
	}

	if( use_dual_lookup ) {
		// Note: This lookup only works if all PDB's have the same goal
		//   (or alternatively there is only one PDB)

		// sanity check
		assert( IsGoalStored() );

		// beta is a remapping of the elements to the goal
		std::vector<int> beta;
		beta.resize( size );
		for( unsigned int i = 0; i < size; i++ )
			beta[goal_state.puzzle[i]] = goal.puzzle[i];

		PancakePuzzleState t = state;
		// remap t with respect to beta
		for( unsigned int i = 0; i < size; i++ )
			t.puzzle[i] = beta[state.puzzle[i]];

		return PDB_Lookup( t );
	}

	if(use_memory_free) {
		std::vector<int> goal_locs(size);
		for(unsigned i = 0; i < size; i++) {
			goal_locs[goal_state.puzzle[i]] = i;
		}
		return Memory_Free_HCost(state, goal_locs);
	}

	if(state == goal_state)
		return 0.0;
	return 1.0;
}

double PancakePuzzle::Memory_Free_HCost(const PancakePuzzleState &state, std::vector<int> &goal_locs)
{
	if(state.puzzle.size() != size) {
		fprintf(stderr, "ERROR: HCost called with state with wrong size.\n");
		exit(1);
	}

	double h_count = 0.0;
	unsigned i = 0;
	for(; i < size - 1; i++) {
		int diff = goal_locs[state.puzzle[i]] - goal_locs[state.puzzle[i+1]];
		if(diff > 1 || diff < - 1)
			h_count++;
	}
	if((unsigned) goal_locs[state.puzzle[i]]!= size -1)
		h_count++;

	return h_count;
}

bool PancakePuzzle::GoalTest(const PancakePuzzleState &state, const PancakePuzzleState &theGoal)
{
	return (state == theGoal);
}

bool PancakePuzzle::GoalTest(const PancakePuzzleState &s) {
	if(!goal_stored) {
		fprintf(stderr, "ERROR: GoalTest called with a single state and goal is not stored.\n");
		exit(1);
	}
	if(s.puzzle.size() != size) {
		fprintf(stderr, "ERROR: GoalTest called with a single state with wrong size.\n");
		exit(1);
	}
	return (s == goal);
}

uint64_t PancakePuzzle::GetActionHash(unsigned act) const
{
	return (uint64_t) act;
}

void PancakePuzzle::StoreGoal(PancakePuzzleState &g) {
	assert(g.puzzle.size() == size);

	goal = g;
	goal_stored = true;

	goal_locations.resize(size);
	for(unsigned i = 0; i < size; i++) {
		goal_locations[goal.puzzle[i]] = i;
	}
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


void PancakePuzzle::Create_Random_Pancake_Puzzles(std::vector<PancakePuzzleState> &puzzle_vector, unsigned size, unsigned num_puzzles) {

	std::map<uint64_t, uint64_t> puzzle_map; // used to ensure uniqueness

	PancakePuzzle my_puzz(size);

	unsigned count = 0;

	std::vector<int> perm;
	PancakePuzzleState potential_puzz(size);
	while (count < num_puzzles) {
		perm = Get_Random_Permutation(size);

		// construct puzzle
		for(unsigned i = 0; i < size; i++) {
			potential_puzz.puzzle[i] = perm[i];
		}

		uint64_t next_hash = my_puzz.GetStateHash(potential_puzz);


		// make sure is not a duplicate
		if (puzzle_map.find(next_hash) != puzzle_map.end()) {
			continue;
		}

		puzzle_map[next_hash] = next_hash;
		puzzle_vector.push_back(potential_puzz);
		count++;
	}

}

int PancakePuzzle::read_in_pancake_puzzles(const char *filename, bool puzz_num_start, unsigned size, unsigned max_puzzles, std::vector<PancakePuzzleState> &puzzles) {

	std::vector<std::vector<int> > permutations;
	Read_In_Permutations(filename, size, max_puzzles, permutations, puzz_num_start);

	// convert permutations into PancakePuzzleStates
	for(unsigned i = 0; i < permutations.size(); i++) {
		PancakePuzzleState new_state(size);

		for(unsigned j = 0; j < size; j++) {
			new_state.puzzle[j] = permutations[i][j];
		}
		puzzles.push_back(new_state);
	}
	return 0;
}

bool PancakePuzzle::Path_Check(PancakePuzzleState start, PancakePuzzleState theGoal, std::vector<unsigned> &actions) {

	if(start.puzzle.size() != size || theGoal.puzzle.size() != size)
		return false;

	for(unsigned i = 0; i < actions.size(); i++) {
		if(actions[i] < 2 || actions[i] > size)
			return false;
		ApplyAction(start, actions[i]);
	}

	if(start == theGoal)
		return true;

	return false;
}

std::vector<unsigned> PancakePuzzle::Get_Puzzle_Order(int64_t order_num, unsigned num_pancakes) {
	std::vector<unsigned> ops;
	assert(order_num >= 0);
	assert(num_pancakes > 0);


	std::vector<int64_t> op_nums(num_pancakes -1);

	int64_t num_left = 1;
	for(int64_t x = num_pancakes - 2; x >= 0; x--) {
		op_nums[x] = order_num % num_left;
		order_num /= num_left;
		num_left++;

		for(int64_t y = x+1; y < num_pancakes-1; y++) {
			if(op_nums[y] >= op_nums[x]) {
				op_nums[y]++;
			}
		}
	}

	std::vector<bool> actions;

	for(unsigned i = 0; i < num_pancakes - 1; i++) {
		actions.push_back(false);
	}

	for(unsigned i = 0; i < num_pancakes - 1; i++) {
		ops.push_back(op_nums[i] + 2);
		actions[op_nums[i]] = true;
	}

	for(unsigned i = 0; i < num_pancakes - 1; i++) {
		assert(actions[i]);
	}
	return ops;
}
