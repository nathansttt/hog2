#include "experiment_basics.h"

std::string int_to_string(int i) {
	std::stringstream s;
	s << i;
	return s.str();
}

void warnings() {
	MNPuzzleState start(4, 4);
	if(start == start) {
		std::cout << kUp;
		std::cout << start;
	}
}

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss(s);
	std::string item;
	while(std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
	std::vector<std::string> elems;
	return split(s, delim, elems);
}


int read_in_extra_puzz_info(const char *filename, std::vector<Puzzle_Info> &info, char delim, unsigned max_info) {
	std::ifstream ifs(filename);

	if(ifs.fail()) {
		return 1;
	}

	std::string s, temp;
	std::vector<std::string> tokens;

	unsigned count = 0;
	while(!ifs.eof() && count < max_info) {
		getline(ifs, s);

		tokens.resize(0);

		tokens = split(s, delim);

		if(tokens.size() < 2)
			return 1;

		double nodes = atof(tokens[0].c_str());
		double cost = atof(tokens[1].c_str());

		Puzzle_Info new_info(nodes, cost);
		info.push_back(new_info);
		count++;
	}

	ifs.close();

	return 0;
}

template <class state, class action, class environment>
uint64_t general_batch(environment *env, GenericStepAlgorithm<state, action, environment> *solver, std::vector<state> &puzzles, state &goal, bool print_all_stats, unsigned type) {
	double total_checked = 0;
	double total_touched = 0;
	double total_expanded = 0;

	double total_cost = 0.0;

	std::vector<action> action_path;
	std::vector<state> state_path;

	unsigned solved_problems = 0;
	for(unsigned i = 0; i < puzzles.size(); i++) {

		unsigned status = 0;
		if(type == ACTION_PATH) {
			status = solver->GetPath(env, puzzles[i], goal, action_path);
		}
		else if(type == STATE_PATH) {
			status = solver->GetPath(env, puzzles[i], goal, state_path);
		}

		if(status != 1) { // if no solution was found
			if(print_all_stats) {
				printf("-1\n");
			}
			continue;
		}

		solved_problems++;

		total_cost += solver->GetPathCost();
		total_checked += solver->GetNodesChecked();
		total_touched += solver->GetNodesTouched();
		total_expanded += solver->GetNodesExpanded();

		if(print_all_stats) {
			printf("%.0f\t", solver->GetPathCost());
			printf("%llu\t", solver->GetNodesExpanded());
			printf("%llu\t", solver->GetNodesChecked());
			printf("%llu\t", solver->GetNodesTouched());
			printf("\n");
		}
	}

	printf("TOTALS\t%.0f\t%.0f\t%.0f\t%.0f\t%d", total_cost, total_expanded, total_checked, total_touched, solved_problems);
	printf("\n");
	return (uint64_t) total_checked;
}

uint64_t general_batch_puzzles(unsigned num_cols, unsigned num_rows, GenericStepAlgorithm<MNPuzzleState, slideDir, MNPuzzle> *solver, std::vector<MNPuzzleState> &puzzles, const std::vector<slideDir> op_order, bool print_all_stats, unsigned type) {
	MNPuzzleState goal(num_cols, num_rows);

	MNPuzzle mnp(num_cols, num_rows, op_order);
	mnp.StoreGoal(goal);

	for(unsigned i = 0; i < puzzles.size(); i++) {
		if(puzzles[i].width != goal.width || puzzles[i].height != goal.height) {
			std::cerr << puzzles[i] << '\n';
			std::cerr << goal << '\n';
			std::cerr << "Invalid Puzzle\n";
			return 0;
		}
	}

	return general_batch(&mnp, solver, puzzles, goal, print_all_stats, type);
}

unsigned best_of_combo(std::vector<unsigned> &combo, double **nodes, unsigned problem_num) {
	unsigned best_index = 0;
	for(unsigned i = 0; i < combo.size(); i++) {
		if(nodes[combo[i]][problem_num] < nodes[combo[best_index]][problem_num]) { // of best needs updating
			best_index = i;
		}
	}
	return best_index;
}

void thousand_info_nodes(std::vector<double> &t_info) {
	t_info.push_back(392554316);
	t_info.push_back(85591103);
	t_info.push_back(80675597);
	t_info.push_back(76056204);
	t_info.push_back(78210698);
	t_info.push_back(86768788);
	t_info.push_back(87620477);
	t_info.push_back(125487454);
	t_info.push_back(181722642);
	t_info.push_back(227524899);
	t_info.push_back(318783997);
	t_info.push_back(428986311);
	t_info.push_back(608318026);
	t_info.push_back(549063618);
	t_info.push_back(802526526);
	t_info.push_back(863793037);
	t_info.push_back(1226969606.0);
	t_info.push_back(1716175992.0);
	t_info.push_back(2083351131.0);
	t_info.push_back(2783187131.0);
	t_info.push_back(2912838379.0);
	t_info.push_back(4217010467.0);
	t_info.push_back(5178759400.0);
	t_info.push_back(7665834854.0);
}

void hundred_info_nodes(std::vector<double> &t_info) {
	t_info.push_back(22940770);
	t_info.push_back(5947832);
	t_info.push_back(6045138);
	t_info.push_back(6798974);
	t_info.push_back(7925958);
	t_info.push_back(5721882);
	t_info.push_back(10313715);
	t_info.push_back(10138364);
	t_info.push_back(13882196);
	t_info.push_back(21788151);
	t_info.push_back(33227438);
	t_info.push_back(30463845);
	t_info.push_back(39212893);
	t_info.push_back(44965543);
}

void get_standard_test_set(std::vector<MNPuzzleState> &puzzles, std::vector<Puzzle_Info> &info, std::vector<double> &solver_info, unsigned num) {
	solver_info.clear();
	solver_info.push_back(22940770);
	solver_info.push_back(5947832);
	solver_info.push_back(6045138);
	solver_info.push_back(6798974);
	solver_info.push_back(7925958);
	solver_info.push_back(5721882);
	solver_info.push_back(10313715);
	solver_info.push_back(10138364);
	solver_info.push_back(13882196);
	solver_info.push_back(21788151);
	solver_info.push_back(33227438);
	solver_info.push_back(30463845);
	solver_info.push_back(39212893);
	solver_info.push_back(44965543);

	read_in_extra_puzz_info("../../apps/dynamicsearch/input/4x4_100_info", info, ' ', num);
	MNPuzzle::read_in_mn_puzzles("../../apps/dynamicsearch/input/4x4_100", false, 4, 4, num, puzzles);
}

void get_big_4x4_test_set(std::vector<MNPuzzleState> &puzzles, std::vector<Puzzle_Info> &info, std::vector<double> &solver_info, unsigned num) {
	solver_info.clear();
	solver_info.push_back(392554316);
	solver_info.push_back(85591103);
	solver_info.push_back(80675597);
	solver_info.push_back(76056204);
	solver_info.push_back(78210698);
	solver_info.push_back(86768788);
	solver_info.push_back(87620477);
	solver_info.push_back(125487454);
	solver_info.push_back(181722642);
	solver_info.push_back(227524899);
	solver_info.push_back(318783997);
	solver_info.push_back(428986311);
	solver_info.push_back(608318026);
	solver_info.push_back(549063618);
	solver_info.push_back(802526526);
	solver_info.push_back(863793037);
	solver_info.push_back(1226969606.0);
	solver_info.push_back(1716175992.0);
	solver_info.push_back(2083351131.0);
	solver_info.push_back(2783187131.0);
	solver_info.push_back(2912838379.0);
	solver_info.push_back(4217010467.0);
	solver_info.push_back(5178759400.0);
	solver_info.push_back(7665834854.0);
	read_in_extra_puzz_info("../../apps/dynamicsearch/input/4x4_1000_info", info, ' ', num);
	MNPuzzle::read_in_mn_puzzles("../../apps/dynamicsearch/input/4x4_1000", true, 4, 4, num, puzzles);
}

void get_4x5_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num) {
	MNPuzzle::read_in_mn_puzzles("../../apps/dynamicsearch/input/4x5_1000", false, 4, 5, num, puzzles);
}

void get_5x4_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num) {
	MNPuzzle::read_in_mn_puzzles("../../apps/dynamicsearch/input/5x4_1000", false, 5, 4, num, puzzles);
}

void get_3x6_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num) {
	MNPuzzle::read_in_mn_puzzles("../../apps/dynamicsearch/input/3x6_1000", false, 3, 6, num, puzzles);
}

void get_6x3_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num) {
	if(MNPuzzle::read_in_mn_puzzles("../../apps/dynamicsearch/input/6x3_1000", false, 6, 3, num, puzzles)) {
		std::cerr << "File Reading Failed\n";
	}
}

void get_5x5_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num) {
	if(MNPuzzle::read_in_mn_puzzles("../../apps/dynamicsearch/input/5x5_1000", false, 5, 5, num, puzzles)) {
		std::cerr << "File Reading Failed\n";
	}
}

void get_node_buckets(std::vector<double> &node_vec, double smallest, unsigned num_buckets, double bucket_size) {
	printf("Num of Buckets: %d\n", num_buckets);
	printf("Bucket Size: %.2f\n", bucket_size);

	unsigned counter[num_buckets];
	unsigned curr_bucket = 0;
	for(unsigned i = 0; i < num_buckets; i++) {
		counter[i] = 0;
	}

	for(unsigned i = 0; i < node_vec.size(); i++) {
		curr_bucket = (unsigned)((node_vec[i] - smallest)/bucket_size);
		counter[curr_bucket]++;
	}

	for(unsigned i = 0; i < num_buckets; i++) {
		printf("%d\t%.0f\t%d\n", i, smallest + bucket_size*(i + 1), counter[i]);
	}
}
int get_distribution(const char *filename, double _num_buckets, double _bucket_size) {
	std::ifstream ifs(filename);

	if(ifs.fail()) {
		return 1;
	}

	std::string s, temp;
	std::vector<std::string> tokens;
	std::vector<double> node_vec;

	double smallest = 0;
	double biggest = 0;

	unsigned count = 0;
	while(!ifs.eof()) {
		getline(ifs, s);
		//std::cout << s << "\n";
		tokens.resize(0);

		tokens = split(s, '\t');

		if(tokens.size() < 2)
			continue;

		double nodes = atof(tokens[1].c_str());
		//std::cout << nodes << "\n";
		if(count == 0) {
			biggest = nodes;
			smallest = nodes;
		}
		else if(nodes > biggest) {
			biggest = nodes;
		}
		else if(nodes < smallest) {
			smallest = nodes;
		}
		node_vec.push_back(nodes);
		count++;
	}

	ifs.close();
	printf("Smallest: %.0f\n", smallest);
	printf("Biggest: %.0f\n", biggest);

	double num_buckets = 0;
	double bucket_size = 0;
	if(_bucket_size > 0) {
		bucket_size = ceil(_bucket_size);
		num_buckets = ceil((biggest - smallest)/ bucket_size);

		get_node_buckets(node_vec, smallest, (unsigned)num_buckets, (unsigned)bucket_size);

	}
	printf("\n\n\n");
	if(_num_buckets > 0) {
		num_buckets = ceil(_num_buckets);
		bucket_size = ceil((biggest - smallest)/ num_buckets);
		get_node_buckets(node_vec, smallest, (unsigned)num_buckets, bucket_size);
	}
	return 0;
}

bool get_next_combo(std::vector<unsigned> &current_combination, unsigned max_num, unsigned pos, bool set_as_smallest) {
	assert(max_num >= current_combination.size() - 1);

	// if are to set all positions from here on out as minimum possible
	if(set_as_smallest) {
		// if is the first position
		if(pos == 0) {
			current_combination[0] = 0;
			return get_next_combo(current_combination, max_num, pos + 1, true);
		}

		// make sure there are enough positions for remaining pieces
		assert(current_combination[pos - 1] + current_combination.size() - pos <= max_num);

		// assign as smallest possible
		current_combination[pos] = current_combination[pos - 1] + 1;
		if(pos == current_combination.size() - 1)
			return true;

		return get_next_combo(current_combination, max_num, pos+1, true);
	}

	// if have hit a number that is too high, must backtrack
	if(current_combination[pos] + current_combination.size() - pos - 1 == max_num) {
		return false;
	}
	else if(pos == current_combination.size() - 1) {
		current_combination[pos]++;
		return true;
	}

	if(!get_next_combo(current_combination, max_num, pos+1, false)) {
		current_combination[pos]++;
		return get_next_combo(current_combination, max_num, pos+1, true);
	}

	return true;
}

void print_combo(std::vector<unsigned> &current_combination) {
	for(unsigned i = 0; i < current_combination.size(); i++) {
		printf("%u ", current_combination[i]);
	}
	printf("\n");
}

void get_rand_combo(std::vector<unsigned> &combo, unsigned start_w_index, unsigned end_w_index, unsigned size) {
	std::vector<unsigned> solver_index;
	solver_index.clear();
	combo.clear();

	// initialize solver_index
	for(unsigned i = start_w_index; i <= end_w_index; i++) {
		solver_index.push_back(i);
	}

	// construct combo
	for(unsigned i = 0; i < size; i++) {
		int r = rand();
		int index = r % solver_index.size();
		combo.push_back(solver_index[index]);

		solver_index[index] = solver_index.back();
		solver_index.pop_back();
	}
}
