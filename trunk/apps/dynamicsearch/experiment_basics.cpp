#include "experiment_basics.h"

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
		getline(ifs, s);;

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

double batch_puzzles(GeneralIDA<MNPuzzleState, slideDir> *ida, std::vector<MNPuzzleState> &puzzles, std::vector<Puzzle_Info> &info, std::vector<slideDir> &op_order, bool print_all_stats) {
	double total_checked = 0;
	double total_generated = 0;
	double total_expanded = 0;

	double total_av_checked = 0.0;
	double total_cost = 0.0;
	double total_av_cost = 0.0;

	unsigned total_iters = 0;

	MNPuzzleState goal(4, 4);
	std::vector<slideDir> path;

	MNPuzzle mnp(4, 4, op_order);
	mnp.StoreGoal(goal);

	for(unsigned i = 0; i < puzzles.size(); i++) {
		ida->GetPath(&mnp, puzzles[i], goal, path);

		total_cost += ida->GetPathCost();
		total_checked += ida->GetNodesChecked();
		total_generated += ida->GetNodesGenerated();
		total_expanded += ida->GetNodesExpanded();
		total_iters += ida->GetNumIters();

		if(info.size() == puzzles.size()) {
			total_av_checked += info[i].first/ ida->GetNodesChecked();
			total_av_cost += ida->GetPathCost() / info[i].second;
		}

		if(print_all_stats) {
			printf("%.0f %.0f %d", ida->GetNodesChecked(), ida->GetPathCost(), ida->GetNumIters());

			if(info.size() == puzzles.size()) {
				printf(" %.2f %.2f", info[i].first/ ida->GetNodesChecked(), ida->GetPathCost() / info[i].second);
			}
			std::vector<double> iter = ida->Get_Checked_Iters();
			for(unsigned j = 0; j < iter.size(); j++) {
				printf("\t%.0f", iter[j]);
			}
			printf("\n");
		}
	}

	printf("%.0f %.0f %d", total_checked, total_cost, total_iters);
	if(info.size() == puzzles.size()) {
		printf(" %.2f %.2f", total_av_checked / (double) puzzles.size(), total_av_cost / (double) puzzles.size());
	}
	printf("\n");
	return total_checked;
}

/**
Performs the oracle experiment on the given set of problem with the given set of solvers. That is, what is
the total number of nodes if the perfect one was chosen for each problem (in terms of nodes checked)
**/
double oracle_experiment(std::vector<GeneralIDA<MNPuzzleState, slideDir> *> solvers, std::vector<MNPuzzleState> &puzzles, std::vector<Puzzle_Info> &info, std::vector<slideDir> &op_order, bool print_all_stats) {
	double best_total_checked = 0;
	double best_total_generated = 0;
	double best_total_expanded = 0;

	double best_total_av_checked = 0.0;
	double best_total_cost = 0.0;
	double best_total_av_cost = 0.0;

	MNPuzzleState goal(4, 4);
	std::vector<slideDir> path;

	MNPuzzle mnp(4, 4, op_order);
	mnp.StoreGoal(goal);

	double current_best_checked = 0.0;
	double current_best_generated = 0.0;
	double current_best_expanded = 0.0;

	double current_best_cost = 0.0;
	double current_best_av_checked = 0.0;
	double current_best_av_cost = 0.0;

	double current_largest = 0;
	std::vector<double> its;

	unsigned solver_counter[solvers.size()];
	std::vector<std::vector <double> > solver_nodes;
	std::vector<std::vector <double> > solver_cost;
	double solver_total[solvers.size()];

	for(unsigned i = 0; i < solvers.size(); i++) {
		solver_counter[i] = 0;
		solver_total[i] = 0.0;
		std::vector<double> new_vec;
		std::vector<double> new_vec2;

		solver_nodes.push_back(new_vec);
		solver_cost.push_back(new_vec2);
	}

	unsigned best_solver;
	for(unsigned i = 0; i < puzzles.size(); i++) {
		current_best_checked = 0.0;
		best_solver = 0;
		for(unsigned j = 0; j < solvers.size(); j++) {
			solvers[j]->SetCheckedLimit(current_best_checked);
			// didn't solve problem with current limit
			if(solvers[j]->GetPath(&mnp, puzzles[i], goal, path) != 1)
				continue;

			if(current_best_checked == 0 || solvers[j]->GetNodesChecked() < current_best_checked || (solvers[j]->GetNodesChecked() == current_best_checked && solvers[j]->GetPathCost() < current_best_cost)) {

				best_solver = j;
				current_best_checked = solvers[j]->GetNodesChecked();
				current_best_generated = solvers[j]->GetNodesGenerated();
				current_best_expanded = solvers[j]->GetNodesExpanded();

				current_best_cost = solvers[j]->GetPathCost();

				if(info.size() == puzzles.size()) {
					current_best_av_checked = info[i].first/ solvers[j]->GetNodesChecked();
					current_best_av_cost = solvers[j]->GetPathCost() / info[i].second;
				}
				its = solvers[j]->Get_Checked_Iters();
			}
		}

		best_total_checked += current_best_checked;
		best_total_generated += current_best_generated;
		best_total_expanded += current_best_expanded;

		best_total_cost += current_best_cost;
		best_total_av_checked += current_best_av_checked;
		best_total_av_cost += current_best_av_cost;

		if(print_all_stats) {
			printf("Solver: %d, %.0f %.0f", best_solver, current_best_checked, current_best_cost);
			if(info.size() == puzzles.size()) {
				printf(" %.2f %.2f", current_best_av_checked, current_best_av_cost);
			}
			for(unsigned j = 0; j < its.size(); j++) {
				printf("\t%.0f", its[j]);
			}
			printf("\n");
		}
		solver_counter[best_solver]++;
		solver_total[best_solver] += current_best_checked;
		solver_nodes[best_solver].push_back(current_best_checked);
		solver_cost[best_solver].push_back(current_best_cost);


		if(current_best_checked > current_largest)
			current_largest = current_best_checked;
	}

	printf("%.0f %.0f", best_total_checked, best_total_cost);
	if(info.size() == puzzles.size()) {
		printf(" %.2f %.2f", best_total_av_checked / (double) puzzles.size(), best_total_av_cost / (double) puzzles.size());
	}
	printf("\n");
	if (print_all_stats) {
		printf("Largest: %.0f\n", current_largest);

		bool go = true;
		unsigned index = 0;
		while (go) {
			go = false;
			for (unsigned i = 0; i < solver_nodes.size(); i++) {
				if (index < solver_nodes[i].size()) {
					printf("\t%.0f", solver_nodes[i][index]);
					go = true;
				} else {
					printf("\t");
				}
			}
			index++;
			printf("\n");
		}

		for (unsigned i = 0; i < solvers.size(); i++) {
			printf("\t%.2f", solver_total[i] / ((double) solver_counter[i]));
		}
		printf("\n");
		for (unsigned i = 0; i < solvers.size(); i++) {
			printf("%d ", solver_counter[i]);
		}
		printf("\n");
	}
	return best_total_checked;
}

void weight_gradient (std::vector<MNPuzzleState> &puzzles, std::vector<slideDir> &op_order) {
	GeneralIDA<MNPuzzleState, slideDir> ida;

	double base = 10000.0;
	unsigned init_marker = 39000.0;
	double marker = init_marker;// starting weight
	double upper_bound = 45000.0; // largest possible weight

	double current_marker = marker;
	double current_w = marker;
	double increment = 1024.0; // value incrementing by
	double increment_max = increment;

	std::vector<Puzzle_Info> info; // dummy
	double marker_nodes = 0.0; // current marker nodes

	std::vector<std::pair<double, double> > points; // recorded points - first is second, second is size of change
	std::vector<double> points_node;
	double calculated_weights[(unsigned)upper_bound - init_marker + 1]; // holds calculated weights

	for(unsigned i = 0; i < (unsigned)upper_bound - init_marker + 1; i++) {
		calculated_weights[i] = 0.0;
	}
	double temp_nodes;
	// while still have space to search
	while(marker <= upper_bound) {
		printf("%.0f: ", current_w);
		// if this weight has already been searched
		if(calculated_weights[(unsigned) current_w - init_marker] > 0.0) {
			temp_nodes = calculated_weights[(unsigned) current_w - init_marker];
			printf("%.0f\n", temp_nodes);
		}
		else { // otherwise, actually search
			ida.Change_Weights(base, current_w);
			temp_nodes = batch_puzzles(&ida, puzzles, info, op_order, false);
			calculated_weights[(unsigned)current_w - init_marker] = temp_nodes;
		}

		// if first iteration for current marker
		if(current_w == current_marker) {
			marker_nodes = temp_nodes;
			current_w += increment;
			continue;
		}

		// if current search produced the same as the stored marker nodes
		if(fequal(temp_nodes, marker_nodes)) {
			current_marker = current_w;
		}
		increment /= 2.0;

		// if binary search has hit limit
		if(increment < 1.0) {

			// if more than one position in a row
			if(current_marker > marker) {
				std::pair<double, double> new_pair(marker, current_marker - marker);
				points.push_back(new_pair);
			}
			else { // if binary search finds adjacent weights are different
				std::pair<double, double> new_pair(marker, 0.0);
				points.push_back(new_pair);
			}

			printf("%.0f %.0f %.0f\n", points.back().first, points.back().second, marker_nodes);
			std::cerr << points.back().first << std::endl;
			points_node.push_back(marker_nodes);
			increment = increment_max;
			marker = current_marker + 1.0;
			current_w = marker;
			current_marker = current_w;

		}
		else {
			current_w = current_marker + increment;
		}
	}

	for(unsigned i = 0; i < points.size(); i++) {
		printf("%.0f %.0f %.0f\n", points[i].first, points[i].second, points_node[i]);
	}
}

unsigned independent_solvers_run(std::vector<GeneralIDA<MNPuzzleState, slideDir> *> solvers, MNPuzzle *mnp, MNPuzzleState start, MNPuzzleState goal) {

	unsigned num_solvers = solvers.size();

	for(unsigned i = 0; i < num_solvers; i++) {
		solvers[i]->initialize_step_by_step(mnp, start, goal, false);
	}

	int status = 0;
	unsigned solver_counter = num_solvers - 1;
	std::vector<slideDir> path;
	while(status == 0) {
		solver_counter = (solver_counter + 1) % num_solvers;
		status = solvers[solver_counter]->move_one_step(mnp, goal, path);
	}

	return solver_counter;
}




void random_k_oracle(std::vector<std::pair<double, double> > &weights, std::vector<double> &solver_info, std::vector<MNPuzzleState> &puzzles, std::vector<Puzzle_Info> &info, std::vector<slideDir> &op_order, unsigned size, unsigned num){
	std::vector<GeneralIDA<MNPuzzleState, slideDir> *> solvers, rand_solvers;
	//srand(5);

	for(unsigned i = 0; i < weights.size(); i++) {
		solvers.push_back(new GeneralIDA<MNPuzzleState, slideDir>(weights[i].first, weights[i].second, false, false, false, false));
	}

	std::vector<unsigned> solver_index;
	unsigned count = 0;
	double max = 0.0;
	double min = 0.0;

	while(count < num) {
		solver_index.clear();
		rand_solvers.clear();
		for(unsigned i = 0; i < weights.size(); i++) {
			solver_index.push_back(i);
		}

		for(unsigned i = 0; i < size; i++) {
			int r = rand();
			int index = r % solver_index.size();
			printf("%.0f ", solvers[solver_index[index]]->Get_H_Weight());
			rand_solvers.push_back(solvers[solver_index[index]]);
			solver_index[index] = solver_index.back();
			solver_index.pop_back();
		}
		printf("\t");

		double temp_nodes = oracle_experiment(rand_solvers, puzzles, info, op_order, false);

		if(count == 0) {
			max = temp_nodes;
			min = temp_nodes;
		}
		else if(min > temp_nodes) {
			min = temp_nodes;
		}
		else if(max < temp_nodes) {
			max = temp_nodes;
		}

		count++;
	}
	printf("\n\n\n");
	printf("MIN: %.0f\n", min);
	printf("MAX: %.0f\n", max);
}

double ind_batch_puzzles(std::vector<GeneralIDA<MNPuzzleState, slideDir> *> solvers, std::vector<MNPuzzleState> &puzzles, std::vector<Puzzle_Info> &info, std::vector<slideDir> &op_order, bool print_all_stats) {

	double total_checked = 0;
	double total_generated = 0;
	double total_expanded = 0;

	double total_av_checked = 0.0;
	double total_cost = 0.0;
	double total_av_cost = 0.0;

	MNPuzzleState goal(4, 4);
	std::vector<slideDir> path;

	MNPuzzle mnp(4, 4, op_order);
	mnp.StoreGoal(goal);

	double total_prob_nodes_checked;

	unsigned solved_by;

	unsigned solver_counter[solvers.size()];
	std::vector<std::vector <double> > solver_nodes;
	std::vector<std::vector <double> > solver_cost;
	double solver_total[solvers.size()];

	for(unsigned i = 0; i < solvers.size(); i++) {
		solver_counter[i] = 0;
		solver_total[i] = 0.0;
		std::vector<double> new_vec;
		std::vector<double> new_vec2;

		solver_nodes.push_back(new_vec);
		solver_cost.push_back(new_vec2);
	}

	for(unsigned i = 0; i < puzzles.size(); i++) {
		solved_by = independent_solvers_run(solvers, &mnp, puzzles[i], goal);

		total_prob_nodes_checked = 0.0;
		// total up node information
		for(unsigned j = 0; j < solvers.size(); j++) {
			solvers[j]->End_Step_By_Step();
			total_checked += solvers[j]->GetNodesChecked();
			total_prob_nodes_checked += solvers[j]->GetNodesChecked();

			total_generated += solvers[j]->GetNodesGenerated();
			total_expanded += solvers[j]->GetNodesExpanded();
		}
		total_cost += solvers[solved_by]->GetPathCost();

		if(info.size() == puzzles.size()) {
			total_av_checked += info[i].first/ total_prob_nodes_checked;
			total_av_cost += solvers[solved_by]->GetPathCost() / info[i].second;
		}

		if(print_all_stats) {
			printf("Solver: %d, ", solved_by);
			printf("%.0f %.0f %d", total_prob_nodes_checked, solvers[solved_by]->GetPathCost(), solvers[solved_by]->GetNumIters());

			if(info.size() == puzzles.size()) {
				printf(" %.2f %.2f", info[i].first/ total_prob_nodes_checked, solvers[solved_by]->GetPathCost() / info[i].second);
			}
			std::vector<double> iter = solvers[solved_by]->Get_Checked_Iters();
			for(unsigned j = 0; j < iter.size(); j++) {
				printf("\t%.0f", iter[j]);
			}
			printf("\n");
		}

		solver_counter[solved_by]++;
		solver_total[solved_by] += total_prob_nodes_checked;
		solver_nodes[solved_by].push_back(total_prob_nodes_checked);
		solver_cost[solved_by].push_back(solvers[solved_by]->GetPathCost());
	}

	printf("%.0f %.0f", total_checked, total_cost);
	if(info.size() == puzzles.size()) {
		printf(" %.2f %.2f", total_av_checked / (double) puzzles.size(), total_av_cost / (double) puzzles.size());
	}

	if(print_all_stats) {
		bool go = true;
		unsigned index = 0;
		while (go) {
			go = false;
			for (unsigned i = 0; i < solver_nodes.size(); i++) {
				if (index < solver_nodes[i].size()) {
					printf("\t%.0f", solver_nodes[i][index]);
					go = true;
				} else {
					printf("\t");
				}
			}
			index++;
			printf("\n");
		}

		for (unsigned i = 0; i < solvers.size(); i++) {
			printf("\t%.2f", solver_total[i] / ((double) solver_counter[i]));
		}
		printf("\n");
		for (unsigned i = 0; i < solvers.size(); i++) {
			printf("%d ", solver_counter[i]);
		}
		printf("\n");
	}
	printf("\n");
	return total_checked;
}

void random_ind_puzzles(std::vector<GeneralIDA<MNPuzzleState, slideDir> *> solvers, std::vector<double> &solver_info, std::vector<MNPuzzleState> &puzzles, std::vector<slideDir> &op_order) {
	std::vector<GeneralIDA<MNPuzzleState, slideDir> *> rand_solvers;

	std::vector<unsigned> solver_index;
	unsigned count = 0;
	double max = 0.0;
	double min = 0.0;

	while(count < num) {
		solver_index.clear();
		rand_solvers.clear();
		for(unsigned i = 0; i < weights.size(); i++) {
			solver_index.push_back(i);
		}

		for(unsigned i = 0; i < size; i++) {
			int r = rand();
			int index = r % solver_index.size();
			printf("%.0f ", solvers[solver_index[index]]->Get_H_Weight());
			rand_solvers.push_back(solvers[solver_index[index]]);
			solver_index[index] = solver_index.back();
			solver_index.pop_back();
		}
		printf("\t");

		double temp_nodes = oracle_experiment(rand_solvers, puzzles, info, op_order, false);

		if(count == 0) {
			max = temp_nodes;
			min = temp_nodes;
		}
		else if(min > temp_nodes) {
			min = temp_nodes;
		}
		else if(max < temp_nodes) {
			max = temp_nodes;
		}

		count++;
	}
	printf("\n\n\n");
	printf("MIN: %.0f\n", min);
	printf("MAX: %.0f\n", max);
}
