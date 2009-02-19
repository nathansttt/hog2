#include "experiment_basics.h"
#include <stdio.h>

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

int output_exp_info(const char *filename, int num_probs, int num_solvers, const char *nodes_filename, const char *costs_filename) {

	FILE * output_nodes;
	FILE * output_costs;
	output_nodes = fopen (nodes_filename,"w");
	output_costs = fopen (costs_filename,"w");

	std::ifstream ifs(filename);

	if(ifs.fail()) {
		return 1;
	}

	std::string s, temp;
	std::vector<std::string> tokens;

	double nodes [num_solvers][num_probs];
	double costs [num_solvers][num_probs];
	double total_nodes [num_probs];
	double total_cost [num_probs];
	int total_its[num_solvers];
	double info1[num_solvers];
	double info2[num_solvers];

	int prob_count = 0;
	int solver_count = 0;

	int min_solver = 0;
	int max_solver = 0;
	double all_nodes = 0;
	double all_cost = 0;

	while(!ifs.eof()) {
		getline(ifs, s);
		//std::cout << "line: " << s << std::endl;

		if(s.find("=") != std::string::npos) {
		//if(s.find(":") != std::string::npos) {
			//printf("HERE\n");
			prob_count = 0;
			continue;
		}

		tokens.resize(0);

		tokens = split(s, '\t');

		if(tokens.size() == 0)
			continue;

		if(tokens.size() == 2 && prob_count < num_probs){
			assert(solver_count < num_solvers);
			assert(prob_count < num_probs);


			nodes[solver_count][prob_count] = atof(tokens[0].c_str());
			costs[solver_count][prob_count] = atof(tokens[1].c_str());

			prob_count++;
		}
		else {
			//std::cout << s << "\n";
			//printf("%s\n", s);
			assert(solver_count < num_solvers);

			tokens.resize(0);

			tokens = split(s, ' ');

			total_nodes[solver_count] = atof(tokens[1].c_str());
			total_cost[solver_count] = atof(tokens[0].c_str());

			if(tokens.size() == 5) {
				total_its[solver_count] = atoi(tokens[2].c_str());
				info1[solver_count] = atof(tokens[3].c_str());
				info2[solver_count] = atof(tokens[4].c_str());
			}


			if(total_nodes[solver_count] < total_nodes[min_solver])
				min_solver = solver_count;

			if(total_nodes[solver_count] > total_nodes[max_solver])
				max_solver = solver_count;

			all_nodes += total_nodes[solver_count];
			all_cost += total_cost[solver_count];
			solver_count++;
		}
	}
	ifs.close();

	for(int i = 0; i < num_probs; i++) {
		for(int j = 0; j < num_solvers; j++) {
			fprintf(output_nodes, "%.0f\t", nodes[j][i]);
		}
		fprintf(output_nodes, "\n");
	}
	//printf("\n");

	for(int i = 0; i < num_probs; i++) {
		for(int j = 0; j < num_solvers; j++) {
			fprintf(output_costs, "%.0f\t", costs[j][i]);
		}
		fprintf(output_costs,"\n");
	}

	//printf("\n");
	for(int i = 0; i < num_solvers; i++) {
		printf("%.0f\t%.0f\t%d\t%.2f\t%.2f\n", total_cost[i], total_nodes[i],
		       total_its[i], info1[i], info2[i]);
	}
	printf("\n%.0f\t%.0f\t\t\t%.0f\t%.0f\t\t\t%.0f\t%.0f\n", total_cost[min_solver], total_nodes[min_solver], total_cost[max_solver], total_nodes[max_solver], all_cost/ (double) num_solvers, all_nodes / (double) num_solvers);
	//printf("\nMin: %.0f\t%.0f\n", total_cost[min_solver], total_nodes[min_solver]);
	//printf("Max: %.0f\t%.0f\n", total_cost[max_solver], total_nodes[max_solver]);
	//printf("Av: %.2f\t%.2f\n", all_cost/ (double) num_solvers, all_nodes / (double) num_solvers);
	fclose(output_nodes);
	fclose(output_costs);
	return 0;
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

double batch_puzzles(unsigned num_cols, unsigned num_rows, GeneralIDA<MNPuzzleState, slideDir> *ida, std::vector<MNPuzzleState> &puzzles, std::vector<Puzzle_Info> &info, std::vector<slideDir> &op_order, bool print_all_stats, bool print_iters) {
	double total_checked = 0;
	double total_generated = 0;
	double total_expanded = 0;

	double total_av_checked = 0.0;
	double total_cost = 0.0;
	double total_av_cost = 0.0;

	unsigned total_iters = 0;

	MNPuzzleState goal(num_cols, num_rows);
	std::vector<slideDir> path;

	MNPuzzle mnp(num_cols, num_rows, op_order);
	mnp.StoreGoal(goal);

	for(unsigned i = 0; i < puzzles.size(); i++) {

		if(puzzles[i].width != goal.width || puzzles[i].height != goal.height) {
			std::cerr << puzzles[i] << '\n';
			std::cerr << goal << '\n';
			std::cerr << "Invalid Puzzle\n";
			return -1.0;
		}
		ida->GetPath(&mnp, puzzles[i], goal, path);

		total_cost += ida->GetPathCost();
		total_checked += ida->GetNodesChecked();
		total_generated += ida->GetNodesTouched();
		total_expanded += ida->GetNodesExpanded();
		total_iters += ida->GetNumIters();

		if(info.size() == puzzles.size()) {
			total_av_checked += info[i].first/ ida->GetNodesChecked();
			total_av_cost += ida->GetPathCost() / info[i].second;
		}

		if(print_all_stats) {
			printf("%llu\t%.0f\t%u", ida->GetNodesChecked(), ida->GetPathCost(), ida->GetNumIters());

			if(info.size() == puzzles.size()) {
				printf("\t%.2f\t%.2f", info[i].first/ ida->GetNodesChecked(), ida->GetPathCost() / info[i].second);
			}
			if (print_iters) {
				std::vector<unsigned long long> iter = ida->Get_Checked_Iters();
				for (unsigned j = 0; j < iter.size(); j++) {
					printf("\t%llu", iter[j]);
				}
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
	std::vector<unsigned long long> its;

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
				current_best_generated = solvers[j]->GetNodesTouched();
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
				printf("\t%lld", its[j]);
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
			temp_nodes = batch_puzzles(4, 4, &ida, puzzles, info, op_order, false, false);
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
		solvers[i]->SetReverseOrder(false);
		solvers[i]->Initialize(mnp, start, goal);
	}

	int status = 0;
	unsigned solver_counter = num_solvers - 1;
	std::vector<slideDir> path;
	while(status == 0) {
		solver_counter = (solver_counter + 1) % num_solvers;
		status = solvers[solver_counter]->StepAlgorithm(path);
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

			total_generated += solvers[j]->GetNodesTouched();
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
			std::vector<unsigned long long> iter = solvers[solved_by]->Get_Checked_Iters();
			for(unsigned j = 0; j < iter.size(); j++) {
				printf("\t%lld", iter[j]);
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

void random_ind_puzzles(std::vector<GeneralIDA<MNPuzzleState, slideDir> *> solvers, std::vector<double> &solver_info, std::vector<MNPuzzleState> &puzzles, std::vector<Puzzle_Info> &info, std::vector<slideDir> &op_order, unsigned size, unsigned num) {
	std::vector<GeneralIDA<MNPuzzleState, slideDir> *> rand_solvers;

	std::vector<unsigned> solver_index;
	std::vector<unsigned> combo;

	unsigned count = 0;

	double best_nodes = 0.0;
	double max = 0.0;
	double max_ratio = 0.0;
	double min = 0.0;
	double min_ratio = 0.0;

	double av_weight;
	double av_nodes = 0.0;
	double av_ratio = 0.0;

	std::vector<std::pair<double, double> > run_info;
	std::map<std::string, std::string> combo_map;

	while(count < num) {

		solver_index.clear();
		rand_solvers.clear();
		combo.clear();
		for(unsigned i = 0; i < solvers.size(); i++) {
			solver_index.push_back(i);
		}

		best_nodes = 0.0;
		av_weight = 0.0;
		for(unsigned i = 0; i < size; i++) {
			int r = rand();
			int index = r % solver_index.size();
			combo.push_back(solver_index[index]);

			solver_index[index] = solver_index.back();
			solver_index.pop_back();
		}

		std::sort(combo.begin(), combo.end());
		std::string combo_key = "";
		for(unsigned i = 0; i < size; i++) {
			combo_key.append(int_to_string(combo[i]));
			combo_key.push_back('_');
		}
		// if have already seen this combo
		if(combo_map.find(combo_key) != combo_map.end()) {
			continue;
		}

		combo_map[combo_key] = combo_key;

		for(unsigned i = 0; i < size; i++) {
			printf("%.0f ", solvers[combo[i]]->Get_H_Weight());
			rand_solvers.push_back(solvers[combo[i]]);
			av_weight += solvers[combo[i]]->Get_H_Weight();

			if(best_nodes == 0.0 || solver_info[combo[i]] < best_nodes)
				best_nodes = solver_info[combo[i]];
		}

		av_weight /= (double) size;
		printf("\t");

		double temp_nodes = ind_batch_puzzles(rand_solvers, puzzles, info, op_order, false);
		av_nodes += temp_nodes;

		double best_ratio = temp_nodes / best_nodes;
		av_ratio += best_ratio;

		std::pair<double, double> new_info(av_weight, best_ratio);
		run_info.push_back(new_info);

		if(count == 0) {
			max = temp_nodes;
			min = temp_nodes;

			min_ratio = best_ratio;
			max_ratio = best_ratio;
		}
		else if(min > temp_nodes) {
			min = temp_nodes;
		}
		else if(max < temp_nodes) {
			max = temp_nodes;
		}

		if(fgreater(min_ratio, best_ratio)) {
			min_ratio = best_ratio;
		}
		else if(fless(max_ratio, best_ratio)) {
			max_ratio = best_ratio;
		}

		count++;
	}

	av_nodes /= (double) num;
	av_ratio /= (double) num;
	printf("\nMIN:\t%.0f\t%.4f\n", min, min_ratio);
	printf("MAX:\t%.0f\t%.4f\n", max, max_ratio);
	printf("Averages:\t%.2f\t%.4f\n", av_nodes, av_ratio);

	for(unsigned i = 0; i < run_info.size(); i++) {
		printf("%.2f\t%.4f\n", run_info[i].first, run_info[i].second);
	}
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

int read_in_approx_info_file(const char *filename, double **container, unsigned num_weights, unsigned puzzle_num){
	std::ifstream ifs(filename);

	if(ifs.fail()) {
		assert(false);
		return 1;
	}

	std::string s, temp;
	std::vector<std::string> tokens;
	unsigned count = 0;
	while(!ifs.eof() && count < puzzle_num) {
		getline(ifs, s);

		tokens.clear();
		tokens = split(s, '\t');

		if(tokens.size() < num_weights)
			continue;

		for(unsigned i = 0; i < tokens.size(); i++) {
			container[i][count] = atof(tokens[i].c_str());
		}
		count++;
	}
	assert(count == puzzle_num);
	ifs.close();

	return 0;
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

unsigned ind_approx (const char *nodes_file, const char *cost_file, unsigned num_weights, unsigned puzzle_num, unsigned start_w_index, unsigned end_w_index, unsigned start_size, unsigned end_size, unsigned num, bool print_all_stats){

	double **nodes;
	double **costs;

	double all_bests[end_size - start_size + 1][3];
	double all_worsts[end_size - start_size + 1][3];
	double all_ratios[end_size - start_size + 1][2];
	double all_avs[end_size - start_size + 1][4];

	nodes = new double *[num_weights];
	costs = new double *[num_weights];
	for(unsigned i = 0; i < num_weights; i++) {
		nodes[i] = new double[puzzle_num];
		costs[i] = new double[puzzle_num];
	}

	double total_nodes[num_weights];
	unsigned count = 0;

	// get info
	if(read_in_approx_info_file(cost_file, costs, num_weights, puzzle_num) || read_in_approx_info_file(nodes_file, nodes, num_weights, puzzle_num)) {
		delete nodes;
		delete costs;
		return 1;
	}

	for(unsigned i = 0; i < num_weights; i++) {
		total_nodes[i] = 0.0;
	}

	// calculate total nodes
	for(unsigned i = 0; i < puzzle_num; i++) {
		for(unsigned j = 0; j < num_weights; j++) {
			total_nodes[j] += nodes[j][i];
		}
	}

	std::vector<unsigned> combo;
	std::map<std::string, std::string> combo_map;

	double best_nodes, current_nodes, current_cost = 0;
	double total_combo_nodes, total_combo_cost, problem_nodes;
	double av_weight, best_ratio;
	double node_size_av = 0, cost_size_av = 0, weight_size_av = 0, ratio_size_av = 0;
	double node_size_best = 0, cost_size_best = 0, weight_size_best = 0, ratio_size_best = 0;
	double node_size_worst = 0, cost_size_worst = 0, weight_size_worst = 0, ratio_size_worst = 0;

	unsigned best_index = 0;

	unsigned size_count = 0;
	// do each desired size
	for(unsigned size = start_size; size <= end_size; size++) {
		count = 0; // number of problems
		node_size_av = 0.0;
		cost_size_av = 0.0;
		weight_size_av = 0.0;
		ratio_size_av = 0.0;

		// for total per size
		while(count < num) {

			get_rand_combo(combo, start_w_index, end_w_index, size);

			std::sort(combo.begin(), combo.end());
			std::string combo_key = "";

			// get combo hash value
			for(unsigned i = 0; i < size; i++) {
				combo_key.append(int_to_string(combo[i]));
				combo_key.push_back('_');
			}
			// if have already seen this combo
			if(combo_map.find(combo_key) != combo_map.end()) {
				continue;
			}

			combo_map[combo_key] = combo_key;

			best_nodes = 0.0;
			av_weight = 0.0;

			for(unsigned i = 0; i < size; i++) {
				if(print_all_stats)
					printf("%d\t", combo[i] + 2);
				av_weight += combo[i] + 2;
			}

			// find best_nodes and print combo
			for(unsigned i = 0; i < size; i++) {
				if(best_nodes == 0.0 || total_nodes[combo[i]] < best_nodes)
					best_nodes = total_nodes[combo[i]];
			}
			av_weight /= (double) size;

			total_combo_nodes = 0.0;
			total_combo_cost = 0.0;

			// iterate through all problems
			for(unsigned i = 0; i < puzzle_num; i++) {
				current_nodes = 0.0;
				best_index = best_of_combo(combo, nodes, i);
				current_nodes = nodes[combo[best_index]][i];
				current_cost = costs[combo[best_index]][i];

				assert(current_nodes > 0);
				// gets number of nodes expanded
				problem_nodes = (current_nodes - 1.0)*size + best_index + 1.0;
				total_combo_nodes += problem_nodes; // increment problem
				total_combo_cost += current_cost;
			}

			best_ratio = total_combo_nodes / best_nodes;

			if(print_all_stats)
				printf("\t\t%.0f\t%.0f\t%.2f\t%.4f\n", total_combo_cost, total_combo_nodes, av_weight, best_ratio);

			node_size_av += total_combo_nodes;
			cost_size_av += total_combo_cost;
			weight_size_av += av_weight;
			ratio_size_av += best_ratio;

			// if is first combo
			if(count == 0) {
				node_size_best = total_combo_nodes;
				cost_size_best = total_combo_cost;
				weight_size_best = av_weight;
				ratio_size_best = best_ratio;

				node_size_worst = total_combo_nodes;
				cost_size_worst = total_combo_cost;
				weight_size_worst = av_weight;
				ratio_size_worst = best_ratio;
			}

			if(node_size_best > total_combo_nodes) {
				node_size_best = total_combo_nodes;
				cost_size_best = total_combo_cost;
				weight_size_best = av_weight;
			}
			if(node_size_worst < total_combo_nodes) {
				node_size_worst = total_combo_nodes;
				cost_size_worst = total_combo_cost;
				weight_size_worst = av_weight;
			}
			if(ratio_size_best > best_ratio)
				ratio_size_best = best_ratio;
			if(ratio_size_worst < best_ratio)
				ratio_size_worst = best_ratio;

			count++;
		}

		cost_size_av /= num;
		node_size_av /= num;
		weight_size_av /= num;
		ratio_size_av /= num;

		all_bests[size_count][0] = cost_size_best; all_bests[size_count][1] = node_size_best; all_bests[size_count][2]= weight_size_best;
		all_worsts[size_count][0] = cost_size_worst; all_worsts[size_count][1] = node_size_worst; all_worsts[size_count][2] = weight_size_worst;
		all_ratios[size_count][0] = ratio_size_best; all_ratios[size_count][1] = ratio_size_worst;
		all_avs[size_count][0] = cost_size_av; all_avs[size_count][1] = node_size_av; all_avs[size_count][2] = weight_size_av; all_avs[size_count][3] = ratio_size_av;
		/*
		printf("Min: %.0f\t%.0f\t%.2f\n",cost_size_best, node_size_best, weight_size_best);
		printf("Max: %.0f\t%.0f\t%.2f\n",cost_size_worst, node_size_worst, weight_size_worst);
		printf("Ratio: %.4f\t%.4f\n", ratio_size_best, ratio_size_worst);
		printf("Av: %.0f\t%.0f\t%.2f\t%.4f\n", cost_size_av, node_size_av, weight_size_av, ratio_size_av);
		printf("\n");*/

		size_count++;
	}


	for(unsigned i = 0; i < size_count; i++) {
		printf("%.0f\t%.0f\t%.2f\n", all_bests[i][0], all_bests[i][1], all_bests[i][2]);
	}
	printf("\n");
	for(unsigned i = 0; i < size_count; i++) {
		printf("%.0f\t%.0f\t%.2f\n", all_worsts[i][0], all_worsts[i][1], all_worsts[i][2]);
	}
	printf("\n");

	for(unsigned i = 0; i < size_count; i++) {
		printf("%.0f\t%.0f\t%.2f\t%.4f\n", all_avs[i][0], all_avs[i][1], all_avs[i][2], all_avs[i][3]);
	}
	printf("\n");

	for(unsigned i = 0; i < size_count; i++) {
		printf("%.4f\t%.4f\n", all_ratios[i][0], all_ratios[i][1]);
	}
	printf("\n");

	delete nodes;
	delete costs;
	return 0;
}

/*
typedef pair<double, double> W_K_pair;

void calculate_earned() {
	double num_w = 15.0;
	double num_k = 25.0;

	vector<vector <W_K_pair> > groups;
	vector<double> group_values;

	unsigned i = 0;
	for(double w = 2.0; w <= num_w; w++) {
		for(double k = 2.0; k <= num_k; k++) {
			W_K_pair new_pair(w, k);

			double numerator = w*k - k - w + 1.0;
			double denominator = w*k + k + w - 1.0;

			double earned = numerator/denominator;

			assert(group_values.size() == groups.size());

			for(i = 0; i < group_values.size(); i++) {
				if(fequal(group_values[i], earned)) {
					break;
				}
			}
			if(i < group_values.size()) {
				groups[i].push_back(new_pair);
			}
			else {
				vector<W_K_pair> new_group;
				new_group.push_back(new_pair);
				groups.push_back(new_group);
				group_values.push_back(earned);
			}
			printf("%.5f\t", earned);
		}
		W_K_pair new_pair(w, 0.0);
		double earned = (w - 1.0)/(w + 1.0);

		for(i = 0; i < group_values.size(); i++) {
			if(fequal(group_values[i], earned)) {
				break;
			}
		}
		if(i < group_values.size()) {
			groups[i].push_back(new_pair);
		}
		else {
			vector<W_K_pair> new_group;
			new_group.push_back(new_pair);
			groups.push_back(new_group);
			group_values.push_back(earned);
		}

		printf("\t%.5f\n", earned);
	}

	cout << endl << endl << endl;
	for(i = 0; i < groups.size(); i++) {

		printf("%.5f - %d\t\t", group_values[i], groups[i].size());
		for(unsigned j = 0; j < groups[i].size(); j++) {
			printf("%.0f,%.0f - ", groups[i][j].first, groups[i][j].second);
		}
		cout << endl;
	}
}*/

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
