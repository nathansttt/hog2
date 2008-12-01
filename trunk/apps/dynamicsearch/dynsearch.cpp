#include <vector>
#include <stdio.h>
#include <sstream>
#include "Common.h"
#include "dynsearch.h"
#include "MNPuzzle.h"
#include "GeneralIDA.h"
#include "experiment_basics.h"

using namespace std;

void handle_warnings() {
	MNPuzzleState start(4, 4);
	if(start == start) {
		cout << kUp;
		cout << start;
	}
}

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
}

void get_points() {
	std::ifstream ifs("temp3");

	if(ifs.fail()) {
		cout << "FAIL" << endl;
	}

	std::string s, temp;
	std::vector<std::string> tokens;

	while(!ifs.eof()) {
		getline(ifs, s);;

		tokens.resize(0);

		tokens = split(s, ' ');

		if(tokens.size() == 3)
			cout << s << endl;
	}
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
}
int main(int argc, char** argv)
{
	//calculate_earned();
	std::vector<Puzzle_Info> info;
	read_in_extra_puzz_info("../../apps/dynamicsearch/input/std_test_set_info", info, ' ', 100);

	vector<MNPuzzleState> puzzles;

	if(MNPuzzle::read_in_mn_puzzles("../../apps/dynamicsearch/input/standard_test_set", false, 4, 4, 100, puzzles)) {
		printf("FAILED\n");
	}

	std::vector<slideDir> f_op_order;
	std::vector<slideDir> b_op_order;

	f_op_order.push_back(kUp);
	f_op_order.push_back(kLeft);
	f_op_order.push_back(kRight);
	f_op_order.push_back(kDown);

	b_op_order.push_back(kDown);
	b_op_order.push_back(kRight);
	b_op_order.push_back(kLeft);
	b_op_order.push_back(kUp);

	vector<GeneralIDA<MNPuzzleState, slideDir> *> solvers;

	for(unsigned i = 3; i < 5; i++) {
		solvers.push_back(new GeneralIDA<MNPuzzleState, slideDir>(1.0, i, false, false, false, false));
	}

	/*
	MNPuzzleState goal(4, 4);
	MNPuzzle b_mnp(4, 4, b_op_order);
	b_mnp.StoreGoal(goal);
	*/
	//std::vector<slideDir> path;

	//unsigned solved_by = independent_solvers_run(solvers, &b_mnp, puzzles[0], goal);
	//cout << "Nodes Checked: " << solvers[solved_by]->GetNodesChecked() << endl;

	ind_batch_puzzles(solvers, puzzles, info, b_op_order, true);

	//oracle_experiment(solvers, puzzles, info, f_op_order, true);
	/*
	Timer t, t2;
	t.startTimer();
	ida.GetPath(&mnp, puzzles[0], goal, path);
	t.endTimer();
	cout << "Path found, length: " << path.size() << endl;
	cout << "Nodes Checked: " << ida.GetNodesChecked() << endl;
	cout << "Nodes Generated: " << ida.GetNodesGenerated() << endl;
	cout << "Nodes Expanded: " << ida.GetNodesExpanded() << endl;
	cout << "Time:" << t.getElapsedTime() << endl;
	path.resize(0);

	t2.startTimer();
	ida.initialize_step_by_step(&mnp, puzzles[0], goal, false);
	while(ida.move_one_step(goal, path) != 1) {}
	t2.endTimer();
	cout << "Path found, length: " << path.size() << endl;
	cout << "Nodes Checked: " << ida.GetNodesChecked() << endl;
	cout << "Nodes Generated: " << ida.GetNodesGenerated() << endl;
	cout << "Nodes Expanded: " << ida.GetNodesExpanded() << endl;
	cout << "Time:" << t2.getElapsedTime() << endl;*/
	return 0;
}
