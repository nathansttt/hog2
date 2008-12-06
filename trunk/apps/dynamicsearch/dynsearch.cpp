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

int main(int argc, char** argv)
{
	//calculate_earned();
	std::vector<Puzzle_Info> info;
	vector<MNPuzzleState> puzzles;
	std::vector<double> solver_info;

	//get_big_4x4_test_set(puzzles, info, solver_info, 1000);
	get_5x4_test_set(puzzles, 100);
	unsigned num_cols = 5;
	unsigned num_rows = 4;

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

	GeneralIDA<MNPuzzleState, slideDir> ida;

	vector<slideDir> path;
	//ida.SetCheckedLimit(10);
	//ida.GetPath(&mnp, puzzles[0], goal, path);
	//cout << ida.GetNodesChecked() << endl;
	for(unsigned i = 3; i <= 10; i++) {
		cout << "Weight: " << i << endl;
		ida.Change_Weights(1.0, i);
		batch_puzzles(num_cols, num_rows, &ida, puzzles, info, f_op_order, true, false);
	}
	/*
	puzzles.clear();
	MNPuzzle::Create_Random_MN_Puzzles(5, 4, puzzles, 1000);
	MNPuzzle::Output_Puzzles(puzzles, 5, 4, false);

	MNPuzzleState goal(4, 4);
	MNPuzzle b_mnp(4, 4, b_op_order);
	b_mnp.StoreGoal(goal);
	*/
	//std::vector<slideDir> path;

	//unsigned solved_by = independent_solvers_run(solvers, &b_mnp, puzzles[0], goal);
	//cout << "Nodes Checked: " << solvers[solved_by]->GetNodesChecked() << endl;

	//ind_batch_puzzles(solvers, puzzles, info, b_op_order, true);

	//rbfs_ind_approx(0,23,24,24,1);
	/*
	for(unsigned i = 2; i < 24; i++) {
		//batch_puzzles(solvers[i], puzzles, info, f_op_order, false);
		random_ind_puzzles(solvers, solver_info, puzzles, info, b_op_order, i, 30);
	}*/
	//random_ind_puzzles(solvers, solver_info, puzzles, info, b_op_order, 14, 1);

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
