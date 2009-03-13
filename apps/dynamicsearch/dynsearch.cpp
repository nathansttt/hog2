#include <vector>
#include <stdio.h>
#include <sstream>
#include "Common.h"
#include "dynsearch.h"
#include "MNPuzzle.h"
#include "GeneralIDA.h"
#include "GeneralRBFS.h"
#include "GeneralBeamSearch.h"
#include "GeneralBulb.h"
#include "experiment_basics.h"
#include "TestSearch.h"

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
	std::vector<Puzzle_Info> info;
	vector<MNPuzzleState> puzzles;
	std::vector<double> solver_info;

	//get_standard_test_set(puzzles, info, solver_info, 100);
	//get_4x5_test_set(puzzles, 100);
	get_5x5_test_set(puzzles, 100);

	unsigned num_cols = 5;
	unsigned num_rows = 5;

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

	GeneralIDA<MNPuzzleState, slideDir, MNPuzzle> ida;

	vector<slideDir> path;
	vector<MNPuzzleState> state_path;
	MNPuzzleState goal(num_cols, num_rows);
	MNPuzzle mnp(num_cols, num_rows, f_op_order);
	mnp.StoreGoal(goal);

	//ida.SetCheckedLimit(10);
	//ida.GetPath(&mnp, puzzles[0], goal, path);
	//cout << ida.GetNodesChecked() << endl;

	for(double i = 2.0; i <= 25.0; i+= 1.0) {
		cout << "\nSOLVER IDA*, OP ORDER: " << kUp << ", " << kLeft << ", " << kRight << ", " << kDown << ", Weight: " << i << endl;
		//cerr << "\nWeight: " << i << endl;
		ida.Change_Weights(1.0, i);
		general_batch_puzzles(num_cols, num_rows, &ida, puzzles, f_op_order, true, ACTION_PATH);
	}

	//output_exp_info("../../apps/dynamicsearch/input/output_tester", 100, "c_f", "ex_f", "ch_f", "ch_t");

	//ida.Change_Weights(1.0, 3.0);

	unsigned num = 0;
	GeneralRBFS<MNPuzzleState, slideDir, MNPuzzle> rbfs;
	//rbfs.Change_Weights(1.0, 4.0);
	//general_batch_puzzles(4, 4, &rbfs, puzzles, info, f_op_order, true, ACTION_PATH);
	//rbfs.SetExpandedLimit(5000);

	/*
	for(; num < 10; num++) {
	rbfs.GetPath(&mnp, puzzles[num], goal, path);

	cout << rbfs.GetPathCost() << "\t" << rbfs.GetNodesChecked() << endl;
	}*/

	num = 0;
	GeneralBeamSearch<MNPuzzleState, slideDir, MNPuzzle> bs;
	bs.Change_Beam_Size(15);
	bs.Change_Memory_Limit(10000);
	bs.Select_Duplicate_Prune(true);
	//general_batch_puzzles(4, 4, &bs, puzzles, info, f_op_order, true, STATE_PATH);
	//bs.GetPath(&mnp, puzzles[num], goal, state_path);

	//cout << state_path.size() << endl;
	//bs.Initialize(&mnp, puzzles[num], goal);

	/*
	bs.print_beams();
	bs.StepAlgorithm(state_path);
	bs.print_beams();
	bs.StepAlgorithm(state_path);
	bs.print_beams();
	bs.StepAlgorithm(state_path);
	bs.print_beams();
	bs.StepAlgorithm(state_path);
	bs.print_beams();
	bs.StepAlgorithm(state_path);
	bs.print_beams();
	bs.StepAlgorithm(state_path);
	bs.print_beams();
	bs.StepAlgorithm(state_path);
	bs.print_beams();
	bs.StepAlgorithm(state_path);
	bs.print_beams();
	bs.StepAlgorithm(state_path);
	bs.print_beams();*/
	/*
	int status = 0;
	while(status == 0) {
		status = bs.StepAlgorithm(state_path);
	}
	cout << state_path.size() << endl;
	for(unsigned l = 0; l < state_path.size() ; l++) {
		cout << state_path[l] << endl;
	}*/

	GeneralBulb<MNPuzzleState, slideDir, MNPuzzle> bulb;
	for(unsigned j = 100000; j <= 100000; j *= 100) {
		bulb.Change_Memory_Limit(j);
		//printf("Limit: %d\n", j);
		/*
	cout << "\nBeam Size: " << 2 << endl;
	cerr << "\nBeam Size: " << 2 << endl;
	bulb.Change_Beam_Size(2);
		//general_batch_puzzles(num_cols, num_rows, &bulb, puzzles, info, f_op_order, true, STATE_PATH);

	cout << "\nBeam Size: " << 3 << endl;
	cerr << "\nBeam Size: " << 3 << endl;
	bulb.Change_Beam_Size(3);
		//general_batch_puzzles(num_cols, num_rows, &bulb, puzzles, info, f_op_order, true, STATE_PATH);

	cout << "\nBeam Size: " << 4 << endl;
	cerr << "\nBeam Size: " << 4 << endl;
	bulb.Change_Beam_Size(4);
		//general_batch_puzzles(num_cols, num_rows, &bulb, puzzles, info, f_op_order, true, STATE_PATH);


	for(unsigned i = 5; i <= 100; i+= 5) {
		cout << "\nBeam Size: " << i << endl;
		cerr << "\nBeam Size: " << i << endl;
		bulb.Change_Beam_Size(i);
		general_batch_puzzles(num_cols, num_rows, &bulb, puzzles, info, f_op_order, true, STATE_PATH);
	}*/
	}
	//general_batch_puzzles(4, 4, &bulb, puzzles, info, f_op_order, true, STATE_PATH);
	//bulb.SetExpandedLimit(30);
	//bulb.GetPath(&mnp, puzzles[num], goal, state_path);

	//cout << state_path.size() << endl;
/*
	for(unsigned l = 0; l < state_path.size() ; l++) {
		cout << state_path[l] << endl;
	}*/

	//ida.Initialize(&mnp, puzzles[0], goal);
	//cout << puzzles[0] << endl;
	//ida.GetPath(&mnp, puzzles[0], goal, path);'
	/*
	while(!ida.StepAlgorithm(path)) {

	}*/

	//cout << ida.GetPathCost() << " " << ida.GetNodesChecked() << endl;
	string nodes_filename = "../../apps/dynamicsearch/input/4x4_ida_mnp_100__smaller_nodes";
	string cost_filename = "../../apps/dynamicsearch/input/4x4_ida_mnp_100__smaller_costs";

	unsigned num_weights = 24;
	unsigned stored_weights = 25;
	vector<unsigned> desired_puzzles;
	for(unsigned i = 75; i < 100; i++) {
		desired_puzzles.push_back(i);
	}

	vector<unsigned> desired_weights;
	for(unsigned i = 0; i < num_weights; i++) {
		desired_weights.push_back(i);
	}

	vector<unsigned> set_sizes;
	for(unsigned i = 2; i <= num_weights; i++) {
		set_sizes.push_back(i);
	}

	vector<unsigned> num_per_size;
	num_per_size.push_back(276);
	num_per_size.push_back(1000);
	num_per_size.push_back(1000);
	num_per_size.push_back(1000);
	num_per_size.push_back(1000);
	num_per_size.push_back(1000);
	num_per_size.push_back(1000);
	num_per_size.push_back(1000);
	num_per_size.push_back(1000);
	num_per_size.push_back(1000);
	num_per_size.push_back(1000);
	num_per_size.push_back(1000);
	num_per_size.push_back(1000);
	num_per_size.push_back(1000);
	num_per_size.push_back(1000);
	num_per_size.push_back(1000);
	num_per_size.push_back(1000);
	num_per_size.push_back(1000);
	num_per_size.push_back(1000);
	num_per_size.push_back(1000);
	num_per_size.push_back(276);
	num_per_size.push_back(24);
	num_per_size.push_back(1);

	//tt_simulation(nodes_filename.c_str(), cost_filename.c_str(), stored_weights, 100, desired_puzzles, desired_weights, set_sizes, num_per_size, false);
	/*
	ind_approx(nodes_filename.c_str(), cost_filename.c_str(), 24, 100, 0, 23, 2, 2, 276, false);
	cout << "\n3-22\n";
	ind_approx(nodes_filename.c_str(), cost_filename.c_str(), 24, 100, 0, 23, 3, 22, 250, false);
	cout << "\n23\n";
	ind_approx(nodes_filename.c_str(), cost_filename.c_str(), 24, 100, 0, 23, 23, 23, 24, false);
	cout << "\n24\n";
	ind_approx(nodes_filename.c_str(), cost_filename.c_str(), 24, 100, 0, 23, 24, 24, 1, false);*/

	//output_exp_info("../../apps/dynamicsearch/input/std_ida_tests", 100, 24);

	//get_distribution("../../apps/dynamicsearch/input/temp_stuff", 1000, 0);
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

	vector<int> pattern;
	pattern.push_back(0);
	pattern.push_back(6);
	pattern.push_back(7);
	//pattern.push_back(10);
	//pattern.push_back(11);
	//pattern.push_back(14);
	//pattern.push_back(15);

	//mnp.Build_Regular_PDB(goal, pattern, "tempdb");
	return 0;
}
