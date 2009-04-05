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
#include "TurnTakingSimulation.h"
#include "PancakePuzzle.h"

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
	get_3x6_test_set(puzzles, 10);
	//get_5x5_test_set(puzzles, 100);

	unsigned num_cols = 3;
	unsigned num_rows = 6;

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

	for(unsigned i = 0; i < puzzles.size(); i++) {
		if(MNPuzzle::GetParity(puzzles[i]) != MNPuzzle::GetParity(goal)) {
			printf("Bad Puzzle %d\n", i);
			assert(false);
		}
	}

	//ida.SetCheckedLimit(10);
	//ida.GetPath(&mnp, puzzles[0], goal, path);
	//cout << ida.GetNodesChecked() << endl;

	for(double i = 3.0; i <= 3.0; i+= 0.05) {
		//cout << "\nSOLVER IDA*, OP ORDER: " << kUp << ", " << kLeft << ", " << kRight << ", " << kDown << ", Weight: " << i << endl;
		//cerr << "\nWeight: " << i << endl;
		ida.Change_Weights(1.0, i);
		//general_batch_puzzles(num_cols, num_rows, &ida, puzzles, f_op_order, true, ACTION_PATH);
		//general_batch_puzzles(num_cols, num_rows, &ida, puzzles, get_puzzle_order(4), true, ACTION_PATH);
		//mnp.Change_Op_Order(get_puzzle_order(4));
	}

	/*
	GeneralRBFS<MNPuzzleState, slideDir, MNPuzzle> rbfs;
	for(double i = 3.0; i <= 3.0; i+= 1.0) {
		cout << "\nSOLVER RBFS, OP ORDER: " << kUp << ", " << kLeft << ", " << kRight << ", " << kDown << ", Weight: " << i << endl;
		rbfs.Change_Weights(1.0, i);
		general_batch_puzzles(num_cols, num_rows, &rbfs, puzzles, f_op_order, true, ACTION_PATH);
	}*/

	//TurnTakingSimulation::output_turntaking_input_file("../../apps/dynamicsearch/input/4x5_ida_1000_weight_stats", 1000, "tt");

	vector<unsigned> desired_puzzles;
	for(unsigned i = 0; i < 100; i++) {
		desired_puzzles.push_back(i);
	}

	unsigned start = 0;
	unsigned inc = 1;
	unsigned total_solvers = 24;
	vector<unsigned> desired_weights;
	for(unsigned i = start; i < start + inc*(total_solvers - 1) + 1; i+= inc) {
		desired_weights.push_back(i);
	}

	vector<unsigned> set_sizes;
	for(unsigned i = 2; i <= 24; i++) {
		set_sizes.push_back(i);
	}

	vector<unsigned> num_per_size;
	num_per_size.push_back(0);
	num_per_size.push_back(0);

	for(unsigned i = 4; i <= 20; i++) {
		num_per_size.push_back(5000);
	}
	num_per_size.push_back(0);
	num_per_size.push_back(0);
	num_per_size.push_back(0);
	num_per_size.push_back(0);

	//TurnTakingSimulation simulator("../../apps/dynamicsearch/input/4x5_ida_1000_weight_tt");
	//simulator.output_solver_names(desired_weights);
	//simulator.simulate(desired_puzzles, desired_weights, set_sizes, num_per_size, false);

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

/*
	vector<int> pattern;	vector<slideDir> ops;
	for(unsigned i = 0; i < 24; i++) {
		ops = MNPuzzle::Get_Puzzle_Order(i);
		cout << i;
		for(unsigned j = 0; j < ops.size(); j++) {
			cout << " " << ops[j];
		}
		cout << endl;
	}
	pattern.push_back(0);
	pattern.push_back(6);
	pattern.push_back(7);*/
	//pattern.push_back(10);
	//pattern.push_back(11);
	//pattern.push_back(14);
	//pattern.push_back(15);

	//mnp.Build_Regular_PDB(goal, pattern, "tempdb");

	unsigned pan_size = 12;
	PancakePuzzle pancake_puzz(pan_size);
	PancakePuzzleState s(pan_size);

	vector<int> distinct;
	distinct.push_back(7);
	distinct.push_back(8);
	distinct.push_back(9);
	distinct.push_back(10);
	distinct.push_back(11);
	distinct.push_back(12);
	distinct.push_back(13);

	//pancake_puzz.Build_Regular_PDB(s, distinct, "../../apps/dynamicsearch/input/14panc_pdb_7_8_9_10_11_12_13_distinct");

	//puzzles.clear();
	//MNPuzzle::Create_Random_MN_Puzzles(goal, puzzles, 13);
	//mnp.Output_Puzzles(puzzles, false);

	vector<PancakePuzzleState> pancake_puzzles;
	get_12pancake_test_set(pancake_puzzles, 2);

	GeneralIDA<PancakePuzzleState, unsigned, PancakePuzzle> pan_ida;
	PancakePuzzleState pan_goal(pan_size);

	pancake_puzz.Load_Regular_PDB("../../apps/dynamicsearch/input/12panc_pdb_0_1_2_3_4_5_6_distinct", pan_goal, false);
	//pancake_puzz.Load_Regular_PDB("../../apps/dynamicsearch/input/14panc_pdb_7_8_9_10_11_12_13_distinct", pan_goal, false);


	for(double w = 1.0; w <= 25.0; w++) {
		pan_ida.Change_Weights(1.0, w);
		cout << "Weight: " << w << endl;
		general_batch_pancake_puzzles(pancake_puzz, pan_size, &pan_ida, pancake_puzzles, true, ACTION_PATH);

	}

	//cout << pan_ida.GetPathCost() << endl;
	//cout << pan_ida.GetNodesExpanded() << endl;
	return 0;
}
