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
	//get_3x6_test_set(puzzles, 10);
	//get_6x6_test_set(puzzles, 100);

	unsigned num_cols = 6;
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

	/**
	IDA* Puzzle Experiments
	**/
	/*
	for(double i = 3.0; i <= 3.0; i+= 0.05) {
		cout << "\nSOLVER IDA*, OP ORDER: " << kUp << ", " << kLeft << ", " << kRight << ", " << kDown << ", Weight: " << i << endl;
		ida.Change_Weights(1.0, i);
		general_batch_puzzles(num_cols, num_rows, &ida, puzzles, f_op_order, true, ACTION_PATH);
	}*/

	/**
	IDA* Puzzle Order Experiments
	**/
	/*
	double weight = 3.0;
	std::vector<slideDir> puzz_op_order;
	ida.Change_Weights(1.0, weight);
	for(unsigned i = 0; i < 24; i++) {
		puzz_op_order = MNPuzzle::Get_Puzzle_Order(i);
		cout << "\nSOLVER IDA*, OP ORDER: ";

		for(unsigned j = 0; j < 4; j++) {
			cout << puzz_op_order[j] << ", ";
		}
		cout << "Weight: " << weight << endl;
		general_batch_puzzles(num_cols, num_rows, &ida, puzzles, f_op_order, true, ACTION_PATH);
	}
	*/

	/**
	RBFS Puzzle Experiments
	**/
	/*
	GeneralRBFS<MNPuzzleState, slideDir, MNPuzzle> rbfs;
	for(double i = 3.0; i <= 3.0; i+= 1.0) {
		cout << "\nSOLVER RBFS, OP ORDER: " << kUp << ", " << kLeft << ", " << kRight << ", " << kDown << ", Weight: " << i << endl;
		rbfs.Change_Weights(1.0, i);
		general_batch_puzzles(num_cols, num_rows, &rbfs, puzzles, f_op_order, true, ACTION_PATH);
	}
	*/

	/**
	BEAM / BULB Puzzle Experiments
	**/
	/*
	GeneralBeamSearch<MNPuzzleState, slideDir, MNPuzzle> beam_search;
	beam_search.Change_Beam_Size(100);
	beam_search.Change_Memory_Limit(100000);
	beam_search.debug = false;
	beam_search.Select_Duplicate_Prune(true);
	beam_search.Select_Full_Check(true);

	GeneralBulb<MNPuzzleState, slideDir, MNPuzzle> bulb;
	bulb.Select_Full_Check(true);
	bulb.Change_Beam_Size(100);
	bulb.Change_Memory_Limit(1000000);
	bulb.debug = false;
	bulb.Select_Duplicate_Prune(true);
	bulb.Select_Full_Check(true);
	bulb.Set_Max_Discrepancies(0);


	for(unsigned i = 3; i <= 3; i++) {
		cout << "\nSOLVER BULB, OP ORDER: " << kUp << ", " << kLeft << ", " << kRight << ", " << kDown << ", Beam Size: " << i << endl;
		bulb.Change_Beam_Size(i);
		general_batch_puzzles(num_cols, num_rows, &bulb, puzzles, f_op_order, true, STATE_PATH);
	}*/
	/*
	unsigned my_size = 5;
	bulb.Change_Beam_Size(my_size);
	for(unsigned i = 0; i <= 3; i++) {
		cout << "\nSOLVER Bulb, OP ORDER: " << kUp << ", " << kLeft << ", " << kRight << ", " << kDown << ", Beam Size " << my_size << ", Discrepancy " << i  << endl;
		bulb.Set_Initial_Discrepancies(i);
		bulb.Set_Max_Discrepancies(i);
		general_batch_puzzles(num_cols, num_rows, &bulb, puzzles, f_op_order, true, STATE_PATH);
	}*/

	/**
	Prob Bulb Experiments
	**/
	/*
	unsigned my_size = 10;
	GeneralProbBulb<MNPuzzleState, slideDir, MNPuzzle> prob_bulb;
	prob_bulb.Select_Full_Check(true);
	prob_bulb.Change_Beam_Size(my_size);
	prob_bulb.Change_Memory_Limit(100000);
	prob_bulb.debug = false;
	prob_bulb.Select_Duplicate_Prune(true);
	prob_bulb.Select_Full_Check(true);
	//prob_bulb.Set_Max_Discrepancies(2);

	double rand_weight = 1.5;
	prob_bulb.Set_Random_Weight(rand_weight);
	prob_bulb.Set_Init_Seed(0);
	prob_bulb.Set_Seed_Inc(100);

	for(double j  = 1.0; j < 3.0; j += 0.5) {
		for(unsigned i = 0; i < 10000; i+= 1000) {

			cout << "\nSOLVER Bulb, OP ORDER: " << kUp << ", " << kLeft << ", " << kRight << ", " << kDown << ", Beam Size " << my_size << ", Seed " << i  << ", Rand Weight: " << j << endl;
			prob_bulb.Set_Random_Weight(j);
			prob_bulb.Set_Init_Seed(i);
			prob_bulb.Set_Seed_Inc(100);


			general_batch_puzzles(num_cols, num_rows, &prob_bulb, puzzles, f_op_order, true, STATE_PATH);
		}
	}*/
	/**
	Turn-taking experiments
	**/
	//TurnTakingSimulation::output_turntaking_input_file("../../apps/dynamicsearch/input/6x6_ida_weight5", 100, "../../apps/dynamicsearch/input/6x6_ida_weight5_tt");


	vector<unsigned> desired_puzzles;
	for(unsigned i = 0; i < 100; i++) {
		desired_puzzles.push_back(i);
	}

	unsigned start = 0;
	unsigned inc = 1;
	unsigned total_solvers = 4;
	vector<unsigned> desired_weights;
	for(unsigned i = start; i < start + inc*(total_solvers - 1) + 1; i+= inc) {
		desired_weights.push_back(i);
	}

	vector<unsigned> set_sizes;
	//for(unsigned i = 2; i <= total_solvers; i++) {
	for(unsigned i = total_solvers; i <= total_solvers; i++) {
		set_sizes.push_back(i);
	}

	vector<unsigned> num_per_size;

	//for(unsigned i = 2; i <= total_solvers; i++) {
	for(unsigned i = total_solvers; i <= total_solvers; i++) {
		num_per_size.push_back(0);
	}


	/*
	num_per_size.push_back(0);
	num_per_size.push_back(0);

	for(unsigned i = 4; i <= 20; i++) {
		num_per_size.push_back(5000);
	}
	num_per_size.push_back(0);
	num_per_size.push_back(0);
	num_per_size.push_back(0);
	num_per_size.push_back(0);
*/

	vector<unsigned> combo;
	double total_combo_nodes, total_combo_cost;
	unsigned combo_solved;
	for(unsigned i = 0; i < 4; i++) {
		combo.push_back(i);
	}
	/*
	TurnTakingSimulation simulator("../../apps/dynamicsearch/input/6x6_ida_weight5_tt");
	simulator.output_solver_names(desired_weights);
	//simulator.simulate(desired_puzzles, desired_weights, set_sizes, num_per_size, true, false);
	simulator.turn_taking_on_weight_set(desired_puzzles, desired_weights, combo, total_combo_nodes, total_combo_cost, combo_solved, true, false, true);*/

	/*
	Timer t, t2;
	t.StartTimer();
	ida.GetPath(&mnp, puzzles[0], goal, path);
	t.EndTimer();
	cout << "Path found, length: " << path.size() << endl;
	cout << "Nodes Checked: " << ida.GetNodesChecked() << endl;
	cout << "Nodes Generated: " << ida.GetNodesGenerated() << endl;
	cout << "Nodes Expanded: " << ida.GetNodesExpanded() << endl;
	cout << "Time:" << t.GetElapsedTime() << endl;
	path.resize(0);

	t2.StartTimer();
	ida.initialize_step_by_step(&mnp, puzzles[0], goal, false);
	while(ida.move_one_step(goal, path) != 1) {}
	t2.EndTimer();
	cout << "Path found, length: " << path.size() << endl;
	cout << "Nodes Checked: " << ida.GetNodesChecked() << endl;
	cout << "Nodes Generated: " << ida.GetNodesGenerated() << endl;
	cout << "Nodes Expanded: " << ida.GetNodesExpanded() << endl;
	cout << "Time:" << t2.GetElapsedTime() << endl;*/

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

	unsigned pan_size = 14;
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
	//MNPuzzle::Create_Random_MN_Puzzles(goal, puzzles, 1000);
	//mnp.Output_Puzzles(puzzles, false);

	vector<PancakePuzzleState> pancake_puzzles;
	get_14pancake_test_set(pancake_puzzles, 10);

	PancakePuzzleState pan_goal(pan_size);


	//pancake_puzz.Load_Regular_PDB("../../apps/dynamicsearch/input/14panc_pdb_0_1_2_3_4_5_6_distinct", pan_goal, false);
	//pancake_puzz.Load_Regular_PDB("../../apps/dynamicsearch/input/14panc_pdb_7_8_9_10_11_12_13_distinct", pan_goal, false);


	/**
	IDA* Pancake Experiments
	**/
	/*
	GeneralIDA<PancakePuzzleState, unsigned, PancakePuzzle> pan_ida;

	for (double w = 1.0; w <= 1.0; w += 1.0) {
		pan_ida.Change_Weights(1.0, w);
		cout << "\nSOLVER IDA*, OP ORDER: ";
		for (unsigned j = pan_size; j > 1; j--) {
			cout << j << ", ";
		}
		cout << "Weight: " << w << endl;
		general_batch_pancake_puzzles(pancake_puzz, pan_size, &pan_ida, pancake_puzzles, true, ACTION_PATH);
	}
	*/

	/**
	RBFS Pancake Experiments
	**/

	/*
	GeneralRBFS<PancakePuzzleState, unsigned, PancakePuzzle> pan_rbfs;

	for (double w = 2.0; w <= 152.0; w += 10.0) {
		pan_rbfs.Change_Weights(1.0, w);
		cout << "\nSOLVER RBFS, OP ORDER: ";
		for (unsigned j = pan_size; j > 1; j--) {
			cout << j << ", ";
		}
		cout << "Weight: " << w << endl;
		general_batch_pancake_puzzles(pancake_puzz, pan_size, &pan_rbfs, pancake_puzzles, false, ACTION_PATH);
	}
	*/
	/**
	BEAM / BULB Puzzle Experiments
	**/
	/*
	GeneralBeamSearch<PancakePuzzleState, unsigned, PancakePuzzle> pan_beam_search;
	pan_beam_search.Change_Beam_Size(100);
	pan_beam_search.Change_Memory_Limit(100000);
	pan_beam_search.debug = false;
	pan_beam_search.Select_Duplicate_Prune(true);
	pan_beam_search.Select_Full_Check(true);

	GeneralBulb<PancakePuzzleState, unsigned, PancakePuzzle> pan_bulb;
	pan_bulb.Select_Full_Check(true);
	pan_bulb.Change_Beam_Size(100);
	pan_bulb.Change_Memory_Limit(100000);
	pan_bulb.debug = false;
	pan_bulb.Select_Duplicate_Prune(true);
	pan_bulb.Select_Full_Check(true);
	//pan_bulb.Set_Max_Discrepancies(0);

	for (unsigned i = 0; i <= 10; i++) {
		pan_bulb.Change_Beam_Size(i);
		cout << "\nSOLVER BULB, OP ORDER: ";
		for (unsigned j = pan_size; j > 1; j--) {
			cout << j << ", ";
		}
		cout << "Beam Size: " << i << endl;
		general_batch_pancake_puzzles(pancake_puzz, pan_size, &pan_bulb, pancake_puzzles, true, STATE_PATH);
	}

	*/

	/*
	vector<unsigned> test_ops;
	int64_t num = 0;
	for(int64_t j = 0; j < 13; j++) {
		num = j*479001600ll;
		int r = rand() % 479001600;
		cout << r << endl;
		if(j == 12)
			num += 479001599ll;
		else if(j != 0)
			num += r;

		cout << "Num: " << num << "     ";
		test_ops = PancakePuzzle::Get_Puzzle_Order(num, 14);

		for(unsigned i = 0; i < test_ops.size(); i++) {
			cout << test_ops[i] << " ";
		}
		cout << endl;
	}*/
	info.clear();
	read_in_extra_puzz_info("../../apps/dynamicsearch/input/6x6_ida_input", info, '\t', 100);

	vector<double> my_weights;

	my_weights.push_back(4);

	for(unsigned i = 6; i < 11; i++) {
		my_weights.push_back(i);
	}
	my_weights.push_back(3);
	my_weights.push_back(2);

	my_weights.push_back(11);
	my_weights.push_back(12);
	my_weights.push_back(13);
	my_weights.push_back(14);
	my_weights.push_back(15);

	ida_6x6_experiments(info, my_weights, 1);
	return 0;
}
