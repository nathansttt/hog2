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

	get_standard_test_set(puzzles, info, solver_info, 100);
	/*
	unsigned num_cols = 3;
	unsigned num_rows = 6;
*/
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
	vector<MNPuzzleState> state_path;
	MNPuzzleState goal(4, 4);
	MNPuzzle mnp(4, 4, f_op_order);
	mnp.StoreGoal(goal);

	//ida.SetCheckedLimit(10);
	//ida.GetPath(&mnp, puzzles[0], goal, path);
	//cout << ida.GetNodesChecked() << endl;
	for(unsigned i = 2; i <= 20; i++) {
		//cout << "\nWeight: " << i << endl;
		ida.Change_Weights(1.0, i);
		//batch_puzzles(num_cols, num_rows, &ida, puzzles, info, f_op_order, true, false);
		//ida.GetPath(&mnp, puzzles[49], goal, path);
		//printf("%.0f\t", ida.GetNodesChecked());
	}
	//cout << '\n';

	/*
	output_exp_info("../../../../prog_projects/order_weight18", 100, 24, "order_weight18_nodes", "order_weight18_costs");
	output_exp_info("../../../../prog_projects/order_weight19", 100, 24, "order_weight19_nodes", "order_weight19_costs");
	output_exp_info("../../../../prog_projects/order_weight20", 100, 24, "order_weight20_nodes", "order_weight20_costs");
	output_exp_info("../../../../prog_projects/order_weight21", 100, 24, "order_weight21_nodes", "order_weight21_costs");
	output_exp_info("../../../../prog_projects/order_weight22", 100, 24, "order_weight22_nodes", "order_weight22_costs");
	output_exp_info("../../../../prog_projects/order_weight23", 100, 24, "order_weight23_nodes", "order_weight23_costs");
	output_exp_info("../../../../prog_projects/order_weight24", 100, 24, "order_weight24_nodes", "order_weight24_costs");
	output_exp_info("../../../../prog_projects/order_weight25", 100, 24, "order_weight25_nodes", "order_weight25_costs");*/


	//ida.Change_Weights(1.0, 3.0);
	/*
	unsigned num = 0;
	GeneralRBFS<MNPuzzleState, slideDir> rbfs;
	rbfs.Change_Weights(1.0, 3.0);
	//rbfs.SetExpandedLimit(5000);

	for(; num < 10; num++) {
	rbfs.GetPath(&mnp, puzzles[num], goal, path);

	cout << rbfs.GetPathCost() << "\t" << rbfs.GetNodesChecked() << endl;
	}*/

	unsigned num = 0;
	GeneralBeamSearch<MNPuzzleState, slideDir> bs;
	bs.Change_Beam_Size(30);
	bs.Change_Memory_Limit(10000);
	bs.Select_Duplicate_Prune(false);
	bs.Initialize(&mnp, puzzles[num], goal);
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

	GeneralBulb<MNPuzzleState, slideDir> bulb;
	bulb.Change_Beam_Size(15);
	bulb.Change_Memory_Limit(70000);
	//bulb.SetExpandedLimit(30);
	bulb.GetPath(&mnp, puzzles[num], goal, state_path);

	cout << state_path.size() << endl;/*
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

	/*
	string nodes_filename = "../../apps/dynamicsearch/input/order_weight10_nodes";
	string cost_filename = "../../apps/dynamicsearch/input/order_weight10_costs";
	ind_approx(nodes_filename.c_str(), cost_filename.c_str(), 24, 100, 0, 23, 2, 2, 276, false);
	cout << "\n3-22\n";
	ind_approx(nodes_filename.c_str(), cost_filename.c_str(), 24, 100, 0, 23, 3, 22, 250, false);
	cout << "\n23\n";
	ind_approx(nodes_filename.c_str(), cost_filename.c_str(), 24, 100, 0, 23, 23, 23, 24, false);
	cout << "\n24\n";
	ind_approx(nodes_filename.c_str(), cost_filename.c_str(), 24, 100, 0, 23, 24, 24, 1, false);
*/
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

	return 0;
}
