#include <vector>
#include <stdio.h>
#include <sstream>
#include "Common.h"
#include "dynsearch.h"
#include "MNPuzzle.h"
#include "GeneralIDA.h"

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

	vector<MNPuzzleState> puzzles;

	if(MNPuzzle::read_in_mn_puzzles("../input/standard_test_set", false, 4, 4, 1000, puzzles)) {
		printf("FAILED\n");
	}

	MNPuzzleState start(4, 4);
	start.puzzle[0] = 1;
	start.puzzle[1] = 2;
	start.puzzle[2] = 3;
	start.puzzle[3] = 7;
	start.puzzle[7] = 11;
	start.puzzle[11] = 15;
	start.puzzle[15] = 0;
	start.blank = 15;

	MNPuzzleState goal(4, 4);

	GeneralIDA<MNPuzzleState, slideDir> ida;
	std::vector<slideDir> path;

	std::vector<slideDir> op_order;

	op_order.push_back(kUp);
	op_order.push_back(kLeft);
	op_order.push_back(kRight);
	op_order.push_back(kDown);

	MNPuzzle mnp(4, 4, op_order);
	mnp.StoreGoal(goal);

	ida.SetExpandFullIteration(true);
	Timer t, t2;
	t.startTimer();
	ida.GetPath(&mnp, puzzles[0], goal, path);
	t.endTimer();
	cout << "Path found, length: " << path.size() << endl;
	cout << "Nodes Checked: " << ida.GetNodesChecked() << endl;
	cout << "Nodes Generated: " << ida.GetNodesGenerated() << endl;
	cout << "Nodes Expanded: " << ida.GetNodesExpanded() << endl;
	cout << "Time:" << t.getElapsedTime() << endl;

	t2.startTimer();
	ida.initialize_step_by_step(&mnp, puzzles[0], goal);

	while(ida.move_one_step(path) != 1) {}
	t2.endTimer();
	cout << "Path found, length: " << path.size() << endl;
	cout << "Nodes Checked: " << ida.GetNodesChecked() << endl;
	cout << "Nodes Generated: " << ida.GetNodesGenerated() << endl;
	cout << "Nodes Expanded: " << ida.GetNodesExpanded() << endl;
	cout << "Time:" << t2.getElapsedTime() << endl;

	return 0;
}

/*
void CreateSimulation(int id)
{
}*/
