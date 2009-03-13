/*
 *  MNPuzzle.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/9/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "MNPuzzle.h"
#include "Heap.h"
#include "GraphEnvironment.h"
#include <map>

MNPuzzle::MNPuzzle(unsigned int _width, unsigned int _height)
		: width(_width), height(_height)
{

	// stores applicable operators at each of the width*height positions
	// The order of operators is Right, Left, Down, Up
	std::vector<slideDir> ops(4);
	for (unsigned int blank = 0; blank < width*height; blank++)
	{
		ops.resize(0);

		if (blank % width < width - 1)
		{
			ops.push_back(kRight);
		}
		if (blank % width > 0)
		{
			ops.push_back(kLeft);
		}
		if (blank < width*height - width)
		{
			ops.push_back(kDown);
		}
		if (blank > width - 1)
		{
			ops.push_back(kUp);
		}

		operators.push_back(ops);
	}

	goal_stored = false;
}

MNPuzzle::MNPuzzle(unsigned int _width, unsigned int _height,
                   std::vector<slideDir> &op_order) :
		width(_width), height(_height)
{
	bool up_act = false;
	bool down_act = false;
	bool left_act = false;
	bool right_act = false;

	assert(op_order.size() == 4);

	for (unsigned int op_num = 0; op_num < 4; op_num++)
	{
		if (op_order[op_num] == kUp)
		{
			up_act = true;
		}
		else if (op_order[op_num] == kDown)
		{
			down_act = true;
		}
		else if (op_order[op_num] == kLeft)
		{
			left_act = true;
		}
		else if (op_order[op_num] == kRight)
		{
			right_act = true;
		}
	}

	assert(up_act && down_act && left_act && right_act);

	// stores applicable operators at each of the width*height positions
	std::vector<slideDir> ops(4);
	for (unsigned int blank = 0; blank < width*height; blank++)
	{
		ops.resize(0);
		for (unsigned int op_num = 0; op_num < 4; op_num++)
		{
			if (op_order[op_num] == kUp && blank > width - 1)
			{
				ops.push_back(kUp);
			}
			if (op_order[op_num] == kLeft && blank % width > 0)
			{
				ops.push_back(kLeft);
			}
			if (op_order[op_num] == kRight && blank % width < width - 1)
			{
				ops.push_back(kRight);
			}
			if (op_order[op_num] == kDown && blank < width*height - width)
			{
				ops.push_back(kDown);
			}
		}

		operators.push_back(ops);
	}
	goal_stored = false;
}

MNPuzzle::~MNPuzzle()
{
	ClearGoal();
}


Graph *MNPuzzle::GetGraph()
{
	Graph *g = new Graph();
	//Factorial(width*height);

	for (int x = 0; x < 362880; x++)
	{
		node *n;
		g->AddNode(n = new node(""));
		n->SetLabelF(GraphSearchConstants::kXCoordinate, (32768.0-random()%65536)/32768.0);
		n->SetLabelF(GraphSearchConstants::kYCoordinate, (32768.0-random()%65536)/32768.0);
		n->SetLabelF(GraphSearchConstants::kZCoordinate, (32768.0-random()%65536)/32768.0);
	}
	for (int x = 0; x < 362880; x++)
	{
		MNPuzzleState t(3, 3);
		GetStateFromHash(t, x);
		std::vector<slideDir> moves;
		GetActions(t, moves);
		g->GetNode(x)->SetLabelF(GraphSearchConstants::kZCoordinate, GetParity(t));
		for (unsigned int y = 0; y < moves.size(); y++)
		{
			ApplyAction(t, moves[y]);
			uint64_t hash = GetStateHash(t);
			InvertAction(moves[y]);
			ApplyAction(t, moves[y]);
			if (!g->FindEdge(x, hash))
				g->AddEdge(new edge(x, hash, 1));
		}
	}
	// 362880 states
	// 2x2 area -- roughly 600x600 states, or distance 1 = 1/600
//	for (int t = 0; t < 362880; t++)
//	{
//		node *n = g->GetNode(t);
//		neighbor_iterator ni = n->getNeighborIter();
//		for (long tmp = n->nodeNeighborNext(ni); tmp != -1; tmp = n->nodeNeighborNext(ni))
//		{
//			double x, y, z;
//			x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
//			y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
//			z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
//
//			double x1, y1, z1;
//			node *nb = g->GetNode(tmp);
//			x1 = nb->GetLabelF(GraphSearchConstants::kXCoordinate);
//			y1 = nb->GetLabelF(GraphSearchConstants::kYCoordinate);
//			z1 = nb->GetLabelF(GraphSearchConstants::kZCoordinate);
//			// now move n to be 1/600 away from nb!
//			n->SetLabelF(GraphSearchConstants::kXCoordinate, 0.5*x + 0.5*x1);
//			n->SetLabelF(GraphSearchConstants::kYCoordinate, 0.5*y + 0.5*y1);
//			n->SetLabelF(GraphSearchConstants::kZCoordinate, 0.5*z + 0.5*z1);
//
//		}
//	}
	return g;
}

void MNPuzzle::StoreGoal(MNPuzzleState &s)
{
	assert(s.height == height);
	assert(s.width == width);

	// makes sure goal contains all legal
	bool goal_tester[width*height];
	for(unsigned int i = 0; i < width*height; i++) {
		goal_tester[i] = false;
	}

	for(unsigned int i = 0; i < width*height; i++) {
		goal_tester[s.puzzle[i]] = true;
	}

	for(unsigned int i = 0; i < width*height; i++) {
		if(!goal_tester[i]) {
			printf("INVALID GOAL\n");
			assert(goal_tester[i]);
		}
	}

	h_increment = new unsigned*[width*height];

	for(unsigned int i = 1; i < width*height; i++) {
		h_increment[i] = new unsigned [width*height];
	}

	for(unsigned goal_pos = 0; goal_pos < width*height; goal_pos++) {
		unsigned tile = s.puzzle[goal_pos];
		if(tile == 0)
			continue;

		for(unsigned pos = 0; pos < width*height; pos++) {
			h_increment[tile][pos] = abs((int) (goal_pos % width) - (int)(pos % width)); // difference in column
			h_increment[tile][pos] += abs((int) (goal_pos / width) - (int)(pos / width)); // difference in row
		}
	}

	goal_stored = true;

	goal = s;
}

bool MNPuzzle::GoalTest(MNPuzzleState &s) {
	return (s == goal);
}
void MNPuzzle::ClearGoal()
{
	if(goal_stored) {
		goal_stored = false;
		// clears memory allocated for h_increment
		for(unsigned int i = 1; i < width*height; i++) {
			delete h_increment[i];
		}
		delete h_increment;
	}
}
void MNPuzzle::GetSuccessors(MNPuzzleState &stateID,
                             std::vector<MNPuzzleState> &neighbors)
{
	neighbors.resize(0);

	for (unsigned int i = 0; i < operators[stateID.blank].size(); i++)
	{
		neighbors.push_back(stateID);
		ApplyAction(neighbors.back(), operators[stateID.blank][i]);
	}
}

void MNPuzzle::GetActions(MNPuzzleState &stateID, std::vector<slideDir> &actions)
{
	actions.resize(0);
	for (unsigned int i = 0; i < operators[stateID.blank].size(); i++)
	{
		actions.push_back(operators[stateID.blank][i]);
	}
}

slideDir MNPuzzle::GetAction(MNPuzzleState &s1, MNPuzzleState &s2)
{
	return kUp;
}

void MNPuzzle::ApplyAction(MNPuzzleState &s, slideDir a)
{
	switch (a)
	{
		case kUp:
			if (s.blank >= s.width)
			{
				s.puzzle[s.blank] = s.puzzle[s.blank-s.width];
				s.blank -= s.width;
				s.puzzle[s.blank] = 0;
			}
			break;
		case kDown:
			if (s.blank < s.puzzle.size() - s.width)
			{
				s.puzzle[s.blank] = s.puzzle[s.blank+s.width];
				s.blank += s.width;
				s.puzzle[s.blank] = 0;
			}
			break;
		case kRight:
			if ((s.blank%s.width) < s.width-1)
			{
				s.puzzle[s.blank] = s.puzzle[s.blank+1];
				s.blank += 1;
				s.puzzle[s.blank] = 0;
			}
			break;
		case kLeft:
			if ((s.blank%s.width) > 0)
			{
				s.puzzle[s.blank] = s.puzzle[s.blank-1];
				s.blank -= 1;
				s.puzzle[s.blank] = 0;
			}
			break;
	}
}

bool MNPuzzle::InvertAction(slideDir &a)
{
	switch (a)
	{
		case kLeft: a = kRight; break;
		case kUp: a = kDown; break;
		case kDown: a = kUp; break;
		case kRight: a = kLeft; break;
	}
	return true;
}

double MNPuzzle::HCost(MNPuzzleState &state) {
	assert(goal_stored);

	assert(state.height == height);
	assert(state.width == width);
	double hval = 0;

	// increments amound for each tile location
	// this calculates the Manhattan distance
	for(unsigned loc = 0; loc < width*height; loc++) {
		if(state.puzzle[loc] != 0)
			hval += h_increment[state.puzzle[loc]][loc];
	}

	if (PDB.size() != 0) // select between PDB and Manhattan distance if given the chance
		return std::max(hval, DoPDBLookup(state));
	return hval;
}

// TODO Remove PDB heuristic from this heuristic evaluator.
double MNPuzzle::HCost(MNPuzzleState &state1, MNPuzzleState &state2)
{
	assert(state1.height==state2.height);
	assert(state1.width==state2.width);
	double hval = 0;

	if (goal_stored) // if goal is stored
	{
		assert(state1.height == height);
		assert(state1.width == width);

		// increments amound for each tile location
		for(unsigned loc = 0; loc < width*height; loc++) {
			if(state1.puzzle[loc] != 0)
				hval += h_increment[state1.puzzle[loc]][loc];
		}
	}
	else
	{
		std::vector<int> xloc(state2.width*state2.height);
		std::vector<int> yloc(state2.width*state2.height);

		for (unsigned int x = 0; x < state2.width; x++)
		{
			for (unsigned int y = 0; y < state2.height; y++)
			{
				xloc[state2.puzzle[x + y*state2.width]] = x;
				yloc[state2.puzzle[x + y*state2.width]] = y;
			}
		}
		for (unsigned int x = 0; x < state1.width; x++)
		{
			for (unsigned int y = 0; y < state1.height; y++)
			{
				if (state1.puzzle[x + y*state1.width] != 0)
				{
					hval += (abs(xloc[state1.puzzle[x + y*state1.width]] - x) +
					         abs(yloc[state1.puzzle[x + y*state1.width]] - y));
				}
			}
		}
	}
	if (PDB.size() != 0)
		return std::max(hval, DoPDBLookup(state1));
	return hval;
}

double MNPuzzle::GCost(MNPuzzleState &, MNPuzzleState &)
{
	return 1;
}

bool MNPuzzle::GoalTest(MNPuzzleState &state, MNPuzzleState &goal)
{
	return (state == goal);
}

uint64_t MNPuzzle::GetStateHash(MNPuzzleState &state)
{
	std::vector<int> puzzle = state.puzzle;
	uint64_t hashVal = 0;
	int numEntriesLeft = state.puzzle.size();
	for (unsigned int x = 0; x < state.puzzle.size(); x++)
	{
		hashVal += puzzle[x]*Factorial(numEntriesLeft-1);
		numEntriesLeft--;
		for (unsigned y = x; y < puzzle.size(); y++)
		{
			if (puzzle[y] > puzzle[x])
				puzzle[y]--;
		}
	}
	return hashVal;
}

void MNPuzzle::GetStateFromHash(MNPuzzleState &state, uint64_t hash)
{
	std::vector<int> puzzle = state.puzzle;
	uint64_t hashVal = hash;

	int numEntriesLeft = 1;
	for (int x = state.puzzle.size()-1; x >= 0; x--)
	{
		puzzle[x] = hashVal%numEntriesLeft;
		hashVal /= numEntriesLeft;
		numEntriesLeft++;
		for (int y = x+1; y < (int) state.puzzle.size(); y++)
		{
			if (puzzle[y] >= puzzle[x])
				puzzle[y]++;
		}
	}

	state.puzzle = puzzle;
	for (unsigned int x = 0; x < puzzle.size(); x++)
		if (puzzle[x] == 0)
			state.blank = x;

}


uint64_t MNPuzzle::GetActionHash(slideDir act)
{
	switch (act)
	{
		case kUp: return 0;
		case kDown: return 1;
		case kRight: return 2;
		case kLeft: return 3;
	}
	return 4;
}

void MNPuzzle::OpenGLDraw(int window)
{
}


void MNPuzzle::OpenGLDraw(int window, MNPuzzleState &s)
{
	glLineWidth(1.0);
	glEnable(GL_LINE_SMOOTH);

	float w = width;
	float h = height;

	for (unsigned int y = 0; y < height; y++)
	{
		for (unsigned int x = 0; x < width; x++)
		{
			glPushMatrix();
			glColor3f(0.0, 1.0, 0.0);
			glTranslatef(x*2.0/w-1.0, (1+y)*2.0/h-1.0, -0.001);
			glScalef(1.0/(w*120.0), 1.0/(h*120.0), 1);
			glRotatef(180, 0.0, 0.0, 1.0);
			glRotatef(180, 0.0, 1.0, 0.0);
			//glTranslatef((float)x/width-0.5, (float)y/height-0.5, 0);
			if (s.puzzle[x+y*width] > 9)
				glutStrokeCharacter(GLUT_STROKE_ROMAN, '0'+(((s.puzzle[x+y*width])/10)%10));
			if (s.puzzle[x+y*width] > 0)
				glutStrokeCharacter(GLUT_STROKE_ROMAN, '0'+((s.puzzle[x+y*width])%10));
			//glTranslatef(-x/width+0.5, -y/height+0.5, 0);
			glPopMatrix();
		}
	}

	glBegin(GL_LINES);
	for (unsigned int y = 0; y <= height; y++)
	{
		for (unsigned int x = 0; x <= width; x++)
		{
			glVertex3f(x*2.0/w-1.0, -1, -0.001);
			glVertex3f(x*2.0/w-1.0, 1, -0.001);
			glVertex3f(-1, (y)*2.0/h-1.0, -0.001);
			glVertex3f(1, (y)*2.0/h-1.0, -0.001);
		}
	}
	glEnd();

	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glEnable(GL_BLEND);
	//glEnable(GL_LINE_SMOOTH);
	//output(200, 225, "This is antialiased.");

	//int width, height;
	//std::vector<int> puzzle;
}

uint64_t MNPuzzle::Factorial(int val)
{
	static uint64_t table[21] =
	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
	6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
	6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
	if (val > 20)
		return (uint64_t)-1;
	return table[val];
}

/*
void MNPuzzle::LoadPDB(char *fname, const std::vector<int> &tiles, bool )
{
	std::vector<int> values(256);
	uint64_t COUNT = Factorial(width*height)/Factorial(width*height-tiles.size());
	PDB.resize(PDB.size()+1);
	PDB.back().resize(COUNT);
	FILE *f = fopen(fname, "r");
	if (f)
	{
		fread(&(PDB.back()[0]), sizeof(uint8_t), COUNT, f);
		fclose(f);
	}
	PDBkey.push_back(tiles);
	for (unsigned int x = 0; x < COUNT; x++)
	{
		values[(PDB.back()[x])]++;
	}
	for (int x = 0; x < 256; x++)
		printf("%d:\t%d\n", x, values[x]);
}*/

double MNPuzzle::DoPDBLookup(MNPuzzleState &state)
{
	double val = 0;
	for (unsigned int x = 0; x < PDB.size(); x++)
	{
		uint64_t index = GetPDBHash(state, PDBkey[x]);
		val = std::max(val, (double)PDB[x][index]);
		//val += (double)PDB[x][index];
	}
	if (width == height) // symmetry
	{
	}
	return val;
}

/**
Reads in MNPuzzle states from the given filename. Each line of the
input file contains a puzzle state of the given width and height.
The state is written in the form of listing the tile in each puzzle
position from 1 to width*height. The blank is represented by the 0
character.

If any line is found to not be legitimate, either because it does
not have the proper number of tiles, or the proper tiles, it is
simply not added to the list of puzzles.

The function will return 1 if reading from the given filename failed,
and 0 otherwise.

puzz_num_start - should be set as true if the first non-whitespace
element on each line is the puzzle number, otherwise should be set
to false.

max_puzzles - the maximum number of puzzles that will be added to
the puzzle list.
**/
int MNPuzzle::read_in_mn_puzzles(const char *filename, bool puzz_num_start, unsigned int width, unsigned int height, unsigned int max_puzzles, std::vector<MNPuzzleState> &puzzles) {

	std::ifstream ifs(filename, std::ios::in);

	if(ifs.fail()) {
		return 1;
	}

	std::string s, temp;

	std::vector<unsigned int> puzz_ints;
	std::vector<bool> tiles_in_puzzle(width*height);
	bool first = true;
	unsigned puzz_count = 0;

	while(!ifs.eof() && puzz_count < max_puzzles) {
		puzz_ints.clear();

		for(unsigned i = 0; i < tiles_in_puzzle.size(); i++) {
			tiles_in_puzzle[i] = false;
		}

		getline(ifs, s);
		first = true;
		for(unsigned int i = 0; i < s.length(); i++) {
			if(s.at(i) == ' ' || s.at(i) == '\t') {
				if(temp.length() > 0) {
					if(puzz_num_start && first) {
						temp.clear();
						first = false;
					}
					else {
						puzz_ints.push_back(atoi(temp.c_str()));
						temp.clear();
					}
				}
			}
			else {
				temp.push_back(s.at(i));
			}
		}

		if(temp.length() > 0) {

			puzz_ints.push_back(atoi(temp.c_str()));
			temp = "";
		}

		if(puzz_ints.size() > 0 && puzz_ints.size() == width*height) {
			MNPuzzleState new_state(width, height);
			for(unsigned int i = 0; i < puzz_ints.size(); i++) {
				new_state.puzzle[i] = puzz_ints[i];
				tiles_in_puzzle[puzz_ints[i]] = true;
				if(new_state.puzzle[i] == 0) {
					new_state.blank = i;
				}
			}

			bool is_good = true;
			for(unsigned int i = 0; i < tiles_in_puzzle.size(); i++) {
				if(tiles_in_puzzle[i] = false) {
					is_good = false;
					break;
				}
			}

			if(is_good) {
				puzz_count++;
				puzzles.push_back(new_state);
			}

		}
	}

	ifs.close();

	return 0;
}

GraphPuzzleDistanceHeuristic::GraphPuzzleDistanceHeuristic(MNPuzzle &mnp, Graph *graph, int count)
:GraphDistanceHeuristic(graph), puzzle(mnp)
{
	for (int x = 0; x < count /*10*/; x++)
	{
		AddHeuristic();
	}
}

double GraphPuzzleDistanceHeuristic::HCost(graphState &state1, graphState &state2)
{
	MNPuzzleState a(3, 3), b(3, 3);
	puzzle.GetStateFromHash(a, state1);
	puzzle.GetStateFromHash(b, state2);
	double val = puzzle.HCost(a, b);

	for (unsigned int i=0; i < heuristics.size(); i++)
	{
		double hval = heuristics[i][state1]-heuristics[i][state2];
		if (hval < 0)
			hval = -hval;
		if (fgreater(hval,val))
			val = hval;
	}

	return val;
}


/**
Randomly generates a puzzle of the specified dimensions
and returns that puzzle.
**/
MNPuzzleState random_puzzle_generator(unsigned num_cols, unsigned num_rows) {
	MNPuzzleState new_puzz(num_cols, num_rows);
	unsigned size = num_cols*num_rows;
	int index = 0;
	int temp;
	while(size > 1) {
		index = rand() % size;
		temp = new_puzz.puzzle[size - 1];
		new_puzz.puzzle[size - 1] = new_puzz.puzzle[index];
		new_puzz.puzzle[index] = temp;

		size--;
	}

	for(unsigned i = 0; i < num_cols*num_rows; i++) {
		if(new_puzz.puzzle[i] == 0)
			new_puzz.blank = i;
	}

	return new_puzz;
}

//	if q is odd, the invariant is the parity (odd or even) of the number of pairs of pieces in reverse order (the parity of the permutation). For the order of the 15 pieces consider line 2 after line 1, etc., like words on a page.
//	if q is even, the invariant is the parity of the number of pairs of pieces in reverse order plus the row number of the empty square counting from bottom and starting from 0.
unsigned MNPuzzle::GetParity(MNPuzzleState &state)
{
	unsigned swaps = 0; // counts number of swaps
	for (unsigned x = 0; x < state.puzzle.size(); x++)
	{
		if (state.puzzle[x] == 0) // skip blank
			continue;
		for (unsigned y = x + 1; y < state.puzzle.size(); y++)
		{
			if (state.puzzle[y] == 0) // skip blank
				continue;
			if (state.puzzle[y] < state.puzzle[x])
				swaps++;
		}
	}
	// if odd num of columns
	if ((state.width % 2) == 1)
	{
		return swaps % 2;
	}

	// if even num of columns
	return (swaps + (state.puzzle.size()-state.blank-1)/state.width)%2;
}

void MNPuzzle::Create_Random_MN_Puzzles(MNPuzzleState &goal, std::vector<MNPuzzleState> &puzzle_vector, unsigned num_puzzles)
{
	std::map<uint64_t, uint64_t> puzzle_map; // used to ensure uniqueness

	MNPuzzle my_puzz(goal.width, goal.height);

	unsigned count = 0;
	unsigned goal_parity = GetParity(goal);

	while (count < num_puzzles)
	{
		MNPuzzleState next = random_puzzle_generator(goal.width, goal.height);
		uint64_t next_hash = my_puzz.GetStateHash(next);

		if (puzzle_map.find(next_hash) != puzzle_map.end())
		{
			continue;
		}

		// checks parity to make sure problem is solvable
		if (GetParity(next) == goal_parity)
		{
			puzzle_map[next_hash] = next_hash;
			puzzle_vector.push_back(next);
			count++;
		}

	}
}

int MNPuzzle::Output_Puzzles(std::vector<MNPuzzleState> &puzzle_vector, unsigned num_cols, unsigned num_rows, bool write_puzz_num) {

	unsigned size = num_cols*num_rows;
	bool in_puzzle[size];

	// check validity of puzzles
	for(unsigned i = 0; i < puzzle_vector.size(); i++) {
		for(unsigned j = 0; j < size; j++) {
			in_puzzle[j] = false;
		}

		if(puzzle_vector[i].width != num_cols || puzzle_vector[i].height != num_rows) {
			std::cerr << "Invalid Puzzle: " << puzzle_vector[i] << '\n';
			return 1;
		}

		for(unsigned j = 0; j < size; j++) {
			if(puzzle_vector[i].puzzle[j] >= (int) size || puzzle_vector[i].puzzle[j] < 0) {
				std::cerr << "Invalid Puzzle: " << puzzle_vector[i] << '\n';
				return 1;
			}
			else if(puzzle_vector[i].puzzle[j] == 0) {
				assert(puzzle_vector[i].blank == j);
			}

			in_puzzle[puzzle_vector[i].puzzle[j]] = true;
		}

		for(unsigned j = 0; j < size; j++) {
			if(!in_puzzle[j]) {
				std::cerr << "Invalid Puzzle: " << puzzle_vector[i] << '\n';
				return 1;
			}
		}

	}

	for(unsigned i = 0; i < puzzle_vector.size(); i++) {
		if(write_puzz_num) {
			printf("%u ", i + 1);
		}
		printf("%d", puzzle_vector[i].puzzle[0]);

		for(unsigned j = 1; j < size; j++) {
			printf(" %d", puzzle_vector[i].puzzle[j]);
		}
		printf("\n");
	}
	return 0;
}
