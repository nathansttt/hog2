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
	weighted = false;
	// stores applicable operators at each of the width*height positions
	Change_Op_Order(Get_Op_Order_From_Hash(15)); // Right, Left, Down, Up is default operator ordering
	goal_stored = false;
	use_manhattan = true;
}

MNPuzzle::MNPuzzle(unsigned int _width, unsigned int _height,
                   const std::vector<slideDir> op_order) :
width(_width), height(_height)
{
	Change_Op_Order(op_order);
	goal_stored = false;
	use_manhattan = true;
	weighted = false;
}

MNPuzzle::~MNPuzzle()
{
	ClearGoal();
}

void MNPuzzle::Change_Op_Order(std::vector<slideDir> op_order)
{
	operators.clear();
	ops_in_order.clear();
	
	bool up_act = false;
	bool down_act = false;
	bool left_act = false;
	bool right_act = false;
	
	if (op_order.size() != 4)
	{
		fprintf(stderr, "ERROR: Not enough operators in operator sequence for construction of MNPuzzle\n");
		exit(1);
	}
	
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
	
	if (!up_act || !down_act || !left_act || !right_act)
	{
		fprintf(stderr, "ERROR: Invalid operator sequence for construction of MNPuzzle\n");
		exit(1);
	}
	
	for (unsigned i = 0; i < op_order.size(); i++)
	{
		ops_in_order.push_back(op_order[i]);
	}
	
	// stores applicable operators at each of the width*height positions
	std::vector<slideDir> ops(4);
	for (unsigned int blank = 0; blank < width*height; blank++) {
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
			if (op_order[op_num] == kRight && (blank % width) < width - 1)// && (blank != 0))
			{
				ops.push_back(kRight);
			}
			if (op_order[op_num] == kDown && blank < width * height - width)// && (blank != 1))
			{
				ops.push_back(kDown);
			}
		}
		
		operators.push_back(ops);
	}
}

const std::string MNPuzzle::GetName(){
	std::stringstream name;
	name << width;
	name << "x";
	name << height;
	name << " Sliding Tile Puzzle";
	
	if (PDB_distincts.size() > 0)
	{
		name << ", PDBS:";
		for (unsigned i = 0; i < PDB_distincts.size(); i++)
		{
			name << " <";
			for (unsigned j = 0; j < PDB_distincts[i].size() - 1; j++)
			{
				name << PDB_distincts[i][j];
				name << ", ";
			}
			name << PDB_distincts[i].back();
			name << ">";
		}
		name << ", Manhattan Distance";
	}
	else {
		name << ", Manhattan Distance";
	}
	
	name << ", Op Order: ";
	for (unsigned op_num = 0; op_num < ops_in_order.size() - 1; op_num++){
		name << ops_in_order[op_num];
		name << ", ";
	}
	name << ops_in_order.back();
	return name.str();
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
	for (unsigned int i = 0; i < width*height; i++)
	{
		goal_tester[i] = false;
	}
	
	for (unsigned int i = 0; i < width*height; i++)
	{
		goal_tester[s.puzzle[i]] = true;
	}
	
	for (unsigned int i = 0; i < width*height; i++)
	{
		if (!goal_tester[i])
		{
			printf("INVALID GOAL\n");
			assert(goal_tester[i]);
		}
	}
	
	h_increment = new unsigned*[width*height];
	
	for (unsigned int i = 1; i < width*height; i++)
	{
		h_increment[i] = new unsigned [width*height];
	}
	
	for (unsigned goal_pos = 0; goal_pos < width*height; goal_pos++)
	{
		unsigned tile = s.puzzle[goal_pos];
		if (tile == 0)
			continue;
		
		for (unsigned pos = 0; pos < width*height; pos++)
		{
			h_increment[tile][pos] = abs((int) (goal_pos % width) - (int)(pos % width)); // difference in column
			h_increment[tile][pos] += abs((int) (goal_pos / width) - (int)(pos / width)); // difference in row
		}
	}
	
	goal_stored = true;
	
	goal = s;
}

bool MNPuzzle::GoalTest(const MNPuzzleState &s)
{
	return (s == goal);
}
void MNPuzzle::ClearGoal()
{
	if (goal_stored)
	{
		goal_stored = false;
		// clears memory allocated for h_increment
		for (unsigned int i = 1; i < width*height; i++)
		{
			delete h_increment[i];
		}
		delete h_increment;
	}
}
void MNPuzzle::GetSuccessors(const MNPuzzleState &stateID,
                             std::vector<MNPuzzleState> &neighbors) const
{
	neighbors.resize(0);
	
	for (unsigned int i = 0; i < operators[stateID.blank].size(); i++)
	{
		neighbors.push_back(stateID);
		ApplyAction(neighbors.back(), operators[stateID.blank][i]);
	}
}

void MNPuzzle::GetActions(const MNPuzzleState &stateID, std::vector<slideDir> &actions) const
{
	actions.resize(0);
	for (unsigned int i = 0; i < operators[stateID.blank].size(); i++)
	{
		actions.push_back(operators[stateID.blank][i]);
	}
}

slideDir MNPuzzle::GetAction(const MNPuzzleState &a, const MNPuzzleState &b) const
{
	int row1 = a.blank%width;
	int col1 = a.blank/height;
	int row2 = b.blank%width;
	int col2 = b.blank/height;
	if (row1 == row2)
	{
		if (col1 > col2)
			return kUp;
		return kDown;
	}
	if (row1 > row2)
		return kLeft;
	return kRight;
}

void MNPuzzle::ApplyAction(MNPuzzleState &s, slideDir a) const
{
	// we actually do the swap to maintain consistency when using abstract states
	// (these contain -1 in some positions, including possibly the blank position.)
	switch (a)
	{
		case kUp:
			if (s.blank >= s.width)
			{
				int tmp = s.puzzle[s.blank];
				s.puzzle[s.blank] = s.puzzle[s.blank-s.width];
				s.blank -= s.width;
				s.puzzle[s.blank] = tmp;
			}
			break;
		case kDown:
			if (s.blank < s.puzzle.size() - s.width)
			{
				int tmp = s.puzzle[s.blank];
				s.puzzle[s.blank] = s.puzzle[s.blank+s.width];
				s.blank += s.width;
				s.puzzle[s.blank] = tmp;
			}
			break;
		case kRight:
			if ((s.blank%s.width) < s.width-1)
			{
				int tmp = s.puzzle[s.blank];
				s.puzzle[s.blank] = s.puzzle[s.blank+1];
				s.blank += 1;
				s.puzzle[s.blank] = tmp;
			}
			break;
		case kLeft:
			if ((s.blank%s.width) > 0)
			{
				int tmp = s.puzzle[s.blank];
				s.puzzle[s.blank] = s.puzzle[s.blank-1];
				s.blank -= 1;
				s.puzzle[s.blank] = tmp;
			}
			break;
	}
}

bool MNPuzzle::InvertAction(slideDir &a) const
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

//double MNPuzzle::HCost(const MNPuzzleState &state)
//{
//	if (!goal_stored)
//	{
//		fprintf(stderr, "ERROR: HCost called with a single state and goal is not stored.\n");
//		exit(1);
//	}
//	if (state.height != height || state.width != width)
//	{
//		fprintf(stderr, "ERROR: HCost called with a single state with wrong size.\n");
//		exit(1);
//	}
//	
//	double hval = 0;
//
////	if (PDB.size() != 0) // select between PDB and Manhattan distance if given the chance
////		hval = std::max(hval, DoPDBLookup(state));
//
//	if (additive && PDB.size() != 0)
//	{
//		int val = 0;
//
//		for (unsigned loc = 0; loc < width*height; loc++)
//		{
//			if (state.puzzle[loc] > 0)
//				val += h_increment[state.puzzle[loc]][loc];
//		}
//		if (val >= hDist.size())
//			hDist.resize(val+1);
//		
//		uint8_t tmp;
//		uint8_t total = 0;
//		for (unsigned int x = 0; x < PDB.size(); x++)
//		{
//			uint64_t index = GetPDBHash(state, PDB_distincts[x]);
//			histogram[PDB[x][index]]++;
//			tmp = PDB[x][index];
//			if (tmp >= hDist[val].size())
//				hDist[val].resize(tmp+1);
//			hDist[val][tmp]++;
//			if (tmp > 4) tmp = 4;
//			total += (double)tmp;
//		}
//		return val+total;
//
////		hval = PDB_Lookup(state);
////		for (unsigned loc = 0; loc < width*height; loc++)
////		{
////			if (state.puzzle[loc] > 0)
////				hval += h_increment[state.puzzle[loc]][loc];
////		}
////		return hval;
//	}
//	
//	if (use_manhattan)
//	{
//		double man_dist = 0;
//		// increments amound for each tile location
//		// this calculates the Manhattan distance
//		for (unsigned loc = 0; loc < width*height; loc++)
//		{
//			if (state.puzzle[loc] > 0)
//				man_dist += h_increment[state.puzzle[loc]][loc];
//		}
//		hval = std::max(hval, man_dist);
//	}
//	// if no heuristic
//	else if (PDB.size()==0)
//	{
//		if (goal == state)
//			return 0;
//		else
//			return 1;
//	}
//	
//	return hval;
//}
//
// TODO Remove PDB heuristic from this heuristic evaluator.
double MNPuzzle::HCost(const MNPuzzleState &state1, const MNPuzzleState &state2)
{
	if (goal_stored)
		return PermutationPuzzleEnvironment<MNPuzzleState, slideDir>::HCost(state1);
	if (state1.height != height || state1.width != width)
	{
		fprintf(stderr, "ERROR: HCost called with a state with wrong size.\n");
		exit(1);
	}
	if (state2.height != height || state2.width != width)
	{
		fprintf(stderr, "ERROR: HCost called with a state with wrong size.\n");
		exit(1);
	}
		
	double hval = 0;
	if (PDB.size() != 0)
		hval = std::max(hval, PDB_Lookup(state1));
	
	if (use_manhattan)
	{
		double man_dist = 0;
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
					man_dist += (abs(xloc[state1.puzzle[x + y*state1.width]] - x)
								 + abs(yloc[state1.puzzle[x + y*state1.width]] - y));
				}
			}
		}
		hval = std::max(hval, man_dist);
	}
	// if no heuristic
	else if (PDB.size()==0)
	{
		if (state1 == state2)
			return 0;
		else
			return 1;
	}
	
	return hval;
}

double MNPuzzle::DefaultH(const MNPuzzleState &state) const
{
	double man_dist = 0;
	// increments amound for each tile location
	// this calculates the Manhattan distance
	for (unsigned loc = 0; loc < width*height; loc++)
	{
		if (state.puzzle[loc] > 0)
			man_dist += h_increment[state.puzzle[loc]][loc];
	}
	return man_dist;
}

static int costs[25] =
{
	3, 5, 4, 7, 10, 5, 3, 3, 8, 9, 2, 10, 10, 1, 2, 1, 1, 4, 7, 9, 6, 10, 2, 8, 8
};

double MNPuzzle::AdditiveGCost(const MNPuzzleState &s, const slideDir &d)
{
	int tile;
	switch (d)
	{
		case kLeft: tile = s.puzzle[s.blank-1]; break;
		case kUp: tile = s.puzzle[s.blank-s.height]; break;
		case kDown: tile = s.puzzle[s.blank+s.height]; break;
		case kRight: tile = s.puzzle[s.blank+1]; break;
	}
	if (tile == -1)
		return 0;
	if (weighted)
		return costs[s.blank];
	return 1;
}

double MNPuzzle::GCost(const MNPuzzleState &a, const MNPuzzleState &b)
{
//	int diff = a.blank - b.blank;
//	
	if (weighted)
		return costs[a.blank];
	return 1;
}

double MNPuzzle::GCost(const MNPuzzleState &s, const slideDir &d)
{
//	double cost;
//	switch (d)
//	{
//		case kLeft: cost = s.puzzle[s.blank-1]; break;
//		case kUp: cost = s.puzzle[s.blank-s.height]; break;
//		case kDown: cost = s.puzzle[s.blank+s.height]; break;
//		case kRight: cost = s.puzzle[s.blank+1]; break;
//	}
//	if (cost < 1) // in PDB might have negative costs
//		cost = 1;
//	return cost;
	if (weighted)
		return costs[s.blank];
	return 1;
}


bool MNPuzzle::GoalTest(const MNPuzzleState &state, const MNPuzzleState &theGoal)
{
	return (state == theGoal);
}

uint64_t MNPuzzle::GetActionHash(slideDir act) const
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

void MNPuzzle::GetStateFromPDBHash(uint64_t hash, MNPuzzleState &s,
								   int count, const std::vector<int> &pattern,
								   std::vector<int> &dual)
{
	uint64_t hashVal = hash;
	dual.resize(pattern.size());
	
	int numEntriesLeft = count-pattern.size()+1;
	for (int x = pattern.size()-1; x >= 0; x--)
	{
		dual[x] = hashVal%numEntriesLeft;
		hashVal /= numEntriesLeft;
		numEntriesLeft++;
		for (int y = x+1; y < pattern.size(); y++)
		{
			if (dual[y] >= dual[x])
				dual[y]++;
		}
	}
	s.puzzle.resize(count);
	std::fill(s.puzzle.begin(), s.puzzle.end(), -1);
	for (int x = 0; x < dual.size(); x++)
	{
		s.puzzle[dual[x]] = pattern[x];
		if (pattern[x] == 0)
			s.blank = dual[x];
	}
	s.width = width;
	s.height = height;
}


void MNPuzzle::OpenGLDraw() const
{
}

void DrawTile(float x, float y, char c1, char c2, int w, int h)
{
	glLineWidth(10.0);
	int textWidth = 0;
	if (c1 != 0)
		textWidth += glutStrokeWidth(GLUT_STROKE_ROMAN, c1);
	if (c2 != 0)
		textWidth += glutStrokeWidth(GLUT_STROKE_ROMAN, c2);
	if (textWidth == 0)
		return;
	//printf("%d\n", textWidth);
	glPushMatrix();
	glColor3f(0.0, 0.0, 1.0);
	glTranslatef(x*2.0/w-1.0, (1+y)*2.0/h-1.0-0.15, -0.001);
	glScalef(1.0/(w*120.0), 1.0/(h*120.0), 1);
	glRotatef(180, 0.0, 0.0, 1.0);
	glRotatef(180, 0.0, 1.0, 0.0);
	glTranslatef(120-textWidth/2, 0, 0);
	if (c1 != 0)
		glutStrokeCharacter(GLUT_STROKE_ROMAN, c1);
	if (c2 != 0)
		glutStrokeCharacter(GLUT_STROKE_ROMAN, c2);
	//glTranslatef(-x/width+0.5, -y/height+0.5, 0);
	glPopMatrix();
	
	glLineWidth(1.0);
	glColor3f(1, 1, 1);
	glBegin(GL_QUADS);
	glVertex3f(x*2.0/w-1+.05/w, (y)*2.0/h-1+.05/h, 0.002);
	glVertex3f((x+1)*2.0/w-1-.05/w, (y)*2.0/h-1+.05/h, 0.002);
	glVertex3f((x+1)*2.0/w-1-.05/w, (y+1)*2.0/h-1-.05/h, 0.002);
	glVertex3f(x*2.0/w-1+.05/w, (y+1)*2.0/h-1-.05/h, 0.002);
	glEnd();
}

void DrawFrame(int w, int h)
{
	// frame
	glLineWidth(3.0);
	glColor3f(1.0, 1.0, 1.0);
	glBegin(GL_LINE_LOOP);
	glVertex3f(-1-.05/w, -1-.05/h, 0.002);
	glVertex3f(1+.05/w, -1-.05/h, 0.002);
	glVertex3f(1+.05/w, 1+.05/h, 0.002);
	glVertex3f(-1-.05/w, 1+.05/h, 0.002);
	glEnd();
	
	glColor3f(0.25, 0.25, 0.25);
	glBegin(GL_QUADS);
	glVertex3f(-1-.05/w, -1-.05/h, 0.003);
	glVertex3f(1+.05/w, -1-.05/h, 0.003);
	glVertex3f(1+.05/w, 1+.05/h, 0.003);
	glVertex3f(-1-.05/w, 1+.05/h, 0.003);
	glEnd();
	glLineWidth(1.0);
}

void MNPuzzle::OpenGLDraw(const MNPuzzleState &s) const
{
	glEnable(GL_LINE_SMOOTH);
	for (unsigned int y = 0; y < height; y++)
	{
		for (unsigned int x = 0; x < width; x++)
		{
			char c1=0, c2=0;
			if (s.puzzle[x+y*width] > 9)
				c1 = '0'+(((s.puzzle[x+y*width])/10)%10);
			if (s.puzzle[x+y*width] > 0)
				c2 = '0'+((s.puzzle[x+y*width])%10);
			DrawTile(x, y, c1, c2, width, height);
		}
	}
	DrawFrame(width, height);
}

void MNPuzzle::OpenGLDraw(const MNPuzzleState &s1, const MNPuzzleState &s2, float v) const
{
	glEnable(GL_LINE_SMOOTH);
	for (unsigned int y = 0; y < height; y++)
	{
		for (unsigned int x = 0; x < width; x++)
		{
			char c1=0, c2=0;
			if (s2.puzzle[x+y*width] > 9)
				c1 = '0'+(((s2.puzzle[x+y*width])/10)%10);
			if (s2.puzzle[x+y*width] > 0)
				c2 = '0'+((s2.puzzle[x+y*width])%10);

			if (s1.puzzle[x+y*width] == s2.puzzle[x+y*width])
			{
				DrawTile(x, y, c1, c2, width, height);
			}
			else {
				switch (GetAction(s1, s2))
				{
					case kUp: DrawTile(x, (y-1)*v + (y)*(1-v), c1, c2, width, height); break;
					case kDown: DrawTile(x, (y+1)*v + (y)*(1-v), c1, c2, width, height); break;
					case kLeft: DrawTile((x)*(1-v)+(x-1)*v, y, c1, c2, width, height); break;
					case kRight: DrawTile((x+1)*v+(x)*(1-v), y, c1, c2, width, height); break;
					default: assert(!"action not found");
				}
			}
		}
	}
	DrawFrame(width, height);
}

//double MNPuzzle::DoPDBLookup(const MNPuzzleState &state)
//{
//	double val = 0;
//	for (unsigned int x = 0; x < PDB.size(); x++)
//	{
//		uint64_t index = GetPDBHash(state, PDBkey[x]);
//		val = std::max(val, (double)PDB[x][index]);
//		//val += (double)PDB[x][index];
//	}
//	if (width == height) // symmetry
//	{
//	}
//	return val;
//}

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
int MNPuzzle::read_in_mn_puzzles(const char *filename, bool puzz_num_start, unsigned num_cols, unsigned num_rows, unsigned int max_puzzles, std::vector<MNPuzzleState> &puzzles)
{
	
	std::vector<std::vector<int> > permutations;
	Read_In_Permutations(filename, num_cols*num_rows, max_puzzles, permutations, puzz_num_start);
	
	// convert permutations into MNPuzzleStates
	for (unsigned i = 0; i < permutations.size(); i++)
	{
		MNPuzzleState new_state(num_cols, num_rows);
		
		for (unsigned j = 0; j < num_cols*num_rows; j++)
		{
			new_state.puzzle[j] = permutations[i][j];
			if (new_state.puzzle[j] == 0)
				new_state.blank = j;
		}
		puzzles.push_back(new_state);
	}
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

double GraphPuzzleDistanceHeuristic::HCost(const graphState &state1, const graphState &state2)
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
MNPuzzleState MNPuzzle::Generate_Random_Puzzle(unsigned num_cols, unsigned num_rows)
{
	MNPuzzleState new_puzz(num_cols, num_rows);
	std::vector<int> permutation = Get_Random_Permutation(num_cols*num_rows);
	
	for (unsigned i = 0; i < permutation.size(); i++)
	{
		new_puzz.puzzle[i] = permutation[i];
		if (permutation[i] == 0)
			new_puzz.blank = i;
	}
	
	return new_puzz;
}

void MNPuzzle::GetStateFromHash(MNPuzzleState &s, uint64_t hash)
{
	PermutationPuzzleEnvironment<MNPuzzleState,slideDir>::GetStateFromHash(s, hash);
	for (unsigned int x = 0; x < s.puzzle.size(); x++)
	{
		if (s.puzzle[x] == 0)
		{
			s.blank = x;
			return;
		}
	}
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
		MNPuzzleState next = Generate_Random_Puzzle(goal.width, goal.height);
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

bool MNPuzzle::State_Check(const MNPuzzleState &to_check)
{
	if (to_check.puzzle.size() != width*height)
		return false;
	
	if (to_check.width != width)
		return false;
	
	if (to_check.height != width)
		return false;
	
	if (to_check.blank >= width*height)
		return false;
	
	if (to_check.puzzle[to_check.blank] != 0)
		return false;
	
	return true;
}

std::vector<slideDir> MNPuzzle::Get_Op_Order_From_Hash(int order_num)
{
	std::vector<slideDir> ops;
	assert(order_num <= 23);
	assert(order_num >= 0);
	
	
	std::vector<int> op_nums(4);
	
	int num_left = 1;
	for (int x = 3; x >= 0; x--)
	{
		op_nums[x] = order_num % num_left;
		order_num /= num_left;
		num_left++;
		
		for (int y = x+1; y < 4; y++)
		{
			if (op_nums[y] >= op_nums[x])
			{
				op_nums[y]++;
			}
		}
	}
	
	bool up_act = false;
	bool down_act = false;
	bool left_act = false;
	bool right_act = false;
	
	for (unsigned i = 0; i < 4; i++)
	{
		if (op_nums[i] == 0)
		{
			ops.push_back(kUp);
			up_act = true;
		}
		else if (op_nums[i] == 1)
		{
			ops.push_back(kLeft);
			left_act = true;
		}
		else if (op_nums[i] == 2)
		{
			ops.push_back(kRight);
			right_act = true;
		}
		else if (op_nums[i] == 3)
		{
			ops.push_back(kDown);
			down_act = true;
		}
	}
	
	assert(up_act);
	assert(left_act);
	assert(right_act);
	assert(down_act);
	
	return ops;
}

