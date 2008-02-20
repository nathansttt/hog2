/*
 *  MNPuzzle.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/9/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "MNPuzzle.h"

MNPuzzle::MNPuzzle(int _width, int _height)
:width(_width), height(_height)
{
}

MNPuzzle::~MNPuzzle()
{
}

void MNPuzzle::GetSuccessors(MNPuzzleState &stateID, std::vector<MNPuzzleState> &neighbors)
{
	//std::cout << stateID << std::endl;
	neighbors.resize(0);
	if ((stateID.blank%stateID.width) != stateID.width-1)
	{
		neighbors.push_back(stateID);
		ApplyAction(neighbors.back(), kRight);
		//std::cout << neighbors.back() << std::endl;
	}
	if ((stateID.blank%stateID.width) != 0)
	{
		neighbors.push_back(stateID);
		ApplyAction(neighbors.back(), kLeft);
		//std::cout << neighbors.back() << std::endl;
	}

	if ((stateID.blank/stateID.width) != stateID.height-1)
	{
		neighbors.push_back(stateID);
		ApplyAction(neighbors.back(), kDown);
		//std::cout << neighbors.back() << std::endl;
	}
	if ((stateID.blank/stateID.width) != 0)
	{
		neighbors.push_back(stateID);
		ApplyAction(neighbors.back(), kUp);
		//std::cout << neighbors.back() << std::endl;
	}
}

void MNPuzzle::GetActions(MNPuzzleState &stateID, std::vector<slideDir> &actions)
{
	actions.resize(0);
	if ((stateID.blank%stateID.width) != stateID.width-1)
		actions.push_back(kRight);
	if ((stateID.blank%stateID.width) != 0)
		actions.push_back(kLeft);
	if ((stateID.blank/stateID.width) != stateID.height-1)
		actions.push_back(kDown);
	if ((stateID.blank/stateID.width) != 0)
		actions.push_back(kUp);
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

double MNPuzzle::HCost(MNPuzzleState &state1, MNPuzzleState &state2)
{
	assert(state1.height==state2.height);
	assert(state1.width==state2.width);
	std::vector<int> xloc(state2.width*state2.height);
	std::vector<int> yloc(state2.width*state2.height);
	double hval = 0;

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

/**
 * Tiles are the tiles we care about
 */
uint64_t MNPuzzle::GetPDBHash(MNPuzzleState &state, const std::vector<int> &tiles)
{
	std::vector<int> locs;
	locs.resize(tiles.size());
	for (unsigned int x = 0; x < state.puzzle.size(); x++)
	{
		for (unsigned int y = 0; y < tiles.size(); y++)
		{
			if (state.puzzle[x] == tiles[y])
				locs[y] = x;
		}
	}

	uint64_t hashVal = 0;
	int numEntriesLeft = state.puzzle.size();
	for (unsigned int x = 0; x < locs.size(); x++)
	{
		hashVal += locs[x]*Factorial(numEntriesLeft-1)/Factorial(state.puzzle.size()-tiles.size());
		numEntriesLeft--;
		for (unsigned y = x; y < locs.size(); y++)
		{
			if (locs[y] > locs[x])
				locs[y]--;
		}
	}
	assert(hashVal*Factorial(state.puzzle.size()-tiles.size()) < Factorial(state.puzzle.size()));
	return hashVal;
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
	
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
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
	for (int y = 0; y <= height; y++)
	{
		for (int x = 0; x <= width; x++)
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
}

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
