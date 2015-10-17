//
//  PDBRankingTest.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/14/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#include "PDBRankingTest.h"
#include "MR1PermutationPDB.h"
#include "PancakePuzzle.h"

void FullRankingTest()
{
	PancakePuzzle p(4);
	PancakePuzzleState s(4);
	std::vector<int> pattern = {0, 1, 2, 3};
	MR1PermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> pdb(&p, s, pattern);
	
	for (int x = 0; x < pdb.GetPDBSize(); x++)
	{
		pdb.GetStateFromPDBHash(x, s);
		uint64_t hash = pdb.GetPDBHash(s);
		std::cout << x << ": " << s << "(" << hash << ")\n";
		if (hash != x)
		{
			std::cout << "Error: " << s << " came from rank " << x << " but ranked back to " << pdb.GetPDBHash(s) << "\n";
			exit(0);
		}
	}
}

void PDBTest()
{
	PancakePuzzle p(10);
	PancakePuzzleState s(10);
	std::vector<int> pattern = {0, 9, 3, 2};
	MR1PermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> pdb(&p, s, pattern);
	
	for (int x = 0; x < pdb.GetPDBSize(); x++)
	{
		pdb.GetStateFromPDBHash(x, s);
		uint64_t hash = pdb.GetPDBHash(s);
		std::cout << x << ": " << s << "(" << hash << ")\n";
		if (hash != x)
		{
			std::cout << "Error: " << s << " came from rank " << x << " but ranked back to " << pdb.GetPDBHash(s) << "\n";
			exit(0);
		}
	}

}

void JointTest()
{
	PancakePuzzle p(6);
	PancakePuzzleState s(6);
	PancakePuzzleState s2(6);
	std::vector<int> pattern = {0, 1, 2, 3, 4, 5};
	std::vector<int> pattern2 = {3, 1, 4};
	MR1PermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> pdb(&p, s, pattern);
	MR1PermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> pdb2(&p, s, pattern2);
	
	for (int x = 0; x < pdb.GetPDBSize(); x++)
	{
		pdb.GetStateFromPDBHash(x, s);
		uint64_t hash = pdb2.GetPDBHash(s);
		pdb2.GetStateFromPDBHash(hash, s2);
		std::cout << x << ": " << s << "\n";
		std::cout << "   " << hash << ": " << s2 << "\n";
		for (int x = 0; x < s.puzzle.size(); x++)
		{
			if (s2.puzzle[x] != -1 && s2.puzzle[x] != s.puzzle[x])
			{
				std::cout << "Full and pdb state don't match " << s << " " << s2 << "\n";
				exit(0);
			}
		}
	}
}

void PDBRankingTest()
{
	printf("Full Rank\n");
	FullRankingTest();
	printf("\n\nPDB Rank\n");
	PDBTest();
	printf("\nMix PDB/Full Rank\n");
	JointTest();
}
