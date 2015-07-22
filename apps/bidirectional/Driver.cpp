/*
 * $Id: sample.cpp,v 1.23 2006/11/01 23:33:56 nathanst Exp $
 *
 *  sample.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Copyright 2005 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <cassert>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unordered_map>
#include <set>
#include <unordered_set>
#include <vector>
#include "Timer.h"
#include "RubiksCube.h"

struct hash128
{
	uint32_t parent;
	uint32_t cornerHash;
	uint64_t edgeHash;
};

bool operator< (const hash128 &left, const hash128 &right)
{
	if (left.cornerHash != right.cornerHash)
		return (left.cornerHash < right.cornerHash);
	return left.edgeHash < right.edgeHash;
}

bool operator==(const hash128 &left, const hash128 &right)
{
	return (left.cornerHash == right.cornerHash && left.edgeHash == right.edgeHash);
}


namespace std {
	template <> struct hash<hash128>
	{
		size_t operator()(const hash128 & x) const
		{
			return x.edgeHash^(uint64_t(x.cornerHash)<<40);
		}
	};
}


void GetInstanceFromStdin(RubiksState &start);
void WriteStatesToDisk(std::vector<hash128> &states, int depth);
void ExpandLayer(int depth);
bool DuplicateDetectLayer(int depthToRemove);
void ClearFiles();
void TestPruning(int depth, int bucket);
const int kNumBuckets = 512;
void BFS();

int main(int argc, char* argv[])
{
	if (argc > 1 && strcmp(argv[1], "-bfs") == 0)
	{
		BFS();
	}
	if (argc > 3 && strcmp(argv[1], "-testPruning") == 0)
	{
		TestPruning(atoi(argv[2]), atoi(argv[3]));
	}
}


void BFS()
{
	Timer t;
	t.StartTimer();
	ClearFiles();
	RubiksCube c;
	RubiksState s;
	//uint64_t start1 = strtoll(argv[0], 0, 10);
	GetInstanceFromStdin(s);
	//GetSuperFlip(s);
	hash128 start;
	start.parent = 20;
	start.edgeHash = c.GetEdgeHash(s);
	start.cornerHash = c.GetCornerHash(s);
	std::vector<hash128> states;
	states.push_back(start);
	WriteStatesToDisk(states, 0);

	// write goal to disk
	s.Reset();
	start.parent = 20;
	start.edgeHash = c.GetEdgeHash(s);
	start.cornerHash = c.GetCornerHash(s);
	states.clear();
	states.push_back(start);
	WriteStatesToDisk(states, 1);
	
	int depth = 2;
	bool done = false;
	while (!done)
	{
		Timer t2, t3;
		t2.StartTimer();
		t3.StartTimer();
		printf("***Starting layer: %d\n", depth);
		ExpandLayer(depth-2);
		printf("%1.2fs expanding\n", t2.EndTimer());
		t2.StartTimer();
		done = DuplicateDetectLayer(depth);
		printf("%1.2fs dd\n", t2.EndTimer());
		printf("%1.2fs total elapsed at depth %d\n", t3.EndTimer(), depth);
		depth++;
	}
	printf("%1.2fs elapsed; found solution at depth %d (cost %d)\n", t.EndTimer(), depth-1, depth-2);
}

const char *GetFileName(int depth, int bucket)
{
	static char fname[255];
	sprintf(fname, "bfs-d%d-b%d.dat", depth, bucket);
	return fname;
}

void ClearFiles()
{
	for (int depth = 0; depth < 20; depth++)
	{
		for (int bucket = 0; bucket < kNumBuckets; bucket++)
			remove(GetFileName(depth, bucket));
	}
}

void ConvertToBucketHash(const hash128 &s, uint64_t &bucket, uint64_t &hash)
{
	static uint64_t maxEdgeRank = RubikEdge().getMaxSinglePlayerRank();
	bucket = s.cornerHash%kNumBuckets;
	hash   = (((s.cornerHash/kNumBuckets)*maxEdgeRank + s.edgeHash)<<5) + s.parent;
}

void ConvertBucketHashToState(uint64_t bucket, uint64_t hash, hash128 &s)
{
	static uint64_t maxEdgeRank = RubikEdge().getMaxSinglePlayerRank();
	
	s.parent = hash&0x1F;
	hash >>= 5;
	s.edgeHash = hash%maxEdgeRank;
	s.cornerHash = uint32_t(hash/maxEdgeRank);
	s.cornerHash = s.cornerHash*kNumBuckets + uint32_t(bucket);

}

void WriteStatesToDisk(std::vector<hash128> &states, int depth)
{
	RubiksCorner c;
	RubikEdge e;

	std::vector<FILE *> files;
	files.resize(kNumBuckets);
	std::vector<uint64_t> counts;
	counts.resize(kNumBuckets);
	
	// 40 bits for edges
	// 27 bits for corners
	// 67 bits
	// 512 buckets (9 bits)
	// 58 bits total
	// 6 bits for extra information (parent)
	for (auto &s : states)
	{
		uint64_t bucket, hash;
		ConvertToBucketHash(s, bucket, hash);
//		uint64_t bucket = s.cornerHash%kNumBuckets;
//		uint64_t hash   = (s.cornerHash/kNumBuckets)*e.getMaxSinglePlayerRank() + s.edgeHash;
		if (files[bucket] == 0)
		{
			files[bucket] = fopen(GetFileName(depth, (int)bucket), "a");
			if (files[bucket] == 0)
			{
				printf("Can't open file %s\n", GetFileName(depth, (int)bucket));
			}
		}
//		if (depth < 2)
//			printf("%2d): Writing %llu %llu\n", depth, bucket, hash);
		fwrite(&hash, sizeof(uint64_t), 1, files[bucket]);
		counts[bucket]++;
	}
	for (int x = 0; x < files.size(); x++)
	{
		FILE *f = files[x];
		if (f)
		{
			fclose(f);
			//printf("%llu states written to depth %d bucket %d [%s]\n", counts[x], depth, x, GetFileName(depth, (int)x));
		}
	}
}

void ExpandLayer(int depth)
{
	RubikEdge e;
	RubiksCube c;
	RubiksState s;
	std::vector<RubiksAction> acts;
	std::vector<hash128> nextLevel;
	hash128 parent, child;
	uint64_t total = 0;
	for (int x = 0; x < kNumBuckets; x++)
	{
		FILE *f = fopen(GetFileName(depth, x), "r");
		if (f == 0)
			continue;
		//printf("-Expanding depth %d bucket %d [%s]\n", depth, x, GetFileName(depth, x));
		uint64_t next;
		uint64_t count = 0;
		while (fread(&next, sizeof(uint64_t), 1, f) == 1)
		{
			total++;
			count++;
			ConvertBucketHashToState(x, next, parent);
//			uint64_t cornerRank;
//			uint64_t edgeRank = next%e.getMaxSinglePlayerRank();
//			cornerRank = next/e.getMaxSinglePlayerRank();
//			cornerRank = cornerRank*kNumBuckets + x;
			//printf("%2d): Expanding %llu %llu\n", depth, x, next);
			c.GetStateFromHash(parent.cornerHash, parent.edgeHash, s);
			c.GetActions(s, acts);
			for (int a = 0; a < acts.size(); a++)
			{
				if (acts[a] == parent.parent)
					continue;
				c.ApplyAction(s, acts[a]);
				int toParent = acts[a];
				c.InvertAction(toParent);
				child.parent = toParent;
				child.edgeHash = c.GetEdgeHash(s);
				child.cornerHash = c.GetCornerHash(s);
				c.UndoAction(s, acts[a]);
				nextLevel.push_back(child);
			}
		}
		fclose(f);
		//printf("-Read %llu states from depth %d bucket %d\n", count, depth, x);
		WriteStatesToDisk(nextLevel, depth+2);
		nextLevel.clear();
	}
	printf("%llu states at depth %d\n", total, depth);
}

bool DuplicateDetectLayer(int depth)
{
	const int bufferSize = 128;
	
	double m0Dups = 0;
	double m2Dups = 0;
	double m4Dups = 0;
	double fDups = 0;
	double writeTime = 0;
	std::vector<uint64_t> values;
	values.resize(bufferSize);
	Timer t;
	uint64_t count = 0;
	bool dups = false;
	std::unordered_map<uint64_t, uint8_t> map;
	for (int x = 0; x < kNumBuckets; x++)
	{
		t.StartTimer();
		FILE *f = fopen(GetFileName(depth, x), "r");
		if (f == 0)
			continue;

		map.clear();
		count = 0;
		uint64_t numRead;
		while ((numRead = fread(&values[0], sizeof(uint64_t), bufferSize, f)) > 0)
		{
			for (int x = 0; x < numRead; x++)
				map[values[x]>>5] = values[x]&0x1F;
			count++;
		}
		fclose(f);
		m0Dups += t.EndTimer();
		//printf("Read %llu states from depth %d bucket %d [%s]\n", count, depth, x, GetFileName(depth, x));
		
		t.StartTimer();
		f = fopen(GetFileName(depth-2, x), "r");
		if (f != 0)
		{
			while ((numRead = fread(&values[0], sizeof(uint64_t), bufferSize, f)) > 0)
			{
				for (int x = 0; x < numRead; x++)
				{
					//printf("Looking for duplicate %d %llu\n", x, next);
					auto loc = map.find(values[x]>>5);
					if (loc != map.end())
					{
						//printf("Removing duplicate %d %llu\n", x, loc->first);
						map.erase(loc);
					}
				}
			}
			fclose(f);
		}
		m2Dups += t.EndTimer();

		t.StartTimer();
		f = fopen(GetFileName(depth-4, x), "r");
		if (f != 0)
		{
			while ((numRead = fread(&values[0], sizeof(uint64_t), bufferSize, f)) > 0)
			{
				for (int x = 0; x < numRead; x++)
				{
					auto loc = map.find(values[x]>>5);
					if (loc != map.end())
					{
						map.erase(loc);
					}
				}
			}
			fclose(f);
		}
		m4Dups += t.EndTimer();

		t.StartTimer();
		f = fopen(GetFileName(depth-1, x), "r");
		if (f != 0)
		{
			while ((numRead = fread(&values[0], sizeof(uint64_t), bufferSize, f)) > 0)
			{
				for (int x = 0; x < numRead; x++)
				{
					auto loc = map.find(values[x]>>5);
					if (loc != map.end())
					{
						//printf("Found duplicate!\n");
						dups = true;
					}
				}
			}
			fclose(f);
		}
		fDups += t.EndTimer();
		
		t.StartTimer();
		count = 0;
		f = fopen(GetFileName(depth, x), "w");
		if (f == 0)
		{
			printf("Error with %s\n", GetFileName(depth, x));
			exit(0);
		}
		values.resize(0);
		for (auto val : map)
		{
			//if (val.second)
			{
				values.push_back((val.first<<5)|(val.second));
				count++;
				//printf("%2d): Writing %llu %llu\n", depth, x, val.first);
				
				if (values.size() >= bufferSize)
				{
					fwrite(&(values[0]), sizeof(uint64_t), values.size(), f);
					values.resize(0);
				}
			}
		}
		if (values.size() > 0)
			fwrite(&(values[0]), sizeof(uint64_t), values.size(), f);
		//printf("Wrote %llu states to depth %d bucket %d [%s]\n", count, depth, x, GetFileName(depth, x));
		fclose(f);
		writeTime += t.EndTimer();
	}
	printf("%1.2f dup vs 0, %1.2f dup vs -2, %1.2f dup vs -4, %1.2f dup vs other frontier, %1.2f write\n",
		   m0Dups, m2Dups, m4Dups, fDups, writeTime);
	return dups;
}













int GetNextMove(char *input, int &base)
{
	int used = 0;
	if (isdigit(input[0])) // this is our move notation - numeric
	{
		int curr = 0;
		base = input[curr]-'0';
		curr++;
		if (isdigit(input[curr]))
		{
			base = base*10+input[curr]-'0';
			curr++;
		}
		while (!isdigit(input[curr]) && input[curr] != '\n' && input[curr] != 0)
		{
			curr++;
		}
		return curr;
	}
	switch (input[0])
	{
		case 'F': used = 1; base = 2*3; break;
		case 'B': used = 1; base = 3*3; break;
		case 'L': used = 1; base = 4*3; break;
		case 'R': used = 1; base = 5*3; break;
		case 'U': used = 1; base = 0*3; break;
		case 'D': used = 1; base = 1*3; break;
		default: break;
	}
	if (used == 0)
		return 0;
	if (input[0] != 'U')
	{
		bool stillGoing = true;
		int offset = 1;
		while (stillGoing)
		{
			switch (input[offset++])
			{
				case ' ': used++; break;
				case '\n': stillGoing = false; break;
				case '2': base += 2; used++; break;
				case '-': base += 1; used++; break;
				default: stillGoing = false; break;
			}
		}
	}
	else {
		base++;
		bool stillGoing = true;
		int offset = 1;
		while (stillGoing)
		{
			switch (input[offset++])
			{
				case ' ': used++; break;
				case '\n': stillGoing = false; break;
				case '2': base += 1; used++; break;
				case '-': base -= 1; used++; break;
				default: stillGoing = false; break;
			}
		}
	}
	return used;
}

void GetSuperFlip(RubiksState &start)
{
	RubiksCube c;
	const int maxStrLength = 1024;
	char string[maxStrLength] = "U R2 F B R B2 R U2 L B2 R U- D- R2 F R- L B2 U2 F2";

	start.Reset();
	
	int index = 0;
	string[maxStrLength-1] = 0;
	if (strlen(string) == maxStrLength-1)
	{
		printf("Warning: input hit maximum string length!\n");
		exit(0);
	}
	while (true)
	{
		int act;
		int cnt = GetNextMove(&string[index], act);
		if (cnt == 0)
		{
			break;
		}
		else {
			index += cnt;
		}
		c.ApplyAction(start, act);
	}
}

void GetInstanceFromStdin(RubiksState &start)
{
	RubiksCube c;
	
	const int maxStrLength = 1024;
//	char string[maxStrLength] =
//	"L2 B  D- L- F- B R- F- B R-";
//	char string[maxStrLength] =
//	"L2 B  D- L- F- B  R2 F  B  R- F- R2 B- L2 D2 L2 D2 L2 U2 L2 F- D  L- D2 L- F2 B2 L  U- D- L  B  D2 F  D- F- U- B  L2 D2 L2 R2";
	char string[maxStrLength] =
	"B2 D- B- U- R- D- B- U2 L- R- B2 U2 B2 L- U  B- D  F  L2 F2 D  F- L- D- B2 L- U2 F  B2 R2 D  L- U  D2 F  B- L2 B  R- U B- L  B2 D  F  R  U- D- F  R2 U2 L- B2 L- R- D- L2 R- F2 D L- D  B2 D  L  B- R- D  B- L2 B2 D2 F  B2 U2 R- D- L- B2 R- D- L2 F- D- R  U  F  L2 D- R- U- L2 B  U- F2 U  B- D  F2 D2";
//	char string[maxStrLength] =
//	"L2 B  D- L- F- B  R2 F  B  R- F- R2 B- L2 D2 L2 D2 L2 U2 L2 F- D  L- D2 L- F2 B2 L  U- D- L  B  D2 F  D- F- U- B  L2 D2 L2 R2 B- U  R- D2 F  R- B  L R  U- B- R2 F- L2 R  F  R2 B L- F- D- F2 U2 R  U- L  D  F2 B- R- D- L2 B- L- B2 L- D2 B2 D- B  D  R- B  D  L- B- R  F- L- F- R2 D2 L2 B- L2 B2 U  L2";
	// 	const char *result = fgets(string, maxStrLength, stdin);

	start.Reset();
	
//	if (result == 0)
//	{
//		printf("No more entries found; exiting.\n");
//		exit(0);
//	}
	int index = 0;
	string[maxStrLength-1] = 0;
	if (strlen(string) == maxStrLength-1)
	{
		printf("Warning: input hit maximum string length!\n");
		exit(0);
	}
	while (true)
	{
		int act;
		int cnt = GetNextMove(&string[index], act);
		if (cnt == 0)
		{
			break;
		}
		else {
			index += cnt;
		}
		c.ApplyAction(start, act);
	}
}


void TestPruning(int depth, int bucket)
{
	RubikEdge e;
	RubiksCube c;
	RubiksState s;
	std::vector<uint64_t> depths(17);
	for (int x = 0; x < kNumBuckets; x++)
	{
		FILE *f = fopen(GetFileName(depth, x), "r");
		if (f == 0)
			continue;
		//printf("-Expanding depth %d bucket %d [%s]\n", depth, x, GetFileName(depth, x));
		uint64_t next;
		while (fread(&next, sizeof(uint64_t), 1, f) == 1)
		{
			uint64_t cornerRank;
			uint64_t edgeRank = next%e.getMaxSinglePlayerRank();
			cornerRank = next/e.getMaxSinglePlayerRank();
			cornerRank = cornerRank*kNumBuckets + x;
			//printf("%2d): Expanding %llu %llu\n", depth, x, next);
			c.GetStateFromHash(cornerRank, edgeRank, s);
			
			depths[c.Edge12PDBDist(s)]++;
		}
		fclose(f);
		//printf("-Read %llu states from depth %d bucket %d\n", count, depth, x);
	}
	for (int x = 0; x < depths.size(); x++)
	{
		printf("%d\t%llu\n", x, depths[x]);
	}
}