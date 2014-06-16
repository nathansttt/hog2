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
#include <limits.h>
#include <xlocale.h>
#include "Common.h"
#include "Driver.h"
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include "IDAStar.h"
#include "TemplateAStar.h"
#include "Timer.h"
#include "RubiksCube.h"
#include "DiskBitFile.h"
#include "RubiksCube7Edges.h"
#include "RubiksCubeCorners.h"
#include "BFS.h"
#include "BloomFilter.h"
#include <string>
#include "BitVector.h"
#include "MinBloom.h"

RubiksCube c;
RubiksAction a;
RubiksState s;

Rubik7Edge e7;
Rubik7EdgeState e7s;
Rubik7EdgeAction e7a;

void TestMinBloom();
void GetInstanceFromStdin(RubiksState &start);

bool readFromStdin = false;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 300);
}


/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
void CreateSimulation(int id)
{
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');

	InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a randomly moving unit", kShiftDown, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, 1);

	InstallCommandLineHandler(MyCLHandler, "-test", "-test entries", "Test using 'entries' billion entries from edge pdb");

	InstallCommandLineHandler(MyCLHandler, "-buildBloom", "-buildBloom <size> <#GB> <#hash> <dataloc>", "Build a bloom filter using a size/hash combo.");
	InstallCommandLineHandler(MyCLHandler, "-buildMinBloom", "-buildMinBloom <#GB> <#hash> <maxDepth> <dataloc>", "Build a bloom filter using a size/hash combo.");
	InstallCommandLineHandler(MyCLHandler, "-showStats", "-showStats <size> <#hash> <prefix>", "Print out bloom filter stats.");
	
	InstallCommandLineHandler(MyCLHandler, "-bloomSample", "-bloomSample <corner-prefix> <other-prefix> <8size> <8hash> <9size> <9hash>", "Use bloom filter + corner pdb. Pass data locations");
	InstallCommandLineHandler(MyCLHandler, "-bloomSearch", "-bloomSearch <corner-prefix> <other-prefix> <8size> <8hash> <9size> <9hash>", "Use bloom filter + corner pdb. Pass data locations");
	InstallCommandLineHandler(MyCLHandler, "-minBloomSearch", "-minBloomSearch <corner-prefix> <other-prefix> <8size> <8hash> <9size> <9hash>", "Use bloom filter + corner pdb. Pass data locations");
	InstallCommandLineHandler(MyCLHandler, "-measure", "-measure interleave", "Measure loss from interleaving versus min");
	InstallCommandLineHandler(MyCLHandler, "-extract", "-extract <file>", "Extract levels from <file>");
	InstallCommandLineHandler(MyCLHandler, "-testBloom", "-testBloom <entires> <accuracy>", "Test bloom filter with <entries> total and given <accuracy>");
	InstallCommandLineHandler(MyCLHandler, "-testCompression", "-testCompression <factor> <type> <edgepdb> <cornerpdb>", "");
	InstallCommandLineHandler(MyCLHandler, "-compress", "-compress <type [corner,n-edge,edge]> <input> <factor> <output>", "Compress provided pdb by a factor of <factor>");
	InstallCommandLineHandler(MyCLHandler, "-pdb", "-pdb <edge> <corner>", "Run tests using edge and corner pdbs");
	
	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler);
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		//CreateSimulation(windowID);
		SetNumPorts(windowID, 4);
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	glClearColor(1.0, 1.0, 1.0, 1.0);
	if (viewport == 3)
	{
		c.OpenGLDrawEdgeDual(s);
	}
	else if (viewport == 2)
	{
		c.OpenGLDrawEdges(s);
	}
	else if (viewport == 1)
	{
		//e7.OpenGLDraw(e7s);
		c.OpenGLDrawCorners(s);
	}
	else {
		c.OpenGLDraw(s);
	}
}

void RunTest(int billionEntriesToLoad);
void MeasurePDBCompression(int itemsCompressed);
void Compress(const char *pdbType, const char *theFile, const char *compresType, int ratio, const char *outFile);
//void RunCompressionTest(int factor, const char *compType, const char *edgePDB, const char *cornerPDB);
void RunCompressionTest(int factor, const char *compType, const char *edgePDBmin, const char *edgePDBint, const char *cornerPDB);
void RunSimpleTest(const char *edgePDB, const char *cornerPDB);
void TestBloom(int entries, double accuracy);
void TestBloom2(int entries, double accuracy);
void ExtractStatesAtDepth(const char *theFile);
void SampleBloomHeuristics(const char *cornerPDB, const char *depthPrefix, float size8, int hash8, float size9, int hash9);
void RunBloomFilterTest(const char *cornerPDB, const char *depthPrefix, float size8, int hash8, float size9, int hash9);
void RunMinBloomFilterTest(const char *cornerPDB, const char *depthPrefix, float space, int numHash);
void ManyCompression();
void BuildDepthBloomFilter(int size, float space, int numHash, const char *dataLoc);
void GetBloomStats(uint64_t size, int hash, const char *prefix);
void BuildMinBloomFilter(float space, int numHash, int finalDepth, const char *dataLoc);

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs < 1)
		return 0;

	if (strcmp(argument[0], "-test") == 0)
	{
		RunTest(atoi(argument[1]));
	}
	else if (strcmp(argument[0], "-extract") == 0)
	{
		ExtractStatesAtDepth(argument[1]);
		exit(0);
	}
	else if (strcmp(argument[0], "-measure") == 0)
	{
		MeasurePDBCompression(atoi(argument[1]));
	}
	else if (strcmp(argument[0], "-testBloom") == 0)
	{
		TestBloom(atoi(argument[1]), atof(argument[2]));
		TestBloom2(atoi(argument[1]), atof(argument[2]));
		exit(0);
	}
	else if (strcmp(argument[0], "-buildBloom") == 0)
	{
		BuildDepthBloomFilter(atoi(argument[1]), atof(argument[2]), atoi(argument[3]), argument[4]);
		exit(0);
	}
	else if (strcmp(argument[0], "-buildMinBloom") == 0)
	{
		BuildMinBloomFilter(atof(argument[1]), atoi(argument[2]), atoi(argument[3]), argument[4]);
		exit(0);
	}
	else if (strcmp(argument[0], "-bloomSearch") == 0)
	{
		RunBloomFilterTest(argument[1], argument[2], atof(argument[3]), atoi(argument[4]), atof(argument[5]), atoi(argument[6]));
		exit(0);
	}
	else if (strcmp(argument[0], "-bloomSample") == 0)
	{
		SampleBloomHeuristics(argument[1], argument[2], atof(argument[3]), atoi(argument[4]), atof(argument[5]), atoi(argument[6]));
		exit(0);
	}
	else if (strcmp(argument[0], "-minBloomSearch") == 0)
	{
		RunMinBloomFilterTest(argument[1], argument[2], atof(argument[3]), atoi(argument[4]));
		exit(0);
	}
	else if (strcmp(argument[0], "-pdb") == 0)
	{
		RunSimpleTest(argument[1], argument[2]);
		exit(0);
	}
	else if (strcmp(argument[0], "-compress") == 0)
	{
		ManyCompression(); exit(0);
		if (maxNumArgs < 5)
		{
			printf("Insufficient number of arguments\n");
			exit(0);
		}
		//InstallCommandLineHandler(MyCLHandler, "-compress", "-compress <e> <file> <factor>", "Compress provided pdb by a factor of <factor>");
		Compress(argument[1], argument[2], argument[3], atoi(argument[4]), argument[5]);
		exit(0);
	}
	else if (strcmp(argument[0], "-testCompression") == 0)
	{
		RunCompressionTest(atoi(argument[1]), argument[2], argument[3], argument[4], argument[5]);
		exit(0);
	}
	else if (strcmp(argument[0], "-showStats"))
	{
		GetBloomStats(strtoull(argument[1], 0, 10), strtol(argument[2], 0, 10), argument[3]);
	}
	//	strncpy(gDefaultMap, argument[1], 1024);
	return 2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
//		case '6':
//		case '7':
//		case '8':
			//e7.GetStateFromHash(e7.GetStateHash(e7s), e7s);
			//printf("Old hash is %llu. ", c.GetStateHash(s));
			c.ApplyAction(s, (key-'0')*3);
			e7.ApplyAction(e7s, (key-'0')*3);
			//printf("New is %llu\n", c.GetStateHash(s));
			printf("rank is %llu\n", e7.GetStateHash(e7s));
			break;
		case '9':
		{
			static int a = 0;
			e7.GetStateFromHash(a++, e7s);
//			int a = random()%18;
//			c.ApplyAction(s, a);
//			e7.ApplyAction(e7s, a);
			printf("rank is %llu\n", e7.GetStateHash(e7s));
		}
			break;
		case '\t':
			if (mod != kShiftDown)
				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
			else
			{
				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
			}
			break;
		case 'p':
			s.Reset();
			printf("Resetting state\n");
			break;
		case 'o':
		{
			Rubik7EdgeState es7, dual7;
			Rubik7Edge e7;

			RubikEdgeState es, dual;
			RubikEdge e;

			static std::vector<RubikEdgeAction> acts;
			for (int t = 0; t < 5; t++)
				acts.push_back(random()%18);
			es7.Reset();
			es.Reset();
			e.GetStateFromHash(12345, es);
			es7.state = es.state;
			std::cout << "Strt: " << es7 << std::endl;
			std::cout << "Strt: " << es << std::endl;
			std::cout << "Acts: ";
			for (unsigned int x = 0; x < acts.size(); x++)
			{
				if (x > 0)
					std::cout << ", ";
				e7.ApplyAction(es7, acts[x]);
				e.ApplyAction(es, acts[x]);
				std::cout << acts[x];
			}
			std::cout << std::endl;;
			std::cout << "Init: " << es7 << std::endl;
			std::cout << "Init: " << es << std::endl;
			es7.GetDual(dual7);
			es.GetDual(dual);
			std::cout << "Dual: " << dual7 << std::endl;
			std::cout << "Dual: " << dual << std::endl;
			es7.Reset();
			es.Reset();
			e.GetStateFromHash(12345, es);
			es7.state = es.state;
			for (unsigned int x = 0; x < acts.size(); x++)
			{
				e7.UndoAction(es7, acts[acts.size()-x-1]);
				e.UndoAction(es, acts[acts.size()-x-1]);
			}
			std::cout << "Undo: " << es7 << std::endl;
			std::cout << "Undo: " << es << std::endl;
			//std::cout << dual << std::endl << std::endl;
			assert(dual7 == es7);
			assert(dual == es);
			for (unsigned int x = 0; x < acts.size(); x++)
			{
				e7.ApplyAction(es7, acts[x]);
				e.ApplyAction(es, acts[x]);
			}
			std::cout << es7 << std::endl << std::endl;
			std::cout << es << std::endl << std::endl;
		}
			break;
		case ']':
			s.Reset();
			for (int x = 0; x < 5; x++)
				c.ApplyAction(s, x);
			break;
		case '[':
			s.Reset();
			for (int x = 4; x >= 0; x--)
				c.ApplyAction(s, x);
			break;
//		case '{': unitSim->setPaused(true); unitSim->offsetDisplayTime(-0.5); break;
//		case '}': unitSim->offsetDisplayTime(0.5); break;
		default:
			//if (unitSim)
			//	unitSim->GetEnvironment()->GetMapAbstraction()->ToggleDrawAbstraction(((mod == kControlDown)?10:0)+(key-'0'));
			break;
	}
}

void Compress(const char *pdbType, const char *theFile, const char *compressType, int ratio, const char *outFile)
{
	bool minCompression = false;
	if (strcmp(compressType, "interleave") == 0)
	{
		minCompression = false;
	}
	else if (strcmp(compressType, "min") == 0)
	{
		minCompression = true;
	}
	else {
		printf("Unknown compression '%s'\n", compressType);
		exit(0);
	}
	if (strcmp(pdbType, "corner") == 0)
	{
		FourBitArray b;
		
		std::vector<bucketInfo> data;
		std::vector<bucketData> buckets;
		
		uint64_t maxBuckSize = GetMaxBucketSize<RubiksCorner, RubiksCornerState>(true);
		InitTwoPieceData<RubiksCorner, RubiksCornerState>(data, maxBuckSize);
		InitBucketSize<RubiksCorner, RubiksCornerState>(buckets, maxBuckSize);
		int64_t totalSize = 0;
		for (unsigned int x = 0; x < buckets.size(); x++)
		{
			totalSize += buckets[x].theSize;
		}
		totalSize = (totalSize+ratio-1)/ratio;
		b.Resize(totalSize);
		
		DiskBitFile f(theFile);

		uint64_t avgReg = 0;
		uint64_t avgComp = 0;

		if (!minCompression)
		{ // just write first  value
			printf("Performing interleave compression\n");
			int64_t index = 0;
			int64_t realIndex = 0;
			for (unsigned int x = 0; x < data.size(); x++)
			{
				for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
				{
					int val = f.ReadFileDepth(data[x].bucketID, y);
					avgReg += val;
					//mem[index++] = val;
					assert(realIndex < b.Size());
					if (0 == index%ratio)
					{
						b.Set(realIndex++, val);
						avgComp += val;
					}
					index++;
				}
			}
			b.Write(outFile);
			printf("%llu entries compressed into %llu\n", index, realIndex);
			printf("Original average: %f; Compressed average: %f\n", (float)avgReg/index, (float)avgComp/realIndex);
		}
		else { // write the min of the values
			printf("Performing min compression\n");
			int64_t index = 0;
			int64_t realIndex = 0;
			int minValue = 0xFF;
			for (unsigned int x = 0; x < data.size(); x++)
			{
				for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
				{
					int val = f.ReadFileDepth(data[x].bucketID, y);
					avgReg += val;
					assert(realIndex < b.Size());
					index++;
					minValue = std::min(minValue, val);
					if (0 == index%ratio)
					{
						b.Set(realIndex++, minValue);
						avgComp += minValue;
						minValue = 0xFF;
					}
				}
			}
			if (minValue != 0xFF)
			{
				b.Set(realIndex++, minValue);
			}
			b.Write(outFile);
			printf("%llu entries compressed into %llu\n", index, realIndex);
			printf("Original average: %f; Compressed average: %f\n", (float)avgReg/index, (float)avgComp/realIndex);
		}
	}
	if (strcmp(pdbType, "n-edge") == 0)
	{
		FourBitArray b;
		
		std::vector<bucketInfo> data;
		std::vector<bucketData> buckets;
		
		uint64_t maxBuckSize = GetMaxBucketSize<Rubik7Edge, Rubik7EdgeState>(true);
		InitTwoPieceData<Rubik7Edge, Rubik7EdgeState>(data, maxBuckSize);
		InitBucketSize<Rubik7Edge, Rubik7EdgeState>(buckets, maxBuckSize);
		int64_t totalSize = 0;
		for (unsigned int x = 0; x < buckets.size(); x++)
		{
			totalSize += buckets[x].theSize;
		}
		totalSize = (totalSize+ratio-1)/ratio;
		b.Resize(totalSize);
		
		DiskBitFile f(theFile);
		
		uint64_t avgReg = 0;
		uint64_t avgComp = 0;
		
		if (!minCompression)
		{ // just write first  value
			printf("Performing interleave compression\n");
			int64_t index = 0;
			int64_t realIndex = 0;
			for (unsigned int x = 0; x < data.size(); x++)
			{
				for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
				{
					int val = f.ReadFileDepth(data[x].bucketID, y);
					avgReg += val;
					//mem[index++] = val;
					assert(realIndex < b.Size());
					if (0 == index%ratio)
					{
						b.Set(realIndex++, val);
						avgComp += val;
					}
					index++;
				}
			}
			b.Write(outFile);
			printf("%llu entries compressed into %llu\n", index, realIndex);
			printf("Original average: %f; Compressed average: %f\n", (float)avgReg/index, (float)avgComp/realIndex);
		}
		else { // write the min of the values
			printf("Performing min compression\n");
			int64_t index = 0;
			int64_t realIndex = 0;
			int minValue = 0xFF;
			for (unsigned int x = 0; x < data.size(); x++)
			{
				for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
				{
					int val = f.ReadFileDepth(data[x].bucketID, y);
					avgReg += val;
					assert(realIndex < b.Size());
					index++;
					minValue = std::min(minValue, val);
					if (0 == index%ratio)
					{
						b.Set(realIndex++, minValue);
						avgComp += minValue;
						minValue = 0xFF;
					}
				}
			}
			if (minValue != 0xFF)
			{
				b.Set(realIndex++, minValue);
			}
			b.Write(outFile);
			printf("%llu entries compressed into %llu\n", index, realIndex);
			printf("Original average: %f; Compressed average: %f\n", (float)avgReg/index, (float)avgComp/realIndex);
		}
	}
	if (strcmp(pdbType, "edge") == 0)
	{
		FourBitArray b;
		
		std::vector<bucketInfo> data;
		std::vector<bucketData> buckets;
		
		uint64_t maxBuckSize = GetMaxBucketSize<RubikEdge, RubikEdgeState>(true);
		InitTwoPieceData<RubikEdge, RubikEdgeState>(data, maxBuckSize);
		InitBucketSize<RubikEdge, RubikEdgeState>(buckets, maxBuckSize);
		int64_t totalSize = 0;
		for (unsigned int x = 0; x < buckets.size(); x++)
		{
			totalSize += buckets[x].theSize;
		}
		totalSize = (totalSize+ratio-1)/ratio;
		b.Resize(totalSize);
		
		DiskBitFile f(theFile);

		uint64_t avgReg = 0;
		uint64_t avgComp = 0;
		
		float currPerc = 1;
		if (!minCompression)
		{ // just write first  value
			printf("Performing interleave compression\n");
			int64_t index = 0;
			int64_t realIndex = 0;
			for (unsigned int x = 0; x < data.size(); x++)
			{
				for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
				{
					int val = f.ReadFileDepth(data[x].bucketID, y);
					avgReg += val;
					//mem[index++] = val;
					assert(realIndex < b.Size());
					if (0 == index%ratio)
					{
						b.Set(realIndex++, val);
						avgComp += val;
					}
					index++;
					if (float(realIndex)/float(totalSize) > currPerc)
					{
						printf("%1.0f%% ", currPerc);
						currPerc++;
					}
				}
			}
			printf("\n");
			b.Write(outFile);
			printf("%llu entries compressed into %llu\n", index, realIndex);
			printf("Original average: %f; Compressed average: %f\n", (float)avgReg/index, (float)avgComp/realIndex);
		}
		else { // write the min of the values
			printf("Performing min compression\n");
			int64_t index = 0;
			int64_t realIndex = 0;
			int minValue = 0xFF;
			for (unsigned int x = 0; x < data.size(); x++)
			{
				for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
				{
					int val = f.ReadFileDepth(data[x].bucketID, y);
					avgReg += val;
					assert(realIndex < b.Size());
					index++;
					minValue = std::min(minValue, val);
					if (0 == index%ratio)
					{
						b.Set(realIndex++, minValue);
						avgComp += minValue;
						minValue = 0xFF;
					}
				}
			}
			if (minValue != 0xFF)
			{
				b.Set(realIndex++, minValue);
			}
			b.Write(outFile);
			printf("%llu entries compressed into %llu\n", index, realIndex);
			printf("Original average: %f; Compressed average: %f\n", (float)avgReg/index, (float)avgComp/realIndex);
		}
	}
}

void ManyCompression()
{
	const int toCompress = 3;
	FourBitArray b[toCompress];
	int factor[toCompress] = {15, 20, 25};//{ 30, 35, 40, 45, 50 }; //10, 15, 20, 25,
	std::vector<bucketInfo> data;
	std::vector<bucketData> buckets;
	
	uint64_t maxBuckSize = GetMaxBucketSize<RubikEdge, RubikEdgeState>(true);
	InitTwoPieceData<RubikEdge, RubikEdgeState>(data, maxBuckSize);
	InitBucketSize<RubikEdge, RubikEdgeState>(buckets, maxBuckSize);
	int64_t totalSize = 0;
	for (unsigned int x = 0; x < buckets.size(); x++)
	{
		totalSize += buckets[x].theSize;
	}
	//totalSize = (totalSize+ratio-1)/ratio;
	for (int x = 0; x < toCompress; x++)
	{
		b[x].Resize((totalSize+factor[x]-1)/factor[x]);
	}
	//b.Resize(totalSize);
	
	DiskBitFile f("/data/cc/rubik/res/RC");

	
	printf("Performing min compression\n"); fflush(stdout);
	uint64_t avgReg = 0;
	uint64_t avgComp[toCompress];//(0);// = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	int64_t realIndex[toCompress];// = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	int64_t index = 0;
	int minValue[toCompress];// = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	for (int x = 0; x < toCompress; x++)
	{
		avgComp[x] = 0; realIndex[x] = 0;
		minValue[x] = 0xFF;
	}
	for (unsigned int x = 0; x < data.size(); x++)
	{
		for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
		{
			int val = f.ReadFileDepth(data[x].bucketID, y);
			avgReg += val;
			index++;
			for (int r = 0; r < toCompress; r++)
			{
				minValue[r] = std::min(minValue[r], val);
				if (0 == index%factor[r])
				{
					b[r].Set(realIndex[r]++, minValue[r]);
					avgComp[r] += minValue[r];
					minValue[r] = 0xFF;
				}
			}
		}
	}
	for (int r = 0; r < toCompress; r++)
	{
		if (minValue[r] != 0xFF)
		{
			b[r].Set(realIndex[r]++, minValue[r]);
		}
		char name[255];
		sprintf(name, "edge-min-%dx.pdb", factor[r]);
		b[r].Write(name);
		printf("%dx compression:\n", factor[r]);
		printf("%llu entries compressed into %llu\n", index, realIndex[r]);
		printf("Original average: %f; Compressed average: %f\n\n", (float)avgReg/index, (float)avgComp[r]/realIndex[r]);
	}

}

void LoadCornerPDB()
{
	if (c.GetCornerPDB().Size() != 0)
		return;
	FourBitArray &b = c.GetCornerPDB();
	
	//uint8_t *mem;
	std::vector<bucketInfo> data;
	std::vector<bucketData> buckets;
	
	uint64_t maxBuckSize = GetMaxBucketSize<RubiksCorner, RubiksCornerState>(true);
	InitTwoPieceData<RubiksCorner, RubiksCornerState>(data, maxBuckSize);
	InitBucketSize<RubiksCorner, RubiksCornerState>(buckets, maxBuckSize);
	int64_t totalSize = 0;
	for (unsigned int x = 0; x < buckets.size(); x++)
		totalSize += buckets[x].theSize;
	b.Resize(totalSize);
	//mem = new uint8_t[totalSize];
	DiskBitFile f("/home/sturtevant/sturtevant/code/cc/rubik/RC");
	
	int64_t index = 0;
	for (unsigned int x = 0; x < data.size(); x++)
	{
		for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
		{
			int val = f.ReadFileDepth(data[x].bucketID, y);
			//mem[index++] = val;
			b.Set(index++, val);
		}
	}
	//c.SetCornerPDB(mem);
}

void LoadEdgePDB(uint64_t sizeLimit)
{
	if (c.GetEdgePDB().Size() != 0)
		return;
	FourBitArray &b = c.GetEdgePDB();

	//uint8_t *mem;
	std::vector<bucketInfo> data;
	std::vector<bucketData> buckets;
	
	uint64_t maxBuckSize = GetMaxBucketSize<RubikEdge, RubikEdgeState>(true);
	InitTwoPieceData<RubikEdge, RubikEdgeState>(data, maxBuckSize);
	InitBucketSize<RubikEdge, RubikEdgeState>(buckets, maxBuckSize);
	int64_t totalSize = 0;
	for (unsigned int x = 0; x < buckets.size(); x++)
		totalSize += buckets[x].theSize;

	b.Resize((totalSize+sizeLimit-1)/sizeLimit);
	//DiskBitFile f("/data/cc/rubik/final/RC");
	DiskBitFile f("/store/rubik/RC");
	int64_t index = 0;
	for (unsigned int x = 0; x < data.size(); x++)
	{
		for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
		{
		  if (0 == y%sizeLimit)
		    {
		      int val = f.ReadFileDepth(data[x].bucketID, y);
		      b.Set(index++, val);
		    }
		}
	}
//	c.SetEdgePDB(mem, totalSize);
}

void LoadEdgePDBOLD(uint64_t sizeLimit)
{
	//
	if (c.GetEdgePDB().Size() != 0)
		return;
	FourBitArray &b = c.GetEdgePDB();

	//uint8_t *mem;
	std::vector<bucketInfo> data;
	std::vector<bucketData> buckets;
	
	uint64_t maxBuckSize = GetMaxBucketSize<RubikEdge, RubikEdgeState>(true);
	InitTwoPieceData<RubikEdge, RubikEdgeState>(data, maxBuckSize);
	InitBucketSize<RubikEdge, RubikEdgeState>(buckets, maxBuckSize);
	int64_t totalSize = 0;
//	const int64_t sizeLimit = 1000000000;
	for (unsigned int x = 0; x < buckets.size(); x++)
		totalSize += buckets[x].theSize;
	if (totalSize > sizeLimit)
		totalSize = sizeLimit;
	//mem = new uint8_t[totalSize];
	b.Resize(totalSize);
	DiskBitFile f("/data/cc/rubik/final/RC");
	//DiskBitFile f("/store/rubik/RC");
	int64_t index = 0;
	for (unsigned int x = 0; x < data.size(); x++)
	{
		for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
		{
			int val = f.ReadFileDepth(data[x].bucketID, y);
			b.Set(index++, val);
			//mem[index++] = val;
			if (index >= totalSize)
				break;
		}
		if (index >= totalSize)
			break;
	}
//	c.SetEdgePDB(mem, totalSize);
}

void LoadEdge7PDB(uint64_t sizeLimit = 0x7FFFFFFFFFFFFFFFull);

void LoadEdge7PDB(uint64_t sizeLimit)
{
//	if (c.GetEdge7PDB().Size() != 0)
//		return;
//	FourBitArray &b = c.GetEdge7PDB();
//	
//	//uint8_t *mem;
//	std::vector<bucketInfo> data;
//	std::vector<bucketData> buckets;
//	
//	uint64_t maxBuckSize = GetMaxBucketSize<Rubik7Edge, Rubik7EdgeState>(true);
//	InitTwoPieceData<Rubik7Edge, Rubik7EdgeState>(data, maxBuckSize);
//	InitBucketSize<Rubik7Edge, Rubik7EdgeState>(buckets, maxBuckSize);
//	int64_t totalSize = 0;
//	//	const int64_t sizeLimit = 1000000000;
//	for (unsigned int x = 0; x < buckets.size(); x++)
//	  totalSize += buckets[x].theSize;
//	//mem = new uint8_t[totalSize];
//	b.Resize(totalSize);
//	DiskBitFile f("/home/sturtevant/sturtevant/code/cc/rubik/RC-7edge");//RC-9edge");
//	int64_t index = 0;
//	for (unsigned int x = 0; x < data.size(); x++)
//	{
//		for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
//		{
//			int val = f.ReadFileDepth(data[x].bucketID, y);
//			b.Set(index++, val);
//			//mem[index++] = val;
//			if (index >= totalSize)
//				break;
//		}
//		if (index >= totalSize)
//			break;
//	}
//	//	c.SetEdgePDB(mem, totalSize);
}

void MyRandomUnitKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	std::vector<RubiksAction> acts;
	c.SetPruneSuccessors(true);
	while (true)
	{
		c.GetActions(s, acts);
		for (unsigned int x = 0; x < acts.size(); x++)
			printf("%d) %d\n", x, acts[x]);
		printf("Choose action to apply: ");
		int which;
		std::cin >> which;
		c.ApplyAction(s, acts[which]);
	}
}

void GetInstance(RubiksState &start, int which);
void SolveOneProblem(int instance, const char *txt)
{
	RubiksState start, goal;
	goal.Reset();
	start.Reset();
	c.SetPruneSuccessors(true); // clears history

	if (readFromStdin)
	{
		GetInstanceFromStdin(start);
	}
	else {
		GetInstance(start, instance);
	}
	
	std::vector<RubiksAction> acts;
	c.SetPruneSuccessors(true);
//	c.percentage = percentage;
	s = start;
	IDAStar<RubiksState, RubiksAction> ida;
	c.SetPruneSuccessors(true);
	Timer t;
	t.StartTimer();
	ida.SetUseBDPathMax(true);
	ida.GetPath(&c, start, goal, acts);
	t.EndTimer();
	printf("[%s] Problem %d - %llu expanded; %1.2f elapsed\n", txt, instance+1, ida.GetNodesExpanded(), t.GetElapsedTime());
	for (unsigned int x = 0; x < acts.size(); x++)
	{
		printf("%d ", acts[x]);
		c.ApplyAction(s, acts[x]);
	}
	printf("\n");
	fflush(stdout);
}

void SolveOneProblemAStar(int instance)
{
	RubiksState start, goal;
	goal.Reset();
	start.Reset();
	c.SetPruneSuccessors(false); // clears history

	if (readFromStdin)
	{
		GetInstanceFromStdin(start);
	}
	else {
		GetInstance(start, instance);
	}
	
	std::vector<RubiksState> acts;
	c.SetPruneSuccessors(true);
	//	c.percentage = percentage;
	s = start;
	TemplateAStar<RubiksState, RubiksAction, RubiksCube> astar;
	Timer t;
	t.StartTimer();
	astar.SetUseBPMX(1);
	astar.GetPath(&c, start, goal, acts);
	t.EndTimer();
	printf("[AStar] Problem %d - %llu expanded; %1.2f elapsed\n", instance+1, astar.GetNodesExpanded(), t.GetElapsedTime());
	for (unsigned int x = 0; x < acts.size()-1; x++)
	{
		printf("%d ", c.GetAction(acts[x], acts[x+1]));
//		c.ApplyAction(s, acts[x]);
	}
	printf("\n");
	fflush(stdout);
}


void MyPathfindingKeyHandler(unsigned long , tKeyboardModifier , char)
{
	s.Reset();
	GetInstanceFromStdin(s);
	//TestMinBloom();
	//	LoadCornerPDB();
//	//LoadEdge7PDB();
//	LoadEdgePDB(10);
//	//LoadEdgePDBOLD(2000000000);
//
//	for (int x = 3; x < 4; x++)
//	{
//		srandom(9283+x*23);
//		SolveOneProblem(x, "online");
//	}
//	
//	LoadCornerPDB();
	//	static int t = 0;
//	c.GetStateFromHash(t, s);
//	t++;
//	return;
}

void RunSimpleTest(const char *edgePDB, const char *cornerPDB)
{
	FourBitArray &corner = c.GetCornerPDB();
	corner.Read(cornerPDB);
	FourBitArray &edge = c.GetEdgePDB();
	edge.Read(edgePDB);
	c.compressionFactor = 2;
	for (int x = 0; x < 100; x++)
	{
		srandom(9283+x*23);
		SolveOneProblem(x, "2x");
		//		SolveOneProblemAStar(x);
	}
}

int countBits(int64_t val)
{
	int count = 0;
	while (val)
	{
		if (val&1)
			count++;
		val >>= 1;
	}
	return count;
}

void BuildDepthBloomFilter(int size, float space, int numHash, const char *dataLoc)
{
	printf("Creating bloom filter using %2.1f GB of mem.\n", space);fflush(stdout);
	space = space*8*1024*1024*1024;
	uint64_t depth9states = 11588911021ull;
	
	BloomFilter *bf = new BloomFilter(space, numHash, true, true);
	printf("Approximate storage (%d): %llu bits (%1.2f MB / %1.2f GB)\n", size, bf->GetStorage(),
		   bf->GetStorage()/8.0/1024.0/1024.0,
		   bf->GetStorage()/8.0/1024.0/1024.0/1024.0);
	printf("%d hashes being used\n", bf->GetNumHash());
	
	printf("Building hash table/bloom filter\n"); fflush(stdout);
	RubikEdge e;
	RubikEdgeState es;
	
	int x = size;
	
	char name[255];
	sprintf(name, "%s12edge-depth-%d.dat", dataLoc, x);
	FILE *f = fopen(name, "r");
	if (f == 0)
	{
		printf("Error opening %s; aborting!\n", name);
		exit(0);
	}
	printf("Reading from '%s'\n", name);
	fflush(stdout);
	
	uint64_t nextItem;
	uint64_t count = 0;
	while (fread(&nextItem, sizeof(uint64_t), 1, f) == 1)
	{
		if (0 != countBits(nextItem&0xFFF)%2)
			nextItem ^= 1;
		//es.state = nextItem;
		//nextItem = e.GetStateHash(es);
		
		count++;
		bf->Insert(nextItem);
		
		if (0 == count%100000000ull)
		{
			printf("%llu added to table\n", count);
			fflush(stdout);
		}
	}
	printf("%llu items read at depth %d\n", count, x);fflush(stdout);
	fclose(f);
	bf->Analyze();
	delete bf;
}

void BuildMinBloomFilter(float space, int numHash, int finalDepth, const char *dataLoc)
{
	printf("Creating bloom filter using %2.1f GB of mem.\n", space);fflush(stdout);
	space = space*8*1024*1024*1024;
				
	MinBloomFilter *bf = new MinBloomFilter(space, numHash, true, true);
	printf("Approximate storage: %llu bits (%1.2f MB / %1.2f GB)\n", bf->GetStorage(),
		   bf->GetStorage()*4.0/8.0/1024.0/1024.0,
		   bf->GetStorage()*4.0/8.0/1024.0/1024.0/1024.0);
	printf("%d hashes being used\n", bf->GetNumHash());

	printf("Building hash table/bloom filter\n"); fflush(stdout);
	RubikEdge e;
	RubikEdgeState es;
	
	for (int x = 0; x < finalDepth; x++)
	{
		char name[255];
		sprintf(name, "%s12edge-depth-%d.dat", dataLoc, x);
		FILE *f = fopen(name, "r");
		if (f == 0)
		{
			printf("Error opening %s; aborting!\n", name);
			exit(0);
		}
		printf("Reading from '%s'\n", name);
		fflush(stdout);
		
		uint64_t nextItem;
		uint64_t count = 0;
		while (fread(&nextItem, sizeof(uint64_t), 1, f) == 1)
		{
			if (0 != countBits(nextItem&0xFFF)%2)
				nextItem ^= 1;
			//es.state = nextItem;
			//nextItem = e.GetStateHash(es);
			
			count++;
			bf->Insert(nextItem, x);
			
			if (0 == count%100000000ull)
			{
				printf("%llu added to table\n", count);
				fflush(stdout);
			}
		}
		printf("%llu items read at depth %d\n", count, x);fflush(stdout);
		fclose(f);
	}
	bf->Analyze();
	delete bf;
}

void SampleBloomHeuristics(const char *cornerPDB, const char *depthPrefix, float size8, int hash8, float size9, int hash9)
{
	// setup corner pdb
	{
		FourBitArray &corner = c.GetCornerPDB();
		printf("Loading corner pdb: %s\n", cornerPDB);
		corner.Read(cornerPDB);
		printf("Done.\n");
	}
	
	bool zero = false;
	
	// setup bloom filters
	{
		printf("Loading bloom filters.\n");fflush(stdout);
		//uint64_t depth8states = 1050559626ull;
		//uint64_t depth9states = 11588911021ull;
		
		uint64_t size8filter = size8*1024*1024*1024*8ull;
		uint64_t size9filter = size9*1024ull*1024ull*1024ull*8ull;
		
		printf("Loading filter with %d GB of entries (%llu) and %d hashes\n", size8, size8filter, hash8);
		c.depth8 = new BloomFilter(size8filter, hash8, depthPrefix);
		printf("Approximate storage (%d): %llu bits (%1.2f MB / %1.2f GB)\n",
			   8, c.depth8->GetStorage(),
			   c.depth8->GetStorage()/8.0/1024.0/1024.0,
			   c.depth8->GetStorage()/8.0/1024.0/1024.0/1024.0);
		printf("%d hashes being used\n", c.depth8->GetNumHash());
		
		printf("Loading filter with %d GB of entries (%llu) and %d hashes\n", size9, size9filter, hash9);
		c.depth9 = new BloomFilter(size9filter, hash9, depthPrefix);
		printf("Approximate storage (%d): %llu bits (%1.2f MB / %1.2f GB)\n",
			   9, c.depth9->GetStorage(),
			   c.depth9->GetStorage()/8.0/1024.0/1024.0,
			   c.depth9->GetStorage()/8.0/1024.0/1024.0/1024.0);
		printf("%d hashes being used\n", c.depth9->GetNumHash());
		
		c.bloomFilter = true;
		printf("Done.\n");fflush(stdout);
	}
	
	// load hash table / bloom filter data
	if (1)
	{
		printf("Building hash table/bloom filter\n"); fflush(stdout);
		RubikEdge e;
		RubikEdgeState es;
		for (int x = 0; x < 8; x++)
		{
			char name[255];
			sprintf(name, "%s12edge-depth-%d.dat", depthPrefix, x);
			FILE *f = fopen(name, "r");
			if (f == 0)
			{
				printf("Error opening %s; aborting!\n", name);
				exit(0);
			}
			printf("Reading from '%s'\n", name);
			fflush(stdout);
			
			uint64_t nextItem;
			uint64_t count = 0;
			while (fread(&nextItem, sizeof(uint64_t), 1, f) == 1)
			{
				if (0 != countBits(nextItem&0xFFF)%2)
					nextItem ^= 1;
				if (x < 8)
				{
					count++;
					c.depthTable[nextItem] = x;
				}
				else {
					printf("Error; hit depth %d\n", x);
				}
			}
			printf("%llu items read at depth %d\n", count, x);fflush(stdout);
			fclose(f);
		}
		
	}
	
	RubiksState start;
	start.Reset();
	RubikEdge edge;
	uint64_t max = edge.getMaxSinglePlayerRank();
	double val = 0;
	int cnt = 100000;
	for (int x = 0; x < cnt; x++)
	{
		uint64_t r1 = random(), r2 = random();
		r1 = (r1<<32)|(r2);
		r1 = r1%max;
		edge.GetStateFromHash(r1, start.edge);
		val += c.HCost(start, start);
//		srandom(9283+x*23);
//		SolveOneProblem(x, "hash7bloom89");
//		//		SolveOneProblemAStar(x);
	}
	printf("Edge heuristic distribution\n");
	for (int x = 0; x < c.edgeDist.size(); x++)
	{
		printf("%d : %llu\n", x, c.edgeDist[x]);
	}
	printf("Average: %1.4f\n", val/cnt);
	//	::std::tr1::unordered_map<uint64_t, uint8_t> hash;
	//	bloom_filter *depth8, *depth9;
	
}

void RunBloomFilterTest(const char *cornerPDB, const char *depthPrefix, float size8, int hash8, float size9, int hash9)
{
	// setup corner pdb
	{
		FourBitArray &corner = c.GetCornerPDB();
		printf("Loading corner pdb: %s\n", cornerPDB);
		corner.Read(cornerPDB);
		printf("Done.\n");
	}

	bool zero = false;

	// setup bloom filters
	{
		printf("Loading bloom filters.\n");fflush(stdout);
		//uint64_t depth8states = 1050559626ull;
		//uint64_t depth9states = 11588911021ull;
		
		uint64_t size8filter = size8*1024*1024*1024*8ull;
		uint64_t size9filter = size9*1024ull*1024ull*1024ull*8ull;

		printf("Loading filter with %d GB of entries (%llu) and %d hashes\n", size8, size8filter, hash8);
		c.depth8 = new BloomFilter(size8filter, hash8, depthPrefix);
		printf("Approximate storage (%d): %llu bits (%1.2f MB / %1.2f GB)\n",
			   8, c.depth8->GetStorage(),
			   c.depth8->GetStorage()/8.0/1024.0/1024.0,
			   c.depth8->GetStorage()/8.0/1024.0/1024.0/1024.0);
		printf("%d hashes being used\n", c.depth8->GetNumHash());

		printf("Loading filter with %d GB of entries (%llu) and %d hashes\n", size9, size9filter, hash9);
		c.depth9 = new BloomFilter(size9filter, hash9, depthPrefix);
		printf("Approximate storage (%d): %llu bits (%1.2f MB / %1.2f GB)\n",
			   9, c.depth9->GetStorage(),
			   c.depth9->GetStorage()/8.0/1024.0/1024.0,
			   c.depth9->GetStorage()/8.0/1024.0/1024.0/1024.0);
		printf("%d hashes being used\n", c.depth9->GetNumHash());

		c.bloomFilter = true;
		printf("Done.\n");fflush(stdout);
	}
	
	// load hash table / bloom filter data
	if (1)
	{
		printf("Building hash table/bloom filter\n"); fflush(stdout);
		RubikEdge e;
		RubikEdgeState es;
		for (int x = 0; x < 8; x++)
		{
			char name[255];
			sprintf(name, "%s12edge-depth-%d.dat", depthPrefix, x);
			FILE *f = fopen(name, "r");
			if (f == 0)
			{
				printf("Error opening %s; aborting!\n", name);
				exit(0);
			}
			printf("Reading from '%s'\n", name);
			fflush(stdout);

			uint64_t nextItem;
			uint64_t count = 0;
			while (fread(&nextItem, sizeof(uint64_t), 1, f) == 1)
			{
				if (0 != countBits(nextItem&0xFFF)%2)
					nextItem ^= 1;
				if (x < 8)
				{
					count++;
					c.depthTable[nextItem] = x;
				}
				else {
					printf("Error; hit depth %d\n", x);
				}
			}
			printf("%llu items read at depth %d\n", count, x);fflush(stdout);
			fclose(f);
		}

	}
	
	for (int x = 0; x < 100; x++)
	{
		srandom(9283+x*23);
		SolveOneProblem(x, "hash7bloom89");
		//		SolveOneProblemAStar(x);
	}
	printf("Edge heuristic distribution\n");
	for (int x = 0; x < c.edgeDist.size(); x++)
	{
		printf("%d : %llu\n", x, c.edgeDist[x]);
	}
//	::std::tr1::unordered_map<uint64_t, uint8_t> hash;
//	bloom_filter *depth8, *depth9;

}

void RunMinBloomFilterTest(const char *cornerPDB, const char *depthPrefix, float space, int numHash)
{
	// setup corner pdb
	{
		FourBitArray &corner = c.GetCornerPDB();
		printf("Loading corner pdb: %s\n", cornerPDB);
		corner.Read(cornerPDB);
		printf("Done.\n");
	}
	
	bool zero = false;
	
	// setup bloom filters
	{
		printf("Loading min bloom filters.\n");fflush(stdout);
		
		printf("Creating bloom filter using %2.1f GB of mem.\n", space);fflush(stdout);
		space = space*8*1024*1024*1024;
		
		MinBloomFilter *bf = new MinBloomFilter(space, numHash, depthPrefix);
		printf("Approximate storage: %llu bits (%1.2f MB / %1.2f GB)\n", bf->GetStorage(),
			   bf->GetStorage()*4.0/8.0/1024.0/1024.0,
			   bf->GetStorage()*4.0/8.0/1024.0/1024.0/1024.0);
		printf("%d hashes being used\n", bf->GetNumHash());
		c.minBloom = bf;
		c.minBloomFilter = true;
	}
	
	for (int x = 0; x < 100; x++)
	{
		srandom(9283+x*23);
		SolveOneProblem(x, "hash7bloom89");
		//		SolveOneProblemAStar(x);
	}
	printf("Edge heuristic distribution\n");
	for (int x = 0; x < c.edgeDist.size(); x++)
	{
		printf("%d : %llu\n", x, c.edgeDist[x]);
	}
	//	::std::tr1::unordered_map<uint64_t, uint8_t> hash;
	//	bloom_filter *depth8, *depth9;
	
}


void RunCompressionTest(int factor, const char *compType, const char *edgePDBmin, const char *edgePDBint, const char *cornerPDB)
{
	FourBitArray &corner = c.GetCornerPDB();
	printf("Loading corner pdb: %s\n", cornerPDB);
	corner.Read(cornerPDB);
	FourBitArray &edgemin = c.GetEdgePDB();
	printf("Loading edge min pdb: %s\n", edgePDBmin);
	edgemin.Read(edgePDBmin);
	FourBitArray &edgeint = c.GetEdge7PDB(false);
	//printf("Loading edge interleave pdb: %s\n", edgePDBint);
	//edgeint.Read(edgePDBint);

	c.compressionFactor = factor;
	if (strcmp(compType, "min") == 0)
	{
		printf("Using min compression (%dx)\n", factor);
		c.minCompression = true;
	}
	else {
		printf("Using interleave compression (%dx)\n", factor);
		c.minCompression = false;
	}

	for (int x = 0; x < 100; x++)
	{
		for (int t = 0; t <= 0; t++)
		{
			if (t == 0)
			{
				printf("Solving with min compression\n");
				c.minCompression = true;
			}
			else {
				printf("Solving with interleave compression\n");
				c.minCompression = false;
			}
			{
				srandom(9283+x*23);
				SolveOneProblem(x, (t==0)?"min":"int");
				//SolveOneProblemAStar(x);
			}
		}
	}
//	for (int x = 0; x < c.edgeDist.size(); x++)
//	{
//		printf("edgeDist[%3d] = %llu\n", x, c.edgeDist[x]);
//	}
//	for (int x = 0; x < c.cornDist.size(); x++)
//	{
//		printf("cornDist[%3d] = %llu\n", x, c.cornDist[x]);
//	}
}

void RunTest(int billionEntriesToLoad)
{
//	uint64_t numEntries = 1000000000;
//	numEntries *= billionEntriesToLoad;
//	LoadCornerPDB();
//	//LoadEdgePDB(numEntries);
//	LoadEdge7PDB();
//	
//	for (int t = 100; t >= 80; t--)
//	{
//		for (int x = 0; x < 100; x++)
//		{
//			srandom(9283+x*23);
//			SolveOneProblem(x, t);
//		}
//	}
}

bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	return false;
}

int GetNextMove(char *input, int &base)
{
	int used = 0;
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

void GetInstanceFromStdin(RubiksState &start)
{
	const int maxStrLength = 1024;
	char string[maxStrLength];
	char *result = fgets(string, maxStrLength, stdin);

	start.Reset();
	
	if (result == 0)
	{
		printf("No more entries found; exiting.\n");
		exit(0);
	}
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
		c.ApplyAction(s, act);
	}
}

void GetInstance(RubiksState &start, int which)
{
	int instances[100][14] = {
	{4, 10, 5, 10, 17, 6, 1, 11, 0, 10, 16, 9, 14, 4},
	{15, 0, 12, 16, 1, 6, 17, 8, 17, 8, 2, 11, 4, 15},
	{6, 12, 2, 16, 9, 13, 8, 2, 15, 11, 4, 8, 4, 16},
	{9, 13, 16, 1, 14, 0, 11, 13, 4, 10, 12, 6, 5, 15},
	{1, 9, 4, 6, 0, 4, 15, 5, 12, 0, 14, 9, 4, 16},
	{5, 8, 4, 8, 5, 9, 5, 11, 0, 7, 11, 12, 8, 17},
	{14, 11, 4, 14, 5, 6, 4, 10, 1, 4, 15, 9, 13, 7},
	{17, 1, 4, 10, 4, 16, 4, 16, 8, 13, 15, 2, 4, 17},
	{7, 12, 11, 4, 15, 1, 17, 2, 5, 14, 9, 5, 8, 10},
	{10, 13, 15, 1, 16, 3, 7, 9, 17, 4, 9, 17, 6, 15},
	{0, 5, 10, 16, 8, 14, 7, 2, 6, 1, 4, 12, 8, 13},
	{4, 6, 5, 17, 7, 4, 17, 9, 17, 3, 9, 3, 17, 3},
	{15, 7, 13, 17, 11, 3, 10, 4, 12, 10, 0, 16, 2, 9},
	{6, 10, 16, 2, 4, 7, 0, 14, 16, 9, 2, 17, 2, 9},
	{9, 16, 0, 5, 16, 7, 1, 6, 0, 3, 10, 2, 12, 11},
	{1, 7, 3, 10, 2, 10, 2, 8, 4, 11, 13, 17, 5, 7},
	{5, 11, 16, 4, 9, 13, 4, 15, 8, 15, 0, 11, 12, 7},
	{14, 17, 11, 4, 8, 3, 10, 4, 10, 0, 5, 7, 13, 3},
	{17, 8, 16, 11, 14, 5, 13, 2, 16, 10, 3, 12, 0, 5},
	{8, 9, 2, 3, 11, 15, 0, 17, 9, 2, 13, 10, 3, 17},
	{10, 17, 2, 17, 10, 2, 11, 16, 8, 16, 10, 5, 16, 1},
	{0, 6, 17, 1, 15, 9, 4, 15, 3, 14, 1, 11, 1, 3},
	{4, 7, 17, 9, 17, 9, 0, 9, 17, 3, 13, 5, 15, 11},
	{13, 15, 1, 5, 16, 6, 15, 6, 1, 10, 2, 14, 3, 7},
	{6, 16, 1, 16, 3, 9, 1, 13, 8, 12, 7, 4, 9, 14},
	{9, 13, 2, 5, 16, 5, 9, 17, 7, 4, 6, 9, 14, 3},
	{1, 13, 11, 5, 15, 3, 7, 12, 5, 7, 12, 5, 14, 0},
	{5, 17, 10, 4, 9, 4, 13, 1, 5, 6, 0, 6, 3, 17},
	{14, 8, 10, 13, 11, 12, 6, 11, 0, 5, 12, 0, 14, 1},
	{17, 10, 0, 11, 16, 9, 5, 6, 17, 8, 3, 11, 13, 5},
	{8, 17, 10, 15, 4, 8, 0, 15, 1, 16, 10, 13, 15, 6},
	{10, 14, 15, 6, 13, 5, 13, 0, 3, 11, 5, 7, 5, 6},
	{0, 12, 11, 17, 9, 1, 11, 16, 7, 13, 5, 14, 17, 5},
	{4, 15, 11, 3, 17, 7, 13, 6, 17, 7, 12, 0, 5, 6},
	{13, 16, 2, 3, 11, 14, 2, 11, 17, 7, 2, 13, 8, 1},
	{6, 12, 3, 14, 8, 5, 12, 5, 17, 4, 14, 1, 12, 17},
	{9, 13, 7, 9, 5, 7, 2, 3, 6, 2, 15, 2, 13, 1},
	{1, 17, 7, 9, 14, 0, 3, 11, 15, 10, 0, 4, 15, 5},
	{3, 9, 2, 17, 5, 10, 17, 10, 1, 12, 9, 3, 8, 4},
	{14, 4, 15, 11, 5, 15, 11, 0, 14, 10, 1, 10, 12, 11},
	{17, 4, 11, 13, 4, 14, 9, 13, 11, 17, 1, 6, 16, 0},
	{8, 5, 8, 15, 6, 4, 15, 11, 14, 7, 4, 7, 4, 9},
	{10, 14, 8, 3, 15, 6, 5, 17, 3, 12, 11, 16, 6, 2},
	{0, 15, 3, 17, 0, 9, 3, 14, 16, 2, 9, 14, 0, 4},
	{4, 16, 5, 7, 2, 9, 4, 12, 6, 17, 2, 13, 7, 14},
	{13, 6, 9, 3, 16, 8, 9, 13, 15, 4, 12, 0, 15, 3},
	{6, 13, 6, 5, 7, 4, 11, 15, 10, 13, 3, 8, 10, 2},
	{9, 16, 0, 6, 1, 9, 12, 7, 9, 5, 16, 5, 15, 4},
	{1, 4, 9, 3, 8, 4, 8, 4, 6, 14, 5, 14, 16, 3},
	{3, 14, 17, 11, 13, 4, 8, 15, 6, 15, 8, 4, 10, 2},
	{14, 17, 0, 13, 9, 3, 17, 10, 3, 8, 3, 17, 3, 16},
	{17, 5, 8, 15, 2, 9, 3, 17, 6, 17, 0, 4, 13, 4},
	{8, 14, 2, 17, 7, 10, 5, 13, 0, 9, 16, 7, 17, 0},
	{11, 15, 2, 11, 17, 11, 4, 17, 1, 11, 12, 3, 14, 1},
	{0, 3, 15, 0, 7, 10, 3, 12, 6, 17, 9, 0, 7, 1},
	{4, 12, 15, 7, 3, 7, 15, 7, 13, 3, 15, 11, 5, 10},
	{13, 1, 6, 11, 16, 3, 14, 5, 16, 1, 9, 0, 16, 0},
	{6, 3, 7, 2, 15, 6, 2, 15, 6, 11, 17, 7, 0, 16},
	{9, 4, 13, 9, 5, 13, 11, 17, 9, 15, 11, 3, 14, 9},
	{1, 14, 9, 14, 4, 9, 1, 16, 10, 16, 11, 13, 17, 1},
	{3, 11, 13, 15, 9, 16, 8, 13, 10, 15, 0, 11, 5, 15},
	{14, 5, 11, 0, 13, 6, 1, 11, 3, 9, 14, 8, 10, 12},
	{17, 2, 9, 14, 11, 12, 10, 4, 9, 14, 0, 5, 17, 6},
	{8, 2, 14, 10, 0, 17, 7, 5, 8, 12, 7, 12, 10, 3},
	{11, 3, 14, 7, 0, 6, 13, 0, 6, 15, 5, 7, 3, 12},
	{0, 12, 9, 3, 8, 9, 14, 15, 0, 4, 6, 5, 10, 17},
	{4, 9, 16, 8, 1, 4, 12, 4, 15, 2, 4, 6, 3, 15},
	{13, 3, 12, 1, 7, 0, 5, 8, 17, 10, 17, 5, 9, 17},
	{16, 4, 12, 9, 4, 13, 9, 1, 8, 2, 3, 12, 10, 0},
	{9, 1, 7, 4, 14, 2, 13, 8, 4, 13, 5, 11, 13, 10},
	{1, 7, 13, 16, 10, 16, 8, 5, 11, 0, 14, 8, 5, 7},
	{3, 15, 8, 13, 3, 10, 3, 12, 16, 11, 17, 0, 5, 12},
	{14, 10, 5, 10, 2, 13, 17, 7, 16, 7, 0, 3, 8, 0},
	{17, 2, 8, 12, 2, 16, 5, 10, 0, 17, 5, 16, 10, 3},
	{8, 5, 8, 0, 7, 3, 12, 10, 12, 6, 17, 3, 16, 7},
	{11, 17, 6, 11, 2, 6, 4, 14, 0, 11, 17, 0, 6, 13},
	{0, 3, 8, 2, 9, 17, 5, 7, 12, 8, 0, 14, 4, 16},
	{4, 14, 7, 5, 11, 13, 4, 6, 1, 15, 1, 13, 1, 7},
	{13, 9, 16, 7, 12, 9, 4, 12, 8, 15, 1, 4, 11, 17},
	{16, 1, 6, 14, 15, 8, 16, 4, 11, 13, 2, 7, 1, 3},
	{9, 0, 14, 5, 6, 1, 15, 2, 12, 4, 7, 17, 10, 16},
	{1, 7, 0, 13, 1, 14, 0, 14, 7, 13, 9, 0, 6, 1},
	{3, 10, 17, 11, 1, 15, 2, 8, 11, 16, 5, 14, 16, 7},
	{12, 2, 17, 8, 2, 6, 10, 0, 12, 6, 12, 0, 11, 12},
	{17, 7, 1, 6, 9, 15, 9, 5, 9, 14, 15, 7, 10, 3},
	{8, 11, 2, 17, 6, 1, 7, 11, 3, 6, 10, 16, 3, 14},
	{11, 1, 14, 8, 0, 6, 5, 8, 2, 4, 14, 9, 3, 17},
	{2, 8, 15, 7, 4, 16, 8, 5, 16, 0, 10, 13, 2, 6},
	{4, 9, 15, 5, 12, 1, 15, 1, 5, 10, 5, 13, 5, 9},
	{13, 1, 10, 16, 3, 13, 9, 15, 0, 14, 3, 6, 16, 8},
	{16, 6, 10, 13, 4, 12, 3, 14, 11, 15, 0, 9, 12, 17},
	{9, 13, 9, 1, 13, 4, 17, 6, 4, 16, 4, 14, 1, 10},
	{1, 10, 13, 4, 16, 8, 10, 17, 5, 13, 8, 0, 15, 9},
	{3, 16, 11, 2, 8, 16, 4, 9, 5, 16, 11, 16, 5, 12},
	{12, 7, 11, 15, 11, 17, 6, 0, 16, 0, 17, 9, 15, 4},
	{17, 11, 12, 10, 2, 13, 2, 7, 2, 12, 4, 16, 0, 7},
	{8, 17, 8, 5, 14, 11, 14, 17, 8, 11, 1, 12, 7, 4},
	{11, 14, 8, 4, 11, 4, 7, 5, 9, 12, 10, 1, 14, 16},
	{2, 9, 16, 2, 5, 14, 10, 16, 2, 11, 0, 5, 6, 2},
	{4, 17, 9, 1, 12, 5, 17, 8, 11, 13, 15, 6, 12, 4}
	};

	for (int x = 13; x >= 0; x--)
	{
		c.UndoAction(start, instances[which][x]);
		printf("%d ", instances[which][x]);
	}
	printf("\n");
}

void MeasurePDBCompression(int itemsCompressed)
{
	std::vector<bucketInfo> data;
	std::vector<bucketData> buckets;
	
	uint64_t maxBuckSize = GetMaxBucketSize<RubikEdge, RubikEdgeState>(true);
	InitTwoPieceData<RubikEdge, RubikEdgeState>(data, maxBuckSize);
	InitBucketSize<RubikEdge, RubikEdgeState>(buckets, maxBuckSize);
	int64_t totalSize = 0;

	//DiskBitFile f("/data/cc/rubik/final/RC");
	DiskBitFile f("/store/rubik/RC");
	int64_t index = 0;
	int64_t records[20];
	int64_t averageFirst[20];
	int64_t averageMin[20];
	int64_t averageMax[20];
	int64_t averageValue[20];

	int tempSum[20];
	int firstVal[20];
	int minVal[20];
	int maxVal[20];
	for (int x = 0; x < 20; x++)
	{
		records[x] = 0;
		averageFirst[x] = 0;
		averageMin[x] = 0;
		averageMax[x] = 0;
		averageValue[x] = 0;

		tempSum[x] = 0;
		firstVal[x] = 0;
		minVal[x] = 0;
		maxVal[x] = 0;
	}
	printf("Measuring PDB compression stats!\n");
	fflush(stdout);
	for (unsigned int x = 0; x < data.size(); x++)
    {
		for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
		{
			int val = f.ReadFileDepth(data[x].bucketID, y);

			for (int i = 0; i < 20; i++)
			{
				// reset computation
				if (0 == index%(i+1))
				{
					// record results
					if (x != 0 || y != 0)
					{
						records[i]++;
						averageFirst[i] += firstVal[i];
						averageMin[i] += minVal[i];
						averageMax[i] += maxVal[i];
						averageValue[i] += tempSum[i];
					}
					
					tempSum[i] = val;
					firstVal[i] = val;
					minVal[i] = val;
					maxVal[i] = val;
				}
				else {
					tempSum[i] += val;
					if (val < minVal[i])
						minVal[i] = val;
					if (val > maxVal[i])
						maxVal[i] = val;
				}
			}

			index++;
			if (0 == index%100000000)
			{
				for (int i = 0; i < 20; i++)
				{
					printf("---- %d ---\n", i+1);
					printf("%lld processed (%1.2fGB); %lld records total:\n", index, index/1024.0/1024.0/1024.0/2.0, records[i]);
					printf(": Average first: %1.4f\n", (float)averageFirst[i]/records[i]);
					printf(": Average min: %1.4f\n", (float)averageMin[i]/records[i]);
					printf(": Average max: %1.4f\n", (float)averageMax[i]/records[i]);
					printf(": Average value: %1.4f\n", (float)averageValue[i]/records[i]/(i+1));
				}
				fflush(stdout);
			}
		}
    }

}

void ExtractStatesAtDepth(const char *theFile)
{
	RubikEdge e;
	RubikEdgeState es;

	std::vector<std::vector<uint64_t> > vals;
	std::vector<FILE *> files;
	std::vector<long> counts;
	vals.resize(10); // 0...9
	counts.resize(10);
	std::vector<bucketInfo> data;
	std::vector<bucketData> buckets;
	
	uint64_t maxBuckSize = GetMaxBucketSize<RubikEdge, RubikEdgeState>(true);
	InitTwoPieceData<RubikEdge, RubikEdgeState>(data, maxBuckSize);
	InitBucketSize<RubikEdge, RubikEdgeState>(buckets, maxBuckSize);
	
	DiskBitFile f(theFile);
	
	uint64_t entry = 0;
	for (int x = 0; x < 10; x++)
	{
		char name[255];
		sprintf(name, "12edge-depth-%d.dat", x);
		FILE *f = fopen(name, "w+");
		if (f == 0)
		{
			printf("Error opening %s; aborting!\n", name);
			exit(0);
		}
		files.push_back(f);
	}
	for (unsigned int x = 0; x < data.size(); x++)
	{
		for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
		{
			bool debug = false;
			int val = f.ReadFileDepth(data[x].bucketID, y);
			if (val < 10)
			{
				int64_t r1, r2;
				e.unrankPlayer(entry, es, 0);
				e.rankPlayer(es, 0, r1, r2);
				if (r1 != x || r2 != y-data[x].bucketOffset)
				{
					printf("Error: entry is %llu, which should map to x: %llu y: %llu. Instead we got r1: %llu, r2: %llu\n", entry, x, y-data[x].bucketOffset, r1, r2);
					exit(0);
				}
				vals[val].push_back(es.state);
				counts[val]++;
			}
			entry++;
			if (0 == entry%1000000000)
			{
				printf("Processing entry %llu\n", entry);
				fflush(stdout);
				printf("Flushing files:\n"); fflush(stdout);
				for (int t = 0; t < 10; t++)
				{
					printf("File %d, %ld entries\n", t, vals[t].size()); fflush(stdout);
					fwrite(&(vals[t][0]), sizeof(uint64_t), vals[t].size(), files[t]);
					vals[t].resize(0);
				}
			}
		}
	}
	printf("%llu entries read total\n", entry);
	{
		printf("Processing entry %llu\n", entry); fflush(stdout);
		printf("Flushing files:\n"); fflush(stdout);
		for (int x = 0; x < 10; x++)
		{
			printf("File %d, %ld entries\n", x, vals[x].size()); fflush(stdout);
			fwrite(&(vals[x][0]), sizeof(uint64_t), vals[x].size(), files[x]);
			vals[x].resize(0);
		}
	}
	for (int x = 0; x < 10; x++)
	{
		printf("Total entries for file %d: %lu\n", x, counts[x]);
		fclose(files[x]);
	}
}

void TestBloom(int entries, double accuracy)
{
	bloom_parameters parameters;
	
	// How many elements roughly do we expect to insert?
	parameters.projected_element_count = entries;//1000;
	
	// Maximum tolerable false positive probability? (0,1)
	parameters.false_positive_probability = accuracy/100.0;//0.0001; // 1 in 10000
	
	// Simple randomizer (optional)
	parameters.random_seed = 0xA5A5A5A5;
	
	if (!parameters)
	{
		std::cout << "Error - Invalid set of bloom filter parameters!" << std::endl;
		return;
	}
	
	parameters.compute_optimal_parameters();
	
	//Instantiate Bloom Filter
	bloom_filter filter(parameters);
	
	printf("Approximate storage: %lld bits (%1.2f MB / %1.2f GB)\n", parameters.optimal_parameters.table_size,
		   parameters.optimal_parameters.table_size/8.0/1024.0/1024.0,
		   parameters.optimal_parameters.table_size/8.0/1024.0/1024.0/1024.0);

	Timer t;
	t.StartTimer();
	// Insert into Bloom Filter
	RubikEdge e;
	RubikEdgeState s;
	{
		// Insert some numbers
		for (std::size_t i = 0; i < entries; ++i)
		{
			//e.unrankPlayer(i, s, 0);
			filter.insert(i); // s.state
		}
	}
	t.EndTimer();
	printf("%1.6f spend adding entries (%lu bytes each)\n", t.GetElapsedTime(), sizeof(std::size_t));
	
	// Query Bloom Filter
	{
		t.StartTimer();
		int correct = 0;
		int incorrect = 0;
		// Query the existence of numbers
		for (std::size_t i = 0; i < entries; ++i)
		{
			//e.unrankPlayer(i, s, 0);
			if (filter.contains(i)) //s.state
			{
				//std::cout << "BF contains: " << i << std::endl;
				correct++;
			}
		}
		
		srandom(12345);
		// Query the existence of invalid numbers
		for (int i = entries; i < 10*entries; ++i)
		{
			uint64_t next;
			do {
				next = random()%e.getMaxSinglePlayerRank();
			} while (next < entries);
			//e.unrankPlayer(next, s, 0);
			if (filter.contains(next)) // s.state
			{
				//std::cout << "BF falsely contains: " << i << std::endl;
				incorrect++;
			}
		}
		t.EndTimer();
		printf("%1.6f spend querying %d entries\n", t.GetElapsedTime(), entries*11);
		printf("%d of %d items (%5.2f%%) correctly stored. %d of %d items (%5.2f%%) incorrectly detected\n",
			   correct, entries, (100.0*correct/entries), incorrect, entries*10, incorrect*100.0/(entries*10.0));
	}
	return;

}
#include "Bloom.h"

void TestBloom2(int entries, double accuracy)
{
	BloomFilter filter(entries, accuracy/100.0, false);
	
	printf("Approximate storage: %lld bits (%1.2f MB / %1.2f GB)\n", filter.GetStorage(),
		   filter.GetStorage()/8.0/1024.0/1024.0,
		   filter.GetStorage()/8.0/1024.0/1024.0/1024.0);
	
	Timer t;
	t.StartTimer();
	// Insert into Bloom Filter
	RubikEdge e;
	RubikEdgeState s;
	{
		// Insert some numbers
		for (std::size_t i = 0; i < entries; ++i)
		{
			//e.unrankPlayer(i, s, 0);
			filter.Insert(i);
		}
	}
	t.EndTimer();
	printf("%1.6f spend adding entries (%lu bytes each)\n", t.GetElapsedTime(), sizeof(std::size_t));
	
	// Query Bloom Filter
	{
		t.StartTimer();
		int correct = 0;
		int incorrect = 0;
		// Query the existence of numbers
		for (std::size_t i = 0; i < entries; ++i)
		{
			//e.unrankPlayer(i, s, 0);
			if (filter.Contains(i))
			{
				//std::cout << "BF contains: " << i << std::endl;
				correct++;
			}
		}
		
		srandom(12345);
		// Query the existence of invalid numbers
		for (int i = entries; i < 10*entries; ++i)
		{
			uint64_t next;
			do {
				next = random()%e.getMaxSinglePlayerRank();
			} while (next < entries);
			//e.unrankPlayer(next, s, 0);
			if (filter.Contains(next))
			{
				//std::cout << "BF falsely contains: " << i << std::endl;
				incorrect++;
			}
		}
		t.EndTimer();
		printf("%1.6f spend querying %d entries\n", t.GetElapsedTime(), entries*11);
		printf("%d of %d items (%5.2f%%) correctly stored. %d of %d items (%5.2f%%) incorrectly detected\n",
			   correct, entries, (100.0*correct/entries), incorrect, entries*10, incorrect*100.0/(entries*10.0));
	}
	return;
	
}

void GetBloomStats(uint64_t size, int hash, const char *prefix)
{
	BloomFilter bf(size, hash, prefix);
	bf.Analyze();
	exit(0);
}

void TestMinBloom()
{
	int entries = 10000;
	int totalSpace = entries*80/4;
	{
		MinBloomFilter f4(totalSpace, 6, true);
		
		for (int x = 0; x < entries; x++)
			f4.Insert(x, x%15);
		
		printf("Testing correctness\n");
		for (int x = 0; x < entries; x++)
		{
			if (f4.Contains(x) != (x%15))
			{
				printf("For an insertion of %d, we got %d out instead of %d\n",
					   x, f4.Contains(x), (x%15));
			}
		}
		printf("Done!\n");
	
		int hits = 0;
		int total = 0;
		for (int x = entries+1; x < entries*1000; x++)
		{
			total++;
			if (f4.Contains(x) != 0xF)
			{
//				printf("For a false lookup of %d, we got out %d\n",
//					   x, f4.Contains(x));
				hits++;
			}
		}
		printf("%d bytes for %d entries. (%1.1fbits/entry)\n%d of %d hit; fp rate = %1.6f%%\n",
			   totalSpace/2, entries, float(8*totalSpace/2)/float(entries),
			   hits, total, 100*float(hits)/float(total));
	}
	
	{
		MinBloomFilter t4(totalSpace, 6, "");
		printf("Testing correctness\n");
		for (int x = 0; x < entries; x++)
		{
			if (t4.Contains(x) != (x%15))
			{
				printf("For an insertion of %d, we got %d out instead of %d\n",
					   x, t4.Contains(x), (x%15));
			}
		}
		printf("Done!\n");
	}
	
}

