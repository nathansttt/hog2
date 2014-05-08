//
//  MinBloom.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 5/4/14.
//  Copyright (c) 2014 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__MinBloom__
#define __hog2_glut__MinBloom__

#include <iostream>
#include "FourBitArray.h"

class MinBloomFilter {
public:
	MinBloomFilter(uint64_t numItems, double targetHitRate, bool save, bool zero=true);
	MinBloomFilter(uint64_t filterSize, int numHash, bool save, bool zero=true);
	MinBloomFilter(uint64_t filterSize, int numHash, const char *loadPrefix);
	~MinBloomFilter();
	void Analyze();
	void Insert(uint64_t item, int depth);
	int Contains(uint64_t item);
	uint64_t GetStorage() { return filterSize; }
	int GetNumHash() { return numHash; }
	void Load();
private:
	uint64_t Hash(uint64_t value, int which);
	int numHash;
	bool saveAtExit;
	uint64_t filterSize;
	FourBitArray *bits;
};

#endif /* defined(__hog2_glut__MinBloom__) */
