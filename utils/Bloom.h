//
//  Bloom.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/25/14.
//  Copyright (c) 2014 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__Bloom__
#define __hog2_glut__Bloom__

#include <iostream>
#include <stdint.h>
#include "BitVector.h"

class BloomFilter {
public:
	BloomFilter(uint64_t numItems, double targetHitRate, bool save, bool zero=true);
	BloomFilter(uint64_t filterSize, int numHash, bool save, bool zero=true);
	BloomFilter(uint64_t filterSize, int numHash, const char *loadPrefix);
	~BloomFilter();
	void Insert(uint64_t item);
	bool Contains(uint64_t item);
	uint64_t GetStorage() { return filterSize; }
	int GetNumHash() { return numHash; }
	void Load();
private:
	uint64_t Hash(uint64_t value, int which);
	int numHash;
	bool saveAtExit;
	uint64_t filterSize;
	BitVector *bits;
};

#endif /* defined(__hog2_glut__Bloom__) */
