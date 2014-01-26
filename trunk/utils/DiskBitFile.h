//
//  DiskBitFile.h
//  Rubik
//
//  Created by Nathan Sturtevant on 4/5/13.
//  Copyright (c) 2013 Nathan Sturtevant. All rights reserved.
//

#ifndef __Rubik__DiskBitFile__
#define __Rubik__DiskBitFile__

#include "EnvUtil.h"
#include <iostream>
#include <cassert>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

//const int BITS = 4;
#define BITS 4

class DiskBitFile
{
public:
	DiskBitFile(const char *pre);
	~DiskBitFile();
	void Init(const std::vector<bucketData> &buckets);
	
	// incoming offset is in entries, not bytes
	void WriteFileDepth(int bucket, int64_t offset, uint8_t value);
	int ReadFileDepth(int bucket, int64_t offset);
	void CloseReadWriteFile();
	
	uint8_t *ReadChunk(int bucket, int64_t offset, int numEntries, uint8_t *data);
	void CloseReadFile();

	uint64_t GetBytesRead() const { return bytesRead; }
	uint64_t GetBytesWritten() const { return bytesWritten; }
private:
	void FlushCache();
	const char *getBucketFileName(int bucket, int subBucket);

	// data for reading and writing depths
	FILE *outputFile;
	int outputBucket;
	int outputSubBucket;
	int64_t cacheOffset; // beginning of cache [in bytes]
	
	// data for reading chunks
	bool fileOpen;
	FILE *chunkFile;
	int64_t fileOffset;
	int64_t currBucket;
	int64_t currSubBucket;

	const static int subBucketBits = 30;
	const static int64_t cacheSize = 1ull<<22;//4096;//1024*512; // no problem(?) // 4 MB!
	int64_t theCacheSize;      // valid bytes in the cache
	int64_t cacheFilePosition; // current offset in file (bytes)
	bool cacheChanged;
	uint8_t cache[cacheSize];

	uint64_t bytesRead, bytesWritten;
	char bucketFileName[255];
	char prefix[64];
};



#endif
