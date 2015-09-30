//
//  NBitArray.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 9/29/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//

#ifndef hog2_glut_NBitArray_h
#define hog2_glut_NBitArray_h

#include <stdint.h>
#include <string.h>
#include <algorithm>

/**
 * This class supports compact n-bit arrays. For (1 <= n <= 64). 
 * It is efficient for powers of two, but less so
 * for non-powers of two. (Currently about 3x slower.)
 */
template <uint64_t numBits>
class NBitArray
{
public:
	NBitArray(uint64_t numEntries = 0);
	NBitArray(const char *);
	~NBitArray();
	void FillMax();
	void Clear();
	void Resize(uint64_t newMaxEntries);
	uint64_t Size() const;
	uint64_t Get(uint64_t index) const;
	void Set(uint64_t index, uint64_t val);

	void Write(FILE *);
	void Read(FILE *);
	void Write(const char *);
	void Read(const char *);
private:
	uint64_t *mem;
	uint64_t entries;
	uint64_t memorySize;
};

template <uint64_t numBits>
NBitArray<numBits>::NBitArray(uint64_t numEntries)
:entries(numEntries), memorySize(((entries*numBits+63)/64))
{
	static_assert(numBits >= 1 && numBits <= 64, "numBits out of bounds!");

	mem = new uint64_t[memorySize];
}

template <uint64_t numBits>
NBitArray<numBits>::NBitArray(const char *file)
:mem(0)
{
	static_assert(numBits >= 1 && numBits <= 64, "numBits out of bounds!");
	Read(file);
}

template <uint64_t numBits>
NBitArray<numBits>::~NBitArray()
{
	delete [] mem;
}

template <uint64_t numBits>
void NBitArray<numBits>::FillMax()
{
	memset(mem, 0xFF, memorySize*8);
}

template <uint64_t numBits>
void NBitArray<numBits>::Clear()
{
	memset(mem, 0, memorySize*8);
}

template <uint64_t numBits>
void NBitArray<numBits>::Resize(uint64_t newMaxEntries)
{
	entries = newMaxEntries;
	memorySize = ((entries*numBits+63)/64);
	delete [] mem;
	mem = new uint64_t[memorySize];
}

template <uint64_t numBits>
uint64_t NBitArray<numBits>::Size() const
{
	return entries;
}

template <uint64_t numBits>
void NBitArray<numBits>::Write(FILE *f)
{
	fwrite(&entries, sizeof(uint64_t), 1, f);
	fwrite(&memorySize, sizeof(uint64_t), 1, f);
	fwrite(mem, sizeof(uint64_t), memorySize, f);
}

template <uint64_t numBits>
void NBitArray<numBits>::Read(FILE *f)
{
	delete [] mem;
	fread(&entries, sizeof(uint64_t), 1, f);
	fread(&memorySize, sizeof(uint64_t), 1, f);
	mem = new uint64_t[memorySize];
	fread(mem, sizeof(uint64_t), memorySize, f);
}

template <uint64_t numBits>
void NBitArray<numBits>::Write(const char *file)
{
	FILE *f = fopen(file, "w+b");
	if (f == 0)
	{
		perror("Could not open file for writing in NBitArray");
		return;
	}
	Write(f);
	fclose(f);
}

template <uint64_t numBits>
void NBitArray<numBits>::Read(const char *file)
{
	FILE *f = fopen(file, "rb");
	if (f == 0)
	{
		perror("Could not open file for reading in NBitArray");
		return;
	}
	Read(f);
	fclose(f);
}

template <uint64_t numBits>
uint64_t NBitArray<numBits>::Get(uint64_t index) const
{
	uint64_t startingBit = index*numBits;
	uint64_t offset1 = startingBit/64;
	uint64_t bitOffset1 = startingBit&0x3F; // same as mod 64
	uint64_t bitCount1 = std::min(64-bitOffset1, numBits);
	uint64_t bitCount2 = numBits - bitCount1;
	uint64_t bitMask1 = (1ull<<bitCount1)-1;
	uint64_t bitMask2 = (1ull<<bitCount2)-1;
	uint64_t result = (mem[offset1]>>bitOffset1)&bitMask1;
	result = ((mem[offset1+1]&bitMask2)<<bitCount1) | result;
	return result;
}

template <>
uint64_t NBitArray<64>::Get(uint64_t index) const
{
	return mem[index];
}

template <>
uint64_t NBitArray<32>::Get(uint64_t index) const
{
	return (mem[index>>1]>>(32*(index&0x1)))&0xFFFFFFFF;
}

template <>
uint64_t NBitArray<16>::Get(uint64_t index) const
{
	return (mem[index>>2]>>(16*(index&0x3)))&0xFFFF;
}

template <>
uint64_t NBitArray<8>::Get(uint64_t index) const
{
	return (mem[index>>3]>>(8*(index&0x7)))&0xFF;
}

template <>
uint64_t NBitArray<4>::Get(uint64_t index) const
{
	return (mem[index>>4]>>(4*(index&0xF)))&0xF;
}

template <>
uint64_t NBitArray<2>::Get(uint64_t index) const
{
	return (mem[index>>5]>>(2*(index&0x1F)))&0x3;
}

template <>
uint64_t NBitArray<1>::Get(uint64_t index) const
{
	return (mem[index>>6]>>(1*(index&0x3F)))&0x1;
}


template <uint64_t numBits>
void NBitArray<numBits>::Set(uint64_t index, uint64_t val)
{
	uint64_t startingBit = index*numBits;
	uint64_t offset1 = startingBit/64;
	uint64_t bitOffset1 = startingBit&0x3F; // same as mod 64
	uint64_t bitCount1 = std::min(64-bitOffset1, (uint64_t)numBits);
	uint64_t bitCount2 = numBits - bitCount1;
	uint64_t bitMask1 = (1ull<<bitCount1)-1;
	uint64_t bitMask2 = (1ull<<bitCount2)-1;
	mem[offset1] = (mem[offset1]&(~(bitMask1<<bitOffset1))) | ((val&bitMask1)<<bitOffset1);
	mem[offset1+1] = (mem[offset1+1]&(~(bitMask2))) | ((val>>bitCount1)&bitMask2);
	//	uint64_t result = (mem[offset1]>>bitOffset1)&bitMask1;
//	result = ((mem[offset1+1]&bitMask2)<<bitCount2) | result;
}

template <>
void NBitArray<64>::Set(uint64_t index, uint64_t val)
{
	mem[index] = val;
}

template <>
void NBitArray<32>::Set(uint64_t index, uint64_t val)
{
	val = (val&0xFFFFFFFF)<<(32*(index&0x1));
	uint64_t mask = ~((0xFFFFFFFFull)<<(32*(index&0x1)));
	mem[index>>1] = (mem[index>>1]&mask)|val;
}

template <>
void NBitArray<16>::Set(uint64_t index, uint64_t val)
{
	val = (val&0xFFFF)<<(16*(index&0x3));
	uint64_t mask = ~((0xFFFFull)<<(16*(index&0x3)));
	mem[index>>2] = (mem[index>>2]&mask)|val;
}

template <>
void NBitArray<8>::Set(uint64_t index, uint64_t val)
{
	val = (val&0xFF)<<(8*(index&0x7));
	uint64_t mask = ~((0xFFull)<<(8*(index&0x7)));
	mem[index>>3] = (mem[index>>3]&mask)|val;
}

template <>
void NBitArray<4>::Set(uint64_t index, uint64_t val)
{
	// 2^4 = 16;
	val = (val&0xF)<<(4*(index&0xF));
	uint64_t mask = ~((0xFull)<<(4*(index&0xF)));
	mem[index>>4] = (mem[index>>4]&mask)|val;
}

template <>
void NBitArray<2>::Set(uint64_t index, uint64_t val)
{
	// 2^4 = 16;
	val = (val&0x3)<<(2*(index&0x1F));
	uint64_t mask = ~((0x3ull)<<(2*(index&0x1F)));
	mem[index>>5] = (mem[index>>5]&mask)|val;
}

template <>
void NBitArray<1>::Set(uint64_t index, uint64_t val)
{
	// 2^4 = 16;
	val = (val&0x1)<<(1*(index&0x3F));
	uint64_t mask = ~((0x1ull)<<(1*(index&0x3F)));
	mem[index>>6] = (mem[index>>6]&mask)|val;
}



#endif
