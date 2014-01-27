// HOG File
/*
 * $Id: BitVector.h,v 1.4 2006/09/18 06:20:15 nathanst Exp $
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
*/ 

#ifndef _BITVECTOR_
#define _BITVECTOR_

#include <stdint.h>
#include "MMapUtil.h"

/**
 * An efficient bit-wise vector implementation.
 */

//typedef uint32_t storageElement;
//const int storageBits = 32;
//const int storageBitsPower = 5;
//const int storageMask = 0x1F;

typedef uint8_t storageElement;
const int storageBits = 8;
const int storageBitsPower = 3;
const int storageMask = 0x7;


class BitVector {
public:
	BitVector(uint64_t size);
	BitVector(uint64_t entries, const char *file, bool zero);
	~BitVector();
	void clear();
//	BitVector *Clone();
	uint64_t GetSize() { return true_size; }
	bool Get(uint64_t index) const;
	void Set(uint64_t index, bool value);
	void SetTrue(uint64_t index);
//	void Merge(BitVector *);
	void Save(const char *);
	void Load(const char *);
	bool Equals(BitVector *);
	int GetNumSetBits();
private:
	uint64_t size, true_size;
	storageElement *storage;
	bool memmap;
	int fd;
};

inline bool BitVector::Get(uint64_t index) const
{
	return (((storage[index>>storageBitsPower])>>(index&storageMask))&0x1);
}

inline void BitVector::SetTrue(uint64_t index)
{
	storage[index>>storageBitsPower] = storage[index>>storageBitsPower]|(1<<(index&storageMask));
}

#endif
