/*
 * $Id: BitVector.cpp,v 1.3 2006/09/18 06:20:15 nathanst Exp $
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

#include <cstdlib>
#include <cstdio>
#include "BitVector.h"
#include "MMapUtil.h"
#include <sys/mman.h>


BitVector::BitVector(uint64_t _size)
{
	true_size = _size;
	size = (_size>>storageBitsPower)+1;
	storage = new storageElement[size];
	for (int x = 0; x < size; x++)
		storage[x] = 0;
	memmap = false;
}

BitVector::BitVector(uint64_t entries, const char *file, bool zero)
{
	// make sure we allocate even space for 32 bit entries
	while (0 != entries%storageBits)
	{
		entries++;
	}
	uint8_t *mem = GetMMAP(file, entries/8, fd, zero); // number of bytes needed
	storage = (storageElement*)mem;
	size = (entries>>storageBitsPower)+1;
	true_size = entries;
	memmap = true;
}

BitVector::~BitVector()
{
	if (!memmap)
	{
		delete [] storage;
	}
	else {
		printf("Closing mmap\n");
		// close memmap
		CloseMMap((uint8_t*)storage, true_size/8, fd);
	}
}

void BitVector::Save(const char *file)
{
	FILE *f = fopen(file, "w+");
	if (f == 0)
	{
		printf("File write error\n");
		exit(0);
	}
	fwrite(&true_size, sizeof(true_size), 1, f);
	fwrite(storage, sizeof(storageElement), size, f);
	fclose(f);
}

void BitVector::Load(const char *file)
{
	if (memmap)
	{
		printf("BitVector is memmapped; not loading\n");
		return;
	}
	
	delete [] storage;
	FILE *f = fopen(file, "r");
	if (f == 0)
	{
		printf("File write error\n");
		exit(0);
	}
	//fread(&size, sizeof(size), 1, f);
	fread(&true_size, sizeof(true_size), 1, f);
	printf("Loading %llu entries\n", true_size);
	// allocate storage
	size = (true_size>>storageBitsPower)+1;

	storage = new storageElement[size];
	//	for (int x = 0; x < size; x++)
	//		storage[x] = 0;

	// TODO:
	fread(storage, sizeof(storageElement), size, f);
	fclose(f);
}

void BitVector::clear()
{
  for (int x = 0; x < size; x++)
	  storage[x] = 0;
}

//BitVector *BitVector::Clone()
//{
//  BitVector *bv = new BitVector(true_size);
//  bv->Merge(this);
//  return bv;
//}

void BitVector::Set(uint64_t index, bool value)
{
  if ((index>>storageBitsPower) > size) {
    printf("SET %llu OUT OF RANGE\n", index);
    exit(0);
  }
  if (value)
    storage[index>>storageBitsPower] = storage[index>>storageBitsPower]|(1<<(index&storageMask));
  else
    storage[index>>storageBitsPower] = storage[index>>storageBitsPower]&(~(1<<(index&storageMask)));
}


//void BitVector::Merge(BitVector *bv)
//{
//  if (bv == 0) return;
//  if (bv->size != size) {
//    printf("Error; can't merge vectors of different sizes (%d/%d)\n", bv->true_size, true_size);
//    return;
//  }
//  for (int x = 0; x < size; x++) storage[x] |= bv->storage[x];
//}

bool BitVector::Equals(BitVector *bv)
{
  if (bv->size != size) return false;
  for (int x = 0; x < size; x++)
    if (storage[x] != bv->storage[x])
      return false;
  return true;
}

int BitVector::GetNumSetBits()
{
  int sum = 0;
  for (int x = 0; x < size; x++) {
    storageElement iter = storage[x];
    while (iter) {
      if (iter&1) sum++;
      iter = iter>>1;
    }
  }
  return sum;
}
