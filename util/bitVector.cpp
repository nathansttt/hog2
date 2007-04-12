/*
 * $Id: bitVector.cpp,v 1.3 2006/09/18 06:20:15 nathanst Exp $
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
#include "bitVector.h"

bitVector::bitVector(int _size)
{
  true_size = _size;
  size = (_size>>5)+1;
  storage = new uint32_t[size];
  for (int x = 0; x < size; x++) storage[x] = 0;
}

bitVector::~bitVector()
{
  delete [] storage;
}

void bitVector::clear()
{
  for (int x = 0; x < size; x++) storage[x] = 0;
}

bitVector *bitVector::clone()
{
  bitVector *bv = new bitVector(true_size);
  bv->merge(this);
  return bv;
}

bool bitVector::get(int index) const
{
  if ((index>>5) > size) {
    printf("GET %d OUT OF RANGE\n", index);
    exit(0);
  }
  return (((storage[index>>5])>>(index&0x1F))&0x1);
}

void bitVector::set(int index, bool value)
{
  if ((index>>5) > size) {
    printf("SET %d OUT OF RANGE\n", index);
    exit(0);
  }
  if (value)
    storage[index>>5] = storage[index>>5]|(1<<(index&0x1F));
  else
    storage[index>>5] = storage[index>>5]&(~(1<<(index&0x1F)));
}

void bitVector::merge(bitVector *bv)
{
  if (bv == 0) return;
  if (bv->size != size) {
    printf("Error; can't merge vectors of different sizes (%d/%d)\n", bv->true_size, true_size);
    return;
  }
  for (int x = 0; x < size; x++) storage[x] |= bv->storage[x];
}

bool bitVector::equals(bitVector *bv)
{
  if (bv->size != size) return false;
  for (int x = 0; x < size; x++)
    if (storage[x] != bv->storage[x])
      return false;
  return true;
}

int bitVector::getNumSetBits()
{
  int sum = 0;
  for (int x = 0; x < size; x++) {
    uint32_t iter = storage[x];
    while (iter) {
      if (iter&1) sum++;
      iter = iter>>1;
    }
  }
  return sum;
}
