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

/**
 * An efficient bit-wise vector implementation.
 */

class BitVector {
public:
  BitVector(int size);
  ~BitVector();
  void clear();
  BitVector *Clone();
  int GetSize() { return true_size; }
  bool Get(int index) const;
  void Set(int index, bool value);
  void Merge(BitVector *);
  bool Equals(BitVector *);
  int GetNumSetBits();
private:
  int size, true_size;
  uint32_t *storage;
};

#endif
