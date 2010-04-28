/*
 *  swapendian.h
 *  games
 *
 *  Created by Nathan Sturtevant on 3/2/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include <stdint.h>

#ifndef ENDIAN_H
#define ENDIAN_H

static inline bool is_little() {
  int32_t e = 1;
  return *reinterpret_cast<char*>(&e);
}

inline uint16_t swap_endian16(uint16_t x)
{
  return (x << 8) | ((x >> 8) & 0xff);
}

inline uint32_t swap_endian32(uint32_t x)
{
  return (x << 24) | ((x << 8) & 0xff0000) |
	((x >> 8) & 0xff00) | ((x >> 24) & 0xff);
}

inline uint64_t swap_endian64(uint64_t x)
{
  return ((x << 24)&0xFF000000FF000000ll) | ((x << 8) & 0x00FF000000FF0000ll) |
	((x >> 8) & 0x0000ff000000ff00ll) | ((x >> 24) & 0x000000FF000000FFll);
}

inline void little2machine(uint16_t &x)
{
  if (!is_little())
    x = swap_endian16(x);
}

inline void little2machine(uint32_t &x)
{
  if (!is_little())
    x = swap_endian32(x);
}

inline void little2machine(float &x)
{
  if (!is_little()) {
		
    union {
      uint32_t i;
      float f;
    };
		
    f = x;
    swap_endian32(i);
    x = f;
  }
}

inline void little2machine(double &x)
{
  if (!is_little()) {
		
    union {
      uint64_t i;
      double f;
    };
		
    f = x;
    swap_endian64(i);
    x = f;
  }
}

#endif
