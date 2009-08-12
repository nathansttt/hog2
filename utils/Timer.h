/*
 * $Id: timer.h,v 1.5 2006/10/30 17:45:10 nathanst Exp $
 *
 * timer.h
 * HOG file
 * 
 * Written by Renee Jansen on 08/28/06
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

#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include <fstream>
#ifdef OS_MAC
#include <Carbon/Carbon.h>
#undef check
#else
#include <sys/time.h>
#endif

#ifndef OS_MAC
//#define TIMER_USE_CYCLE_COUNTER
#endif

class Timer {

#if !defined( OS_MAC ) && defined( TIMER_USE_CYCLE_COUNTER )

struct CycleCounter {
public:
	union {
		uint64_t c8;
		struct { uint32_t l, h; } c4;
	} count_;
	
	void stamp() {
#ifdef __i386__
		__asm__ __volatile__ (".byte 0x0f,0x31" : "=a"(count_.c4.l),"=d"(count_.c4.h));
#else
		count_.c8 = 0;
#endif    
	}
	
	uint64_t count() const { return count_.c8; }
	CycleCounter() { stamp(); }
};

#endif


private:
#ifdef OS_MAC
  AbsoluteTime startTime;
#elif defined( TIMER_USE_CYCLE_COUNTER )
  // clock_t startTime;
  uint64_t startTime;
#else
  struct timeval startTime;
#endif		

	double elapsedTime;

	float getCPUSpeed();

public:
	Timer();
	~Timer(){}

	void StartTimer();
	double EndTimer();
	double GetElapsedTime(){return elapsedTime;}

};

#endif
