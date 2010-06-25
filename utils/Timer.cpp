/*
 * $Id: timer.cpp,v 1.6 2006/11/30 20:33:25 nathanst Exp $
 *
 * timer.cpp
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

//#include "UnitSimulation.h"
#include "Timer.h"
#include <stdint.h>
#include <cstring>

Timer::Timer()
{
	elapsedTime = 0;
}

void Timer::StartTimer()
{
#ifdef OS_MAC
	startTime = UpTime();
#elif defined( TIMER_USE_CYCLE_COUNTER )
	CycleCounter c;
	startTime = c.count();
#else
	gettimeofday( &startTime, NULL );
#endif
}

#ifdef linux

float Timer::getCPUSpeed()
{
	FILE *f;

	static float answer = -1;
	
	if (answer != -1)
		return answer;
	
	f = fopen("/proc/cpuinfo", "r");
	if (f)
	{
		while (!feof(f))
		{
			char entry[1024];
			char temp[1024];
			fgets(entry, 1024, f);
			if (strstr(entry, "cpu MHz"))
			{
				//                              cpu MHz         : 997.399
				float answer;
				sscanf(entry, "%[^0-9:] : %f", temp, &answer);
				//printf("Read CPU speed: %1.2f\n", answer);
				fclose(f);
				return answer;
			}
		}
		fclose(f);
	}
	return 0;
}

#endif

double Timer::EndTimer()
{
#ifdef OS_MAC
	AbsoluteTime stopTime = UpTime();
	Nanoseconds diff = AbsoluteDeltaToNanoseconds(stopTime, startTime);
	uint64_t nanosecs = UnsignedWideToUInt64(diff);
	//cout << nanosecs << " ns elapsed (" << (double)nanosecs/1000000.0 << " ms)" << endl;
	return elapsedTime = (double)(nanosecs/1000000000.0);
#elif defined( TIMER_USE_CYCLE_COUNTER )
	Timer::CycleCounter c;
	double diffTime = (double)(c.count() - startTime);
	const static double ClocksPerSecond = getCPUSpeed() * 1000000.0;
	elapsedTime = diffTime / ClocksPerSecond;
	return elapsedTime;
#else
	struct timeval stopTime;
	
	gettimeofday( &stopTime, NULL );
	uint64_t microsecs = stopTime.tv_sec - startTime.tv_sec;
	microsecs = microsecs * 1000000 + stopTime.tv_usec - startTime.tv_usec;
	elapsedTime = (double)microsecs / 1000000.0;
	return elapsedTime;
#endif
}
