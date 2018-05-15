/* 
 * arduino.cpp
 * 
 *
 */

#define DEBUG
#include "debug_macros.h"
#include "fake_arduino.h"


serialport Serial;
static unsigned long time_reference;
bool timer_started = false;

unsigned long ts_to_ul (struct timespec &t);

unsigned long millis()
{
    struct timespec temp;
    unsigned long PL;
    if (!timer_started) {
        timer_started = true;
        clock_gettime(CLOCK_REALTIME, &temp);
        time_reference = ts_to_ul(temp);
    }
    clock_gettime(CLOCK_REALTIME, &temp);
    PL = (ts_to_ul(temp) - time_reference);
    return PL;
}

unsigned long ts_to_ul (struct timespec &t) 
{ return t.tv_sec * 1000 + t.tv_nsec / 1000000; } 

