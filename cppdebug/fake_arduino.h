/* 
 * arduino.h
 * 
 *
 */

#ifndef FAKE_ARDUINO_H
#define FAKE_ARDUINO_H

#include <iostream>
#include <stdint.h>
#include <time.h> 
#include <sys/types.h>

struct serialport 
{
    template <typename T>
    void print(T toPrint)   { std::cout << toPrint;}

    template <typename T>
    void println(T toPrint) { std::cout << toPrint << '\n';}
};

extern unsigned long millis();
/*
void set_time_factor(unsigned long  factor);
*/
extern serialport Serial;

#endif
