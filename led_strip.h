/* 
 * led_strip.h
 *
 * purpose: to simulate an LED pixel strip in terminal
 *
 * date:5/14/18
 * by: Ashton Stephens
 *
 */

#ifndef LED_STRIP_H_
#define LED_STRIP_H_

#include "led_strip.h"

template <int N>
struct led_strip 
{
    led_pixel leds[N];
    friend std::ostream &operator << (std::ostream &os, led_strip<N> &s)
    {
        os << "\033[F";
        for (int i = 0; i < N; ++i) {
            os << s.leds[i];
        }
        return os;
    }
};

#endif
