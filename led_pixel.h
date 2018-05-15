/* 
 * led_pixel.h
 *
 * purpose: to simulate an LED pixel in terminal
 *
 * date:5/14/18
 * by: Ashton Stephens
 *
 */

#ifndef LED_PIXEL_H_
#define LED_PIXEL_H_

#include <iostream>

struct led_pixel {
public:
    uint8_t r;
    uint8_t g;
    uint8_t b;
    led_pixel (uint8_t r = 0, uint8_t g = 0, uint8_t b = 0): r(r), g(g), b(b) {};
    friend std::ostream &operator << (std::ostream &os, led_pixel p);
};

#endif
