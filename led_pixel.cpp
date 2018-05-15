/* 
 * led_pixel.cpp
 *
 * purpose: to simulate an LED pixel in terminal
 *
 * date:5/14/18
 * by: Ashton Stephens
 *
 */

#include "led_pixel.h"

std::ostream &operator << (std::ostream &os, led_pixel p)
{
    return os << 
        "\033[48;2;"    << 
        (int)p.r << ";" << // red
        (int)p.g << ";" << // green
        (int)p.b << "m" << // blue
        " " << "\033[0m";
}
