/* 
 * main.cpp
 *
 */

#define DEBUG
#include "debug_macros.h"
#include "led_pixel.h"
#include "led_strip.h"
#include "fake_arduino.h"

#define NUM_LEDS 100

int main  (int argc, char *argv[])
{

    (void) argc; (void) argv;
    long long timer;

    led_strip<NUM_LEDS> strip;
    led_pixel p  = {10,100,10};
    
    for (int i = 0; i < NUM_LEDS; ++i) {
        
        timer = millis();
        while ( millis() - timer < 30) {}

        strip.leds[i] = {10,((uint8_t)i),10};
        std::cout << strip << std::endl;
    }

}


