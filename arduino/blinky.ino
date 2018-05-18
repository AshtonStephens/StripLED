#include <FastLED.h>

FASTLED_USING_NAMESPACE

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

#define INPUT_PIN   A0
#define DATA_PIN    2
//#define CLK_PIN   4
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    300
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          150
#define FRAMES_PER_SECOND  120

void sinelon();

void setup() {
    delay(3000); // 3 second delay for recovery
    FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);
    pinMode(INPUT_PIN, INPUT);
}

typedef void (*SimplePatternList[])();

uint8_t gHue = 0; // rotating "base color" used by many of the patterns
  
void loop()
{
    static int i = 0;
    static long long timer = 0;    
    static uint16_t hue = 0;
    
    CRGB buff;

    FastLED.show();  
    FastLED.delay(1000/FRAMES_PER_SECOND);
    
    hue = analogRead(INPUT_PIN) >> 2;
    Serial.println(hue);
    buff = CHSV( 45, 169, hue);

    for (int i = 0; i < NUM_LEDS; ++i) {
        leds[i] = buff;
    }
    //gHue = hue >> 4;
    //sinelon();    
}

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

bool backwards = false;
void sinelon()
{
    // a colored dot sweeping back and forth, with fading trails
    fadeToBlackBy( leds, NUM_LEDS, 20); 
    int pos = beatsin16(5, 0, NUM_LEDS-1 );
    static int lastpos = pos;
    
    if (pos == NUM_LEDS-1) backwards = true;
    if (pos == 0) backwards = false;
    if (backwards) pos = NUM_LEDS-pos-1 ;
    
    if (pos >= lastpos) {
        for (int i = lastpos; i <= pos; ++i) 
            leds[i] += CHSV( gHue, 255, 192);
    } /*else {
        for (int i = pos; i < lastpos; ++i) 
            leds[i] += CHSV( gHue, 255, 192);
    }*/

    lastpos = pos; 
}

