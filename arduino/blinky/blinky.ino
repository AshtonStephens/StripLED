#include <FastLED.h>

FASTLED_USING_NAMESPACE

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

typedef uint8_t     uint8;
typedef uint16_t    uint16;
typedef int16_t		int16;

#define ANAL_HUE	A0
#define ANAL_SAT	A1
#define ANAL_VAL	A2
#define ANAL_A 		A3
#define ANAL_B 		A4
#define ANAL_C 		A5

#define DATA_PIN    2
//#define CLK_PIN   4
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    300

#define BRIGHTNESS         150
#define FRAMES_PER_SECOND  120

#undef min
#undef max
#undef abs

template<typename T> T min(T a, T b) 	{ return a < b ? a : b; }
template<typename T> T max(T a, T b) 	{ return a < b ? b : a; }
template<typename T> T abs(T a) 		{ return a < 0 ? -a : a; }

struct Location {
	uint16	pos;
	int16	time;
};
struct Blinky {
	CRGB leds[NUM_LEDS];
	Location	locs[64];
	int			num_locs;
	uint16		smooth[4];
	int			frame;		
	int			mode;
	int			lastpos;

    Blinky() : mode(0) {}

	void	clear() {
		memset(leds, 0, sizeof(leds));
	}
	void	fade(uint8 b) {
		fadeToBlackBy(leds, NUM_LEDS, b);
	}
	void	add_range(int a, int b, CHSV hsv) {
		if (a > b) {
			b	= a - b;
			a	= a - b;
			b	= a - b;
		}
        for (int i = a; i <= b; ++i) 
            leds[i] += hsv;
	}
	void	set_range(int a, int b, CRGB rgb) {
		if (a > b) {
			b	= a - b;
			a	= a - b;
			b	= a - b;
		}
        for (int i = a; i <= b; ++i) 
            leds[i] = rgb;
	}

    void setup() {
        FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
        FastLED.setBrightness(BRIGHTNESS);
        pinMode(ANAL_HUE, INPUT);
        pinMode(ANAL_SAT, INPUT);
        pinMode(ANAL_VAL, INPUT);
    
        pinMode(ANAL_A, INPUT);
        pinMode(ANAL_B, INPUT);
        pinMode(ANAL_C, INPUT);

		clear();
		num_locs = 16;
		for (int i = 0; i < num_locs; i++) {
			locs[i].pos = random(NUM_LEDS);
			locs[i].time = random(65536);
		}
	
	}
    void loop() {
        FastLED.show();  
        FastLED.delay(1000/FRAMES_PER_SECOND);

    	uint16	n = analogRead(ANAL_HUE);
    	uint8	hue = n >> 2;
    	uint8	sat = analogRead(ANAL_SAT) >> 2;
    	uint8	val = analogRead(ANAL_VAL) >> 2;

		sat	= 255;
		val	= 255;

        CHSV 	hsv = CHSV(hue, sat, val);
        CRGB 	rgb = hsv;

		Serial.println(n);

		smooth[frame & 3] = n;
		int		n2 = (smooth[0] + smooth[1] + smooth[2] + smooth[3]) / 4;

		switch (mode) {
			case 0: {
				int	e	= 0;
				n2 &= ~3;
				for (int i = 0; i < NUM_LEDS; i++) {
					e += n2;
					leds[i] = CHSV(0, 0, e > 1023 ? 255 : 0);
					e &= 1023;
				}
				break;
			}
			case 4: {	// sparkles
				int	x = min(abs((int)n - (int)lastpos) * 256, 0x7fff);
				Serial.println(x);
				for (int i = 0; i < num_locs; i++) {
					int	t = locs[i].time += x;
					if (t >= 0 && t < x) {
						leds[locs[i].pos] = CHSV(0, 0, 0);
						locs[i].pos = random(NUM_LEDS);
					}
					leds[locs[i].pos] = CHSV(0, 0, abs(t) >> 7);
				}
				lastpos = n;
				break;
			}
			case 5: {	//single led
				clear();
				int	pos = (n * (long)NUM_LEDS) >> 10;
				leds[pos] = CRGB(255,255,255);
				lastpos	= pos;
				break;
			}
			case 6:		// solid colour
				set_range(0, NUM_LEDS - 1, rgb);
				break;

			case 7: {	// a colored dot sweeping back and forth, with fading trails
				fade(20);
				int pos = beatsin16(5, 0, NUM_LEDS * 2);
				if (pos >= NUM_LEDS)
					pos = NUM_LEDS * 2 - 1 - pos;

				add_range(lastpos, pos, hsv);
				break;
			}

		}
		++frame;
    }
} blinky;

void setup() {
	Serial.begin(9600);
    delay(500); // 0.5 second delay for recovery
    blinky.setup();
}
void loop() {
    blinky.loop();
}



