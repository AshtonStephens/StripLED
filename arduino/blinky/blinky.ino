#include <FastLED.h>

FASTLED_USING_NAMESPACE

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

typedef uint8_t     uint8;
typedef uint16_t    uint16;
typedef int16_t		int16;
typedef int32_t 	int32;
typedef uint32_t 	uint32;

#define ANAL_HUE	A0
#define ANAL_SAT	A1
#define ANAL_VAL	A2
#define ANAL_MODE	A3
#define ANAL_B 		A4
#define ANAL_C 		A5

#define DATA_PIN    2
//#define CLK_PIN   4
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    300

#define BRIGHTNESS         100
#define FRAMES_PER_SECOND  120

#undef min
#undef max
#undef abs

template<typename T> void clear(T &t) 		{ memset(&t, 0, sizeof(T)); }
template<typename T> T min(T a, T b) 		{ return a < b ? a : b; }
template<typename T> T max(T a, T b) 		{ return a < b ? b : a; }
template<typename T> T clamp(T t, T a, T b) { return min(max(t, a), b); }
template<typename T> T abs(T a) 			{ return a < 0 ? -a : a; }

int16	mul16(int16 a, int16 b) { return (int32(a) * b) >> 16; }
int16	div16(int16 a, int16 b) { return (int32(a) << 16) / b; }

struct v3 {
	int16	x, y, z;
	v3()	{}
	v3(int16 x, int16 y, int16 z) : x(x), y(y), z(z) {}
	v3&	operator+=(const v3 &b) { x += b.x; y += b.y; z += b.z; return *this; }
	v3&	operator-=(const v3 &b) { x -= b.x; y -= b.y; z -= b.z; return *this; }
	v3&	operator*=(const v3 &b) { x = mul16(x, b.x); y = mul16(y, b.y); z = mul16(z, b.z); return *this; }
	v3&	operator/=(const v3 &b) { x = div16(x, b.x); y = div16(y, b.y); z = div16(z, b.z); return *this; }

	friend v3	operator+(const v3 &a, const v3 &b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
	friend v3	operator-(const v3 &a, const v3 &b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
	friend v3	operator*(const v3 &a, const v3 &b) { return v3(mul16(a.x, b.x), mul16(a.y, b.y), mul16(a.z, b.z)); }
	friend v3	operator/(const v3 &a, const v3 &b) { return v3(div16(a.x, b.x), div16(a.y, b.y), div16(a.z, b.z)); }
};

struct ModeChanger {
	int16	prev;
	static int	mode(int16 p)	{ return p >> 8; }

	ModeChanger() : prev(0) {}
	bool	update(int16 x) {
		if (abs(x - prev) < 64)
			return false;
		prev += clamp(x - prev, -64, +64);
		return true;
	}
	uint8	frac() const	{ return prev & 0xff; }
	operator int() const 	{ return mode(prev); }
};

template<typename T, unsigned N> struct Averaged {
	T	vals[N];
	void	update(uint8 i, T v) { vals[i % N] = v; }
	operator T() const { T t = 0; for (int i = 0; i < N; i++) t += vals[i]; return t / N; }
};

struct Fire {
	enum {NUM = 300};
  	uint8 	heat[NUM];

	void	init() { clear(heat); }
	void	update() {
		uint8	h0 = heat[0];
    	for (int i = 1; i < NUM - 1; i++) {
			uint8	h1 = heat[i];
      		h0 = (h0 + h1 + heat[i + 1]) / 3;
      		heat[i] = h0 > 1 && random8() < 128 ? h0 - 1 : h0;
			h0 = h1;
		}    
		int y = random16(NUM);
		heat[y] = qadd8( heat[y], random8(160,255) );
	}
	void	read(CRGB *leds, uint8 brightness) {
    	for (int i = 0; i < NUM; i++)
		    leds[i] = HeatColor(min(heat[i] * 2, 255)) % brightness;
	}
};

struct Blinky {
	CRGB 		leds[NUM_LEDS];
	Averaged<uint16,4>	pot[6];
	uint8		frame;
	uint8		mode_hold;
	uint32		lastpos;

	ModeChanger	mode;

	union {
		struct {
			struct Location {
				uint16	pos;
				int16	time;
			};
			int			num_locs;
			Location	locs[64];
		};
		Fire	fire;
	};

    Blinky() {}

	void	fade(uint8 b) {
		fadeToBlackBy(leds, NUM_LEDS, b);
	}
	void	add_range(int i, int n, CRGB rgb) {
        while (n--)
        	leds[i++] += rgb;
	}
	void	set_range(int i, int n, CRGB rgb) {
        while (n--)
        	leds[i++] = rgb;
	}

	CRGB	curr_rgb()	const { return CRGB(pot[0] >> 2, pot[1] >> 2, pot[2] >> 2); }
	CHSV	curr_hsv()	const { return CHSV(pot[0] >> 2, pot[1] >> 2, max(pot[2] >> 2, 1u)); }

	static uint8 to8u(int16 x) 	{ return (uint16(x) + 0x8000) >> 8; }
	static uint8 to8u2(int16 x) { return to8u(x) / 2 + 0x80; }
	static CRGB grey(uint8 x)	{ return CRGB(x, x, x); }

    void setup() {
		FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
//		FastLED.setBrightness(BRIGHTNESS);
		for (int i = 0; i < 6; i++)
        	pinMode(A0 + i, INPUT);

		clear(leds);
	}
    void loop() {
        FastLED.show();  
        FastLED.delay(1000/FRAMES_PER_SECOND);

		for (int i = 0; i < 6; i++)
			pot[i].update(frame, 1023 - analogRead(A0 + i));

		int	n = pot[4];
		int	m = pot[5];
		Serial.println(pot[2]>>2);

		int		m0 = pot[3] * 8;
		bool	mb = mode.update(m0);
		if (mb && (mode_hold == 0 || mode_hold > 10))
			mode_hold = FRAMES_PER_SECOND / 2;

		bool	mode_init = mode_hold == 10;
		if (mode_hold && mode_hold-- > 10) {
			clear(leds);
			int		m 	= mode, f = mode.frac();
			CRGB	col = CHSV(m * 32, 255, 255);
			for (int i = 0; i < 8; i++)
				leds[m * 8 + i] = col % uint8(max(255 - abs(i * 32 - f), 0));

		} else switch (mode) {
			case 0:		// solid colour HSV
				set_range(0, NUM_LEDS - 1, CRGB(curr_hsv()));
				break;

			case 1:		// solid colour RGB
				set_range(0, NUM_LEDS - 1, curr_rgb());
				break;

			case 2: {
				auto	hsv		= curr_hsv();
				CRGB	col1	= hsv;
				CRGB	col2	= CHSV(hsv.h + (n >> 2), hsv.s, hsv.v);
				CRGB	col3	= CHSV(hsv.h + (m >> 2), hsv.s, hsv.v);
				uint32	t 		= millis();
				uint16	phase1 	= t * 64;
				uint16	phase2 	= ~t * 3 / 2;
				uint16	phase3 	= ~t * 2;
				uint16	freq1	= 0x0300;
				uint16	freq2	= 0x0700;
				uint16	freq3	= 0x0500;
				for (int i = 0; i < NUM_LEDS; i++) {
					leds[i] = col1 % to8u(sin16(phase1)) + col2 % to8u2(sin16(phase2)) + col3 % to8u2(sin16(phase3));
					phase1 += freq1;
					phase2 += freq2;
					phase3 += freq3;
				}
				break;
			}
			case 3: {//dither
				CRGB	cols[2] = {
					CRGB(0,0,0),
					curr_hsv()
				};
				int	e	= 0;
				n 	= max(n, (1024 + NUM_LEDS - 1) / NUM_LEDS);
				for (int i = 0; i < NUM_LEDS; i++) {
					e += n;
					leds[i] = cols[e >> 10];
					e &= 1023;
				}
				break;
			}
			case 4: {	// sparkles
				if (mode_init) {
					num_locs = 64;
					for (int i = 0; i < num_locs; i++) {
						locs[i].pos = random(NUM_LEDS);
						locs[i].time = random(1024);
					}
				}
				clear(leds);
				m = max(m - 16, 0);
				lastpos += m * 8;
				int		t 	= (lastpos >> 8) + n;
				CRGB	col = curr_hsv();
				for (int i = 0; i < num_locs; i++) {
					int	dt = t - (int)locs[i].time;
					if (dt <= -1024 || dt >= 256) {
						if (m) {
							locs[i].pos = random(NUM_LEDS);
							locs[i].time = t + random(1024);
						}
					} else if (dt > 0) {
						leds[locs[i].pos] = col % uint8((127 - abs(dt - 128)) * 2);
					}
				}
				break;
			}
			case 5: {	//single block of leds
				clear(leds);
				int		num 	= (m >> 3) + 1;
				long	pos		= (n * (long)NUM_LEDS) / (1024 / 256);
				uint16	ipos 	= pos >> 8;
				uint8	fpos 	= pos & 255;
				CRGB	rgb		= curr_hsv();
				leds[ipos] = rgb % uint8(255 - fpos);
				if (ipos + num < NUM_LEDS) {
					set_range(ipos + 1, num - 1, rgb);
				} else {
					set_range(ipos + 1, NUM_LEDS - ipos - 1, rgb);
					set_range(0, ipos + num - NUM_LEDS, rgb);
				}
				ipos += num;
				if (ipos >= NUM_LEDS)
					ipos -= NUM_LEDS;
				leds[ipos] = rgb % fpos;
				break;
			}

			case 6: {	//Fire2012
				if (mode_init)
					fire.init();
				fire.update();
				fire.read(leds, curr_hsv().v);
				break;
			}

			case 7: {	// a colored dot sweeping back and forth, with fading trails
				fade(20);
				int pos = beatsin16(5, 0, NUM_LEDS * 2);
				if (pos >= NUM_LEDS)
					pos = NUM_LEDS * 2 - 1 - pos;

				if (lastpos > pos)
					add_range(pos, lastpos - pos, curr_hsv());
				else
					add_range(lastpos, pos - lastpos, curr_hsv());

				lastpos = pos;
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



