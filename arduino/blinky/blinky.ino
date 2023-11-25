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
#define ANAL_VAL 	A2
#define ANAL_MODE	A3
#define ANAL_B 		A4
#define ANAL_C 		A5

#define DATA_PIN    2
#define CLK_PIN   4
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    300 // 100

#define BRIGHTNESS         100
#define FRAMES_PER_SECOND  120

#undef min
#undef max
#undef abs

template<typename T> void clear(T &t) 		{ memset(&t, 0, sizeof(T)); }
template<typename A, typename B> A min(A a, B b) 		{ return a < b ? a : b; }
template<typename A, typename B> A max(A a, B b) 		{ return a < b ? b : a; }
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
	T	val;
	void	update(T v) { val = (val * (N - 1) + v + N / 2) / N; }
	operator T() const { return val; }
};

CHSV rgb_to_hsv(uint8 r, uint8 g, uint8 b) {
	uint8	v = max(max(r, g), b);
	uint8	c = v - min(min(r, g), b);

	return	CHSV(
		c == 0 ? 0 : (
			v == r ?	int(g) - int(b)
		:	v == g ?	int(b) - int(r)
		:				int(r) - int(g)
		) * 128 / (c * 3) + (v == r ? 0 : v == g ? 1 : 2) * 256 / 3,
		v == 0 ? 0 : c * 255u / v,
		v
	);
}

CRGB hsv_to_rgb(uint8 h, uint8 s, uint8 v) {
	uint8	c = (v * s) >> 8;
	uint8	m = v - c, x = v, y = v;
	uint16	i = h * 3;

	if (i & 0x80)
		x -= (c * ( i & 0x7f)) >> 7;
	else
		y -= (c * (~i & 0x7f)) >> 7;

	switch (i >> 8) {
		case 0:	return CRGB(x, y, m);
		case 1:	return CRGB(m, x, y);
		case 2:	return CRGB(y, m, x);
	}
}
CHSV to_hsv(const CRGB &x) {
	return rgb_to_hsv(x.r, x.g, x.b);
}

CRGB to_rgb(const CHSV &x) {
	return hsv_to_rgb(x.h, x.s, x.v);
}

CRGB Tint(CRGB c, CHSV tint) {
	CHSV	c2 = to_hsv(c);
	return hsv_to_rgb(c2.h + tint.h, scale8(c2.s, tint.s), scale8(c2.v, tint.v));
}

CRGB HeatColor2( uint8_t temperature) {
	uint16	t = temperature * 3;
	return CRGB(
		min(t, 255u),
		t < 256 ? 0 : min(t - 256, 255u),
		t < 512 ? 0 : min(t - 512, 255u)
	);
}

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
	void	read(CRGB *leds, CHSV tint) {
    	for (int i = 0; i < NUM; i++)
		    leds[i] = Tint(HeatColor2(min(heat[i] * 2, 255)), tint);
	}
};

template<typename T> HardwareSerial &operator<<(HardwareSerial &serial, const T &t) {
	serial.print(t);
	return serial;
}

struct Blinky {
	CRGB 		leds[NUM_LEDS];
	Averaged<uint16,16>	pot[6];
	uint32		lastpos;
	uint8		mode_hold;

	ModeChanger	mode;

	struct Location {
		uint16	pos:10, hue:6;
		int16	time;
	};

	union {
		struct {
			int			num_locs;
			Location	locs[64];
		};
		Fire	fire;
		struct {
			uint16 counter1, counter2;
		};
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
	#if 0
		for (int r = 0; r < 256; r += 8)
		for (int g = 0; g < 256; g += 8)
		for (int b = 0; b < 256; b += 8) {
			CHSV	hsv = rgb_to_hsv(r, g, b);
			CRGB	rgb = to_rgb(hsv);
			if (abs(r - rgb.r) > 6 || abs(g - rgb.g) > 6 || abs(b - rgb.b) > 6)
				(Serial
					<< "(" << r << "," << g << "," << b
					<< ") to (" << hsv.h << "," << hsv.s << "," << hsv.v << ")"
					<< ") to (" << rgb.r << "," << rgb.g << "," << rgb.b << ")"
				).println();
		}
	#endif
		
	}
    void loop() {
        FastLED.show();  
        FastLED.delay(1000/FRAMES_PER_SECOND);
#if 0
		for (int i = 0; i < 6; i++)
			pot[i].update(1023 - analogRead(A0 + i));
#else
		static const uint16 pots0[] = {1023, 1023, 1023, 0x0000, 0x0000, 0x0000};
		for (int i = 0; i < 6; i++) {
      if (pots0[i] !=0xffff)
        pot[i].update(pots0[i]);
      else
        pot[i].update(1023 - analogRead(A0 + i));
		}
#endif
		int	n = pot[4];
		int	m = pot[5];
		Serial.println(analogRead(A3));

		bool	mb = mode.update(analogRead(A3) * 8);
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
			case 5:	// coloured sparkles
				if (mode_init) {
					num_locs = 64;
					for (int i = 0; i < num_locs; i++) {
						locs[i].pos = random(NUM_LEDS);
						locs[i].hue = random(64);
						locs[i].time = random(1024);
					}
				}
				clear(leds);
				m = max(m - 16, 0);
				lastpos += m * 8;
				int		t 	= (lastpos >> 8) + n;
				CHSV	hsv = curr_hsv();
				CRGB	col = hsv;
				for (int i = 0; i < num_locs; i++) {
					int	dt = t - (int)locs[i].time;
					if (dt <= -1024 || dt >= 256) {
						if (m) {
							locs[i].pos = random(NUM_LEDS);
							locs[i].hue = random(64);
							locs[i].time = t + random(1024);
						}
					} else if (dt > 0) {
						uint8	v = uint8((127 - abs(dt - 128)) * 2);
						if (mode == 5)
							leds[locs[i].pos] = CHSV(locs[i].hue * 4, hsv.s, scale8(hsv.v, v));
						else
							leds[locs[i].pos] = col % v;
					}
				}
				break;
			}
			case 6: {	//single block of leds
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

			case 7: {	//Fire2012
				if (mode_init)
					fire.init();
				fire.update();
				fire.read(leds, curr_hsv());
				break;
			}

			case 8: {	// a colored dot sweeping back and forth, with fading trails
				if (mode_init)
					lastpos = 0;
				fade(20);
				counter1 += n;
				counter2 += m;
				uint16	beat = sin16(counter1) + 32768;
				int pos = scale16(beat, NUM_LEDS * 2);
				if (pos >= NUM_LEDS)
					pos = NUM_LEDS * 2 - 1 - pos;

				CHSV	col((pot[0] >> 2) + (counter2 >> 8), pot[1] >> 2, max(pot[2] >> 2, 1u));
				if (lastpos > pos)
					add_range(pos, lastpos - pos, col);
				else
					add_range(lastpos, pos - lastpos, col);

				lastpos = pos;
				break;
			}

		}
    }
} blinky;

void setup() {
	Serial.begin(9600);
    delay(500); // 0.5 second delay for recovery
    blinky.setup();
}
void loop() {
	Serial.println("hello");
    blinky.loop();
}
