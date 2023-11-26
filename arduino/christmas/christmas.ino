#include <FastLED.h>
#include <WiFi.h>
#include <WiFiUdp.h>

FASTLED_USING_NAMESPACE

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

/* ----------------------- MODIFIERS ------------------ */

#define MOD_PAT 0
#define MOD_FPS 1
#define MOD_BRI 2

#define MOD_HUE 4
#define MOD_SAT 5

/* ----------------------- WIFI ----------------------- */
const char* ssid = "The LAN Before Time 2.4G";
const char* password = "littlefootbigdreams";
const uint udpPort = 123;

WiFiUDP Udp;
#define PACKET_BUFFER_LEN 256
uint8_t packetBuffer[PACKET_BUFFER_LEN]; //buffer to hold incoming packet

void setup_wifi() {
  // Setup Wifi as Station
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

	// Wait for the WiFi connection to continue.
  Serial.println("\nConnecting");
  while(WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100); // Wait 100 ms.
  }

	// Display IP information.
  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());

  // Start up the UDP port.
  Udp.begin(udpPort);
}

void read_udp_packet(void(*use_buffer)(uint8_t*, int)) {
  int packetSize = Udp.parsePacket();
  // Only read the data if the packet is small enough to be read into the buffer.
  if (packetSize && packetSize < PACKET_BUFFER_LEN << 1) {
		// Use the buffer.
    int len = Udp.read(packetBuffer, PACKET_BUFFER_LEN);
		use_buffer(packetBuffer, len);
  }
}

/* ------------------- FASTLED ------------------------ */

#define DATA_PIN 3
#define LED_TYPE WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS 300

/* ------------------ FASTLED/GARBAGE ----------------- */

#undef min
#undef max
#undef abs
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

template<typename T> void		   clear(T& t) { memset(&t, 0, sizeof(T)); }
template<typename A, typename B> A min(A a, B b) { return a < b ? a : b; }
template<typename A, typename B> A max(A a, B b) { return a < b ? b : a; }
template<typename T> T			   clamp(T t, T a, T b) { return min(max(t, a), b); }
template<typename T> T			   abs(T a) { return a < 0 ? -a : a; }
template<typename T> bool		   between(T x, T a, T b) { return a <= x && x <= b; }

int16_t mul16(int16_t a, int16_t b) { return (int32_t(a) * b) >> 16; }
int16_t div16(int16_t a, int16_t b) { return (int32_t(a) << 16) / b; }

float lerp(float a, float b, float t) { return a + t * (b - a); }

CHSV rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b) {
	uint8_t v = max(max(r, g), b);
	uint8_t c = v - min(min(r, g), b);
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

CRGB hsv_to_rgb(uint8_t h, uint8_t s, uint8_t v) {
	uint8_t	 c = (v * s) >> 8;
	uint8_t	 m = v - c, x = v, y = v;
	uint16_t i = h * 3;

	if (i & 0x80)
		x -= (c * (i & 0x7f)) >> 7;
	else
		y -= (c * (~i & 0x7f)) >> 7;

	switch (i >> 8) {
		case 0: return CRGB(x, y, m);
		case 1: return CRGB(m, x, y);
		case 2: return CRGB(y, m, x);
	}
}

CHSV to_hsv(const CRGB& x) { return rgb_to_hsv(x.r, x.g, x.b); }
CRGB to_rgb(const CHSV& x) { return hsv_to_rgb(x.h, x.s, x.v); }

uint8_t to8u(int16_t x) { return (uint16_t(x) + 0x8000) >> 8; }
uint8_t to8u2(int16_t x) { return to8u(x) / 2 + 0x80; }
CRGB grey(uint8_t x) { return CRGB(x, x, x); }

CRGB Tint(CRGB c, CHSV tint) {
	CHSV c2 = to_hsv(c);
	return hsv_to_rgb(c2.h + tint.h, scale8(c2.s, tint.s), scale8(c2.v, tint.v));
}

int16_t to_hex(char x) {
	return  between(x, '0', '9') ? x - '0'
    :   between(x, 'a', 'f') ? x - 'a' + 10
    :   between(x, 'A', 'F') ? x - 'A' + 10
    :   0;
}

int16_t to_hex4(const char* p) { return to_hex(p[0]) * 0x1000 + to_hex(p[1]) * 0x100 + to_hex(p[2]) * 0x10 + to_hex(p[3]); }

int32_t to_hex(const char* p, int n) {
	int32_t r = 0;
	while (n--)
		r = (r << 4) | to_hex(*p++);
	return r;
}

// Debug print
#define DBGP(v) Serial.print(#v); Serial.print(" = "); Serial.println(v)

/* ------------------------------------------------------------------ *
 | Main Program
 * ------------------------------------------------------------------ */

struct Main;

struct Pattern {
	virtual void tick(Main* program) = 0;
	virtual ~Pattern() {}
};

struct PatternDef {
  static char space[2048];
  Pattern* (*make)();

	template<typename T> static Pattern* _make() {
    return new(space) T;
  }

	template<typename T> PatternDef(T*) : make(_make<T>) {
		static_assert(sizeof(T) <= sizeof(space), "not enough space");
	}
};

char PatternDef::space[2048];

struct Main {
	enum { NUM_MODS = 32 };
	CRGB leds[NUM_LEDS];
	int	fps = 120;
	CHSV hsv = CHSV(128, 128, 128);
	int16_t	m[NUM_MODS];
	Pattern *pattern = nullptr;

	void set_pattern(int p);
	void set_variable(uint16_t id, int16_t value);

	void set_range(int i, int n, CRGB rgb) {
		while (n--) {
			leds[i++] = rgb;
		}
	}

	void add_range(int i, int n, CRGB rgb) {
		while (n--) {
			leds[i++] += rgb;
		}
	}

	void setup() {
		FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
		clear(leds);
	}

  /* ---------- */

  void read_udp_packet() {
    int packetSize = Udp.parsePacket();
    // Only read the data if the packet is small enough to be read into the buffer.
    if (packetSize && packetSize < PACKET_BUFFER_LEN << 1) {
      // Use the buffer.
      int len = Udp.read(packetBuffer, PACKET_BUFFER_LEN);

      // For each pair of numbers put the first number into
      // the array, and then the second as its value.
      const uint16_t *items = (uint16_t*) packetBuffer;
      const uint16_t num_pairs = (len >> 2);

      for (int pair_i = 0; pair_i < num_pairs; ++pair_i) {
        const uint16_t item_index = (pair_i << 1);
        const uint16_t id = items[item_index];
        const int16_t value = items[item_index + 1];

        if (id < NUM_MODS) {
          Serial.print("[");
          Serial.print(id);
          Serial.print(", ");
          Serial.print(value);
          Serial.println("]");
          set_variable(id, value);
        }
      }
    }
  }

  /* ---------- */

	void tick() {
    read_udp_packet();
		if (pattern) {
			pattern->tick(this);
		}

		leds[547] = CRGB(255,255,255);
		//leds[547] = CRGB(0,0,0);

		FastLED.show();
		FastLED.delay(1000 / fps);
	}
} program;

void setup() {
	delay(500);	 // 0.5 second delay for recovery
	Serial.begin(115200);

	setup_wifi();
	program.setup();
	program.set_range(0, NUM_LEDS - 1, to_rgb(program.hsv));
}

void loop() {
	//Serial.println("loop");
	// read_udp_packet(set_variables_from_buffer);
	program.tick();
}

/* ------------------------------------------------------------------------- *\
|  Perlin
\* ------------------------------------------------------------------------- */

struct PatternPerlin : Pattern {
	enum { MAX_PERLIN = 32 };
	uint8_t perlin_array_h1[MAX_PERLIN], perlin_array_h0[MAX_PERLIN];
	uint8_t perlin_array_s1[MAX_PERLIN], perlin_array_s0[MAX_PERLIN];
	uint8_t perlin_array_v1[MAX_PERLIN], perlin_array_v0[MAX_PERLIN];

	uint8_t *py0_h, *py1_h;
	uint8_t *py0_s, *py1_s;
	uint8_t *py0_v, *py1_v;
	uint32_t ti_h = 0;
	uint32_t ti_s = 0;
	uint32_t ti_v = 0;

	static float perlin_fade(float t) { return 6 * t * t * t * t * t - 15 * t * t * t * t + 10 * t * t * t; }

	static float dotGridGradient(int h, float dx, float dy) {
		static const float xvec[] = {0, 0, 1, -1};
		static const float yvec[] = {1, -1, 0, 0};
		return xvec[h] * dx + yvec[h] * dy;
	}

	static uint8_t* assign_random(uint8_t* p) {
		for (int i = 0; i < MAX_PERLIN; ++i)
			p[i] = random(4);
		return p;
	}

	static void update_perlin_arrays(uint8_t*& p0, uint8_t*& p1) {
		uint8_t* temp = p0;
		p0 = p1;
		p1 = assign_random(temp);
	}

	static float perlin_value(float tf, float v, float x, uint8_t* p0, uint8_t* p1) {
		// break up x into integer and non-integer components
		uint8_t xi = x;
		float	xf = x - xi;
		float	u  = perlin_fade(xf);

		float ix0 = lerp(dotGridGradient(p0[xi], xf, tf), dotGridGradient(p0[xi + 1], xf - 1, tf), u);
		float ix1 = lerp(dotGridGradient(p1[xi], xf, tf - 1), dotGridGradient(p1[xi + 1], xf - 1, tf - 1), u);

		// squish perlin value into approximate range [0, 1]
		return 0.5 + lerp(ix0, ix1, v) / 1.07f;
	}

	static int perlin_value1(float tf, float v, int i, int chunks, int min, int max, uint8_t* p0, uint8_t* p1) {
         return (int)lerp(min, max, perlin_value(tf, v, float(i) / NUM_LEDS * (chunks % (MAX_PERLIN + 1)), p0, p1));
    }

	PatternPerlin() {
		py0_h = assign_random(perlin_array_h0);
		py1_h = assign_random(perlin_array_h1);
		py0_s = assign_random(perlin_array_s0);
		py1_s = assign_random(perlin_array_s1);
		py0_v = assign_random(perlin_array_v0);
		py1_v = assign_random(perlin_array_v1);
	}
	void tick(Main* program) {
		float t	  = millis();
		float t_h = t / program->m[0];
		float t_s = t / program->m[4];
		float t_v = t / program->m[8];

		if (ti_h != (uint32_t)t_h) {
			ti_h = (uint32_t)t_h;
			update_perlin_arrays(py0_h, py1_h);
		}

		if (ti_s != (uint32_t)t_s) {
			ti_s = (uint32_t)t_s;
			update_perlin_arrays(py0_s, py1_s);
		}

		if (ti_v != (uint32_t)t_v) {
			ti_v = (uint32_t)t_v;
			update_perlin_arrays(py0_v, py1_v);
		}

		float tf_h = t_h - ti_h, v_h = perlin_fade(tf_h);
		float tf_s = t_s - ti_s, v_s = perlin_fade(tf_s);
		float tf_v = t_v - ti_v, v_v = perlin_fade(tf_v);

		for (int i = 0; i < NUM_LEDS; ++i) {
			program->leds[i] = CHSV(
        perlin_value1(tf_h, v_h, i, program->m[3], program->m[1], program->m[2], py0_h, py1_h),
				perlin_value1(tf_s, v_s, i, program->m[7], program->m[5], program->m[6], py0_s, py1_s),
				perlin_value1(tf_v, v_v, i, program->m[11], program->m[9], program->m[10], py0_v, py1_v)
      );
		}
	}
};

/* ------------------------------------------------------------------------- *\
|  Solid Colour HSV
\* ------------------------------------------------------------------------- */

struct PatternSolidHSV : Pattern {
	void tick(Main* program) {
    program->set_range(0, NUM_LEDS - 1, to_rgb(program->hsv));
  }
};

struct PatternSolidRGB : Pattern {
	void tick(Main* program) {
    program->set_range(0, NUM_LEDS - 1, CRGB(program->hsv.h, program->hsv.s, program->hsv.v));
	}
};

/* ------------------------------------------------------------------------- *\
|  Solid Colour HSV
\* ------------------------------------------------------------------------- */

struct Pattern2 : Pattern {
	void tick(Main* program) {
    // h    = hue of wave 1
    // m[0] = frequency of wave 1
    // m[1] = wavelength of wave 1

    // m[2] = hue offset of wave 2
    // m[3] = frequency of wave 2
    // m[4] = wavelength of wave 2

    // m[5] = hue offset of wave 3
    // m[6] = frequency of wave 3
    // m[7] = wavelength of wave 3

		CRGB col1 = program->hsv;
		CRGB col2 = CHSV(program->hsv.h + program->m[2], program->hsv.s, program->hsv.v);
		CRGB col3 = CHSV(program->hsv.h + program->m[5], program->hsv.s, program->hsv.v);

		uint32_t t		= millis();
    uint16_t phase1 = t * program->m[0];	 // * 64;
    uint16_t phase2 = t * program->m[3];	 // * 3 / 2;
    uint16_t phase3 = t * program->m[6];	 //* 2;

    uint16_t freq1 = program->m[1];	// 0x0300;
    uint16_t freq2 = program->m[3];	// 0x0700;
    uint16_t freq3 = program->m[7];	// 0x0500;

    for (auto &led : program->leds) {
      led = col1 % to8u(sin16(phase1)) + col2 % to8u2(sin16(phase2)) + col3 % to8u2(sin16(phase3));
			phase1 += freq1;
      phase2 += freq2;
      phase3 += freq3;
    }
  }
};

/* ------------------------------------------------------------------------- *\
|  Dither
\* ------------------------------------------------------------------------- */

struct PatternDither : Pattern {
	void tick(Main* program) {
    CRGB  cols[2] = {CRGB(0, 0, 0), program->hsv};
    uint16_t N = max(program->m[0], (1024 + NUM_LEDS - 1) / NUM_LEDS);

		int e = 0;
		for (auto& led : program->leds) {
			e += N;
      led = cols[e >> 10];
      e &= 1023;
		}
	}
};

/* ------------------------------------------------------------------------- *\
|  Sparkles
\* ------------------------------------------------------------------------- */

struct PatternSparkles : Pattern {
  enum { MAX_LOCS = 300 };
  struct Location {
    uint16_t pos : 10, hue : 6;
    int16_t	 time;
  };

  uint32_t lastpos = 0;
  Location locs[MAX_LOCS];

	PatternSparkles() {
		Serial.println("start sparkles");
		for (auto &i : locs) {
			i.pos   = random(NUM_LEDS);
			i.hue   = random(0);
			i.time  = random(1024);
		}
	}

	void tick(Main* program) {
		int num_locs = min(program->m[0], MAX_LOCS);
		uint16_t speed = program->m[1];
		uint16_t hue_range = program->m[2];

		clear(program->leds);

		lastpos += speed;
		int t = lastpos >> 8;

		uint8_t h = program->hsv.h;
		uint8_t s = program->hsv.s;
		uint8_t v = program->hsv.v;

		for (int i = 0; i < num_locs; i++) {
			int dt = (int16_t)(t - (int)locs[i].time);

			if (dt <= -1024 || dt >= 256) {
				locs[i].pos	 = random(NUM_LEDS);
				locs[i].hue	 = random(hue_range);
				locs[i].time = t + random(1024);

			} else if (dt > 0) {
				program->leds[locs[i].pos] = to_rgb(CHSV(h + locs[i].hue * 4, s, scale8(v, uint8_t((127 - abs(dt - 128)) * 2))));
			}
		}
	}
};

/* ------------------------------------------------------------------------- *\
|  Single Block Leds
\* ------------------------------------------------------------------------- */

struct PatternBlock : Pattern {
	void tick(Main* program) {
		clear(program->leds);
		int num = (program->m[0] >> 3) + 1;
		long pos = (program->m[1] * (long)NUM_LEDS) / (1024 / 256);
		uint16_t ipos = pos >> 8;
    uint8_t fpos = pos & 255;
		CRGB rgb  = program->hsv;

		program->leds[ipos] = rgb % uint8_t(255 - fpos);
		if (ipos + num < NUM_LEDS) {
      program->set_range(ipos + 1, num - 1, rgb);
    } else {
			program->set_range(ipos + 1, NUM_LEDS - ipos - 1, rgb);
			program->set_range(0, ipos + num - NUM_LEDS, rgb);
		}

    ipos += num;

    if (ipos >= NUM_LEDS) {
      ipos -= NUM_LEDS;
    }

		program->leds[ipos] = rgb % fpos;
	}
};

/* ------------------------------------------------------------------------- *\
|  Fire
\* ------------------------------------------------------------------------- */

struct PatternFire : Pattern {
	enum { NUM = NUM_LEDS };
	uint8_t     heat[NUM];

	static CRGB HeatColour(uint8_t temperature) {
		uint16_t t = temperature * 3;
    return CRGB(min(t, 255u), t < 256 ? 0 : min(t - 256, 255u), t < 512 ? 0 : min(t - 512, 255u));
	}

	PatternFire() { clear(heat); }

  void tick(Main* program) {
		uint8_t h0 = heat[0];
		for (int i = 1; i < NUM - 1; i++) {
			uint8_t h1 = heat[i];
			h0 = (h0 + h1 + heat[i + 1]) / 3;
			heat[i]	= h0 > 1 && random8() < 128 ? h0 - 1 : h0;
			h0 = h1;
		}
		int y	= random16(NUM);
		heat[y] = qadd8(heat[y], random8(160, 255));

		for (int i = 0; i < NUM; i++) {
			program->leds[i] = Tint(HeatColour(min(heat[i] * 2, 255)), program->hsv);
    }

	}
};

/* ------------------------------------------------------------------------- *\
|  Sweeping Dot
\* ------------------------------------------------------------------------- */

struct PatternSweepingDot : Pattern {
	uint16_t counter1;
	uint16_t counter2;
	uint32_t lastpos = 0;

	void tick(Main* program) {
		fadeToBlackBy(program->leds, NUM_LEDS, 20);

		counter1 += program->m[0];
		counter2 += program->m[1];

		uint16_t beat = sin16(counter1) + 32768;

		int pos = scale16(beat, NUM_LEDS * 2);

		if (pos >= NUM_LEDS) {
			pos = NUM_LEDS * 2 - 1 - pos;
    }

		CRGB col = program->hsv;

		if (lastpos > pos) {
			program->add_range(pos, lastpos - pos, col);
		} else {
			program->add_range(lastpos, pos - lastpos, col);
    }

		lastpos = pos;
	}
};

/* ------------------------------------------------------------------------- *\
|  Rainbow
\* ------------------------------------------------------------------------- */

struct PatternRainbow : Pattern {
	uint8_t rainbow_pos = 0;
	void	tick(Main* program) {
		rainbow_pos += program->m[0] / 100;
		// FastLED's built-in rainbow generator
		fill_rainbow(program->leds, NUM_LEDS, rainbow_pos, 7);
	}
};

/* ------------------------------------------------------------------ *
 | Patterns
 * ------------------------------------------------------------------ */

PatternDef patterns[] = {
    (PatternSolidHSV*)0,
    (Pattern2*)0,
    (PatternDither*)0,
    (PatternSparkles*)0,
    (PatternBlock*)0,
    (PatternFire*)0,
    (PatternSweepingDot*)0,
    (PatternRainbow*)0,
    (PatternPerlin*)0,
    (PatternSolidRGB*)0
};

void Main::set_pattern(int p) {
	if (pattern) {
		pattern->~Pattern();
	}
	pattern = patterns[p % ARRAY_SIZE(patterns)].make();
	clear(m);
}


void Main::set_variable(uint16_t id, int16_t value) {

  Serial.print("INSIDE SET VARIABLE -> [");
  Serial.print(id);
  Serial.print(", ");
  Serial.print(value);
  Serial.println("]");

  switch (id) {

    // Pattern
    case MOD_PAT:
      set_pattern(value);
      break;

    case MOD_FPS:
      fps = value;
      break;

    // Brightness
    case MOD_BRI:
      FastLED.setBrightness(uint8_t(value));
      break;

    case MOD_HUE:
      hsv.h = value;
      break;

    case MOD_SAT:
      hsv.s = value;
      break;

    // Modifiers
    default:
      if ((id-8) < NUM_MODS) {
        Serial.print("Setting m[");
        Serial.print(id-8);
        Serial.print("] = ");
        Serial.print(value);
        Serial.println(";");
        m[id-8] = value;
      }
      break;
  }
}

// void Main::set_variable(char var, int i, int value) {
// 	Serial.printf("set %c[%i] = %i\n", var, i, value);

// 	switch (var) {
// 		// Pattern
// 		case MOD_PAT:
// 			set_pattern(value);
// 			break;

// 		// Brightness
// 		case MOD_BRI:
//       FastLED.setBrightness(uint8_t(value));
//       break;

// 		case MOD_FPS:
//       fps = value;
//       break;

// 		case MOD_HUE:
// 			hsv.h = value;
// 			break;

// 		case MOD_SAT:
// 			hsv.s = value;
// 			break;

// 		// Modifiers
// 		case 'm':
// 			if (i < NUM_MODS)
// 				m[i] = value;
// 			break;
// 	}
// }