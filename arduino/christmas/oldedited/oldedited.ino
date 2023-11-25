#include <FastLED.h>
#include <WiFi.h>

FASTLED_USING_NAMESPACE

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif


/* ----------------------- WIFI ----------------------- */
const char* ssid     = "stephens";
const char* password = "c0406a0222";

WiFiServer wifiServer(12345);

/* ------------------- WIFI/FASTLED ------------------- */

#define DATA_PIN    2
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    600

#undef min
#undef max
#undef abs

typedef void (*Pattern)();

/* ------------------ FASTLED/GARBAGE ----------------- */

static uint8_t to8u(int16_t x)  { return (uint16_t(x) + 0x8000) >> 8; }
static uint8_t to8u2(int16_t x) { return to8u(x) / 2 + 0x80; }
static CRGB    grey (uint8_t x)  { return CRGB(x, x, x); }

template<typename T> void clear(T &t)            { memset(&t, 0, sizeof(T)); }
template<typename A, typename B> A min(A a, B b) { return a < b ? a : b; }
template<typename A, typename B> A max(A a, B b) { return a < b ? b : a; }
template<typename T> T clamp(T t, T a, T b)      { return min(max(t, a), b); }
template<typename T> T abs(T a)                  { return a < 0 ? -a : a; }

int16_t	mul16(int16_t a, int16_t b) { return (int32_t(a) * b) >> 16; }
int16_t	div16(int16_t a, int16_t b) { return (int32_t(a) << 16) / b; }

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

// Debug print
#define DBGP(v) Serial.print(#v); Serial.print(" = "); Serial.println(v)

/* ------------------------------------------------------------------ *
 | Main Program
 * ------------------------------------------------------------------ */

CRGB leds[NUM_LEDS];
uint32_t lastpos;

Pattern patterns[] = {
	solid_colour_hsv,
	case2,
	dither,
	sparkles, 
    single_block_leds,
	fire2012,
	sweeping_dot,
	rainbow,
    perlin,
	solid_colour_rgb
};

uint8_t p 	= 0;
int		fps = 120;
CHSV 	hsv = CHSV(128, 128, 128); // should be yellow / orange
bool 	mode_init	= true;

// Modifiers
#define NUM_MODS 16
int16_t m[NUM_MODS];

void setup() 
{    
    delay(500); // 0.5 second delay for recovery

    // Setup serial port
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Serial port ready!");

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

    clear(leds);

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    wifiServer.begin();

	for (int i = 0; i < 255; i++) {
		CHSV	hsv(170, i, 255);
		auto	rgb = to_rgb(hsv);
		Serial.printf("s=%i: r=%i, g=%i, b=%i\n", i, rgb.r, rgb.g, rgb.b);
	}

}

int16_t to_hex4 (char n1000, char n100, char n10, char n1)
{
    return to_hex(n1000) * 0x1000
         + to_hex(n100) * 0x100 
         + to_hex(n10) * 0x10 
         + to_hex(n1);
}

int16_t to_hex (char num)
{
    if (('0' <= num) && (num <= '9')) { return num-'0';}
    else if (('a' <= num) && (num <= 'f')) { return num-'a'+10;}
    else if (('A' <= num) && (num <= 'F')) { return num-'A'+10;}
    else { return 0;}
}

void set_variable (char var, int i, int value)
{
    switch (var) {

        // Pattern
        case 'p': 
            p = value %  ARRAY_SIZE(patterns);
            DBGP(p); 
            mode_init = true; 
			clear(m);
            break;

        // Brightness
        case 'b': 
            FastLED.setBrightness(uint8_t(value));
            break;

		case 'f':
			fps	= value;
			break;

        // HSV
        case 'h': hsv.h = value; DBGP(hsv.h); break;
        case 's': hsv.s = value; DBGP(hsv.s); break;
        case 'v': hsv.v = value; DBGP(hsv.v); break;

        // Modifiers

        case 'm':
		 	if (i < NUM_MODS) {
				m[i] = value;
				Serial.print("m["); Serial.print(i); Serial.print("] = "); Serial.println(value);
			}
			break;

    }
}

void loop() {
    
    /* WIFI code */
    WiFiClient c = wifiServer.available();

    if (c) {
		while (c.connected()) {
			while (c.available() > 0) {
				char	com[6];
				for (auto &i : com)
					i = c.read();
				set_variable(com[0], to_hex(com[1]), to_hex4(com[2], com[3], com[4], com[5]));
			}
			delay(10);
        }
	    c.stop();
    }
        
    patterns[p]();

    FastLED.show();  
    FastLED.delay(1000 / fps);
    mode_init = false;
}

/* ------------------------------------------------------------------ *
 | Patterns
 * ------------------------------------------------------------------ */

/* ------------------------------------------------------------------------- *\
|  Perlin
\* ------------------------------------------------------------------------- */

static float perlin_fade (float t) { return 6*t*t*t*t*t - 15*t*t*t*t + 10*t*t*t; }

#define MAX_PERLIN 32
// #define CHUNKS_H 16
// #define CHUNKS_S 16
// #define CHUNKS_V 16

// #define PMIN_H 0
// #define PMAX_H 255

// #define PMIN_S 120
// #define PMAX_S 150

// #define PMIN_V 0
// #define PMAX_V 170

// #define SWAPRATE_H 1000.00 // once  second
// #define SWAPRATE_S  250.00 // 4 times per second 
// #define SWAPRATE_V  500.00  // 2 times per second

#define lerp(a,b,x) (a+x*(b-a))
#define PERLIN_RAND_TEST 10

uint8_t perlin_array_h1 [MAX_PERLIN];
uint8_t perlin_array_h0 [MAX_PERLIN];

uint8_t perlin_array_s1 [MAX_PERLIN];
uint8_t perlin_array_s0 [MAX_PERLIN];

uint8_t perlin_array_v1 [MAX_PERLIN];
uint8_t perlin_array_v0 [MAX_PERLIN];

float xvec[] = {0,0,1,-1};
float yvec[] = {1,-1,0,0};

float dotGridGradient (int h, float dx, float dy) 
{
    return xvec[h]*dx + yvec[h]*dy;
}

uint8_t *assign_random (uint8_t *p) 
{
    for (int i = 0; i < MAX_PERLIN; ++i) p[i] = random(4);
    return p;
}

void update_perlin_arrays (uint8_t **p0, uint8_t **p1)
{
    uint8_t *temp;
    temp  = *p0; *p0 = *p1; *p1 = assign_random(temp);
}

void perlin()
{

    // 
    //
    // CHUNKS_H
    // CHUNKS_S
    // CHUNKS_V
    //
    static bool init = true;
    static uint8_t *py0_h, *py1_h, 
                   *py0_s, *py1_s, 
                   *py0_v, *py1_v;
    
    if (init) {
        
        init = false;

        py0_h = assign_random(perlin_array_h0);
        py1_h = assign_random(perlin_array_h1);
        
        py0_s = assign_random(perlin_array_s0);
        py1_s = assign_random(perlin_array_s1);

        py0_v = assign_random(perlin_array_v0);
        py1_v = assign_random(perlin_array_v1);

    }

    float swaprate_h = m[0]; 
    float swaprate_s = m[4];
    float swaprate_v = m[8];
    
    //unsigned long t = millis();
    float t   = millis();
    float t_h = t/swaprate_h;
    float t_s = t/swaprate_s;
    float t_v = t/swaprate_v;
    
    static uint32_t ti_h = t_h;
    static uint32_t ti_s = t_s;
    static uint32_t ti_v = t_v;

    if (ti_h != (uint32_t)t_h) {
        ti_h  = (uint32_t)t_h;
        update_perlin_arrays(&py0_h, &py1_h);
    }

    if (ti_s != (uint32_t)t_s) {
        ti_s  = (uint32_t)t_s;
        update_perlin_arrays(&py0_s, &py1_s);
    }

    if (ti_v != (uint32_t)t_v) {
        ti_v  = (uint32_t)t_v;
        update_perlin_arrays(&py0_v, &py1_v);
    }

    float tf_h = t_h - ti_h;
    float tf_s = t_s - ti_s;
    float tf_v = t_v - ti_v;

    float v_h  = perlin_fade(tf_h);
    float v_s  = perlin_fade(tf_s);
    float v_v  = perlin_fade(tf_v);

    float H;
    int   Hi;

    float S;
    int   Si;

    float V;
    int   Vi;

    for (int i = 0; i < NUM_LEDS; ++i) 
    {
        // m[1] = MIN_H
        // m[2] = MAX_H
        // m[3] = NUM_CHUNKS_H
        H  = perlin_value(tf_h, v_h, 
                          ((float)i)/NUM_LEDS*(m[3]%(MAX_PERLIN+1)),
                          py0_h, py1_h);
        Hi = m[1]+(m[2]-m[1])*H; 

        // m[5] = MIN_S
        // m[6] = MAX_S
        // m[7] = NUM_CHUNKS_S
        S  = perlin_value(tf_s, v_v, 
                          ((float)i)/NUM_LEDS*(m[7]%(MAX_PERLIN+1)),
                          py0_s, py1_s);
        Si = m[5]+(m[6]-m[5])*S;

        // m[9] = MIN_V
        // m[10] = MAX_V
        // m[11] = NUM_CHUNKS_V
        V  = perlin_value(tf_v, v_v, 
                          ((float)i)/NUM_LEDS*(m[11]%(MAX_PERLIN+1)), 
                          py0_v, py1_v);
        Vi = m[9]+(m[10]-m[9])*V;

        leds[i] = CHSV(Hi,Si,Vi);
    }

}


float perlin_value (float tf, float v, float x, uint8_t *py0, uint8_t *py1) 
{

    // break up x into integer and non-integer components
    uint8_t xi = x; 
    float   xf = x - xi;

    float n0, n1, ix0, ix1, u;

    u = perlin_fade(xf);

    n0  = dotGridGradient(py0[xi]  , xf  , tf);
    n1  = dotGridGradient(py0[xi+1], xf-1, tf);
    ix0 = lerp(n0,n1,u);

    n0  = dotGridGradient(py1[xi]  , xf  , tf-1);
    n1  = dotGridGradient(py1[xi+1], xf-1, tf-1);
    ix1 = lerp(n0,n1,u);

    // squish perlin value into approximate range [0, 1]
    return 0.5 + lerp(ix0, ix1, v)/1.07;
}

/* ------------------------------------------------------------------------- *\
|  Solid Colour HSV
\* ------------------------------------------------------------------------- */

void solid_colour_hsv() 
{
    set_range(0, NUM_LEDS - 1, to_rgb(hsv));
}

void solid_colour_rgb() 
{
    set_range(0, NUM_LEDS - 1, CRGB(hsv.h, hsv.s, hsv.v));
}

/* ------------------------------------------------------------------------- *\
|  Solid Colour HSV
\* ------------------------------------------------------------------------- */

void case2 () 
{
    // h    = hue of wave 1
	// m[0] = frequency of wave 1
    // m[1] = wavelength of wave 1
 
	// m[2] = hue offset of wave 2
    // m[3] = frequency of wave 2
    // m[4] = wavelength of wave 2

    // m[5] = hue offset of wave 3
    // m[6] = frequency of wave 3
    // m[7] = wavelength of wave 3

    CRGB col1  = hsv;
    CRGB col2  = CHSV(hsv.h + m[2], hsv.s, hsv.v);
    CRGB col3  = CHSV(hsv.h + m[5], hsv.s, hsv.v);
    
    uint32_t t = millis();
    uint16_t phase1 = t * m[0]; // * 64;
    uint16_t phase2 = t * m[3]; // * 3 / 2;
    uint16_t phase3 = t * m[6]; //* 2;

    uint16_t freq1  = m[1]; // 0x0300;
    uint16_t freq2  = m[3]; // 0x0700;
    uint16_t freq3  = m[7]; // 0x0500;

    for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = col1 % to8u(sin16(phase1)) + col2 % to8u2(sin16(phase2)) + col3 % to8u2(sin16(phase3));
        phase1 += freq1;
        phase2 += freq2;
        phase3 += freq3;
    }
}

/* ------------------------------------------------------------------------- *\
|  Dither
\* ------------------------------------------------------------------------- */

void dither () 
{
    CRGB cols[2] = {
        CRGB(0,0,0),
        hsv
    };
    
    int	e = 0;
    uint16_t N = max(m[0], (1024 + NUM_LEDS - 1) / NUM_LEDS);

    for (int i = 0; i < NUM_LEDS; i++) {
        e += N;
        leds[i] = cols[e >> 10];
        e &= 1023;
    }
}

/* ------------------------------------------------------------------------- *\
|  Sparkles
\* ------------------------------------------------------------------------- */

#define MAX_LOCS 300
struct Location {
    uint16_t pos:10, hue:6;
    int16_t	 time;
};
Location locs[MAX_LOCS];

void sparkles () 
{   
	int			num_locs 	= min(m[0], MAX_LOCS);
	uint16_t	speed 		= m[1];
    uint16_t 	hue_range	= m[2];

    if (mode_init) {
        for (int i = 0; i < MAX_LOCS; i++) {
            locs[i].pos = random(NUM_LEDS);
            locs[i].hue = random(hue_range);
            locs[i].time = random(1024);
        }
    }

    clear(leds);
    
    lastpos += speed;
    int t = lastpos >> 8;

    for (int i = 0; i < num_locs; i++) {
        
        int	dt = (int16_t)(t-(int)locs[i].time);
              
        if (dt <= -1024 || dt >= 256) {
			locs[i].pos = random(NUM_LEDS);
			locs[i].hue = random(hue_range);
			locs[i].time = t + random(1024);

        } else if (dt > 0) {
            uint8_t v = uint8_t((127 - abs(dt - 128)) * 2);3-0-cameron
            leds[locs[i].pos] = to_rgb(CHSV(hsv.h + locs[i].hue * 4, hsv.s, scale8(hsv.v, v)));
        }
    } 
}

/* ------------------------------------------------------------------------- *\
|  Single Block Leds
\* ------------------------------------------------------------------------- */

void single_block_leds () 
{
    clear(leds);
    int      num    = (m[0] >> 3) + 1;
    long     pos    = (m[1] * (long)NUM_LEDS) / (1024 / 256);
    uint16_t ipos   = pos >> 8;
    uint8_t  fpos   = pos & 255;
    CRGB     rgb    = hsv;

    leds[ipos] = rgb % uint8_t(255 - fpos);
    if (ipos + num < NUM_LEDS) {
        set_range(ipos + 1, num - 1, rgb);
    } else {
        set_range(ipos + 1, NUM_LEDS - ipos - 1, rgb);
        set_range(0, ipos + num - NUM_LEDS, rgb);
    }

    ipos += num;

    if (ipos >= NUM_LEDS) ipos -= NUM_LEDS;

    leds[ipos] = rgb % fpos;
}

/* ------------------------------------------------------------------------- *\
|  Fire
\* ------------------------------------------------------------------------- */

struct Fire {

    enum {NUM = NUM_LEDS};
    uint8_t heat[NUM];

    void	init() { clear(heat); }

    void	update() {
    uint8_t h0 = heat[0];
        for (int i = 1; i < NUM - 1; i++) {
            uint8_t	h1 = heat[i];
                h0 = (h0 + h1 + heat[i + 1]) / 3;
                heat[i] = h0 > 1 && random8() < 128 ? h0 - 1 : h0;
            h0 = h1;
        }    
        int y = random16(NUM);
        heat[y] = qadd8( heat[y], random8(160,255) );
    }

    void read(CRGB *leds, CHSV tint) {
        for (int i = 0; i < NUM; i++)
            leds[i] = Tint(HeatColor2(min(heat[i] * 2, 255)), tint);
    }
}; 

Fire fire;
void fire2012 () 
{
    if (mode_init)
		fire.init();
    fire.update();
    fire.read(leds, hsv);
}

/* ------------------------------------------------------------------------- *\
|  Sweeping Dot
\* ------------------------------------------------------------------------- */

uint16_t counter1;
uint16_t counter2;
void sweeping_dot ()
{
    if (mode_init) {
        // counter1 = 0;
        // counter2 = 0;
        lastpos  = 0;
    }

    fade(20);

    counter1 += m[0];
    counter2 += m[1];

    uint16_t beat = sin16(counter1) + 32768;

    int pos = scale16(beat, NUM_LEDS * 2);
    
    if (pos >= NUM_LEDS) pos = NUM_LEDS * 2 - 1 - pos;

    auto col = hsv;

    if (lastpos > pos) {
        add_range(pos, lastpos - pos, col);
    } else {
        add_range(lastpos, pos - lastpos, col);
    }

    lastpos = pos;
}

/* ------------------------------------------------------------------------- *\
|  Rainbow
\* ------------------------------------------------------------------------- */

uint8_t rainbow_pos = 0;
void rainbow() 
{
    rainbow_pos += m[0] / 100;
    // FastLED's built-in rainbow generator
    fill_rainbow(leds, NUM_LEDS, rainbow_pos, 7);
}

/* ------------------------------------------------------------------ *
 | Helper Functions
 * ------------------------------------------------------------------ */


void fade(uint8_t b) 
{ 
    fadeToBlackBy(leds, NUM_LEDS, b);
}

void add_range(int i, int n, CRGB rgb) 
{
    while (n--) leds[i++] += rgb;
}

void set_range(int i, int n, CRGB rgb) 
{
    while (n--) leds[i++] = rgb;
}

CHSV rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b)
{
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

CRGB hsv_to_rgb(uint8_t h, uint8_t s, uint8_t v) 
{
    uint8_t	c = (v * s) >> 8;
    uint8_t	m = v - c, x = v, y = v;
    uint16_t	i = h * 3;

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

CHSV to_hsv(const CRGB &x) 
{
    return rgb_to_hsv(x.r, x.g, x.b);
}

CRGB to_rgb(const CHSV &x) 
{
    return hsv_to_rgb(x.h, x.s, x.v);
}

CRGB Tint(CRGB c, CHSV tint) 
{
    CHSV	c2 = to_hsv(c);
    return hsv_to_rgb(c2.h + tint.h, scale8(c2.s, tint.s), scale8(c2.v, tint.v));
}

CRGB HeatColor2( uint8_t temperature) 
{
    uint16_t t = temperature * 3;
    return CRGB(
        min(t, 255u),
        t < 256 ? 0 : min(t - 256, 255u),
        t < 512 ? 0 : min(t - 512, 255u)
    );
}

