#include <esp_gdbstub.h>
#include <WiFi.h>

//#define FASTLED_RMT_BUILTIN_DRIVER 1
#include <FastLED.h>

FASTLED_USING_NAMESPACE

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

typedef int8_t     	int8;
typedef uint8_t    	uint8;
typedef uint16_t   	uint16;
typedef int16_t		int16;
typedef int32_t 	int32;
typedef uint32_t 	uint32;

struct PacketSpec { uint8	dir, w, h; };
struct PacketRect { uint8 	x, y, w, h; };

struct Packet {
	enum {MAGIC = 0xa5};
	enum COMMAND : uint8 {
		COM_RESET,
		COM_GET_SPEC,
		COM_SET_SPEC,
		COM_CLEAR,
		COM_LOAD_RGB,
		COM_FILL_RGB,
		COM_END = 0xff,
	};
	uint8	magic;
	COMMAND	command;
//	Packet(COMMAND command) : magic(MAGIC), command(command) {}
};
/*
template<Packet::COMMAND C> struct PacketT : Packet {
	PacketT() : Packet(C) {}
};

template<> struct PacketT<Packet::COM_SET_SPEC> : Packet, PacketSpec {
	PacketT() : Packet(COM_SET_SPEC) {};
};
template<> struct PacketT<Packet::COM_BOX_RGB> : Packet {
	uint8	x, y, w, h;
	PacketT() : Packet(COM_BOX_RGB) {};
};
 */
#define DATA_PIN    19
//#define CLK_PIN   4
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define UDP_PORT	2390

#define FRAMES_PER_SECOND  120

#undef min
#undef max
#undef abs

template<typename R, typename T> bool read(R &r, T &t) {
	return r.read((char*)&t, sizeof(T)) == sizeof(T);
}
template<typename W, typename T> bool write(W &w, const T &t) {
	return w.write((const uint8*)&t, sizeof(T)) == sizeof(T);
}

template<typename T> struct wrapped {
	T 	t;
	wrapped() {}
	wrapped(const T &t) : t(t) {}
	operator T&() 				{ return t; }
	operator const T&() const 	{ return t; }
	T&	operator=(const T &u)	{ return t = u; }

};

template<typename T, int B> struct baseint : wrapped<T> {
	baseint() {}
	baseint(const T &t) : wrapped<T>(t) {}
};

template<typename T> baseint<T, 16> hex(const T &t) { return t; } 

template<typename T> Print &operator<<(Print &serial, const T &t) {
	serial.print(t);
	return serial;
}
template<typename T, int B> Print &operator<<(Print &serial, const baseint<T,B> &t) {
	serial.print(t, B);
	return serial;
}

template<typename T> void clear(T &t) 		{ memset(&t, 0, sizeof(T)); }
template<typename A, typename B> A min(A a, B b) 		{ return a < b ? a : b; }
template<typename A, typename B> A max(A a, B b) 		{ return a < b ? b : a; }
template<typename T> T clamp(T t, T a, T b) { return min(max(t, a), b); }
template<typename T> T abs(T a) 			{ return a < 0 ? -a : a; }
template<typename T> T square(T a) 			{ return a * a; }

int16	mul16(int16 a, int16 b) { return (int32(a) * b) >> 15; }
int16	div16(int16 a, int16 b) { return (int32(a) << 15) / b; }
int16 	square16(int16 a) 		{ return mul16(a, a); }

int16	cheap_curve(int16 t)	{ return 32767 - square16(32767 - square16(t)); }
int16	hermite16(int16 t)		{ return (int32(t) * ((int32(t) * (int16(0x5fff) - t / 2)) >> 13)) >> 15; }
int		hermite(int t, int r)	{ return t * t * (3 * r - 2 * t); }

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

int in_heart(int x, int y, int r) {
	if (y > r / 2)
		return square(abs(x * 2) - r) + square(y * 2 - r) - square(r);

	if (y < -r)
		return 0;

	return abs(x) * square(r) - hermite(r + y * 2 / 3, r);
}

struct Screen  {
	static const int DEF_DIR = 0, DEF_WIDTH = 50, DEF_HEIGHT = 12, NUM_LEDS = 600;
#ifdef UDP_PORT
	WiFiUDP		udp;
#else
	WiFiServer 	server;
	WiFiClient 	client;
#endif
	int			state, dir, width, height;
	unsigned int	frame = 0;

	CRGB 		leds[NUM_LEDS];

	int pixel_to_index(int x, int y) {
		if (dir & 2)
			y = height - 1 - y;
		return y * width + ((y ^ dir) & 1 ? width - 1 - x : x);
	}
	int index_to_pixel(int i, int &y) {
		y = i / width;
		int	x = i - y * width;
		return y & 1 ? width - 1 - x : x;
	}

	CRGB& pixel(int x, int y) {
		return leds[pixel_to_index(x, y)];
	}

	void shape(int sx, int sy, int cx, int cy, int r) {
		for (int y = 0; y < height; y++) {
			for (int x = 0; x < width; x++) {
				int z = in_heart(x * sx - cx, y * sy - cy, r);
				pixel(x,y) = z < 0
					//square(x - 25) + square(y * 5 - 25) < square(r)
					? /*CRGB(64,64,64) */CRGB(z / 4096, (z / 64) % 64, z %64)
					: CRGB(0,0,0);
			}
		}
	}


	Screen() : state(0), dir(DEF_DIR), width(DEF_WIDTH), height(DEF_HEIGHT)
	#ifndef UDP_PORT
	, server(TCP_PORT)
	#endif
	{}

    void setup() {
		FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
		clear(leds);
	}

    void loop() {
		switch (state) {
			case 0:
				if (WiFi.status() == WL_CONNECTED) {
				#ifdef UDP_PORT
					udp.begin(UDP_PORT);
				#else
					server.begin();
				#endif
					Serial << "Server started at IP:" << WiFi.localIP() << '\n';
					state = 1;
				}
				break;
			case 1:
			case 2: {
#ifdef UDP_PORT
	#if 1
				bool	end = false;
				while (int size = udp.parsePacket()) {
					Serial << "UDP packet size: " << size << '\n';
					if (udp.read() == Packet::MAGIC) {
						switch (udp.read()) {
							case Packet::COM_RESET: {
								Serial << "RESET:\n";
								dir		= DEF_DIR;
								width 	= DEF_WIDTH;
								height 	= DEF_HEIGHT;
								state 	= 1;
								break;
							}
							case Packet::COM_GET_SPEC: {
								Serial << "GET_SPEC:\n";
    							udp.beginPacket(udp.remoteIP(), udp.remotePort());
    							write(udp, PacketSpec{dir, width, height});
    							udp.endPacket();
								break;
							}
							case Packet::COM_SET_SPEC: {
								Serial << "SET_SPEC:\n";
								PacketSpec	spec;
								if (read(udp, spec)) {
									dir 	= spec.dir;
									width	= spec.w;
									height	= spec.h;
								}
								break;
							}
							case Packet::COM_CLEAR: {
								Serial << "CLEAR:\n";
								clear(leds);
								state = 2;
								break;
							}
							case Packet::COM_LOAD_RGB: {
								Serial << "LOAD_RGB:\n";
								PacketRect	rect;
								if (read(udp, rect)) {
									for (int y = 0; y < rect.h; ++y)
										for (int x = 0; x < rect.w; ++x)
											read(udp, pixel(rect.x + x, rect.y + y));
									state = 2;
								}
								break;
							}
							case Packet::COM_FILL_RGB: {
								Serial << "FILL_RGB:\n";
								PacketRect	rect;
								CRGB		rgb;
								if (read(udp, rect) && read(udp, rgb)) {
									for (int y = 0; y < rect.h; ++y)
										for (int x = 0; x < rect.w; ++x)
											pixel(rect.x + x, rect.y + y) = rgb;

									state = 2;
								}
								break;
							}
							case Packet::COM_END:
								end = true;
								break;

						}
					}
					udp.flush();
					if (end)
						break;
				}
	#endif
#else
				if (!client.connected()) {
			  		if (client = server.available())
	    				Serial << "new client at " << client.localIP() << '\n';
				}
				if (client) {
					#if 0
					for (int i = 0; i < NUM_LEDS; i++) {
						client.readBytes((byte*)&pixel(i), 3;
					#else
					client.readBytes((byte*)leds, sizeof(leds));
					#endif
					//client.stop();
					state = 2;
			  	}
#endif
			  	break;
			}
		}

		if (state < 2) {
			int	i = short(frame * 1024);
			int	r = hermite16(i < 0 ? ~i : i) / 1024 * 3 / 4;
			shape(1, 5, width / 2, 28, r);
		}
        FastLED.show();  
        FastLED.delay(1000/FRAMES_PER_SECOND);
		++frame;
    }
} screen;

struct MAC {
	baseint<byte,16>	mac[6];
	operator byte*() { return (byte*)mac; }
	friend Print &operator<<(Print &serial, const MAC &t) {
		return serial << t.mac[0] << ':' << t.mac[1] << ':' << t.mac[2] << ':' << t.mac[3] << ':' << t.mac[4] << ':' << t.mac[5];
	}
};

Print &operator<<(Print &serial, wifi_auth_mode_t t) {
	switch (t) {
		default:
		case WIFI_AUTH_OPEN:	return serial << "None";
		case WIFI_AUTH_WEP:		return serial << "WEP";
		case WIFI_AUTH_WPA_PSK:	return serial << "WPA";
		case WIFI_AUTH_WPA2_PSK:return serial << "WPA2";
	}
}
void setup() {
	Serial.begin(115200);
	gdbstub_init();
	while (!Serial);
	Serial.println("-------START-------");

  	MAC	mac;
  	WiFi.macAddress(mac);
  	Serial << "MAC: " << mac << '\n';
  	Serial << "Scanning available networks...";

	int num_ssid = WiFi.scanNetworks();
	Serial << num_ssid << '\n';
	for (int i = 0; i < num_ssid; i++)
		Serial << i << ") " << WiFi.SSID(i) << '\t' << WiFi.RSSI(i) << " dBm" << '\t' << WiFi.encryptionType(i) << '\n';

	WiFi.begin("stephens", "c0406a0222");
    screen.setup();
}
void loop() {
    screen.loop();
}
