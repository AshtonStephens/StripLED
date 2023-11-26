/// @file    DemoReel100.ino
/// @brief   FastLED "100 lines of code" demo reel, showing off some effects
/// @example DemoReel100.ino

#include <FastLED.h>
#include <WiFi.h>
#include <WiFiUdp.h>

FASTLED_USING_NAMESPACE

// FastLED "100-lines-of-code" demo reel, showing just a few
// of the kinds of animation patterns you can quickly and easily
// compose using FastLED.
//
// This example also shows one easy way to define multiple
// animations patterns and have them automatically rotate.
//
// -Mark Kriegsman, December 2014

#define DATA_PIN    3
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    300
CRGB leds[NUM_LEDS];

#define BRIGHTNESS         200
#define FRAMES_PER_SECOND  120

#define NUM_SPARKLES 60
#define MAX_INT_16 65536
#define EZ(x) Serial.print(#x); Serial.print(": "); Serial.println(x)
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
#define PACKET_BUFFER_LEN 256
#define CHSV_MOD CHSV(mod[MOD_HUE], mod[MOD_SAT], mod[MOD_BRI])

#define MOD_PAT 0
#define MOD_FPS 1
#define MOD_BRI 2
#define MOD_HUE 8
#define MOD_SAT 9

#define NUM_MOD 32

const CHSV gDefaultChsv = CHSV(0, 0, 0);

// Structure that represents a sparkle on the LED chain.
struct Sparkle {
  // The index of the led being lit up.
  uint32_t led_index;

  // The brightness x value on the brightness function.
  int16_t brightness_x;

  // The hue of the sparkle
  uint8_t hue;

  // The speed modifier of the brighness
  uint8_t speed_mod;
};
Sparkle sparkles[NUM_SPARKLES];

const char* ssid = "The LAN Before Time 2.4G";
const char* password = "littlefootbigdreams";
const uint serverPort = 123;

// Setup server.
WiFiServer Server(serverPort);

// Setup Udp.
WiFiUDP Udp;

int16_t mod[NUM_MOD];
uint16_t packetBuffer[PACKET_BUFFER_LEN/2]; //buffer to hold incoming packet

// Connect to wifi
void setup_wifi(
  const char* ssid,
  const char* password
) {
  // Setup Wifi as Station
  WiFi.mode(WIFI_STA); // Optional.
  WiFi.begin(ssid, password);

  Serial.println("\nConnecting");
  while(WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100); // Wait 100 ms.
  }

  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void setup_udp_port(const uint port) {
  Udp.begin(port);
}

void read_udp_packet() {
  int packetSize = Udp.parsePacket();

  // Only read the data if the packet is small enough to be read into the buffer.
  if (packetSize && packetSize < PACKET_BUFFER_LEN << 1) {

    // read the packet into packetBuffer.
    int len = Udp.read((uint8_t*)packetBuffer, PACKET_BUFFER_LEN) >> 1;

    // For each pair of numbers put the first number into
    // the array, and then the second as its value.
    const uint16_t num_pairs = len & (~1); // Fix length to have even number.
    for (int i = 0; i < num_pairs; i += 2) {
      const uint16_t modifier_index = packetBuffer[i];
      const uint16_t modifier_number = packetBuffer[i + 1];
      if (modifier_index < NUM_MOD) {
        mod[modifier_index] = modifier_number;
      }
    }
  }
}

void setup_leds() {
  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  // FastLED.addLeds<LED_TYPE,DATA_PIN,CLK_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  FastLED.setBrightness(BRIGHTNESS);
  for (uint i = 0; i < NUM_LEDS; ++i) {
    leds[i] = gDefaultChsv;
  }
  FastLED.show();
}

uint8_t brightness(
  int16_t brightness_x
) {
  return abs(brightness_x) >> 8;
}

int16_t update_brightness_x(
  int16_t brightness_x,
  uint8_t speed_mod
) {
  int16_t new_brightness_x = brightness_x + speed_mod * ((MAX_INT_16/FRAMES_PER_SECOND)/10);

  if (brightness_x < 0 && new_brightness_x > 0) {
    return 0;
  } else {
    return new_brightness_x;
  }
}

void sparkletime()
{

  // Update every sparkle or reset the sparkles.
  for (uint32_t i = 0; i < NUM_SPARKLES; ++i) {

    const Sparkle sparkle = sparkles[i];
    int16_t brightness_x = update_brightness_x(sparkle.brightness_x, sparkle.speed_mod);

    if (brightness_x != 0) {

      // Update sparkle brightness.
      sparkles[i].brightness_x = brightness_x;
      leds[sparkles[i].led_index] = CHSV(sparkle.hue, 200, brightness(brightness_x));

    } else {
      // Turn off the sparkling LED.
      leds[sparkles[i].led_index] = CHSV(0, 0, 0);

      // Create the default sparkle.
      sparkles[i].led_index=random16(NUM_LEDS);
      sparkles[i].brightness_x=0;
      sparkles[i].hue=mod[MOD_HUE];

      // Speed mod is how many times faster than 0.1 Hz the lights will be.
      sparkles[i].speed_mod=5 + random16(10);
    }
  }
}

int sinelon_prev = 0;
void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16( 13, 0, NUM_LEDS-1 );
  if (pos < sinelon_prev) {
    for (int i = pos; i < sinelon_prev; ++i) {
      leds[i] += CHSV_MOD;
    }
  } else {
    for (int i = sinelon_prev; i <= pos; ++i) {
      leds[i] += CHSV_MOD;
    }
  }
  sinelon_prev = pos;
}

void rainbow()
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, 0, 7);
}

void solid()
{
  for (int i = 0 ; i < NUM_LEDS; ++i) {
    leds[i] = CHSV_MOD;
  }
}

typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = {solid, rainbow, sparkletime, sinelon};

void setup_mods()
{
  // Set default mod
  for (int i = 0; i < NUM_MOD; ++i) {
    mod[i] = 0;
  }
  mod[MOD_PAT] = 0;   // Pattern.
  mod[MOD_FPS] = 120; // Frames per second.
  mod[MOD_BRI] = 255; // Brightness.
}
// * ------------------------------------------------------- *
// | Main
// * ------------------------------------------------------- *

void setup() {

  delay(1000); // 1 second delay for recovery
  Serial.begin(115200);

  // Connect to WiFi
  setup_wifi(ssid, password);
  setup_udp_port(serverPort);
  setup_mods();
  setup_leds();

  // // set master brightness control
  // FastLED.setBrightness(BRIGHTNESS);
}

void loop()
{
  while(true) {
    // delay(100); // Wait 100 milliseconds.
    read_udp_packet();
    gPatterns[mod[MOD_PAT]]();
    FastLED.setBrightness((uint8_t) mod[MOD_BRI]);

    FastLED.show();
    FastLED.delay(1000/mod[MOD_FPS]);
  }
}

// * ------------------------------------------------------- *
// | Helpers
// * ------------------------------------------------------- *




