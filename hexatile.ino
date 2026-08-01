#include <FastLED.h>

#define NUM_LEDS 91
#define LED_PIN 2

CRGB leds[NUM_LEDS];

struct Pos { float r; uint8_t theta; };
const PROGMEM struct Pos led_pos_progmem[91] = {
  {0, 0},
  {8.66, 0},
  {8.66, 300},
  {8.66, 240},
  {8.66, 180},
  {8.66, 120},
  {8.66, 60},
  {17.32, 0},
  {17.32, 330},
  {17.32, 300},
  {17.32,270},
  {17.32,240},
  {17.32,210},
  {17.32,180},
  {17.32,150},
  {17.32,120},
  {17.32,90},
  {17.32,60},
  {17.32,30},
  {25.98,0},
  {25.98,340},
  {25.98,320},
  {25.98,300},
  {25.98,280},
  {25.98,260},
  {25.98,240},
  {25.98,220},
  {25.98,200},
  {25.98,180},
  {25.98,160},
  {25.98,140},
  {25.98,120},
  {25.98,100},
  {25.98,80},
  {25.98,60},
  {25.98,40},
  {25.98,20},
  {34.64,0},
  {34.64,345},
  {34.64,330},
  {34.64,315},
  {34.64,300},
  {34.64,285},
  {34.64,270},
  {34.64,255},
  {34.64,240},
  {34.64,225},
  {34.64,210},
  {34.64,195},
  {34.64,180},
  {34.64,165},
  {34.64,150},
  {34.64,135},
  {34.64,120},
  {34.64,105},
  {34.64,90},
  {34.64,75},
  {34.64,60},
  {34.64,45},
  {34.64,30},
  {34.64,15},
  {43.3,0},
  {43.3,348},
  {43.3,336},
  {43.3,324},
  {43.3,312},
  {43.3,300},
  {43.3,288},
  {43.3,276},
  {43.3,264},
  {43.3,252},
  {43.3,240},
  {43.3,228},
  {43.3,216},
  {43.3,204},
  {43.3,192},
  {43.3,180},
  {43.3,168},
  {43.3,156},
  {43.3,144},
  {43.3,132},
  {43.3,120},
  {43.3,108},
  {43.3,96},
  {43.3,84},
  {43.3,72},
  {43.3,60},
  {43.3,48},
  {43.3,36},
  {43.3,24},
  {43.3,12}
};

const float max_r = 43.3;

void setup() {
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
}

void loop() {
  uint32_t t = millis() / 20;
  uint8_t brightness = 50;
  for (int i = 0; i < NUM_LEDS; i++) {
    float r = pgm_read_float_near(&led_pos_progmem[i].r) / max_r;
    float hue = uint32_t(t + r * 255) % 255;  // wave radiates outward
    leds[i] = CHSV(hue, 255, brightness);
  }
  FastLED.show();
}

// start at a node and mark it as (0, 0)
// then traverse to to a neighboring node and use the angle between nodes and diameter of the hexagon to calculate the position of the new node
// repeat until all nodes are visited
// then find the center of all the nodes and translate all node coordinates to be relative to the center
