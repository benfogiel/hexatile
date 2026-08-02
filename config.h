#pragma once
#include <Arduino.h>

// ============================================================================
// Hardware configuration — VERIFY THESE AGAINST YOUR BOARD
// ============================================================================

// WS2812B data pin (Arduino pin 2 on megaTinyCore = PA6)
#define LED_PIN            PIN_PA6

// Number of LEDs per tile
#define NUM_LEDS           91

// Global brightness cap, 0-255. 64 = 25%. At 25% a full-white tile draws
// roughly 1.3 A — size this to what your magnetic connectors can carry.
#define BRIGHTNESS         20

// Centre-to-centre distance between two mated tiles (2 * hexagon apothem), in
// mm. Animations stay coherent across tile boundaries only if this matches
// reality, so measure it on real hardware. Odd values are fine.
//
// Keep it an integer: this feeds integer grid maths, and a decimal here
// promotes those expressions to double, dragging the AVR float library into
// the build for ~610 bytes of flash to compute two numbers that only change
// when a tile is moved.
#define TILE_PITCH_MM      85

// strip.show() bit-bangs WS2812 timing with interrupts off, so millis() stops
// for its duration: 24 bits per LED at 1.25 us each. The animation clock adds
// this back after every frame.
#define LED_SHOW_US        ((uint16_t)(NUM_LEDS * 30U))

// Side numbering. SIDE_PIN[s] is the pin for side s. Sides MUST be numbered
// going COUNTER-CLOCKWISE around the tile (viewed from the LED face), and
// side 0 must face the direction of LED theta = 0 in your layout table.
// If your physical numbering is clockwise, set SIDE_CCW to 0.
#define NUM_SIDES          6
#define SIDE_CCW           1

// ============================================================================
// Protocol tuning
// ============================================================================
#define COMMS_BAUD         9600UL
#define BEACON_MS          400      // per-side beacon interval
#define NEIGHBOR_TIMEOUT_MS 1500    // side considered vacated after this
#define AIR_DELAY_MS       13       // airtime of one 12-byte packet (for time sync)
#define ANIM_PERIOD_MS     20000U   // root cycles animation every 20 s (must be < 65535)
#define NUM_ANIMS          3

// Root-information aging — the spanning tree's loop breaker. Every beacon
// carries how long ago the root generated the information it is relaying.
// The root always says 0; everyone else adds the delay it took to reach them.
// In a cycle with no root, nothing resets the age, so it climbs past the limit,
// every tile refuses the stale root and the election re-runs from scratch.
// ROOT_MAX_AGE_MS caps the usable network diameter at roughly
// ROOT_MAX_AGE_MS / BEACON_MS hops.
#define ROOT_AGE_UNIT_MS   32       // resolution of the age byte (255 * 32 = 8.1 s)
#define ROOT_MAX_AGE_MS    5000U

// ============================================================================
// Shared types
// ============================================================================

// What every beacon carries: the sender's view of the network.
struct NodeInfo {
  uint8_t  rootID;   // smallest tile ID seen — the elected root
  int8_t   q, r;     // sender's axial hex-grid coordinate (root = 0,0)
  uint8_t  rot;      // sender's rotation, 0-5 (multiples of 60 deg CCW)
  uint8_t  side;     // which of the sender's sides this beacon left on
  uint8_t  hop;      // distance from root
  uint8_t  age;      // age of this root info, in ROOT_AGE_UNIT_MS units
  uint16_t t;        // sender's animation clock (ms, wraps)
  uint8_t  animID;   // current animation, chosen by root
};

struct Neighbor {
  NodeInfo info;
  uint16_t lastHeard;  // (uint16_t)millis() of last valid packet
  bool     present;
};
