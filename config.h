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
// going COUNTER-CLOCKWISE around the tile (viewed from the LED face).
// If your physical numbering is clockwise, set SIDE_CCW to 0.
#define NUM_SIDES          6
#define SIDE_CCW           1

// Where LED theta = 0 sits relative to side 0's outward normal, counted in
// 30 deg CCW steps viewed from the LED face. Every ring in led_layout.h starts
// at theta = 0, so this is asking one question about the board: does that spoke
// run through the MIDDLE of a side (0), or does it point at a CORNER (+1 or -1,
// i.e. half a side away)?
//
// Only 60 deg of this is free — a whole-side error just rotates the finished
// picture uniformly, which is harmless. The half-side part is not: the rotation
// cache can only turn LEDs in 60 deg steps, so a 30 deg error survives to the
// screen as every tile's picture twisted about its own centre while the tile
// centres stay right. That reads as a seam no pitch tuning can close.
//
// Check it with DBG_TOPOLOGY: the side arcs must sit centred on the physical
// sides. Straddling a corner means this wants +/-1; centred on the NEXT side
// over means the sign is backwards.
#define LED_THETA0_HALF_STEPS  1

// ============================================================================
// Protocol tuning
// ============================================================================
#define COMMS_BAUD         9600UL
#define BEACON_MS          400      // beacon interval on a side that has a neighbor

// Beacon interval on a side that has none. Transmitting masks every side's pin
// interrupt for the whole 13 ms packet, so a beacon into an open edge is 13 ms
// of deafness on the edges that are actually mated — and on a small assembly
// most edges are open, which makes that the single largest source of loss on
// the links that matter. Open sides only have to beacon often enough to notice
// a tile being attached, so they back off to discovery duty.
#define IDLE_BEACON_MS     1600

// Side considered vacated after this. It has to cover a burst of loss, not just
// one packet: at BEACON_MS this is the number of consecutive misses that will
// drop a live neighbor and send the tile back to believing it is a root.
#define NEIGHBOR_TIMEOUT_MS 3000
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
// Debug patterns
// ============================================================================
// Set DEBUG_PATTERN to one of these and every tile renders it instead of the
// normal animation cycle (animID is ignored, and the unused effects compile
// out). They exist as separate modes because a single pattern cannot tell you
// whether a fault is in the clock or in the map — each of the first two holds
// one of those constant so the other is the only thing you are looking at.
#define DBG_OFF        0
#define DBG_SYNC       1   // clock only — no position input at all
#define DBG_GEOMETRY   2   // map only — frozen in time
#define DBG_TOPOLOGY   3   // direct readout of what each tile believes
#define DBG_RINGS      4   // end-to-end: rings expanding from the root tile

#define DEBUG_PATTERN  DBG_TOPOLOGY

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
