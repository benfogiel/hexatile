#pragma once
#include <Arduino.h>

// ============================================================================
// Hardware — VERIFY THESE AGAINST YOUR BOARD
// ============================================================================

#define LED_PIN            PIN_PA6      // megaTinyCore Arduino pin 2
#define NUM_LEDS           91

// 0-255. At 64 (25%) a full-white tile draws ~1.3 A — size this to what the
// magnetic connectors can carry.
#define BRIGHTNESS         20

// Centre-to-centre distance between two mated tiles (2 * apothem), in mm.
// Measure it on real hardware: animations stay coherent across tile boundaries
// only if this matches reality. Keep it an integer — a decimal here promotes
// the grid maths to double and drags in the AVR float library (~610 B of flash)
// to compute two numbers that only change when a tile is moved.
#define TILE_PITCH_MM      85

// strip.show() bit-bangs WS2812 timing with interrupts off, so millis() stops
// for its duration: 24 bits per LED at 1.25 us each.
#define LED_SHOW_US        ((uint16_t)(NUM_LEDS * 30U))

// Sides MUST be numbered counter-clockwise around the tile, viewed from the LED
// face. Set SIDE_CCW to 0 if the physical numbering runs clockwise.
#define NUM_SIDES          6
#define SIDE_CCW           1

// Where LED theta = 0 sits relative to side 0's outward normal, in 30 deg CCW
// steps: does that spoke run through the MIDDLE of a side (0) or point at a
// CORNER (+1 or -1)?
//
// Only the half-side part matters. A whole-side error just rotates the finished
// picture uniformly; a 30 deg error cannot be absorbed by the rotation cache
// (which turns LEDs in 60 deg steps) and survives to the screen as every tile's
// picture twisted about its own centre while the tile centres stay right.
//
// Check with DBG_TOPOLOGY: side arcs must sit centred on the physical sides.
// Straddling a corner means this wants +/-1; centred on the NEXT side over means
// the sign is backwards.
#define LED_THETA0_HALF_STEPS  1

// ============================================================================
// Protocol tuning
// ============================================================================
#define COMMS_BAUD         9600UL
#define BEACON_MS          400

// Transmitting masks every side's pin interrupt for the whole 12 ms packet, so
// a beacon into an open edge is 12 ms of deafness on the edges that are mated —
// the largest source of loss on a small assembly, where most edges are open.
// Open sides only need to beacon often enough to notice a tile being attached.
#define IDLE_BEACON_MS     1600

// Fast beacons a side keeps sending after its neighbor disappears. Backing off
// immediately makes a dropped link self-sustaining: the side goes quiet exactly
// when it needs to re-acquire, so the outage lasts seconds rather than one
// beacon.
#define LINK_GRACE_BEACONS 25

// Has to cover a burst of loss, not just one packet: at BEACON_MS this is the
// number of consecutive misses that drops a live neighbor and sends the tile
// back to believing it is a root.
#define NEIGHBOR_TIMEOUT_MS 3000

#define AIR_DELAY_MS       12       // airtime of one 11-byte packet
#define NUM_ANIMS          3

// The shared clock counts in CLOCK_TICK_MS units, not milliseconds. What has to
// fit its 16 bits is ANIM_CYCLE_MS — one full pass through the animation list —
// because which animation is showing is not a piece of state that propagates,
// it is that clock divided by the dwell. A changeover is therefore exactly as
// simultaneous as the clock, rather than one beacon later per hop from the root.
// At 1 ms a count the cycle would cap out at 65 s, which three 30 s animations
// do not fit; 4 ms is a third of the ~13 ms of drift between beacons, so
// quantising to it is not what limits how closely the tiles agree.
#define CLOCK_TICK_MS      4
#define ANIM_PERIOD_MS     30720U   // dwell per animation
#define ANIM_CYCLE_MS      ((uint32_t)NUM_ANIMS * ANIM_PERIOD_MS)
#define ANIM_PERIOD_TICKS  ((uint16_t)(ANIM_PERIOD_MS / CLOCK_TICK_MS))
#define ANIM_CYCLE_TICKS   ((uint16_t)(ANIM_CYCLE_MS / CLOCK_TICK_MS))

// 92160 ms = 90 * 1024, and the cycle's excess over the 16-bit millisecond wrap
// (92160 - 65536 = 26624) is 26 * 1024 as well, so DBG_SYNC's 1.024 s ramp
// divides both the clock and the discontinuity at its wrap, and stays seamless.

// Root-information aging — the spanning tree's loop breaker. Every beacon
// carries how long ago the root generated the information it relays: the root
// says 0, everyone else adds the delay it took to reach them. In a cycle with no
// root nothing resets the age, so it climbs past the limit, every tile refuses
// the stale root and the election re-runs. This caps the usable network diameter
// at roughly ROOT_MAX_AGE_MS / BEACON_MS hops.
#define ROOT_AGE_UNIT_MS   32       // resolution of the age byte (255 * 32 = 8.1 s)
#define ROOT_MAX_AGE_MS    5000U

// ============================================================================
// Debug patterns
// ============================================================================
// Set DEBUG_PATTERN and every tile renders that instead of the animation cycle
// (animID is ignored, unused effects compile out). A single pattern cannot tell
// you whether a fault is in the clock or in the map, so the first two each hold
// one of those constant.
#define DBG_OFF        0
#define DBG_SYNC       1   // clock only — no position input at all
#define DBG_GEOMETRY   2   // map only — frozen in time
#define DBG_TOPOLOGY   3   // direct readout of what each tile believes
#define DBG_RINGS      4   // end-to-end: rings expanding from the root tile

#define DEBUG_PATTERN  DBG_OFF

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
  uint16_t t;        // sender's animation clock, in CLOCK_TICK_MS units
};

struct Neighbor {
  NodeInfo info;
  uint16_t lastHeard;  // (uint16_t)millis() of last valid packet
  bool     present;
};
