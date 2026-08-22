#include "anim.h"
#include "topology.h"

#define RENDERS_FROM_COORDS (DEBUG_PATTERN == DBG_OFF      || \
                             DEBUG_PATTERN == DBG_GEOMETRY || \
                             DEBUG_PATTERN == DBG_RINGS)

#if DEBUG_PATTERN == DBG_OFF
// ---------------------------------------------------------------------------
// Quarter-wave parabola approximation of a sine (no table, ~40 B of flash).
// sin8(0)=128, sin8(64)=255, sin8(128)=128, sin8(192)=1.
static uint8_t sin8(uint8_t x) {
  uint8_t p = x & 0x7F;                 // half-wave phase
  if (p > 63) p = 127 - p;              // fold to quarter wave, 0..63
  uint16_t v = (uint16_t)p * (uint16_t)(190 - p);
  uint8_t half = (uint8_t)(v >> 6);     // 0..126
  return (x & 0x80) ? (uint8_t)(127 - half) : (uint8_t)(128 + half);
}
#endif

#if RENDERS_FROM_COORDS
static uint16_t isqrt32(uint32_t v) {
  uint32_t res = 0, bit = 1UL << 30;
  while (bit > v) bit >>= 2;
  while (bit) {
    if (v >= res + bit) { v -= res + bit; res = (res >> 1) + bit; }
    else res >>= 1;
    bit >>= 2;
  }
  return (uint16_t)res;
}

// s = 255, with the global brightness cap folded into v.
static void hsv2rgb(uint8_t h, uint8_t v, uint8_t* rgb) {
  v = (uint8_t)(((uint16_t)v * (BRIGHTNESS + 1)) >> 8);
  uint8_t region = h / 43;
  uint8_t rem = (uint8_t)((h - region * 43) * 6);
  uint8_t p = 0;
  uint8_t q = (uint8_t)(((uint16_t)v * (255 - rem)) >> 8);
  uint8_t t = (uint8_t)(((uint16_t)v * rem) >> 8);
  switch (region) {
    case 0:  rgb[0] = v; rgb[1] = t; rgb[2] = p; break;
    case 1:  rgb[0] = q; rgb[1] = v; rgb[2] = p; break;
    case 2:  rgb[0] = p; rgb[1] = v; rgb[2] = t; break;
    case 3:  rgb[0] = p; rgb[1] = q; rgb[2] = v; break;
    case 4:  rgb[0] = t; rgb[1] = p; rgb[2] = v; break;
    default: rgb[0] = v; rgb[1] = p; rgb[2] = q; break;
  }
}
#else
static void put_rgb(uint8_t* grbBuf, uint8_t i, uint8_t r, uint8_t g, uint8_t b) {
  uint16_t k = (uint16_t)i * 3;
  grbBuf[k + 0] = (uint8_t)(((uint16_t)g * (BRIGHTNESS + 1)) >> 8);   // WS2812B is GRB
  grbBuf[k + 1] = (uint8_t)(((uint16_t)r * (BRIGHTNESS + 1)) >> 8);
  grbBuf[k + 2] = (uint8_t)(((uint16_t)b * (BRIGHTNESS + 1)) >> 8);
}
#endif

#if DEBUG_PATTERN == DBG_TOPOLOGY
#include "comms.h"

// The layout is six concentric rings around a centre LED: ring k holds 6k LEDs
// (1 + 6 + 12 + 18 + 24 + 30 = 91), evenly spaced and running CLOCKWISE from
// local theta = 0.
static uint8_t ring_first(uint8_t ring) { return (uint8_t)(1 + 3 * ring * (ring - 1)); }

// Paint the LEDs of one ring that face a given side. Side s sits at local angle
// 60s - 30*LED_THETA0_HALF_STEPS, which on a corner-aligned table falls exactly
// BETWEEN two LEDs, so positions are counted in half-LED steps: the arc is the
// k LEDs straddling the normal, or the k-1 between two shared corner LEDs.
static void paint_side(uint8_t* grbBuf, uint8_t ring, uint8_t side,
                       uint8_t r, uint8_t g, uint8_t b) {
  int16_t count = 6 * (int16_t)ring;
  int16_t turn  = 2 * count;                  // half-steps in a full turn
#if SIDE_CCW
  int16_t normal = (int16_t)LED_THETA0_HALF_STEPS * ring - 2 * (int16_t)side * ring;
#else
  int16_t normal = (int16_t)LED_THETA0_HALF_STEPS * ring + 2 * (int16_t)side * ring;
#endif
  for (int16_t j = 0; j < count; j++) {
    int16_t d = ((2 * j - normal) % turn + turn) % turn;
    if (d > count) d = turn - d;              // circular distance, half-steps
    if (d <= (int16_t)ring - 1) put_rgb(grbBuf, (uint8_t)(ring_first(ring) + j), r, g, b);
  }
}

// No animation at all — each tile just states its own beliefs, so a wrong
// picture can be traced to the tile that holds the wrong belief. See the
// bring-up table in the README for how to read the result.
static void render_topology(uint8_t* grbBuf) {
  for (uint16_t k = 0; k < (uint16_t)NUM_LEDS * 3; k++) grbBuf[k] = 0;

  bool root = topo_is_root();
  put_rgb(grbBuf, 0, root ? 255 : 60, root ? 255 : 0, root ? 255 : 0);

  uint8_t hop = topo_hop();                       // ring 1: one blue LED per hop
  for (uint8_t h = 0; h < hop && h < 6; h++) put_rgb(grbBuf, (uint8_t)(1 + h), 0, 0, 255);

  // Ring 2: a spinner stepping once per beacon folded into the clock, so the
  // sync loop's input rate is visible.
  put_rgb(grbBuf, (uint8_t)(ring_first(2) + topo_sync_count() % 12), 0, 200, 200);

  // Ring 3: where this tile believes global +x lies.
  paint_side(grbBuf, 3, (uint8_t)((6 - topo_rot()) % 6), 255, 140, 0);

  // Ring 4: clock error against the parent, one LED per 8 ms, growing clockwise
  // when behind and anticlockwise when ahead, red past 16 ms.
  if (!root) {
    int16_t err = topo_clock_err();
    uint16_t mag = (uint16_t)(err < 0 ? -err : err);   // in clock ticks
    uint8_t arm = (uint8_t)(mag / (8 / CLOCK_TICK_MS) + 1);
    if (arm > 24) arm = 24;
    for (uint8_t k = 0; k < arm; k++) {
      int16_t j = (err >= 0) ? (int16_t)k : -(int16_t)k;
      put_rgb(grbBuf, (uint8_t)(ring_first(4) + ((j % 24) + 24) % 24),
              mag <= 16 / CLOCK_TICK_MS ? 0 : 255,
              mag <= 16 / CLOCK_TICK_MS ? 255 : 0, 0);
    }
  }

  // Outer ring: green arc per side with a live neighbor, white on the side this
  // tile took its coordinate from.
  int8_t parent = topo_parent_side();
  for (uint8_t s = 0; s < NUM_SIDES; s++) {
    Neighbor n;
    comms_get_neighbor(s, &n);
    if (!n.present) continue;
    uint8_t rb = ((int8_t)s == parent) ? 255 : 0;
    paint_side(grbBuf, 5, s, rb, 255, rb);
  }
}
#endif

// ---------------------------------------------------------------------------
// Every effect is color = f(global x, global y, t). Identical math runs on
// every tile; only (x, y) differs, so the picture is seamless across tiles.
void anim_render(uint8_t* grbBuf) {
#if DEBUG_PATTERN == DBG_SYNC
  // Position is never consulted, so anything on screen is the clock alone: the
  // whole tile ramps over 1.024 s and drops. That period divides both the cycle
  // (92160 = 90 * 1024) and its excess over the 16-bit millisecond wrap, so
  // neither wrap shows.
  uint8_t v = (uint8_t)((topo_anim_time() & 1023U) >> 2);
  for (uint8_t i = 0; i < NUM_LEDS; i++) put_rgb(grbBuf, i, v, v, v);

#elif DEBUG_PATTERN == DBG_TOPOLOGY
  render_topology(grbBuf);

#else
#if DEBUG_PATTERN != DBG_GEOMETRY
  uint16_t t = topo_anim_time();
#endif
#if DEBUG_PATTERN == DBG_OFF
  uint8_t anim = topo_anim_id();
#endif
  int16_t  tx = topo_tile_x(), ty = topo_tile_y();
  const int8_t* lx = topo_led_gx();
  const int8_t* ly = topo_led_gy();

  uint8_t rgb[3];
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    int16_t x = tx + lx[i];
    int16_t y = ty + ly[i];
    uint8_t hue, val = 255;

#if DEBUG_PATTERN == DBG_OFF
    switch (anim) {
      case 0: {   // expanding rainbow rings centred on the root tile
        uint16_t d = isqrt32((int32_t)x * x + (int32_t)y * y);
        hue = (uint8_t)((d << 1) - (t >> 4));
        break;
      }
      case 1: {   // diagonal rainbow sweep across the whole assembly
        hue = (uint8_t)(((x + y) >> 1) + (t >> 5));
        break;
      }
      default: {  // plasma
        uint8_t a = sin8((uint8_t)((x >> 1) + (t >> 5)));
        uint8_t b = sin8((uint8_t)((y >> 1) - (t >> 6)));
        uint8_t c = sin8((uint8_t)(((x - y) >> 2) + (t >> 4)));
        hue = (uint8_t)((a + b + c) / 3);
        val = (uint8_t)(128 + (sin8((uint8_t)(a + (t >> 6))) >> 1));
        break;
      }
    }
#elif DEBUG_PATTERN == DBG_GEOMETRY
    // Time is frozen, so the picture is pure map: a still rainbow centred on
    // the root with a black contour ring every 128 mm. A tile in the wrong cell
    // or at the wrong rotation breaks the contour at its own edge.
    uint16_t d = isqrt32((int32_t)x * x + (int32_t)y * y);
    hue = (uint8_t)(d << 1);
    if ((d & 127) < 3) val = 0;
#else   // DBG_RINGS
    // 32 mm bands travelling outward at 125 mm/s. Hard edges rather than a
    // gradient: a rainbow can look continuous across a bad seam, a band cannot.
    uint16_t d = isqrt32((int32_t)x * x + (int32_t)y * y);
    hue = 128;
    val = ((uint8_t)(d - (t >> 3)) & 63) < 32 ? 255 : 0;
#endif

    hsv2rgb(hue, val, rgb);
    grbBuf[(uint16_t)i * 3 + 0] = rgb[1];   // WS2812B is GRB
    grbBuf[(uint16_t)i * 3 + 1] = rgb[0];
    grbBuf[(uint16_t)i * 3 + 2] = rgb[2];
  }
#endif
}
