#include "anim.h"
#include "topology.h"

// ---------------------------------------------------------------------------
// 8-bit sine, quarter-wave parabola approximation (no table, ~40 B of flash).
// sin8(0)=128, sin8(64)=255, sin8(128)=128, sin8(192)=1.
static uint8_t sin8(uint8_t x) {
  uint8_t p = x & 0x7F;                 // half-wave phase
  if (p > 63) p = 127 - p;              // fold to quarter wave, 0..63
  uint16_t v = (uint16_t)p * (uint16_t)(190 - p);  // peaks near 64*126=8064
  uint8_t half = (uint8_t)(v >> 6);     // 0..126
  return (x & 0x80) ? (uint8_t)(127 - half) : (uint8_t)(128 + half);
}

// Integer sqrt of a 32-bit value (used for radial distance).
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

// HSV -> RGB, s=255, with the global brightness cap folded into v.
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

// ---------------------------------------------------------------------------
// Every effect is color = f(global x, global y, t). Identical math runs on
// every tile; only (x, y) differs, so the picture is seamless across tiles.
void anim_render(uint8_t* grbBuf) {
  uint16_t t = topo_anim_time();
  uint8_t  anim = topo_anim_id();
  int16_t  tx = topo_tile_x(), ty = topo_tile_y();
  const int8_t* lx = topo_led_gx();
  const int8_t* ly = topo_led_gy();

  uint8_t rgb[3];
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    int16_t x = tx + lx[i];
    int16_t y = ty + ly[i];
    uint8_t hue, val = 255;

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

    hsv2rgb(hue, val, rgb);
    grbBuf[(uint16_t)i * 3 + 0] = rgb[1];   // WS2812B is GRB
    grbBuf[(uint16_t)i * 3 + 1] = rgb[0];
    grbBuf[(uint16_t)i * 3 + 2] = rgb[2];
  }
}
