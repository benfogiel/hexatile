#pragma once
#include "config.h"

// Animations are pure functions of (global x mm, global y mm, time) -> color,
// evaluated locally per LED. That is the whole trick: no pixel data ever
// crosses a tile boundary, so a slow link is enough.

void anim_render(uint8_t* grbBuf);   // fills NUM_LEDS*3 bytes, GRB order
