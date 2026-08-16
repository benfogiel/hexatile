// Hexatile firmware — identical image on every tile.
//
// Target: ATtiny1604 @ 20 MHz internal, megaTinyCore, default millis timer.
// Do NOT use Arduino's attachInterrupt() anywhere: comms.cpp owns the
// PORTA/PORTB interrupt vectors directly.
//
// Tiles exchange beacons on the six side pins, elect the lowest-ID tile as
// root, and derive their own hex-grid coordinate, rotation and a shared clock
// from any neighbor's beacon. Each frame every tile then evaluates
// color = f(global_x, global_y, t) for its own LEDs — same function
// everywhere, so no pixel data ever has to cross a tile boundary.

#include "config.h"
#include "led_layout.h"
#include "comms.h"
#include "topology.h"
#include "anim.h"
#include <tinyNeoPixel_Static.h>

byte pixels[NUM_LEDS * 3];
tinyNeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB, pixels);

void setup() {
  pinMode(LED_PIN, OUTPUT);
  topo_init();
  comms_init(topo_my_id());
}

void loop() {
  comms_poll();
  topo_update();
  anim_render(pixels);
  strip.show();          // interrupts off inside; a byte arriving during this
                         // is lost and simply retried on the next beacon
  topo_clock_compensate_us(LED_SHOW_US);
  delay(8);              // ~30+ fps, and guarantees WS2812 latch time
}
