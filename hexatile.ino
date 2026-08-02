// Hexatile firmware — identical image on every tile.
//
// Target: ATtiny1604 @ 20 MHz internal, megaTinyCore
//   Tools > Board: ATtiny1604 (0-series) ... Clock: 20 MHz internal
//   millis timer: default. Do NOT use Arduino's attachInterrupt() anywhere
//   (comms.cpp owns the PORTA/PORTB interrupt vectors directly).
//
// How it works (short version):
//   * No pixel data ever crosses tiles. Tiles exchange tiny beacons on the
//     six side pins, elect the lowest-ID tile as root, and each tile derives
//     its own hex-grid coordinate + rotation from any neighbor's beacon.
//   * A network-synced millisecond clock rides along in the beacons.
//   * Each frame, every tile evaluates color = f(global_x, global_y, t)
//     for its own 91 LEDs. Same function everywhere => seamless animation.

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
  comms_poll();          // beacons out, expire vacated sides
  topo_update();         // election / coordinates / clock slew (20 Hz internally)
  anim_render(pixels);
  strip.show();          // ~2.7 ms, interrupts off inside; a byte arriving
                         // during this is lost and simply retried next beacon
  topo_clock_compensate_us(LED_SHOW_US);   // millis() stopped for that window
  delay(8);              // ~30+ fps and guarantees WS2812 latch time
}
