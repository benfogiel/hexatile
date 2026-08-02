#pragma once
#include "config.h"

// Single-wire, half-duplex, bit-banged 8N1 UART on each of the 6 side pins
// (board sides 1..6 = PA5, PA4, PA2, PB0, PB1, PA7; index here is label - 1).
// Lines idle high via internal pull-ups on both mated tiles. RX is driven by
// falling-edge pin interrupts (one ISR entry per byte, ~1.1 ms blocking at
// 9600). TX disables the pin's own edge interrupt while driving the line.
// Collisions are possible and tolerated: CRC drops the packet, the next
// beacon retries.

void comms_init(uint8_t myID);
void comms_poll();                       // send due beacons, expire neighbors

// Neighbor state as heard on each of my sides (index = MY side number).
// Copies atomically — neighbor data is written from the RX interrupt.
void comms_get_neighbor(uint8_t side, Neighbor* out);
