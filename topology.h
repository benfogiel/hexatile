#pragma once
#include "config.h"

// Distributed topology: every tile beacons its NodeInfo on all 6 sides.
// Root = tile with the smallest ID reachable. Each tile picks as parent the
// neighbor advertising the best (rootID, hop) and derives its own coordinate
// and rotation purely from that neighbor's beacon:
//
//   my_rot  = (nb.rot + nb.side + 3 - my_rx_side) mod 6
//   my_pos  = nb.pos + DIR[(nb.rot + nb.side) mod 6]
//
// so a single received beacon fully places a tile. Moving a tile silences
// one side, lights up another, and the map heals within ~2 s.
//
// Beacons also carry the age of the root information they relay, which is what
// makes the tree loop-safe: see ROOT_MAX_AGE_MS in config.h.

void     topo_init();
void     topo_update();                    // call from loop(), self rate-limits
void     topo_fill_beacon(NodeInfo* out, uint8_t txSide);  // used by comms
uint8_t  topo_my_id();

// Hand back time the caller spent with interrupts off (strip.show()), which
// millis() did not count. Without it the clock runs slow by the LED duty cycle.
void     topo_clock_compensate_us(uint16_t us);

// For the renderer:
uint16_t topo_anim_time();                 // network-synced ms clock
uint8_t  topo_anim_id();
int16_t  topo_tile_x();                    // tile centre in global mm
int16_t  topo_tile_y();
const int8_t* topo_led_gx();               // per-LED rotated local x (mm)
const int8_t* topo_led_gy();

#if DEBUG_PATTERN == DBG_TOPOLOGY
bool     topo_is_root();
uint8_t  topo_hop();
uint8_t  topo_rot();
int8_t   topo_parent_side();               // my side the coordinate came from, -1 if root
uint8_t  topo_sync_count();                // beacons folded into the clock, wraps
int16_t  topo_clock_err();                 // error the last of them carried, ms
#endif
