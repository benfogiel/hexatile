// Host-side regression test for the tile topology: root election, aging,
// coordinate/rotation derivation and clock sync. Build and run with `make`.
//
// The real ../topology.cpp is compiled once per tile inside its own namespace,
// so each tile gets a private copy of the module's file-scope state and the
// test exercises the shipping algorithm rather than a paraphrase of it. The
// firmware headers are included globally first, so the namespaced includes
// pick up only the .cpp body (their include guards are already satisfied).
//
// Everything below the firmware is a model: ../sim/Arduino.h fakes millis()
// and SIGROW, and this file fakes the wire — beacons are handed to whichever
// tile is physically mated to the sending side, after AIR_DELAY_MS.

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "config.h"
#include "comms.h"
#include "topology.h"
#include "led_layout.h"

uint32_t g_now_ms = 0;
SigRow   SIGROW;

#define NTILE 7

static int      g_tile = 0;
static Neighbor g_nbr[NTILE][NUM_SIDES];

void comms_get_neighbor(uint8_t side, Neighbor* out) { *out = g_nbr[g_tile][side]; }

namespace T0 {
#include "../topology.cpp"
}
namespace T1 {
#include "../topology.cpp"
}
namespace T2 {
#include "../topology.cpp"
}
namespace T3 {
#include "../topology.cpp"
}
namespace T4 {
#include "../topology.cpp"
}
namespace T5 {
#include "../topology.cpp"
}
namespace T6 {
#include "../topology.cpp"
}

struct TileApi {
  void     (*init)();
  void     (*update)();
  void     (*fill)(NodeInfo*, uint8_t);
  uint8_t  (*id)();
  uint16_t (*clock)();
};
#define API(N) { N::topo_init, N::topo_update, N::topo_fill_beacon, \
                 N::topo_my_id, N::topo_anim_time }
static const TileApi API_TBL[NTILE] = {
  API(T0), API(T1), API(T2), API(T3), API(T4), API(T5), API(T6)
};

// ---------------------------------------------------------------------------
// Physical truth: where each tile really sits and how it is really turned.
// The firmware never sees any of this; it is only used to score the result.
static const int DQ6[6] = { 1, 0, -1, -1, 0, 1 };
static const int DR6[6] = { 0, 1, 1, 0, -1, -1 };

struct Truth { int q, r, rot; bool present; };
static Truth truth[NTILE];

static double   rate[NTILE];      // per-tile millis() rate error
static uint32_t skew[NTILE];      // per-tile millis() origin
static uint16_t nextBeacon[NTILE][NUM_SIDES];
static uint32_t trueMs;
static int      lossPercent;

static void enter(int t) {
  g_tile   = t;
  g_now_ms = skew[t] + (uint32_t)(trueMs * rate[t]);
}

static int find_tile(int q, int r) {
  for (int i = 0; i < NTILE; i++)
    if (truth[i].present && truth[i].q == q && truth[i].r == r) return i;
  return -1;
}

// The tile mated to side s of tile t, and which of its sides faces back.
static int mate_of(int t, int s, int* backSide) {
  int d = (truth[t].rot + s) % 6;
  int u = find_tile(truth[t].q + DQ6[d], truth[t].r + DR6[d]);
  if (u < 0) return -1;
  *backSide = ((d + 3) - truth[u].rot % 6 + 12) % 6;
  return u;
}

struct Packet { uint32_t dueMs; int dst; uint8_t dstSide; NodeInfo info; };
static Packet queue[256];
static int    qn;

// ---------------------------------------------------------------------------
// A tile's ID is hashed from its factory serial; invert that hash to put a
// chosen ID on a chosen tile (31^9 == 31 mod 256, and 31 * 223 == 1 mod 256).
static void set_id(int t, uint8_t wantID) {
  memset(&SIGROW, 0, sizeof(SIGROW));
  SIGROW.SERNUM0 = (uint8_t)((wantID * 223) & 0xFF);
  enter(t);
  API_TBL[t].init();
  if (API_TBL[t].id() != wantID) {
    printf("  !! tile %d hashed to id %u, wanted %u\n", t, API_TBL[t].id(), wantID);
    exit(2);
  }
  for (uint8_t s = 0; s < NUM_SIDES; s++) {
    memset(&g_nbr[t][s], 0, sizeof(Neighbor));
    nextBeacon[t][s] = (uint16_t)g_now_ms + (uint16_t)s * (BEACON_MS / NUM_SIDES)
                     + (wantID & 0x1F);
  }
}

// comms_poll(), minus the bit-banging.
static void tile_comms(int t) {
  uint16_t now = (uint16_t)g_now_ms;
  for (uint8_t s = 0; s < NUM_SIDES; s++) {
    if (g_nbr[t][s].present &&
        (uint16_t)(now - g_nbr[t][s].lastHeard) > NEIGHBOR_TIMEOUT_MS)
      g_nbr[t][s].present = false;

    if ((int16_t)(now - nextBeacon[t][s]) >= 0) {
      NodeInfo info;
      API_TBL[t].fill(&info, s);
      int back, u = mate_of(t, s, &back);
      if (u >= 0 && qn < 256 && (rand() % 100) >= lossPercent) {
        queue[qn].dueMs   = trueMs + AIR_DELAY_MS;
        queue[qn].dst     = u;
        queue[qn].dstSide = (uint8_t)back;
        queue[qn].info    = info;
        qn++;
      }
      nextBeacon[t][s] = now + BEACON_MS
                       + ((API_TBL[t].id() * 13u + s * 29u + now) & 0x3F);
    }
  }
}

static void deliver_due() {
  for (int i = 0; i < qn; ) {
    if (queue[i].dueMs > trueMs) { i++; continue; }
    int d = queue[i].dst;
    if (truth[d].present) {
      enter(d);
      g_nbr[d][queue[i].dstSide].info      = queue[i].info;
      g_nbr[d][queue[i].dstSide].lastHeard = (uint16_t)g_now_ms;
      g_nbr[d][queue[i].dstSide].present   = true;
    }
    queue[i] = queue[--qn];
  }
}

static void run_ms(uint32_t ms) {
  for (uint32_t k = 0; k < ms; k++) {
    trueMs++;
    deliver_due();
    for (int t = 0; t < NTILE; t++) {
      if (!truth[t].present) continue;
      enter(t);
      tile_comms(t);
      API_TBL[t].update();
    }
  }
}

// ---------------------------------------------------------------------------
#define MAX_CLOCK_SPREAD_MS 60

static void rot60(int* q, int* r) { int nq = -*r, nr = *q + *r; *q = nq; *r = nr; }

// Converged means: one agreed root, and every tile's derived (q, r, rot) is the
// true layout put through one shared rotation + translation — the one that
// lands the elected root at (0, 0, rot 0). Absolute coordinates are arbitrary;
// only their consistency with the physical arrangement matters.
static bool converged(char* why, size_t whyLen) {
  NodeInfo v[NTILE];
  for (int t = 0; t < NTILE; t++) {
    if (!truth[t].present) continue;
    enter(t);
    API_TBL[t].fill(&v[t], 0);
  }

  int root = -1;
  uint8_t rootID = 0;
  bool haveID = false;
  for (int t = 0; t < NTILE; t++) {
    if (!truth[t].present) continue;
    if (!haveID) { rootID = v[t].rootID; haveID = true; }
    else if (v[t].rootID != rootID) {
      snprintf(why, whyLen, "roots disagree");
      return false;
    }
    if (v[t].hop == 0 && API_TBL[t].id() == v[t].rootID) root = t;
  }
  if (root < 0) {
    snprintf(why, whyLen, "root %u is not present (stale)", rootID);
    return false;
  }

  int K = (6 - truth[root].rot % 6) % 6;
  for (int t = 0; t < NTILE; t++) {
    if (!truth[t].present) continue;
    int eq = truth[t].q - truth[root].q, er = truth[t].r - truth[root].r;
    for (int i = 0; i < K; i++) rot60(&eq, &er);
    int erot = (truth[t].rot + K) % 6;
    if (v[t].q != eq || v[t].r != er || v[t].rot != erot) {
      snprintf(why, whyLen, "tile %d placed (%d,%d,rot%u), should be (%d,%d,rot%d)",
               t, v[t].q, v[t].r, v[t].rot, eq, er, erot);
      return false;
    }
  }

  int spread = 0;
  for (int a = 0; a < NTILE; a++) {
    if (!truth[a].present) continue;
    enter(a); uint16_t ca = API_TBL[a].clock();
    for (int b = 0; b < NTILE; b++) {
      if (!truth[b].present) continue;
      enter(b); uint16_t cb = API_TBL[b].clock();
      int d = (int)(int16_t)(ca - cb); if (d < 0) d = -d;
      if (d > spread) spread = d;
    }
  }
  if (spread > MAX_CLOCK_SPREAD_MS) {
    snprintf(why, whyLen, "clocks %d ms apart", spread);
    return false;
  }
  snprintf(why, whyLen, "root %u, clock spread %d ms", rootID, spread);
  return true;
}

static int failures;

// Run up to limitSec, reporting how long convergence actually took.
static void expect_within(const char* what, int limitSec) {
  char why[128];
  for (int sec = 1; sec <= limitSec; sec++) {
    run_ms(1000);
    if (converged(why, sizeof why)) {
      printf("  PASS  %-38s %2d s  (%s)\n", what, sec, why);
      return;
    }
  }
  printf("  FAIL  %-38s >%d s  (%s)\n", what, limitSec, why);
  failures++;
}

// ---------------------------------------------------------------------------
// Centre tile plus a full ring: dense with 3-cycles, which is the topology that
// makes a distance-vector tree count to infinity when its root disappears.
static void build_flower() {
  static const uint8_t IDS[NTILE] = { 40, 90, 70, 20, 110, 60, 130 };
  truth[0] = { 0, 0, 0, true };
  for (int i = 0; i < 6; i++) truth[1 + i] = { DQ6[i], DR6[i], (i * 5) % 6, true };

  qn = 0; trueMs = 0;
  for (int t = 0; t < NTILE; t++) {
    rate[t] = 1.0 + (t - 3) * 0.002;          // +/-0.6% clock rate error
    skew[t] = (uint32_t)t * 9173u;            // different power-on instants
  }
  for (int t = 0; t < NTILE; t++) set_id(t, IDS[t]);
}

int main() {
  srand(1);

  printf("\n7-tile flower, clock skew + /-0.6%% rate error\n");
  lossPercent = 0;
  build_flower();
  expect_within("cold start converges", 10);

  // The point of the age byte in the beacon. Before it existed, the six
  // survivors kept relaying the dead root forever, hop count counting up to
  // 255 and wrapping, and this never recovered at all.
  printf("\nroot tile (id 20) unplugged from the ring\n");
  truth[3].present = false;
  for (int s = 0; s < NUM_SIDES; s++) g_nbr[3][s].present = false;
  expect_within("re-elects a new root", 15);

  printf("\ntile 6 moved to a different cell and rotation\n");
  truth[6] = { -1, -1, 2, true };
  expect_within("map heals", 10);

  printf("\n7-tile flower, 20%% packet loss\n");
  lossPercent = 20;
  build_flower();
  expect_within("cold start converges", 20);

  printf("\n%s\n\n", failures ? "FAILED" : "all checks passed");
  return failures ? 1 : 0;
}
