// Host-side regression test for the tile topology: root election, aging,
// coordinate/rotation derivation and clock sync. Build and run with `make`.
//
// The real ../topology.cpp is compiled once per tile inside its own namespace,
// so each tile gets a private copy of the module's file-scope state and the
// test exercises the shipping algorithm rather than a paraphrase of it. The
// firmware headers are included globally first, so the namespaced includes
// pick up only the .cpp body (their include guards are already satisfied).
//
// Everything else here is a model: Arduino.h fakes millis() and SIGROW, and
// this file fakes the wire.

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

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
  uint16_t (*clockTicks)();
  uint8_t  (*anim)();
  void     (*compensate)(uint16_t);
  int16_t  (*tileX)();
  int16_t  (*tileY)();
  const int8_t* (*ledX)();
  const int8_t* (*ledY)();
};
#define API(N) { N::topo_init, N::topo_update, N::topo_fill_beacon, \
                 N::topo_my_id, N::topo_clock_ticks, N::topo_anim_id, \
                 N::topo_clock_compensate_us, \
                 N::topo_tile_x, N::topo_tile_y, N::topo_led_gx, N::topo_led_gy }
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
static uint8_t  linkGrace[NTILE][NUM_SIDES];

static uint32_t trueMs;
static int      lossPercent;

// Transmitting masks every side's pin interrupt for the whole packet, so a tile
// is deaf on all six sides while it beacons on any one of them.
static uint32_t deafUntil[NTILE];
static int      pktSent, pktLostDeaf, pktDelivered;

// The render loop hands LED_SHOW_US back to the clock after every strip.show(),
// assuming millis() lost exactly that much. At ~70 fps that is ~190 ms/s of
// open-loop rate adjustment, so whatever part of the assumption is wrong is a
// rate error — and it scales with a tile's frame rate, which two tiles doing
// different amounts of comms do not share. showLoss is the fraction millis()
// REALLY loses: 1.0 makes the compensation exactly right, 0.0 makes all of it
// error.
static double fps[NTILE];
static double showLoss;
static double   frameAcc[NTILE];
static double   lostMs[NTILE];
static uint32_t lastNow[NTILE];

// millis() never runs backwards on the chip: strip.show() stalls the timer, it
// does not rewind it. lostMs lands in one lump at the end of a frame, so clamp
// the result monotonic — the tile sees its clock flat for the blackout and then
// resume, which is what the hardware does.
static void enter(int t) {
  g_tile = t;
  uint32_t v = skew[t] + (uint32_t)(trueMs * rate[t] - lostMs[t]);
  if (v < lastNow[t]) v = lastNow[t];
  lastNow[t] = v;
  g_now_ms = v;
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
    linkGrace[t][s] = LINK_GRACE_BEACONS;
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
    if (g_nbr[t][s].present) linkGrace[t][s] = LINK_GRACE_BEACONS;

    if ((int16_t)(now - nextBeacon[t][s]) >= 0) {
      NodeInfo info;
      API_TBL[t].fill(&info, s);
      deafUntil[t] = trueMs + AIR_DELAY_MS;
      int back, u = mate_of(t, s, &back);
      if (u >= 0 && qn < 256 && (rand() % 100) >= lossPercent) {
        pktSent++;
        queue[qn].dueMs   = trueMs + AIR_DELAY_MS;
        queue[qn].dst     = u;
        queue[qn].dstSide = (uint8_t)back;
        queue[qn].info    = info;
        qn++;
      }
      uint16_t interval = IDLE_BEACON_MS;
      if (g_nbr[t][s].present) interval = BEACON_MS;
      else if (linkGrace[t][s]) { interval = BEACON_MS; linkGrace[t][s]--; }
      nextBeacon[t][s] = now + interval
                       + ((API_TBL[t].id() * 13u + s * 29u + now) & 0x3F);
    }
  }
}

static void deliver_due() {
  for (int i = 0; i < qn; ) {
    if (queue[i].dueMs > trueMs) { i++; continue; }
    int d = queue[i].dst;
    // Receiving needs the whole packet: a destination that transmitted at any
    // point while this one was in the air missed bytes, and the CRC drops it.
    bool deaf = deafUntil[d] > queue[i].dueMs - AIR_DELAY_MS;
    if (truth[d].present && !deaf) {
      pktDelivered++;
      enter(d);
      g_nbr[d][queue[i].dstSide].info      = queue[i].info;
      g_nbr[d][queue[i].dstSide].lastHeard = (uint16_t)g_now_ms;
      g_nbr[d][queue[i].dstSide].present   = true;
    } else if (truth[d].present) {
      pktLostDeaf++;
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

      // Render frames at this tile's own rate, each stopping millis() for its
      // share of the LED blackout and handing back the compile-time guess.
      frameAcc[t] += fps[t] / 1000.0;
      while (frameAcc[t] >= 1.0) {
        frameAcc[t] -= 1.0;
        lostMs[t] += showLoss * LED_SHOW_US / 1000.0;
        enter(t);
        API_TBL[t].compensate(LED_SHOW_US);
      }
    }
  }
}

// ---------------------------------------------------------------------------
#define MAX_CLOCK_SPREAD_MS 60

// The shared clock is a position on a circle of ANIM_CYCLE_TICKS, so two
// readings are compared the short way round it; the answer is in ms.
static int clock_gap(uint16_t a, uint16_t b) {
  uint16_t fwd = b >= a ? b - a : (uint16_t)(b + ANIM_CYCLE_TICKS - a);
  uint16_t d = fwd <= ANIM_CYCLE_TICKS / 2 ? fwd : (uint16_t)(ANIM_CYCLE_TICKS - fwd);
  return d * CLOCK_TICK_MS;
}

// A changeover is only as sharp as the clock the tiles derive it from, so the
// budget is the clock spread itself plus a frame.
#define MAX_ANIM_SPLIT_MS   (MAX_CLOCK_SPREAD_MS + 20)

// Where the renderer may put an LED versus where it physically is, once the
// whole assembly is put through its one shared transform. The budget covers
// integer rounding only: the >>7 fixed-point rotation truncates, and 127/111
// stand in for 128 cos 0 / 128 cos 30.
#define MAX_LED_ERR_MM 2.5

static void rot60(int* q, int* r) { int nq = -*r, nr = *q + *r; *q = nq; *r = nr; }

static void rot_deg(double* x, double* y, double deg) {
  double c = cos(deg * M_PI / 180.0), s = sin(deg * M_PI / 180.0);
  double nx = *x * c - *y * s;
  *y = *x * s + *y * c;
  *x = nx;
}

// Physical truth for one LED: tile centre from the axial cell, plus the LED's
// own table entry turned to the tile's real orientation. The table's theta = 0
// sits LED_THETA0_HALF_STEPS half-sides from side 0's normal, which physically
// points along the tile's rotation — that offset is the subject of the check
// below.
static void led_truth(int t, int i, int qOrigin, int rOrigin, double* x, double* y) {
  *x = (double)(int8_t)pgm_read_byte(&LED_X[i]);
  *y = (double)(int8_t)pgm_read_byte(&LED_Y[i]);
  rot_deg(x, y, 60.0 * (truth[t].rot % 6) + 30.0 * LED_THETA0_HALF_STEPS);
  int dq = truth[t].q - qOrigin, dr = truth[t].r - rOrigin;
  *x += (2.0 * dq + dr) * TILE_PITCH_MM / 2.0;
  *y += dr * TILE_PITCH_MM * 0.866;
}

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

  // Agreeing on (q, r, rot) is not the same as rendering in one frame. Every
  // tile's LED coordinates, as the renderer will actually use them, must be the
  // physical layout put through that same transform. A half-side twist between
  // a tile's LED table and its own sides passes every check above and still
  // tears the picture at each seam, because tile centres stay right while their
  // contents rotate. (Models a counter-clockwise board: see SIDE_CCW.)
  double worstLed = 0;
  for (int t = 0; t < NTILE; t++) {
    if (!truth[t].present) continue;
    enter(t);
    double ox = API_TBL[t].tileX(), oy = API_TBL[t].tileY();
    const int8_t* lx = API_TBL[t].ledX();
    const int8_t* ly = API_TBL[t].ledY();
    for (int i = 0; i < NUM_LEDS; i++) {
      double ex, ey;
      led_truth(t, i, truth[root].q, truth[root].r, &ex, &ey);
      rot_deg(&ex, &ey, 60.0 * K);
      double dx = (ox + lx[i]) - ex, dy = (oy + ly[i]) - ey;
      double d = sqrt(dx * dx + dy * dy);
      if (d > worstLed) worstLed = d;
    }
  }
  if (worstLed > MAX_LED_ERR_MM) {
    snprintf(why, whyLen, "LED frame off by %.1f mm (%.0f deg twist at the rim?)",
             worstLed, worstLed / 43.0 * 180.0 / M_PI);
    return false;
  }

  int spread = 0;
  for (int a = 0; a < NTILE; a++) {
    if (!truth[a].present) continue;
    enter(a); uint16_t ca = API_TBL[a].clockTicks();
    for (int b = 0; b < NTILE; b++) {
      if (!truth[b].present) continue;
      enter(b); uint16_t cb = API_TBL[b].clockTicks();
      int d = clock_gap(ca, cb);
      if (d > spread) spread = d;
    }
  }
  if (spread > MAX_CLOCK_SPREAD_MS) {
    snprintf(why, whyLen, "clocks %d ms apart", spread);
    return false;
  }
  snprintf(why, whyLen, "root %u, clock spread %d ms, LEDs within %.1f mm",
           rootID, spread, worstLed);
  return true;
}

static int failures;

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
    fps[t]  = 70.0 + t;                       // slightly different render rates
    frameAcc[t] = 0; lostMs[t] = 0; lastNow[t] = 0; deafUntil[t] = 0;
  }
  for (int t = 0; t < NTILE; t++) set_id(t, IDS[t]);
}

// Two tiles on a bench: one mated edge each, ten open ones between them. The
// interesting number is not convergence but how much of the only link that
// exists survives the deafness both tiles inflict on themselves.
static void build_pair() {
  static const uint8_t IDS[2] = { 40, 90 };
  for (int t = 0; t < NTILE; t++) truth[t].present = false;
  truth[0] = { 0, 0, 0, true };
  truth[1] = { 1, 0, 4, true };
  qn = 0; trueMs = 0;
  pktSent = pktLostDeaf = pktDelivered = 0;
  for (int t = 0; t < NTILE; t++) {
    rate[t] = 1.0; skew[t] = 0; deafUntil[t] = 0;
    fps[t] = 70.0; frameAcc[t] = 0; lostMs[t] = 0; lastNow[t] = 0;
  }
  rate[0] = 0.98; rate[1] = 1.02;
  fps[0] = 72.0; fps[1] = 66.0;    // the two tiles do not render in lockstep
  skew[1] = 5000;
  for (int t = 0; t < 2; t++) set_id(t, IDS[t]);
}

// Convergence is scored the instant it first passes, which is far too early to
// see a rate error win: a sync loop that cannot hold its parent still looks
// perfect one second in. So soak the pair and watch what it *holds* — the worst
// clock spread, and the longest the follower spends believing it is a root
// again, which is what a dropped neighbor looks like on the tile.
static void soak_pair(int seconds, int* worstGapMs, int* worstSpreadMs) {
  int gap = 0;
  *worstGapMs = 0;
  *worstSpreadMs = 0;
  run_ms(5000);            // let the cold-start offset jump settle out first
  for (int k = 0; k < seconds * 1000; k++) {
    run_ms(1);

    enter(1);
    NodeInfo v; API_TBL[1].fill(&v, 0);
    if (v.hop == 0) { if (++gap > *worstGapMs) *worstGapMs = gap; }
    else gap = 0;

    enter(0); uint16_t c0 = API_TBL[0].clockTicks();
    enter(1); uint16_t c1 = API_TBL[1].clockTicks();
    int d = clock_gap(c0, c1);
    if (d > *worstSpreadMs) *worstSpreadMs = d;
  }
}

// Clock spread is only half of "in sync": the tiles also have to change
// animation together. Watch the whole flower across a changeover and time how
// long it spends showing two different animations at once.
static void soak_anim(int seconds, int* worstSplitMs) {
  int split = 0;
  *worstSplitMs = 0;
  for (int k = 0; k < seconds * 1000; k++) {
    run_ms(1);
    uint8_t first = 0;
    bool have = false, agree = true;
    for (int t = 0; t < NTILE; t++) {
      if (!truth[t].present) continue;
      enter(t);
      uint8_t a = API_TBL[t].anim();
      if (!have) { first = a; have = true; }
      else if (a != first) agree = false;
    }
    if (agree) split = 0;
    else if (++split > *worstSplitMs) *worstSplitMs = split;
  }
}

// ---------------------------------------------------------------------------
int main() {
  srand(1);
  showLoss = 1.0;    // compensation exactly right: the honest case for noise

  printf("\n7-tile flower, clock skew + /-0.6%% rate error\n");
  lossPercent = 0;
  build_flower();
  expect_within("cold start converges", 10);

  // The case that motivated the age byte: before it existed the six survivors
  // relayed the dead root forever and this never recovered at all.
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

  printf("\n2 tiles, 1 mated edge each, 4%% apart oscillators\n");
  lossPercent = 0;
  build_pair();
  expect_within("pair converges", 10);
  int dropped, spread;
  soak_pair(60, &dropped, &spread);
  printf("  ....  link %d/%d delivered (%.0f%% lost to self-deafness)\n",
         pktDelivered, pktSent,
         pktSent ? 100.0 * pktLostDeaf / pktSent : 0.0);
  printf("  %s  follower held the root for %d ms over 60 s\n",
         dropped ? "!!  " : "PASS", dropped);
  printf("  %s  clock spread over 60 s: %d ms\n",
         spread > MAX_CLOCK_SPREAD_MS ? "FAIL" : "PASS", spread);
  if (dropped) failures++;
  if (spread > MAX_CLOCK_SPREAD_MS) failures++;

  // The old design carried the animation index in the beacon, so a changeover
  // walked out from the root one beacon per hop and stalled wherever a packet
  // was lost — seconds of the assembly showing two animations at once. Deriving
  // it from the shared clock instead makes loss irrelevant to it, so this runs
  // at the harsher loss rate.
  printf("\n7-tile flower, animation changeover under 20%% loss\n");
  lossPercent = 20;
  build_flower();
  expect_within("cold start converges", 10);
  int split;
  soak_anim(100, &split);
  printf("  %s  split across two animations for %d ms\n",
         split > MAX_ANIM_SPLIT_MS ? "FAIL" : "PASS", split);
  if (split > MAX_ANIM_SPLIT_MS) failures++;

  printf("\n%s\n\n", failures ? "FAILED" : "all checks passed");
  return failures ? 1 : 0;
}
