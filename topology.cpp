#include "topology.h"
#include "comms.h"
#include "led_layout.h"

// Axial steps for global hex direction d (0-5, CCW, d=0 along +x).
static const int8_t DQ[6] = { 1, 0, -1, -1, 0, 1 };
static const int8_t DR[6] = { 0, 1, 1, 0, -1, -1 };

// cos/sin of d*30 deg scaled by 128. Rotation is a whole number of 60 deg
// steps, but the LED table's own theta = 0 can sit half a side away from side 0
// (LED_THETA0_HALF_STEPS), so these are indexed in 30 deg steps to absorb that.
static const int8_t COS128[12] = { 127, 111, 64, 0, -64, -111, -127, -111, -64, 0, 64, 111 };
static const int8_t SIN128[12] = { 0, 64, 111, 127, 111, 64, 0, -64, -111, -127, -111, -64 };

static_assert(ANIM_CYCLE_MS / CLOCK_TICK_MS <= 65535UL,
              "one pass through the animation list must fit the 16-bit clock");
static_assert(ANIM_PERIOD_MS % CLOCK_TICK_MS == 0,
              "the dwell must be a whole number of clock ticks");

static uint8_t  myID;
static uint8_t  myRoot, myHop, myRot;
static int8_t   myQ, myR;
static int16_t  tileX, tileY;
static uint16_t animClock;           // shared clock, 0 .. ANIM_CYCLE_TICKS - 1
static uint16_t clockStamp;          // millis16 already folded into animClock
static uint8_t  clockCarryMs;        // sub-tick carry of ordinary elapsed time
static uint16_t compensateUs;        // sub-tick carry of interrupt-blackout time
static uint16_t rootAgeBaseMs;       // age of my root info as of rootAgeStamp
static uint16_t rootAgeStamp;
static uint16_t lastSyncHeard;       // beacon already folded into the clock
static int8_t   lastSyncSide;
static int8_t   gx[NUM_LEDS], gy[NUM_LEDS];   // rotated local LED coords, mm
static uint16_t lastUpdate;

// The clock runs round a circle of ANIM_CYCLE_TICKS rather than round the 16-bit
// wrap, so that the animation index is just clock / ANIM_PERIOD_TICKS and the
// list still starts at 0 after a wrap. Every step along it goes through here; d
// must be less than one full circle.
static uint16_t clock_add(uint16_t v, uint16_t d) {
  uint16_t room = ANIM_CYCLE_TICKS - v;
  return d < room ? (uint16_t)(v + d) : (uint16_t)(d - room);
}

// Fold in the millis() that have passed since the clock was last touched, the
// remainder carried rather than dropped so the tick rate stays exact. Every
// accessor starts here, so the clock is current at the moment it is used rather
// than at the last topo_update().
static void clock_tick() {
  uint16_t now = (uint16_t)millis();
  uint16_t ms  = (uint16_t)(now - clockStamp) + clockCarryMs;
  clockStamp   = now;
  clockCarryMs = (uint8_t)(ms % CLOCK_TICK_MS);
  animClock    = clock_add(animClock, (uint16_t)(ms / CLOCK_TICK_MS));
}

uint16_t topo_clock_ticks() { clock_tick(); return animClock; }

// Milliseconds for the effects. This wraps at 65536 partway through the cycle,
// which every effect's period divides, so unlike the cycle's own wrap it is
// invisible.
uint16_t topo_anim_time() { return (uint16_t)(topo_clock_ticks() * CLOCK_TICK_MS); }

// Not a stored choice that has to reach every tile — a reading of the shared
// clock, so tiles change together to within their clock spread no matter how
// many hops from the root they sit.
uint8_t topo_anim_id() { return (uint8_t)(topo_clock_ticks() / ANIM_PERIOD_TICKS); }

void topo_clock_compensate_us(uint16_t us) {
  compensateUs += us;
  while (compensateUs >= 1000U * CLOCK_TICK_MS) {
    compensateUs -= 1000U * CLOCK_TICK_MS;
    animClock = clock_add(animClock, 1);
  }
}

static uint16_t root_age_ms() {
  return rootAgeBaseMs + (uint16_t)((uint16_t)millis() - rootAgeStamp);
}
uint8_t  topo_my_id()     { return myID; }
int16_t  topo_tile_x()    { return tileX; }
int16_t  topo_tile_y()    { return tileY; }
const int8_t* topo_led_gx() { return gx; }
const int8_t* topo_led_gy() { return gy; }

#if DEBUG_PATTERN == DBG_TOPOLOGY
static int8_t  myParent = -1;
static uint8_t syncCount;      // beacons actually folded into the clock
static int16_t syncErr;        // clock error the last of them carried, in ticks
bool     topo_is_root()     { return myRoot == myID; }
uint8_t  topo_hop()         { return myHop; }
uint8_t  topo_rot()         { return myRot; }
int8_t   topo_parent_side() { return myParent; }
uint8_t  topo_sync_count()  { return syncCount; }
int16_t  topo_clock_err()   { return syncErr; }
#endif

static uint8_t mod6(int8_t v) { while (v < 0) v += 6; return (uint8_t)v % 6; }

// Divide to nearest, away from zero on a tie, so +r and -r rows stay symmetric.
static int16_t div_round(int32_t num, int32_t den) {
  return (int16_t)((num + (num >= 0 ? den / 2 : -(den / 2))) / den);
}

// Turn the tile's own LED table into the shared frame: 60 deg per rotation
// step, plus the fixed half-side offset between the table's theta = 0 and
// side 0's normal.
static void rebuild_led_cache() {
#if SIDE_CCW
  int8_t half = LED_THETA0_HALF_STEPS;
#else
  int8_t half = -LED_THETA0_HALF_STEPS;
#endif
  uint8_t a = (uint8_t)((2 * (int8_t)myRot + half + 12) % 12);
  int8_t c = COS128[a], s = SIN128[a];
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    int16_t x = (int8_t)pgm_read_byte(&LED_X[i]);
    int16_t y = (int8_t)pgm_read_byte(&LED_Y[i]);
#if !SIDE_CCW
    // A clockwise-numbered tile is a mirrored counter-clockwise one, so the
    // whole picture comes out mirrored — still seamless, which is all that
    // matters. It must be a reflection and not a backwards rotation: a rotation
    // cannot undo a handedness flip, and leaves every tile individually
    // mirrored against correct tile centres.
    y = -y;
#endif
    gx[i] = (int8_t)((x * c - y * s) >> 7);
    gy[i] = (int8_t)((x * s + y * c) >> 7);
  }
  // Axial -> mm: x = (2q + r) * pitch/2, y = r * pitch * sin60. Both derived
  // from q/r directly rather than from a rounded per-row constant, so no error
  // accumulates across the assembly. Worst case |r| = 127 fits int32 easily.
  tileX = div_round((int32_t)((int16_t)myQ * 2 + myR) * TILE_PITCH_MM, 2L);
  tileY = div_round((int32_t)myR * TILE_PITCH_MM * 866L, 1000L);
}

void topo_init() {
  // Stable per-chip ID hashed from the factory serial (0x00/0xFF reserved).
  uint8_t id = 0;
  for (uint8_t i = 0; i < 10; i++)
    id = (uint8_t)(id * 31 + ((volatile uint8_t*)&SIGROW.SERNUM0)[i]);
  if (id == 0x00) id = 0x01;
  if (id == 0xFF) id = 0xFE;
  myID = id;

  myRoot = myID; myHop = 0; myRot = 0; myQ = 0; myR = 0;
  animClock = 0; clockCarryMs = 0; compensateUs = 0;
  clockStamp = (uint16_t)millis();
  rootAgeBaseMs = 0; rootAgeStamp = clockStamp;
  lastSyncSide = -1; lastSyncHeard = 0;
  rebuild_led_cache();
}

void topo_fill_beacon(NodeInfo* out, uint8_t txSide) {
  out->rootID = myRoot;
  out->q = myQ; out->r = myR;
  out->rot = myRot; out->side = txSide;
  out->hop = myHop;
  uint16_t ageUnits = root_age_ms() / ROOT_AGE_UNIT_MS;
  out->age = ageUnits > 255 ? 255 : (uint8_t)ageUnits;
  out->t = topo_clock_ticks();
}

void topo_update() {
  uint16_t now = (uint16_t)millis();
  if ((uint16_t)(now - lastUpdate) < 50) return;   // 20 Hz is plenty
  lastUpdate = now;

  // ---- pick the best root/parent among fresh neighbors ----
  uint8_t  bestRoot = myID, bestHop = 0;
  uint16_t bestAge = 0;
  int8_t   parent = -1;
  Neighbor pn{};    // only read when parent >= 0; zeroed to keep -Wall quiet

  for (uint8_t s = 0; s < NUM_SIDES; s++) {
    Neighbor n;
    comms_get_neighbor(s, &n);
    if (!n.present) continue;

    // How stale the root's information would be if I relayed it now. Only a
    // path that actually reaches the root resets this to zero, so refusing
    // anything past the limit starves out a root-less cycle and re-runs the
    // election; hop count alone would count to 255, wrap, and make the dead
    // root look adjacent again forever.
    uint16_t candAge = (uint16_t)n.info.age * ROOT_AGE_UNIT_MS
                     + AIR_DELAY_MS + (uint16_t)(now - n.lastHeard);
    if (candAge >= ROOT_MAX_AGE_MS) continue;

    uint8_t candHop = (uint8_t)(n.info.hop + 1);
    bool better;
    if (n.info.rootID != bestRoot) better = (n.info.rootID < bestRoot);
    else if (parent < 0)           better = false;   // my own ID echoed back
    else if (candHop != bestHop)   better = (candHop < bestHop);
    else                           better = (candAge < bestAge);
    if (!better) continue;

    bestRoot = n.info.rootID;
    bestHop  = candHop;
    bestAge  = candAge;
    parent   = (int8_t)s;
    pn       = n;
  }

  uint8_t newRot; int8_t newQ, newR;
  if (parent < 0) {
    // I am root (or isolated) — my own information is never stale.
    bestRoot = myID; bestHop = 0;
    newRot = 0; newQ = 0; newR = 0;
    rootAgeBaseMs = 0;
    rootAgeStamp  = now;
  } else {
    // Place myself from the parent's beacon: it transmitted on its side
    // pn.info.side, I received on side `parent`, and mated sides face opposite
    // global directions.
    uint8_t parentDir = mod6((int8_t)(pn.info.rot + pn.info.side));
    newRot = mod6((int8_t)(pn.info.rot + pn.info.side + 3 - parent));
    newQ = (int8_t)(pn.info.q + DQ[parentDir]);
    newR = (int8_t)(pn.info.r + DR[parentDir]);
    rootAgeBaseMs = (uint16_t)pn.info.age * ROOT_AGE_UNIT_MS + AIR_DELAY_MS;
    rootAgeStamp  = pn.lastHeard;
  }

#if DEBUG_PATTERN == DBG_TOPOLOGY
  myParent = parent;
#endif

  bool moved = (newRot != myRot) || (newQ != myQ) || (newR != myR);
  myRoot = bestRoot; myHop = bestHop;
  myRot = newRot; myQ = newQ; myR = newR;
  if (moved) rebuild_led_cache();

  // ---- animation clock ----
  // Which animation is showing rides on this clock and nothing else, so there
  // is nothing here to converge separately: the root free-runs, everyone else
  // steers onto their parent, and the changeover is as tight as the sync is.
  if (parent >= 0) {
    // Only ever act on a beacon once. This runs at 20 Hz but beacons arrive at
    // ~2.5 Hz, so re-slewing on a stale measurement would apply one correction
    // eight times over; and (now - lastHeard) undercounts real time by the LED
    // blackout duty cycle, a bias proportional to the measurement's own age.
    if (parent != lastSyncSide || pn.lastHeard != lastSyncHeard) {
      lastSyncSide  = parent;
      lastSyncHeard = pn.lastHeard;

      // The timestamp arrived over the wire, and a value off the circle would
      // make every later step on it wrap somewhere else.
      uint16_t sent = pn.info.t < ANIM_CYCLE_TICKS ? pn.info.t : 0;

      // Parent's clock at "now" = beacon timestamp + airtime + elapsed since RX,
      // that delay rounded to the nearest tick rather than truncated so it does
      // not bias every tile in the assembly the same way.
      uint16_t delayMs = AIR_DELAY_MS + (uint16_t)(now - pn.lastHeard);
      uint16_t target  = clock_add(sent, (delayMs + CLOCK_TICK_MS / 2) / CLOCK_TICK_MS);
#if DEBUG_PATTERN == DBG_TOPOLOGY
      uint16_t mine  = topo_clock_ticks();
      uint16_t ahead = target >= mine ? (uint16_t)(target - mine)
                                      : (uint16_t)(target + ANIM_CYCLE_TICKS - mine);
      syncCount++;
      syncErr = ahead <= ANIM_CYCLE_TICKS / 2 ? (int16_t)ahead  // short way round
                                              : (int16_t)(ahead - ANIM_CYCLE_TICKS);
#endif
      // Take the whole error, every time. Where the clock sits is the only
      // handle on it — nothing adjusts its *rate* — so a capped correction is not a
      // gentler version of this, it is a ceiling on how fast a parent's
      // oscillator may run before the child can never catch it. An 8 ms cap at
      // ~2.3 beacons/s put that ceiling at 1.9%, which two untrimmed RC
      // oscillators clear easily. Correcting fully leaves only the drift
      // between beacons (~13 ms at 3%) plus measurement noise, which is under a
      // frame and invisible in any of the effects.
      animClock    = target;
      clockStamp   = now;
      clockCarryMs = 0;
    }
  }
}
