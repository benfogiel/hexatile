#include "topology.h"
#include "comms.h"
#include "led_layout.h"

// Global hex direction d (0-5, CCW, d=0 along +x): axial coordinate steps.
static const int8_t DQ[6] = { 1, 0, -1, -1, 0, 1 };
static const int8_t DR[6] = { 0, 1, 1, 0, -1, -1 };

// cos/sin of d*60deg scaled by 128, for rotating LED coordinates.
static const int8_t COS128[6] = { 127, 64, -64, -127, -64, 64 };
static const int8_t SIN128[6] = { 0, 111, 111, 0, -111, -111 };

static_assert(ANIM_PERIOD_MS < 65535U, "anim period must fit the 16-bit clock");

static uint8_t  myID;
static uint8_t  myRoot, myHop, myRot, myAnim;
static int8_t   myQ, myR;
static int16_t  tileX, tileY;
static uint16_t animOffset;          // anim_time = millis16 + offset (wraps freely)
static uint16_t compensateUs;        // sub-ms carry of interrupt-blackout time
static uint16_t rootAgeBaseMs;       // age of my root info as of rootAgeStamp
static uint16_t rootAgeStamp;
static uint16_t animChangedAt;
static uint16_t lastSyncHeard;       // beacon already folded into the clock
static int8_t   lastSyncSide;
static int8_t   gx[NUM_LEDS], gy[NUM_LEDS];   // rotated local LED coords, mm
static uint16_t lastUpdate;

// Max clock correction per beacon. One beacon per ~400 ms gives 2% of authority
// over the parent's rate — ample once the blackout compensation is in, and slow
// enough that the correction is never visible.
#define SLEW_MS 8

uint16_t topo_anim_time() { return (uint16_t)millis() + animOffset; }

// Time the caller spent with interrupts off, which millis() therefore missed.
void topo_clock_compensate_us(uint16_t us) {
  compensateUs += us;
  while (compensateUs >= 1000) { compensateUs -= 1000; animOffset++; }
}

static uint16_t root_age_ms() {
  return rootAgeBaseMs + (uint16_t)((uint16_t)millis() - rootAgeStamp);
}
uint8_t  topo_anim_id()   { return myAnim; }
uint8_t  topo_my_id()     { return myID; }
int16_t  topo_tile_x()    { return tileX; }
int16_t  topo_tile_y()    { return tileY; }
const int8_t* topo_led_gx() { return gx; }
const int8_t* topo_led_gy() { return gy; }

static uint8_t mod6(int8_t v) { while (v < 0) v += 6; return (uint8_t)v % 6; }

// Divide to nearest, away from zero on a tie, so +r and -r rows stay symmetric.
static int16_t div_round(int32_t num, int32_t den) {
  return (int16_t)((num + (num >= 0 ? den / 2 : -(den / 2))) / den);
}

static void rebuild_led_cache() {
  int8_t c = COS128[myRot], s = SIN128[myRot];
#if !SIDE_CCW
  s = -s;                      // clockwise physical side numbering
#endif
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    int16_t x = (int8_t)pgm_read_byte(&LED_X[i]);
    int16_t y = (int8_t)pgm_read_byte(&LED_Y[i]);
    gx[i] = (int8_t)((x * c - y * s) >> 7);
    gy[i] = (int8_t)((x * s + y * c) >> 7);
  }
  // Axial -> mm, both terms derived from q/r directly rather than from a
  // rounded per-row constant, so no error accumulates across the assembly and
  // a fractional pitch costs nothing.
  //   x = (2q + r) * pitch/2        y = r * pitch * sin60
  // Worst case here is |r| = 127: 127 * 85 * 866 fits int32 with room to spare.
  tileX = div_round((int32_t)((int16_t)myQ * 2 + myR) * TILE_PITCH_MM, 2L);
  tileY = div_round((int32_t)myR * TILE_PITCH_MM * 866L, 1000L);
}

void topo_init() {
  // Stable per-chip ID from the factory serial number (avoid 0x00/0xFF).
  uint8_t id = 0;
  for (uint8_t i = 0; i < 10; i++)
    id = (uint8_t)(id * 31 + ((volatile uint8_t*)&SIGROW.SERNUM0)[i]);
  if (id == 0x00) id = 0x01;
  if (id == 0xFF) id = 0xFE;
  myID = id;

  myRoot = myID; myHop = 0; myRot = 0; myQ = 0; myR = 0;
  myAnim = 0; animOffset = 0; compensateUs = 0;
  rootAgeBaseMs = 0; rootAgeStamp = (uint16_t)millis();
  animChangedAt = rootAgeStamp;
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
  out->t = topo_anim_time();
  out->animID = myAnim;
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
    // path that actually reaches the root ever gets this reset to zero, so in
    // a root-less cycle every tile's age climbs without bound; refusing
    // anything past the limit starves the cycle out and re-runs the election.
    // Without this the hop count just counts to 255, wraps, and the dead root
    // looks adjacent again — forever.
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
    // Place myself from the parent's beacon. Parent transmitted on its side
    // pn.info.side; I received on side `parent`. Mated sides face opposite
    // global directions.
    uint8_t parentDir = mod6((int8_t)(pn.info.rot + pn.info.side));
    newRot = mod6((int8_t)(pn.info.rot + pn.info.side + 3 - parent));
    newQ = (int8_t)(pn.info.q + DQ[parentDir]);
    newR = (int8_t)(pn.info.r + DR[parentDir]);
    rootAgeBaseMs = (uint16_t)pn.info.age * ROOT_AGE_UNIT_MS + AIR_DELAY_MS;
    rootAgeStamp  = pn.lastHeard;
  }

  bool moved = (newRot != myRot) || (newQ != myQ) || (newR != myR);
  myRoot = bestRoot; myHop = bestHop;
  myRot = newRot; myQ = newQ; myR = newR;
  if (moved) rebuild_led_cache();

  // ---- animation clock + selection ----
  if (parent < 0) {
    // Root free-runs its clock and picks the animation. Advancing a counter on
    // elapsed time rather than dividing the clock keeps the dwell equal for
    // every animation; the old modulo of a 16-bit clock jumped mid-sequence
    // every time that clock wrapped, since 65536 is not a multiple of the
    // period.
    if ((uint16_t)(now - animChangedAt) >= ANIM_PERIOD_MS) {
      animChangedAt = now;
      if (++myAnim >= NUM_ANIMS) myAnim = 0;
    }
  } else {
    myAnim = pn.info.animID;
    animChangedAt = now;    // so a later promotion to root starts a full period

    // Only ever act on a beacon once. Two reasons: this runs at 20 Hz but
    // beacons arrive at ~2.5 Hz, so re-slewing on a stale measurement would
    // apply one correction eight times over; and (now - lastHeard) is measured
    // in millis(), which undercounts real time by the LED blackout duty cycle,
    // so a stale measurement carries a bias proportional to its own age.
    // Acting only on a just-arrived packet keeps that term near zero.
    if (parent != lastSyncSide || pn.lastHeard != lastSyncHeard) {
      lastSyncSide  = parent;
      lastSyncHeard = pn.lastHeard;

      // Parent's clock at "now" = beacon timestamp + airtime + elapsed since RX.
      uint16_t target = pn.info.t + AIR_DELAY_MS + (uint16_t)(now - pn.lastHeard);
      int16_t err = (int16_t)(target - topo_anim_time());
      if (err > 800 || err < -800)   animOffset += (uint16_t)err;   // jump
      else if (err > SLEW_MS)        animOffset += SLEW_MS;
      else if (err < -(int16_t)SLEW_MS) animOffset -= SLEW_MS;
      else                           animOffset += (uint16_t)err;
    }
  }
}
