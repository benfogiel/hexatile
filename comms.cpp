#include "comms.h"
#include "topology.h"
#include <util/delay.h>
#include <util/atomic.h>

#define BIT_US   (1000000UL / COMMS_BAUD)   // 104 us at 9600
#define PKT_SYNC 0xA5
#define PKT_VER  0x03

enum PacketOffset {
  OFF_SYNC, OFF_VER, OFF_ROOT, OFF_Q, OFF_R, OFF_ROT_SIDE,
  OFF_HOP, OFF_AGE, OFF_T_LO, OFF_T_HI, OFF_CRC,
  PKT_LEN
};
#define CRC_COVERED (OFF_CRC - OFF_VER)

struct SideHW {
  PORT_t* port;
  uint8_t mask;
  volatile uint8_t* pinctrl;
};

// The board labels these sides 1..6; index here is label - 1. Only the cyclic
// order matters for correctness — starting elsewhere in the cycle rotates the
// whole picture uniformly, which is harmless, but reversing it is not.
static const SideHW SIDES[NUM_SIDES] = {
  { &PORTA, 1 << 5, &PORTA.PIN5CTRL },
  { &PORTA, 1 << 4, &PORTA.PIN4CTRL },
  { &PORTA, 1 << 2, &PORTA.PIN2CTRL },
  { &PORTB, 1 << 0, &PORTB.PIN0CTRL },
  { &PORTB, 1 << 1, &PORTB.PIN1CTRL },
  { &PORTA, 1 << 7, &PORTA.PIN7CTRL },
};

struct RxState {
  uint8_t  buf[PKT_LEN];
  uint8_t  idx;
  uint16_t lastByteMs;
};

static volatile RxState  rxs[NUM_SIDES];
static Neighbor          nbr[NUM_SIDES];   // ISR-owned; main loop reads it only
                                           // inside an ATOMIC_BLOCK
static uint16_t          nextBeacon[NUM_SIDES];
static uint8_t           linkGrace[NUM_SIDES];
static uint8_t           gID;
static uint8_t           sideMaskA, sideMaskB;

// ---------------------------------------------------------------------------
// Side-pin interrupt gating. Bit-banging a byte takes ~1.1 ms and a packet
// ~12 ms; blocking *every* interrupt for that long starves millis(), and by an
// amount that scales with link traffic (~40% of wall time on a 6-neighbour
// tile, ~6% on a corner one). That spread is far beyond what the sync loop can
// chase, so mask only the six side pins: no other link can preempt the bits we
// are timing, but the millis timer keeps counting, and its ~2 us ISR is well
// inside the half-bit sampling margin.
static void sides_irq_disable() {
  for (uint8_t s = 0; s < NUM_SIDES; s++) *SIDES[s].pinctrl = PORT_PULLUPEN_bm;
}

static void sides_irq_enable() {
  for (uint8_t s = 0; s < NUM_SIDES; s++)
    *SIDES[s].pinctrl = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
}

static void sides_irq_clear_flags() {
  PORTA.INTFLAGS = sideMaskA;
  PORTB.INTFLAGS = sideMaskB;
}

// ---------------------------------------------------------------------------
static uint8_t crc8(const uint8_t* d, uint8_t n) {   // Dallas/Maxim
  uint8_t crc = 0;
  while (n--) {
    uint8_t b = *d++;
    for (uint8_t i = 0; i < 8; i++) {
      uint8_t mix = (crc ^ b) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      b >>= 1;
    }
  }
  return crc;
}

// ---------------------------------------------------------------------------
// RX path (runs inside the port ISRs)
// ---------------------------------------------------------------------------
static void rx_packet_done(uint8_t s) {
  volatile RxState& r = rxs[s];
  if (r.buf[OFF_SYNC] != PKT_SYNC || r.buf[OFF_VER] != PKT_VER) return;
  if (crc8((const uint8_t*)&r.buf[OFF_VER], CRC_COVERED) != r.buf[OFF_CRC]) return;

  Neighbor& n = nbr[s];
  n.info.rootID = r.buf[OFF_ROOT];
  n.info.q      = (int8_t)r.buf[OFF_Q];
  n.info.r      = (int8_t)r.buf[OFF_R];
  n.info.rot    = (r.buf[OFF_ROT_SIDE] >> 3) & 0x07;
  n.info.side   = r.buf[OFF_ROT_SIDE] & 0x07;
  n.info.hop    = r.buf[OFF_HOP];
  n.info.age    = r.buf[OFF_AGE];
  n.info.t      = (uint16_t)r.buf[OFF_T_LO] | ((uint16_t)r.buf[OFF_T_HI] << 8);
  n.lastHeard   = (uint16_t)millis();
  n.present     = true;
}

// Receive one byte after a falling (start-bit) edge on side s. Takes ~1.1 ms,
// during which only the millis timer may interrupt us.
static void rx_byte(uint8_t s) {
  const SideHW& h = SIDES[s];

  // Clearing every side's flag before sei() matters: we are inside one port's
  // vector, and an edge already latched on the *other* port would otherwise
  // fire its ISR on top of us and recurse into rx_byte.
  sides_irq_disable();
  sides_irq_clear_flags();
  sei();

  _delay_us(BIT_US / 2 - 6);                 // to middle of start bit (~ISR latency)
  bool startOk = !(h.port->IN & h.mask);     // else glitch / stale edge
  uint8_t v = 0;
  bool stopOk = false;

  if (startOk) {
    for (uint8_t i = 0; i < 8; i++) {
      _delay_us(BIT_US - 2);
      if (h.port->IN & h.mask) v |= (1 << i);  // LSB first
    }
    _delay_us(BIT_US - 2);
    stopOk = (h.port->IN & h.mask) != 0;
  }

  // Data-bit edges re-set our INTFLAG during the byte; clear before re-arming
  // so we don't immediately re-enter for a phantom start bit.
  cli();
  sides_irq_clear_flags();
  sides_irq_enable();

  if (!startOk) return;

  volatile RxState& r = rxs[s];
  uint16_t now = (uint16_t)millis();
  if (!stopOk) { r.idx = 0; return; }                   // framing error -> resync
  if ((uint16_t)(now - r.lastByteMs) > 30) r.idx = 0;   // inter-packet gap
  r.lastByteMs = now;

  if (r.idx == 0 && v != PKT_SYNC) return;
  r.buf[r.idx++] = v;
  if (r.idx >= PKT_LEN) {
    rx_packet_done(s);
    r.idx = 0;
  }
}

static void port_isr(PORT_t* port, uint8_t flags) {
  for (uint8_t s = 0; s < NUM_SIDES; s++) {
    if (SIDES[s].port == port && (flags & SIDES[s].mask)) rx_byte(s);
  }
}

ISR(PORTA_PORT_vect) {
  uint8_t f = PORTA.INTFLAGS;
  PORTA.INTFLAGS = f;
  port_isr(&PORTA, f);
}

ISR(PORTB_PORT_vect) {
  uint8_t f = PORTB.INTFLAGS;
  PORTB.INTFLAGS = f;
  port_isr(&PORTB, f);
}

// ---------------------------------------------------------------------------
// TX path
// ---------------------------------------------------------------------------
static void tx_byte(const SideHW& h, uint8_t v) {
  h.port->OUTCLR = h.mask;                   // start bit
  _delay_us(BIT_US - 1);
  for (uint8_t i = 0; i < 8; i++) {
    if (v & 1) h.port->OUTSET = h.mask; else h.port->OUTCLR = h.mask;
    v >>= 1;
    _delay_us(BIT_US - 2);
  }
  h.port->OUTSET = h.mask;                   // stop bit
  _delay_us(BIT_US);
}

static bool tx_packet(uint8_t s, const uint8_t* p) {
  const SideHW& h = SIDES[s];
  if (!(h.port->IN & h.mask)) return false;  // line busy -> collision avoidance

  // Every side stays masked for the whole packet: an RX ISR on another link
  // blocks ~1.1 ms and would shred these bits.
  sides_irq_disable();
  h.port->OUTSET = h.mask;
  h.port->DIRSET = h.mask;                   // drive idle-high
  _delay_us(3 * BIT_US);

  for (uint8_t i = 0; i < PKT_LEN; i++) {
    tx_byte(h, p[i]);
    _delay_us(40);                           // inter-byte gap
  }

  h.port->DIRCLR = h.mask;                   // release line to pull-ups
  sides_irq_clear_flags();                   // drop edges we caused ourselves
  sides_irq_enable();
  return true;
}

static void send_beacon(uint8_t s) {
  NodeInfo me;
  topo_fill_beacon(&me, s);

  uint8_t p[PKT_LEN];
  p[OFF_SYNC]     = PKT_SYNC;
  p[OFF_VER]      = PKT_VER;
  p[OFF_ROOT]     = me.rootID;
  p[OFF_Q]        = (uint8_t)me.q;
  p[OFF_R]        = (uint8_t)me.r;
  p[OFF_ROT_SIDE] = (uint8_t)((me.rot << 3) | (s & 0x07));
  p[OFF_HOP]      = me.hop;
  p[OFF_AGE]      = me.age;
  p[OFF_T_LO]     = (uint8_t)(me.t & 0xFF);
  p[OFF_T_HI]     = (uint8_t)(me.t >> 8);
  p[OFF_CRC]      = crc8(&p[OFF_VER], CRC_COVERED);
  tx_packet(s, p);
}

// ---------------------------------------------------------------------------
void comms_init(uint8_t myID) {
  gID = myID;
  sideMaskA = sideMaskB = 0;
  for (uint8_t s = 0; s < NUM_SIDES; s++) {
    SIDES[s].port->DIRCLR = SIDES[s].mask;
    if (SIDES[s].port == &PORTA) sideMaskA |= SIDES[s].mask;
    else                         sideMaskB |= SIDES[s].mask;
    rxs[s].idx = 0;
    nbr[s].present = false;
    linkGrace[s] = LINK_GRACE_BEACONS;   // beacon fast at power-up so mates find each other
    nextBeacon[s] = (uint16_t)millis() + (uint16_t)s * (BEACON_MS / NUM_SIDES)
                    + (myID & 0x1F);
  }
  sides_irq_enable();
}

void comms_poll() {
  uint16_t now = (uint16_t)millis();
  for (uint8_t s = 0; s < NUM_SIDES; s++) {
    // ATOMIC_BLOCK rather than noInterrupts()/interrupts() so the caller's
    // interrupt state is restored, not blindly enabled; its memory barriers are
    // also what keep the compiler from caching nbr[] across the section.
    bool present;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      if (nbr[s].present && (uint16_t)(now - nbr[s].lastHeard) > NEIGHBOR_TIMEOUT_MS)
        nbr[s].present = false;
      present = nbr[s].present;
    }

    if (present) linkGrace[s] = LINK_GRACE_BEACONS;

    if ((int16_t)(now - nextBeacon[s]) >= 0) {
      send_beacon(s);
      uint16_t interval = IDLE_BEACON_MS;
      if (present) {
        interval = BEACON_MS;
      } else if (linkGrace[s]) {
        interval = BEACON_MS;
        linkGrace[s]--;
      }
      // Jitter keyed on ID and side breaks collision lockstep.
      nextBeacon[s] = now + interval + ((gID * 13u + s * 29u + now) & 0x3F);
    }
  }
}

void comms_get_neighbor(uint8_t side, Neighbor* out) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    *out = nbr[side];
  }
}
