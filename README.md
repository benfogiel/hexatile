# Hexatile firmware

Identical firmware on every tile. Tiles self-organize into a hex-grid map and
render one shared animation seamlessly across the whole assembly.

## The core idea

No pixel data ever crosses a tile boundary. Every animation is a pure function
`color = f(global_x_mm, global_y_mm, t)`. Once a tile knows (a) where it sits
on the grid, (b) how it is rotated, and (c) a network-synced clock, it renders
its own 91 LEDs locally. The single-wire links only carry tiny, infrequent
"beacon" packets, so slow comms are fine.

## Modules

| File | Role |
|---|---|
| `hexatile.ino` | init + main loop (comms → topology → render → show) |
| `config.h` | everything you must verify/tune |
| `led_layout.h` | generated per-LED (x, y) in integer mm, from your polar table |
| `comms.cpp/.h` | bit-banged half-duplex 9600-baud UART on the 6 side pins, beaconing, neighbor table |
| `topology.cpp/.h` | root election, coordinate + rotation derivation, clock sync, rotated-LED cache |
| `anim.cpp/.h` | fixed-point effects (rainbow rings, sweep, plasma), bring-up patterns, HSV→RGB, brightness cap |
| `sim/` | host-side regression test for the topology layer (see below) |

## Protocol (12-byte beacon, ~13 ms airtime)

`[A5][ver][rootID][q][r][rot<<3|txSide][hop][age][t_lo][t_hi][animID][crc8]`

* A side with a neighbor beacons every ~400 ms (jittered); an open side backs
  off to `IDLE_BEACON_MS`. Transmitting masks *every* side's interrupt for the
  whole 13 ms packet, so a beacon into an open edge is 13 ms of deafness on the
  edges that are mated, bought for nothing. On two tiles that was five sixths of
  all self-inflicted deafness, landing on the only link in the assembly: 32% of
  it lost, enough to time out a live neighbor and send the follower back to
  believing it is a root. Open sides only need to notice a tile being attached.
* Lines idle high on internal pull-ups; a falling edge triggers a per-byte RX
  interrupt (~1.1 ms each).
* Collisions/corruption are expected and cheap: CRC drops the packet, the
  next beacon retries. `strip.show()` also masks interrupts for ~2.7 ms;
  bytes lost there are recovered the same way.
* **Root election:** every tile has a stable 8-bit ID hashed from the chip's
  factory serial. Lowest reachable ID wins; ties broken by hop count, then by
  age.
* **Aging is what makes the tree loop-safe.** `age` is how long ago the root
  generated the information being relayed — the root says 0, everyone else
  adds the delay it took to reach them. Only a path that really reaches the
  root resets it. Pull the root out of a ring and nothing resets anything, so
  age climbs past `ROOT_MAX_AGE_MS`, every tile refuses the stale root, and the
  election re-runs (~6 s). Hop count alone cannot do this: it just counts up to
  255, wraps, and the dead root looks adjacent again. The cost is a diameter
  limit of roughly `ROOT_MAX_AGE_MS / BEACON_MS` ≈ 12 hops.
* **Placement:** if a neighbor's beacon says it is at `(q,r)` with rotation
  `rot` and was sent from its side `s_p`, and I heard it on my side `s_c`:
  * `my_rot = (rot + s_p + 3 − s_c) mod 6`
  * `my_pos = (q,r) + step[(rot + s_p) mod 6]`
  One packet fully places a tile. Move a tile → old side times out (1.5 s),
  new side hears a beacon, map heals, animation re-flows.
* **Clock:** children slew toward `parent_t + airtime + elapsed`, at most 8 ms
  per *beacon* (not per update — folding the same measurement in at 20 Hz
  would both over-correct and bake in the staleness bias of an old packet).
  Large errors jump instead. Root free-runs and advances the animation index on
  a counter every `ANIM_PERIOD_MS`.
* **Interrupt discipline:** bit-banging blocks for ~1.1 ms a byte and ~13 ms a
  packet. Doing that with `cli()` starves `millis()` in proportion to link
  traffic — a 6-neighbour tile loses ~40% of wall time, a corner tile ~6% —
  which is a rate spread no slew can chase. So comms masks *only the six side
  pins* and leaves the timer running. `strip.show()` genuinely cannot be
  interrupted, so the loop hands that known blackout back to the animation
  clock via `topo_clock_compensate_us()`.

## Memory budget (SRAM, ~1024 B total)

* Measured: 4620 B flash (28%), 695 B SRAM, 329 B left for stack. LED buffer
  273 B, rotated-coord cache 182 B, RX buffers + neighbor table ~170 B.
  FastLED does not fit; megaTinyCore's `tinyNeoPixel_Static` (no malloc, you
  own the buffer) does.
* Interrupts now nest two deep at most (a side ISR plus the millis timer), so
  budget stack accordingly if you add anything.

## Build

* megaTinyCore, board **ATtiny1604/1614/1616/1617/…**, then **Tools → Chip →
  ATtiny1604**. The chip submenu is separate from the board entry and defaults
  elsewhere; picking a 512 B part (204/404/804) fails at link with
  ``address 0x… of … section `.bss' is not within region `data'``. This
  firmware needs the full 1 KB — the LED buffer and coordinate cache are 455 B
  before anything else.
* Clock **20 MHz internal** (bit-bang timing uses `_delay_us`, which follows
  `F_CPU`; 16 MHz also works).
* millis: leave enabled (default timer).
* **Do not call `attachInterrupt()` anywhere** — `comms.cpp` owns the
  `PORTA`/`PORTB` interrupt vectors directly.

## Testing the topology without hardware

```
cd sim && make run
```

`sim/` compiles the real `topology.cpp` once per tile, each inside its own
namespace so every tile gets a private copy of the module's file-scope state.
The test therefore exercises the shipping algorithm, not a paraphrase of it.
Around it sits a model of the things the firmware cannot see: seven tiles in a
flower (a centre plus a full ring, dense with 3-cycles), each with its own
`millis()` origin and ±0.6% rate error, beacons delivered to whichever tile is
physically mated to the sending side after `AIR_DELAY_MS`, and optional packet
loss.

A run is scored as converged only if all tiles agree on one root **and** every
tile's derived `(q, r, rot)` is the true physical layout put through a single
shared rotation + translation. Absolute coordinates are arbitrary — the elected
root defines the frame — so only their mutual consistency is meaningful.

It then checks the frame the renderer will actually draw in: every tile's LED
coordinates, tile offset included, must be the physical LED layout put through
that same one transform, to within 2.5 mm of integer rounding. Agreeing on
`(q, r, rot)` is not sufficient — a tile can hold an entirely correct coordinate
and still render its own contents rotated against its physical sides, which
tears every seam while each tile looks individually fine. Against the code that
ignored `LED_THETA0_HALF_STEPS`, this check fails at 23.8 mm.

The wire model includes self-deafness: a tile that is transmitting on any side
receives nothing on any side for the duration, so beacon scheduling shows up in
delivery rates. A two-tile scenario reports what fraction of the single link
survives, and fails if the follower ever falls back to believing it is a root —
which at `IDLE_BEACON_MS 400` / `NEIGHBOR_TIMEOUT_MS 1500` it did for 2150 ms
out of every 60 s, matching what the hardware showed.

Covered: cold start, root unplugged mid-ring, a tile moved to a new cell and
rotation, 20% packet loss, and a two-tile pair with ±1% oscillators. The root-removal case is the one that motivated
the age byte in the beacon: against the pre-aging code it never recovers at
all, and the test does fail there, so it is not scoring vacuously.

What this does **not** cover: anything below `topo_update()`. Bit timing, the
interrupt masking in `comms.cpp`, WS2812 output and real RF/connector behaviour
all still need a scope and real tiles.

## Bring-up patterns (`DEBUG_PATTERN` in config.h)

Set `DEBUG_PATTERN` to one of these, reflash every tile, and the normal
animation cycle is replaced by a diagnostic (`animID` is ignored; the unused
effects and accessors compile out). A single pattern cannot tell you whether a
fault is in the clock or in the map, so the first two hold one of those constant
and leave the other as the only variable. Work down the list — each one assumes
the ones above it pass.

| Mode | Shows | Reading it |
|---|---|---|
| `DBG_SYNC` | clock only, no position input | Every tile ramps white over 1.024 s and drops. All tiles should ramp as one surface. A tile at a different brightness is off by that fraction of a second; one that drifts and snaps back is slewing, and is not hearing beacons often enough. |
| `DBG_GEOMETRY` | map only, frozen in time | A still rainbow centred on the root with a black contour ring every 128 mm. Contours must cross tile seams unbroken. A rotated-looking tile has the wrong `rot`; one whose rings are centred on itself never got placed; a discontinuity that jumps by a whole tile is a wrong `TILE_PITCH_MM`. |
| `DBG_TOPOLOGY` | what each tile believes, no animation at all | Centre white = root, dim red = follower. Ring 1: one blue LED per hop. Ring 2: a cyan spinner stepping once per beacon folded into the clock (~2.5 steps/s per parent) — frozen under a green arc means packets arrive but never reach the clock. Ring 3: an orange marker pointing where the tile thinks global +x is; all markers must point the same physical direction. Ring 4: clock error against the parent, one LED per 8 ms, clockwise = behind, green and short = locked. Outer ring: a green arc per side with a live neighbor, white on the side the coordinate came from. Following the white arcs walks you back up the tree. |
| `DBG_RINGS` | end-to-end | 32 mm cyan bands expanding from the root at 125 mm/s. Hard edges rather than a gradient, because a rainbow can look continuous across a bad seam and a band edge cannot. |

`DBG_TOPOLOGY` is the one that separates a comms fault from a placement fault:
it reads the neighbor table directly, so a side that is mated but shows no arc
is a link problem, not a topology problem.

## Things you must verify on real hardware (config.h)

1. `TILE_PITCH_MM` — centre-to-centre distance of two mated tiles (2 ×
   hexagon apothem). Cross-tile coherence depends on this being right. Odd
   values are fine, but keep it an integer: a decimal here pulls the AVR float
   library into the build for ~610 bytes.
   Sanity-check it against `led_layout.h`: the outer LED ring sits at r ≈ 43 mm,
   so the apothem cannot be smaller than that. The current 85 implies an apothem
   of 42.5 and would put the outermost LEDs off the edge of the board, so it is
   at best 1–2 mm short and wants measuring.
2. `LED_THETA0_HALF_STEPS` — whether the LED table's theta = 0 spoke runs
   through the middle of side 0 (`0`) or points at a corner (`±1`). The rotation
   cache only turns LEDs in 60° steps, so a half-side error here cannot be
   absorbed anywhere downstream: tile centres land correctly while each tile's
   *contents* sit 30° rotated, which is ~24 mm of misplacement at the rim.
   `DBG_TOPOLOGY` shows it directly — the side arcs must be centred on the
   physical sides, not straddling the corners.
3. Side numbering: `SIDES[]` in comms.cpp maps side index 0..5 → PA5, PA4,
   PA2, PB0, PB1, PA7, i.e. the board's silkscreen labels 1..6. The firmware
   assumes sides run **counter-clockwise viewed from the LED face**; if your
   numbering is clockwise, set `SIDE_CCW 0`. Which side is index 0 only rotates
   the whole picture uniformly — harmless — but CCW vs CW does not.
   **Check which face you counted from**: reading the silkscreen from the back
   of the PCB reverses the handedness, and that is the single easiest way to
   get a plausible-looking build that places every neighbour in the wrong cell.
   `DBG_GEOMETRY` catches it — with the handedness wrong, tiles land in
   mirrored cells and the contour rings break at every seam even though each
   tile individually looks fine.
4. `BRIGHTNESS` vs. connector current: at 64/255 a full-white tile is
   roughly 1.3 A.
5. RX bit timing: scope one side; if sampling drifts, trim the `- 2` fudge
   constants in `rx_byte`/`tx_byte` or drop `COMMS_BAUD` to 4800. Note that
   bits are now timed with the millis interrupt live, which adds a couple of
   microseconds of jitter per bit against a 52 µs half-bit margin — 4800 baud
   doubles that margin if the links turn out to be marginal.
6. `ROOT_MAX_AGE_MS` — raise it if you build an assembly more than ~12 tiles
   across, lower it for faster recovery when the root is unplugged.

## Known limitations / next steps

* Coordinate conflicts (two tiles told the same cell via different parents in
  a ring) are simply last-writer-wins; fine for small assemblies.
* Root recovery costs ~6 s of scrambled animation while the age limit expires.
  Faster would need an explicit "root gone" flood rather than passive aging.
* Byte-level RX means heavy traffic steals CPU; at 6 neighbors × 2.5
  beacons/s it is a few percent.
* Two tiles hashing to the same ID would confuse election (1-in-256 per
  pair); add a 16-bit ID + wider packet if you build many tiles.
* Natural extensions: propagate a "user event" (button/tap) through beacons,
  animation parameters chosen at the root, per-tile power reporting.
