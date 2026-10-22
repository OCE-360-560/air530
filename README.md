# air530: Air530 GPS driver for the Pico SDK

A UART driver for the Air530 GPS receiver on the Seeed Grove GPS board (SKU 109020022),
for the Raspberry Pi Pico SDK, written for OCE360/560. The receiver sends NMEA 0183
sentences once a second. This driver catches every byte in the UART interrupt, so an SD card
write in main can't make it lose data, and parses RMC, GGA, GSA, GSV and TXT when you call
`air530_service()`. Plain C11, no dynamic memory.

**Status (2026-10-22): NOT RUN ON HARDWARE YET.** The boards arrive this week. Compiles with no
warnings (SDK 2.3.0, `pico2`, toolchain 14_2_Rel1) and passes 146 host-test checks against a
fake UART (`sh tests/run_tests.sh`). The Air530 command strings match every example in the Air530
user booklet v1.7. The Air530Z (`$PCAS`) commands are from the CASIC protocol and have not been checked
against a datasheet or a chip. See [Hardware checks](#hardware-checks) below.

## Get it into your project

1. On this page: **Code → Download ZIP**, then unzip it.
2. Rename the folder from `air530-main` to `air530` and move it into your project's `lib/` folder,
   so you have `lib/air530/air530.c`, `lib/air530/air530.h`, and `lib/air530/CMakeLists.txt`.
3. Add the two CMake lines below, build, and commit `lib/air530` with your code, so a fresh clone still builds.

You don't need `example/` or `tests/` in your project.

## Wiring

| Grove GPS | Pico 2 |
|---|---|
| TX | GP5 (uart1 RX) |
| RX | GP4 (uart1 TX), only needed for commands |
| VCC | 3V3(OUT) |
| GND | GND |

Check the labels printed on the Grove board: the cable colors are not a promise.

## Use it

```cmake
add_subdirectory(lib/air530)
target_link_libraries(<your project> pico_stdlib hardware_uart air530)
```

```c
#include "air530.h"

air530_config_t config = {
    .uart = uart1, .tx_pin = 4, .rx_pin = 5,   // tx_pin = AIR530_PIN_NOT_WIRED to only listen
    .baud = AIR530_BAUD_DEFAULT, .chip = AIR530_CHIP_AIR530,
};
air530_init(&config);

while (true) {
    air530_service();                // never waits
    air530_fix_t fix;
    if (air530_get_fix(&fix) && fix.valid) {
        printf("%.6f, %.6f  %u satellites\n", fix.latitude_deg, fix.longitude_deg, fix.satellites_used);
    }
    // other sensors, SD card ...
}
```

`example/` is a complete program: it prints each fix and takes one-key commands from the Serial
Monitor (`?` lists them) so every function can be tried on the bench. Build it with the Pico VS Code
extension (open the `example` folder) or from the command line.

## Functions

| Function | What it does |
|---|---|
| `air530_init(&config)` | sets up the UART and its receive interrupt; false if config makes no sense |
| `air530_service()` | parses every complete sentence received since the last call; call it every pass of your loop |
| `air530_get_fix(&fix)` | copies the latest fix; true if an RMC or GGA arrived since the last call |
| `air530_antenna()` | OK, OPEN or SHORT, from the receiver's `$GPTXT` messages |
| `air530_get_stats(&stats)` | sentences, bad checksums, too-long lines, bytes dropped because the buffer was full |
| `air530_set_echo(on)` | copy every received byte to USB serial too (debugging) |
| `air530_set_sentence_callback(fn)` | call `fn` with every sentence that passes its checksum (raw NMEA logging) |
| `air530_parse(sentence)` | parse one sentence by hand, no UART needed (testing) |
| `air530_send_command(body)` | send `$<body>*hh\r\n` |
| `air530_restart(kind)` | hot, warm, cold or full cold restart; clears the stored fix |
| `air530_set_update_interval_ms(ms)` | time between fixes |
| `air530_set_sentences(mask)` | which sentences the receiver sends |
| `air530_set_constellations(mask)` | GPS, GLONASS, BeiDou, Galileo |
| `air530_set_baud(baud)` | changes the receiver's baud rate, then the Pico's |
| `air530_last_ack(&command, &result)` | the receiver's last `$PGKC001` answer (Air530 only) |

Hover over any of them in VS Code for the full documentation from `air530.h`.

What `air530_fix_t` holds, and where each field comes from:

| Field | From | Notes |
|---|---|---|
| `valid` | RMC status, GGA quality | position fields keep their last values while false |
| `latitude_deg`, `longitude_deg` | RMC, GGA | decimal degrees, `double`; south and west negative |
| `altitude_m`, `geoid_separation_m` | GGA | |
| `speed_knots`, `course_deg` | RMC | course keeps its last value when the field is empty (standing still) |
| `fix_quality`, `satellites_used` | GGA | |
| `fix_type`, `pdop`, `hdop`, `vdop` | GSA (`hdop` also GGA) | |
| `satellites_in_view` | GSV | summed over GPS, GLONASS, BeiDou, Galileo, QZSS |
| `hour` ... `millisecond`, `year`, `month`, `day` | RMC only | so time and date always belong to the same second |
| `updated_us` | Pico clock | `time_us_64()` when `air530_service()` parsed the last RMC or GGA |

## Design notes

- **Interrupt + ring buffer.** The UART FIFO holds 32 bytes, 33 ms at 9600 baud; an SD card sync
  can block longer than that. The interrupt only copies bytes into a 1024-byte ring (about 1 s);
  all parsing happens in `air530_service()`, outside the interrupt. If main stalls longer than about
  a second, `bytes_dropped` counts it.
- **Any talker.** A multi-constellation receiver reports its combined fix as `$GNRMC`, not `$GPRMC`;
  the parser accepts any two-letter talker.
- **Empty fields kept.** Fields are split by hand, not with `strtok`, which skips empty fields and
  shifts everything after them in a no-fix sentence.
- **Checksums required.** A sentence without exactly two hex digits after `*` is thrown away.

## Hardware checks

To do when the boards arrive, in order. Record results here.

1. Read the marking on the module: **Air530 or Air530Z?** Set `.chip` to match. If Air530Z,
   every `$PCAS` command below needs checking.
2. Run `example/` indoors with `e` (echo): copy 10 lines. Talker IDs (`$GNRMC` or `$GPRMC`)?
   Any `$GPTXT` at power-up (antenna, firmware)? Sentence order within a second?
3. Without echo: does each second print twice (GGA and RMC both count as new)? If that's
   awkward for a 1 Hz logger, change the "new fix" rule to RMC only.
4. `v`: does `$PGKC463,<version>` come back? `s`: does `last ack` show up after `1` (`$PGKC001,101,3`)?
5. Outdoors: time to first fix from power-up; then `c` (cold) and `h` (hot), time each.
   Does a sentence that was already on its way still say "fix" after the restart?
6. `b` then `5` (38400 baud, 5 fixes a second): do fixes come 200 ms apart with no bad checksums?
   Power-cycle: does the receiver come back at 38400 or 9600? (The README promises nothing yet.)
7. Log through an SD card program for 5 minutes: `bytes_dropped` and `bad_checksums` should stay 0.
8. GP4 idle voltage with the GPS connected: the Pico drives 3.3 V into a chip RX rated 2.8 V.
   Is there level shifting on the Grove board (schematic says?)? If not, decide whether that's OK.

## Credits

The NMEA parser started from the A7_GPS_UART example in OCE360_560_codebase (Mingxi Zhou),
originally by Yousef Yasser, Rasheed Atia, and Seifeldin Khaled (2025, MIT).
Air530 commands: Air530 user booklet v1.7 (Shanghai Luat), section 7.

MIT License, see `LICENSE`.
