/*
 * air530.h
 * UART driver for the Air530 GPS receiver (Seeed Grove GPS, SKU 109020022) on
 * the Raspberry Pi Pico SDK. The receiver sends NMEA 0183 sentences once a
 * second; this driver catches every byte in an interrupt, parses RMC, GGA, GSA,
 * GSV and TXT in air530_service(), and keeps the latest fix. It never waits for
 * the receiver, so it can share a loop with other sensors and an SD card.
 *
 * Author:  OCE360/560 instructors
 * License: MIT (see LICENSE)
 */
#ifndef AIR530_H
#define AIR530_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/uart.h"

#define AIR530_BAUD_DEFAULT 9600
#define AIR530_PIN_NOT_WIRED 0xFF   // tx_pin: listen only, no commands

/** Which command set the receiver speaks. Read the marking on the module. */
typedef enum {
    AIR530_CHIP_AIR530,    // $PGKC commands (Air530 user booklet v1.7)
    AIR530_CHIP_AIR530Z,   // $PCAS commands (CASIC protocol); NOT yet checked on hardware
} air530_chip_t;

typedef struct {
    uart_inst_t *uart;     // uart0 or uart1
    uint8_t tx_pin;        // Pico TX -> GPS RX, or AIR530_PIN_NOT_WIRED
    uint8_t rx_pin;        // Pico RX <- GPS TX
    uint32_t baud;         // AIR530_BAUD_DEFAULT unless you changed the receiver's rate
    air530_chip_t chip;
} air530_config_t;

/**
 * The receiver's latest report. Position fields keep their last good values
 * while valid is false, so check valid before using them.
 */
typedef struct {
    bool valid;                // RMC status A, or GGA quality above 0
    double latitude_deg;       // decimal degrees, north positive
    double longitude_deg;      // decimal degrees, east positive (Rhode Island is negative)
    float altitude_m;          // GGA: above mean sea level
    float geoid_separation_m;  // GGA: geoid above the WGS84 ellipsoid
    float speed_knots;         // RMC: speed over ground
    float course_deg;          // RMC: course over ground, degrees true; keeps its last value if empty
    uint8_t fix_quality;       // GGA: 0 none, 1 GPS, 2 differential, 6 dead reckoning
    uint8_t fix_type;          // GSA: 1 none, 2 2D, 3 3D
    uint8_t satellites_used;   // GGA
    uint8_t satellites_in_view;// GSV, summed over all constellations
    float pdop, hdop, vdop;    // GSA (hdop also from GGA); lower is better

    bool time_valid;           // RMC carried a time; the receiver can know the time before it has a fix
    uint8_t hour, minute, second;
    uint16_t millisecond;
    bool date_valid;
    uint16_t year;             // 4 digits
    uint8_t month, day;

    uint64_t updated_us;       // time_us_64() when air530_service() parsed the last RMC or GGA
} air530_fix_t;

typedef enum {
    AIR530_ANTENNA_UNKNOWN,    // no $GPTXT antenna message seen yet
    AIR530_ANTENNA_OK,
    AIR530_ANTENNA_OPEN,
    AIR530_ANTENNA_SHORT,
} air530_antenna_t;

typedef struct {
    uint32_t sentences;        // good checksum
    uint32_t bad_checksums;    // thrown away
    uint32_t too_long;         // longer than 120 characters: thrown away
    uint32_t bytes_dropped;    // the receive buffer was full: air530_service() not called often enough
} air530_stats_t;

typedef enum {
    AIR530_RESTART_HOT = 1,         // keeps time, position, ephemeris: fix in seconds
    AIR530_RESTART_WARM = 2,        // keeps time and position, forgets ephemeris
    AIR530_RESTART_COLD = 3,        // forgets everything it learned from the sky
    AIR530_RESTART_FULL_COLD = 4,   // cold, and resets settings
} air530_restart_t;

// Sentence masks for air530_set_sentences()
#define AIR530_SENTENCE_GGA (1u << 0)
#define AIR530_SENTENCE_GLL (1u << 1)
#define AIR530_SENTENCE_GSA (1u << 2)
#define AIR530_SENTENCE_GSV (1u << 3)
#define AIR530_SENTENCE_RMC (1u << 4)
#define AIR530_SENTENCE_VTG (1u << 5)
#define AIR530_SENTENCE_ZDA (1u << 6)   // Air530Z only

// Constellation masks for air530_set_constellations()
#define AIR530_GNSS_GPS (1u << 0)
#define AIR530_GNSS_GLONASS (1u << 1)
#define AIR530_GNSS_BEIDOU (1u << 2)
#define AIR530_GNSS_GALILEO (1u << 3)   // Air530 only

/**
 * Set up the UART and its receive interrupt. Call once, after stdio_init_all().
 * Returns false if config makes no sense (no UART, no RX pin, baud 0).
 * The receiver needs no setup: it starts talking at power-up.
 */
bool air530_init(const air530_config_t *config);

/**
 * Parse every complete sentence received since the last call. Returns right
 * away; call it every pass of your main loop (at least a few times a second).
 * Returns the number of sentences that updated the fix (RMC or GGA).
 */
int air530_service(void);

/**
 * Copy the latest fix into fix. Returns true if an RMC or GGA arrived since the
 * last call (about twice a second at the default rate: one of each).
 */
bool air530_get_fix(air530_fix_t *fix);

/** Latest antenna report from the receiver's $GPTXT messages. */
air530_antenna_t air530_antenna(void);

/** Counters since air530_init(). */
void air530_get_stats(air530_stats_t *stats);

/** Copy every byte received to USB serial as well (true), or stop (false). For debugging. */
void air530_set_echo(bool echo_on);

/**
 * Call fn with every sentence that passes its checksum, before it is parsed:
 * for logging raw NMEA, or for sentences this driver doesn't read. NULL turns it off.
 *   sentence: "$GNRMC,...*2E", no "\r\n"; valid only during the call
 */
void air530_set_sentence_callback(void (*fn)(const char *sentence));

/**
 * Check and parse one sentence without the UART: the same parser air530_service() uses.
 * For testing with a known sentence and no sky.
 *   sentence: e.g. "$GNRMC,...*2E", no "\r\n"
 * Returns true if it updated the fix (RMC or GGA).
 */
bool air530_parse(const char *sentence);

// ---- Commands (need tx_pin wired) ----------------------------------------
// Each returns false if tx_pin is not wired or an argument is out of range.
// The receiver answers some Air530 commands with $PGKC001; see air530_last_ack().

/** Send "$<body>*hh\r\n". body is the text between '$' and '*', e.g. "PGKC030,3,1". */
bool air530_send_command(const char *body);

/** Restart the receiver. Clears the stored fix (valid false) so a stale fix can't count as the new one. */
bool air530_restart(air530_restart_t kind);

/** Time between fixes: Air530 200 to 10000 ms; Air530Z 100, 200, 250, 500 or 1000 ms. */
bool air530_set_update_interval_ms(uint32_t interval_ms);

/** Turn sentences on (1 per fix) or off. mask: AIR530_SENTENCE_* ORed together. Keep RMC and GGA on. */
bool air530_set_sentences(uint32_t mask);

/** Satellite systems to search. mask: AIR530_GNSS_* ORed together. */
bool air530_set_constellations(uint32_t mask);

/**
 * Change the baud rate of the receiver, then of the Pico to match.
 * Faster rates are needed for update intervals below about 500 ms at 9600 baud.
 * Whether the receiver keeps the new rate after power-off is not yet checked.
 */
bool air530_set_baud(uint32_t baud);

/**
 * The last $PGKC001 answer (Air530 only).
 *   command: the command it answers, e.g. 101
 *   result: 1 not supported, 2 valid but failed, 3 done
 * Returns false if none received yet.
 */
bool air530_last_ack(int *command, int *result);

#endif
