/*
 * air530_example.c
 * Prints the GPS fix once a second, and takes one-key commands from the
 * Serial Monitor so every driver function can be tried on the bench.
 *
 * Wiring (Grove cable to jumpers; check the labels printed on the board):
 *   GPS TX  -> GP5 (uart1 RX)
 *   GPS RX  -> GP4 (uart1 TX)   only needed for commands
 *   GPS VCC -> 3V3(OUT), GPS GND -> GND
 *
 * Author:  OCE360/560 instructors
 * License: MIT (see LICENSE)
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "air530.h"

static void print_help(void) {
    printf("keys: e echo raw NMEA on/off, h hot / w warm / c cold restart,\n"
           "      1 one fix a second, 5 five a second (needs 38400 baud: b first),\n"
           "      b 38400 baud, n 9600 baud, v firmware version, s stats, ? help\n");
}

static void print_fix(const air530_fix_t *fix) {
    if (fix->time_valid) {
        printf("%02u:%02u:%02u.%03u ", fix->hour, fix->minute, fix->second, fix->millisecond);
    } else {
        printf("--:--:--.--- ");
    }
    if (fix->date_valid) {
        printf("%04u-%02u-%02u ", fix->year, fix->month, fix->day);
    }
    if (fix->valid) {
        printf("%.6f, %.6f  alt %.1f m  %u used / %u in view  hdop %.2f  %.2f kn %.1f deg\n",
               fix->latitude_deg, fix->longitude_deg, fix->altitude_m, fix->satellites_used,
               fix->satellites_in_view, fix->hdop, fix->speed_knots, fix->course_deg);
    } else {
        printf("no fix  %u in view\n", fix->satellites_in_view);
    }
}

static void handle_key(int key, bool *echo_on) {
    bool sent = true;
    switch (key) {
    case 'e': *echo_on = !*echo_on; air530_set_echo(*echo_on); break;
    case 'h': sent = air530_restart(AIR530_RESTART_HOT); break;
    case 'w': sent = air530_restart(AIR530_RESTART_WARM); break;
    case 'c': sent = air530_restart(AIR530_RESTART_COLD); break;
    case '1': sent = air530_set_update_interval_ms(1000); break;
    case '5': sent = air530_set_update_interval_ms(200); break;
    case 'b': sent = air530_set_baud(38400); break;
    case 'n': sent = air530_set_baud(9600); break;
    case 'v': sent = air530_send_command("PGKC462"); air530_set_echo(true); *echo_on = true; break;
    case 's': {
        air530_stats_t stats;
        air530_get_stats(&stats);
        printf("sentences %lu  bad checksums %lu  too long %lu  bytes dropped %lu  antenna %d\n",
               (unsigned long)stats.sentences, (unsigned long)stats.bad_checksums,
               (unsigned long)stats.too_long, (unsigned long)stats.bytes_dropped, air530_antenna());
        int command, result;
        if (air530_last_ack(&command, &result)) {
            printf("last ack: command %d result %d (3 = done)\n", command, result);
        }
        break;
    }
    case '?': print_help(); break;
    default: return;
    }
    if (!sent) {
        printf("command not sent (TX not wired, or not supported by this chip)\n");
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);   // time to open the Serial Monitor

    air530_config_t config = {
        .uart = uart1, .tx_pin = 4, .rx_pin = 5,
        .baud = AIR530_BAUD_DEFAULT, .chip = AIR530_CHIP_AIR530,
    };
    if (!air530_init(&config)) {
        printf("air530_init failed\n");
    }
    printf("air530 example\n");
    print_help();

    bool echo_on = false;
    while (true) {
        air530_service();

        air530_fix_t fix;
        if (air530_get_fix(&fix) && !echo_on) {
            print_fix(&fix);
        }

        int key = getchar_timeout_us(0);
        if (key != PICO_ERROR_TIMEOUT) {
            handle_key(key, &echo_on);
        }
    }
}
