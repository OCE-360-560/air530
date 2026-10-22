/*
 * test_air530.c
 * Host tests for air530.c: real-format sentences (no-fix sentences with empty
 * fields, several talkers, bad checksums) through air530_parse() and through
 * the fake UART and its interrupt; commands checked against the examples in
 * the Air530 user booklet v1.7.
 *
 * Author:  OCE360/560 instructors
 * License: MIT
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "air530.h"
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "fake_sdk.h"

static int checks, failures;

#define CHECK(condition) do { checks++; if (!(condition)) { failures++; \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); } } while (0)
#define CHECK_NEAR(actual, expected, tolerance) do { checks++; \
    double actual_value = (actual), expected_value = (expected); \
    if (fabs(actual_value - expected_value) > (tolerance)) { failures++; \
    printf("FAIL %s:%d: %s = %.9f, expected %.9f\n", __FILE__, __LINE__, #actual, actual_value, expected_value); } \
    } while (0)
#define CHECK_SENT(expected) do { checks++; if (strcmp(fake_uart_sent(), expected) != 0) { failures++; \
    printf("FAIL %s:%d: sent \"%s\", expected \"%s\"\n", __FILE__, __LINE__, fake_uart_sent(), expected); } \
    fake_uart_clear_sent(); } while (0)

// No-fix sentences reported for the Grove Air530 (checksums verified).
#define NO_FIX_GGA "$GNGGA,,,,,,0,00,25.5,,,,,,*64"
#define NO_FIX_RMC "$GNRMC,,V,,,,,,,,,,N,V*37"
// Made-up fixes on the Kingston campus, real format, correct checksums.
#define FIX_RMC "$GNRMC,143012.000,A,4129.1234,N,07131.5678,W,0.12,87.30,291026,,,A,V*2E"
#define FIX_GGA "$GNGGA,143013.000,4129.1240,N,07131.5690,W,1,09,0.85,18.1,M,-34.2,M,,*77"
#define STILL_RMC_EMPTY_COURSE "$GNRMC,143014.000,A,4129.1241,N,07131.5691,W,0.00,,291026,,,A,V*0C"
#define SOUTH_EAST_RMC "$GPRMC,021500.000,A,3345.1234,S,15112.5678,E,1.50,270.00,150326,,,A*78"
#define BEIDOU_RMC_V40 "$BDRMC,143015.000,A,4129.1242,N,07131.5692,W,0.05,12.50,291026,,,A*55"
#define LOST_FIX_GGA "$GNGGA,143016.000,,,,,0,03,9.99,,,,,,*73"
#define MODE_N_RMC "$GNRMC,143012.000,A,4129.1234,N,07131.5678,W,0.12,87.30,291026,,,N,V*21"
#define HALF_SECOND_RMC "$GNRMC,235959.500,A,4129.1234,N,07131.5678,W,0.12,87.30,311226,,,N,V*2B"
#define ANTENNA_OK "$GPTXT,01,01,01,ANTENNA OK*35"
#define ANTENNA_OPEN "$GPTXT,01,01,01,ANTENNA OPEN*25"
#define GSA_SENTENCE "$GNGSA,A,3,10,12,15,24,,,,,,,,,1.21,0.85,0.86,1*00"
#define GPS_GSV "$GPGSV,3,1,11,10,63,137,17,07,61,098,15,05,59,290,20,08,54,157,30,0*6C"
#define BEIDOU_GSV "$BDGSV,1,1,05,10,63,137,17,07,61,098,15,0*73"
#define ACK_101_DONE "$PGKC001,101,3*2D"

static const double FIX_LATITUDE = 41.0 + 29.1234 / 60.0;     // 41.485390
static const double FIX_LONGITUDE = -(71.0 + 31.5678 / 60.0);  // -71.526130

static void start(air530_chip_t chip, uint8_t tx_pin) {
    fake_sdk_reset();
    air530_config_t config = {
        .uart = uart1, .tx_pin = tx_pin, .rx_pin = 5, .baud = AIR530_BAUD_DEFAULT, .chip = chip,
    };
    air530_init(&config);
    air530_set_sentence_callback(NULL);
}

static air530_fix_t current_fix(void) {
    air530_fix_t fix;
    air530_get_fix(&fix);
    return fix;
}

static void test_init(void) {
    fake_sdk_reset();
    air530_config_t bad = {.uart = NULL, .rx_pin = 5, .baud = 9600};
    CHECK(!air530_init(&bad));
    CHECK(!air530_init(NULL));

    start(AIR530_CHIP_AIR530, 4);
    CHECK(fake_uart_baud() == 9600);
    CHECK(fake_pin_function(4) == GPIO_FUNC_UART);
    CHECK(fake_pin_function(5) == GPIO_FUNC_UART);
    CHECK(fake_irq_ready(UART1_IRQ));

    start(AIR530_CHIP_AIR530, AIR530_PIN_NOT_WIRED);
    CHECK(fake_pin_function(4) == -1);   // TX left alone
    CHECK(!air530_send_command("PGKC462"));
    CHECK(!air530_restart(AIR530_RESTART_COLD));
    CHECK(strcmp(fake_uart_sent(), "") == 0);

    // Calling init again (the real irq_set_exclusive_handler panics if a handler is already set)
    air530_config_t again = {.uart = uart1, .tx_pin = 4, .rx_pin = 5, .baud = 9600};
    CHECK(air530_init(&again));
    CHECK(fake_irq_ready(UART1_IRQ));
}

static void test_no_fix(void) {
    start(AIR530_CHIP_AIR530, 4);
    CHECK(air530_parse(NO_FIX_RMC));
    air530_fix_t fix = current_fix();
    CHECK(!fix.valid);
    CHECK(!fix.time_valid);
    CHECK(!fix.date_valid);
    CHECK(air530_parse(NO_FIX_GGA));
    fix = current_fix();
    CHECK(!fix.valid);
    CHECK(fix.fix_quality == 0);
    CHECK(fix.satellites_used == 0);
    CHECK_NEAR(fix.hdop, 25.5, 1e-4);
}

static void test_fix(void) {
    start(AIR530_CHIP_AIR530, 4);
    CHECK(air530_parse(FIX_RMC));
    air530_fix_t fix = current_fix();
    CHECK(fix.valid);
    CHECK_NEAR(fix.latitude_deg, FIX_LATITUDE, 1e-9);
    CHECK_NEAR(fix.longitude_deg, FIX_LONGITUDE, 1e-9);
    CHECK_NEAR(fix.speed_knots, 0.12, 1e-4);
    CHECK_NEAR(fix.course_deg, 87.3, 1e-4);
    CHECK(fix.time_valid && fix.hour == 14 && fix.minute == 30 && fix.second == 12 && fix.millisecond == 0);
    CHECK(fix.date_valid && fix.year == 2026 && fix.month == 10 && fix.day == 29);
    CHECK(fix.updated_us > 0);

    CHECK(air530_parse(FIX_GGA));
    fix = current_fix();
    CHECK(fix.valid);
    CHECK(fix.fix_quality == 1);
    CHECK(fix.satellites_used == 9);
    CHECK_NEAR(fix.hdop, 0.85, 1e-4);
    CHECK_NEAR(fix.altitude_m, 18.1, 1e-4);
    CHECK_NEAR(fix.geoid_separation_m, -34.2, 1e-4);
    CHECK_NEAR(fix.latitude_deg, 41.0 + 29.1240 / 60.0, 1e-9);
    CHECK(fix.second == 12);   // time and date come only from RMC

    CHECK(air530_parse(STILL_RMC_EMPTY_COURSE));
    fix = current_fix();
    CHECK(fix.valid);
    CHECK_NEAR(fix.course_deg, 87.3, 1e-4);   // empty course keeps the last one
    CHECK(fix.second == 14);

    CHECK(air530_parse(LOST_FIX_GGA));
    fix = current_fix();
    CHECK(!fix.valid);
    CHECK(fix.satellites_used == 3);
    CHECK_NEAR(fix.latitude_deg, 41.0 + 29.1241 / 60.0, 1e-9);   // kept from before

    CHECK(air530_parse(MODE_N_RMC));
    CHECK(!current_fix().valid);   // status A but mode N: no fix

    CHECK(air530_parse(HALF_SECOND_RMC));
    fix = current_fix();
    CHECK(fix.hour == 23 && fix.minute == 59 && fix.second == 59 && fix.millisecond == 500);
    CHECK(fix.year == 2026 && fix.month == 12 && fix.day == 31);
}

static void test_other_talkers(void) {
    start(AIR530_CHIP_AIR530, 4);
    CHECK(air530_parse(SOUTH_EAST_RMC));
    air530_fix_t fix = current_fix();
    CHECK_NEAR(fix.latitude_deg, -(33.0 + 45.1234 / 60.0), 1e-9);
    CHECK_NEAR(fix.longitude_deg, 151.0 + 12.5678 / 60.0, 1e-9);
    CHECK(fix.year == 2026 && fix.month == 3 && fix.day == 15);
    CHECK(air530_parse(BEIDOU_RMC_V40));
    CHECK(current_fix().valid);
}

static void test_status_sentences(void) {
    start(AIR530_CHIP_AIR530, 4);
    CHECK(air530_antenna() == AIR530_ANTENNA_UNKNOWN);
    CHECK(!air530_parse(ANTENNA_OK));   // checked and read, but not a fix update
    CHECK(air530_antenna() == AIR530_ANTENNA_OK);
    air530_parse(ANTENNA_OPEN);
    CHECK(air530_antenna() == AIR530_ANTENNA_OPEN);

    CHECK(!air530_parse(GSA_SENTENCE));
    air530_fix_t fix = current_fix();
    CHECK(fix.fix_type == 3);
    CHECK_NEAR(fix.pdop, 1.21, 1e-4);
    CHECK_NEAR(fix.hdop, 0.85, 1e-4);
    CHECK_NEAR(fix.vdop, 0.86, 1e-4);

    air530_parse(GPS_GSV);
    CHECK(current_fix().satellites_in_view == 11);
    air530_parse(BEIDOU_GSV);
    CHECK(current_fix().satellites_in_view == 16);
    air530_parse(GPS_GSV);
    CHECK(current_fix().satellites_in_view == 16);   // replaced, not added again

    int command = 0, result = 0;
    CHECK(!air530_last_ack(&command, &result));
    air530_parse(ACK_101_DONE);
    CHECK(air530_last_ack(&command, &result));
    CHECK(command == 101 && result == 3);
}

static void test_bad_sentences(void) {
    start(AIR530_CHIP_AIR530, 4);
    air530_stats_t stats;
    CHECK(!air530_parse("$GNRMC,143012.000,A,4129.1234,N,07131.5678,W,0.12,87.30,291026,,,A,V*2F"));
    CHECK(!air530_parse("$GNRMC,143012.000,A,4129.1234,N,07131.5678,W,0.12,87.30,291026,,,A,V*2"));
    CHECK(!air530_parse("$GNRMC,143012.000,A,4129.1234,N,07131.5678,W,0.12,87.30,291026,,,A,V*2EX"));
    CHECK(!air530_parse("$GNRMC,143012.000,A,4129.1234,N,07131.5678,W,0.12,87.30,291026,,,A,V"));
    CHECK(!air530_parse("$GNRMC,143012.000,A,4129.1234,N,07131.5678,W,0.12,87.30,291026,,,A,V*G1"));
    air530_get_stats(&stats);
    CHECK(stats.bad_checksums == 5);
    CHECK(stats.sentences == 0);
    CHECK(!current_fix().valid);

    CHECK(!air530_parse("GNRMC,143012.000*00"));   // no '$': the tail of a sentence
    CHECK(!air530_parse("$GNRMC,A*38"));           // good checksum, too few fields
    CHECK(!current_fix().valid);
}

static int callback_count;
static char callback_last[128];
static void on_sentence(const char *sentence) {
    callback_count++;
    snprintf(callback_last, sizeof callback_last, "%s", sentence);
}

static void test_stream(void) {
    start(AIR530_CHIP_AIR530, 4);
    air530_fix_t fix;
    CHECK(!air530_get_fix(&fix));

    // Half a sentence from before init, then a burst split at odd places
    fake_uart_receive("1,0.85,18.1,M,-34.2,M,,*77\r\n" NO_FIX_GGA "\r\n$GNRM");
    CHECK(air530_service() == 1);
    CHECK(air530_get_fix(&fix));
    CHECK(!fix.valid);
    CHECK(!air530_get_fix(&fix));   // nothing new since

    fake_uart_receive("C,143012.000,A,4129.1234,N,07131.5678,W,0.12,87.30,291026,,,A,V*2E\r");
    CHECK(air530_service() == 1);   // '\r' ends the sentence; the '\n' after it is ignored
    CHECK(air530_get_fix(&fix));
    CHECK(fix.valid);
    fake_uart_receive("\n" GPS_GSV "\r\n" FIX_GGA "\r\n");
    CHECK(air530_service() == 1);

    // Bytes lost mid-sentence: the '$' of the next sentence restarts the line
    fake_uart_receive("$GNRMC,143014.000,A,41" FIX_RMC "\r\n");
    CHECK(air530_service() == 1);

    // A 400-byte line of noise is thrown away and counted once
    char noise[402];
    memset(noise, 'x', 400);
    noise[0] = '$';
    noise[400] = '\n';
    noise[401] = '\0';
    fake_uart_receive(noise);
    fake_uart_receive(FIX_RMC "\r\n");
    CHECK(air530_service() == 1);

    air530_stats_t stats;
    air530_get_stats(&stats);
    CHECK(stats.too_long == 1);
    CHECK(stats.bad_checksums == 0);
    CHECK(stats.bytes_dropped == 0);
    CHECK(stats.sentences == 6);

    // Raw sentences go to the callback, checked, before parsing
    air530_set_sentence_callback(on_sentence);
    callback_count = 0;
    fake_uart_receive(ANTENNA_OK "\r\n$GPTXT,bad*00\r\n");
    air530_service();
    CHECK(callback_count == 1);
    CHECK(strcmp(callback_last, ANTENNA_OK) == 0);
    air530_set_sentence_callback(NULL);
}

static void test_buffer_full(void) {
    start(AIR530_CHIP_AIR530, 4);
    // About 1.5 s of data at 9600 baud while main is busy: the ring keeps 1023 bytes
    for (int i = 0; i < 20; i++) {
        fake_uart_receive(FIX_RMC "\r\n");   // 73 bytes each, 1460 in all
    }
    air530_stats_t stats;
    air530_get_stats(&stats);
    CHECK(stats.bytes_dropped == 1460 - 1023);
    CHECK(air530_service() == 14);   // the 14 whole sentences that fit (14 * 73 = 1022)
    fake_uart_receive(FIX_RMC "\r\n");
    CHECK(air530_service() == 1);    // back to normal
    air530_get_stats(&stats);
    CHECK(stats.bad_checksums <= 1);  // at most the one sentence cut by the overflow
}

static void test_air530_commands(void) {
    start(AIR530_CHIP_AIR530, 4);
    CHECK(air530_send_command("PGKC462"));
    CHECK_SENT("$PGKC462*2F\r\n");   // booklet examples from here on

    air530_parse(FIX_RMC);
    air530_fix_t fix;
    air530_get_fix(&fix);
    CHECK(air530_restart(AIR530_RESTART_COLD));
    CHECK_SENT("$PGKC030,3,1*2E\r\n");
    CHECK(!air530_get_fix(&fix));
    CHECK(!fix.valid);
    CHECK(air530_restart(AIR530_RESTART_HOT));
    CHECK_SENT("$PGKC030,1,1*2C\r\n");
    CHECK(!air530_restart(0));
    CHECK(!air530_restart(5));

    CHECK(air530_set_update_interval_ms(1000));
    CHECK_SENT("$PGKC101,1000*02\r\n");
    CHECK(!air530_set_update_interval_ms(100));
    CHECK(!air530_set_update_interval_ms(20000));

    CHECK(air530_set_constellations(AIR530_GNSS_GPS));
    CHECK_SENT("$PGKC115,1,0,0,0*2B\r\n");
    CHECK(air530_set_constellations(AIR530_GNSS_GPS | AIR530_GNSS_BEIDOU));
    CHECK_SENT("$PGKC115,1,0,1,0*2A\r\n");
    CHECK(!air530_set_constellations(0));

    CHECK(air530_set_sentences(AIR530_SENTENCE_GLL | AIR530_SENTENCE_RMC | AIR530_SENTENCE_VTG |
                               AIR530_SENTENCE_GGA | AIR530_SENTENCE_GSA | AIR530_SENTENCE_GSV));
    CHECK_SENT("$PGKC242,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*37\r\n");
    CHECK(!air530_set_sentences(AIR530_SENTENCE_ZDA));

    CHECK(air530_set_baud(115200));
    CHECK_SENT("$PGKC147,115200*06\r\n");
    CHECK(fake_uart_baud() == 115200);
    CHECK(!air530_set_baud(4800));
    CHECK(!air530_set_baud(12345));
    CHECK(fake_uart_baud() == 115200);
}

static void test_air530z_commands(void) {
    start(AIR530_CHIP_AIR530Z, 4);
    CHECK(air530_restart(AIR530_RESTART_COLD));
    CHECK_SENT("$PCAS10,2*1E\r\n");
    CHECK(air530_set_update_interval_ms(200));
    CHECK_SENT("$PCAS02,200*1D\r\n");
    CHECK(!air530_set_update_interval_ms(300));
    CHECK(air530_set_constellations(AIR530_GNSS_GPS | AIR530_GNSS_GLONASS));
    CHECK_SENT("$PCAS04,5*1C\r\n");
    CHECK(!air530_set_constellations(AIR530_GNSS_GALILEO));
    CHECK(air530_set_baud(115200));
    CHECK_SENT("$PCAS01,5*19\r\n");
    CHECK(!air530_set_baud(230400));
    CHECK(air530_set_sentences(AIR530_SENTENCE_RMC | AIR530_SENTENCE_GGA));
    CHECK(strncmp(fake_uart_sent(), "$PCAS03,1,0,0,0,1,0,0,", 22) == 0);
    fake_uart_clear_sent();
}

int main(void) {
    test_init();
    test_no_fix();
    test_fix();
    test_other_talkers();
    test_status_sentences();
    test_bad_sentences();
    test_stream();
    test_buffer_full();
    test_air530_commands();
    test_air530z_commands();
    printf("test_air530: %d checks, %d failed\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
