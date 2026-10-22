/*
 * air530.c
 * UART driver for the Air530 GPS receiver. See air530.h for how to use it.
 *
 * How it works: the UART receive interrupt copies every byte into a ring buffer
 * (1024 bytes, about 1 s at 9600 baud), so nothing is lost while main is busy
 * with an SD write. air530_service() takes bytes out of the ring, builds lines,
 * checks each checksum, and parses the sentences it knows. Parsing never runs
 * in the interrupt.
 *
 * The NMEA parser started from the A7_GPS_UART example in OCE360_560_codebase
 * (Mingxi Zhou), originally by Yousef Yasser, Rasheed Atia, and Seifeldin
 * Khaled (2025, MIT). Air530 commands: Air530 user booklet v1.7, section 7.
 *
 * Author:  OCE360/560 instructors
 * License: MIT (see LICENSE)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "air530.h"

#define RX_BUFFER_SIZE 1024        // power of two
#define MAX_SENTENCE_LENGTH 121    // NMEA caps a sentence at 82 characters; room for long $PGKC replies
#define MAX_FIELDS 24
#define TALKER_COUNT 5             // GP, GL, BD/GB, GA, GQ: for summing satellites in view

#define RMC_MIN_FIELDS 10          // through the date field
#define GGA_MIN_FIELDS 10          // through the altitude field
#define GSA_MIN_FIELDS 18          // through VDOP
#define GSV_MIN_FIELDS 4           // through satellites in view
#define TXT_MIN_FIELDS 5

// ---- Driver state --------------------------------------------------------

static air530_config_t config;
static bool initialized = false;

static volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile uint32_t rx_head = 0;   // written only by the interrupt
static volatile uint32_t rx_tail = 0;   // written only by air530_service()
static volatile uint32_t bytes_dropped = 0;

static char line[MAX_SENTENCE_LENGTH];
static unsigned int line_length = 0;
static bool line_overflowed = false;

static air530_fix_t latest_fix;
static bool fix_is_new = false;
static uint8_t satellites_in_view_by_talker[TALKER_COUNT];
static air530_antenna_t antenna = AIR530_ANTENNA_UNKNOWN;
static air530_stats_t stats;
static bool echo_raw = false;
static void (*sentence_callback)(const char *sentence) = NULL;
static int ack_command = -1;
static int ack_result = -1;

// ---- Receive interrupt ---------------------------------------------------

static void on_uart_rx(void) {
    while (uart_is_readable(config.uart)) {
        uint8_t received = (uint8_t)uart_getc(config.uart);
        uint32_t next_head = (rx_head + 1) & (RX_BUFFER_SIZE - 1);
        if (next_head == rx_tail) {
            bytes_dropped++;   // full: main hasn't called air530_service() for about a second
            continue;
        }
        rx_buffer[rx_head] = received;
        rx_head = next_head;
    }
}

static unsigned int uart_irq_number(uart_inst_t *uart) {
    return uart_get_index(uart) == 0 ? UART0_IRQ : UART1_IRQ;
}

// ---- NMEA helpers --------------------------------------------------------

/** XOR of every character from start up to (not including) end. */
static unsigned int nmea_checksum(const char *start, const char *end) {
    unsigned int checksum = 0;
    for (const char *p = start; p < end; p++) {
        checksum ^= (unsigned char)*p;
    }
    return checksum;
}

/** True if text ends in '*' and two hex digits that match the XOR of everything between '$' and '*'. */
static bool checksum_matches(const char *text) {
    const char *star = strchr(text, '*');
    if (star == NULL || strlen(star) != 3) {
        return false;
    }
    char *end_of_number;
    unsigned int received = strtoul(star + 1, &end_of_number, 16);
    if (end_of_number != star + 3) {
        return false;   // not two hex digits
    }
    return nmea_checksum(text + 1, star) == received;
}

/**
 * Split text in place at every ',' and '*'. Unlike strtok, empty fields are kept,
 * so field 6 is always field 6 even when the fields before it are blank (no fix).
 */
static int split_fields(char *text, char *fields[], int max_fields) {
    int count = 0;
    fields[count++] = text;
    for (char *p = text; *p != '\0' && count < max_fields; p++) {
        if (*p == ',' || *p == '*') {
            *p = '\0';
            fields[count++] = p + 1;
        }
    }
    return count;
}

/** "4129.1234" + "N" -> 41.485390. NMEA sends degrees and minutes run together (ddmm.mmmm). */
static double nmea_to_degrees(const char *degrees_minutes, const char *hemisphere) {
    double value = strtod(degrees_minutes, NULL);
    int whole_degrees = (int)(value / 100.0);
    double minutes = value - whole_degrees * 100.0;
    double degrees = whole_degrees + minutes / 60.0;
    bool negative = hemisphere[0] == 'S' || hemisphere[0] == 'W';
    return negative ? -degrees : degrees;
}

/** True if the header is "$xxTYPE" for any two-letter talker, e.g. "$GNRMC" for "RMC". */
static bool is_sentence_type(const char *header, const char *type) {
    return strlen(header) == 6 && strcmp(header + 3, type) == 0;
}

static bool is_digits(const char *text, int count) {
    for (int i = 0; i < count; i++) {
        if (text[i] < '0' || text[i] > '9') {
            return false;
        }
    }
    return true;
}

static int two_digits(const char *text) {
    return (text[0] - '0') * 10 + (text[1] - '0');
}

/** "hhmmss.sss" -> fix time. Leaves the fix alone and returns false if the field is empty or malformed. */
static bool parse_time(const char *field, air530_fix_t *fix) {
    if (strlen(field) < 6 || !is_digits(field, 6)) {
        return false;
    }
    fix->hour = two_digits(field);
    fix->minute = two_digits(field + 2);
    fix->second = two_digits(field + 4);
    fix->millisecond = field[6] == '.' ? (uint16_t)(strtod(field + 6, NULL) * 1000.0 + 0.5) : 0;
    return true;
}

/** "ddmmyy" -> fix date (years 2000 to 2099). */
static bool parse_date(const char *field, air530_fix_t *fix) {
    if (strlen(field) != 6 || !is_digits(field, 6)) {
        return false;
    }
    fix->day = two_digits(field);
    fix->month = two_digits(field + 2);
    fix->year = 2000 + two_digits(field + 4);
    return true;
}

static int talker_index(const char *header) {
    if (strncmp(header + 1, "GP", 2) == 0) return 0;
    if (strncmp(header + 1, "GL", 2) == 0) return 1;
    if (strncmp(header + 1, "BD", 2) == 0 || strncmp(header + 1, "GB", 2) == 0) return 2;
    if (strncmp(header + 1, "GA", 2) == 0) return 3;
    if (strncmp(header + 1, "GQ", 2) == 0) return 4;
    return -1;
}

// ---- Sentence parsers ----------------------------------------------------

// RMC: $--RMC,time,status,lat,N/S,lon,E/W,speed_kn,course,date,magvar,E/W[,mode[,nav_status]]*hh
// Time and date come only from RMC, so they always belong to the same second.
static void parse_rmc(char *fields[], int field_count, air530_fix_t *fix) {
    fix->time_valid = parse_time(fields[1], fix);
    fix->date_valid = parse_date(fields[9], fix);
    bool mode_says_no_fix = field_count > 12 && fields[12][0] == 'N';
    fix->valid = fields[2][0] == 'A' && !mode_says_no_fix;
    if (fix->valid) {
        fix->latitude_deg = nmea_to_degrees(fields[3], fields[4]);
        fix->longitude_deg = nmea_to_degrees(fields[5], fields[6]);
        fix->speed_knots = strtof(fields[7], NULL);
        if (fields[8][0] != '\0') {
            fix->course_deg = strtof(fields[8], NULL);   // empty when standing still
        }
    }
}

// GGA: $--GGA,time,lat,N/S,lon,E/W,quality,satellites,hdop,altitude,M,geoid,M,age,station*hh
static void parse_gga(char *fields[], int field_count, air530_fix_t *fix) {
    fix->fix_quality = (uint8_t)atoi(fields[6]);
    fix->satellites_used = (uint8_t)atoi(fields[7]);
    if (fields[8][0] != '\0') {
        fix->hdop = strtof(fields[8], NULL);
    }
    fix->valid = fix->fix_quality > 0;
    if (fix->valid) {
        fix->latitude_deg = nmea_to_degrees(fields[2], fields[3]);
        fix->longitude_deg = nmea_to_degrees(fields[4], fields[5]);
        fix->altitude_m = strtof(fields[9], NULL);
        if (field_count > 11) {
            fix->geoid_separation_m = strtof(fields[11], NULL);
        }
    }
}

// GSA: $--GSA,mode,fix_type,sv1,...,sv12,pdop,hdop,vdop[,system_id]*hh
// A multi-constellation receiver sends one GSA per system with the same DOPs.
static void parse_gsa(char *fields[], air530_fix_t *fix) {
    fix->fix_type = (uint8_t)atoi(fields[2]);
    if (fields[15][0] != '\0') fix->pdop = strtof(fields[15], NULL);
    if (fields[16][0] != '\0') fix->hdop = strtof(fields[16], NULL);
    if (fields[17][0] != '\0') fix->vdop = strtof(fields[17], NULL);
}

// GSV: $--GSV,total_messages,message_number,satellites_in_view,...*hh, one set per constellation
static void parse_gsv(char *fields[], air530_fix_t *fix) {
    int talker = talker_index(fields[0]);
    if (talker < 0 || fields[3][0] == '\0') {
        return;
    }
    satellites_in_view_by_talker[talker] = (uint8_t)atoi(fields[3]);
    int total = 0;
    for (int i = 0; i < TALKER_COUNT; i++) {
        total += satellites_in_view_by_talker[i];
    }
    fix->satellites_in_view = (uint8_t)(total > 255 ? 255 : total);
}

// TXT: $GPTXT,total,number,type,text*hh, e.g. "ANTENNA OK"
static void parse_txt(char *fields[]) {
    const char *text = fields[4];
    if (strstr(text, "ANTENNA OK") != NULL) antenna = AIR530_ANTENNA_OK;
    else if (strstr(text, "ANTENNA OPEN") != NULL) antenna = AIR530_ANTENNA_OPEN;
    else if (strstr(text, "ANTENNA SHORT") != NULL) antenna = AIR530_ANTENNA_SHORT;
}

/** Check, report, and parse one sentence. text is changed in place. */
static bool process_sentence(char *text) {
    if (text[0] != '$') {
        return false;   // the tail of a sentence that started before air530_init()
    }
    if (!checksum_matches(text)) {
        stats.bad_checksums++;
        return false;
    }
    stats.sentences++;
    if (sentence_callback != NULL) {
        sentence_callback(text);
    }

    char *fields[MAX_FIELDS];
    int field_count = split_fields(text, fields, MAX_FIELDS);
    const char *header = fields[0];

    if (is_sentence_type(header, "RMC") && field_count >= RMC_MIN_FIELDS) {
        parse_rmc(fields, field_count, &latest_fix);
    } else if (is_sentence_type(header, "GGA") && field_count >= GGA_MIN_FIELDS) {
        parse_gga(fields, field_count, &latest_fix);
    } else if (is_sentence_type(header, "GSA") && field_count >= GSA_MIN_FIELDS) {
        parse_gsa(fields, &latest_fix);
        return false;
    } else if (is_sentence_type(header, "GSV") && field_count >= GSV_MIN_FIELDS) {
        parse_gsv(fields, &latest_fix);
        return false;
    } else if (is_sentence_type(header, "TXT") && field_count >= TXT_MIN_FIELDS) {
        parse_txt(fields);
        return false;
    } else if (strcmp(header, "$PGKC001") == 0 && field_count >= 3) {
        ack_command = atoi(fields[1]);
        ack_result = atoi(fields[2]);
        return false;
    } else {
        return false;   // VTG, GLL, ZDA ...: checked but not used
    }
    latest_fix.updated_us = time_us_64();
    fix_is_new = true;
    return true;
}

static void clear_line(void) {
    line_length = 0;
    line_overflowed = false;
}

// ---- Public functions ----------------------------------------------------

bool air530_init(const air530_config_t *new_config) {
    if (new_config == NULL || new_config->uart == NULL || new_config->baud == 0) {
        return false;
    }
    if (initialized) {
        unsigned int old_irq = uart_irq_number(config.uart);
        irq_set_enabled(old_irq, false);
        irq_remove_handler(old_irq, on_uart_rx);
    }
    config = *new_config;

    uart_init(config.uart, config.baud);   // 8 data bits, no parity, 1 stop bit by default
    uart_set_hw_flow(config.uart, false, false);
    gpio_set_function(config.rx_pin, GPIO_FUNC_UART);
    if (config.tx_pin != AIR530_PIN_NOT_WIRED) {
        gpio_set_function(config.tx_pin, GPIO_FUNC_UART);
    }

    rx_head = rx_tail = 0;
    bytes_dropped = 0;
    clear_line();
    memset(&latest_fix, 0, sizeof latest_fix);
    memset(satellites_in_view_by_talker, 0, sizeof satellites_in_view_by_talker);
    memset(&stats, 0, sizeof stats);
    fix_is_new = false;
    antenna = AIR530_ANTENNA_UNKNOWN;
    ack_command = ack_result = -1;

    unsigned int irq = uart_irq_number(config.uart);
    irq_set_exclusive_handler(irq, on_uart_rx);
    irq_set_enabled(irq, true);
    uart_set_irq_enables(config.uart, true, false);   // receive only
    initialized = true;
    return true;
}

int air530_service(void) {
    int updates = 0;
    while (rx_tail != rx_head) {
        char received = (char)rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) & (RX_BUFFER_SIZE - 1);
        if (echo_raw) {
            putchar(received);
        }
        if (received == '$') {
            clear_line();   // a new sentence: drop any half sentence before it
        }
        if (received == '\r' || received == '\n') {
            if (line_length > 0 && !line_overflowed) {
                line[line_length] = '\0';
                updates += process_sentence(line);
            }
            clear_line();
        } else if (line_length < MAX_SENTENCE_LENGTH - 1) {
            line[line_length++] = received;
        } else if (!line_overflowed) {
            line_overflowed = true;   // throw the rest away until the next '$'
            stats.too_long++;
        }
    }
    return updates;
}

bool air530_get_fix(air530_fix_t *fix) {
    *fix = latest_fix;
    bool was_new = fix_is_new;
    fix_is_new = false;
    return was_new;
}

air530_antenna_t air530_antenna(void) {
    return antenna;
}

void air530_get_stats(air530_stats_t *out) {
    *out = stats;
    out->bytes_dropped = bytes_dropped;
}

void air530_set_echo(bool echo_on) {
    echo_raw = echo_on;
}

void air530_set_sentence_callback(void (*fn)(const char *sentence)) {
    sentence_callback = fn;
}

bool air530_parse(const char *sentence) {
    char copy[MAX_SENTENCE_LENGTH];
    if (strlen(sentence) >= sizeof copy) {
        return false;
    }
    strcpy(copy, sentence);
    return process_sentence(copy);
}

bool air530_send_command(const char *body) {
    if (!initialized || config.tx_pin == AIR530_PIN_NOT_WIRED) {
        return false;
    }
    char command[MAX_SENTENCE_LENGTH + 8];
    unsigned int checksum = nmea_checksum(body, body + strlen(body));
    int length = snprintf(command, sizeof command, "$%s*%02X\r\n", body, checksum);
    if (length < 0 || length >= (int)sizeof command) {
        return false;
    }
    uart_puts(config.uart, command);   // waits while the 32-byte TX FIFO is full: about 1 ms per byte past 32
    return true;
}

bool air530_restart(air530_restart_t kind) {
    if (kind < AIR530_RESTART_HOT || kind > AIR530_RESTART_FULL_COLD) {
        return false;
    }
    char body[24];
    if (config.chip == AIR530_CHIP_AIR530) {
        snprintf(body, sizeof body, "PGKC030,%d,1", (int)kind);
    } else {
        snprintf(body, sizeof body, "PCAS10,%d", (int)kind - 1);   // 0 hot, 1 warm, 2 cold, 3 factory
    }
    if (!air530_send_command(body)) {
        return false;
    }
    // A sentence already on its way can still say "fix": callers timing a restart
    // should wait until valid goes false once before trusting it again.
    latest_fix.valid = false;
    latest_fix.fix_quality = 0;
    latest_fix.fix_type = 0;
    latest_fix.satellites_used = 0;
    fix_is_new = false;
    return true;
}

bool air530_set_update_interval_ms(uint32_t interval_ms) {
    char body[24];
    if (config.chip == AIR530_CHIP_AIR530) {
        if (interval_ms < 200 || interval_ms > 10000) {
            return false;
        }
        snprintf(body, sizeof body, "PGKC101,%lu", (unsigned long)interval_ms);
    } else {
        if (interval_ms != 100 && interval_ms != 200 && interval_ms != 250 &&
            interval_ms != 500 && interval_ms != 1000) {
            return false;
        }
        snprintf(body, sizeof body, "PCAS02,%lu", (unsigned long)interval_ms);
    }
    return air530_send_command(body);
}

static int bit(uint32_t mask, uint32_t flag) {
    return (mask & flag) != 0;
}

bool air530_set_sentences(uint32_t mask) {
    char body[64];
    if (config.chip == AIR530_CHIP_AIR530) {
        if (mask & AIR530_SENTENCE_ZDA) {
            return false;
        }
        // GLL, RMC, VTG, GGA, GSA, GSV, GRS, GST, then 11 reserved
        snprintf(body, sizeof body, "PGKC242,%d,%d,%d,%d,%d,%d,0,0,0,0,0,0,0,0,0,0,0,0,0",
                 bit(mask, AIR530_SENTENCE_GLL), bit(mask, AIR530_SENTENCE_RMC),
                 bit(mask, AIR530_SENTENCE_VTG), bit(mask, AIR530_SENTENCE_GGA),
                 bit(mask, AIR530_SENTENCE_GSA), bit(mask, AIR530_SENTENCE_GSV));
    } else {
        // GGA, GLL, GSA, GSV, RMC, VTG, ZDA, ANT, DHV, LPS, -, -, UTC, GST, -, -, -, TIM
        snprintf(body, sizeof body, "PCAS03,%d,%d,%d,%d,%d,%d,%d,0,0,0,,,0,0,,,,0",
                 bit(mask, AIR530_SENTENCE_GGA), bit(mask, AIR530_SENTENCE_GLL),
                 bit(mask, AIR530_SENTENCE_GSA), bit(mask, AIR530_SENTENCE_GSV),
                 bit(mask, AIR530_SENTENCE_RMC), bit(mask, AIR530_SENTENCE_VTG),
                 bit(mask, AIR530_SENTENCE_ZDA));
    }
    return air530_send_command(body);
}

bool air530_set_constellations(uint32_t mask) {
    char body[24];
    if (config.chip == AIR530_CHIP_AIR530) {
        if (mask == 0) {
            return false;
        }
        snprintf(body, sizeof body, "PGKC115,%d,%d,%d,%d",
                 bit(mask, AIR530_GNSS_GPS), bit(mask, AIR530_GNSS_GLONASS),
                 bit(mask, AIR530_GNSS_BEIDOU), bit(mask, AIR530_GNSS_GALILEO));
    } else {
        if (mask == 0 || (mask & AIR530_GNSS_GALILEO)) {
            return false;
        }
        // 1 GPS, 2 BeiDou, 4 GLONASS, added together
        int systems = bit(mask, AIR530_GNSS_GPS) | bit(mask, AIR530_GNSS_BEIDOU) << 1 |
                      bit(mask, AIR530_GNSS_GLONASS) << 2;
        snprintf(body, sizeof body, "PCAS04,%d", systems);
    }
    return air530_send_command(body);
}

bool air530_set_baud(uint32_t baud) {
    static const uint32_t rates[] = {4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
    int code = -1;
    for (int i = 0; i < (int)(sizeof rates / sizeof rates[0]); i++) {
        if (rates[i] == baud) {
            code = i;
        }
    }
    char body[24];
    if (config.chip == AIR530_CHIP_AIR530) {
        if (code < 1) {
            return false;   // the booklet lists 9600 and up
        }
        snprintf(body, sizeof body, "PGKC147,%lu", (unsigned long)baud);
    } else {
        if (code < 0 || code > 5) {
            return false;   // PCAS01 codes 0 to 5: 4800 to 115200
        }
        snprintf(body, sizeof body, "PCAS01,%d", code);
    }
    if (!air530_send_command(body)) {
        return false;
    }
    uart_tx_wait_blocking(config.uart);   // let the command leave at the old rate
    sleep_ms(100);
    uart_set_baudrate(config.uart, baud);
    config.baud = baud;
    clear_line();
    return true;
}

bool air530_last_ack(int *command, int *result) {
    if (ack_command < 0) {
        return false;
    }
    *command = ack_command;
    *result = ack_result;
    return true;
}
