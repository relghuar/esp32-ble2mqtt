
#include "rcscan.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

static const char *TAG = "RCSCAN";

static rc_protocol_t rc_protocols[] = {
//  { id, type,   tol_per, .pwm = { plen_us, sync-hl, zero-hl, one-hl, inverted, add-last }, },
    {  1, PTYPE_PWM, 25, .pwm = { 350, {  1, 31 }, {  1,  3 }, {  3,  1 }, false, false }, },    // protocol 1
    {  2, PTYPE_PWM, 25, .pwm = { 650, {  1, 10 }, {  1,  2 }, {  2,  1 }, false, false }, },    // protocol 2
    {  3, PTYPE_PWM, 25, .pwm = { 100, { 30, 71 }, {  4, 11 }, {  9,  6 }, false, false }, },    // protocol 3
    {  4, PTYPE_PWM, 25, .pwm = { 380, {  2,  6 }, {  1,  3 }, {  3,  1 }, false, false }, },    // protocol 4
    {  5, PTYPE_PWM, 25, .pwm = { 500, {  6, 14 }, {  1,  2 }, {  2,  1 }, false, false }, },    // protocol 5
    {  6, PTYPE_PWM, 25, .pwm = { 450, { 23,  1 }, {  1,  2 }, {  2,  1 }, true , false }, },    // protocol 6 (HT6P20B)
    {  7, PTYPE_PWM, 25, .pwm = { 150, {  2, 62 }, {  1,  6 }, {  6,  1 }, false, false }, },    // protocol 7 (HS2303-PT, i. e. used in AUKEY Remote)
    {  8, PTYPE_PWM, 25, .pwm = { 200, {  3, 130}, {  7, 16 }, {  3, 16 }, false, false }, },    // protocol 8 Conrad RS-200 RX
    {  9, PTYPE_PWM, 25, .pwm = { 200, { 130, 7 }, {  16, 7 }, { 16,  3 }, true , false }, },    // protocol 9 Conrad RS-200 TX
    { 10, PTYPE_PWM, 25, .pwm = { 365, { 18,  1 }, {  3,  1 }, {  1,  3 }, true , false }, },    // protocol 10 (1ByOne Doorbell)
    { 11, PTYPE_PWM, 25, .pwm = { 270, { 36,  1 }, {  1,  2 }, {  2,  1 }, true , false }, },    // protocol 11 (HT12E)
    { 12, PTYPE_PWM, 25, .pwm = { 320, { 36,  1 }, {  1,  2 }, {  2,  1 }, true , false }  },    // protocol 12 (SM5212)
    {101, PTYPE_PWM, 25, .pwm = { 500, {  1, 18 }, {  1,  4 }, {  1,  8 }, false, false }, },    // protocol 101 (Prologue outdoor meteo sensor, temp+humi)
    {102, PTYPE_PWM, 40, .pwm = { 333, {  1,  8 }, {  1,  2 }, {  2,  1 }, false, false }, },    // protocol 102 (Custom cesspit or soil sensor transmitter)
    {103, PTYPE_PWM, 30, .pwm = { 500, {  1, 18 }, {  3,  2 }, {  1,  2 }, false, true  }, },    // protocol 103 (WH2 outdoor meteo sensor, temp+humi)
};
static size_t rc_protocol_count = sizeof(rc_protocols)/sizeof(rc_protocol_t);

static inline unsigned int diff(int A, int B)
{
    return abs(A - B);
}

static inline unsigned int tolerance(uint16_t len, uint8_t tolratio)
{
    return (len*tolratio)/100;
}

static inline bool isInTolerance(uint16_t len, uint16_t pulse, uint8_t tolratio)
{
    return ( diff(len, pulse) < tolerance(pulse, tolratio) );
}

static bool rc_pulse_match(uint16_t l1, uint16_t l2, int plen,
    high_low_t *match, uint8_t tolratio, bool higher_tolerance, bool acc_zero)
{
    if (!isInTolerance(l1, match->high*plen, higher_tolerance ? tolratio*2 : tolratio)) return false;
    if (acc_zero && l2 == 0) return true;
    if (isInTolerance(l2, match->low*plen, higher_tolerance ? tolratio*2 : tolratio)) return true;
    return false;
}

#define HIGHER_TOL_PULSES 2

static bool rc_decode_signal_protocol(rc_protocol_t *proto, size_t num_pulses,
    rc_pulse_t *pulses, size_t *num_bits, uint64_t *value)
{
    *num_bits = 0;
    *value = 0;

    int plen = proto->pwm.pulse_us;
    int hi = proto->pwm.inverted ? 0 : 1;
    if (!proto->pwm.add_last_pulse && pulses[num_pulses-1].duration == 0) num_pulses--;

    int cpi = 0;
    if (pulses[cpi].level != hi) cpi++;
    if (rc_pulse_match(pulses[cpi].duration, pulses[cpi+1].duration, plen,
            &proto->pwm.sync, proto->tolerance, (cpi<=HIGHER_TOL_PULSES), false))
    {
        // sync pulse pair caught (shorter than RMT idle limit) - skip it
        cpi += 2;
    }
    while (cpi < num_pulses-1)
    {
        if (rc_pulse_match(pulses[cpi].duration, pulses[cpi+1].duration, plen,
                &proto->pwm.zero, proto->tolerance, (cpi<=HIGHER_TOL_PULSES), (cpi==num_pulses-2)))
            *value <<= 1L;
        else if (rc_pulse_match(pulses[cpi].duration, pulses[cpi+1].duration,
                plen, &proto->pwm.one, proto->tolerance, (cpi<=HIGHER_TOL_PULSES), (cpi==num_pulses-2)))
            *value = (*value<<1L) | 1;
        else
            break;
        cpi+=2;
        (*num_bits)++;
    }
    return cpi >= num_pulses-1;
}

rc_protocol_t *rc_decode_signal(size_t num_pulses, rc_pulse_t *pulses,
    size_t *num_bits, uint64_t *value)
{
    if (num_bits == NULL || value == NULL) return false;
    if (num_pulses < 16) return false;

    for (int i=0; i<rc_protocol_count; i++)
    {
        if (rc_decode_signal_protocol(&rc_protocols[i], num_pulses, pulses,
                num_bits, value))
            return &rc_protocols[i];
    }

    *num_bits = 0;
    *value = 0;
    return NULL;
}

// Signal log is used to filter out multiple transmissions of the same data,
// a mechanism used by many 433MHz transmitters to improve reception reliability.
// Because of length of individual transmissions including sync and delays,
// probability of receiving more than 20 different signals within 1s period
// is practically zero, so a buffer of that length is sufficient.

#define SIGNAL_LOG_LEN 20
#define DUPLICATE_WINDOW_MS 1000

typedef struct rc_signal_log_s
{
    uint32_t last_ms;
    size_t num_bits;
    uint64_t value;
} rc_signal_log_t;

rc_signal_log_t rc_signal_log[SIGNAL_LOG_LEN];

bool rc_signal_duplicate(uint64_t rxt_us, rc_protocol_t *proto, size_t num_bits,
    uint64_t value)
{
    uint32_t cur_ms = rxt_us/1000;
    int first_older = -1;
    for (int i=0; i<SIGNAL_LOG_LEN; i++)
    {
        // TODO: this check should take into account number of bits
        // shorter signals can be considered duplicates, even if they differ in MSBs
        if (rc_signal_log[i].value == value)
        {
            rc_signal_log[i].num_bits = num_bits;
            if (cur_ms-rc_signal_log[i].last_ms > DUPLICATE_WINDOW_MS)
            {
                rc_signal_log[i].last_ms = cur_ms;
                return false;
            } else {
                ESP_LOGD(TAG, "Duplicate RC signal detected (%u, %u, %d, %08llx)",
                    rc_signal_log[i].last_ms, cur_ms, num_bits, value);
                rc_signal_log[i].last_ms = cur_ms;
                return true;
            }
        }
        else if (cur_ms-rc_signal_log[i].last_ms > DUPLICATE_WINDOW_MS
                && first_older < 0)
        {
            first_older = i;
            rc_signal_log[i].last_ms = cur_ms;
            rc_signal_log[i].num_bits = num_bits;
            rc_signal_log[i].value = value;
        }
    }
    return false;
}

