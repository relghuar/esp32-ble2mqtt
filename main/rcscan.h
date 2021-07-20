#ifndef RCSCAN_H
#define RCSCAN_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef struct rc_pulse_s {
    union {
        struct {
            uint16_t duration :15;
            uint16_t level :1;
        };
        uint16_t val;
    };
} rc_pulse_t;

typedef struct high_low_s {
  uint8_t high;
  uint8_t low;
} high_low_t;

typedef struct rc_protocol_s {
    uint8_t id;
    uint8_t type;
    uint8_t tolerance;
    union {
        struct {
            uint16_t pulse_us;
            high_low_t sync;
            high_low_t zero;
            high_low_t one;
            bool inverted;
        } pwm;
    };
} rc_protocol_t;

#define PTYPE_PWMSS 1

rc_protocol_t *rc_decode_signal(size_t num_pulses, rc_pulse_t *pulses,
    size_t *num_bits, uint64_t *value);

bool rc_signal_duplicate(uint64_t rxt_us, rc_protocol_t *proto, size_t num_bits,
    uint64_t value);

#endif
