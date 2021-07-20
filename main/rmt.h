#ifndef RMT_H
#define RMT_H

#include <driver/rmt.h>

typedef void (*rmt_on_rx_items_received_cb_t)(uint64_t rxt_us, size_t num_items,
    rmt_item32_t *items);

int rmt_initialize(void);

void rmt_set_on_rx_items_received_cb(rmt_on_rx_items_received_cb_t cb);

#endif
