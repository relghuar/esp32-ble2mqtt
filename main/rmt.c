
#include "config.h"
#include "rmt.h"
#include <esp_log.h>
#include <freertos/task.h>

static const char *TAG = "RMT";

gpio_num_t rx_gpio = -1;

static rmt_on_rx_items_received_cb_t on_rx_items_received_cb = NULL;

void rmt_set_on_rx_items_received_cb(rmt_on_rx_items_received_cb_t cb)
{
    on_rx_items_received_cb = cb;
}

static bool rmt_validate_rx_items(size_t num_items, rmt_item32_t *items,
    uint8_t min_length)
{
    // pattern must have at least 24bits data and 4+ sync bits, 28 total
    // plus some preamble - worst-case manchester would make 30 signals,
    // but that's not gonna happen, so let's take ~150%
    // --> 46 level changes make a minimum of 23 items
    if (num_items < 23)
        return false;

    // RMT driver filters minimum length of the whole item (duration0+duration1),
    // which still allows very short pulses/glitches.
    // Ideally, these should be "removed" from the items by merging them, but for
    // now let's just scratch whole sequences with such glitches.
    for (size_t i=0; i<num_items; i++)
        if (items[i].duration0 < min_length ||
                (i < num_items-1 && items[i].duration1 < min_length))
            return false;
    return true;
}

static void rmt_rx_task(void *arg)
{
    size_t length = 0;
    RingbufHandle_t rb = NULL;
    rmt_item32_t *items = NULL;
    rmt_channel_t rx_channel = RMT_CHANNEL_0;

    rmt_config_t rmt_rx_config = RMT_DEFAULT_CONFIG_RX(rx_gpio, rx_channel);
    rmt_rx_config.mem_block_num = 8;
    int idleus = config_rmt_rx_thr_idle_us();
    if (idleus >= 0 && idleus <= 65535)
        rmt_rx_config.rx_config.idle_threshold = idleus;
    int ignoreus = config_rmt_rx_thr_ignore_us();
    if (ignoreus >= 0 && ignoreus <= 255)
        rmt_rx_config.rx_config.filter_ticks_thresh = ignoreus;
    rmt_config(&rmt_rx_config);
    rmt_driver_install(rx_channel, 4096, 0);
    rmt_set_rx_idle_thresh(rx_channel, rmt_rx_config.rx_config.idle_threshold);
    rmt_set_rx_filter(rx_channel, true, rmt_rx_config.rx_config.filter_ticks_thresh);

    //get RMT RX ringbuffer
    rmt_get_ringbuf_handle(rx_channel, &rb);
    assert(rb != NULL);
    // Start receive
    rmt_rx_start(rx_channel, true);
    rmt_set_rx_idle_thresh(rx_channel, rmt_rx_config.rx_config.idle_threshold);
    rmt_set_rx_filter(rx_channel, true, rmt_rx_config.rx_config.filter_ticks_thresh);
    while (1)
    {
        items = (rmt_item32_t *) xRingbufferReceive(rb, &length, portMAX_DELAY);
        if (items)
        {
            uint64_t rxt_us = esp_timer_get_time();
            size_t num_items = length/4;
            if (on_rx_items_received_cb != NULL &&
                rmt_validate_rx_items(num_items, items,
                    rmt_rx_config.rx_config.filter_ticks_thresh))
            {
                on_rx_items_received_cb(rxt_us, num_items, items);
            }
            // after parsing the data, return spaces to ringbuffer.
            vRingbufferReturnItem(rb, (void *) items);
        }
    }
    rmt_driver_uninstall(rx_channel);
    vTaskDelete(NULL);
}

int rmt_initialize(void)
{
    ESP_LOGI(TAG, "Initializing RMT receiver");

    rx_gpio = config_rmt_rx_pin();
    ESP_LOGI(TAG, "RX_pin: %d", rx_gpio);

    if (rx_gpio >= 0)
        xTaskCreate(rmt_rx_task, "rmt_rx_task", 2048, NULL, 10, NULL);
    else
        ESP_LOGI(TAG, "No RX GPIO defined - RMT receiver inactive");

    return 0;
}

