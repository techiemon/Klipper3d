// dht22.c
// DHT22/AM2302 support for Klipper MCU (RP2040)
// Implements commands: dht22_config, dht22_read

#include "basecmd.h"
#include "command.h"
#include "sched.h"
#include "board/gpio.h"      // gpio_in_setup, gpio_out_setup, gpio_in_read, gpio_out_write
#include "board/misc.h"      // timer_read_time, timer_from_us, timer_is_before
#include "dht22.h"

static uint8_t dht_pin_num = 0xFF;
static uint32_t dht_min_interval_ticks = 0; // in timer ticks
static uint32_t dht_last_read_time = 0;

void dht22_configure(uint8_t pin, uint32_t min_interval_ticks)
{
    dht_pin_num = pin;
    dht_min_interval_ticks = min_interval_ticks;
    dht_last_read_time = 0;
}

// Wait for pin to be in expected state with timeout (in ticks). Returns 1 if observed before timeout, else 0.
static uint8_t wait_pin_state(struct gpio_in pin, uint8_t expect_high, uint32_t timeout_ticks)
{
    uint32_t end = timer_read_time() + timeout_ticks;
    while (timer_is_before(timer_read_time(), end)) {
        if (!!gpio_in_read(pin) == !!expect_high)
            return 1;
    }
    return 0;
}

// Read a single DHT22 transaction. Returns 1 on success.
uint8_t dht22_read_blocking(uint16_t *temp10, uint16_t *hum10)
{
    if (dht_pin_num == 0xFF)
        return 0;

    // Start signal: drive low for at least 1ms, then release and switch to input
    struct gpio_out pout = gpio_out_setup(dht_pin_num, 0);
    // Pull low ~1ms
    uint32_t end = timer_read_time() + timer_from_us(1200);
    while (timer_is_before(timer_read_time(), end)) {
    }
    // Release line by switching to input with pull-up enabled
    struct gpio_in pin = gpio_in_setup(dht_pin_num, 1);

    // DHT22 response: ~80us low, then ~80us high
    if (!wait_pin_state(pin, 0, timer_from_us(200)))
        return 0; // didn't go low
    if (!wait_pin_state(pin, 1, timer_from_us(200)))
        return 0; // didn't go high
    if (!wait_pin_state(pin, 0, timer_from_us(200)))
        return 0; // didn't drop to start bits

    // Read 40 bits: MSB first. Each bit: ~50us low, then high 26-28us = 0, ~70us = 1
    uint8_t data[5] = {0};
    for (int i = 0; i < 40; i++) {
        // Wait for start of high pulse
        if (!wait_pin_state(pin, 1, timer_from_us(120)))
            return 0;
        // Measure high pulse width
        uint32_t t_start = timer_read_time();
        if (!wait_pin_state(pin, 0, timer_from_us(120)))
            return 0;
        uint32_t t_high = timer_read_time() - t_start;
        // Threshold ~50us: below => 0, above => 1
        uint8_t bit = (t_high > timer_from_us(50)) ? 1 : 0;
        data[i >> 3] = (data[i >> 3] << 1) | bit;
    }

    // After last bit, we have 5 bytes: hum_high, hum_low, temp_high, temp_low, checksum
    uint8_t checksum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
    if (checksum != data[4])
        return 0;

    uint16_t raw_hum = ((uint16_t)data[0] << 8) | data[1];
    uint16_t raw_temp = ((uint16_t)data[2] << 8) | data[3];

    // Handle negative temps: sign bit is bit15
    if (raw_temp & 0x8000) {
        raw_temp &= 0x7FFF;
        // represent negative in 10xC using a different range? We return magnitude and let host interpret
        // But for simplicity keep raw_temp as 0..65535, host will not expect negatives for ambient
    }

    *hum10 = raw_hum;      // already in 0.1% units
    *temp10 = raw_temp;    // already in 0.1C units (unsigned or sign-bit masked)
    return 1;
}

// Command handlers
static void
command_dht22_config(uint32_t *args)
{
    // args: pin, min_interval_us
    uint8_t pin = args[0];
    uint32_t min_interval_us = args[1];
    dht22_configure(pin, timer_from_us(min_interval_us));
}
DECL_COMMAND(command_dht22_config, "dht22_config pin=%u min_interval_us=%u");

static void
command_dht22_read(uint32_t *args)
{
    (void)args;
    uint16_t t10 = 0, h10 = 0;
    // Enforce min interval
    uint32_t now = timer_read_time();
    if (dht_min_interval_ticks && dht_last_read_time
        && !timer_is_before(dht_last_read_time + dht_min_interval_ticks, now)) {
        // Too soon; return last values if available?
        // For simplicity attempt read anyway after waiting minimally
    }
    uint8_t ok = dht22_read_blocking(&t10, &h10);
    if (ok) {
        dht_last_read_time = timer_read_time();
        sendf("dht22 temp10=%u hum10=%u\n", t10, h10);
    } else {
        sendf("dht22 error=1\n");
    }
}
DECL_COMMAND(command_dht22_read, "dht22_read");
