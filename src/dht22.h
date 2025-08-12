// dht22.h
// DHT22/AM2302 support for Klipper MCU (RP2040)
#pragma once

#include <stdint.h>

// Configure DHT22 pin and minimum interval between reads (in ticks)
void dht22_configure(uint8_t pin, uint32_t min_interval_ticks);

// Perform a blocking read; returns 1 on success, 0 on failure
// On success, writes tenths units to *temp10 (C*10) and *hum10 (%*10)
uint8_t dht22_read_blocking(uint16_t *temp10, uint16_t *hum10);
