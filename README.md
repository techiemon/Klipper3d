Welcome to the Klipper project!

[![Klipper](docs/img/klipper-logo-small.png)](https://www.klipper3d.org/)

https://www.klipper3d.org/

The Klipper firmware controls 3d-Printers. It combines the power of a
general purpose computer with one or more micro-controllers. See the
[features document](https://www.klipper3d.org/Features.html) for more
information on why you should use the Klipper software.

Start by [installing Klipper software](https://www.klipper3d.org/Installation.html).

Klipper software is Free Software. See the [license](COPYING) or read
the [documentation](https://www.klipper3d.org/Overview.html). We
depend on the generous support from our
[sponsors](https://www.klipper3d.org/Sponsors.html).

```
<!-- max6675_heater.cfg -->


[mcu pico]
serial: /dev/serial/by-id/usb-Klipper_rp2040_E664B49507598426-if00

# MCU internal temperature (loads heaters)
[temperature_sensor mcu_temp]
sensor_type: temperature_mcu
sensor_mcu: pico

# Now safe to load AHT10 driver (needs 'heaters' to exist)
[aht10]

# Ambient sensor: AHT20 on RP2040 I2C1 (GP6 SDA, GP7 SCL)
[temperature_sensor my_ambient]
sensor_type: AHT10
i2c_mcu: pico
i2c_bus: i2c1a
i2c_address: 56

[temperature_sensor element_temp] 
sensor_type: MAX6675
sensor_pin: pico:gpio5
spi_speed: 100000
spi_software_sclk_pin: pico:gpio6
spi_software_miso_pin: pico:gpio4
spi_software_mosi_pin: pico:gpio7

# Ambient sensor: AHT20 on RP2040 I2C1 (GP6 SDA, GP7 SCL)
[aht10 my_ambient]
i2c_bus: i2c1a
i2c_address: 0x38
# Optional:
# rate: 1.0

# Heater using MAX6675 on SPI0 (GP2 SCLK, GP3 CS, GP4 MISO)
[max6675_heater my_heater]
mcu: pico
ssr_pin: pico:gpio14
fan_pin: pico:gpio8
fan_active_high: True
ambient_sensor: aht10 my_ambient
element_sensor: temperature_sensor element_temp
# Optional:
# target_air: 40.0
# element_offset: 10.0
# If your fan wiring is active-low, set fan_active_high: False.
# After saving, send RESTART, then:
# QUERY_HEATER to see air/element temps.
# SET_HEATER_POWER S=0.2 to verify SSR PWM and that the fan turns on when heating.

# Add the following to your [include] or printer.cfg
#[include config/max6675_heater.cfg]

# Example GCODEs:
# SET_HEATER_TEMP S=100 — Set target temperature to 100°C.
# SET_HEATER_POWER S=0.5 — Set heater power to 50%.
# QUERY_HEATER
#  — Report current temp and power.
```