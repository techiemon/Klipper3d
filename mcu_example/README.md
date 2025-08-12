This is a custom hardened Klipper heater control module that’s designed for:

MAX6675 thermocouple temperature sensing (usually for heater elements, like a hot air gun or heated chamber)

SSR PWM control of the heating element

Optional DHT22 sensor for ambient air temperature and humidity monitoring

Safety checks, logging, and runtime configurability

How it works
1. Core components
PID class – A self-contained PID controller with:

Real elapsed-time calculation for more stable control

Output clamping and anti-windup

Adjustable sample_time and limits

Max6675Heater class – Main heater manager that:

Validates the Klipper config section when loading

Looks up the target MCU and pins

Spawns three threads:

Control loop – Reads sensors, runs safety checks, adjusts heater power via PID

DHT22 loop (if enabled) – Reads air temp/humidity periodically

Logging loop – Aggregates data every 60s into a CSV-like buffer

2. Safety features
Temperature limits

Throttle at 220 °C (temporarily cuts heating)

Shutdown at 260 °C (hard-off)

Sensor fail timeout – If MAX6675 returns None for > 10 s, shutdown

Thermal runaway detection – If temp rises > 5 °C/s, shutdown

MCU command probing – Checks that max6675_read and ssr_pwm_set are actually supported; logs warnings if not

3. Control logic
If air temp is below target → full power

Otherwise, PID controls element temp to be air temp + offset

If no air sensor, falls back to element offset above last known temp

Uses 0–1 float for internal power, scaled to 16-bit PWM for MCU

4. G-code commands
Registered for use inside Klipper:

SET_HEATER_TEMP S=<temp> – Set target air temperature

SET_HEATER_POWER S=<0..1 or 0..100> – Force heater power

QUERY_HEATER – Show air temp, humidity, element temp, and power

TUNE_HEATER_PID KP=... KI=... KD=... – Adjust PID parameters live

EXPORT_HEATER_LOG – Dump aggregated log in CSV-like format

5. Shutdown handling
_shutdown_heater(reason):

Disables heater

Sends 0 PWM

Sets error message

Stops all threads

Automatic shutdown happens on:

Overtemperature

Sensor failure

Runaway detection

Fatal control loop error

Is it any good?
Strengths:

Thread-safe (locks around all shared state)

Robust config validation with meaningful errors

Fails safe — if something goes wrong, heater powers down

PID with anti-windup is well done

Logging for historical analysis

Graceful teardown when Klipper unloads it

Potential concerns:

Relies on mcu.send(...) calls — not standard for Klipper Python API; might need a custom MCU module/command

No rate-limiting for G-code commands (spamming SET_HEATER_POWER could block PID briefly)

MAX6675 read parsing assumes MCU text output — may fail if protocol changes

Adafruit_DHT reading can block for ~2 s — not a huge issue since it's in its own thread, but could cause timing drift

If your goal is safety + maintainability, this script is quite solid compared to many DIY heater control scripts I’ve seen.
However, it’s only compatible if your MCU firmware implements:

max6675_read

ssr_pwm_set

Otherwise it’ll start in "degraded mode" (logs warnings, doesn’t control heater).

If you want, I can make you a visual block diagram of how the threads, sensors, and heater control interact so you can see the whole flow at a glance. That would make the control logic very clear.



Raspberry Pi RP2040 microcontroller PIN assignments:

RP2040 pin suggestions (Pico)
Below is a sane, low-conflict mapping that keeps wiring simple and leaves common pins free. Adjust if you’ve already used some pins.

MAX6675 (SPI0)
SCK: GP18 (SPI0 SCK)
SO (DO/MISO): GP16 (SPI0 MISO)
CS: GP17 (any free GPIO works for CS)
SSR PWM (heater)
GP15 (RP2040 PWM is available on all GPIOs 0–29; GP15 is a good, free default)
Fan GPIO (on/off)
GP14 (simple digital output; use a MOSFET/driver for the fan + flyback diode)
Leave free/avoid
GP25 is Pico’s onboard LED; best to leave it alone.
Keep UART/I2C pins free if you need serial or I2C later (e.g., GP0/GP1 for UART0, GP4/GP5 for I2C0).
Important note about DHT22
In 
klipper/klippy/extras/max6675_heater.py
, DHT22 is read on the Klipper host via Adafruit_DHT (Raspberry Pi), not on the RP2040. So:

Wire DHT22 to the Raspberry Pi GPIO (BCM numbering), e.g. BCM4.
Set dht22_pin: 4 in 
klipper/config/max6675_heater.cfg
.
Don’t connect DHT22 to the RP2040 unless we move that sensor to MCU-side code.
Example config (update with your choices)
In 
klipper/config/max6675_heater.cfg
 within [max6675_heater my_heater]:

RP2040 pins
cs_pin: gpio17
ssr_pin: gpio15
fan_pin: gpio14
Raspberry Pi (host)
dht22_pin: 4 (BCM numbering)
If your Klipper pin syntax requires mcu prefix (depending on your firmware side), use pico:gpio17 etc.

Electrical tips
Fan: drive through a MOSFET/driver; do not power a fan directly from a GPIO. Add a flyback diode across the fan.
SSR: if it’s a logic SSR input, GPIO is fine; if it’s a mains SSR module, obey module input specs.
MAX6675: keep wires short and twisted for SCK/SO/GND if noise is an issue.
If you prefer SPI1 instead of SPI0:

SCK GP10, MISO GP12, CS any free pin (e.g., GP11 or GP9). Update wiring and config accordingly.
Want me to tailor pin choices around other peripherals you’ve already assigned? Tell me what’s occupied and I’ll adjust.

Thought Process


Great. Next steps to get this running smoothly:

Recommended Actions
Confirm MCU support: The Python calls fan_gpio_set on the MCU. If your RP2040 firmware doesn’t expose that yet, I can add a tiny handler to toggle your chosen fan_pin. Otherwise, tell me the existing command name/signature and I’ll align the host code.
Wire per plan:
MAX6675 (SPI0): GP18 SCK, GP16 MISO, GP17 CS.
SSR PWM: GP15.
Fan GPIO: GP14 through a MOSFET/driver with flyback diode.
DHT22 to Raspberry Pi GPIO (e.g., BCM4), not RP2040.
Update config 
klipper/config/max6675_heater.cfg
:
cs_pin: gpio17, ssr_pin: gpio15, fan_pin: gpio14
dht22_pin: 4
Optional: fan_active_high, cool_ratio
Test flow:
Start heating with 
SET_HEATER_TEMP
 or 
SET_HEATER_POWER
.
Verify fan turns on with heat.
After stopping heat, ensure fan runs until abs(element-air)/max(element,air) <= cool_ratio (10% default), then turns off.
Use 
QUERY_HEATER
 to watch temps.
If you want, I’ll add the MCU-side fan_gpio_set implementation for RP2040 and wire it to your selected fan_pin. Just say the word.

Summary: Fan control and post-cooling logic are implemented in 
klipper/klippy/extras/max6675_heater.py
. Confirm/implement the MCU fan_gpio_set command, wire as suggested, set config pins, and you’re set.
`