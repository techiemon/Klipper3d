# max6675_heater_hardened.py
# Hardened Klipper extension for controlling a heater with MAX6675 thermocouple and SSR PWM
# Improvements: thread-safety, config validation, MCU command checks, robust shutdown & logging.

import logging
import threading
import time
from typing import Optional

try:
    import Adafruit_DHT
except ImportError:
    Adafruit_DHT = None

class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0, sample_time=2.0, output_limits=(0.0, 1.0)):
        self.kp, self.ki, self.kd = float(kp), float(ki), float(kd)
        self.setpoint = float(setpoint)
        self.sample_time = float(sample_time)
        self.output_limits = (float(output_limits[0]), float(output_limits[1]))
        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = None
        self._last_output = 0.0

    def compute(self, value):
        """Compute PID output using real elapsed time between calls."""
        now = time.monotonic()
        value = float(value or 0.0)
        error = self.setpoint - value

        if self._last_time is None:
            # first run: assume sample_time to avoid division by tiny dt
            dt = self.sample_time
        else:
            dt = now - self._last_time
            # Protect against extremely small dt or zero
            if dt <= 0.0:
                dt = 1e-6

        # Integral with anti-windup: clamp integral to reasonable bounds based on output limits
        self._integral += error * dt
        # Derivative
        derivative = (error - self._last_error) / dt if dt > 0 else 0.0

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        # Clamp
        low, high = self.output_limits
        if output < low:
            output = low
            # Anti-windup: reduce integral when saturated
            self._integral *= 0.9
        elif output > high:
            output = high
            self._integral *= 0.9

        # Save state
        self._last_error = error
        self._last_time = now
        self._last_output = output
        return output

    def set_setpoint(self, setpoint):
        self.setpoint = float(setpoint)
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None


class Max6675Heater:
    def __init__(self, config):
        # Basic klipper plumbing
        self.printer = config.get_printer()
        self._log = logging.getLogger('max6675_heater')

        # Validate config fields, raise helpful errors
        try:
            mcu_name = config.get('mcu')
            if not mcu_name:
                raise ValueError("Missing required config: 'mcu'")
            self.mcu = self.printer.lookup_object(mcu_name)

            # name: keep original behavior but robust
            full_name = config.get_name()
            # handle if name contains spaces or single word
            try:
                self.name = full_name.split()[1]
            except Exception:
                self.name = full_name

            # Pins and params
            self.cs_pin = config.get('cs_pin', fallback=None)
            self.ssr_pin = config.get('ssr_pin', fallback=None)
            if self.cs_pin is None or self.ssr_pin is None:
                raise ValueError("cs_pin and ssr_pin must be specified")

            self.pwm_freq = config.getfloat('pwm_freq', 1.0)
            self.target_air = config.getfloat('target_air', 40.0)
            self.max_temp = config.getfloat('max_temp', 300.0)
            self.min_temp = config.getfloat('min_temp', 0.0)
            self.element_offset = config.getfloat('element_offset', 10.0)
            # Optional fan GPIO to cool elements during/after heat
            self.fan_pin = config.get('fan_pin', fallback=None)
            self.fan_active_high = bool(config.getboolean('fan_active_high', True))
            # DHT22 options
            # Mode 'host' uses Adafruit_DHT on the Raspberry Pi GPIO.
            # Mode 'mcu' uses MCU firmware commands to read DHT22 from RP2040.
            self.dht22_mode = config.get('dht22_mode', fallback='host').strip().lower()
            try:
                self.dht22_pin = config.getint('dht22_pin')
            except Exception:
                self.dht22_pin = None
            self.dht22_interval = config.getfloat('dht22_interval', 2.0)
            # MCU DHT22
            self.dht22_mcu_pin = config.get('dht22_mcu_pin', fallback=None)
            self.dht22_mcu_interval = config.getfloat('dht22_mcu_interval', self.dht22_interval)
            # PID
            self.pid_kp = config.getfloat('pid_kp', 2.0)
            self.pid_ki = config.getfloat('pid_ki', 0.1)
            self.pid_kd = config.getfloat('pid_kd', 1.0)
            sample_time = config.getfloat('pid_sample_time', 2.0)
        except Exception as e:
            # Re-raise as a more helpful error for Klipper loading
            self._log.exception("Configuration error during Max6675Heater init")
            raise

        # State & locks
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self.heater_power = 0.0
        self._heater_enabled = True
        self._error = None

        # Sensors & logs
        self._dht22_temp: Optional[float] = None
        self._dht22_humidity: Optional[float] = None
        self._element_temp: Optional[float] = None

        self._log_entries = []  # list of tuples
        self._log_accum = {'power': [], 'air': [], 'elem': [], 'hum': []}
        self._last_log_time = time.time()

        # PID controller
        self._pid = PID(self.pid_kp, self.pid_ki, self.pid_kd, setpoint=0.0, sample_time=sample_time)

        # MCU capability flags (checked below)
        self._mcu_ok = False
        self._max6675_ok = False
        self._ssr_ok = False
        self._fan_ok = False

        # Fan state & cooling control
        self._fan_on = False
        self._cooling_active = False
        # Ratio within which air and element are considered at ambient equilibrium
        self.cool_ratio = float(config.getfloat('cool_ratio', 0.10))

        # Register commands and events (try to keep same hooks as original)
        try:
            self._register_gcodes()
            self.printer.register_event_handler('idle_timeout', self._on_idle)
            self.printer.register_event_handler('klippy:ready', self._on_ready)

            # Print lifecycle handlers
            self.printer.register_event_handler('idle_timeout:printing', self._on_print_start)
            self.printer.register_event_handler('idle_timeout:idle', self._on_print_stop)
            self.printer.register_event_handler('gcode:PRINT_END', self._on_print_stop)
            self.printer.register_event_handler('gcode:PRINT_CANCEL', self._on_print_stop)
            self.printer.register_event_handler('gcode:PRINT_ABORT', self._on_print_stop)
        except Exception:
            self._log.exception("Failed to register gcode/event handlers")

        # Probe MCU commands non-fatally (do not raise on failure; we operate degraded)
        self._probe_mcu_commands()

        # Threads (only start those that are applicable)
        self._control_thread = threading.Thread(target=self._control_loop, name='max6675_control', daemon=True)
        self._control_thread.start()

        # Start DHT22 thread for either MCU mode or host mode
        start_dht = False
        if self.dht22_mode == 'mcu':
            if getattr(self, '_dht22_mcu_ok', False):
                start_dht = True
            else:
                self._log.info("MCU DHT22 not confirmed; skipping DHT22 thread.")
        else:
            if Adafruit_DHT is not None and self.dht22_pin is not None:
                start_dht = True
            else:
                if self.dht22_pin is None:
                    self._log.info("DHT22 pin not configured; skipping DHT22 thread.")
                else:
                    self._log.warning("Adafruit_DHT library not installed; skipping DHT22 support.")

        if start_dht:
            self._dht22_thread = threading.Thread(target=self._dht22_loop, name='max6675_dht22', daemon=True)
            self._dht22_thread.start()

        self._log_thread = threading.Thread(target=self._log_loop, name='max6675_logger', daemon=True)
        self._log_thread.start()

        self._log.info('Max6675Heater initialized (hardened)')

    def _register_gcodes(self):
        # Keep the original names for compatibility.
        self.printer.register_event_handler('gcode:SET_HEATER_TEMP', self.cmd_SET_HEATER_TEMP)
        self.printer.register_event_handler('gcode:SET_HEATER_POWER', self.cmd_SET_HEATER_POWER)
        self.printer.register_event_handler('gcode:QUERY_HEATER', self.cmd_QUERY_HEATER)
        self.printer.register_event_handler('gcode:TUNE_HEATER_PID', self.cmd_TUNE_HEATER_PID)
        # Export log
        self.printer.register_event_handler('gcode:EXPORT_HEATER_LOG', self.cmd_EXPORT_HEATER_LOG)

    def _on_ready(self):
        self._log.info('MAX6675 Heater ready')

    def _on_idle(self, eventtime):
        # When Klipper considers idle, make sure heater off
        self.set_power(0.0)

    def _probe_mcu_commands(self):
        """Check whether MCU supports the expected commands. This function will
        attempt lightweight calls and set capability flags. It never raises."""
        try:
            # Try a harmless max6675_read; handle exceptions and parse results
            resp = None
            try:
                resp = self.mcu.send('max6675_read')
            except Exception as e:
                self._log.debug("max6675_read probe failed: %s", e)
                resp = None

            if resp and isinstance(resp, str):
                for line in resp.splitlines():
                    if line.startswith('max6675_temp'):
                        self._max6675_ok = True
                        break

            # Try sending a small SSR pwm value (don't change real hardware if possible)
            try:
                self.mcu.send('ssr_pwm_set', value=0)
                self._ssr_ok = True
            except Exception as e:
                self._log.debug("ssr_pwm_set probe failed: %s", e)
                self._ssr_ok = False

            # Try probing fan gpio command if configured
            if self.fan_pin is not None:
                try:
                    # Probe fan command; assumes MCU already knows the pin
                    self.mcu.send('fan_gpio_set', value=0)
                    self._fan_ok = True
                except Exception as e:
                    self._log.debug("fan_gpio_set probe failed: %s", e)
                    self._fan_ok = False

            # Probe optional DHT22 on MCU
            if self.dht22_mode == 'mcu' and self.dht22_mcu_pin is not None:
                try:
                    # Configure MCU DHT22 with minimum interval (convert seconds to usec)
                    min_interval_us = int(max(0.5, float(self.dht22_mcu_interval)) * 1_000_000)
                    try:
                        self.mcu.send('dht22_config', pin=int(self.dht22_mcu_pin), min_interval_us=min_interval_us)
                    except Exception as ce:
                        self._log.debug("dht22_config failed: %s", ce)
                    resp = None
                    try:
                        resp = self.mcu.send('dht22_read')
                    except Exception as re:
                        self._log.debug("dht22_read probe failed: %s", re)
                    if resp and isinstance(resp, str):
                        for line in resp.splitlines():
                            line = line.strip()
                            if not line:
                                continue
                            if line.startswith('dht22'):
                                # Accept either float or *10 integer fields
                                self._dht22_mcu_ok = True
                                break
                except Exception as e:
                    self._dht22_mcu_ok = False
                    self._log.debug("MCU dht22 probe failed: %s", e)

            # Basic mcu reachability
            self._mcu_ok = True
        except Exception as e:
            self._log.warning("MCU probe failed; operating in degraded mode: %s", e)
            self._mcu_ok = False

        if not self._max6675_ok:
            self._log.warning("MAX6675 read not confirmed on MCU; get_element_temp() will return None until correct MCU impl is provided.")
        if not self._ssr_ok:
            self._log.warning("ssr_pwm_set not confirmed on MCU; SSR output may not work.")
        if self.fan_pin is not None and not self._fan_ok:
            self._log.warning("fan_gpio_set not confirmed on MCU; fan GPIO control disabled.")
        if self.dht22_mode == 'mcu' and self.dht22_mcu_pin is not None and not getattr(self, '_dht22_mcu_ok', False):
            self._log.warning("MCU DHT22 not confirmed; falling back to no air sensor.")

    def _send_fan_gpio(self, on: bool):
        """Set fan GPIO state. Honors active-high setting. No-op if not available."""
        desired = bool(on)
        # Map logical on/off to electrical level
        level = 1 if (desired == self.fan_active_high) else 0
        if not self._mcu_ok or not self._fan_ok:
            return
        try:
            self.mcu.send('fan_gpio_set', value=int(level))
            self._fan_on = desired
        except Exception as e:
            self._log.debug("Failed to set fan_gpio_set on MCU: %s", e)

    def _dht22_loop(self):
        """Poll DHT22 in its own thread; does not block control loop.
        Supports host mode (Adafruit_DHT on Pi) and MCU mode (dht22_read).
        """
        if self.dht22_mode == 'mcu':
            if not getattr(self, '_dht22_mcu_ok', False):
                self._log.info('MCU DHT22 not available; skipping DHT22 loop')
                return
            interval = max(0.5, float(self.dht22_mcu_interval))
            while not self._stop_event.is_set():
                try:
                    resp = self.mcu.send('dht22_read')
                    # Expected line: "dht22 temp=xx.x hum=yy.y"
                    t, h = None, None
                    if resp:
                        for line in resp.splitlines():
                            line = line.strip()
                            if line.startswith('dht22'):
                                parts = line.split()
                                for p in parts:
                                    if p.startswith('temp='):
                                        try:
                                            t = float(p.split('=',1)[1])
                                        except Exception:
                                            pass
                                    if p.startswith('temp10='):
                                        try:
                                            t = float(int(p.split('=',1)[1]) / 10.0)
                                        except Exception:
                                            pass
                                    if p.startswith('hum='):
                                        try:
                                            h = float(p.split('=',1)[1])
                                        except Exception:
                                            pass
                                    if p.startswith('hum10='):
                                        try:
                                            h = float(int(p.split('=',1)[1]) / 10.0)
                                        except Exception:
                                            pass
                                break
                    with self._lock:
                        if t is not None:
                            self._dht22_temp = t
                            self._log_accum['air'].append(t)
                        if h is not None:
                            self._dht22_humidity = h
                            self._log_accum['hum'].append(h)
                except Exception as e:
                    self._log.debug('Error reading MCU DHT22: %s', e)
                self._stop_event.wait(interval)
            return

        # Host mode (default)
        if Adafruit_DHT is None:
            self._log.warning('Adafruit_DHT not installed, DHT22/AM2302 not available')
            return
        if self.dht22_pin is None:
            self._log.info("DHT22 pin not configured; skipping DHT22 loop")
            return

        while not self._stop_event.is_set():
            try:
                humidity, temp = Adafruit_DHT.read_retry(Adafruit_DHT.DHT22, self.dht22_pin)
                with self._lock:
                    if temp is not None:
                        self._dht22_temp = float(temp)
                        self._log_accum['air'].append(self._dht22_temp)
                    if humidity is not None:
                        self._dht22_humidity = float(humidity)
                        self._log_accum['hum'].append(self._dht22_humidity)
                time.sleep(self.dht22_interval)
            except Exception as e:
                self._log.exception("Error in DHT22 loop: %s", e)
                time.sleep(max(1.0, self.dht22_interval))

    def _send_max6675_read(self):
        """Query MCU for MAX6675 reading. Returns float in C or None."""
        if not self._mcu_ok or not self._max6675_ok:
            return None
        try:
            resp = self.mcu.send('max6675_read')
        except Exception as e:
            self._log.debug("mcu.send('max6675_read') failed: %s", e)
            return None

        if not resp:
            return None

        # Parse response robustly
        try:
            for line in resp.splitlines():
                line = line.strip()
                if not line:
                    continue
                if line.startswith('max6675_temp'):
                    # example: "max6675_temp temp=512"
                    parts = line.split()
                    for p in parts:
                        if p.startswith('temp='):
                            # safe parse
                            try:
                                val = float(p.split('=', 1)[1])
                                # original code multiplied int by 0.25; accept either already-scaled floats
                                # If value looks large (like integer ticks), apply *0.25; otherwise return directly.
                                if val > 1000:  # heuristic tick count
                                    return val * 0.25
                                return val
                            except Exception:
                                self._log.debug("Failed parse temp value from: %s", p)
                                return None
        except Exception as e:
            self._log.debug("Failed to parse MAX6675 response: %s", e)
        return None

    def _send_ssr_pwm_set(self, value: int):
        """Set SSR PWM via MCU. value: 0..65535"""
        if not self._mcu_ok or not self._ssr_ok:
            self._log.debug("Skipping SSR PWM set; MCU/SSR not available.")
            return
        try:
            # clamp value defensively
            v = int(max(0, min(65535, int(value))))
            self.mcu.send('ssr_pwm_set', value=v)
        except Exception as e:
            self._log.warning("Failed to set ssr_pwm_set on MCU: %s", e)

    def get_air_temp(self) -> Optional[float]:
        with self._lock:
            val = self._dht22_temp
            # accumulators are updated in DHT thread already
            return None if val is None else float(val)

    def get_element_temp(self) -> Optional[float]:
        temp = self._send_max6675_read()
        with self._lock:
            self._element_temp = temp
            if temp is not None:
                self._log_accum['elem'].append(float(temp))
        return temp

    def set_power(self, power):
        """Set heater power as 0.0..1.0 (floats); thread-safe and non-blocking."""
        try:
            p = float(power)
        except Exception:
            p = 0.0
        p = max(0.0, min(1.0, p))
        with self._lock:
            if not getattr(self, '_heater_enabled', True):
                p = 0.0
            self.heater_power = p
            self._log_accum['power'].append(p)
        # send PWM outside of the lock to avoid blocking other operations
        pwm_value = int(p * 65535)
        self._send_ssr_pwm_set(pwm_value)
        # Manage fan based on heater power transitions
        try:
            if p > 0.0:
                # Heating started/continuing: ensure fan on and cancel post-cooling state
                if not self._fan_on:
                    self._send_fan_gpio(True)
                self._cooling_active = False
            else:
                # Heating stopped: if we have both sensors, start cooling
                air = self.get_air_temp()
                elem = self.get_element_temp()
                if (air is not None) and (elem is not None):
                    # Begin post-cooling until temps converge
                    self._cooling_active = True
                    if not self._fan_on:
                        self._send_fan_gpio(True)
                else:
                    # No air sensor -> stop fan immediately
                    if self._fan_on:
                        self._send_fan_gpio(False)
                        self._cooling_active = False
        except Exception:
            # Fan control should never disrupt heater control
            pass

    def _control_loop(self):
        """Main control loop. Runs PID and safety checks at PID.sample_time intervals."""
        throttle_temp = 220.0  # Celsius (throttle - reduce power)
        shutdown_temp = 260.0  # Celsius (hard shutdown)
        runaway_rate = 5.0     # degC/sec
        sensor_fail_timeout = 10.0  # seconds
        last_elem_ok = time.time()
        last_elem_temp = None
        last_elem_time = None

        while not self._stop_event.is_set():
            start = time.time()
            try:
                air_temp = self.get_air_temp()
                element_temp = self.get_element_temp()

                now = time.time()
                # Sensor fail detection
                if element_temp is None:
                    if now - last_elem_ok > sensor_fail_timeout:
                        self._shutdown_heater('MAX6675 sensor failure: no reading for %.1f s' % sensor_fail_timeout)
                        break
                else:
                    last_elem_ok = now

                # Overtemp checks
                if element_temp is not None:
                    if element_temp > shutdown_temp:
                        self._shutdown_heater('Element temperature exceeded %dC (%.2fC)' % (shutdown_temp, element_temp))
                        break
                    if element_temp > throttle_temp:
                        # throttle: temporarily stop heating and re-evaluate
                        self._log.warning("Element temp %.2fC > throttle %.2fC. Throttling power.", element_temp, throttle_temp)
                        self.set_power(0.0)
                        # Sleep a short time then continue without PID update to avoid integral wind-up
                        time.sleep(1.0)
                        continue

                # Runaway detection
                if last_elem_temp is not None and element_temp is not None:
                    dt = now - last_elem_time if last_elem_time else 1.0
                    if dt > 0:
                        rate = (element_temp - last_elem_temp) / dt
                        if rate > runaway_rate:
                            self._shutdown_heater('Thermal runaway detected: rate %.2f C/s' % rate)
                            break
                last_elem_temp = element_temp
                last_elem_time = now

                # Control logic:
                # If air sensor present and air < target_air -> full power.
                # Otherwise run PID on element temp towards (air + offset)
                if (air_temp is not None) and (air_temp < self.target_air):
                    # full power until ambient warmed
                    self.set_power(1.0)
                else:
                    # compute target for element; if no air, use element_offset relative to previous element temp or target_air
                    if air_temp is not None:
                        target_elem = air_temp + self.element_offset
                    else:
                        # fallback: if no air reading, keep the setpoint at previous or use element_offset above min_temp
                        prev_elem = element_temp if element_temp is not None else self.min_temp
                        target_elem = prev_elem + self.element_offset
                    # update PID setpoint thread-safely
                    with self._lock:
                        self._pid.set_setpoint(target_elem)
                    # compute using last known element temp (0 if None)
                    power = self._pid.compute(element_temp or 0.0)
                    self.set_power(max(0.0, min(1.0, power)))

                # Post-cooling logic: if heater power is zero and cooling is active, keep fan on
                # until air and element temps are within cool_ratio of each other.
                if self.heater_power == 0.0 and self._cooling_active:
                    if (air_temp is not None) and (element_temp is not None):
                        high = max(air_temp, element_temp)
                        if high > 0:
                            diff = abs(element_temp - air_temp) / high
                            if diff <= self.cool_ratio:
                                # Temps have converged; stop fan and end cooling
                                if self._fan_on:
                                    self._send_fan_gpio(False)
                                self._cooling_active = False
                        # Ensure fan remains on during cooling
                        if not self._fan_on:
                            self._send_fan_gpio(True)
                    else:
                        # Missing sensors -> cancel cooling and stop fan
                        if self._fan_on:
                            self._send_fan_gpio(False)
                        self._cooling_active = False
            except Exception as e:
                self._log.exception("Exception in control loop: %s", e)
                # On unexpected error, do not crash the process; put heater into safe-off state
                self._shutdown_heater("Control loop fatal error: %s" % e)
                break

            # Wait until next sample period but respond to stop_event quickly
            sample = max(0.1, float(self._pid.sample_time))
            elapsed = time.time() - start
            wait = max(0.0, sample - elapsed)
            self._stop_event.wait(wait)

    def _log_loop(self):
        """Periodic logger that aggregates values every 60 seconds (configurable if desired)."""
        while not self._stop_event.is_set():
            try:
                now = time.time()
                if now - self._last_log_time >= 60.0:
                    with self._lock:
                        def avg(arr):
                            return sum(arr) / len(arr) if arr else None
                        entry = (
                            int(now),
                            avg(self._log_accum['power']),
                            avg(self._log_accum['air']),
                            avg(self._log_accum['elem']),
                            avg(self._log_accum['hum']),
                        )
                        self._log_entries.append(entry)
                        # clear accumulators
                        for k in self._log_accum:
                            self._log_accum[k].clear()
                        self._last_log_time = now
                # wait a bit but wake earlier if stopping
                self._stop_event.wait(5.0)
            except Exception:
                self._log.exception("Exception in log loop")
                # small sleep to avoid spin if logging consistently fails
                self._stop_event.wait(1.0)

    # GCODE handlers (expected to be called by Klipper's event system)
    def cmd_SET_HEATER_TEMP(self, gcmd):
        temp = gcmd.get_float('S', None)
        if temp is not None:
            with self._lock:
                self.target_air = float(temp)
                self._heater_enabled = True
            gcmd.respond_info('Target air temperature set to %.2fC' % temp)

    def cmd_SET_HEATER_POWER(self, gcmd):
        power = gcmd.get_float('S', None)
        if power is not None:
            # Accept either 0..1 or 0..100 (common G-code confusion); auto-detect
            if power > 1.5:
                power = power / 100.0
            self.set_power(power)
            gcmd.respond_info('Heater power forced to %.2f%%' % (self.heater_power * 100.0))

    def cmd_QUERY_HEATER(self, gcmd):
        with self._lock:
            air_temp = self._dht22_temp if self._dht22_temp is not None else None
            element_temp = self._element_temp if self._element_temp is not None else None
            humidity = self._dht22_humidity if self._dht22_humidity is not None else None
            power_pct = self.heater_power * 100.0
            msg = 'Air temp: %s, Humidity: %s, Element temp: %s, Power: %.2f%%' % (
                ('%.2fC' % air_temp) if air_temp is not None else 'N/A',
                ('%.1f%%' % humidity) if humidity is not None else 'N/A',
                ('%.2fC' % element_temp) if element_temp is not None else 'N/A',
                power_pct)
            if self._error:
                msg += '\nERROR: ' + self._error
        gcmd.respond_info(msg)

    def cmd_TUNE_HEATER_PID(self, gcmd):
        kp = gcmd.get_float('KP', None)
        ki = gcmd.get_float('KI', None)
        kd = gcmd.get_float('KD', None)
        changed = False
        with self._lock:
            if kp is not None:
                self._pid.kp = float(kp)
                changed = True
            if ki is not None:
                self._pid.ki = float(ki)
                changed = True
            if kd is not None:
                self._pid.kd = float(kd)
                changed = True
        gcmd.respond_info('PID updated: Kp=%.3f Ki=%.3f Kd=%.3f' % (self._pid.kp, self._pid.ki, self._pid.kd))

    def cmd_EXPORT_HEATER_LOG(self, gcmd):
        """Return CSV-like aggregated log data. Handles missing values gracefully."""
        lines = ["timestamp,avg_power,avg_air,avg_element,avg_humidity"]
        with self._lock:
            for entry in self._log_entries:
                ts, power, air, elem, hum = entry
                def fmt(v, f):
                    return ('%.3f' % v) if v is not None else ''
                lines.append("%d,%s,%s,%s,%s" % (ts, fmt(power, '%.3f'), fmt(air, '%.2f'), fmt(elem, '%.2f'), fmt(hum, '%.1f')))
            if self._error:
                lines.append(f"ERROR: {self._error}")
        gcmd.respond_info("\n".join(lines))

    def _shutdown_heater(self, reason):
        """Put the heater into safe state and stop background threads.
        This method is re-entrant and thread-safe.
        """
        with self._lock:
            self._heater_enabled = False
            self.heater_power = 0.0
            self._error = reason
        # attempt to send a final 0 pwm
        try:
            self._send_ssr_pwm_set(0)
        except Exception:
            pass
        self._log.warning('Heater shutdown: %s', reason)
        # stop threads
        self._stop_event.set()

    def _on_print_stop(self, *args, **kwargs):
        with self._lock:
            self._heater_enabled = False
        self.set_power(0.0)

    def _on_print_start(self, *args, **kwargs):
        # Optionally re-enable heater here
        pass

    # Expose a graceful teardown if Klipper unloads the module
    def shutdown(self):
        self._log.info("Shutdown called on Max6675Heater")
        self._stop_event.set()
        # Join threads (best-effort, threads are daemon but join quickly)
        try:
            if hasattr(self, '_control_thread'):
                self._control_thread.join(timeout=2.0)
            if hasattr(self, '_dht22_thread'):
                self._dht22_thread.join(timeout=2.0)
            if hasattr(self, '_log_thread'):
                self._log_thread.join(timeout=2.0)
        except Exception:
            self._log.debug("Exception while joining threads during shutdown", exc_info=True)

def load_config(config):
    return Max6675Heater(config)
