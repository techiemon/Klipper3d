# max6675_heater_hardened.py
# Hardened Klipper extension for controlling a heater with MAX6675 thermocouple and SSR PWM
# Improvements: thread-safety, config validation, MCU command checks, robust shutdown & logging.

import logging
import threading
import time
from typing import Optional
from . import bus
import mcu


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
            # Resolve the MCU by alias using Klipper's helper
            self.mcu = mcu.get_printer_mcu(self.printer, mcu_name)

            # name: keep original behavior but robust
            full_name = config.get_name()
            # handle if name contains spaces or single word
            try:
                self.name = full_name.split()[1]
            except Exception:
                self.name = full_name

            # Pins and params
            # Note: We no longer require a local MAX6675 CS pin because we read
            # the element temperature from Klipper's built-in temperature_sensor.
            # Keep SSR pin optional; SSR control will be disabled if not provided.
            self.cs_pin = config.get('cs_pin', None)
            self.ssr_pin = config.get('ssr_pin', None)

            self.pwm_freq = config.getfloat('pwm_freq', 1.0)
            self.target_air = config.getfloat('target_air', 40.0)
            self.max_temp = config.getfloat('max_temp', 300.0)
            self.min_temp = config.getfloat('min_temp', 0.0)
            self.element_offset = config.getfloat('element_offset', 10.0)
            # Optional fan GPIO to cool elements during/after heat
            self.fan_pin = config.get('fan_pin', None)
            self.fan_active_high = bool(config.getboolean('fan_active_high', True))
            # Names of Klipper [output_pin] objects to drive via SET_PIN
            self.ssr_output = config.get('ssr_output', 'ssr')
            self.fan_output = config.get('fan_output', 'element_fan')
            # Optional ambient sensor provided by another module (e.g. aht10/aht20)
            # Example: ambient_sensor: "aht10 my_ambient"
            self.ambient_sensor = config.get('ambient_sensor', None)
            # Optional element temperature sensor using Klipper's built-in sensor
            # Example: element_sensor: "temperature_sensor element_temp"
            self.element_sensor = config.get('element_sensor', None)
            # Optional MCU internal temperature sensor (temperature_mcu)
            # Example: mcu_temp_sensor: "temperature_mcu my_mcu"
            self.mcu_temp_sensor = config.get('mcu_temp_sensor', None)
            # PID
            self.pid_kp = config.getfloat('pid_kp', 2.0)
            self.pid_ki = config.getfloat('pid_ki', 0.1)
            self.pid_kd = config.getfloat('pid_kd', 1.0)
            sample_time = config.getfloat('pid_sample_time', 2.0)
            # We no longer create an SPI device here. The element temperature
            # will be sourced from the stock temperature_sensor module.
        except Exception as e:
            # Re-raise as a more helpful error for Klipper loading
            self._log.exception("Configuration error during Max6675Heater init")
            raise

        # State & locks
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self.heater_power = 0.0
        # Safety: heater starts disabled and remains off until explicitly enabled by user
        self._heater_enabled = False
        self._error = None

        # Sensors & logs
        self._aht20_temp: Optional[float] = None
        self._aht20_humidity: Optional[float] = None
        self._element_temp: Optional[float] = None
        self._mcu_temp: Optional[float] = None

        self._log_entries = []  # list of tuples
        self._log_accum = {'power': [], 'air': [], 'elem': [], 'hum': [], 'mcu': []}
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
        # Keep probing only for SSR/FAN capabilities; drop MAX6675 custom probe.
        # Defer probing until klippy:ready to ensure all [output_pin] objects are registered
        # and gcode interfaces are fully initialized. Probing too early can cause false
        # negatives and leave control flags disabled for the session.
        # We'll invoke _probe_mcu_commands() in _on_ready().

        # Defer sensor attachment until klippy:ready to avoid race with heaters init
        self._sensors_attached = False

        # Threads (defer start until klippy:ready to let sensors initialize)
        self._control_thread = None
        self._log_thread = None

        self._log.info('Max6675Heater initialized (hardened)')

    def get_mcu_temp(self) -> Optional[float]:
        """Poll MCU temperature sensor for current temperature."""
        if self._mcu_temp_sensor_obj is None:
            return None
        try:
            # Get current temperature from sensor object
            status = self._mcu_temp_sensor_obj.get_status(None)
            temp = status.get('temperature', None)
            if temp is not None:
                temp = float(temp)
                # Update internal state for logging
                with self._lock:
                    self._mcu_temp = temp
                    self._log_accum['mcu'].append(temp)
                return temp
        except Exception as e:
            self._log.debug("Failed to read MCU temperature: %s", e)
        return None

    def get_humidity(self) -> Optional[float]:
        """Get humidity from ambient sensor if available."""
        if self._ambient_sensor_obj is None:
            return None
        try:
            # Try to get humidity from sensor status
            status = self._ambient_sensor_obj.get_status(None)
            humidity = status.get('humidity', None)
            if humidity is not None:
                humidity = float(humidity)
                # Update internal state for logging
                with self._lock:
                    self._aht20_humidity = humidity
                    self._log_accum['hum'].append(humidity)
                return humidity
        except Exception as e:
            self._log.debug("Failed to read humidity: %s", e)
        return None

    def _register_gcodes(self):
        # Register commands with Klipper's gcode object so they're available to users.
        gcode = self.printer.lookup_object('gcode')
        # Save handle for SET_PIN control
        self._gcode = gcode
        gcode.register_command('SET_HEATER_TEMP', self.cmd_SET_HEATER_TEMP,
                               desc='Set target temperatures for MAX6675 heater control')
        gcode.register_command('SET_HEATER_POWER', self.cmd_SET_HEATER_POWER,
                               desc='Manually set heater power (0.0-1.0) for MAX6675 heater')
        gcode.register_command('QUERY_HEATER', self.cmd_QUERY_HEATER,
                               desc='Query current heater, air, and MCU temperatures and state')
        gcode.register_command('TUNE_HEATER_PID', self.cmd_TUNE_HEATER_PID,
                               desc='Run PID autotune for MAX6675 heater')
        gcode.register_command('EXPORT_HEATER_LOG', self.cmd_EXPORT_HEATER_LOG,
                               desc='Export recent heater telemetry log as CSV via M118 responses')
        # Safety control
        gcode.register_command('ENABLE_HEATER', self.cmd_ENABLE_HEATER,
                               desc='Enable MAX6675 heater output (does not set power)')
        gcode.register_command('DISABLE_HEATER', self.cmd_DISABLE_HEATER,
                               desc='Disable MAX6675 heater output and force power to 0')

    def _on_ready(self):
        # Probe outputs now that all config objects are loaded
        try:
            self._probe_mcu_commands()
        except Exception:
            self._log.debug('Exception during output probe on ready', exc_info=True)

        # Attach sensors (only once)
        try:
            if not getattr(self, '_sensors_attached', False):
                self._attach_sensors()
                self._sensors_attached = True
        except Exception:
            self._log.exception('Failed to attach sensors on ready')

        # Start threads once Klippy reports ready so sensors have begun reporting
        try:
            if self._control_thread is None or not self._control_thread.is_alive():
                self._control_thread = threading.Thread(target=self._control_loop, name='max6675_control', daemon=True)
                self._control_thread.start()
            if self._log_thread is None or not self._log_thread.is_alive():
                self._log_thread = threading.Thread(target=self._log_loop, name='max6675_logger', daemon=True)
                self._log_thread.start()
        except Exception:
            self._log.exception('Failed to start background threads on ready')
        self._log.info('MAX6675 Heater ready')

    def _attach_sensors(self):
        # Store sensor object references for polling instead of callbacks
        self._ambient_sensor_obj = None
        self._element_sensor_obj = None
        self._mcu_temp_sensor_obj = None
        
        # Optional: attach to ambient sensor (e.g. AHT10/AHT20) on MCU I2C
        if self.ambient_sensor:
            try:
                self._log.info("Attempting to attach ambient sensor: %s", self.ambient_sensor)
                amb = self.printer.lookup_object(self.ambient_sensor)
                self._log.info("Found ambient sensor object: %s", type(amb).__name__)
                if hasattr(amb, 'setup_minmax'):
                    amb.setup_minmax(self.min_temp, self.max_temp)
                    self._log.debug("Set minmax on ambient sensor")
                # Store reference for polling
                self._ambient_sensor_obj = amb
                self._log.info("Successfully attached ambient sensor: %s", self.ambient_sensor)
            except Exception:
                self._log.exception("Failed to attach ambient sensor '%s'", self.ambient_sensor)

        # Optional: attach to element temperature sensor (built-in temperature_sensor)
        if self.element_sensor:
            try:
                self._log.info("Attempting to attach element sensor: %s", self.element_sensor)
                elem = self.printer.lookup_object(self.element_sensor)
                self._log.info("Found element sensor object: %s", type(elem).__name__)
                if hasattr(elem, 'setup_minmax'):
                    elem.setup_minmax(self.min_temp, self.max_temp)
                    self._log.debug("Set minmax on element sensor")
                # Store reference for polling
                self._element_sensor_obj = elem
                self._log.info("Successfully attached element sensor: %s", self.element_sensor)
            except Exception:
                self._log.exception("Failed to attach element sensor '%s'", self.element_sensor)

        # Optional: attach to MCU internal temperature sensor (temperature_mcu)
        if self.mcu_temp_sensor:
            try:
                self._log.info("Attempting to attach MCU temp sensor: %s", self.mcu_temp_sensor)
                mts = self.printer.lookup_object(self.mcu_temp_sensor)
                self._log.info("Found MCU temp sensor object: %s", type(mts).__name__)
                if hasattr(mts, 'setup_minmax'):
                    mts.setup_minmax(self.min_temp, self.max_temp)
                    self._log.debug("Set minmax on MCU temp sensor")
                # Store reference for polling
                self._mcu_temp_sensor_obj = mts
                self._log.info("Successfully attached MCU temp sensor: %s", self.mcu_temp_sensor)
            except Exception:
                self._log.exception("Failed to attach MCU temp sensor '%s'", self.mcu_temp_sensor)

    def _on_idle(self, eventtime):
        # When Klipper considers idle, make sure heater off
        self.set_power(0.0)

    def _probe_mcu_commands(self):
        """Probe availability of configured [output_pin] objects via SET_PIN.
        Sets _mcu_ok, _ssr_ok, _fan_ok flags; never raises."""
        try:
            # Probe SSR output by attempting a harmless SET_PIN to 0.0
            try:
                self._gcode.run_script(f"SET_PIN PIN={self.ssr_output} VALUE=0")
                self._ssr_ok = True
            except Exception as e:
                self._log.debug("SET_PIN probe for ssr_output '%s' failed: %s", self.ssr_output, e)
                self._ssr_ok = False

            # Probe fan output (optional)
            self._fan_ok = False
            if self.fan_output:
                try:
                    self._gcode.run_script(f"SET_PIN PIN={self.fan_output} VALUE=0")
                    self._fan_ok = True
                except Exception as e:
                    self._log.debug("SET_PIN probe for fan_output '%s' failed: %s", self.fan_output, e)

            # MCU/gcode plumbing reachable
            self._mcu_ok = True
        except Exception as e:
            self._log.warning("MCU probe failed; operating in degraded mode: %s", e)
            self._mcu_ok = False
        if not self._ssr_ok:
            self._log.warning("SSR output '%s' not available; heater power disabled.", self.ssr_output)
        if not self._fan_ok and self.fan_output:
            self._log.warning("Fan output '%s' not available; fan control disabled.", self.fan_output)

    def _send_fan_gpio(self, on: bool):
        """Set fan via SET_PIN on configured [output_pin]. Honors active-high."""
        desired = bool(on)
        level = 1.0 if (desired == self.fan_active_high) else 0.0
        if not self._mcu_ok or not self._fan_ok or not self.fan_output:
            return
        try:
            # Use run_script_from_command to avoid blocking the calling thread
            cmd = f"SET_PIN PIN={self.fan_output} VALUE={level:.3f}"
            self._log.debug("Sending fan command: %s", cmd)
            self._gcode.run_script_from_command(cmd)
            self._fan_on = desired
        except Exception as e:
            self._log.debug("Failed to SET_PIN for fan_output '%s': %s", self.fan_output, e)



    def _send_ssr_pwm_set(self, value: int):
        """Set SSR power via SET_PIN (value 0..65535 scaled to 0..1)."""
        if not self._mcu_ok or not self._ssr_ok:
            self._log.debug("Skipping SSR SET_PIN; SSR output unavailable.")
            return
        try:
            v = int(max(0, min(65535, int(value))))
            val = v / 65535.0
            # Use run_script_from_command to avoid blocking the calling thread
            # This prevents QUERY_HEATER from hanging when SET_HEATER_POWER is called
            cmd = f"SET_PIN PIN={self.ssr_output} VALUE={val:.5f}"
            self._log.debug("Sending SSR command: %s", cmd)
            self._gcode.run_script_from_command(cmd)
        except Exception as e:
            self._log.warning("Failed to SET_PIN for ssr_output '%s': %s", self.ssr_output, e)

    

    def get_air_temp(self) -> Optional[float]:
        """Poll ambient sensor for current temperature."""
        if self._ambient_sensor_obj is None:
            return None
        try:
            # Get current temperature from sensor object
            status = self._ambient_sensor_obj.get_status(None)
            temp = status.get('temperature', None)
            if temp is not None:
                temp = float(temp)
                # Update internal state for logging
                with self._lock:
                    self._aht20_temp = temp
                    self._log_accum['air'].append(temp)
                return temp
        except Exception as e:
            self._log.debug("Failed to read ambient temperature: %s", e)
        return None

    def get_element_temp(self) -> Optional[float]:
        """Poll element sensor for current temperature."""
        if self._element_sensor_obj is None:
            return None
        try:
            # Get current temperature from sensor object
            status = self._element_sensor_obj.get_status(None)
            temp = status.get('temperature', None)
            if temp is not None:
                temp = float(temp)
                # Update internal state for logging
                with self._lock:
                    self._element_temp = temp
                    self._log_accum['elem'].append(temp)
                return temp
        except Exception as e:
            self._log.debug("Failed to read element temperature: %s", e)
        return None

    def set_power(self, power):
        """Set heater power as 0.0..1.0 (floats); thread-safe and non-blocking."""
        try:
            p = float(power)
        except Exception:
            p = 0.0
        p = max(0.0, min(1.0, p))
        with self._lock:
            if not getattr(self, '_heater_enabled', False):
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
        sensor_fail_timeout = 30.0  # seconds (allow time for sensor to start reporting)
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

                # Safety-first control: never raise power automatically.
                # Only reflect current manual power if heater is enabled and element sensor is valid;
                # otherwise force power to 0.
                if (not self._heater_enabled) or (element_temp is None):
                    if self.heater_power != 0.0:
                        self.set_power(0.0)
                # else: leave power as previously set by SET_HEATER_POWER

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
                            avg(self._log_accum['mcu']),
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
        """Set target temperatures/offsets (does not enable or apply power)."""
        air = gcmd.get_float('AIR', None)
        offset = gcmd.get_float('OFFSET', None)
        s_val = gcmd.get_float('S', None)  # legacy: treat as AIR if provided
        changed = False
        with self._lock:
            if s_val is not None:
                self.target_air = float(s_val)
                changed = True
            if air is not None:
                self.target_air = float(air)
                changed = True
            if offset is not None:
                self.element_offset = float(offset)
                changed = True
        if changed:
            gcmd.respond_info('Targets updated: target_air=%.2fC, element_offset=%.2fC' % (
                float(getattr(self, 'target_air', 0.0)), float(getattr(self, 'element_offset', 0.0))))
        else:
            gcmd.respond_info('No changes. Use AIR=<C> and/or OFFSET=<C> (S=<C> sets AIR).')

    def cmd_SET_HEATER_POWER(self, gcmd):
        """Set manual heater power. Requires ENABLE_HEATER to take effect."""
        power = gcmd.get_float('POWER', None)
        if power is None:
            power = gcmd.get_float('S', None)
        if power is None:
            gcmd.respond_info('Usage: SET_HEATER_POWER POWER=<0..1 or 0..100%> (requires ENABLE_HEATER)')
            return
        # Accept percentages if POWER>1
        if power > 1.0:
            power = power / 100.0
        if not self._heater_enabled:
            # Do not apply power when disabled
            self.set_power(0.0)
            gcmd.respond_info('Heater is DISABLED. Run ENABLE_HEATER first; power remains 0%.')
            return
        self.set_power(power)
        gcmd.respond_info('Heater power set to %.2f%%' % (self.heater_power * 100.0))

    def cmd_QUERY_HEATER(self, gcmd):
        """Report current temps, power, enabled state, and any error."""
        # Poll sensors for current readings
        air_temp = self.get_air_temp()
        element_temp = self.get_element_temp()
        mcu_temp = self.get_mcu_temp()
        humidity = self.get_humidity()
        
        with self._lock:
            power_pct = self.heater_power * 100.0
            enabled = self._heater_enabled
            ssr_ok = getattr(self, '_ssr_ok', False)
            fan_on = getattr(self, '_fan_on', False)
            error = self._error
            
        msg = (
            'Enabled: %s, Power: %.2f%%\nAir: %s, Element: %s, MCU: %s, Humidity: %s\nSSR_OK: %s, Fan: %s' % (
                'YES' if enabled else 'NO',
                power_pct,
                ('%.2fC' % air_temp) if air_temp is not None else 'N/A',
                ('%.2fC' % element_temp) if element_temp is not None else 'N/A',
                ('%.2fC' % mcu_temp) if mcu_temp is not None else 'N/A',
                ('%.1f%%' % humidity) if humidity is not None else 'N/A',
                'YES' if ssr_ok else 'NO',
                'ON' if fan_on else 'OFF')
        )
        if error:
            msg += '\nERROR: ' + error
        gcmd.respond_info(msg)

    def cmd_ENABLE_HEATER(self, gcmd):
        """Explicitly enable heater output gating (does not set power)."""
        with self._lock:
            self._heater_enabled = True
            # Keep SSR OFF until a positive power is explicitly set
            self.heater_power = 0.0
        # Ensure hardware is off until user sends SET_HEATER_POWER
        self._send_ssr_pwm_set(0)
        gcmd.respond_info('Heater ENABLED. Power is 0%%. Use SET_HEATER_POWER to apply power.')

    def cmd_DISABLE_HEATER(self, gcmd):
        """Disable heater output and force power to zero immediately."""
        with self._lock:
            self._heater_enabled = False
        self.set_power(0.0)
        gcmd.respond_info('Heater DISABLED. Power forced to 0%.')

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
        lines = ["timestamp,avg_power,avg_air,avg_mcu,avg_element,avg_humidity"]
        with self._lock:
            for entry in self._log_entries:
                ts, power, air, mcu, elem, hum = entry
                def fmt(v, f):
                    return ('%.3f' % v) if v is not None else ''
                lines.append("%d,%s,%s,%s,%s,%s" % (
                    ts,
                    fmt(power, '%.3f'),
                    fmt(air, '%.2f'),
                    fmt(mcu, '%.2f'),
                    fmt(elem, '%.2f'),
                    fmt(hum, '%.1f')
                ))
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
            if hasattr(self, '_log_thread'):
                self._log_thread.join(timeout=2.0)
        except Exception:
            self._log.debug("Exception while joining threads during shutdown", exc_info=True)

def load_config(config):
    return Max6675Heater(config)
