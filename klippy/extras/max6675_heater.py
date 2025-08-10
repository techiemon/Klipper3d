# max6675_heater.py
# Klipper extension for controlling a heater with MAX6675 thermocouple and SSR PWM on a secondary MCU (RP2040), with DHT22 air sensor and PID

import logging
import threading
import time
try:
    import Adafruit_DHT
except ImportError:
    Adafruit_DHT = None

class PID:
    def __init__(self, kp, ki, kd, setpoint=0, sample_time=2.0, output_limits=(0, 1)):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint
        self.sample_time = sample_time
        self.output_limits = output_limits
        self._last_error = 0
        self._integral = 0
        self._last_time = None
        self._last_output = 0
    def compute(self, value):
        now = time.monotonic()
        error = self.setpoint - value
        dt = self.sample_time
        if self._last_time is not None:
            dt = now - self._last_time
        self._integral += error * dt
        derivative = (error - self._last_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        output = max(self.output_limits[0], min(self.output_limits[1], output))
        self._last_error = error
        self._last_time = now
        self._last_output = output
        return output
    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self._integral = 0
        self._last_error = 0

class Max6675Heater:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.mcu = config.getsection('mcu ' + config.get('mcu', 'mcu'))
        self.name = config.get_name().split()[1]
        self.cs_pin = config.get('cs_pin')
        self.ssr_pin = config.get('ssr_pin')
        self.pwm_freq = config.getfloat('pwm_freq', 1.0)
        self.heater_power = 0.0
        self.target_air = config.getfloat('target_air', 40.0)
        self.max_temp = config.getfloat('max_temp', 300.0)
        self.min_temp = config.getfloat('min_temp', 0.0)
        self.element_offset = config.getfloat('element_offset', 10.0)
        self.dht22_pin = config.getint('dht22_pin')
        self.dht22_interval = config.getfloat('dht22_interval', 2.0)
        self.pid_kp = config.getfloat('pid_kp', 2.0)
        self.pid_ki = config.getfloat('pid_ki', 0.1)
        self.pid_kd = config.getfloat('pid_kd', 1.0)
        self._register_gcodes()
        self.printer.register_event_handler('idle_timeout', self._on_idle)
        self.printer.register_event_handler('klippy:ready', self._on_ready)
        self._log = logging.getLogger('max6675_heater')
        self._pid = PID(self.pid_kp, self.pid_ki, self.pid_kd, setpoint=0)
        self._dht22_temp = None
        self._element_temp = None
        self._dht22_humidity = None
        self._log_entries = []  # List of (timestamp, avg_power, avg_air, avg_elem, avg_hum)
        self._log_accum = {'power': [], 'air': [], 'elem': [], 'hum': []}
        self._last_log_time = time.time()
        self._running = True
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._dht22_thread = threading.Thread(target=self._dht22_loop, daemon=True)
        self._log_thread = threading.Thread(target=self._log_loop, daemon=True)
        self._control_thread.start()
        self._dht22_thread.start()
        self._log_thread.start()

    def _register_gcodes(self):
        self.printer.register_event_handler('gcode:SET_HEATER_TEMP', self.cmd_SET_HEATER_TEMP)
        self.printer.register_event_handler('gcode:SET_HEATER_POWER', self.cmd_SET_HEATER_POWER)
        self.printer.register_event_handler('gcode:QUERY_HEATER', self.cmd_QUERY_HEATER)
        self.printer.register_event_handler('gcode:TUNE_HEATER_PID', self.cmd_TUNE_HEATER_PID)

    def _on_ready(self):
        self._log.info('MAX6675 Heater ready')
    def _on_idle(self, eventtime):
        self.set_power(0.0)

    def _dht22_loop(self):
        if Adafruit_DHT is None:
            self._log.warning('Adafruit_DHT not installed, DHT22/AM2302 not available')
            return
        while self._running:
            humidity, temp = Adafruit_DHT.read_retry(Adafruit_DHT.DHT22, self.dht22_pin)
            if temp is not None:
                self._dht22_temp = temp
            if humidity is not None:
                self._dht22_humidity = humidity
                self._log_accum['hum'].append(humidity)
            time.sleep(self.dht22_interval)

    def _control_loop(self):
        while self._running:
            air_temp = self.get_air_temp()
            element_temp = self.get_element_temp()
            if air_temp is None or element_temp is None:
                self.set_power(0.0)
                time.sleep(1)
                continue
            if air_temp < self.target_air:
                self.set_power(1.0)
            else:
                # PID to keep element just above air temp
                self._pid.set_setpoint(air_temp + self.element_offset)
                output = self._pid.compute(element_temp)
                self.set_power(output)
            time.sleep(self._pid.sample_time)

    def set_power(self, power):
        power = max(0.0, min(1.0, power))
        self.heater_power = power
        # Accumulate for logging
        self._log_accum['power'].append(power)
        pwm_value = int(power * 65535)
        self._send_ssr_pwm_set(pwm_value)
    def get_air_temp(self):
        val = self._dht22_temp
        if val is not None:
            self._log_accum['air'].append(val)
        return val
    def get_element_temp(self):
        temp = self._send_max6675_read()
        self._element_temp = temp
        if temp is not None:
            self._log_accum['elem'].append(temp)
        return temp
    def _send_max6675_read(self):
        resp = self.mcu.send('max6675_read')
        for line in resp.splitlines():
            if line.startswith('max6675_temp'):
                parts = line.strip().split()
                for p in parts:
                    if p.startswith('temp='):
                        value = int(p.split('=')[1])
                        return value * 0.25
        return None
    def _send_ssr_pwm_set(self, value):
        self.mcu.send('ssr_pwm_set', value=value)
    # GCODE handlers
    def cmd_SET_HEATER_TEMP(self, gcmd):
        temp = gcmd.get_float('S', None)
        if temp is not None:
            self.target_air = temp
    def cmd_SET_HEATER_POWER(self, gcmd):
        power = gcmd.get_float('S', None)
        if power is not None:
            self.set_power(power)
    def cmd_QUERY_HEATER(self, gcmd):
        air_temp = self.get_air_temp() or -999
        element_temp = self.get_element_temp() or -999
        humidity = self._dht22_humidity if hasattr(self, '_dht22_humidity') and self._dht22_humidity is not None else -999
        gcmd.respond_info('Air temp: %.2fC, Humidity: %.1f%%, Element temp: %.2fC, Power: %.2f%%' % (
            air_temp, humidity, element_temp, self.heater_power * 100.0))
    def cmd_TUNE_HEATER_PID(self, gcmd):
        kp = gcmd.get_float('KP', None)
        ki = gcmd.get_float('KI', None)
        kd = gcmd.get_float('KD', None)
        if kp is not None: self._pid.kp = kp
        if ki is not None: self._pid.ki = ki
        if kd is not None: self._pid.kd = kd
        gcmd.respond_info('PID updated: Kp=%.3f Ki=%.3f Kd=%.3f' % (self._pid.kp, self._pid.ki, self._pid.kd))

    def _log_loop(self):
        while self._running:
            now = time.time()
            # Log every 60 seconds
            if now - self._last_log_time >= 60:
                avg = lambda arr: sum(arr)/len(arr) if arr else None
                entry = (
                    int(now),
                    avg(self._log_accum['power']),
                    avg(self._log_accum['air']),
                    avg(self._log_accum['elem']),
                    avg(self._log_accum['hum'])
                )
                self._log_entries.append(entry)
                # Reset accumulators
                for k in self._log_accum:
                    self._log_accum[k].clear()
                self._last_log_time = now
            time.sleep(5)

    def cmd_EXPORT_HEATER_LOG(self, gcmd):
        lines = ["timestamp,avg_power,avg_air,avg_element,avg_humidity"]
        for entry in self._log_entries:
            ts, power, air, elem, hum = entry
            lines.append(f"{ts},{power:.3f},{air:.2f},{elem:.2f},{hum:.1f}")
        gcmd.respond_info("\n".join(lines))

def load_config(config):
    return Max6675Heater(config)
