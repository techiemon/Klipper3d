# rp2040_gearmotor_filament_cutter.py
# Klipper extension for controlling a filament cutter with a gearmotor controlled by an H-bridge
# Filiment cutter action sequence:
# 1. A printer can have 2 filament cutters, one for each extruder
# 2. The filament cutter is triggered by a GCode command RUN_CUTTER
# 3. There are 2 endstop switches on each filament cutter to detect the park and fully cut positions
# 4. When the filament cutter is triggered, it will move to, or verify it is already at the park position and wait for the endstop switch to be pressed
# 5. Then the motor will reverse and it will move to the fully cut position and wait for the endstop switch to be pressed
# 6. Then the motor will go back to the park position and wait for the next GCode command.
# 7. The filament cutter will be disabled when the printer is powered off.
# 8. The filament cutter will be enabled when the printer is powered on.
# Hardware
# The PR2040 will be supplied with 3.3V from the printer.
# The H-bridge is controlled by GPIO pins on the RP2040.
# The cutter motor is powered by a 12V power supply.
# The cutter motor is controlled by an H-bridge. One pin is forward and the other is reverse. Both pins should not be active at the same time ever!
# The cutter has 2 endstop switches to detect the park and fully cut positions.
# There is a PARK_CUTTER command that interrupts the filament cutter action sequence if the park endstop is not pressed.
# Monitor and log time time to complete the Park to Cut sequence, and then the cut to park sequence time.
# Once we determine the time to complete the Park to Cut sequence, we can calculate the time to complete the Cut to Park sequence.
# This calculation can then be used as a constant to determine if any of the steps is taking longer than expected.
# If any of the steps is taking longer than expected, we can log a warning and attempt to park the filament cutter.

import logging
import time
from . import pins

class FilamentCutter:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        
        # Configuration
        self.name = config.get_name().split()[-1]
        
        # GPIO Pin assignments
        self.forward_pin = config.get('forward_pin')
        self.reverse_pin = config.get('reverse_pin')
        self.park_endstop_pin = config.get('park_endstop_pin')
        self.cut_endstop_pin = config.get('cut_endstop_pin')
        
        # Timing configuration
        self.operation_timeout = config.getfloat('operation_timeout', 120.0)  # 2 minutes default
        self.endstop_timeout = config.getfloat('endstop_timeout', 30.0)      # 30 seconds for endstop
        
        # State variables
        self.state = "IDLE"  # IDLE, MOVING_TO_PARK, MOVING_TO_CUT, PARKING, ERROR
        self.is_enabled = False
        self.operation_start_time = 0
        self.park_to_cut_time = 0
        self.cut_to_park_time = 0
        self.interrupt_requested = False
        
        # Setup pins
        ppins = self.printer.lookup_object('pins')
        self.forward_pin_obj = ppins.setup_pin('digital_out', self.forward_pin)
        self.reverse_pin_obj = ppins.setup_pin('digital_out', self.reverse_pin)
        self.park_endstop_obj = ppins.setup_pin('endstop', self.park_endstop_pin)
        self.cut_endstop_obj = ppins.setup_pin('endstop', self.cut_endstop_pin)
        
        # Initialize pins to safe state
        self.forward_pin_obj.setup_start_value(0, 0)
        self.reverse_pin_obj.setup_start_value(0, 0)
        
        # Setup endstop callbacks
        self.park_endstop_obj.add_stepper(None)
        self.cut_endstop_obj.add_stepper(None)
        
        # Register commands
        self.gcode.register_mux_command(
            "RUN_CUTTER", "CUTTER", self.name,
            self.cmd_RUN_CUTTER, desc=self.cmd_RUN_CUTTER_help)
        self.gcode.register_mux_command(
            "PARK_CUTTER", "CUTTER", self.name,
            self.cmd_PARK_CUTTER, desc=self.cmd_PARK_CUTTER_help)
        self.gcode.register_mux_command(
            "QUERY_CUTTER", "CUTTER", self.name,
            self.cmd_QUERY_CUTTER, desc=self.cmd_QUERY_CUTTER_help)
        
        # Register for printer events
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        
        logging.info("FilamentCutter '%s' initialized", self.name)

    def _handle_ready(self):
        """Called when printer is ready"""
        self.is_enabled = True
        self.state = "IDLE"
        logging.info("FilamentCutter '%s' enabled", self.name)

    def _handle_shutdown(self):
        """Called when printer shuts down"""
        self._stop_motor()
        self.is_enabled = False
        self.state = "IDLE"
        logging.info("FilamentCutter '%s' disabled", self.name)

    def _stop_motor(self):
        """Immediately stop motor - safety critical"""
        self.forward_pin_obj.set_digital(0)
        self.reverse_pin_obj.set_digital(0)

    def _move_forward(self):
        """Move motor forward (towards cut position)"""
        self.reverse_pin_obj.set_digital(0)  # Ensure reverse is off first
        self.reactor.pause(0.001)  # Brief pause to prevent shoot-through
        self.forward_pin_obj.set_digital(1)

    def _move_reverse(self):
        """Move motor reverse (towards park position)"""
        self.forward_pin_obj.set_digital(0)  # Ensure forward is off first
        self.reactor.pause(0.001)  # Brief pause to prevent shoot-through
        self.reverse_pin_obj.set_digital(1)

    def _is_at_park(self):
        """Check if cutter is at park position"""
        return self.park_endstop_obj.query_endstop(self.reactor.monotonic())

    def _is_at_cut(self):
        """Check if cutter is at cut position"""
        return self.cut_endstop_obj.query_endstop(self.reactor.monotonic())

    def _wait_for_endstop(self, endstop_obj, timeout, description):
        """Wait for endstop to be triggered with timeout"""
        start_time = self.reactor.monotonic()
        
        while True:
            if self.interrupt_requested:
                raise Exception("Operation interrupted by PARK_CUTTER command")
                
            current_time = self.reactor.monotonic()
            if current_time - start_time > timeout:
                raise Exception(f"Timeout waiting for {description} endstop")
                
            if endstop_obj.query_endstop(current_time):
                return current_time - start_time
                
            self.reactor.pause(0.1)  # Check every 100ms

    def _execute_cut_sequence(self):
        """Execute the complete cutting sequence"""
        try:
            self.operation_start_time = self.reactor.monotonic()
            self.interrupt_requested = False
            
            # Step 1: Ensure we're at park position
            if not self._is_at_park():
                logging.info("FilamentCutter '%s': Moving to park position", self.name)
                self.state = "MOVING_TO_PARK"
                self._move_reverse()
                self._wait_for_endstop(self.park_endstop_obj, self.endstop_timeout, "park")
                self._stop_motor()
                logging.info("FilamentCutter '%s': Reached park position", self.name)
            else:
                logging.info("FilamentCutter '%s': Already at park position", self.name)

            # Step 2: Move to cut position
            logging.info("FilamentCutter '%s': Moving to cut position", self.name)
            self.state = "MOVING_TO_CUT"
            cut_start_time = self.reactor.monotonic()
            self._move_forward()
            self._wait_for_endstop(self.cut_endstop_obj, self.endstop_timeout, "cut")
            self._stop_motor()
            self.park_to_cut_time = self.reactor.monotonic() - cut_start_time
            logging.info("FilamentCutter '%s': Reached cut position (%.2fs)", 
                        self.name, self.park_to_cut_time)

            # Step 3: Return to park position
            logging.info("FilamentCutter '%s': Returning to park position", self.name)
            self.state = "PARKING"
            park_start_time = self.reactor.monotonic()
            self._move_reverse()
            self._wait_for_endstop(self.park_endstop_obj, self.endstop_timeout, "park")
            self._stop_motor()
            self.cut_to_park_time = self.reactor.monotonic() - park_start_time
            
            total_time = self.reactor.monotonic() - self.operation_start_time
            logging.info("FilamentCutter '%s': Cut sequence complete (%.2fs total, %.2fs park->cut, %.2fs cut->park)", 
                        self.name, total_time, self.park_to_cut_time, self.cut_to_park_time)
            
            self.state = "IDLE"
            
        except Exception as e:
            self._stop_motor()
            self.state = "ERROR"
            logging.error("FilamentCutter '%s': Error during cut sequence: %s", self.name, str(e))
            raise

    cmd_RUN_CUTTER_help = "Execute filament cutting sequence"
    def cmd_RUN_CUTTER(self, gcmd):
        if not self.is_enabled:
            raise gcmd.error("FilamentCutter '%s' is not enabled" % self.name)
        
        if self.state != "IDLE":
            raise gcmd.error("FilamentCutter '%s' is busy (state: %s)" % (self.name, self.state))
        
        try:
            gcmd.respond_info("FilamentCutter '%s': Starting cut sequence" % self.name)
            self._execute_cut_sequence()
            gcmd.respond_info("FilamentCutter '%s': Cut sequence completed successfully" % self.name)
        except Exception as e:
            gcmd.respond_error("FilamentCutter '%s': %s" % (self.name, str(e)))

    cmd_PARK_CUTTER_help = "Interrupt current operation and move to park position"
    def cmd_PARK_CUTTER(self, gcmd):
        if not self.is_enabled:
            raise gcmd.error("FilamentCutter '%s' is not enabled" % self.name)
        
        if self.state == "IDLE":
            if self._is_at_park():
                gcmd.respond_info("FilamentCutter '%s': Already at park position" % self.name)
                return
        
        try:
            gcmd.respond_info("FilamentCutter '%s': Parking cutter" % self.name)
            self.interrupt_requested = True
            self._stop_motor()
            
            # Force move to park if not already there
            if not self._is_at_park():
                self.state = "PARKING"
                self._move_reverse()
                self._wait_for_endstop(self.park_endstop_obj, self.endstop_timeout, "park")
                self._stop_motor()
            
            self.state = "IDLE"
            self.interrupt_requested = False
            gcmd.respond_info("FilamentCutter '%s': Parked successfully" % self.name)
            
        except Exception as e:
            self._stop_motor()
            self.state = "ERROR"
            gcmd.respond_error("FilamentCutter '%s': Error during parking: %s" % (self.name, str(e)))

    cmd_QUERY_CUTTER_help = "Query filament cutter status"
    def cmd_QUERY_CUTTER(self, gcmd):
        at_park = self._is_at_park()
        at_cut = self._is_at_cut()
        
        status_msg = (
            f"FilamentCutter '{self.name}' Status:\n"
            f"  State: {self.state}\n"
            f"  Enabled: {self.is_enabled}\n"
            f"  At Park: {at_park}\n"
            f"  At Cut: {at_cut}\n"
            f"  Park->Cut Time: {self.park_to_cut_time:.2f}s\n"
            f"  Cut->Park Time: {self.cut_to_park_time:.2f}s"
        )
        
        gcmd.respond_info(status_msg)

def load_config_prefix(config):
    return FilamentCutter(config)
