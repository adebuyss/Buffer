# Klipper extras module for Mellow LLL Plus filament buffer
#
# Provides real-time buffer control via hall sensor callbacks and
# TMC2208 VACTUAL velocity mode, independent of the gcode queue.
# Optionally matches extruder velocity for smooth feed-forward control.
#
# Copyright (C) 2026  adebuyss
# License: GPLv3

import logging

FORWARD = 'forward'
BACK = 'back'
STOP = 'stop'

STATE_DISABLED = 'disabled'
STATE_IDLE = 'idle'
STATE_FEEDING = 'feeding'
STATE_STOPPED = 'stopped'
STATE_RETRACTING = 'retracting'
STATE_ERROR = 'error'
STATE_MANUAL_FEED = 'manual_feed'
STATE_MANUAL_RETRACT = 'manual_retract'

# TMC2208/2225 register addresses
REG_GCONF = 0x00
REG_VACTUAL = 0x22
GCONF_SHAFT_BIT = 3


class BufferMotor:
    """Controls buffer stepper motor via TMC2208 VACTUAL register writes."""
    def __init__(self, config, printer):
        self.printer = printer
        self.stepper_name = config.get('stepper')
        self.speed_rpm = config.getfloat('speed_rpm', 260., above=0.)
        self.microsteps = None
        self.vactual_value = 0
        self.mcu_tmc = None
        self.manual_stepper = None
        self.current_direction = STOP
        self._enabled = False
        self._last_shaft = -1
        self._last_vactual = -1

    def handle_ready(self):
        stepper_key = 'manual_stepper %s' % self.stepper_name
        tmc_key = 'tmc2208 %s' % stepper_key
        self.manual_stepper = self.printer.lookup_object(stepper_key)
        tmc_obj = self.printer.lookup_object(tmc_key)
        self.mcu_tmc = tmc_obj.mcu_tmc
        # Read microsteps from the TMC config
        chopconf = self.mcu_tmc.get_register('CHOPCONF')
        mres = (chopconf >> 24) & 0x0F
        self.microsteps = 256 >> mres
        self._update_vactual_value(self.speed_rpm)

    def _update_vactual_value(self, rpm):
        # Match original firmware formula: RPM * microsteps * 200 / 60 / 0.715
        self.vactual_value = int(rpm * self.microsteps * 200 / 60. / 0.715)

    def _rpm_to_vactual(self, rpm):
        return int(rpm * self.microsteps * 200 / 60. / 0.715)

    def set_speed(self, rpm):
        self.speed_rpm = rpm
        self._update_vactual_value(rpm)

    def enable(self):
        if not self._enabled:
            self.manual_stepper.do_enable(True)
            self._enabled = True

    def disable(self):
        if self._enabled:
            self.manual_stepper.do_enable(False)
            self._enabled = False

    def set_velocity(self, direction, vactual_override=None):
        """Set motor direction and speed. direction: FORWARD, BACK, or STOP."""
        if direction == STOP:
            if self.current_direction != STOP:
                self._write_vactual(0)
                self.current_direction = STOP
                self.disable()
            return
        vactual = vactual_override if vactual_override is not None \
            else self.vactual_value
        # If changing between forward/back, stop first
        if (self.current_direction != STOP
                and self.current_direction != direction):
            self._write_vactual(0)
        self.enable()
        shaft_val = 1 if direction == FORWARD else 0
        # Only write shaft if direction actually changed
        if shaft_val != self._last_shaft:
            self._write_shaft(shaft_val)
        # Only write vactual if value actually changed
        if vactual != self._last_vactual:
            self._write_vactual(vactual)
        self.current_direction = direction

    def _write_vactual(self, value):
        if self.mcu_tmc is None:
            return
        for attempt in range(3):
            try:
                self.mcu_tmc.set_register('VACTUAL', value)
                self._last_vactual = value
                return
            except Exception:
                if attempt == 2:
                    logging.warning("buffer: VACTUAL write failed after "
                                    "3 attempts")

    def _write_shaft(self, shaft_val):
        if self.mcu_tmc is None:
            return
        try:
            reg_val = self.mcu_tmc.fields.set_field('shaft', shaft_val)
            self.mcu_tmc.set_register('GCONF', reg_val)
            self._last_shaft = shaft_val
        except Exception as e:
            logging.warning("buffer: GCONF shaft write failed: %s" % e)

    def emergency_stop(self):
        try:
            self._write_vactual(0)
        except Exception:
            pass
        self.current_direction = STOP
        self._enabled = False
        self._last_shaft = -1
        self._last_vactual = -1


class Buffer:
    """Klipper extras module for filament buffer control."""
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.name = config.get_name()

        # Config
        self.forward_timeout = config.getfloat(
            'forward_timeout', 60., minval=0.)
        self.correction_factor = config.getfloat(
            'correction_factor', 1.3, above=1.)
        self.slowdown_factor = config.getfloat(
            'slowdown_factor', 0.5, above=0., below=1.)
        self.full_zone_timeout = config.getfloat(
            'full_zone_timeout', 3.0, above=0.)
        self.min_velocity = config.getfloat(
            'min_extrusion_velocity', 0.05, minval=0.)
        self.burst_feed_time = config.getfloat(
            'burst_feed_time', 0.5, above=0.)
        self.burst_delay = config.getfloat(
            'burst_delay', 0.5, minval=0.)
        self.pause_on_runout = config.getboolean('pause_on_runout', True)
        self.control_interval = config.getfloat(
            'control_interval', 0.1, above=0.)

        # Sensor pin names (resolved at ready time via buttons)
        self.sensor_empty_pin = config.get('sensor_empty_pin')
        self.sensor_middle_pin = config.get('sensor_middle_pin')
        self.sensor_full_pin = config.get('sensor_full_pin')
        self.material_switch_pin = config.get('material_switch_pin')
        self.feed_button_pin = config.get('feed_button_pin', None)
        self.retract_button_pin = config.get('retract_button_pin', None)

        # Motor
        self.motor = BufferMotor(config, self.printer)

        # State
        self.state = STATE_DISABLED
        self.auto_enabled = False
        self.sensor_states = {
            'empty': False, 'middle': False, 'full': False
        }
        self.material_present = False
        self.error_msg = ''
        self.forward_start_time = 0.
        self.forward_elapsed = 0.
        self.motor_direction = STOP
        self.extruder_velocity = 0.
        self.motor_rpm = 0.
        self._last_extruder_pos = 0.
        self._last_ext_time = 0.
        self._feed_button_pressed = False
        self._retract_button_pressed = False
        self._full_zone_feed_time = 0.
        self._last_full_feed_time = 0.
        self._in_full_zone = False
        self._extruder_retracting = False
        self._burst_delay_start = 0.
        self._burst_until = 0.
        self._burst_count = 0
        self._max_burst_cycles = 5

        # Timer handle
        self._control_timer = None

        # Register events
        self.printer.register_event_handler(
            'klippy:ready', self._handle_ready)
        self.printer.register_event_handler(
            'klippy:shutdown', self._handle_shutdown)

        # Register gcode commands
        self.gcode.register_command(
            'BUFFER_STATUS', self.cmd_BUFFER_STATUS,
            desc="Report buffer status")
        self.gcode.register_command(
            'BUFFER_ENABLE', self.cmd_BUFFER_ENABLE,
            desc="Enable automatic buffer control")
        self.gcode.register_command(
            'BUFFER_DISABLE', self.cmd_BUFFER_DISABLE,
            desc="Disable buffer control and stop motor")
        self.gcode.register_command(
            'BUFFER_FEED', self.cmd_BUFFER_FEED,
            desc="Manually feed filament forward")
        self.gcode.register_command(
            'BUFFER_RETRACT', self.cmd_BUFFER_RETRACT,
            desc="Manually retract filament")
        self.gcode.register_command(
            'BUFFER_STOP', self.cmd_BUFFER_STOP,
            desc="Stop motor and return to auto mode")
        self.gcode.register_command(
            'BUFFER_SET_SPEED', self.cmd_BUFFER_SET_SPEED,
            desc="Set buffer motor speed in RPM")
        self.gcode.register_command(
            'BUFFER_CLEAR_ERROR', self.cmd_BUFFER_CLEAR_ERROR,
            desc="Clear buffer error state")

        # Register sensor pins via buttons module
        self._register_sensors(config)

    def _register_sensors(self, config):
        buttons = self.printer.load_object(config, 'buttons')
        # Hall sensors
        for sensor_name, pin_name in [
                ('empty', self.sensor_empty_pin),
                ('middle', self.sensor_middle_pin),
                ('full', self.sensor_full_pin)]:
            buttons.register_buttons(
                [pin_name],
                self._make_sensor_callback(sensor_name))
        # Material switch
        buttons.register_buttons(
            [self.material_switch_pin], self._material_callback)
        # Manual buttons
        if self.feed_button_pin is not None:
            buttons.register_buttons(
                [self.feed_button_pin], self._feed_button_callback)
        if self.retract_button_pin is not None:
            buttons.register_buttons(
                [self.retract_button_pin], self._retract_button_callback)

    def _handle_ready(self):
        self.motor.handle_ready()
        # Initialize extruder tracking
        try:
            extruder = self.printer.lookup_object('extruder')
            self._last_extruder_pos = extruder.last_position[3]
        except Exception:
            self._last_extruder_pos = 0.
        self._last_ext_time = self.reactor.monotonic()
        # Start control timer
        self._control_timer = self.reactor.register_timer(
            self._control_timer_cb, self.reactor.monotonic() + 1.)
        logging.info("buffer: ready")

    def _handle_shutdown(self):
        self.motor.emergency_stop()
        self.state = STATE_DISABLED
        self.auto_enabled = False
        logging.info("buffer: shutdown - motor stopped")

    # --- Sensor callbacks ---

    def _make_sensor_callback(self, sensor_name):
        def callback(eventtime, state):
            self.sensor_states[sensor_name] = bool(state)
            if self.auto_enabled:
                self._update_extruder_velocity(eventtime)
                self._evaluate_and_drive(eventtime)
        return callback

    def _material_callback(self, eventtime, state):
        was_present = self.material_present
        self.material_present = bool(state)
        if not self.material_present and self.auto_enabled:
            self._stop_motor()
            self.state = STATE_IDLE
            if self.pause_on_runout:
                self._trigger_pause("buffer: filament runout detected")
        elif self.material_present and not was_present:
            # Filament inserted - auto-enable and start initial fill
            self.auto_enabled = True
            self.state = STATE_STOPPED
            self.error_msg = ''
            self._burst_count = 0
            self._burst_delay_start = 0.
            self._burst_until = 0.
            self._extruder_retracting = False
            logging.info("buffer: filament detected, auto-enabled")
            self.gcode.respond_info(
                "Buffer: filament detected, starting fill")
            self._update_extruder_velocity(eventtime)
            self._evaluate_and_drive(eventtime)

    def _feed_button_callback(self, eventtime, state):
        # Active LOW button, pin inverted: state=1 means pressed
        self._feed_button_pressed = bool(state)
        if state:
            if self.state == STATE_ERROR:
                return
            self.state = STATE_MANUAL_FEED
            self.motor.set_velocity(FORWARD)
            self.motor_direction = FORWARD
        else:
            if self.state == STATE_MANUAL_FEED:
                self._stop_motor()
                self.state = STATE_IDLE if not self.auto_enabled \
                    else STATE_STOPPED

    def _retract_button_callback(self, eventtime, state):
        self._retract_button_pressed = bool(state)
        if state:
            if self.state == STATE_ERROR:
                return
            self.state = STATE_MANUAL_RETRACT
            self.motor.set_velocity(BACK)
            self.motor_direction = BACK
        else:
            if self.state == STATE_MANUAL_RETRACT:
                self._stop_motor()
                self.state = STATE_IDLE if not self.auto_enabled \
                    else STATE_STOPPED

    # --- Control logic ---

    def _control_timer_cb(self, eventtime):
        if not self.auto_enabled:
            return eventtime + self.control_interval
        # Skip if in manual or error state
        if self.state in (STATE_MANUAL_FEED, STATE_MANUAL_RETRACT,
                          STATE_ERROR, STATE_DISABLED):
            return eventtime + self.control_interval
        # Check forward timeout
        if self.state == STATE_FEEDING and self.forward_timeout > 0.:
            self.forward_elapsed = eventtime - self.forward_start_time
            if self.forward_elapsed > self.forward_timeout:
                self._handle_error(
                    "Continuous forward motion exceeded %.0fs timeout"
                    % self.forward_timeout)
                return eventtime + self.control_interval
        # Compute extruder velocity and drive
        self._update_extruder_velocity(eventtime)
        self._evaluate_and_drive(eventtime)
        return eventtime + self.control_interval

    def _update_extruder_velocity(self, eventtime):
        try:
            extruder = self.printer.lookup_object('extruder')
            cur_pos = extruder.last_position[3]
        except Exception:
            self.extruder_velocity = 0.
            return
        dt = eventtime - self._last_ext_time
        if dt > 0.001:
            delta = cur_pos - self._last_extruder_pos
            if delta > 0.:
                self.extruder_velocity = delta / dt  # mm/s
                self._extruder_retracting = False
            elif delta < -0.01:
                # Retraction detected - immediately stop buffer motor
                self.extruder_velocity = 0.
                if not self._extruder_retracting:
                    self._extruder_retracting = True
                    if self.motor_direction == FORWARD:
                        self._stop_motor()
                        self.state = STATE_STOPPED
            else:
                self.extruder_velocity = 0.
            self._last_extruder_pos = cur_pos
            self._last_ext_time = eventtime

    def _evaluate_and_drive(self, eventtime):
        if not self.auto_enabled or not self.material_present:
            return
        if self.state in (STATE_MANUAL_FEED, STATE_MANUAL_RETRACT,
                          STATE_ERROR, STATE_DISABLED):
            return

        empty = self.sensor_states['empty']
        middle = self.sensor_states['middle']
        full = self.sensor_states['full']

        # Sanity check: both empty and full should not trigger together
        if empty and full:
            self._handle_error("Sensor conflict: empty and full both "
                               "triggered")
            return

        # Compute target velocity
        vactual_override = None
        new_direction = STOP

        if empty:
            # Buffer near empty
            self._in_full_zone = False
            self._full_zone_feed_time = 0.
            if self.extruder_velocity > self.min_velocity:
                # Extruder active - catch up at full speed
                new_direction = FORWARD
                self._burst_delay_start = 0.
                self._burst_until = 0.
                self._burst_count = 0
            elif self._burst_count >= self._max_burst_cycles:
                # Too many bursts without leaving empty zone
                self._handle_error(
                    "Buffer stuck in empty zone after %d burst cycles"
                    % self._burst_count)
                return
            elif self._burst_until > 0. and eventtime < self._burst_until:
                # Active burst in progress
                new_direction = FORWARD
            elif self._burst_delay_start > 0.:
                # Waiting for burst delay
                if eventtime - self._burst_delay_start >= self.burst_delay:
                    self._burst_until = eventtime + self.burst_feed_time
                    self._burst_delay_start = 0.
                    self._burst_count += 1
                    new_direction = FORWARD
                else:
                    new_direction = STOP
            else:
                # First time empty while stopped - start delay
                self._burst_delay_start = eventtime
                new_direction = STOP
        elif full:
            # Buffer near full - slow down proportionally first,
            # only retract after full_zone_timeout of active feeding
            self._burst_delay_start = 0.
            self._burst_until = 0.
            self._burst_count = 0
            if not self._in_full_zone:
                self._in_full_zone = True
                self._full_zone_feed_time = 0.
                self._last_full_feed_time = 0.
            if self._full_zone_feed_time >= self.full_zone_timeout:
                # Timeout exceeded - actively retract
                new_direction = BACK
                vactual_override = None  # Use configured speed_rpm
            elif self.extruder_velocity > self.min_velocity:
                # Still extruding - slow down feed proportionally
                new_direction = FORWARD
                vactual_override = self._velocity_to_vactual(
                    self.extruder_velocity * self.slowdown_factor)
                # Accumulate actual feeding time (not fixed increment)
                if self._last_full_feed_time > 0.:
                    self._full_zone_feed_time += \
                        eventtime - self._last_full_feed_time
                self._last_full_feed_time = eventtime
            else:
                # Not extruding and in full zone - just stop
                self._last_full_feed_time = 0.
                new_direction = STOP
        elif middle:
            # In the ideal zone - match extruder velocity
            self._in_full_zone = False
            self._full_zone_feed_time = 0.
            self._burst_delay_start = 0.
            self._burst_until = 0.
            self._burst_count = 0
            if self.extruder_velocity > self.min_velocity:
                new_direction = FORWARD
                vactual_override = self._velocity_to_vactual(
                    self.extruder_velocity)
            else:
                new_direction = STOP
        else:
            # No sensors triggered - we're between sensors
            self._in_full_zone = False
            self._full_zone_feed_time = 0.
            # Cancel burst if we've left the empty zone
            # (unless burst is active - let it finish)
            if self._burst_until > 0. and eventtime < self._burst_until:
                new_direction = FORWARD
            else:
                self._burst_delay_start = 0.
                self._burst_until = 0.
                self._burst_count = 0
                if self.motor_direction == FORWARD and \
                        self.extruder_velocity > self.min_velocity:
                    new_direction = FORWARD
                    vactual_override = self._velocity_to_vactual(
                        self.extruder_velocity * self.correction_factor)
                elif self.motor_direction == BACK:
                    # Continue retracting until we hit middle
                    new_direction = BACK
                elif self.extruder_velocity > self.min_velocity:
                    # Default: match extruder velocity
                    new_direction = FORWARD
                    vactual_override = self._velocity_to_vactual(
                        self.extruder_velocity)
                else:
                    new_direction = STOP

        # Apply motor command
        if new_direction != self.motor_direction:
            self._set_motor(new_direction, vactual_override)
        elif vactual_override is not None and new_direction != STOP:
            # Update speed even if direction hasn't changed
            self.motor.set_velocity(new_direction, vactual_override)

    def _velocity_to_vactual(self, mm_per_sec):
        """Convert filament velocity (mm/s) to VACTUAL register value."""
        # Get rotation_distance from the manual_stepper config
        # rotation_distance = mm per full revolution of the motor
        # RPM = mm_per_sec * 60 / rotation_distance
        try:
            ms = self.motor.manual_stepper
            # manual_stepper stores rotation_distance in the stepper object
            steppers = ms.steppers
            if steppers:
                rd = steppers[0].get_rotation_distance()[0]
            else:
                rd = 23.2  # fallback
        except Exception:
            rd = 23.2  # fallback
        rpm = mm_per_sec * 60. / rd
        # Clamp to configured max speed
        rpm = min(rpm, self.motor.speed_rpm)
        return self.motor._rpm_to_vactual(rpm)

    def _set_motor(self, direction, vactual_override=None):
        old_direction = self.motor_direction
        self.motor.set_velocity(direction, vactual_override)
        self.motor_direction = direction
        # Track forward timing
        if direction == FORWARD and old_direction != FORWARD:
            self.forward_start_time = self.reactor.monotonic()
            self.state = STATE_FEEDING
        elif direction == BACK:
            self.forward_elapsed = 0.
            self.state = STATE_RETRACTING
        elif direction == STOP:
            self.forward_elapsed = 0.
            self.state = STATE_STOPPED

    def _stop_motor(self):
        self.motor.set_velocity(STOP)
        self.motor_direction = STOP
        self.forward_elapsed = 0.

    def _handle_error(self, msg):
        self._stop_motor()
        self.state = STATE_ERROR
        self.error_msg = msg
        logging.error("buffer: %s" % msg)
        self.gcode.respond_info("Buffer ERROR: %s" % msg)
        if self.pause_on_runout:
            self._trigger_pause(msg)

    def _trigger_pause(self, msg):
        try:
            pause_resume = self.printer.lookup_object('pause_resume')
            if not pause_resume.is_paused:
                self.gcode.respond_info(
                    "Buffer: pausing print - %s" % msg)
                self.gcode.run_script("PAUSE")
        except Exception:
            logging.warning("buffer: failed to trigger pause")

    # --- Status ---

    def get_status(self, eventtime):
        return {
            'state': self.state,
            'motor_direction': self.motor_direction,
            'sensor_empty': self.sensor_states['empty'],
            'sensor_middle': self.sensor_states['middle'],
            'sensor_full': self.sensor_states['full'],
            'material_present': self.material_present,
            'enabled': self.auto_enabled,
            'error': self.error_msg,
            'extruder_velocity': round(self.extruder_velocity, 2),
            'motor_rpm': round(self.motor.speed_rpm, 1),
            'forward_elapsed': round(self.forward_elapsed, 1),
            'forward_timeout': self.forward_timeout,
            'speed_rpm': self.motor.speed_rpm,
            'in_full_zone': self._in_full_zone,
            'full_zone_feed_time': round(self._full_zone_feed_time, 1),
            'full_zone_timeout': self.full_zone_timeout,
            'slowdown_factor': self.slowdown_factor,
            'burst_active': self._burst_until > 0.
                and self.reactor.monotonic() < self._burst_until,
        }

    # --- Gcode commands ---

    def cmd_BUFFER_STATUS(self, gcmd):
        status = self.get_status(self.reactor.monotonic())
        msg = ("Buffer status:\n"
               "  State: %s\n"
               "  Motor: %s\n"
               "  Sensors: empty=%s middle=%s full=%s\n"
               "  Material present: %s\n"
               "  Auto enabled: %s\n"
               "  Extruder velocity: %.1f mm/s\n"
               "  Speed RPM: %.0f\n"
               "  Forward elapsed: %.1f / %.0f s\n"
               "  Full zone: %s (feed %.1f / %.0f s)\n"
               "  Burst active: %s"
               % (status['state'], status['motor_direction'],
                  status['sensor_empty'], status['sensor_middle'],
                  status['sensor_full'], status['material_present'],
                  status['enabled'], status['extruder_velocity'],
                  status['speed_rpm'],
                  status['forward_elapsed'], status['forward_timeout'],
                  status['in_full_zone'],
                  status['full_zone_feed_time'],
                  status['full_zone_timeout'],
                  status['burst_active']))
        if status['error']:
            msg += "\n  ERROR: %s" % status['error']
        gcmd.respond_info(msg)

    def cmd_BUFFER_ENABLE(self, gcmd):
        self.auto_enabled = True
        self.state = STATE_STOPPED
        self.error_msg = ''
        self.forward_elapsed = 0.
        self._in_full_zone = False
        self._full_zone_feed_time = 0.
        self._last_full_feed_time = 0.
        self._burst_delay_start = 0.
        self._burst_until = 0.
        self._burst_count = 0
        self._extruder_retracting = False
        # Reset extruder tracking
        try:
            extruder = self.printer.lookup_object('extruder')
            self._last_extruder_pos = extruder.last_position[3]
        except Exception:
            pass
        self._last_ext_time = self.reactor.monotonic()
        gcmd.respond_info("Buffer: automatic control enabled")

    def cmd_BUFFER_DISABLE(self, gcmd):
        self.auto_enabled = False
        self._stop_motor()
        self.state = STATE_DISABLED
        self._in_full_zone = False
        self._full_zone_feed_time = 0.
        self._last_full_feed_time = 0.
        self._burst_delay_start = 0.
        self._burst_until = 0.
        self._burst_count = 0
        gcmd.respond_info("Buffer: automatic control disabled")

    def cmd_BUFFER_FEED(self, gcmd):
        speed = gcmd.get_float('SPEED', self.motor.speed_rpm, above=0.)
        vactual = self.motor._rpm_to_vactual(speed)
        self.motor.set_velocity(FORWARD, vactual)
        self.motor_direction = FORWARD
        self.state = STATE_MANUAL_FEED
        gcmd.respond_info("Buffer: feeding at %.0f RPM" % speed)

    def cmd_BUFFER_RETRACT(self, gcmd):
        speed = gcmd.get_float('SPEED', self.motor.speed_rpm, above=0.)
        vactual = self.motor._rpm_to_vactual(speed)
        self.motor.set_velocity(BACK, vactual)
        self.motor_direction = BACK
        self.state = STATE_MANUAL_RETRACT
        gcmd.respond_info("Buffer: retracting at %.0f RPM" % speed)

    def cmd_BUFFER_STOP(self, gcmd):
        self._stop_motor()
        if self.auto_enabled:
            self.state = STATE_STOPPED
        else:
            self.state = STATE_IDLE
        gcmd.respond_info("Buffer: motor stopped")

    def cmd_BUFFER_SET_SPEED(self, gcmd):
        speed = gcmd.get_float('SPEED', above=0.)
        self.motor.set_speed(speed)
        gcmd.respond_info("Buffer: speed set to %.0f RPM" % speed)

    def cmd_BUFFER_CLEAR_ERROR(self, gcmd):
        if self.state == STATE_ERROR:
            self.state = STATE_STOPPED if self.auto_enabled \
                else STATE_IDLE
            self.error_msg = ''
            self.forward_elapsed = 0.
            self._in_full_zone = False
            self._full_zone_feed_time = 0.
            self._last_full_feed_time = 0.
            self._burst_delay_start = 0.
            self._burst_until = 0.
            self._burst_count = 0
            gcmd.respond_info("Buffer: error cleared")
        else:
            gcmd.respond_info("Buffer: no error to clear")


def load_config(config):
    return Buffer(config)
