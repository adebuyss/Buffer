# Klipper extras module for Mellow LLL Plus filament buffer
#
# Provides real-time buffer control via hall sensor callbacks and
# TMC2208 VACTUAL velocity mode, independent of the gcode queue.
# Optionally matches extruder velocity for smooth feed-forward control.
#
# Copyright (C) 2026  adebuyss
# License: GPLv3

import logging
import math

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
        # Enable motor once at startup - use VACTUAL=0 for stop
        # instead of toggling enable pin (which triggers full TMC
        # register re-init and overwhelms the MCU UART)
        self.manual_stepper.do_enable(True)
        self._write_vactual(0)

    def _update_vactual_value(self, rpm):
        # Match original firmware formula: RPM * microsteps * 200 / 60 / 0.715
        self.vactual_value = int(rpm * self.microsteps * 200 / 60. / 0.715)

    def _rpm_to_vactual(self, rpm):
        return int(rpm * self.microsteps * 200 / 60. / 0.715)

    def set_speed(self, rpm):
        self.speed_rpm = rpm
        self._update_vactual_value(rpm)

    def enable(self):
        pass

    def disable(self):
        pass

    def set_velocity(self, direction, vactual_override=None):
        """Set motor direction and speed. direction: FORWARD, BACK, or STOP."""
        if direction == STOP:
            if self.current_direction != STOP:
                self._write_vactual(0)
                self.current_direction = STOP
            return
        vactual = vactual_override if vactual_override is not None \
            else self.vactual_value
        # If changing between forward/back, stop first
        if (self.current_direction != STOP
                and self.current_direction != direction):
            self._write_vactual(0)
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
        self.full_zone_retract_length = config.getfloat(
            'full_zone_retract_length', 0., minval=0.)
        self.min_velocity = config.getfloat(
            'min_extrusion_velocity', 0.05, minval=0.)
        self.burst_feed_time = config.getfloat(
            'burst_feed_time', 0.5, above=0.)
        self.burst_delay = config.getfloat(
            'burst_delay', 0.5, minval=0.)
        self.pause_on_runout = config.getboolean('pause_on_runout', True)
        self.follow_retract = config.getboolean('follow_retract', True)
        self.debug = config.getboolean('debug', False)
        self.control_interval = config.getfloat(
            'control_interval', 0.25, above=0.05)
        self.velocity_factor = config.getfloat(
            'velocity_factor', 1.05, above=0.9, maxval=2.0)
        self.manual_feed_full_timeout = config.getfloat(
            'manual_feed_full_timeout', 5.0, above=0.)

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
        self._last_ext_time = 0.
        self._last_e_command_time = 0.
        self._expected_move_end = 0.
        self._feed_button_pressed = False
        self._retract_button_pressed = False
        self._full_zone_feed_time = 0.
        self._last_full_feed_time = 0.
        self._in_full_zone = False
        self._full_zone_retract_start = 0.
        self._extruder_retracting = False
        self._burst_delay_start = 0.
        self._burst_until = 0.
        self._burst_count = 0
        self._max_burst_cycles = 5
        self._manual_feed_full_start = 0.

        # Print state tracking
        self._print_stats = None

        # Gcode move hooks
        self._gcode_move = None
        self._orig_G0 = None
        self._orig_G1 = None
        self._orig_G2 = None
        self._orig_G3 = None

        # Velocity smoothing window
        self._velocity_window = []  # [(timestamp, velocity)]
        self._velocity_window_duration = config.getfloat(
            'velocity_window', 0.3, above=0., maxval=2.0)

        # Deferred drive state
        self._last_drive_time = 0.
        self._min_drive_interval = config.getfloat(
            'drive_interval', 0.1, above=0.02, maxval=1.0)
        self._drive_pending = False

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
        self._last_ext_time = self.reactor.monotonic()
        # Look up print_stats for printing vs idle detection
        try:
            self._print_stats = self.printer.lookup_object('print_stats')
        except Exception:
            logging.info("buffer: print_stats not available, "
                         "retraction following disabled")
        # Hook G0/G1/G2/G3 for immediate E movement detection
        self._gcode_move = self.printer.lookup_object('gcode_move')
        handlers = self.gcode.ready_gcode_handlers
        for cmd in ('G0', 'G1', 'G2', 'G3'):
            orig = handlers.get(cmd)
            if orig:
                setattr(self, '_orig_' + cmd, orig)
                self.gcode.register_command(cmd, None)
                self.gcode.register_command(cmd, self._cmd_move_wrapper)
        # Start control timer (watchdog for timeouts, bursts, decay)
        self._control_timer = self.reactor.register_timer(
            self._control_timer_cb, self.reactor.monotonic() + 1.)
        logging.info("buffer: ready")

    def _is_printing(self):
        """Return True if a print job is actively running."""
        if self._print_stats is None:
            return True  # Conservative fallback: assume printing
        return self._print_stats.get_status(
            self.reactor.monotonic())['state'] == "printing"

    # --- Gcode move hooks ---

    def _cmd_move_wrapper(self, gcmd):
        """Intercept G0/G1/G2/G3 to detect E movement immediately."""
        cmd = gcmd.get_command()
        orig = getattr(self, '_orig_' + cmd, None)
        if orig is None:
            return
        prev_pos = list(self._gcode_move.last_position)
        orig(gcmd)  # updates last_position, queues move
        new_e = self._gcode_move.last_position[3]
        e_delta = new_e - prev_pos[3]
        if abs(e_delta) > 0.001 and self.auto_enabled:
            self._on_e_movement(e_delta, prev_pos)

    def _on_e_movement(self, e_delta, prev_pos):
        """Process detected E movement: compute velocity and drive motor."""
        eventtime = self.reactor.monotonic()
        self._last_e_command_time = eventtime

        # Compute E velocity from commanded path speed
        gm = self._gcode_move
        speed = gm.speed  # mm/s (F/60 * speed_factor), persists across cmds

        # Klipper applies F to XYZ path, not XYZE
        curr = gm.last_position
        dx = curr[0] - prev_pos[0]
        dy = curr[1] - prev_pos[1]
        dz = curr[2] - prev_pos[2]
        xyz_dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if xyz_dist > 0.001:
            # Mixed XYZ+E move: E velocity proportional to E/XYZ ratio
            e_velocity = speed * abs(e_delta) / xyz_dist
        elif abs(e_delta) > 0.001:
            # Pure E move (retraction, load/unload): speed applies to E
            e_velocity = speed
        else:
            # Fallback: time-based estimate
            dt = eventtime - self._last_ext_time
            e_velocity = abs(e_delta) / max(dt, 0.001)

        self._last_ext_time = eventtime

        # Estimate move duration for velocity decay timing
        move_dist = xyz_dist if xyz_dist > 0.001 else abs(e_delta)
        if speed > 0.:
            self._expected_move_end = eventtime + move_dist / speed

        # Update buffer state
        if e_delta > 0.:
            self.extruder_velocity = e_velocity
            self._extruder_retracting = False
            self._velocity_window.append((eventtime, e_velocity))
        else:
            if self.follow_retract and not self._is_printing():
                self.extruder_velocity = e_velocity
                self._extruder_retracting = True
            else:
                self._extruder_retracting = True
                self.extruder_velocity = self._smoothed_velocity(eventtime)
                # Defer drive update — smoothed velocity keeps motor running
                # through brief infill retractions
                self._request_drive_update(eventtime)
                return  # Don't evaluate further during print retraction

        # Schedule deferred drive (rate-limited)
        self._request_drive_update(eventtime)

    def _handle_shutdown(self):
        self.motor.emergency_stop()
        self.state = STATE_DISABLED
        self.auto_enabled = False
        self._velocity_window = []
        self._manual_feed_full_start = 0.
        logging.info("buffer: shutdown - motor stopped")

    def _update_extruder_velocity(self, eventtime):
        """Refresh extruder velocity estimate. No-op — velocity is already
        updated in real time via G-code move hooks (_on_e_movement)."""
        pass

    def _smoothed_velocity(self, eventtime):
        """Return the max forward extrusion velocity seen in the recent window.
        Bridges brief retraction gaps during infill direction changes."""
        cutoff = eventtime - self._velocity_window_duration
        self._velocity_window = [
            (t, v) for t, v in self._velocity_window if t >= cutoff]
        if self._velocity_window:
            return max(v for t, v in self._velocity_window)
        return self.extruder_velocity

    def _request_drive_update(self, eventtime):
        """Schedule a deferred motor write if the rate limit allows."""
        if self._drive_pending:
            return  # callback already queued, it will pick up latest state
        if eventtime - self._last_drive_time >= self._min_drive_interval:
            self._drive_pending = True
            self.reactor.register_callback(self._deferred_drive)

    def _deferred_drive(self, eventtime):
        """Reactor callback: perform the actual UART writes."""
        self._drive_pending = False
        self._last_drive_time = eventtime
        self._evaluate_and_drive(eventtime)

    # --- Sensor callbacks ---

    def _make_sensor_callback(self, sensor_name):
        def callback(eventtime, state):
            # buttons module reports post-inversion pin level;
            # with ^! pins, state=0 means sensor blocked (triggered)
            self.sensor_states[sensor_name] = not bool(state)
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
            self._velocity_window = []
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
            self._manual_feed_full_start = 0.
            self.motor.set_velocity(FORWARD)
            self.motor_direction = FORWARD
        else:
            if self.state == STATE_MANUAL_FEED:
                self._stop_motor()
                self.auto_enabled = False
                self.state = STATE_IDLE
                self._velocity_window = []

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
                self.auto_enabled = False
                self.state = STATE_IDLE
                self._velocity_window = []

    # --- Control logic ---

    def _control_timer_cb(self, eventtime):
        # Auto-stop manual feed when full zone is sustained
        if self.state == STATE_MANUAL_FEED:
            if self.sensor_states['full']:
                if self._manual_feed_full_start == 0.:
                    self._manual_feed_full_start = eventtime
                elif (eventtime - self._manual_feed_full_start
                        >= self.manual_feed_full_timeout):
                    self._stop_motor()
                    self.state = STATE_IDLE
                    self.auto_enabled = False
                    self._manual_feed_full_start = 0.
                    self.gcode.respond_info(
                        "Buffer: manual feed stopped - full zone "
                        "sustained for %.0fs"
                        % self.manual_feed_full_timeout)
            else:
                self._manual_feed_full_start = 0.
            return eventtime + self.control_interval
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
        # Velocity decay: stop motor once the last move should have
        # completed and no new G1 has arrived
        if (self.extruder_velocity > 0.
                and eventtime > self._expected_move_end
                and eventtime - self._last_e_command_time
                    > self.control_interval):
            self.extruder_velocity = 0.
            self._extruder_retracting = False
            self._velocity_window = []
            self._evaluate_and_drive(eventtime)
        # Catch any drive updates that were rate-limited in the hook
        if (eventtime - self._last_drive_time >= self._min_drive_interval
                and (self.motor_direction != STOP
                     or self.extruder_velocity > self.min_velocity)):
            self._evaluate_and_drive(eventtime)
            self._last_drive_time = eventtime
        # Burst/full-zone timing still needs periodic evaluation
        elif (self.sensor_states['empty'] or self._in_full_zone
                or self._burst_until > 0.):
            self._evaluate_and_drive(eventtime)
        return eventtime + self.control_interval

    def _evaluate_and_drive(self, eventtime):
        if not self.auto_enabled or not self.material_present:
            return
        if self.state in (STATE_MANUAL_FEED, STATE_MANUAL_RETRACT,
                          STATE_ERROR, STATE_DISABLED):
            return

        # When not printing and extruder is retracting, follow retraction
        if (self._extruder_retracting and self.follow_retract
                and not self._is_printing()):
            if self.extruder_velocity > self.min_velocity:
                vactual = self._velocity_to_vactual(
                    self.extruder_velocity * self.correction_factor)
                if self.motor_direction != BACK:
                    self._set_motor(BACK, vactual)
                elif vactual != self.motor._last_vactual:
                    self.motor.set_velocity(BACK, vactual)
            else:
                if self.motor_direction != STOP:
                    self._stop_motor()
                    self.state = STATE_STOPPED
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

        # Priority: empty > full-only > full+middle overlap > middle > no sensors
        # Overlap zones (two adjacent sensors triggered) are normal during
        # carriage travel and get transitional behavior.

        if empty:
            # Buffer near empty — full speed feed / burst
            self._in_full_zone = False
            self._full_zone_feed_time = 0.
            if self._smoothed_velocity(eventtime) > self.min_velocity:
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
        elif full and not middle:
            # Deep full zone — slow down, timeout, then retract
            self._burst_delay_start = 0.
            self._burst_until = 0.
            self._burst_count = 0
            if not self._in_full_zone:
                self._in_full_zone = True
                self._full_zone_feed_time = 0.
                self._last_full_feed_time = 0.
                self._full_zone_retract_start = 0.
            if self._full_zone_feed_time >= self.full_zone_timeout:
                # Timeout exceeded - actively retract
                if (self.full_zone_retract_length > 0.
                        and self._full_zone_retract_start > 0.):
                    # Check if we've retracted long enough
                    try:
                        ms = self.motor.manual_stepper
                        rd = ms.steppers[0].get_rotation_distance()[0]
                    except Exception:
                        rd = 23.2
                    retract_speed = self.motor.speed_rpm * rd / 60.
                    elapsed = eventtime - self._full_zone_retract_start
                    if elapsed * retract_speed >= self.full_zone_retract_length:
                        new_direction = STOP
                        self._full_zone_retract_start = 0.
                    else:
                        new_direction = BACK
                        vactual_override = None
                else:
                    new_direction = BACK
                    vactual_override = None  # Use configured speed_rpm
                    if self.full_zone_retract_length > 0.:
                        self._full_zone_retract_start = eventtime
            elif self._smoothed_velocity(eventtime) > self.min_velocity:
                # Still extruding - slow down feed proportionally
                new_direction = FORWARD
                vactual_override = self._velocity_to_vactual(
                    self._smoothed_velocity(eventtime) * self.slowdown_factor)
                # Accumulate actual feeding time (not fixed increment)
                if self._last_full_feed_time > 0.:
                    self._full_zone_feed_time += \
                        eventtime - self._last_full_feed_time
                self._last_full_feed_time = eventtime
            else:
                # Not extruding and in full zone - just stop
                self._last_full_feed_time = 0.
                new_direction = STOP
        elif full and middle:
            # Overlap transition zone — slow down but no timeout/retract
            self._in_full_zone = False
            self._full_zone_feed_time = 0.
            self._burst_delay_start = 0.
            self._burst_until = 0.
            self._burst_count = 0
            if self._smoothed_velocity(eventtime) > self.min_velocity:
                new_direction = FORWARD
                vactual_override = self._velocity_to_vactual(
                    self._smoothed_velocity(eventtime) * self.slowdown_factor)
            else:
                new_direction = STOP
        elif middle:
            # In the ideal zone - match extruder velocity
            self._in_full_zone = False
            self._full_zone_feed_time = 0.
            self._burst_delay_start = 0.
            self._burst_until = 0.
            self._burst_count = 0
            if self._smoothed_velocity(eventtime) > self.min_velocity:
                new_direction = FORWARD
                vactual_override = self._velocity_to_vactual(
                    self._smoothed_velocity(eventtime) * self.velocity_factor)
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
                        self._smoothed_velocity(eventtime) > self.min_velocity:
                    new_direction = FORWARD
                    vactual_override = self._velocity_to_vactual(
                        self._smoothed_velocity(eventtime)
                        * self.correction_factor)
                elif self.motor_direction == BACK:
                    # Continue retracting until we hit middle
                    new_direction = BACK
                elif self._smoothed_velocity(eventtime) > self.min_velocity:
                    # Default: match extruder velocity
                    new_direction = FORWARD
                    vactual_override = self._velocity_to_vactual(
                        self._smoothed_velocity(eventtime)
                        * self.velocity_factor)
                else:
                    new_direction = STOP

        # Apply motor command
        if new_direction != self.motor_direction:
            self._set_motor(new_direction, vactual_override)
        elif vactual_override is not None and new_direction != STOP:
            # Update speed even if direction hasn't changed
            self.motor.set_velocity(new_direction, vactual_override)
            if self.debug:
                self.gcode.respond_info(
                    "Buffer debug: speed update dir=%s ext_vel=%.2f "
                    "smoothed=%.2f"
                    % (new_direction, self.extruder_velocity,
                       self._smoothed_velocity(eventtime)))

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
        if self.debug:
            self.gcode.respond_info(
                "Buffer debug: motor=%s ext_vel=%.2f smoothed=%.2f"
                % (direction, self.extruder_velocity,
                   self._smoothed_velocity(self.reactor.monotonic())))
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
            'smoothed_velocity': round(
                self._smoothed_velocity(eventtime), 2),
            'velocity_factor': self.velocity_factor,
            'motor_rpm': round(self.motor.speed_rpm, 1),
            'forward_elapsed': round(self.forward_elapsed, 1),
            'forward_timeout': self.forward_timeout,
            'speed_rpm': self.motor.speed_rpm,
            'in_full_zone': self._in_full_zone,
            'full_zone_feed_time': round(self._full_zone_feed_time, 1),
            'full_zone_timeout': self.full_zone_timeout,
            'full_zone_retract_length': self.full_zone_retract_length,
            'slowdown_factor': self.slowdown_factor,
            'burst_active': self._burst_until > 0.
                and self.reactor.monotonic() < self._burst_until,
            'is_printing': self._is_printing(),
            'extruder_retracting': self._extruder_retracting,
            'manual_feed_full_timeout': self.manual_feed_full_timeout,
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
               "  Extruder velocity: %.1f mm/s (smoothed: %.1f)\n"
               "  Velocity factor: %.2f\n"
               "  Speed RPM: %.0f\n"
               "  Forward elapsed: %.1f / %.0f s\n"
               "  Full zone: %s (feed %.1f / %.0f s)\n"
               "  Burst active: %s\n"
               "  Printing: %s  Retracting: %s"
               % (status['state'], status['motor_direction'],
                  status['sensor_empty'], status['sensor_middle'],
                  status['sensor_full'], status['material_present'],
                  status['enabled'], status['extruder_velocity'],
                  status['smoothed_velocity'],
                  status['velocity_factor'],
                  status['speed_rpm'],
                  status['forward_elapsed'], status['forward_timeout'],
                  status['in_full_zone'],
                  status['full_zone_feed_time'],
                  status['full_zone_timeout'],
                  status['burst_active'],
                  status['is_printing'],
                  status['extruder_retracting']))
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
        self._velocity_window = []
        self._manual_feed_full_start = 0.
        # Reset timing state
        self._last_ext_time = self.reactor.monotonic()
        self._last_e_command_time = 0.
        self._expected_move_end = 0.
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
        self._velocity_window = []
        self._manual_feed_full_start = 0.
        gcmd.respond_info("Buffer: automatic control disabled")

    def cmd_BUFFER_FEED(self, gcmd):
        speed = gcmd.get_float('SPEED', self.motor.speed_rpm, above=0.)
        vactual = self.motor._rpm_to_vactual(speed)
        self.motor.set_velocity(FORWARD, vactual)
        self.motor_direction = FORWARD
        self.state = STATE_MANUAL_FEED
        self._manual_feed_full_start = 0.
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
        self._manual_feed_full_start = 0.
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
            self._velocity_window = []
            self._manual_feed_full_start = 0.
            gcmd.respond_info("Buffer: error cleared")
        else:
            gcmd.respond_info("Buffer: no error to clear")


def load_config(config):
    return Buffer(config)
