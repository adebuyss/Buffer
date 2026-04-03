"""Tests for _evaluate_and_drive(): sensor zone logic and state transitions."""
import pytest
from conftest import (
    FORWARD, BACK, STOP,
    STATE_STOPPED, STATE_FEEDING, STATE_RETRACTING, STATE_ERROR,
    STATE_MANUAL_FEED, STATE_DISABLED,
    set_sensors, simulate_e_move,
)


class TestMiddleZone:
    def test_extruding_feeds_forward(self, enabled_buf, reactor):
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, 1.0, xyz_dist=10.0, speed=50.0)
        assert enabled_buf.motor_direction == FORWARD
        assert enabled_buf.state == STATE_FEEDING

    def test_idle_stops_motor(self, enabled_buf, reactor):
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        enabled_buf.extruder_velocity = 0.0
        enabled_buf._evaluate_and_drive(10.0)
        assert enabled_buf.motor_direction == STOP

    def test_resets_burst_and_full_zone_state(self, enabled_buf, reactor):
        enabled_buf._burst_count = 3
        enabled_buf._in_full_zone = True
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        enabled_buf._evaluate_and_drive(10.0)
        assert enabled_buf._burst_count == 0
        assert enabled_buf._in_full_zone is False


class TestEmptyZone:
    def test_extruding_feeds_full_speed(self, enabled_buf, reactor):
        set_sensors(enabled_buf, empty=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, 1.0, xyz_dist=10.0, speed=50.0)
        assert enabled_buf.motor_direction == FORWARD
        assert enabled_buf.state == STATE_FEEDING

    def test_no_extrusion_starts_burst_delay(self, enabled_buf, reactor):
        set_sensors(enabled_buf, empty=True)
        t = 10.0
        reactor._monotonic = t
        enabled_buf.extruder_velocity = 0.0
        enabled_buf._evaluate_and_drive(t)
        assert enabled_buf._burst_delay_start == t
        assert enabled_buf.motor_direction == STOP


class TestFullZone:
    def test_extruding_slows_down(self, enabled_buf, reactor):
        set_sensors(enabled_buf, full=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, 1.0, xyz_dist=10.0, speed=50.0)
        assert enabled_buf.motor_direction == FORWARD
        assert enabled_buf._in_full_zone is True

    def test_idle_stops(self, enabled_buf, reactor):
        set_sensors(enabled_buf, full=True)
        reactor._monotonic = 10.0
        enabled_buf.extruder_velocity = 0.0
        enabled_buf._evaluate_and_drive(10.0)
        assert enabled_buf.motor_direction == STOP


class TestFullMiddleOverlap:
    def test_extruding_slows_no_timeout(self, enabled_buf, reactor):
        set_sensors(enabled_buf, full=True, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, 1.0, xyz_dist=10.0, speed=50.0)
        assert enabled_buf.motor_direction == FORWARD
        assert enabled_buf._in_full_zone is False
        assert enabled_buf._full_zone_feed_time == 0.0

    def test_idle_stops(self, enabled_buf, reactor):
        set_sensors(enabled_buf, full=True, middle=True)
        reactor._monotonic = 10.0
        enabled_buf.extruder_velocity = 0.0
        enabled_buf._evaluate_and_drive(10.0)
        assert enabled_buf.motor_direction == STOP


class TestNoSensors:
    def test_was_feeding_continues_with_correction(self, enabled_buf, reactor):
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, 1.0, xyz_dist=10.0, speed=50.0)
        assert enabled_buf.motor_direction == FORWARD
        # Now no sensors
        set_sensors(enabled_buf, empty=False, middle=False, full=False)
        enabled_buf._evaluate_and_drive(10.0)
        assert enabled_buf.motor_direction == FORWARD

    def test_was_retracting_continues_back(self, enabled_buf, reactor):
        enabled_buf.motor_direction = BACK
        enabled_buf.motor.current_direction = BACK
        set_sensors(enabled_buf)
        reactor._monotonic = 10.0
        enabled_buf.extruder_velocity = 0.0
        enabled_buf._evaluate_and_drive(10.0)
        assert enabled_buf.motor_direction == BACK

    def test_idle_stops(self, enabled_buf, reactor):
        enabled_buf.motor_direction = STOP
        set_sensors(enabled_buf)
        reactor._monotonic = 10.0
        enabled_buf.extruder_velocity = 0.0
        enabled_buf._evaluate_and_drive(10.0)
        assert enabled_buf.motor_direction == STOP


class TestSensorConflict:
    def test_empty_and_full_triggers_error(self, enabled_buf, reactor):
        set_sensors(enabled_buf, empty=True, full=True)
        reactor._monotonic = 10.0
        enabled_buf._evaluate_and_drive(10.0)
        assert enabled_buf.state == STATE_ERROR
        assert "Sensor conflict" in enabled_buf.error_msg


class TestGuardConditions:
    def test_not_enabled_returns_early(self, buf, reactor):
        buf.auto_enabled = False
        set_sensors(buf, empty=True)
        buf._evaluate_and_drive(10.0)
        assert buf.motor_direction == STOP

    def test_no_material_returns_early(self, enabled_buf, reactor):
        enabled_buf.material_present = False
        set_sensors(enabled_buf, empty=True)
        enabled_buf._evaluate_and_drive(10.0)
        assert enabled_buf.motor_direction == STOP

    def test_manual_state_returns_early(self, enabled_buf, reactor):
        enabled_buf.state = STATE_MANUAL_FEED
        set_sensors(enabled_buf, empty=True)
        enabled_buf.extruder_velocity = 10.0
        enabled_buf._evaluate_and_drive(10.0)
        # Motor direction not changed by _evaluate_and_drive
        assert enabled_buf.state == STATE_MANUAL_FEED

    def test_error_state_returns_early(self, enabled_buf, reactor):
        enabled_buf.state = STATE_ERROR
        set_sensors(enabled_buf, empty=True)
        enabled_buf._evaluate_and_drive(10.0)
        assert enabled_buf.state == STATE_ERROR

    def test_disabled_state_returns_early(self, enabled_buf, reactor):
        enabled_buf.state = STATE_DISABLED
        set_sensors(enabled_buf, empty=True)
        enabled_buf._evaluate_and_drive(10.0)
        assert enabled_buf.state == STATE_DISABLED


class TestZoneTransitions:
    def test_middle_to_empty_to_middle(self, enabled_buf, reactor):
        # Start in middle zone, extruding
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, 1.0, xyz_dist=10.0, speed=50.0)
        assert enabled_buf.motor_direction == FORWARD

        # Transition to empty
        set_sensors(enabled_buf, empty=True)
        enabled_buf._evaluate_and_drive(10.5)
        assert enabled_buf.motor_direction == FORWARD
        assert enabled_buf.state == STATE_FEEDING

        # Back to middle
        set_sensors(enabled_buf, middle=True)
        enabled_buf._evaluate_and_drive(11.0)
        assert enabled_buf.motor_direction == FORWARD

    def test_middle_to_full_to_full_middle_to_middle(self, enabled_buf, reactor):
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, 1.0, xyz_dist=10.0, speed=50.0)

        # Full zone (deep)
        set_sensors(enabled_buf, full=True)
        enabled_buf._evaluate_and_drive(10.5)
        assert enabled_buf._in_full_zone is True

        # Full+middle overlap
        set_sensors(enabled_buf, full=True, middle=True)
        enabled_buf._evaluate_and_drive(11.0)
        assert enabled_buf._in_full_zone is False

        # Back to middle
        set_sensors(enabled_buf, middle=True)
        enabled_buf._evaluate_and_drive(11.5)
        assert enabled_buf._in_full_zone is False
        assert enabled_buf._full_zone_feed_time == 0.0
