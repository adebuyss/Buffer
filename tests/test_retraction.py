"""Tests for retraction following when not printing."""
import pytest
from conftest import (
    FORWARD, BACK, STOP,
    STATE_STOPPED, STATE_RETRACTING,
    set_sensors, simulate_e_move,
)


class TestRetractionFollowing:
    def test_non_printing_retraction_follows_back(self, enabled_buf, reactor):
        enabled_buf._print_stats.state = "standby"
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, e_delta=-5.0, xyz_dist=0.0, speed=30.0)
        assert enabled_buf._extruder_retracting is True
        assert enabled_buf.motor_direction == BACK

    def test_retraction_below_min_velocity_stops(self, enabled_buf, reactor):
        enabled_buf._print_stats.state = "standby"
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        # Set velocity below min
        enabled_buf._extruder_retracting = True
        enabled_buf.extruder_velocity = 0.01  # below min_velocity (0.05)
        enabled_buf._evaluate_and_drive(10.0)
        assert enabled_buf.motor_direction == STOP

    def test_printing_retraction_preserves_smoothed_velocity(
            self, enabled_buf, reactor):
        enabled_buf._print_stats.state = "printing"
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        enabled_buf.motor_direction = FORWARD
        enabled_buf.state = STATE_STOPPED
        # First extrude to populate the velocity window
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        prev_velocity = enabled_buf.extruder_velocity
        assert prev_velocity > 0
        # Now retract — should preserve smoothed velocity
        reactor._monotonic = 10.05
        simulate_e_move(enabled_buf, e_delta=-2.0, xyz_dist=0.0, speed=30.0)
        assert enabled_buf._extruder_retracting is True
        # Velocity preserved via smoothing window, not zeroed
        assert enabled_buf.extruder_velocity == pytest.approx(
            prev_velocity, abs=0.1)

    def test_follow_retract_disabled(self, enabled_buf, reactor):
        enabled_buf.follow_retract = False
        enabled_buf._print_stats.state = "standby"
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, e_delta=-5.0, xyz_dist=0.0, speed=30.0)
        # With follow_retract=False and not printing, retraction should still
        # set _extruder_retracting but velocity should be 0
        assert enabled_buf._extruder_retracting is True
        assert enabled_buf.extruder_velocity == 0.0

    def test_forward_extrusion_clears_retraction(self, enabled_buf, reactor):
        enabled_buf._print_stats.state = "standby"
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, e_delta=-5.0, xyz_dist=0.0, speed=30.0)
        assert enabled_buf._extruder_retracting is True

        # Forward extrusion
        simulate_e_move(enabled_buf, e_delta=5.0, xyz_dist=0.0, speed=30.0)
        assert enabled_buf._extruder_retracting is False
        assert enabled_buf.extruder_velocity > 0
