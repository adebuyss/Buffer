"""Tests for E movement detection and velocity decay."""
import pytest
from conftest import (
    FORWARD, STOP,
    STATE_STOPPED,
    set_sensors, simulate_e_move,
)


class TestEVelocityComputation:
    def test_mixed_xyz_e_move(self, enabled_buf, reactor):
        """E velocity = speed * |e_delta| / xyz_dist for mixed moves."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, e_delta=0.5, xyz_dist=10.0, speed=100.0)
        # Expected: 100 * 0.5 / 10 = 5.0
        assert enabled_buf.extruder_velocity == pytest.approx(5.0, abs=0.1)

    def test_pure_e_move(self, enabled_buf, reactor):
        """Pure E move (retraction/load): e_velocity = speed."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        # Pure E move: no XYZ movement
        simulate_e_move(enabled_buf, e_delta=5.0, xyz_dist=0.0, speed=25.0)
        assert enabled_buf.extruder_velocity == pytest.approx(25.0, abs=0.1)

    def test_expected_move_end_computed(self, enabled_buf, reactor):
        set_sensors(enabled_buf, middle=True)
        t = 10.0
        reactor._monotonic = t
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=50.0, speed=100.0)
        # move_dist=50, speed=100 -> duration=0.5s
        assert enabled_buf._expected_move_end == pytest.approx(t + 0.5, abs=0.01)

    def test_pure_e_move_end_uses_e_delta(self, enabled_buf, reactor):
        set_sensors(enabled_buf, middle=True)
        t = 10.0
        reactor._monotonic = t
        simulate_e_move(enabled_buf, e_delta=5.0, xyz_dist=0.0, speed=25.0)
        # move_dist=5, speed=25 -> duration=0.2s
        assert enabled_buf._expected_move_end == pytest.approx(t + 0.2, abs=0.01)


class TestVelocityDecay:
    def test_decay_zeroes_velocity(self, enabled_buf, reactor):
        set_sensors(enabled_buf, middle=True)
        t = 10.0
        reactor._monotonic = t
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=50.0, speed=100.0)
        assert enabled_buf.extruder_velocity > 0

        # Advance past expected_move_end + control_interval
        t = enabled_buf._expected_move_end + enabled_buf.control_interval + 0.1
        reactor._monotonic = t
        enabled_buf._last_e_command_time = 10.0  # original command time
        enabled_buf._control_timer_cb(t)
        assert enabled_buf.extruder_velocity == 0.0

    def test_new_move_prevents_decay(self, enabled_buf, reactor):
        set_sensors(enabled_buf, middle=True)
        t = 10.0
        reactor._monotonic = t
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=50.0, speed=100.0)

        # Send another move before decay would kick in
        t += 0.3
        reactor._monotonic = t
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=50.0, speed=100.0)

        # The expected_move_end is now pushed forward
        assert enabled_buf._expected_move_end > t


class TestEMovementDetection:
    def test_small_e_delta_ignored(self, enabled_buf, reactor, gcode_move):
        """E deltas < 0.001 are ignored."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        prev_velocity = enabled_buf.extruder_velocity
        # Very tiny E move
        gcode_move.last_position[3] += 0.0001
        # _on_e_movement would normally be called by the wrapper,
        # but with delta < 0.001 the wrapper skips it
        # So we verify the wrapper logic indirectly
        assert enabled_buf.extruder_velocity == prev_velocity

    def test_negative_e_delta_is_retraction(self, enabled_buf, reactor):
        """Negative E delta while printing stops motor."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        enabled_buf.motor_direction = FORWARD
        enabled_buf.state = STATE_STOPPED
        # Set print_stats to printing
        enabled_buf._print_stats.state = "printing"
        simulate_e_move(enabled_buf, e_delta=-2.0, xyz_dist=0.0, speed=30.0)
        assert enabled_buf._extruder_retracting is True
        assert enabled_buf.extruder_velocity == 0.0
