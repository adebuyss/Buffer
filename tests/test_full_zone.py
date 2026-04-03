"""Tests for full zone timeout and retraction logic."""
import pytest
from conftest import (
    FORWARD, BACK, STOP,
    STATE_FEEDING, STATE_RETRACTING,
    set_sensors, simulate_e_move,
)


class TestFullZoneEntry:
    def test_entering_full_zone_sets_flag(self, enabled_buf, reactor):
        set_sensors(enabled_buf, full=True)
        reactor._monotonic = 10.0
        enabled_buf.extruder_velocity = 5.0
        enabled_buf._evaluate_and_drive(10.0)
        assert enabled_buf._in_full_zone is True
        assert enabled_buf._full_zone_feed_time == 0.0

    def test_reentry_resets_timer(self, enabled_buf, reactor):
        # Enter full zone and accumulate time
        set_sensors(enabled_buf, full=True)
        reactor._monotonic = 10.0
        enabled_buf.extruder_velocity = 5.0
        enabled_buf._evaluate_and_drive(10.0)
        enabled_buf._evaluate_and_drive(11.0)

        # Leave to middle
        set_sensors(enabled_buf, middle=True)
        enabled_buf._evaluate_and_drive(12.0)
        assert enabled_buf._in_full_zone is False
        assert enabled_buf._full_zone_feed_time == 0.0

        # Re-enter full zone
        set_sensors(enabled_buf, full=True)
        enabled_buf._evaluate_and_drive(13.0)
        assert enabled_buf._in_full_zone is True
        assert enabled_buf._full_zone_feed_time == 0.0


class TestFullZoneFeedTimeAccumulation:
    def test_accumulates_while_extruding(self, enabled_buf, reactor):
        set_sensors(enabled_buf, full=True)
        t = 10.0
        reactor._monotonic = t
        enabled_buf.extruder_velocity = 5.0

        # First call: enters full zone, sets _last_full_feed_time
        enabled_buf._evaluate_and_drive(t)
        assert enabled_buf._in_full_zone is True

        # Second call 1s later: accumulates feed time
        t += 1.0
        reactor._monotonic = t
        enabled_buf._evaluate_and_drive(t)
        assert enabled_buf._full_zone_feed_time == pytest.approx(1.0, abs=0.01)

    def test_does_not_accumulate_when_idle(self, enabled_buf, reactor):
        set_sensors(enabled_buf, full=True)
        t = 10.0
        reactor._monotonic = t
        enabled_buf.extruder_velocity = 5.0

        # Enter full zone while extruding
        enabled_buf._evaluate_and_drive(t)

        # Stop extruding
        t += 1.0
        reactor._monotonic = t
        enabled_buf.extruder_velocity = 0.0
        enabled_buf._evaluate_and_drive(t)

        # More time passes, still idle
        t += 2.0
        reactor._monotonic = t
        enabled_buf._evaluate_and_drive(t)
        # Feed time should not include idle time
        # (only accumulated during the extruding period)
        assert enabled_buf._full_zone_feed_time < 2.0


class TestFullZoneTimeout:
    def test_timeout_triggers_retract(self, enabled_buf, reactor):
        set_sensors(enabled_buf, full=True)
        t = 10.0
        reactor._monotonic = t
        enabled_buf.extruder_velocity = 5.0

        # Simulate continuous extrusion in full zone past timeout
        enabled_buf._evaluate_and_drive(t)
        for i in range(1, 10):
            t += 0.5
            reactor._monotonic = t
            enabled_buf._evaluate_and_drive(t)

        # Feed time should now exceed full_zone_timeout (3.0s default)
        assert enabled_buf._full_zone_feed_time >= enabled_buf.full_zone_timeout
        assert enabled_buf.motor_direction == BACK
        assert enabled_buf.state == STATE_RETRACTING


class TestFullZoneLeaving:
    def test_full_middle_overlap_clears_full_zone(self, enabled_buf, reactor):
        set_sensors(enabled_buf, full=True)
        reactor._monotonic = 10.0
        enabled_buf.extruder_velocity = 5.0
        enabled_buf._evaluate_and_drive(10.0)
        assert enabled_buf._in_full_zone is True

        # Transition to overlap
        set_sensors(enabled_buf, full=True, middle=True)
        enabled_buf._evaluate_and_drive(11.0)
        assert enabled_buf._in_full_zone is False
        assert enabled_buf._full_zone_feed_time == 0.0
