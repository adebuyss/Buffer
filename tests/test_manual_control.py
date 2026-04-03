"""Tests for manual feed/retract button overrides."""
import pytest
from conftest import (
    FORWARD, BACK, STOP,
    STATE_IDLE, STATE_STOPPED, STATE_MANUAL_FEED, STATE_MANUAL_RETRACT,
    STATE_ERROR,
)


class TestFeedButton:
    def test_press_starts_feeding(self, buf, buttons, reactor):
        reactor._monotonic = 10.0
        # Press feed button (state=1 means pressed for inverted pin)
        buttons.callbacks['PE4'](10.0, 1)
        assert buf.state == STATE_MANUAL_FEED
        assert buf.motor_direction == FORWARD

    def test_release_stops(self, buf, buttons, reactor):
        reactor._monotonic = 10.0
        buttons.callbacks['PE4'](10.0, 1)
        assert buf.state == STATE_MANUAL_FEED

        buttons.callbacks['PE4'](10.5, 0)
        assert buf.state == STATE_IDLE
        assert buf.motor_direction == STOP

    def test_release_with_auto_enabled(self, enabled_buf, buttons, reactor):
        reactor._monotonic = 10.0
        buttons.callbacks['PE4'](10.0, 1)
        assert enabled_buf.state == STATE_MANUAL_FEED

        buttons.callbacks['PE4'](10.5, 0)
        assert enabled_buf.state == STATE_STOPPED

    def test_press_in_error_ignored(self, buf, buttons, reactor):
        buf.state = STATE_ERROR
        reactor._monotonic = 10.0
        buttons.callbacks['PE4'](10.0, 1)
        assert buf.state == STATE_ERROR


class TestRetractButton:
    def test_press_starts_retracting(self, buf, buttons, reactor):
        reactor._monotonic = 10.0
        buttons.callbacks['PE5'](10.0, 1)
        assert buf.state == STATE_MANUAL_RETRACT
        assert buf.motor_direction == BACK

    def test_release_stops(self, buf, buttons, reactor):
        reactor._monotonic = 10.0
        buttons.callbacks['PE5'](10.0, 1)
        buttons.callbacks['PE5'](10.5, 0)
        assert buf.state == STATE_IDLE
        assert buf.motor_direction == STOP

    def test_press_in_error_ignored(self, buf, buttons, reactor):
        buf.state = STATE_ERROR
        buttons.callbacks['PE5'](10.0, 1)
        assert buf.state == STATE_ERROR


class TestManualBlocksAuto:
    def test_manual_feed_blocks_evaluate(self, enabled_buf, reactor):
        from conftest import set_sensors
        enabled_buf.state = STATE_MANUAL_FEED
        enabled_buf.motor_direction = FORWARD
        set_sensors(enabled_buf, full=True)
        enabled_buf.extruder_velocity = 0.0
        enabled_buf._evaluate_and_drive(10.0)
        # State should still be manual, not changed by evaluate
        assert enabled_buf.state == STATE_MANUAL_FEED

    def test_manual_retract_blocks_evaluate(self, enabled_buf, reactor):
        from conftest import set_sensors
        enabled_buf.state = STATE_MANUAL_RETRACT
        enabled_buf.motor_direction = BACK
        set_sensors(enabled_buf, empty=True)
        enabled_buf.extruder_velocity = 10.0
        enabled_buf._evaluate_and_drive(10.0)
        assert enabled_buf.state == STATE_MANUAL_RETRACT
