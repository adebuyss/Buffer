"""Tests for manual feed/retract button overrides."""
import pytest
from conftest import (
    FORWARD, BACK, STOP,
    STATE_IDLE, STATE_STOPPED, STATE_FEEDING, STATE_MANUAL_FEED,
    STATE_MANUAL_RETRACT, STATE_ERROR,
    set_sensors,
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
        assert enabled_buf.state == STATE_IDLE
        assert enabled_buf.auto_enabled is False

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
        enabled_buf.state = STATE_MANUAL_FEED
        enabled_buf.motor_direction = FORWARD
        set_sensors(enabled_buf, full=True)
        enabled_buf.extruder_velocity = 0.0
        enabled_buf._evaluate_and_drive(10.0)
        # State should still be manual, not changed by evaluate
        assert enabled_buf.state == STATE_MANUAL_FEED

    def test_manual_retract_blocks_evaluate(self, enabled_buf, reactor):
        enabled_buf.state = STATE_MANUAL_RETRACT
        enabled_buf.motor_direction = BACK
        set_sensors(enabled_buf, empty=True)
        enabled_buf.extruder_velocity = 10.0
        enabled_buf._evaluate_and_drive(10.0)
        assert enabled_buf.state == STATE_MANUAL_RETRACT


class TestButtonReleaseDisablesAuto:
    def test_retract_release_disables_auto(self, enabled_buf, buttons,
                                           reactor):
        reactor._monotonic = 10.0
        buttons.callbacks['PE5'](10.0, 1)
        assert enabled_buf.state == STATE_MANUAL_RETRACT

        buttons.callbacks['PE5'](10.5, 0)
        assert enabled_buf.auto_enabled is False
        assert enabled_buf.state == STATE_IDLE

    def test_feed_release_disables_auto(self, enabled_buf, buttons, reactor):
        reactor._monotonic = 10.0
        buttons.callbacks['PE4'](10.0, 1)
        assert enabled_buf.state == STATE_MANUAL_FEED

        buttons.callbacks['PE4'](10.5, 0)
        assert enabled_buf.auto_enabled is False
        assert enabled_buf.state == STATE_IDLE

    def test_retract_release_prevents_burst(self, enabled_buf, buttons,
                                            reactor):
        """After manual retract release, burst mode should not trigger
        even with empty sensor active."""
        set_sensors(enabled_buf, empty=True)
        reactor._monotonic = 10.0
        buttons.callbacks['PE5'](10.0, 1)
        buttons.callbacks['PE5'](10.5, 0)
        assert enabled_buf.auto_enabled is False

        # Advance time past burst delay — should NOT trigger burst
        reactor._monotonic = 12.0
        enabled_buf._control_timer_cb(12.0)
        assert enabled_buf.motor_direction == STOP
        assert enabled_buf.state == STATE_IDLE

    def test_material_reinsert_reenables_after_retract(
            self, enabled_buf, buttons, reactor):
        """Material callback re-enables auto after manual retract."""
        reactor._monotonic = 10.0
        buttons.callbacks['PE5'](10.0, 1)
        buttons.callbacks['PE5'](10.5, 0)
        assert enabled_buf.auto_enabled is False

        # Simulate filament removal then reinsertion
        enabled_buf._material_callback(11.0, 0)  # remove
        enabled_buf._material_callback(12.0, 1)  # insert
        assert enabled_buf.auto_enabled is True
        assert enabled_buf.state == STATE_STOPPED


class TestManualFeedAutoStop:
    def test_stops_on_sustained_full(self, enabled_buf, reactor):
        """Manual feed stops after full sensor sustained for timeout."""
        enabled_buf.state = STATE_MANUAL_FEED
        enabled_buf.motor_direction = FORWARD
        set_sensors(enabled_buf, full=True)

        # First tick: starts timing
        reactor._monotonic = 10.0
        enabled_buf._control_timer_cb(10.0)
        assert enabled_buf.state == STATE_MANUAL_FEED

        # Advance past 5s timeout
        reactor._monotonic = 15.5
        enabled_buf._control_timer_cb(15.5)
        assert enabled_buf.state == STATE_IDLE
        assert enabled_buf.motor_direction == STOP
        assert enabled_buf.auto_enabled is False

    def test_no_stop_on_brief_full(self, enabled_buf, reactor):
        """Full sensor for less than timeout should not stop feed."""
        enabled_buf.state = STATE_MANUAL_FEED
        enabled_buf.motor_direction = FORWARD
        set_sensors(enabled_buf, full=True)

        reactor._monotonic = 10.0
        enabled_buf._control_timer_cb(10.0)

        # 2s later — still within timeout
        reactor._monotonic = 12.0
        enabled_buf._control_timer_cb(12.0)
        assert enabled_buf.state == STATE_MANUAL_FEED

        # Sensor clears
        set_sensors(enabled_buf, full=False)
        reactor._monotonic = 12.5
        enabled_buf._control_timer_cb(12.5)
        assert enabled_buf.state == STATE_MANUAL_FEED

    def test_full_timer_resets_on_clear(self, enabled_buf, reactor):
        """Sensor toggle resets the timeout counter."""
        enabled_buf.state = STATE_MANUAL_FEED
        enabled_buf.motor_direction = FORWARD
        set_sensors(enabled_buf, full=True)

        reactor._monotonic = 10.0
        enabled_buf._control_timer_cb(10.0)

        # 3s in, sensor clears
        reactor._monotonic = 13.0
        set_sensors(enabled_buf, full=False)
        enabled_buf._control_timer_cb(13.0)

        # Sensor re-triggers
        reactor._monotonic = 14.0
        set_sensors(enabled_buf, full=True)
        enabled_buf._control_timer_cb(14.0)

        # 4s after re-trigger (< 5s timeout) — should still be feeding
        reactor._monotonic = 18.0
        enabled_buf._control_timer_cb(18.0)
        assert enabled_buf.state == STATE_MANUAL_FEED

        # 5s+ after re-trigger — NOW it should stop
        reactor._monotonic = 19.5
        enabled_buf._control_timer_cb(19.5)
        assert enabled_buf.state == STATE_IDLE
        assert enabled_buf.motor_direction == STOP

    def test_buffer_feed_gcode_also_auto_stops(self, enabled_buf, reactor,
                                               gcode):
        """cmd_BUFFER_FEED should also auto-stop on sustained full."""
        from conftest import MockGcmd
        gcmd = MockGcmd(params={})
        enabled_buf.cmd_BUFFER_FEED(gcmd)
        assert enabled_buf.state == STATE_MANUAL_FEED

        set_sensors(enabled_buf, full=True)
        reactor._monotonic = 10.0
        enabled_buf._control_timer_cb(10.0)

        reactor._monotonic = 15.5
        enabled_buf._control_timer_cb(15.5)
        assert enabled_buf.state == STATE_IDLE
        assert enabled_buf.auto_enabled is False

    def test_auto_stop_sends_message(self, enabled_buf, reactor, gcode):
        """Auto-stop should send an info message."""
        enabled_buf.state = STATE_MANUAL_FEED
        enabled_buf.motor_direction = FORWARD
        set_sensors(enabled_buf, full=True)

        reactor._monotonic = 10.0
        enabled_buf._control_timer_cb(10.0)
        reactor._monotonic = 15.5
        enabled_buf._control_timer_cb(15.5)

        assert any("manual feed stopped" in r for r in gcode.responses)
