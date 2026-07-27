"""Gamepad state to VESC RPM command mapping."""

from __future__ import annotations

import time
from dataclasses import dataclass

from .gamepad import GamepadSnapshot


@dataclass(frozen=True)
class ControlConfig:
    max_rpm: int = 5000
    deadzone: float = 0.08
    smoothing: float = 0.15
    stale_timeout: float = 0.0


@dataclass(frozen=True)
class ControlOutput:
    rpm: int
    gear: int
    throttle: float
    brake: float
    steering: float
    stale: bool


class ManualController:
    """Applies gear selection, deadzone, braking, and RPM smoothing."""

    def __init__(self, config: ControlConfig | None = None):
        self.config = config or ControlConfig()
        self._gear = 1
        self._filtered_rpm = 0.0

    @property
    def gear(self) -> int:
        return self._gear

    def reset(self) -> None:
        self._gear = 1
        self._filtered_rpm = 0.0

    def update(self, pad: GamepadSnapshot) -> ControlOutput:
        if pad.forward_pressed:
            self._gear = 1
        if pad.reverse_pressed:
            self._gear = -1

        stale = not pad.connected
        if self.config.stale_timeout > 0.0 and pad.updated_at > 0.0:
            stale = stale or time.monotonic() - pad.updated_at > self.config.stale_timeout
        if stale:
            target_rpm = 0.0
            throttle = 0.0
            brake = max(0.0, min(1.0, pad.brake))
        else:
            throttle = self._apply_deadzone(max(0.0, -self._gear * pad.throttle))
            brake = max(0.0, min(1.0, pad.brake))
            target_rpm = self._gear * throttle * self.config.max_rpm
            target_rpm *= 1.0 - brake

        alpha = max(0.0, min(1.0, self.config.smoothing))
        self._filtered_rpm = (1.0 - alpha) * self._filtered_rpm + alpha * target_rpm
        if abs(self._filtered_rpm) < 1.0:
            self._filtered_rpm = 0.0

        return ControlOutput(
            rpm=int(round(self._filtered_rpm)),
            gear=self._gear,
            throttle=throttle,
            brake=brake,
            steering=self._apply_deadzone(pad.steering),
            stale=stale,
        )

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self.config.deadzone:
            return 0.0
        if value > 0:
            return (value - self.config.deadzone) / (1.0 - self.config.deadzone)
        return (value + self.config.deadzone) / (1.0 - self.config.deadzone)
