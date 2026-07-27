"""Logitech F710 gamepad reader using evdev."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class GamepadSnapshot:
    right_y: float = 0.0
    throttle: float = 0.0
    steering: float = 0.0
    brake: float = 0.0
    forward_pressed: bool = False
    reverse_pressed: bool = False
    takeover_pressed: bool = False
    connected: bool = False
    updated_at: float = 0.0


class F710Gamepad:
    """Threaded evdev reader for the Logitech F710.

    On the tested F710 mapping:

    - event code 1 / ABS_Y: left stick Y, raw signed throttle axis
    - event code 3 / ABS_RX: right stick X, steering placeholder
    - event code 4 / ABS_RY: right stick Y, retained for diagnostics
    - event code 2 / ABS_Z: LT brake
    - event code 304 / BTN_SOUTH: A, forward gear
    - event code 305 / BTN_EAST: B, reverse gear
    """

    def __init__(self, device: str = "/dev/input/event1"):
        self.device = device
        self._lock = threading.Lock()
        self._snapshot = GamepadSnapshot()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._axis_ranges: dict[int, tuple[int, int]] = {}

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="f710-evdev", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def snapshot(self) -> GamepadSnapshot:
        with self._lock:
            return self._snapshot

    def _set_snapshot(self, **updates) -> None:
        with self._lock:
            data = self._snapshot.__dict__.copy()
            data.update(updates)
            data["updated_at"] = time.monotonic()
            self._snapshot = GamepadSnapshot(**data)

    def _axis_unit(self, code: int, value: int, center_zero: bool = True) -> float:
        low, high = self._axis_ranges.get(code, (0, 255))
        if high <= low:
            return 0.0
        if center_zero:
            center = (low + high) / 2.0
            scale = (high - low) / 2.0
            return max(-1.0, min(1.0, (value - center) / scale))
        return max(0.0, min(1.0, (value - low) / (high - low)))

    def _run(self) -> None:
        try:
            from evdev import InputDevice, ecodes
        except ImportError as exc:
            self._set_snapshot(connected=False)
            raise RuntimeError("evdev is required on Jetson/Linux: pip install evdev") from exc

        device = InputDevice(self.device)
        self._axis_ranges = {
            code: (info.min, info.max)
            for code, info in device.capabilities(absinfo=True).get(ecodes.EV_ABS, [])
        }
        self._set_snapshot(connected=True)

        for event in device.read_loop():
            if self._stop_event.is_set():
                break
            if event.type == ecodes.EV_ABS:
                if event.code == ecodes.ABS_Y:
                    # Keep the raw stick direction: up is negative, down is positive.
                    self._set_snapshot(throttle=self._axis_unit(event.code, event.value))
                elif event.code == ecodes.ABS_RY:
                    # Keep the raw stick direction: up is negative, down is positive.
                    self._set_snapshot(right_y=self._axis_unit(event.code, event.value))
                elif event.code == ecodes.ABS_RX:
                    self._set_snapshot(steering=self._axis_unit(event.code, event.value))
                elif event.code == ecodes.ABS_Z:
                    self._set_snapshot(brake=self._axis_unit(event.code, event.value, False))
            elif event.type == ecodes.EV_KEY:
                if event.code == ecodes.BTN_SOUTH:
                    self._set_snapshot(forward_pressed=bool(event.value))
                elif event.code == ecodes.BTN_EAST:
                    self._set_snapshot(reverse_pressed=bool(event.value))
                elif event.code == 308:
                    # X on this F710 reports raw code 308. Matched numerically
                    # because the ecodes.BTN_WEST/BTN_NORTH constants disagree
                    # with the device's labels on some evdev builds.
                    self._set_snapshot(takeover_pressed=bool(event.value))
