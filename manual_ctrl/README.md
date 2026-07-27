# manual_ctrl

Gamepad manual control with a **latching exclusive takeover** for the F1TENTH car.

The node stays silent in the background. Press **X** on the F710 to stop the
car instantly and seize control; press **X** again to release. While engaged,
a command mux forwards only manual commands and locks out every autonomous
source.

## Two nodes

- `manual_control_node` — reads the F710 (evdev), publishes manual motor/servo
  commands and a latched `/manual/takeover` flag.
- `manual_mux_node` — the ONLY node wired to the VESC command topics. Forwards
  autonomous commands until takeover, then forwards manual only. Has a watchdog
  that coasts the motor to zero if the active source goes silent.

## Wiring (required for exclusivity)

The mux only works if nothing else publishes to the VESC command topics.
Remap your autonomous stack to the mux inputs:

    /commands/motor/speed     -> /autonomous/motor
    /commands/servo/position  -> /autonomous/servo

The mux republishes the winning source to the real `/commands/motor/speed`
and `/commands/servo/position`.

    autonomous ─▶ /autonomous/motor ┐
                                     ├─▶ manual_mux ─▶ /commands/motor/speed ─▶ vesc_driver
    manual     ─▶ /manual/motor    ─┘        ▲
                        /manual/takeover ────┘

## Controls (F710, XInput mode — switch on the back set to X)

- **X** — toggle manual takeover (engage stops the car)
- **A** — forward gear, **B** — reverse gear
- **Left stick Y** — throttle, **Right stick X** — steering, **LT** — brake

## Run

    colcon build --packages-select manual_ctrl --symlink-install
    ros2 launch manual_ctrl manual_ctrl.launch.py

Find your gamepad device with `ls /dev/input/by-id/` and set `gamepad_device`.

## Notes

- `motor_mode` must match your autonomous units (`speed` = ERPM).
- This node never touches TF or the VESC directly; it is command-only.
