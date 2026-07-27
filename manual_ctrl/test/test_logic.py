"""Tests the takeover edge-toggle and mux gating without ROS.

Reimplements only the decision logic (not the rclpy plumbing) and checks it
against the sequences that matter for safety.
"""

import sys


# --- mirror of the node's rising-edge toggle ---------------------------------
class Takeover:
    def __init__(self):
        self.on = False
        self.prev = False
        self.events = []

    def step(self, x_pressed):
        if x_pressed and not self.prev:
            self.on = not self.on
            self.events.append("engage" if self.on else "release")
        self.prev = x_pressed
        return self.on


# --- mirror of the mux gating ------------------------------------------------
class Mux:
    def __init__(self):
        self.takeover = False
        self.motor_out = None

    def set_takeover(self, v):
        if v != self.takeover:
            self.takeover = v
            self.motor_out = 0.0  # stop on switch

    def auto_motor(self, v):
        if self.takeover:
            return
        self.motor_out = v

    def manual_motor(self, v):
        if not self.takeover:
            return
        self.motor_out = v


def check(name, cond):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}")
    return cond


def main():
    ok = True

    # 1. X is a rising-edge toggle, not level-triggered.
    t = Takeover()
    # X held down across many ticks = a single engage.
    for _ in range(10):
        t.step(True)
    ok &= check("held X engages once", t.events == ["engage"])
    # release then press again = toggle back off.
    t.step(False)
    t.step(True)
    ok &= check("second press releases", t.events == ["engage", "release"])

    # 2. Mux forwards autonomous only while released.
    m = Mux()
    m.auto_motor(1500.0)
    ok &= check("auto passes when released", m.motor_out == 1500.0)
    m.manual_motor(999.0)
    ok &= check("manual blocked when released", m.motor_out == 1500.0)

    # 3. On engage, motor is zeroed regardless of last autonomous value.
    m.set_takeover(True)
    ok &= check("engage zeroes motor", m.motor_out == 0.0)

    # 4. Now manual passes, autonomous is locked out.
    m.manual_motor(300.0)
    ok &= check("manual passes when engaged", m.motor_out == 300.0)
    m.auto_motor(5000.0)
    ok &= check("auto blocked when engaged", m.motor_out == 300.0)

    # 5. Release zeroes again and restores autonomous.
    m.set_takeover(False)
    ok &= check("release zeroes motor", m.motor_out == 0.0)
    m.auto_motor(1200.0)
    ok &= check("auto restored after release", m.motor_out == 1200.0)

    # 6. Full adversarial sequence: autonomous flooding while driver grabs it.
    m = Mux()
    t = Takeover()
    hist = []
    script = [
        # (x_pressed, auto_cmd, manual_cmd)
        (False, 2000.0, 0.0),
        (True,  2000.0, 0.0),   # engage
        (False, 3000.0, 250.0), # auto floods, manual drives
        (False, 4000.0, 250.0),
        (True,  4000.0, 250.0), # release
        (False, 1000.0, 250.0),
    ]
    for x, a, man in script:
        m.set_takeover(t.step(x))
        m.auto_motor(a)
        m.manual_motor(man)
        hist.append(m.motor_out)
    # After engage (idx>=1) until release (idx 4), output must never follow auto.
    ok &= check("during takeover, auto flood never reaches output",
                hist[2] == 250.0 and hist[3] == 250.0)
    ok &= check("after release, auto resumes", hist[5] == 1000.0)

    print("\n" + ("ALL PASS" if ok else "FAILURES ABOVE"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
