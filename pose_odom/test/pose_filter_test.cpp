// Validates the pose gate against the failure actually recorded in
// full_vehicle_after_msg_fix_02.
//
// Build and run:
//   g++ -O2 -std=c++17 -Iinclude test/pose_filter_test.cpp -o /tmp/ptest && /tmp/ptest

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "pose_odom/pose_filter.hpp"

using namespace pose_odom;

namespace
{

int failures = 0;

void check(bool ok, const std::string& what)
{
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
    if (!ok) { failures++; }
}

/// Drives the gate along a straight line at `speed`, feeding it a map pose
/// that is offset by `offset` for the window [t_on, t_off).
struct Replay
{
    double max_innovation{0.0};
    double peak_error{0.0};
    double final_error{0.0};
    long   rejected{0}, accepted{0}, transient{0};
};

Replay run(double offset, double t_on, double t_off, double t_end,
           const GateParams& gp, double speed = 1.5, double rate = 200.0)
{
    JumpGate gate;
    gate.setParams(gp);

    const double dt = 1.0 / rate;
    double truth_x = 0.0;
    Replay r;

    for (double t = 0.0; t < t_end; t += dt)
    {
        truth_x += speed * dt;
        gate.predict(dt, speed, 0.0);

        const bool jumped = (t >= t_on && t < t_off);
        const double map_x = truth_x + (jumped ? offset : 0.0);
        gate.update(map_x, 0.0, 0.0, dt);

        const double err = std::abs(gate.x() - truth_x);
        r.peak_error = std::max(r.peak_error, err);
        r.max_innovation = std::max(r.max_innovation, gate.innovation());
        r.final_error = err;
    }
    r.rejected  = gate.rejectedCount();
    r.accepted  = gate.acceptedCount();
    r.transient = gate.transientCount();
    return r;
}

} // namespace

int main()
{
    GateParams gp;   // defaults as shipped in pose_odom.yaml

    std::printf("\n1. Dead reckoning accuracy over a hold\n");
    {
        // 5 s coast at 1.5 m/s with a 2%% speed scale error and the measured
        // 0.00103 rad/s gyro bias left uncorrected.
        DeadReckoner dr;
        DeadReckonParams dp;
        dp.estimate_gyro_bias = false;
        dr.setParams(dp);
        const double dt = 1.0 / 200.0, truth_v = 1.5;
        double truth_x = 0.0;
        for (double t = 0.0; t < 5.0; t += dt)
        {
            dr.updateGyro(0.00103, truth_v);   // pure bias, car going straight
            dr.predict(dt, truth_v * 1.02);    // 2% scale error
            truth_x += truth_v * dt;
        }
        const double err = std::hypot(dr.x - truth_x, dr.y);
        std::printf("     5 s coast error = %.3f m, heading = %.3f deg\n",
                    err, dr.yaw * 180.0 / M_PI);
        check(err < 0.25, "dead reckoning holds under 0.25 m over 5 s");
    }

    std::printf("\n2. Transient jump is rejected outright\n");
    {
        // A 0.6 m offset lasting 1 s: shorter than hold_timeout, so it must
        // never reach the output at all.
        Replay r = run(0.6, 3.0, 4.0, 10.0, gp);
        std::printf("     peak output error = %.3f m (innovation seen %.3f m)\n",
                    r.peak_error, r.max_innovation);
        check(r.max_innovation > 0.5, "gate observed the 0.6 m innovation");
        check(r.peak_error < 0.05,    "output never moved (transient rejected)");
        check(r.transient == 1,       "classified as transient");
        check(r.accepted == 0,        "not accepted");
    }

    std::printf("\n3. Sustained jump is held, then accepted with a warning\n");
    {
        // The bag case: 0.585 m held for 52 s. Cannot be distinguished from a
        // real relocalization, so it is accepted -- but only after
        // hold_timeout, and via a bounded slew.
        Replay r = run(0.585, 3.0, 60.0, 20.0, gp);
        std::printf("     final output error = %.3f m, accepted=%ld\n",
                    r.final_error, r.accepted);
        check(r.accepted == 1, "accepted after hold_timeout");
        check(r.final_error > 0.4,
              "output followed it (expected: gate cannot reject a stable offset)");
    }

    std::printf("\n4. Regression: the old unconditional slew is gone\n");
    {
        // Under the previous implementation a rejected increment was followed
        // immediately by a slew toward the same pose at correction_linear_rate,
        // so a 0.6 m jump landed over ~1.7 s. Check that 1 s after a jump
        // begins, the output has not moved.
        JumpGate gate; gate.setParams(gp);
        const double dt = 1.0 / 200.0, v = 1.5;
        double truth_x = 0.0;
        for (double t = 0.0; t < 4.0; t += dt)
        {
            truth_x += v * dt;
            gate.predict(dt, v, 0.0);
            gate.update(truth_x + (t >= 3.0 ? 0.6 : 0.0), 0.0, 0.0, dt);
        }
        const double drift = std::abs(gate.x() - truth_x);
        std::printf("     output error 1.0 s after jump = %.4f m "
                    "(old behaviour: ~0.35 m)\n", drift);
        check(drift < 0.02, "no slew toward a rejected pose");
    }

    std::printf("\n5. Wandering offset does not accumulate toward acceptance\n");
    {
        // Scan-match jitter that keeps changing must restart the confirmation
        // timer rather than summing up to hold_timeout.
        JumpGate gate; gate.setParams(gp);
        const double dt = 1.0 / 200.0, v = 1.0;
        double truth_x = 0.0; int n = 0;
        for (double t = 0.0; t < 30.0; t += dt, ++n)
        {
            truth_x += v * dt;
            gate.predict(dt, v, 0.0);
            // Offset alternates well beyond consistency_tol.
            const double off = (n / 200 % 2) ? 0.5 : -0.5;
            gate.update(truth_x + off, 0.0, 0.0, dt);
        }
        std::printf("     accepted=%ld after 30 s of alternating offsets\n",
                    gate.acceptedCount());
        check(gate.acceptedCount() == 0, "never accepted a wandering offset");
    }

    std::printf("\n6. Normal small corrections still pass through\n");
    {
        Replay r = run(0.08, 3.0, 60.0, 12.0, gp);
        std::printf("     final output error = %.3f m\n", r.final_error);
        check(r.rejected == 0, "sub-threshold correction never rejected");
        check(r.final_error > 0.05, "gate followed the small correction");
    }

    std::printf("\n%s (%d failure%s)\n\n",
                failures == 0 ? "ALL PASS" : "FAILURES",
                failures, failures == 1 ? "" : "s");
    return failures == 0 ? 0 : 1;
}
