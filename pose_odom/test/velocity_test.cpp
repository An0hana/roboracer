// Validates the fusion core against synthetic trajectories.
//
// Build:
//   g++ -O2 -std=c++17 -Iinclude test/velocity_test.cpp -o /tmp/vtest

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "pose_odom/velocity_estimator.hpp"

using namespace pose_odom;

namespace
{

double erpmForSpeed(double v, const VelocityParams& p)
{
    const double gain = 2.0 * M_PI * p.wheel_radius /
                        (60.0 * p.pole_pairs * p.gear_ratio);
    return v / gain;
}

struct Summary
{
    double mean{0.0};
    double stddev{0.0};
    double max_abs_err{0.0};
};

Summary summarize(const std::vector<double>& est, double truth)
{
    Summary s;
    for (double e : est) { s.mean += e; }
    s.mean /= est.size();
    for (double e : est)
    {
        s.stddev += (e - s.mean) * (e - s.mean);
        s.max_abs_err = std::max(s.max_abs_err, std::abs(e - truth));
    }
    s.stddev = std::sqrt(s.stddev / est.size());
    return s;
}

} // namespace

int main()
{
    VelocityParams p;
    p.wheel_radius = 0.05017;
    p.gear_ratio   = 8.55;
    p.pole_pairs   = 3.0;

    printf("erpm_to_speed_gain (pp=3): %.6e m/s per ERPM\n\n",
           2.0 * M_PI * p.wheel_radius / (60.0 * p.pole_pairs * p.gear_ratio));

    const double dt_pose = 1.0 / 200.0; // tracked_pose at 200 Hz
    const double dt_rpm  = 1.0 / 50.0;  // /sensors/core at ~50 Hz

    std::mt19937 rng(1);

    // ---- Case 1: constant speed, noisy pose, clean RPM ------------------
    for (double truth : {1.0, 3.0, -2.0})
    {
        VelocityEstimator est(p);
        std::normal_distribution<double> pose_noise(0.0, 0.01); // 1 cm jitter

        double x = 0.0, y = 0.0;
        const double yaw = 0.3;
        double t = 0.0;
        double last_rpm_t = 0.0;
        std::vector<double> out;

        for (int i = 0; i < 2000; ++i)
        {
            t += dt_pose;
            x += truth * std::cos(yaw) * dt_pose;
            y += truth * std::sin(yaw) * dt_pose;

            if (t - last_rpm_t >= dt_rpm)
            {
                est.updateRpm(erpmForSpeed(truth, p));
                last_rpm_t = t;
            }

            est.updatePose(x + pose_noise(rng), y + pose_noise(rng), yaw, t);
            if (i > 400) { out.push_back(est.speed()); } // skip warm-up
        }
        const Summary s = summarize(out, truth);
        printf("constant %+5.1f m/s : mean %+6.3f  std %.4f  maxerr %.4f\n",
               truth, s.mean, s.stddev, s.max_abs_err);
    }

    // ---- Case 2: RPM scale is 15%% high, pose corrects it ---------------
    {
        const double truth = 2.0;
        VelocityParams pw = p;
        pw.pose_weight = 0.15;
        VelocityEstimator est(pw);
        std::normal_distribution<double> pose_noise(0.0, 0.01);

        double x = 0.0, y = 0.0, t = 0.0, last_rpm_t = 0.0;
        const double yaw = 0.0;
        std::vector<double> out, pure_rpm;

        for (int i = 0; i < 3000; ++i)
        {
            t += dt_pose;
            x += truth * dt_pose;
            if (t - last_rpm_t >= dt_rpm)
            {
                est.updateRpm(erpmForSpeed(truth * 1.15, p)); // 15% too high
                last_rpm_t = t;
            }
            est.updatePose(x + pose_noise(rng), y, yaw, t);
            if (i > 600) { out.push_back(est.speed()); pure_rpm.push_back(truth * 1.15); }
        }
        const Summary s = summarize(out, truth);
        printf("\nRPM +15%% scale err, pose_weight 0.15:\n");
        printf("  pure RPM would read %.3f; fused mean %.3f (truth %.1f)\n",
               truth * 1.15, s.mean, truth);
        printf("  -> pose pulls %.0f%% of the error out\n",
               100.0 * (truth * 1.15 - s.mean) / (truth * 1.15 - truth));
    }

    // ---- Case 3: direction from pose when ERPM sign is wrong ------------
    {
        const double truth = -1.5; // reversing
        VelocityEstimator est(p);
        double x = 0.0, y = 0.0, t = 0.0, last_rpm_t = 0.0;
        const double yaw = 0.0;
        int correct_sign = 0, total = 0;

        for (int i = 0; i < 1500; ++i)
        {
            t += dt_pose;
            x += truth * dt_pose;
            if (t - last_rpm_t >= dt_rpm)
            {
                // ERPM reported unsigned (magnitude only) -- a real failure
                // mode of sensorless VESC at low speed.
                est.updateRpm(std::abs(erpmForSpeed(truth, p)));
                last_rpm_t = t;
            }
            est.updatePose(x, y, yaw, t);
            if (i > 400) { total++; if (sgn(est.speed()) == sgn(truth)) correct_sign++; }
        }
        printf("\nreverse with unsigned ERPM: correct direction %d%% of the time\n",
               100 * correct_sign / total);
        printf("  final fused speed %.3f (truth %.1f)\n", est.speed(), truth);
    }

    // ---- Case 4: yaw rate on a steady turn ------------------------------
    {
        VelocityEstimator est(p);
        const double v = 2.0, R = 2.0;      // radius 2 m
        const double omega = v / R;         // 1.0 rad/s
        double x = 0.0, y = 0.0, yaw = 0.0, t = 0.0, last_rpm_t = 0.0;
        std::vector<double> out;

        for (int i = 0; i < 2000; ++i)
        {
            t += dt_pose;
            yaw = normalizeAngle(yaw + omega * dt_pose);
            x += v * std::cos(yaw) * dt_pose;
            y += v * std::sin(yaw) * dt_pose;
            if (t - last_rpm_t >= dt_rpm)
            {
                est.updateRpm(erpmForSpeed(v, p));
                last_rpm_t = t;
            }
            est.updatePose(x, y, yaw, t);
            if (i > 400) { out.push_back(est.yawRate()); }
        }
        const Summary s = summarize(out, omega);
        printf("\nsteady turn omega %.2f rad/s: yaw-rate mean %.3f  std %.4f\n",
               omega, s.mean, s.stddev);
    }

    return 0;
}
