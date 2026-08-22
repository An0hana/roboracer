#pragma once

// Fuses motor-RPM speed with tracked-pose derivatives. No ROS types.
//
// CHANGE FROM THE PREVIOUS VERSION
// --------------------------------
// yawRate() used to be differentiated from /tracked_pose. That made the yaw
// rate a function of the very signal the pose gate is trying to validate: when
// Cartographer flipped, the reported yaw rate flipped with it, so there was no
// independent estimate to coast on. The gyro is now the primary yaw-rate
// source, with the pose derivative kept only as a fallback when no IMU is
// present. Bias correction lives in DeadReckoner (pose_filter.hpp), which owns
// the stationary-bias estimate; feed the corrected value in via
// updateGyroYawRate().

#include <cmath>

namespace pose_odom
{

inline double normalizeAngle(double a)
{
    while (a > M_PI)   { a -= 2.0 * M_PI; }
    while (a <= -M_PI) { a += 2.0 * M_PI; }
    return a;
}

inline int sgn(double v) { return (v > 0.0) - (v < 0.0); }

struct VelocityParams
{
    // Conversion from VESC electrical RPM to ground speed:
    //   v = erpm * erpm_to_speed_gain
    //   erpm_to_speed_gain = 2*pi*wheel_radius / (60 * pole_pairs * gear_ratio)
    // Either set the gain directly, or leave it <= 0 to have it computed
    // from the three physical quantities below.
    double erpm_to_speed_gain{0.0};
    double wheel_radius{0.05017};
    double gear_ratio{8.55};
    double pole_pairs{3.0};

    // Complementary fusion. Magnitude is mostly from RPM (low noise);
    // pose derivative supplies a slow correction for RPM scale error.
    //   speed_magnitude = (1 - pose_weight)*|v_rpm| + pose_weight*|v_pose_lp|
    double pose_weight{0.0};

    // Low-pass smoothing factors in [0,1]; larger = more responsive, noisier.
    double pose_speed_alpha{0.15};
    double yaw_rate_alpha{0.15};

    // Direction fallback used when signed motor telemetry is unavailable.
    double direction_threshold{0.15};

    // Fused speeds below this magnitude are reported as zero. When RPM is
    // also inside this band, forget the previous direction so a new movement
    // can take its sign from RPM instead of inheriting stale reverse motion.
    double speed_deadband{0.02};

    // Ignore pose samples further apart than this [s] (e.g. after a stall).
    double max_pose_dt{0.5};

    // Prefer the gyro for yaw rate. Falls back to the pose derivative
    // automatically if no gyro sample has ever arrived.
    bool prefer_gyro_yaw_rate{true};
};

class VelocityEstimator
{
public:
    explicit VelocityEstimator(VelocityParams p = {}) { setParams(p); }

    void setParams(const VelocityParams& p)
    {
        params_ = p;
        if (params_.erpm_to_speed_gain <= 0.0)
        {
            params_.erpm_to_speed_gain =
                2.0 * M_PI * params_.wheel_radius /
                (60.0 * params_.pole_pairs * params_.gear_ratio);
        }
    }

    const VelocityParams& params() const { return params_; }
    double gain() const { return params_.erpm_to_speed_gain; }

    /// Feed a raw electrical RPM reading.
    void updateRpm(double erpm)
    {
        v_rpm_ = erpm * params_.erpm_to_speed_gain;
        if (std::abs(v_rpm_) <= params_.speed_deadband)
        {
            direction_ = 0;
        }
        have_rpm_ = true;
    }

    /// Feed a bias-corrected, REP-103 signed yaw rate [rad/s] from the IMU.
    void updateGyroYawRate(double wz)
    {
        gyro_yaw_rate_ = wz;
        have_gyro_ = true;
    }

    /// Feed a map-frame pose sample with its timestamp [s].
    void updatePose(double x, double y, double yaw, double t)
    {
        if (have_pose_)
        {
            const double dt = t - last_t_;
            if (dt > 1e-4 && dt < params_.max_pose_dt)
            {
                const double dx = x - last_x_;
                const double dy = y - last_y_;

                // Signed forward speed: displacement projected on heading.
                const double v_raw = (dx * std::cos(yaw) +
                                      dy * std::sin(yaw)) / dt;
                v_pose_lp_ += params_.pose_speed_alpha * (v_raw - v_pose_lp_);

                const double yaw_raw = normalizeAngle(yaw - last_yaw_) / dt;
                yaw_rate_lp_ +=
                    params_.yaw_rate_alpha * (yaw_raw - yaw_rate_lp_);

                if (std::abs(v_pose_lp_) > params_.direction_threshold)
                {
                    direction_ = sgn(v_pose_lp_);
                }
            }
        }
        last_x_ = x; last_y_ = y; last_yaw_ = yaw; last_t_ = t;
        have_pose_ = true;
    }

    /// Fused signed ground speed [m/s].
    double speed() const
    {
        if (have_rpm_ && std::abs(v_rpm_) <= params_.speed_deadband)
        {
            return 0.0;
        }

        const double mag =
            (1.0 - params_.pose_weight) * std::abs(v_rpm_) +
            params_.pose_weight * std::abs(v_pose_lp_);
        if (mag <= params_.speed_deadband)
        {
            return 0.0;
        }

        int dir = have_rpm_ ? sgn(v_rpm_) : direction_;
        if (dir == 0) { dir = (direction_ != 0) ? direction_ : 1; }
        return dir * mag;
    }

    /// Yaw rate [rad/s]. Gyro when available, pose derivative otherwise.
    double yawRate() const
    {
        if (params_.prefer_gyro_yaw_rate && have_gyro_) { return gyro_yaw_rate_; }
        return yaw_rate_lp_;
    }

    double rpmSpeed()      const { return v_rpm_; }
    double poseSpeed()     const { return v_pose_lp_; }
    double poseYawRate()   const { return yaw_rate_lp_; }
    bool   haveGyro()      const { return have_gyro_; }
    bool   haveRpm()       const { return have_rpm_; }
    bool   ready()         const { return have_rpm_ || have_pose_; }

    void reset()
    {
        v_rpm_ = v_pose_lp_ = yaw_rate_lp_ = gyro_yaw_rate_ = 0.0;
        have_rpm_ = have_pose_ = have_gyro_ = false;
        direction_ = 0;
    }

private:
    VelocityParams params_;

    double v_rpm_{0.0};
    double v_pose_lp_{0.0};
    double yaw_rate_lp_{0.0};
    double gyro_yaw_rate_{0.0};

    double last_x_{0.0}, last_y_{0.0}, last_yaw_{0.0}, last_t_{0.0};
    bool   have_rpm_{false};
    bool   have_pose_{false};
    bool   have_gyro_{false};
    int    direction_{0};
};

} // namespace pose_odom
