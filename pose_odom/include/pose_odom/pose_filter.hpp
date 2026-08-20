#pragma once

// Dead reckoning + a reject-and-confirm gate for Cartographer's map pose.
//
// WHY THIS EXISTS
// ---------------
// The previous filter discarded an implausible pose increment and then, in the
// very next statement, slewed straight toward the same jumped pose at
// correction_linear_rate. A 0.6 m pose-graph flip therefore still reached the
// controller, spread over ~1.7 s instead of arriving in one step. Measured in
// full_vehicle_after_msg_fix_02: the separation between /tracked_pose and
// /state_estimation/odom opened to 0.696 m at t=14.09 s and decayed back to the
// 0.250 m lever-arm baseline by t=15.5 s. The warning fired; the jump landed.
//
// Rejecting a correction is only possible if there is something to coast on.
// Hence DeadReckoner: ERPM-derived ground speed plus bias-corrected gyro yaw
// rate, integrated forward. That same dead-reckoned pose is what an "odom"
// frame is supposed to be, which is what makes AMCL possible downstream.
//
// WHAT THIS CAN AND CANNOT DO
// ---------------------------
// It fully rejects transient corrections. It CANNOT distinguish a false
// pose-graph constraint from a genuine relocalization: both observed flips were
// stable constant offsets held for tens of seconds, and no signal in the pose
// stream separates them. After hold_timeout the offset is accepted and
// trusted() goes false so the state machine can react. Bounding a bistable flip
// is mitigation; the cure is upstream (Cartographer constraint config, or not
// using a pose graph for race-time localization).
//
// No ROS types here, so it is unit-testable standalone. See
// test/pose_filter_test.cpp.

#include <algorithm>
#include <cmath>

namespace pose_odom
{

inline double wrapPi(double a)
{
    return std::atan2(std::sin(a), std::cos(a));
}

// ---------------------------------------------------------------------------

struct DeadReckonParams
{
    /// Sign applied to the raw gyro z reading to obtain REP-103 yaw rate
    /// (+z = counter-clockwise). Verified +1.0 on this vehicle: rotating the
    /// car counter-clockwise by hand produced positive angular_velocity.z.
    double gyro_z_sign{1.0};

    /// Stationary gyro bias is estimated whenever the wheels report stopped.
    /// Measured on this IMU at rest: ~0.00103 rad/s, i.e. 3.5 deg/min. Left
    /// uncorrected, a 5 s coast accrues ~0.3 deg of heading error; over a
    /// 169 s run it accounts for the ~4.4 deg of accumulated map->odom yaw
    /// correction seen in the bag.
    bool   estimate_gyro_bias{true};
    double gyro_bias_alpha{0.002};    ///< LPF rate while stationary
    double stationary_speed{0.05};    ///< [m/s] below this, wheels are stopped
    double max_gyro_bias{0.05};       ///< [rad/s] refuse absurd estimates

    double max_dt{0.10};              ///< [s] ignore longer integration steps
};

/// Integrates ground speed and yaw rate into a pose. Never corrected.
class DeadReckoner
{
public:
    explicit DeadReckoner(DeadReckonParams p = {}) : params_(p) {}

    void setParams(const DeadReckonParams& p) { params_ = p; }
    const DeadReckonParams& params() const { return params_; }

    /// Feed a raw gyro z reading [rad/s] and the current wheel speed [m/s].
    /// Updates the bias estimate when the wheels report stationary.
    void updateGyro(double raw_wz, double wheel_speed)
    {
        const double signed_wz = params_.gyro_z_sign * raw_wz;
        if (params_.estimate_gyro_bias &&
            std::abs(wheel_speed) < params_.stationary_speed)
        {
            const double next =
                gyro_bias_ + params_.gyro_bias_alpha * (signed_wz - gyro_bias_);
            if (std::abs(next) <= params_.max_gyro_bias) { gyro_bias_ = next; }
            bias_samples_++;
        }
        yaw_rate_ = signed_wz - gyro_bias_;
        have_gyro_ = true;
    }

    /// Integrate forward by dt using the latest yaw rate and the given speed.
    /// Midpoint heading keeps a constant-radius turn from bowing outward.
    void predict(double dt, double speed)
    {
        if (!(dt > 0.0) || dt > params_.max_dt) { return; }
        const double half = yaw + 0.5 * yaw_rate_ * dt;
        x   += speed * std::cos(half) * dt;
        y   += speed * std::sin(half) * dt;
        yaw  = wrapPi(yaw + yaw_rate_ * dt);
    }

    void reset(double x0, double y0, double yaw0)
    {
        x = x0; y = y0; yaw = yaw0;
    }

    double yawRate()      const { return yaw_rate_; }
    double gyroBias()     const { return gyro_bias_; }
    long   biasSamples()  const { return bias_samples_; }
    bool   haveGyro()     const { return have_gyro_; }

    double x{0.0}, y{0.0}, yaw{0.0};

private:
    DeadReckonParams params_;
    double yaw_rate_{0.0};
    double gyro_bias_{0.0};
    long   bias_samples_{0};
    bool   have_gyro_{false};
};

// ---------------------------------------------------------------------------

enum class GateState
{
    Init,        ///< no map pose accepted yet
    Tracking,    ///< innovation small; following Cartographer normally
    Holding,     ///< implausible correction; coasting on dead reckoning
    Reacquiring  ///< correction confirmed; slewing onto it quickly
};

inline const char* toString(GateState s)
{
    switch (s)
    {
        case GateState::Init:        return "INIT";
        case GateState::Tracking:    return "TRACKING";
        case GateState::Holding:     return "HOLDING";
        case GateState::Reacquiring: return "REACQUIRING";
    }
    return "UNKNOWN";
}

struct GateParams
{
    /// Reject a map pose that disagrees with dead reckoning by more than this.
    /// Chosen from the bag: the observed flips were 0.585 m and 0.618 m, while
    /// legitimate correction of dead-reckoning drift between updates is well
    /// under 0.1 m at the 200 Hz /tracked_pose rate. 0.25 m sits between them.
    double max_innovation{0.25};       ///< [m]
    double max_yaw_innovation{0.15};   ///< [rad]

    /// Normal slew rates onto the map pose while Tracking.
    double correction_linear_rate{0.35};   ///< [m/s]
    double correction_angular_rate{0.60};  ///< [rad/s]

    /// Faster slew used once a correction has been confirmed. Still bounded so
    /// the controller sees a ramp rather than a teleport.
    double reacquire_linear_rate{1.50};    ///< [m/s]
    double reacquire_angular_rate{1.50};   ///< [rad/s]

    /// How long a coherent offset must persist before it is accepted. Dead
    /// reckoning is good for far longer than this (ERPM scale error ~2% and
    /// bias-corrected gyro drift give roughly 0.15 m over 5 s at 1.5 m/s), so
    /// this can be generous. A transient never survives it.
    double hold_timeout{5.0};          ///< [s]

    /// While Holding, the offset must stay this stable to count as the same
    /// correction. A wandering offset restarts the confirmation timer, which
    /// keeps scan-match jitter from accumulating toward acceptance.
    double consistency_tol{0.15};      ///< [m]

    double max_dt{0.10};               ///< [s]
};

/// Gates Cartographer's map pose against a dead-reckoned prediction.
class JumpGate
{
public:
    explicit JumpGate(GateParams p = {}) : params_(p) {}

    void setParams(const GateParams& p) { params_ = p; }
    const GateParams& params() const { return params_; }

    /// Advance the gated pose by dead reckoning. Call this at sensor rate.
    void predict(double dt, double speed, double yaw_rate)
    {
        if (!(dt > 0.0) || dt > params_.max_dt) { return; }
        const double half = yaw_ + 0.5 * yaw_rate * dt;
        x_   += speed * std::cos(half) * dt;
        y_   += speed * std::sin(half) * dt;
        yaw_  = wrapPi(yaw_ + yaw_rate * dt);
        if (state_ == GateState::Holding) { hold_elapsed_ += dt; }
    }

    /// Fuse a map pose measurement. dt is the interval since the previous one.
    void update(double mx, double my, double myaw, double dt)
    {
        const double ex = mx - x_;
        const double ey = my - y_;
        const double d  = std::hypot(ex, ey);
        const double eyaw = wrapPi(myaw - yaw_);
        innovation_ = d;
        yaw_innovation_ = eyaw;

        if (state_ == GateState::Init)
        {
            x_ = mx; y_ = my; yaw_ = myaw;
            state_ = GateState::Tracking;
            return;
        }
        if (!(dt > 0.0) || dt > params_.max_dt)
        {
            // Timestamp stalled or regressed. Hold the last continuous output
            // rather than fusing across an unknown interval.
            return;
        }

        const bool plausible = d <= params_.max_innovation &&
                               std::abs(eyaw) <= params_.max_yaw_innovation;

        switch (state_)
        {
            case GateState::Tracking:
                if (plausible)
                {
                    slew(ex, ey, eyaw, dt,
                         params_.correction_linear_rate,
                         params_.correction_angular_rate);
                }
                else
                {
                    // Do NOT slew. This is the whole point: coast instead.
                    state_ = GateState::Holding;
                    hold_elapsed_ = 0.0;
                    held_ex_ = ex; held_ey_ = ey;
                    rejected_count_++;
                }
                break;

            case GateState::Holding:
                if (plausible)
                {
                    // The correction evaporated: it was transient, and it never
                    // reached the controller.
                    state_ = GateState::Tracking;
                    resolved_transient_++;
                    slew(ex, ey, eyaw, dt,
                         params_.correction_linear_rate,
                         params_.correction_angular_rate);
                }
                else if (std::hypot(ex - held_ex_, ey - held_ey_) >
                         params_.consistency_tol)
                {
                    // Offset is wandering, so it is not one coherent
                    // relocalization. Restart confirmation.
                    held_ex_ = ex; held_ey_ = ey;
                    hold_elapsed_ = 0.0;
                }
                else if (hold_elapsed_ >= params_.hold_timeout)
                {
                    state_ = GateState::Reacquiring;
                    accepted_count_++;
                }
                break;

            case GateState::Reacquiring:
                slew(ex, ey, eyaw, dt,
                     params_.reacquire_linear_rate,
                     params_.reacquire_angular_rate);
                if (plausible) { state_ = GateState::Tracking; }
                break;

            case GateState::Init:
                break;
        }
    }

    /// Force the gated pose onto a known value (startup, manual relocalize).
    void reset(double x0, double y0, double yaw0)
    {
        x_ = x0; y_ = y0; yaw_ = yaw0;
        state_ = GateState::Tracking;
        hold_elapsed_ = 0.0;
        innovation_ = 0.0;
        yaw_innovation_ = 0.0;
    }

    double x()   const { return x_; }
    double y()   const { return y_; }
    double yaw() const { return yaw_; }

    GateState state()      const { return state_; }
    /// False whenever the published pose is not currently following the map.
    bool   trusted()       const { return state_ == GateState::Tracking; }
    double innovation()    const { return innovation_; }
    double yawInnovation() const { return yaw_innovation_; }
    double holdElapsed()   const { return hold_elapsed_; }

    long rejectedCount()    const { return rejected_count_; }
    long acceptedCount()    const { return accepted_count_; }
    long transientCount()   const { return resolved_transient_; }

private:
    void slew(double ex, double ey, double eyaw, double dt,
              double lin_rate, double ang_rate)
    {
        const double norm = std::hypot(ex, ey);
        const double max_lin = lin_rate * dt;
        if (norm > 0.0 && max_lin > 0.0)
        {
            const double s = std::min(1.0, max_lin / norm);
            x_ += s * ex;
            y_ += s * ey;
        }
        const double max_ang = ang_rate * dt;
        yaw_ = wrapPi(yaw_ + std::clamp(eyaw, -max_ang, max_ang));
    }

    GateParams params_;

    double x_{0.0}, y_{0.0}, yaw_{0.0};
    GateState state_{GateState::Init};

    double hold_elapsed_{0.0};
    double held_ex_{0.0}, held_ey_{0.0};
    double innovation_{0.0};
    double yaw_innovation_{0.0};

    long rejected_count_{0};
    long accepted_count_{0};
    long resolved_transient_{0};
};

} // namespace pose_odom
