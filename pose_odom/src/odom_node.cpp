#include "pose_odom/odom_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace pose_odom
{

namespace
{

double yawFromQuat(double z, double w)
{
    // Planar: roll = pitch = 0, so yaw = 2*atan2(z, w).
    return 2.0 * std::atan2(z, w);
}

} // namespace

OdomNode::OdomNode()
: rclcpp::Node("pose_odom")
{
    loadParameters();

    if (params_.map_pose_source == "tf")
    {
        // AMCL path: no pose topic exists at controller rate, so the map pose
        // is composed from TF (map->odom by AMCL, odom->base_link by us).
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
        const double period = 1.0 / std::max(1.0, params_.publish_rate);
        tf_timer_ = create_wall_timer(
            std::chrono::duration<double>(period),
            std::bind(&OdomNode::tfPoseTimer, this));
    }
    else
    {
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            params_.tracked_pose_topic, rclcpp::QoS(rclcpp::KeepLast(20)),
            std::bind(&OdomNode::poseCallback, this, std::placeholders::_1));
    }

    vesc_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>(
        params_.vesc_topic, rclcpp::QoS(rclcpp::KeepLast(20)),
        std::bind(&OdomNode::vescCallback, this, std::placeholders::_1));

    // SensorDataQoS: best-effort, matching typical IMU drivers. A reliable
    // subscription against a best-effort publisher silently receives nothing.
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        params_.imu_topic, rclcpp::SensorDataQoS(),
        std::bind(&OdomNode::imuCallback, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        params_.odom_topic, rclcpp::QoS(rclcpp::KeepLast(20)));

    trusted_pub_ = create_publisher<std_msgs::msg::Bool>(
        params_.trusted_topic, rclcpp::QoS(rclcpp::KeepLast(5)).transient_local());

    if (params_.publish_odom_tf)
    {
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        RCLCPP_INFO(get_logger(),
                    "publishing %s -> %s from dead reckoning. Make sure "
                    "Cartographer is NOT also providing that frame "
                    "(provide_odom_frame must be false).",
                    params_.odom_frame.c_str(), params_.base_frame.c_str());
    }

    RCLCPP_INFO(get_logger(),
                "pose_odom ready. pose <- %s, rpm <- %s, imu <- %s, out -> %s",
                params_.tracked_pose_topic.c_str(),
                params_.vesc_topic.c_str(),
                params_.imu_topic.c_str(),
                params_.odom_topic.c_str());
    RCLCPP_INFO(get_logger(),
                "erpm_to_speed_gain = %.6e m/s per ERPM, gyro_z_sign = %+.0f",
                estimator_.gain(), dr_.params().gyro_z_sign);
}

void OdomNode::loadParameters()
{
    params_.tracked_pose_topic =
        declare_parameter("tracked_pose_topic", params_.tracked_pose_topic);
    params_.vesc_topic = declare_parameter("vesc_topic", params_.vesc_topic);
    params_.imu_topic  = declare_parameter("imu_topic", params_.imu_topic);
    params_.odom_topic = declare_parameter("odom_topic", params_.odom_topic);
    params_.map_pose_source =
        declare_parameter("map_pose_source", params_.map_pose_source);
    if (params_.map_pose_source != "tracked_pose" &&
        params_.map_pose_source != "tf")
    {
        throw std::invalid_argument(
            "map_pose_source must be \"tracked_pose\" or \"tf\"");
    }
    params_.trusted_topic =
        declare_parameter("trusted_topic", params_.trusted_topic);

    params_.map_frame  = declare_parameter("map_frame", params_.map_frame);
    params_.odom_frame = declare_parameter("odom_frame", params_.odom_frame);
    params_.base_frame = declare_parameter("base_frame", params_.base_frame);

    params_.tracking_offset_x =
        declare_parameter("tracking_offset_x", params_.tracking_offset_x);
    params_.tracking_offset_y =
        declare_parameter("tracking_offset_y", params_.tracking_offset_y);

    params_.publish_rate =
        declare_parameter("publish_rate", params_.publish_rate);
    params_.filter_pose_jumps =
        declare_parameter("filter_pose_jumps", params_.filter_pose_jumps);
    params_.publish_odom_tf =
        declare_parameter("publish_odom_tf", params_.publish_odom_tf);
    params_.log_calibration =
        declare_parameter("log_calibration", params_.log_calibration);

    params_.position_variance =
        declare_parameter("position_variance", params_.position_variance);
    params_.yaw_variance =
        declare_parameter("yaw_variance", params_.yaw_variance);
    params_.speed_variance =
        declare_parameter("speed_variance", params_.speed_variance);
    params_.untrusted_position_variance = declare_parameter(
        "untrusted_position_variance", params_.untrusted_position_variance);
    params_.untrusted_yaw_variance = declare_parameter(
        "untrusted_yaw_variance", params_.untrusted_yaw_variance);

    // ---- velocity estimator -------------------------------------------------
    VelocityParams vp;
    vp.erpm_to_speed_gain = declare_parameter("erpm_to_speed_gain", 0.0);
    vp.wheel_radius = declare_parameter("wheel_radius", vp.wheel_radius);
    vp.gear_ratio   = declare_parameter("gear_ratio", vp.gear_ratio);
    vp.pole_pairs   = declare_parameter("pole_pairs", vp.pole_pairs);
    vp.pose_weight  = declare_parameter("pose_weight", vp.pose_weight);
    vp.pose_speed_alpha =
        declare_parameter("pose_speed_alpha", vp.pose_speed_alpha);
    vp.yaw_rate_alpha =
        declare_parameter("yaw_rate_alpha", vp.yaw_rate_alpha);
    vp.direction_threshold =
        declare_parameter("direction_threshold", vp.direction_threshold);
    vp.speed_deadband =
        declare_parameter("speed_deadband", vp.speed_deadband);
    vp.max_pose_dt = declare_parameter("max_pose_dt", vp.max_pose_dt);
    vp.prefer_gyro_yaw_rate =
        declare_parameter("prefer_gyro_yaw_rate", vp.prefer_gyro_yaw_rate);
    estimator_.setParams(vp);

    // ---- dead reckoner ------------------------------------------------------
    DeadReckonParams dp;
    dp.gyro_z_sign = declare_parameter("gyro_z_sign", dp.gyro_z_sign);
    dp.estimate_gyro_bias =
        declare_parameter("estimate_gyro_bias", dp.estimate_gyro_bias);
    dp.gyro_bias_alpha =
        declare_parameter("gyro_bias_alpha", dp.gyro_bias_alpha);
    dp.stationary_speed =
        declare_parameter("stationary_speed", dp.stationary_speed);
    dp.max_gyro_bias = declare_parameter("max_gyro_bias", dp.max_gyro_bias);
    dp.max_dt = declare_parameter("max_filter_dt", dp.max_dt);
    dr_.setParams(dp);

    // ---- jump gate ----------------------------------------------------------
    GateParams gp;
    gp.max_innovation =
        declare_parameter("max_innovation", gp.max_innovation);
    gp.max_yaw_innovation =
        declare_parameter("max_yaw_innovation", gp.max_yaw_innovation);
    gp.correction_linear_rate =
        declare_parameter("correction_linear_rate", gp.correction_linear_rate);
    gp.correction_angular_rate =
        declare_parameter("correction_angular_rate", gp.correction_angular_rate);
    gp.reacquire_linear_rate =
        declare_parameter("reacquire_linear_rate", gp.reacquire_linear_rate);
    gp.reacquire_angular_rate =
        declare_parameter("reacquire_angular_rate", gp.reacquire_angular_rate);
    gp.hold_timeout = declare_parameter("hold_timeout", gp.hold_timeout);
    gp.consistency_tol =
        declare_parameter("consistency_tol", gp.consistency_tol);
    gp.max_dt = dp.max_dt;
    gate_.setParams(gp);

    if (gp.max_innovation <= 0.0 || gp.max_yaw_innovation <= 0.0 ||
        gp.correction_linear_rate < 0.0 || gp.correction_angular_rate < 0.0 ||
        gp.reacquire_linear_rate < 0.0 || gp.reacquire_angular_rate < 0.0 ||
        gp.hold_timeout < 0.0 || gp.consistency_tol <= 0.0 || gp.max_dt <= 0.0)
    {
        throw std::invalid_argument("invalid pose gate parameters");
    }
}

// ---------------------------------------------------------------------------

void OdomNode::vescCallback(
    const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    estimator_.updateRpm(msg->state.speed);
}

void OdomNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Bias is only updated while the wheels report stopped, so the estimator's
    // speed has to be fed in here.
    dr_.updateGyro(msg->angular_velocity.z, estimator_.speed());
    estimator_.updateGyroYawRate(dr_.yawRate());

    // The IMU is the fastest source, so integration is driven from here.
    propagate(rclcpp::Time(msg->header.stamp));
}

void OdomNode::propagate(const rclcpp::Time& stamp)
{
    if (!have_propagate_)
    {
        last_propagate_ = stamp;
        have_propagate_ = true;
        publishOdomTf(stamp);
        return;
    }
    const double dt = (stamp - last_propagate_).seconds();
    last_propagate_ = stamp;
    if (!std::isfinite(dt) || dt <= 0.0) { return; }

    const double v  = estimator_.speed();
    const double wz = estimator_.yawRate();

    dr_.predict(dt, v);
    if (params_.filter_pose_jumps) { gate_.predict(dt, v, wz); }
    publishOdomTf(stamp);
}

void OdomNode::publishOdomTf(const rclcpp::Time& stamp)
{
    if (!tf_broadcaster_) { return; }

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = params_.odom_frame;
    tf.child_frame_id = params_.base_frame;
    tf.transform.translation.x = dr_.x;
    tf.transform.translation.y = dr_.y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.z = std::sin(0.5 * dr_.yaw);
    tf.transform.rotation.w = std::cos(0.5 * dr_.yaw);
    tf_broadcaster_->sendTransform(tf);
}

void OdomNode::poseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    const double raw_yaw = yawFromQuat(
        msg->pose.orientation.z, msg->pose.orientation.w);
    // /tracked_pose is the pose of Cartographer's tracking_frame. Convert it
    // to the rear-axle base_link origin before publishing controller state.
    const double raw_x = msg->pose.position.x -
         std::cos(raw_yaw) * params_.tracking_offset_x +
         std::sin(raw_yaw) * params_.tracking_offset_y;
    const double raw_y = msg->pose.position.y -
         std::sin(raw_yaw) * params_.tracking_offset_x -
         std::cos(raw_yaw) * params_.tracking_offset_y;

    std::lock_guard<std::mutex> lock(mutex_);
    applyMapPose(raw_x, raw_y, raw_yaw, rclcpp::Time(msg->header.stamp));
}

void OdomNode::tfPoseTimer()
{
    geometry_msgs::msg::TransformStamped tf;
    try
    {
        // Latest available. AMCL updates map->odom only on motion, so asking
        // for "now" would throw between updates.
        tf = tf_buffer_->lookupTransform(
            params_.map_frame, params_.base_frame, tf2::TimePointZero);
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "no %s -> %s transform yet: %s",
                             params_.map_frame.c_str(),
                             params_.base_frame.c_str(), ex.what());
        return;
    }

    // Already base_link here, so no tracking-frame offset to remove.
    std::lock_guard<std::mutex> lock(mutex_);
    applyMapPose(tf.transform.translation.x,
                 tf.transform.translation.y,
                 yawFromQuat(tf.transform.rotation.z, tf.transform.rotation.w),
                 rclcpp::Time(tf.header.stamp));
}

void OdomNode::applyMapPose(double raw_x, double raw_y, double raw_yaw,
                            const rclcpp::Time& stamp)
{
    if (!params_.filter_pose_jumps)
    {
        gate_.reset(raw_x, raw_y, raw_yaw);
    }
    else
    {
        double dt = 0.0;
        if (have_pose_stamp_) { dt = (stamp - last_pose_stamp_).seconds(); }
        if (!std::isfinite(dt)) { dt = 0.0; }

        // Seed dead reckoning from the first map pose so the odom frame starts
        // co-located with the map frame rather than at the origin. Skipped in
        // "tf" mode: there the odom frame is already live and AMCL is solving
        // map->odom against it, so moving it underneath AMCL would be circular.
        if (!have_pose_ && params_.map_pose_source != "tf")
        {
            dr_.reset(raw_x, raw_y, raw_yaw);
        }

        gate_.update(raw_x, raw_y, raw_yaw, dt);

        const GateState st = gate_.state();
        if (st != last_reported_state_)
        {
            if (st == GateState::Holding)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Rejecting map pose correction: innovation=%.3f m "
                    "(limit %.3f), yaw=%.3f rad. Coasting on dead reckoning; "
                    "will accept if it persists %.1f s.",
                    gate_.innovation(), gate_.params().max_innovation,
                    gate_.yawInnovation(), gate_.params().hold_timeout);
            }
            else if (st == GateState::Reacquiring)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Map pose correction of %.3f m persisted %.1f s and is "
                    "being ACCEPTED. This is either a real relocalization or a "
                    "sustained localizer flip -- the pose stream cannot tell "
                    "them apart. Check /tf map->odom for a step.",
                    gate_.innovation(), gate_.params().hold_timeout);
            }
            else if (st == GateState::Tracking &&
                     last_reported_state_ == GateState::Holding)
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Map pose correction was transient and never reached "
                    "the controller (rejected=%ld, transient=%ld, accepted=%ld)",
                    gate_.rejectedCount(), gate_.transientCount(),
                    gate_.acceptedCount());
            }
            last_reported_state_ = st;
        }
    }

    have_pose_ = true;
    last_pose_stamp_ = stamp;
    have_pose_stamp_ = true;

    // The estimator's pose derivative is only a fallback; feed it the gated
    // pose so a rejected jump does not contaminate the speed estimate either.
    estimator_.updatePose(gate_.x(), gate_.y(), gate_.yaw(), stamp.seconds());

    if (params_.log_calibration)
    {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "calib: rpm_speed %.3f  pose_speed %.3f  ratio %.3f  "
            "gyro_bias %.5f rad/s (%ld samples)",
            estimator_.rpmSpeed(), estimator_.poseSpeed(),
            std::abs(estimator_.rpmSpeed()) > 1e-3
                ? estimator_.poseSpeed() / estimator_.rpmSpeed()
                : 0.0,
            dr_.gyroBias(), dr_.biasSamples());
    }

    if (params_.publish_rate <= 0.0)
    {
        publish(stamp);
        return;
    }

    if (!have_published_ ||
        (stamp - last_publish_).seconds() >= 1.0 / params_.publish_rate)
    {
        publish(stamp);
        last_publish_ = stamp;
        have_published_ = true;
    }
}

void OdomNode::publish(const rclcpp::Time& stamp)
{
    if (!have_pose_) { return; }

    const bool trusted = gate_.trusted();
    const double yaw = gate_.yaw();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = params_.map_frame;
    odom.child_frame_id = params_.base_frame;

    odom.pose.pose.position.x = gate_.x();
    odom.pose.pose.position.y = gate_.y();
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.z = std::sin(0.5 * yaw);
    odom.pose.pose.orientation.w = std::cos(0.5 * yaw);

    // Twist is body-frame (REP-103): x is forward speed.
    odom.twist.twist.linear.x  = estimator_.speed();
    odom.twist.twist.angular.z = estimator_.yawRate();

    // Report inflated covariance while coasting, so anything downstream that
    // reads it can see the pose is not currently anchored to the map.
    const double pvar = trusted ? params_.position_variance
                                : params_.untrusted_position_variance;
    const double yvar = trusted ? params_.yaw_variance
                                : params_.untrusted_yaw_variance;
    odom.pose.covariance[0]  = pvar;
    odom.pose.covariance[7]  = pvar;
    odom.pose.covariance[35] = yvar;
    odom.twist.covariance[0]  = params_.speed_variance;
    odom.twist.covariance[35] = params_.speed_variance;

    odom_pub_->publish(odom);

    std_msgs::msg::Bool trusted_msg;
    trusted_msg.data = trusted;
    trusted_pub_->publish(trusted_msg);

}

} // namespace pose_odom

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pose_odom::OdomNode>());
    rclcpp::shutdown();
    return 0;
}
