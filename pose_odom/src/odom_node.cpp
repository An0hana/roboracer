#include "pose_odom/odom_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pose_odom
{

namespace
{

double yawFromQuat(double z, double w)
{
    // Planar: roll = pitch = 0, so yaw = 2*atan2(z, w).
    return 2.0 * std::atan2(z, w);
}

double wrapAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

} // namespace

OdomNode::OdomNode()
: rclcpp::Node("pose_odom")
{
    loadParameters();

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        params_.tracked_pose_topic, rclcpp::QoS(rclcpp::KeepLast(20)),
        std::bind(&OdomNode::poseCallback, this, std::placeholders::_1));

    vesc_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>(
        params_.vesc_topic, rclcpp::QoS(rclcpp::KeepLast(20)),
        std::bind(&OdomNode::vescCallback, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        params_.odom_topic, rclcpp::QoS(rclcpp::KeepLast(20)));

    RCLCPP_INFO(get_logger(),
                "pose_odom ready. pose <- %s, rpm <- %s, out -> %s",
                params_.tracked_pose_topic.c_str(),
                params_.vesc_topic.c_str(),
                params_.odom_topic.c_str());
    RCLCPP_INFO(get_logger(),
                "erpm_to_speed_gain = %.6e m/s per ERPM",
                estimator_.gain());
}

void OdomNode::loadParameters()
{
    params_.tracked_pose_topic =
        declare_parameter("tracked_pose_topic", params_.tracked_pose_topic);
    params_.vesc_topic = declare_parameter("vesc_topic", params_.vesc_topic);
    params_.odom_topic = declare_parameter("odom_topic", params_.odom_topic);

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
    params_.pose_step_slack =
        declare_parameter("pose_step_slack", params_.pose_step_slack);
    params_.max_pose_step_speed =
        declare_parameter("max_pose_step_speed", params_.max_pose_step_speed);
    params_.yaw_step_slack =
        declare_parameter("yaw_step_slack", params_.yaw_step_slack);
    params_.max_yaw_step_rate =
        declare_parameter("max_yaw_step_rate", params_.max_yaw_step_rate);
    params_.correction_linear_rate =
        declare_parameter("correction_linear_rate", params_.correction_linear_rate);
    params_.correction_angular_rate =
        declare_parameter("correction_angular_rate", params_.correction_angular_rate);
    params_.max_filter_dt =
        declare_parameter("max_filter_dt", params_.max_filter_dt);
    params_.log_calibration =
        declare_parameter("log_calibration", params_.log_calibration);

    if (params_.pose_step_slack < 0.0 ||
        params_.max_pose_step_speed <= 0.0 ||
        params_.yaw_step_slack < 0.0 ||
        params_.max_yaw_step_rate <= 0.0 ||
        params_.correction_linear_rate < 0.0 ||
        params_.correction_angular_rate < 0.0 ||
        params_.max_filter_dt <= 0.0)
    {
        throw std::invalid_argument("invalid pose jump filter parameters");
    }

    params_.position_variance =
        declare_parameter("position_variance", params_.position_variance);
    params_.yaw_variance =
        declare_parameter("yaw_variance", params_.yaw_variance);
    params_.speed_variance =
        declare_parameter("speed_variance", params_.speed_variance);

    VelocityParams vp;
    // Leave gain <= 0 to compute it from the three physical quantities.
    vp.erpm_to_speed_gain =
        declare_parameter("erpm_to_speed_gain", 0.0);
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

    estimator_.setParams(vp);
}

void OdomNode::vescCallback(
    const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    estimator_.updateRpm(msg->state.speed);
}

void OdomNode::poseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);

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

    const rclcpp::Time stamp(msg->header.stamp);

    if (!params_.filter_pose_jumps || !have_raw_pose_)
    {
        x_ = raw_x;
        y_ = raw_y;
        yaw_ = raw_yaw;
    }
    else
    {
        const double dt = (stamp - last_raw_pose_stamp_).seconds();
        if (std::isfinite(dt) && dt > 0.0 && dt <= params_.max_filter_dt)
        {
            const double raw_dx = raw_x - last_raw_x_;
            const double raw_dy = raw_y - last_raw_y_;
            const double raw_distance = std::hypot(raw_dx, raw_dy);
            const double raw_dyaw = wrapAngle(raw_yaw - last_raw_yaw_);
            const double allowed_distance =
                params_.pose_step_slack + params_.max_pose_step_speed * dt;
            const double allowed_yaw =
                params_.yaw_step_slack + params_.max_yaw_step_rate * dt;
            const bool plausible_increment =
                raw_distance <= allowed_distance &&
                std::abs(raw_dyaw) <= allowed_yaw;

            if (plausible_increment)
            {
                x_ += raw_dx;
                y_ += raw_dy;
                yaw_ = wrapAngle(yaw_ + raw_dyaw);
            }
            else
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "Suppressing Cartographer pose jump: "
                    "translation=%.3f m (limit %.3f), yaw=%.3f rad (limit %.3f)",
                    raw_distance, allowed_distance, std::abs(raw_dyaw), allowed_yaw);
            }

            // Preserve continuity, then converge gently to Cartographer's map
            // correction. This prevents a pose-graph optimization from
            // teleporting both the controller state and local obstacle map.
            const double error_x = raw_x - x_;
            const double error_y = raw_y - y_;
            const double error_norm = std::hypot(error_x, error_y);
            const double max_correction = params_.correction_linear_rate * dt;
            if (error_norm > 0.0 && max_correction > 0.0)
            {
                const double scale = std::min(1.0, max_correction / error_norm);
                x_ += scale * error_x;
                y_ += scale * error_y;
            }
            const double yaw_error = wrapAngle(raw_yaw - yaw_);
            const double max_yaw_correction =
                params_.correction_angular_rate * dt;
            yaw_ = wrapAngle(
                yaw_ + std::clamp(
                    yaw_error, -max_yaw_correction, max_yaw_correction));
        }
        // If timestamps regress or stall, keep the last continuous output and
        // rebase the raw sample below. The next valid increment resumes it.
    }

    qz_ = std::sin(0.5 * yaw_);
    qw_ = std::cos(0.5 * yaw_);
    have_pose_ = true;
    last_raw_x_ = raw_x;
    last_raw_y_ = raw_y;
    last_raw_yaw_ = raw_yaw;
    last_raw_pose_stamp_ = stamp;
    have_raw_pose_ = true;

    estimator_.updatePose(x_, y_, yaw_, stamp.seconds());

    if (params_.log_calibration)
    {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "calib: rpm_speed %.3f  pose_speed %.3f  ratio %.3f "
            "(multiply gain by this ratio to match pose)",
            estimator_.rpmSpeed(), estimator_.poseSpeed(),
            std::abs(estimator_.rpmSpeed()) > 1e-3
                ? estimator_.poseSpeed() / estimator_.rpmSpeed()
                : 0.0);
    }

    // Publish either on every pose or throttled to publish_rate.
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

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = params_.odom_frame;
    odom.child_frame_id = params_.base_frame;

    // Pose converted from tracking_frame to base_link.
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.z = qz_;
    odom.pose.pose.orientation.w = qw_;

    // Twist is body-frame (REP-103): x is forward speed.
    const double v = estimator_.speed();
    odom.twist.twist.linear.x = v;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.z = estimator_.yawRate();

    odom.pose.covariance[0]  = params_.position_variance; // x
    odom.pose.covariance[7]  = params_.position_variance; // y
    odom.pose.covariance[35] = params_.yaw_variance;      // yaw

    odom.twist.covariance[0]  = params_.speed_variance;   // vx
    odom.twist.covariance[35] = params_.speed_variance;   // wz

    odom_pub_->publish(odom);
}

} // namespace pose_odom

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pose_odom::OdomNode>());
    rclcpp::shutdown();
    return 0;
}
