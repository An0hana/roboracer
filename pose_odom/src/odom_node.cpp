#include "pose_odom/odom_node.hpp"

#include <algorithm>
#include <cmath>

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

    params_.publish_rate =
        declare_parameter("publish_rate", params_.publish_rate);
    params_.log_calibration =
        declare_parameter("log_calibration", params_.log_calibration);

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

    x_  = msg->pose.position.x;
    y_  = msg->pose.position.y;
    qz_ = msg->pose.orientation.z;
    qw_ = msg->pose.orientation.w;
    yaw_ = yawFromQuat(qz_, qw_);
    have_pose_ = true;

    const rclcpp::Time stamp(msg->header.stamp);
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

    // Pose straight from tracked_pose.
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
