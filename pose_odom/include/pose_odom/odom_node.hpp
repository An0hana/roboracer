#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include "pose_odom/velocity_estimator.hpp"

namespace pose_odom
{

class OdomNode : public rclcpp::Node
{
public:
    OdomNode();

private:
    struct Params
    {
        std::string tracked_pose_topic{"/tracked_pose"};
        std::string vesc_topic{"/sensors/core"};
        std::string odom_topic{"/car/odom"};

        std::string odom_frame{"map"};   ///< frame of /tracked_pose
        std::string base_frame{"base_link"};

        // Cartographer's /tracked_pose is the tracking frame (gyro_link).
        // Position of that frame expressed in base_link [m].
        double tracking_offset_x{0.0};
        double tracking_offset_y{0.0};

        double publish_rate{50.0};       ///< [Hz] 0 = publish on every pose

        // Set to output a direct ERPM->speed calibration log line. Drive a
        // steady speed and read the suggested gain from the console.
        bool log_calibration{false};

        double position_variance{0.0025};///< (0.05 m)^2
        double yaw_variance{0.0025};
        double speed_variance{0.04};     ///< (0.2 m/s)^2
    };

    void loadParameters();

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void vescCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg);

    void publish(const rclcpp::Time& stamp);

    Params params_;
    VelocityEstimator estimator_;

    std::mutex mutex_;

    // Latest pose held for publishing.
    double x_{0.0}, y_{0.0}, yaw_{0.0};
    double qz_{0.0}, qw_{1.0};
    bool   have_pose_{false};

    rclcpp::Time last_publish_;
    bool         have_published_{false};

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

} // namespace pose_odom
