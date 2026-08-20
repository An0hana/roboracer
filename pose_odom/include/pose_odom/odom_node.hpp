#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "pose_odom/pose_filter.hpp"
#include "pose_odom/velocity_estimator.hpp"

namespace pose_odom
{

/// Publishes a continuous controller state from Cartographer's map pose,
/// gated against dead reckoning so a pose-graph jump cannot reach MPPI.
///
/// Two poses are maintained:
///   * gate_  : map-frame pose, follows Cartographer when the innovation is
///              plausible, coasts on dead reckoning when it is not.
///   * dr_    : pure dead reckoning, never corrected. This is the "odom"
///              frame, and it is what AMCL needs in order to run at all.
class OdomNode : public rclcpp::Node
{
public:
    OdomNode();

private:
    struct Params
    {
        std::string tracked_pose_topic{"/tracked_pose"};
        std::string vesc_topic{"/sensors/core"};
        std::string imu_topic{"/imu/data_raw"};
        std::string odom_topic{"/state_estimation/odom"};

        /// Where the map-frame pose comes from.
        ///   "tracked_pose" - subscribe to Cartographer's /tracked_pose.
        ///   "tf"           - look up map->base_link from TF. Use this with
        ///                    AMCL, which publishes map->odom and never
        ///                    publishes a pose topic at controller rate.
        std::string map_pose_source{"tracked_pose"};
        std::string trusted_topic{"/state_estimation/pose_trusted"};

        std::string map_frame{"map"};
        std::string odom_frame{"odom"};
        std::string base_frame{"base_link"};

        /// Cartographer's /tracked_pose is the pose of tracking_frame
        /// (gyro_link), 0.25 m ahead of the rear-axle base_link origin.
        double tracking_offset_x{0.25};
        double tracking_offset_y{0.0};

        double publish_rate{50.0};      ///< [Hz] 0 = publish on every pose

        bool filter_pose_jumps{true};

        /// Publish odom->base_link from pure dead reckoning. Leave FALSE while
        /// Cartographer runs with provide_odom_frame=true, or the two will
        /// fight over the same edge of the TF tree. Set TRUE for the AMCL
        /// stack, where AMCL owns map->odom and this node owns odom->base_link.
        bool publish_odom_tf{false};

        bool log_calibration{false};

        double position_variance{0.0025};
        double yaw_variance{0.0025};
        double speed_variance{0.04};

        /// Covariance reported while the gate is not tracking the map. Lets a
        /// downstream filter or planner see that the pose is coasting.
        double untrusted_position_variance{1.0};
        double untrusted_yaw_variance{0.25};
    };

    void loadParameters();

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    /// Shared back end for both map-pose sources. Pose is already base_link.
    void applyMapPose(double raw_x, double raw_y, double raw_yaw,
                      const rclcpp::Time& stamp);
    /// Timer body used when map_pose_source == "tf".
    void tfPoseTimer();
    void vescCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    /// Advance both dead-reckoned poses to `stamp`.
    void propagate(const rclcpp::Time& stamp);
    /// Publish the physically grounded odom -> base_link edge independently
    /// of map localization. AMCL needs this edge before it can create
    /// map -> odom, so tying it to the first map pose creates a TF deadlock.
    void publishOdomTf(const rclcpp::Time& stamp);
    void publish(const rclcpp::Time& stamp);

    Params params_;
    VelocityEstimator estimator_;
    DeadReckoner      dr_;
    JumpGate          gate_;

    std::mutex mutex_;

    rclcpp::Time last_propagate_;
    bool         have_propagate_{false};

    rclcpp::Time last_pose_stamp_;
    bool         have_pose_stamp_{false};
    bool         have_pose_{false};

    rclcpp::Time last_publish_;
    bool         have_published_{false};

    GateState    last_reported_state_{GateState::Init};

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trusted_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
};

} // namespace pose_odom
