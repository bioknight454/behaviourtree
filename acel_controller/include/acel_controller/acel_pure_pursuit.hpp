#ifndef ACEL_CONTROLLER__ACEL_PURE_PURSUIT_HPP_
#define ACEL_CONTROLLER__ACEL_PURE_PURSUIT_HPP_

#include <string>
#include <vector>
#include <memory>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"

#include "nav_msgs/msg/odometry.hpp"

namespace acel_pure_pursuit
{
    class Acel_pure_pursuit : public nav2_core::Controller
    {
    private:
        struct e
        {
            float x;
            float y;
            float theta;
            float distance;
            float angle;
        } error;
        struct c
        {
            float distance;
            float angle;
        } controlled;
        struct param_constant
        {
            float kp;
            float ki;
            float kd;
        } params;
        //---------------------------------------------------------------------------------------//
        std::mutex pose_mutex_;
        //---------------------------------------------------------------------------------------//
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        nav_msgs::msg::Odometry odom_robot_msg;
        
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_movement_mode;
        std_msgs::msg::Float32 msg_movement_mode;

        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_amcl_;
        geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;
        //---------------------------------------------------------------------------------------//
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_goal;

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_pub;

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_;

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr control_effort;

        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_pose_inline;
        
        //---------------------------------------------------------------------------------------//
        void movement_call(const std_msgs::msg::Float32 &msg){
            std::lock_guard<std::mutex> lock(pose_mutex_);
            msg_movement_mode = msg;
        }

        void robot_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            current_pose_ = msg;
        }

        void odom_robot_callback(const nav_msgs::msg::Odometry &msg)
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            odom_robot_msg = msg;
        }

    public:
        Acel_pure_pursuit() = default;
        ~Acel_pure_pursuit() override = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name,
            std::shared_ptr<tf2_ros::Buffer> tf,
            const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

        void cleanup() override;
        void activate() override;
        void deactivate() override;
        void setSpeedLimit(const double &speed_limit, const bool &percentage);
        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped &pose,
            const geometry_msgs::msg::Twist &velocity,
            nav2_core::GoalChecker *goal_checker) override;

        void setPlan(const nav_msgs::msg::Path &path) override;

        geometry_msgs::msg::PoseStamped transformGlobalPoseToLocal(
            const geometry_msgs::msg::PoseStamped &pose);

    protected:
        nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped &pose);
        bool transformPose(
            const std::shared_ptr<tf2_ros::Buffer> tf,
            const std::string frame,
            const geometry_msgs::msg::PoseStamped &in_pose,
            geometry_msgs::msg::PoseStamped &out_pose,
            const rclcpp::Duration &transform_tolerance) const;

        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::string plugin_name_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        rclcpp::Logger logger_{rclcpp::get_logger("PurePursuitController")};
        rclcpp::Clock::SharedPtr clock_;

        double desired_linear_vel_;
        struct param
        {
            float kp;
            float ki;
            float kd;
        } parameters;
        double lookahead_dist_;
        double max_angular_vel_;
        rclcpp::Duration transform_tolerance_{0, 0};

        nav_msgs::msg::Path global_plan_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;
    };
}

#endif