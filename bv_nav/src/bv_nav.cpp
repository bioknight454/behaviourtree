#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/float32.hpp"

#include "bv_nav/convertion.hpp"

using std::placeholders::_1;

class Bv_nav : public rclcpp::Node
{

public:
    Bv_nav() : Node("bv_nav")
    {

        this->client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this,
            "navigate_to_pose");

        this->button_sub = this->create_subscription<std_msgs::msg::Int8MultiArray>(
            "button", 10, std::bind(&Bv_nav::sign_callback, this, std::placeholders::_1));

        sub_amcl = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", 10, std::bind(&Bv_nav::robot_pose, this, _1));

        sub_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&Bv_nav::scanCallback, this, std::placeholders::_1));

        marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&Bv_nav::publish_markers, this));

        pub_pose = this->create_publisher<geometry_msgs::msg::Pose2D>("robot_position", 10);
        pub_cmd_auto = this->create_publisher<std_msgs::msg::Int8>("cmd_dribble", 10);
        pub_movement_mode = this->create_publisher<std_msgs::msg::Float32>("movement_mode", 10);
    }

    void send_goal(double x, double y, double theta)
    {
        using namespace std::placeholders;

        if (!this->client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.z = theta; // Set quaternion based on your needs
        goal_msg.pose.header.frame_id = "map";

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&Bv_nav::goal_response_callback, this, _1);
        send_goal_options.result_callback =
            std::bind(&Bv_nav::get_result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

        auto move_mode = std_msgs::msg::Float32();
        move_mode.data = theta;
        pub_movement_mode->publish(move_mode);
    }

    void robot_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
    {
        Convertion::Quaternion q = {
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        };
        double yaw, pitch, roll;
        conv.quat_to_eular(q, yaw, pitch, roll);

        auto cmd_auto_msg = std_msgs::msg::Int8();
        if ((msg.pose.pose.position.x < 3.5 &&
             msg.pose.pose.position.x > 3.0) &&
            fabs(conv.toDeg(yaw)) < 180)
        {
            cmd_auto_msg.data = 1;
        }
        else
        {
            cmd_auto_msg.data = 0;
        }

        pub_cmd_auto->publish(cmd_auto_msg);

        auto robot_pose = geometry_msgs::msg::Pose2D();
        robot_pose.x = msg.pose.pose.position.x;
        robot_pose.y = msg.pose.pose.position.y;
        robot_pose.theta = conv.toDeg(yaw);

        pub_pose->publish(robot_pose);
    }

    void publish_markers()
    {

        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker ring_marker;
        ring_marker.header.frame_id = "map";
        ring_marker.header.stamp = this->get_clock()->now();
        ring_marker.ns = "ring";
        ring_marker.id = 0;
        ring_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        ring_marker.action = visualization_msgs::msg::Marker::ADD;

        ring_marker.pose.position.x = ring_pose_x;
        ring_marker.pose.position.y = ring_pose_y;
        ring_marker.pose.position.z = 0.0;
        ring_marker.pose.orientation.x = 0.0;
        ring_marker.pose.orientation.y = 0.0;
        ring_marker.pose.orientation.z = 0.0;
        ring_marker.pose.orientation.w = 1.0;

        ring_marker.scale.x = 0.45;
        ring_marker.scale.y = 0.45;
        ring_marker.scale.z = 0.2;
        ring_marker.color.r = 1.0;
        ring_marker.color.g = 0.0;
        ring_marker.color.b = 0.0;
        ring_marker.color.a = 1.0;

        marker_array.markers.push_back(ring_marker);

        double x_ring = ring_pose_x;
        double y_ring = ring_pose_y;
        double radius = 3.0;
        int num_points = 180;
        for (int i = 0; i < num_points; ++i)
        {
            float angle = (3 * M_PI / 2) + i * (-M_PI / num_points);
            // float angle = start_angle + i * (end_angle - start_angle) / num_points;
            float x = 0;
            float y = 0;

            ring_target(x, y, angle, x_ring, y_ring, radius, angle);

            visualization_msgs::msg::Marker point_marker;
            point_marker.header.frame_id = "map"; // Change to the appropriate frame
            point_marker.header.stamp = this->get_clock()->now();
            point_marker.ns = "radius";
            point_marker.id = i + 1;
            point_marker.type = visualization_msgs::msg::Marker::SPHERE;
            point_marker.action = visualization_msgs::msg::Marker::ADD;

            // Set point position
            point_marker.pose.position.x = x;
            point_marker.pose.position.y = y;
            point_marker.pose.position.z = 0;
            point_marker.pose.orientation.x = 0.0;
            point_marker.pose.orientation.y = 0.0;
            point_marker.pose.orientation.z = 0.0;
            point_marker.pose.orientation.w = 1.0;

            // Set point scale and color
            point_marker.scale.x = 0.1; // Diameter of the point
            point_marker.scale.y = 0.1;
            point_marker.scale.z = 0.1;
            point_marker.color.r = 0.0;
            point_marker.color.g = 0.0;
            point_marker.color.b = 1.0;
            point_marker.color.a = 1.0;

            marker_array.markers.push_back(point_marker);
        }

        // Publish marker array
        marker_pub->publish(marker_array);
    }

    void ring_target(float &robotX_target, float &robotY_target, float &angle_target,
                     float ring_in_x, float ring_in_y, float radius, float degree)
    {
        robotX_target = ring_in_x + (radius * cos(degree));
        robotY_target = ring_in_y + (radius * sin(degree));

        angle_target = atan2(ring_in_y - robotY_target, ring_in_x - robotX_target) + M_PI;

        if (angle_target > M_PI)
        {
            angle_target -= 2 * M_PI;
        }
        else if (angle_target < -M_PI)
        {
            angle_target += 2 * M_PI;
        }
    }

    bool valid_pose(float x_target,
                    float y_target,
                    const sensor_msgs::msg::LaserScan::SharedPtr &scan_msg)
    {
        if (x_target >= 0 && x_target <= 13 && y_target >= 0 && y_target <= 7)
        {
            return false;
        }

        float angle_to_target = atan2(y_target, x_target);
        float angle_increment = scan_msg->angle_increment;

        int index = (angle_to_target - scan_msg->angle_min) / angle_increment;
        if (index >= 0 && index < static_cast<int>(scan_msg->ranges.size()))
        {
            float distance_to_target = sqrt(x_target * x_target + y_target * y_target);

            if (distance_to_target < scan_msg->ranges[index])
            {
                return true;
            }
        }

        return false;
    }
    int count_obstacles(float x_target, float y_target, const sensor_msgs::msg::LaserScan::SharedPtr &scan_msg)
    {
        int obstacle_count = 0;

        float angle_to_target = atan2(y_target, x_target);
        float angle_increment = scan_msg->angle_increment;

        int index_to_target = (angle_to_target - scan_msg->angle_min) / angle_increment;
        int search_radius = 5;
        for (int i = index_to_target - search_radius; i <= index_to_target + search_radius; i++)
        {
            if (i >= 0 && i < static_cast<int>(scan_msg->ranges.size()))
            {
                float distance = sqrt(x_target * x_target + y_target * y_target);
                if (scan_msg->ranges[i] < distance)
                {
                    obstacle_count++;
                }
            }
        }
        return obstacle_count;
    }
    void find_valid_target(float &x_target, float &y_target, float &angle_target,
                           float x_ring, float y_ring, float radius,
                           const sensor_msgs::msg::LaserScan::SharedPtr &scan_msg)
    {
        float x, y, h;
        // double x_ring = 0.0;
        // double y_ring = 6.850;
        // double radius = 3.0;
        int num_points = 180;

        float min_distance = std::numeric_limits<float>::max();
        int min_obstacles = std::numeric_limits<int>::max();

        for (int i = 0; i < num_points; i++)
        {
            float angle = (3 * M_PI / 2) + i * (-M_PI / num_points);
            ring_target(x, y, h,
                        x_ring, y_ring, radius, angle);

            if (valid_pose(x, y, scan_msg) == true)
            {

                float distance = sqrt(x * x + y * y);

                int obstacles = count_obstacles(x, y, scan_msg);
                if (obstacles < min_obstacles || (obstacles == min_obstacles && distance < min_distance))
                {
                    x_target = x;
                    y_target = y;
                    angle_target = h;
                    min_distance = distance;
                    min_obstacles = obstacles;
                }
            }
        }
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received LaserScan message:");
        // RCLCPP_INFO(this->get_logger(), "Angle Min: %.2f, Angle Max: %.2f", msg->angle_min, msg->angle_max);
        // RCLCPP_INFO(this->get_logger(), "Ranges[0]: %.2f", msg->ranges[0]);
        // RCLCPP_INFO(this->get_logger(), "size of array %zu", msg->ranges.size());

        find_valid_target(x_target, y_target, angle_target, ring_pose_x, ring_pose_y, 3.0, msg);

        RCLCPP_INFO(this->get_logger(), "Valid target found at: x=%.2f, y=%.2f, theta=%.2f",
                    x_target, y_target, conv.toDeg(angle_target));
    }

    void sign_callback(const std_msgs::msg::Int8MultiArray &msg)
    {
        button.A = msg.data[0];
        button.B = msg.data[1];
        button.X = msg.data[3];
        button.Y = msg.data[4];
        button.LB = msg.data[6];
        button.RB = msg.data[7];
        button.LT = msg.data[8];
        button.RT = msg.data[9];
        button.select = msg.data[10];
        button.start = msg.data[11];
        button.home = msg.data[12];

        if (button.X)
        {
            client_ptr_->async_cancel_all_goals();
        }
        else if (button.A)
        {
            // send_goal(-0.789, -2.416, conv.toRad(-2.368));
            send_goal(-0.281, -2.316, -0.119);
        }
        else if (button.Y)
        {
            send_goal(3.3, 2, conv.toRad(180));
        }
        else if (button.B)
        {
            send_goal(x_target, y_target, angle_target);
            // send_goal(6.3, -0.0416, conv.toRad(175));
        }
    }

private:
    float x_target = 0;
    float y_target = 0;
    float angle_target = 0;
    float ring_pose_x = 10.418298947057105;
    float ring_pose_y = -0.7647802456343102;
    struct b
    {
        int A;
        int Y;
        int B;
        int X;
        int Up;
        int Down;
        int Left;
        int home;
        int Right;
        int start;
        int select;
        int RT;
        int RB;
        int LB;
        int LT;
    } button;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_ptr_;

    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_pose;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_cmd_auto;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_movement_mode;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr sub_button;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_amcl;

    rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr button_sub;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;

    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;

    Convertion conv;

    void goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void get_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto robot_nav = std::make_shared<Bv_nav>();

    try
    {
        rclcpp::spin(robot_nav);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(robot_nav->get_logger(), "Exception in robot_nav: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
};
