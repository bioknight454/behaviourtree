#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
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

        RCLCPP_INFO(this->get_logger(), "Subscribed to /button");

        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        pub_pose = this->create_publisher<geometry_msgs::msg::Pose2D>("robot_position", 10);

        sub_amcl = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", 10, std::bind(&Bv_nav::robot_pose, this, _1));

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
            send_goal(-0.789, -2.416, conv.toRad(-2.368));
        }
        else if (button.Y)
        {
            send_goal(3.3, 2, conv.toRad(180));
        }
        else if (button.B)
        {
            send_goal(6.3, -0.0416, conv.toRad(175));
        }
    }

private:
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

    rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr sub_button;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_amcl;

    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
    Convertion conv;

    rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr button_sub;

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