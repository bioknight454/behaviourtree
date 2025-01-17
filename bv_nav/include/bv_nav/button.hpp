#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using std::placeholders::_1;

class GamePadButton : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr button_sub;

public:
    GamePadButton() : Node("gamepadButton")
    {
        this->button_sub = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "button", 10, std::bind(&GamePadButton::button_callback, this, std::placeholders::_1));
    }
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
    void button_callback(const std_msgs::msg::Int32MultiArray &msg)
    {
        if (msg.data.size() >= 15) // Ensure array has at least 15 elements
        {
            button.A = msg.data[0];
            button.B = msg.data[1];
            button.X = msg.data[4];
            button.Y = msg.data[5];
            button.LB = msg.data[6];
            button.RB = msg.data[7];
            button.LT = msg.data[8];
            button.RT = msg.data[9];
            button.select = msg.data[10];
            button.start = msg.data[11];
            button.home = msg.data[12];
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Received array with insufficient size");
        }
    }
};
