#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <icecream.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

char getKeyPress()
{
  char key;
  struct termios oldt, newt;

  tcgetattr(STDIN_FILENO, &oldt);

  newt = oldt;

  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  read(STDIN_FILENO, &key, 1);

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return key;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("keyboard_node");
  auto key_pub = node->create_publisher<std_msgs::msg::Int32MultiArray>("key_board", 10);
  auto cmd_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  rclcpp::Rate loop_rate(10);

  int speed = 0;
  while (rclcpp::ok())
  {
    auto cmd_msg = geometry_msgs::msg::Twist();
    char ch = getKeyPress(); // Reads a single character without pressing Enter
    IC(ch);
    if (ch == 27)
    {
      char next_but = getKeyPress();
      if (next_but == '[')
      {
        char next1_but = getKeyPress();
        if (next1_but == 'A')
        {
          speed += 1;
        }
        else if (next1_but == 'B')
        {
          speed -= 1;
        }
        else if (next1_but == 'C')
        {
          cmd_msg.angular.z = speed;
        }
        else if (next1_but == 'D')
        {
          cmd_msg.angular.z = -speed;
        }
      }
      else
      {
        IC(0);
        break;
      }
    }
    else if (ch == 'w')
    {
      cmd_msg.linear.x = speed;
    }
    else if (ch == 's')
    {
      cmd_msg.linear.x = -speed;
    }
    else if (ch == 'd')
    {
      cmd_msg.linear.y = speed;
    }
    else if (ch == 'a')
    {
      cmd_msg.linear.y = -speed;
    }
    else if (ch == 'x')
    {
      cmd_msg.linear.x = 0;
      cmd_msg.linear.y = 0;
      cmd_msg.angular.z = 0;
    }
    
    IC(speed);
    cmd_pub->publish(cmd_msg);

    auto key_msg = std_msgs::msg::Int32MultiArray();
    key_msg.data.push_back(static_cast<int>(ch));
    key_pub->publish(key_msg);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
