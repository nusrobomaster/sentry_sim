#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "sentry_controller/sentry_control_key.h"
#include <string>
#include <signal.h>

std::string cmd_vel_topic;
int velocity_linear;
int velocity_angular;

KeyboardReader input;

class TeleopTurtle : public rclcpp::Node
{
public:
  TeleopTurtle()
  : Node("sentry_control_node"),
    linear_x(0),
    linear_y(0),
    angular_(0),
    l_scale_(10.0),
    a_scale_(10.0)
  {
    // Declare and get parameters
    // this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    // this->declare_parameter<int>("velocity_linear", 10);
    // this->declare_parameter<int>("velocity_angular", 10);

    l_scale_ = velocity_linear;
    a_scale_ = velocity_angular;

    this->declare_parameter<double>("scale_angular", a_scale_);
    this->declare_parameter<double>("scale_linear", l_scale_);

    this->get_parameter("cmd_vel_topic", cmd_vel_topic);
    this->get_parameter("velocity_linear", velocity_linear);
    this->get_parameter("velocity_angular", velocity_angular);
    this->get_parameter("scale_angular", a_scale_);
    this->get_parameter("scale_linear", l_scale_);

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
  }

  void keyLoop();

private:
  double linear_x, linear_y, angular_, l_scale_, a_scale_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopTurtle>();

  signal(SIGINT, quit);

  node->keyLoop();
  quit(0);

  return 0;
}

void TeleopTurtle::keyLoop()
{
  char c;
  bool dirty = false;

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the sentry in x,y direction.");
  puts("Use Q/E keys to turn left/right.");
  puts("Press 'c' to stop.");

  for (;;)
  {
    // get the next event from the keyboard  
    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error &)
    {
      perror("read():");
      return;
    }

    linear_x = linear_y = angular_ = 0;
    RCLCPP_DEBUG(this->get_logger(), "value: 0x%02X\n", c);

    switch (c)
    {
      case KEYCODE_LEFT:
        RCLCPP_DEBUG(this->get_logger(), "LEFT");
        linear_y = 1.0;
        dirty = true;
        break;
      case KEYCODE_RIGHT:
        RCLCPP_DEBUG(this->get_logger(), "RIGHT");
        linear_y = -1.0;
        dirty = true;
        break;
      case KEYCODE_UP:
        RCLCPP_DEBUG(this->get_logger(), "UP");
        linear_x = 1.0;
        dirty = true;
        break;
      case KEYCODE_DOWN:
        RCLCPP_DEBUG(this->get_logger(), "DOWN");
        linear_x = -1.0;
        dirty = true;
        break;
      case KEYCODE_Q:
        RCLCPP_DEBUG(this->get_logger(), "TURN_LEFT");
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_E:
        RCLCPP_DEBUG(this->get_logger(), "TURN_RIGHT");
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_C:
        RCLCPP_DEBUG(this->get_logger(), "STOP");
        angular_ = 0.0;
        linear_x = 0.0;
        linear_y = 0.0;
        dirty = true;
        break;
    }

    geometry_msgs::msg::Twist twist;
    twist.angular.z = a_scale_ * angular_;
    twist.linear.x = l_scale_ * linear_x;
    twist.linear.y = l_scale_ * linear_y;
    if (dirty == true)
    {
      twist_pub_->publish(twist);
      dirty = false;
    }
  }

  return;
}
