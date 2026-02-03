// publisher in c++
#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
public:
  Talker() : Node("cpp_talker"), count_(0)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&Talker::on_timer, this)
    );

    RCLCPP_INFO(this->get_logger(), "C++ Talker started");
  }

private:
  void on_timer()
  {
    std_msgs::msg::String msg;
    msg.data = "Hello from C++ talker: " + std::to_string(count_++);
    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}

// subscriber in c++
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Listener : public rclcpp::Node
{
public:
  Listener() : Node("cpp_listener")
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10,
      std::bind(&Listener::cb, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "C++ Listener started");
  }

private:
  void cb(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}
