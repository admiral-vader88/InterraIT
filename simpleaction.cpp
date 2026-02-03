// simple action code for C++
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddServer : public rclcpp::Node
{
public:
  AddServer() : Node("cpp_add_server")
  {
    srv_ = this->create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints",
      std::bind(&AddServer::handle, this,
                std::placeholders::_1, std::placeholders::_2)
    );
    RCLCPP_INFO(this->get_logger(), "C++ AddTwoInts service ready");
  }

private:
  void handle(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> req,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> res)
  {
    res->sum = req->a + req->b;
    RCLCPP_INFO(this->get_logger(), "Request: %ld + %ld = %ld",
                req->a, req->b, res->sum);
  }

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddServer>());
  rclcpp::shutdown();
  return 0;
}

// action server for c++
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/action/fibonacci.hpp"

class FibServer : public rclcpp::Node
{
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFib = rclcpp_action::ServerGoalHandle<Fibonacci>;

  FibServer() : Node("cpp_fib_action_server")
  {
    server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fib",
      std::bind(&FibServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FibServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&FibServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "C++ Fibonacci Action Server ready");
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Goal received: order=%d", goal->order);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFib>)
  {
    RCLCPP_INFO(this->get_logger(), "Cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFib> goal_handle)
  {
    std::thread{std::bind(&FibServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFib> goal_handle)
  {
    auto result = std::make_shared<Fibonacci::Result>();
    auto feedback = std::make_shared<Fibonacci::Feedback>();

    int order = goal_handle->get_goal()->order;
    feedback->sequence = {0, 1};

    rclcpp::Rate rate(2);
    for (int i = 2; i < order; i++) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        return;
      }

      feedback->sequence.push_back(
        feedback->sequence[i - 1] + feedback->sequence[i - 2]
      );
      goal_handle->publish_feedback(feedback);
      rate.sleep();
    }

    result->sequence = feedback->sequence;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }

  rclcpp_action::Server<Fibonacci>::SharedPtr server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FibServer>());
  rclcpp::shutdown();
  return 0;
}
