#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "cg_intefaces/srv/MoveCmd.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Pub : public rclcpp::Node
{
  public:
    Pub()
    : Node("Pub"), count_(0)
    {
      publisher_ = this->create_publisher<cg_intefaces::srv::MoveCmd>("/move_command", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&Pub::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = cg_intefaces::srv::MoveCmd();
      message.direction = "right";
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<cg_intefaces::srv::MoveCmd>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pub>());
  rclcpp::shutdown();
  return 0;
}