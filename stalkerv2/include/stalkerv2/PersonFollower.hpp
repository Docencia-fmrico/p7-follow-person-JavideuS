#ifndef STALKERV2__PERSONFOLLOWER_HPP_
#define STALKERV2__PERSONFOLLOWER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace stalkerv2
{
class PersonFollower : public rclcpp::Node
{
public:
  PersonFollower();
  void timer_callback();
private:
  rclcpp::TimerBase::SharedPtr timer_;
};
}

#endif