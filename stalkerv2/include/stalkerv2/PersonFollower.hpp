#ifndef STALKERV2__PERSONFOLLOWER_HPP_
#define STALKERV2__PERSONFOLLOWER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "stalkerv2/PIDController.hpp"
#include "optional"

namespace stalkerv2
{
class PersonFollower : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  PersonFollower();

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state);

private:
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr attractive_sub_;
  void attractive_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);
  std::optional<geometry_msgs::msg::Pose> attractive_pose_;
  
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr repulsive_sub_;
  void repulsive_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  std::optional<geometry_msgs::msg::Vector3> repulsive_vector_;

  rclcpp::Time last_detection_time_;
  rclcpp::Time last_obstacle_time_;
  
  constexpr static double TIMEOUT = 0.3;   // seconds
  constexpr static double OBSTACLE_TIMEOUT = 0.3;   // seconds
  constexpr static double OBSTACLE_THRESHOLD = 1.0;   // meters
  PIDController vlin_pid_, vrot_pid_;

  

};

} // namespace stalkerv2

#endif