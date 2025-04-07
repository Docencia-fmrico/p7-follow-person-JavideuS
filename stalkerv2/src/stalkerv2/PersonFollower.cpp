#include "stalkerv2/PersonFollower.hpp"

//For tf2 calculations
#include <Eigen/Dense>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace stalkerv2
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;
using namespace std::placeholders;

PersonFollower::PersonFollower()
: LifecycleNode("person_follower"),
  tfBuffer_(this->get_clock()),
  tfListener_(tfBuffer_),
  last_detection_time_(this->now()),
  last_obstacle_time_(this->now()),
  vlin_pid_(0.05, 10, 0.1, 0.8),
  vrot_pid_(0.05, M_PI, 0.3, 1.5)
{
  RCLCPP_INFO(get_logger(), "Creating node");
}


CallbackReturn PersonFollower::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;
  RCLCPP_INFO(get_logger(), "Configuring node...Creating publisher");
  publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  attractive_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
    "coordinates/attractive", rclcpp::SensorDataQoS().reliable(),
    std::bind(&PersonFollower::attractive_callback, this, _1));

  repulsive_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
    "coordinates/repulsive", rclcpp::SensorDataQoS().reliable(),
    std::bind(&PersonFollower::repulsive_callback, this, _1));

    //This is essentially for the first time you connect to the transform, then it wont have problems
  if(!tfBuffer_.canTransform("base_footprint", "camera_rgb_optical_frame", tf2::TimePointZero,
      tf2::Duration(5s)))
  {
    RCLCPP_ERROR(get_logger(), "Cannot get transform");
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn PersonFollower::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;
  RCLCPP_INFO(get_logger(), "Activating node... Setting timer to publisher, reading tfs");
  timer_ = create_wall_timer(30ms, std::bind(&PersonFollower::timer_callback, this));
  publisher_->on_activate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PersonFollower::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;
  RCLCPP_INFO(get_logger(), "Deactivating node, stopping operations");
  timer_.reset();
  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0.0;
  msg.angular.z = 0.0;
  publisher_->publish(msg);
  publisher_->on_deactivate();

  return CallbackReturn::SUCCESS;
}


CallbackReturn PersonFollower::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;
  RCLCPP_INFO(get_logger(), "Cleaning up node, releasing resources");
  publisher_.reset();
  attractive_pose_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PersonFollower::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  if(previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(previous_state);     //To clear and stop everything (Works for simulation)
  }
  RCLCPP_INFO(get_logger(), "Shutting down node.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PersonFollower::on_error(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;
  RCLCPP_INFO(get_logger(), "An error happened");
  return CallbackReturn::SUCCESS;
}

void PersonFollower::timer_callback()
{
  geometry_msgs::msg::Twist msg;
  if(((this->now() - last_detection_time_).seconds() > TIMEOUT) &&
    (this->now() - last_obstacle_time_).seconds() > OBSTACLE_TIMEOUT)
  {
    RCLCPP_INFO(get_logger(), "No target detected, looking around");
    msg.linear.x = 0;
    msg.angular.z = 0.5;     //To keep looking around
    publisher_->publish(msg);
  } else {
    if(tfBuffer_.canTransform("base_footprint", "camera_rgb_optical_frame", tf2::TimePointZero)) {

      Eigen::Vector2d attractive_vector(0.0, 0.0);

            // Check both repulsive_vector_ AND attractive_pose_ have values before proceeding
      if (attractive_pose_.has_value() &&
        (this->now() - last_detection_time_).seconds() < TIMEOUT)
      {
        tf2::Transform base2cam;
        tf2::Transform cam2obj;

        auto base2cam_msg = tfBuffer_.lookupTransform("base_footprint", "camera_rgb_optical_frame",
                          tf2::TimePointZero);

        tf2::Stamped<tf2::Transform> stamped_base2cam;
        tf2::fromMsg(base2cam_msg, stamped_base2cam);

                // Extract only the transformation (ignoring timestamp and frame ID)
        base2cam.setOrigin(stamped_base2cam.getOrigin());
        base2cam.setRotation(stamped_base2cam.getRotation());

        auto createTransform = [&cam2obj](geometry_msgs::msg::Pose pose){
            tf2::Vector3 translation(pose.position.x, pose.position.y, pose.position.z);
            tf2::Quaternion rotation(pose.orientation.x, pose.orientation.y, pose.orientation.z,
              pose.orientation.w);

                    //To create the transform (only structure)
                    //I dont mean to publish it, only use it for calculations
            cam2obj.setOrigin(translation);
            cam2obj.setRotation(rotation);
          };

        createTransform(attractive_pose_.value());
        tf2::Transform base2obj = base2cam * cam2obj;

                                // RCLCPP_INFO(get_logger(), "Tf coordinates: %f, %f, %f",
                                //                      base2obj.getOrigin().x(),
                                //                      base2obj.getOrigin().y(),
                                //                      base2obj.getOrigin().z());

        attractive_vector = Eigen::Vector2d(base2obj.getOrigin().x(), base2obj.getOrigin().y());
      }

      Eigen::Vector2d repulsive_vector(0.0, 0.0);

      if (repulsive_vector_.has_value() &&
        (this->now() - last_obstacle_time_).seconds() < OBSTACLE_TIMEOUT)
      {
        repulsive_vector.x() = repulsive_vector_.value().x;
        repulsive_vector.y() = repulsive_vector_.value().y;

                // RCLCPP_INFO(get_logger(), "Applying repulsive adjustment: lin=%f, ang=%f",
                //             repulsive_vector.x(), repulsive_vector.y());
      }

      Eigen::Vector2d resultant_vector = attractive_vector + repulsive_vector;

      double dis = sqrt(resultant_vector.x() * resultant_vector.x() + resultant_vector.y() *
                resultant_vector.y());
      double angle = atan2(resultant_vector.y(), resultant_vector.x());


      RCLCPP_INFO(get_logger(), "Distance: %f, Angle: %f", dis, angle);

            // Check for NaN values before proceeding
            //Sometimes 3d node gives NaN values when the object is too close
      if (std::isnan(dis) || std::isnan(angle)) {
        RCLCPP_WARN(get_logger(), "Invalid position data (NaN) detected");
        msg.linear.x = 0;
        msg.angular.z = 0.5;         // Rotate slowly to search
        publisher_->publish(msg);
        return;
      }

            // Calculating velocity commands using PID controllers
      double vel_rot = std::clamp(vrot_pid_.get_output(angle), -1.0, 1.0);
      double vel_lin = vlin_pid_.get_output(dis - OBSTACLE_THRESHOLD);


            // Checking for NaN velocities after PID calculation (Just in case)
      if (std::isnan(vel_rot) || std::isnan(vel_lin)) {
        RCLCPP_WARN(get_logger(), "PID produced NaN velocities, resetting");
        msg.linear.x = 0;
        msg.angular.z = 0;
        publisher_->publish(msg);
        return;
      }

      if(repulsive_vector_.has_value() && !attractive_pose_.has_value()) {
        vel_lin *= 0.3;                         // Reduce distance if only repulsive vector is present
                                //To make it smoother
      }
      if(dis == 0) {
        vel_lin = 0;
                                //Would act as a keep rotating knowing that theres an obstacle close
        vel_rot = 0.5;
      }

      if(dis < OBSTACLE_THRESHOLD) {
        RCLCPP_INFO(get_logger(), "Target reached");
      }

      msg.linear.x = vel_lin;
      msg.angular.z = vel_rot;

      publisher_->publish(msg);
    }
  }
}

void PersonFollower::attractive_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                // Node is not active, ignore this callback
    return;
  }
        //else
  if(!msg->detections.empty()) {
    for(auto detections : msg->detections) {
      if(detections.results[0].hypothesis.class_id == "person") {
        RCLCPP_INFO(get_logger(), "Detection at: 'x: %f' 'y: %f' 'z: %f'",
                                                        detections.bbox.center.position.x,
                                                        detections.bbox.center.position.y,
                                                        detections.bbox.center.position.z);
        attractive_pose_ = detections.bbox.center;
        RCLCPP_INFO(get_logger(), "Person detected %s",
            detections.results[0].hypothesis.class_id.c_str());
        last_detection_time_ = this->now();
        break;                         // Exit the loop after finding the first person
      }
    }
  } else {
    RCLCPP_INFO(get_logger(), "No target detected");
    attractive_pose_.reset();             // Set to empty
  }
}

void PersonFollower::repulsive_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        // Node is not active, ignore this callback
    return;
  }

  if (msg->z > 0.0) {    // Obstacle detected
    float distance = msg->x;
    float angle = msg->y;

       // Calculate repulsive force
    double repulsive_gain = 3.0;
    double repulsive_magnitude = 0;
    if (distance < (OBSTACLE_THRESHOLD - OBSTACLE_THRESHOLD / repulsive_gain)) {
                        //If the distance is bigger than that, if there is no attractive vector, it would go back to the obstacle again
                        //So this limits it to not have a force less than the threshold itself to not move in the opposite direction
                        //repulsive_magnitude = (1 / distance - 1 / OBSTACLE_THRESHOLD) * repulsive_gain;
      repulsive_magnitude = (OBSTACLE_THRESHOLD - distance) * repulsive_gain;
    }

                // Calculate repulsive vector
    auto repulsive = geometry_msgs::msg::Vector3();

    repulsive.x = -cos(angle) * repulsive_magnitude;
    repulsive.y = -sin(angle) * repulsive_magnitude;
    repulsive.z = repulsive_magnitude;

    repulsive_vector_ = repulsive;
    last_obstacle_time_ = this->now();

    RCLCPP_INFO(get_logger(), "Obstacle detected at dist=%f, angle=%f, repulsive force=%f",
                   distance, angle, repulsive_magnitude);
  } else {
    RCLCPP_INFO(get_logger(), "No obstacle detected in the scan");
    repulsive_vector_.reset();
  }
}

}
