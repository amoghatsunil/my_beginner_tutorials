/**
 * BSD 3-Clause License
 * @file publisher_member_function.cpp
 * @brief Publisher node and service using callback, with tf broadcaster
 * @version 1.0
 * @date 2024-11-07
 * @author Amogha Sunil<amoghats@umd.edu>
 * @copyright Copyright (c) 2024
 */

#include <functional>
#include <memory>
#include <string>

#include "beginner_tutorials/srv/change_string.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/parameter_client.hpp"

/*Parameter Types */
using PARAMETER_EVENT = std::shared_ptr<rclcpp::ParameterEventHandler>;
using PARAMETER_HANDLE = std::shared_ptr<rclcpp::ParameterCallbackHandle>;

/**
 * @class MinimalPublisher
 * @brief A publisher ros2 node which publishes messages to a topic and provides
 * a service to change the message content, along with broadcasting a tf frame.
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for MinimalPublisher. init for node, publisher and
   * service
   */
  MinimalPublisher() : Node("Minimal_Custom_Publisher"), count(0) {
    try {
      publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
      tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

      auto param_desc = rcl_interfaces::msg::ParameterDescriptor();

      param_desc.description = " Parameter updated from launch file";
      this->declare_parameter("frequency", 2, param_desc);

      // Get the parameter value.
      auto frequency =
          this->get_parameter("frequency").get_parameter_value().get<int>();

      RCLCPP_INFO_STREAM(this->get_logger(), " Parameter Value: " << frequency);

      // timer callback function
      timer = this->create_wall_timer(
          std::chrono::milliseconds(1000 / frequency),
          std::bind(&MinimalPublisher::timer_callback, this));

      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialize the Publisher");

      // Create a service that allows changing the base output string.
      service = this->create_service<beginner_tutorials::srv::ChangeString>(
          "service_node",
          std::bind(&MinimalPublisher::changeString, this,
                    std::placeholders::_1, std::placeholders::_2));
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialize the Service");
    } catch (...) {
      // Log an error if exception occurs during initialization.
      RCLCPP_ERROR_STREAM(this->get_logger(), "Error during Initialization");
      RCLCPP_FATAL_STREAM(this->get_logger(),
                          "The publisher may not work as expected");
    }
  }

  /**
   * @brief callback function for service request changes.
   * @param request and response for output modified string.
   */
  void changeString(
      const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::ChangeString::Response>
          response) {
    response->output = request->input + " Service node edited this Message";
    service_response_message = response->output;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\n input: '%s'",
                request->input.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: '%s'",
                response->output.c_str());
  }

 private:
  /**
   * @brief callback function that publishes to the topic.
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = service_response_message;

    ///<  Log the message data
    RCLCPP_DEBUG_STREAM(this->get_logger(), " Inserting the message data");

    ///<  Publish the message
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher->publish(message);
    
    ////<  Broadcast transform
    geometry_msgs::msg::TransformStamped transformStamped;

    ///< Set header and parent frame
    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "talk";

    ///< translation (non-zero values)
    transformStamped.transform.translation.x = 1.0;
    transformStamped.transform.translation.y = 2.0;
    transformStamped.transform.translation.z = 3.0;

    ///< rotation (non-zero quaternion)
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    ///< 45 degrees in z axis
    transformStamped.transform.rotation.z = 0.707;  
    transformStamped.transform.rotation.w = 0.707;

    ///<  Broadcast the transform
    tfBroadcaster->sendTransform(transformStamped);
  }
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
  ///< Service object to handle service requests.
  rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr service;

  ///< tf broadcaster object 
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

  ///< Message to be published, updated by the service.
  std::string service_response_message{" Hi, from Amogha !"};

  size_t count;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // Create a shared pointer to the MinimalPublisher node.
  auto node = std::make_shared<MinimalPublisher>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  RCLCPP_WARN_STREAM(node->get_logger(), "Shutting Down!");
  return 0;
}
