/**
 * BSD 3-Clause License
 * @file test_node.cpp
 * @brief integration test fixture class to test service and publisher nodes.
 * @version 1.0
 * @date 2024-11-15
 * @author Amogha Sunil<amoghats@umd.edu>
 */
#include <catch_ros2/catch_ros2.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "beginner_tutorials/srv/change_string.hpp"

using std_msgs::msg::String;
using namespace std::chrono_literals;

/*create an initial logger*/
auto Logger = rclcpp::get_logger("");

/**
 * @class MyTestsFixture
 * @brief A MyTestsFixture class contains node to perform integration test for
 * publisher and the service nodes.
 */
class MyTestsFixture {
 public:
  MyTestsFixture() {
    /*node creation for test*/
    testerNode = rclcpp::Node::make_shared("IntegrationTestNode1");
    Logger = testerNode->get_logger();

    /*parameter for duration of the test*/
    testerNode->declare_parameter<double>("test_duration");

    /*get duration value*/
    TEST_DURATION = testerNode->get_parameter("test_duration")
                        .get_parameter_value()
                        .get<double>();
    RCLCPP_INFO_STREAM(Logger, "Got test_duration =" << TEST_DURATION);
  }

  ~MyTestsFixture() { /*MyTestsFixture destructor*/
  }

 protected:
  double TEST_DURATION;
  rclcpp::Node::SharedPtr testerNode;
};

/**
 * @brief Test Case 1
 * This test case tests the service node by creating a dummy client to check if
 * the service is available during the duration of the test
 */
TEST_CASE_METHOD(MyTestsFixture, "test service server", "/service_node") {
  /*client for the service_node server*/
  auto client =
      testerNode->create_client<beginner_tutorials::srv::ChangeString>(
          "/service_node");
  RCLCPP_INFO_STREAM(Logger, "ChangeString service client created");

  /*test if the service exists during the time_duration */
  rclcpp::Time start_time = rclcpp::Clock().now();
  bool service_found = false;
  rclcpp::Duration duration = 0s;
  RCLCPP_INFO_STREAM(Logger, "Performing Test...");
  auto timeout = std::chrono::milliseconds((int)(TEST_DURATION * 1000));

  /*test if the service exists during the time_duration */
  if (client->wait_for_service(timeout)) {  // blocking
    duration = rclcpp::Clock().now() - start_time;
    service_found = true;

    /*create request and send to service_node*/
    auto request =
        std::make_shared<beginner_tutorials::srv::ChangeString::Request>();
    request->input = "Test input";

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(testerNode, future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      RCLCPP_INFO_STREAM(Logger,
                         "Service responded with: " << response->output);
      CHECK(response->output == "Test input Service node edited this Message");
    } else {
      RCLCPP_ERROR(Logger, "Failed to call the service");
      /*force test failure if service call fail*/
      CHECK(false);
    }
  }

  RCLCPP_INFO_STREAM(Logger,
                     "duration = " << duration.seconds()
                                   << " service_found=" << service_found);
  /*check service call*/
  CHECK(service_found);
}

/**
 * @brief Test Case 2
 * This test case tests the publisher node by creating a dummy subscriber to
 * check if the topic is available during the duration of the test
 */
TEST_CASE_METHOD(MyTestsFixture, "test topic talker", "topic") {
  bool got_topic = false;

  /*callback definition*/
  struct ListenerCallback {
    explicit ListenerCallback(bool &gotTopic) : gotTopic_(gotTopic) {}
    void operator()(const String msg) const {
      RCLCPP_INFO_STREAM(Logger, "I heard:" << msg.data.c_str());
      gotTopic_ = true;
    }
    bool &gotTopic_;
  };

  /*subscriber to topic*/
  auto subscriber = testerNode->create_subscription<String>(
      "topic", 10, ListenerCallback(got_topic));

  /*test if the talker is available*/
  rclcpp::Rate rate(10.0);
  auto start_time = rclcpp::Clock().now();
  auto duration = rclcpp::Clock().now() - start_time;
  auto timeout = rclcpp::Duration::from_seconds(TEST_DURATION);
  RCLCPP_INFO_STREAM(Logger, "duration = " << duration.seconds()
                                           << " timeout=" << timeout.seconds());
  while (!got_topic && (duration < timeout)) {
    rclcpp::spin_some(testerNode);
    rate.sleep();
    duration = (rclcpp::Clock().now() - start_time);
  }

  RCLCPP_INFO_STREAM(Logger, "duration = " << duration.seconds()
                                           << " got_topic=" << got_topic);
  /* Test assertions - Checks if the topic is received*/
  CHECK(got_topic);
}