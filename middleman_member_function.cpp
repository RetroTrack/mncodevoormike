// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
// #include <fmt>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "date_time", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::String>("time_input", 10);
  }

private:
  void topic_callback(const std_msgs::msg::String &msg) const
  {

    // Initialize a string stream
    // Initialize a string stream
    std::istringstream ss(msg.data);

    // Declare and zero-initialize a tm struct
    std::tm time = {};

    // Parse the date and time (format: "YYYY-MM-DD HH:MM:SS")
    ss >> std::get_time(&time, "%Y-%m-%d %H:%M:%S");

    if (ss.fail())
    {
      RCLCPP_WARN(this->get_logger(), "Failed to parse time from message: '%s'", msg.data.c_str());
      return;
    }

    // Format the time as HH:MM using std::put_time
    std::ostringstream out;
    out << std::put_time(&time, "%H:%M");

    // Create the message to publish
    auto message = std_msgs::msg::String();
    message.data = out.str();

    // Log and publish
    // RCLCPP_INFO(this->get_logger(), "Received date_time: '%s'", msg.data.c_str());
    // RCLCPP_INFO(this->get_logger(), "Publishing time_input: '%s'", message.data.c_str());

    publisher_->publish(message);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
