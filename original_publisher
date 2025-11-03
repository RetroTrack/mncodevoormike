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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

 std::string Return_Time_As_String()
{
  auto ditMoment = std::chrono::system_clock::now();
  auto ditMomentInTijd = std::chrono::system_clock::to_time_t(ditMoment);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&ditMomentInTijd), "%Y-%m-%d %X");
  return ss.str();
}

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("date_time", 10); // "topic"
    timer_ = this->create_wall_timer( 
      500ms, std::bind(&MinimalPublisher::timer_callback, this)); // elke 0.5 sec roept van de klasse minimalpublisher de timer_callback methode
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String(); // message wordt een string
    message.data = Return_Time_As_String(); // de data in de message is deze methode die een string retourneert
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str()); // log het bericht dat je gaat publishen
    publisher_->publish(message); // publish het bericht
    
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
