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
#include <iostream>
#include <memory>
#include <ostream>
#include <string>

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
      "time_input", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }


private:
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Recieved: '%s'", msg.data.c_str());
    convert_to_art(msg.data.c_str());
  }

  static void convert_to_art(const std::string &received_time) {
    std::string digits[10][5] = {
      {
        "█████",
        "█   █",
        "█   █",
        "█   █",
        "█████"
      },
      {
        "    █",
        "    █",
        "    █",
        "    █",
        "    █"
      },
      {
        " ███ ",
        "    █",
        " ███ ",
        "█    ",
        " ███ "
      },
      {
        "████ ",
        "    █",
        "████ ",
        "    █",
        "████ "
      },
      {
        "█   █",
        "█   █",
        "█████",
        "    █",
        "    █"
      },
      {
        " ███ ",
        "█    ",
        " ███ ",
        "    █",
        " ███ "
      },
      {
        " ███ ",
        "█    ",
        " ███ ",
        "█   █",
        " ███ "
      },
      {
        "████ ",
        "    █",
        "    █",
        "    █",
        "    █"
      },
      {
        " ███ ",
        "█   █",
        " ███ ",
        "█   █",
        " ███ "
      },
      {
        " ███ ",
        "█   █",
        " ███ ",
        "    █",
        " ███ "
      }
    };

    std::string colon[5] = {
      "     ",
      "  █  ",
      "     ",
      "  █  ",
      "     "
    };

    std::string lines[5] = {
      "",
      "",
      "",
      "",
      ""
    };

    for (int line = 0; line < 5; line++)
    {
      for (int i = 0; i < 5; i++)
      {
        if (received_time[i] >= '0' && received_time[i] <= '9') {
          const int digit = received_time[i] - '0';
          lines[line] += digits[digit][line];
        }else if(received_time[i] == ':') {
          lines[line] += colon[line];
        }
        lines[line] += " ";
      }
      std::cout << lines[line] << std::endl;
    }
    

  }


  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
