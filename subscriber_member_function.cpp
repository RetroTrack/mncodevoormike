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
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }


private:
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Recieved: '%s'", msg.data.c_str());
    convert_to_art(msg.data.c_str());
  }
  
  void convert_to_art(std::string recieved_time) const {
    std::string num0[5] = {
      "█████",
      "█   █",
      "█   █",
      "█   █",
      "█████"
    };
    std::string num1[5] = {
      "    █",
      "    █",
      "    █",
      "    █",
      "    █"
    };
    std::string num2[5] = {
      " ███ ",
      "    █",
      " ███ ",
      "█    ",
      " ███ "
    };
    std::string num3[5] = {
      "████ ",
      "    █",
      "████ ",
      "    █",
      "████ "
    };
    std::string num4[5] = {
      "█   █",
      "█   █",
      "█████",
      "    █",
      "    █"
    };
    std::string num5[5] = {
      " ███ ",
      "█    ",
      " ███ ",
      "    █",
      " ███ "
    };
    std::string num6[5] = {
      " ███ ",
      "█    ",
      " ███ ",
      "█   █",
      " ███ "
    };
    std::string num7[5] = {
      "████ ",
      "    █",
      "    █",
      "    █",
      "    █"
    };
    std::string num8[5] = {
      " ███ ",
      "█   █",
      " ███ ",
      "█   █",
      " ███ "
    };
    std::string num9[5] = {
      " ███ ",
      "█   █",
      " ███ ",
      "    █",
      " ███ "
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
        if(recieved_time[i] == '0') {
          lines[line] += num0[line];
        }
        if(recieved_time[i] == '1') {
          lines[line] += num1[line];
        }
        if(recieved_time[i] == '2') {
          lines[line] += num2[line];
        }
        if(recieved_time[i] == '3') {
          lines[line] += num3[line];
        }
        if(recieved_time[i] == '4') {
          lines[line] += num4[line];
        }
        if(recieved_time[i] == '5') {
          lines[line] += num5[line];
        }
        if(recieved_time[i] == '6') {
          lines[line] += num6[line];
        }
        if(recieved_time[i] == '7') {
          lines[line] += num7[line];
        }
        if(recieved_time[i] == '8') {
          lines[line] += num8[line];
        }
        if(recieved_time[i] == '9') {
          lines[line] += num9[line];
        }
        if(recieved_time[i] == ':') {
          lines[line] += colon[line];
        }
        lines[line] += " ";
      }
      std::cout << lines[0] << std::endl;
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
