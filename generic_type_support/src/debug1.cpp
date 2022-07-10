// Copyright 2021 Takagi, Isamu
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

#include "generic_type_support/generic_type_support.hpp"

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

std::ostream& operator<<(std::ostream& os, const rclcpp::SerializedMessage & msg)
{
  const auto array = msg.get_rcl_serialized_message();
  const auto flags = os.flags();
  os << "size     : " << msg.size() << std::endl;
  os << "capacity : " << msg.capacity() << std::endl;
  os << "size     : " << array.buffer_length << std::endl;
  os << "capacity : " << array.buffer_capacity << std::endl;
  os << "buffer   : " << std::hex << std::setfill('0');
  for (size_t i = 0; i < msg.size(); ++i)
  {
    os << std::setw(2) << static_cast<uint32_t>(array.buffer[i]) << " ";
  }
  os.flags(flags);
  return os;
}

std::shared_ptr<generic_type_support::GenericMessage> message_;
void callback(const std::shared_ptr<rclcpp::SerializedMessage> serialized)
{
  const auto yaml = message_->ConvertYAML(*serialized);
  const auto field1 = message_->GetAccess("frame_id");
  const auto field2 = message_->GetAccess("stamp.sec");

  std::cout << "==================== DATA ====================" << std::endl;
  std::cout << *serialized << std::endl;
  std::cout << "==================== YAML ====================" << std::endl;
  std::cout << yaml << std::endl;
  std::cout << "==============================================" << std::endl;
  std::cout << field1.Access(yaml) << std::endl;
  std::cout << field2.Access(yaml) << std::endl;

  rclcpp::shutdown();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("generic");
  auto type = node->declare_parameter("type", "std_msgs/msg/Header");
  auto subs = node->create_generic_subscription("/monitor/header", type, rclcpp::QoS(1), callback);

  RCLCPP_INFO(node->get_logger(), "type: %s", type.c_str());
  message_ = std::make_shared<generic_type_support::GenericMessage>(type);

  rclcpp::spin(node);
  rclcpp::shutdown();
}
