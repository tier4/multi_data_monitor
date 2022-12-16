// Copyright 2022 Takagi, Isamu
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

#include "topic.hpp"
#include "common/yaml.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>

namespace multi_data_monitor
{

void TopicStream::setting(YAML::Node yaml)
{
  name_ = yaml::take_required(yaml, "name").as<std::string>("");
  type_ = yaml::take_optional(yaml, "type").as<std::string>("");
}

void TopicStream::message(const Packet & packet)
{
  (void)packet;
}

void TopicStream::update(ros::Node node)
{
  const auto infos = node->get_publishers_info_by_topic(name_);
  for (const auto & info : infos)
  {
    RCLCPP_INFO_STREAM(node->get_logger(), info.node_name());
    RCLCPP_INFO_STREAM(node->get_logger(), info.node_namespace());
    RCLCPP_INFO_STREAM(node->get_logger(), info.topic_type());
    RCLCPP_INFO_STREAM(node->get_logger(), ros::to_string(info.qos_profile()));
  }
}

}  // namespace multi_data_monitor
