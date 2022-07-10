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

#include "subscription.hpp"

#include <iostream>

namespace monitors
{

TopicSubscription::TopicSubscription(const TopicConfig & config) : message_(config.type)
{
  config_ = config;
}

void TopicSubscription::Start(const rclcpp::Node::SharedPtr & node)
{
  RCLCPP_INFO_STREAM(node->get_logger(), "start subscription: " << config_.name << " " << config_.type << " " << config_.depth << " " << config_.reliability << " " << config_.durability);

  // TODO: throw unknown settings
  rclcpp::QoS qos(config_.depth);
  if (config_.reliability == "reliable") { qos.reliable(); }
  if (config_.reliability == "best_effort") { qos.best_effort(); }
  if (config_.durability == "volatile") { qos.durability_volatile(); }
  if (config_.durability == "transient_local") { qos.transient_local(); }

  const auto callback = [this](const std::shared_ptr<rclcpp::SerializedMessage> serialized)
  {
    const YAML::Node yaml = message_.ConvertYAML(*serialized);
    std::cout << "===============================================================" << std::endl;
    std::cout << yaml << std::endl;
    //for (const auto & monitor : monitors_)
    //{
    //  monitor->Callback(yaml);
    //}
  };
  subscription_ = node->create_generic_subscription(config_.name, config_.type, qos, callback);
}

}  // namespace monitors
