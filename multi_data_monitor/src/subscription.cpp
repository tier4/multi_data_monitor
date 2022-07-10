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
  std::cout << "start subscription: " << config_.name << " " << config_.type << std::endl;

  rclcpp::QoS qos(1);
  /*
  rclcpp::QoS qos(qos_depth);
  if (qos_reliability == "reliable") { qos.reliable(); }
  if (qos_reliability == "best_effort") { qos.best_effort(); }
  if (qos_durability == "volatile") { qos.durability_volatile(); }
  if (qos_durability == "transient_local") { qos.transient_local(); }
  */

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

/*
void TopicSubscription::Add(Monitor * monitor, const YAML::Node & qos)
{
  YAML::Node temp_qos = qos ? qos : YAML::Node();
  const auto temp_depth = temp_qos["depth"].as<size_t>(1);
  const auto temp_reliability = temp_qos["reliability"].as<std::string>("default");
  const auto temp_durability = temp_qos["durability"].as<std::string>("default");
  if (qos_empty)
  {
    qos_empty = false;
    qos_depth = temp_depth;
    qos_reliability = temp_reliability;
    qos_durability = temp_durability;
  }
  else
  {
    if (qos_depth != temp_depth || qos_reliability != temp_reliability || qos_durability != temp_durability)
    {
      throw std::runtime_error("QoS setting does not match");
    }
  }

  if (support_ != monitor->GetTypeSupport())
  {
    const auto type1 = support_->GetTypeName();
    const auto type2 = monitor->GetTypeSupport()->GetTypeName();
    throw std::runtime_error("Topic '" + name_ + "' has multiple types [" + type1 + ", " + type2 + "]");
  }
  monitors_.push_back(monitor);
}


*/

}  // namespace monitors
