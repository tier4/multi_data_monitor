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

namespace multi_data_monitor
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
    for (const auto & field : fields_)
    {
      const YAML::Node node = field.second.access->Access(yaml);
      for (const auto & monitor : field.second.monitors)
      {
        monitor->Callback(node);
      }
    }
  };
  subscription_ = node->create_generic_subscription(config_.name, config_.type, qos, callback);
}

void TopicSubscription::AddField(const FieldConfig & config)
{
  if (fields_.count(config.name) == 0)
  {
    const auto access = message_.GetAccess(config.name);
    fields_.insert(std::make_pair(config.name, TopicField{config, access, {}}));
  }

  const auto & access = fields_.at(config.name).access;
  if (config.type.empty())
  {
    if (access->IsMessage())
    {
      throw ConfigError("field '" + config.name + "' is not a primitive type");
    }
  }
  else
  {
    if (access->GetTypeName() != config.type)
    {
      throw ConfigError("field '" + config.name + "' is not '" + config.type +"'");
    }
  }
}

TopicField & TopicSubscription::GetField(const std::string & name)
{
  return fields_.at(name);
}

}  // namespace multi_data_monitor
