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
#include "config.hpp"
#include "stream.hpp"
#include <memory>
#include <string>

// clang-format off
#include <iostream>
#include <fmt/format.h>
using std::cout;
using std::endl;
// clang-format on

namespace multi_data_monitor
{

Field::Field(const FieldConfig & config, const generic_type_support::GenericMessage & support)
{
  data_ = config.data;
  access_ = support.GetAccess(config.data);
}

void Field::Callback(const YAML::Node & yaml)
{
  cout << data_ << ": " << access_->Access(yaml) << endl;
}

Topic::Topic(const TopicConfig & config) : qos_(config.depth)
{
  name_ = config.name;
  type_ = config.type;

  // clang-format off
  if (config.reliability == "R") { qos_.reliable(); }
  if (config.reliability == "B") { qos_.best_effort(); }
  if (config.durability == "V") { qos_.durability_volatile(); }
  if (config.durability == "T") { qos_.transient_local(); }
  // clang-format on

  support_ = std::make_shared<generic_type_support::GenericMessage>(config.type);
  for (const auto & field : config.fields)
  {
    fields_.push_back(std::make_unique<Field>(field, *support_));
  }
}

void Topic::Subscribe(rclcpp::Node::SharedPtr node)
{
  const auto callback = [this](const std::shared_ptr<rclcpp::SerializedMessage> message)
  {
    cout << "========== " << name_ << endl;
    const auto yaml = support_->ConvertYAML(*message);
    for (auto & field : fields_)
    {
      field->Callback(yaml);
    }
  };
  subscription_ = node->create_generic_subscription(name_, type_, qos_, callback);
}

void Topic::Unsubscribe()
{
  subscription_.reset();
}

}  // namespace multi_data_monitor
