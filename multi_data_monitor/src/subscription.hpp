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

#ifndef SUBSCRIPTION_HPP_
#define SUBSCRIPTION_HPP_

#include "config/config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <generic_type_support/generic_type_support.hpp>

namespace monitors
{

struct TopicField
{
  FieldConfig config;
  generic_type_support::GenericMessage::GenericAccess access;
};

class TopicSubscription
{
public:
  TopicSubscription(const TopicConfig & config);
  void Start(const rclcpp::Node::SharedPtr & node);
  void AddField(const FieldConfig & config);
  TopicField & GetField(const std::string & name);

private:
  // NOTE: declaration order where the subscription stops first
  TopicConfig config_;
  rclcpp::GenericSubscription::SharedPtr subscription_;
  generic_type_support::GenericMessage message_;
  std::unordered_map<std::string, TopicField> fields_;
};

}  // namespace monitors

#endif  // SUBSCRIPTION_HPP_
