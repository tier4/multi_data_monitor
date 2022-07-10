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

class TopicSubscription
{
public:
  TopicSubscription(const TopicConfig & config);
  void Start(const rclcpp::Node::SharedPtr & node);

private:
  // NOTE: declaration order where the subscription stops first
  TopicConfig config_;
  rclcpp::GenericSubscription::SharedPtr subscription_;
  generic_type_support::GenericMessage message_;
};

/*
class TopicSubscription
{
public:
  TopicSubscription(const std::string & name, const generic_type_support::GenericMessageSupport * support);
  void Add(Monitor * monitor, const YAML::Node & qos);

private:
  void Callback(const std::shared_ptr<rclcpp::SerializedMessage> serialized) const;

  // No need to release raw pointer since it is a reference to unique pointer.
  const std::string name_;
  const generic_type_support::GenericMessageSupport * const support_;
  std::vector<Monitor *> monitors_;

  // temporary
  bool qos_empty;
  size_t qos_depth;
  std::string qos_reliability;
  std::string qos_durability;
};
*/

}  // namespace monitors

#endif  // SUBSCRIPTION_HPP_
