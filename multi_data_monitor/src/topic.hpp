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

#ifndef TOPIC_HPP_
#define TOPIC_HPP_

#include <generic_type_support/generic_type_support.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>
#include <vector>

namespace multi_data_monitor
{

class Stream;
class TopicConfig;

class Topic
{
public:
  explicit Topic(const TopicConfig & config);
  ~Topic();
  void Subscribe(rclcpp::Node::SharedPtr node);
  void Unsubscribe();
  void Register(const std::string & field, Stream * output);

private:
  std::string name_;
  std::string type_;
  std::shared_ptr<generic_type_support::GenericMessage> support_;
  rclcpp::QoS qos_;
  rclcpp::GenericSubscription::ConstSharedPtr subscription_;

  class Field;
  std::vector<Field> fields_;
};

}  // namespace multi_data_monitor

#endif  // TOPIC_HPP_
