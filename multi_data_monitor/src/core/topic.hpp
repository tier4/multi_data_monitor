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

#ifndef CORE__TOPIC_HPP_
#define CORE__TOPIC_HPP_

#include "stream.hpp"
#include <generic_type_support/generic_type_support.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>
#include <vector>

namespace multi_data_monitor
{

class FieldConfig;
class TopicConfig;

// TODO(Takagi, Isamu): This class can be unified with filters.
class Field : public OutputStream
{
public:
  Field(const FieldConfig & config, const generic_type_support::GenericMessage & support);
  void Callback(const MonitorValues & input) override;
  std::string GetData() const { return data_; }

private:
  std::string data_;
  std::shared_ptr<generic_type_support::GenericMessage::GenericAccess> access_;
};

// TODO(Takagi, Isamu): This class can inherit from output stream.
class Topic
{
public:
  explicit Topic(const TopicConfig & config);
  void Subscribe(rclcpp::Node::SharedPtr & node);
  void Unsubscribe();
  std::vector<std::unique_ptr<Field>> & GetFields() { return fields_; }
  std::string GetName() const { return name_; }

private:
  std::string name_;
  std::string type_;
  std::shared_ptr<generic_type_support::GenericMessage> support_;
  rclcpp::QoS qos_;
  rclcpp::GenericSubscription::ConstSharedPtr subscription_;
  std::vector<std::unique_ptr<Field>> fields_;
};

}  // namespace multi_data_monitor

#endif  // CORE__TOPIC_HPP_
