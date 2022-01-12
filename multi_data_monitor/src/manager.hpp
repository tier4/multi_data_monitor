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

#ifndef MANAGER_HPP_
#define MANAGER_HPP_

#include "monitor.hpp"
#include "subscription.hpp"
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "generic_type_support/generic_type_support.hpp"

namespace monitors
{

class Manager
{
public:
  void Load(const std::string & path);
  void CreateMonitors();
  void CreateSubscription(const rclcpp::Node::SharedPtr & node);
  void Build(QWidget * panel);

private:
  using GenericMessageSupport = generic_type_support::GenericMessageSupport;
  YAML::Node yaml_;

  // NOTE: declaration order for the destructors
  std::map<std::string, std::unique_ptr<const GenericMessageSupport>> supports_;
  std::map<std::string, std::unique_ptr<Monitor>> monitors_;
  std::map<std::string, std::unique_ptr<TopicSubscription>> subscriptions_;
};

}  // namespace monitors

#endif  // MANAGER_HPP_
