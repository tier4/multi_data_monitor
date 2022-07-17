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
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

namespace multi_data_monitor
{

class Manager
{
public:
  void Load(const std::string & path, rclcpp::Node::SharedPtr node);
  void Build(QWidget * panel);
  void Start(const rclcpp::Node::SharedPtr & node);

private:
  // NOTE: declaration order where the subscription stops first
  Monitor * root_;
  std::unordered_map<std::string, TopicSubscription> subscriptions_;
  std::unordered_map<std::string, std::unique_ptr<Monitor>> monitors_;
};

}  // namespace multi_data_monitor

#endif  // MANAGER_HPP_