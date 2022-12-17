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

#ifndef CORE__COMMON__RCLCPP_HPP_
#define CORE__COMMON__RCLCPP_HPP_

#include <memory>
#include <string>

namespace rclcpp
{

class Node;
class TimerBase;
class GenericSubscription;
class QoS;

}  // namespace rclcpp

namespace multi_data_monitor::ros
{

using Node = std::shared_ptr<rclcpp::Node>;
using Timer = std::shared_ptr<rclcpp::TimerBase>;
using Subscription = std::shared_ptr<rclcpp::GenericSubscription>;

std::string to_string(const rclcpp::QoS & qos);

}  // namespace multi_data_monitor::ros

#endif  // CORE__COMMON__RCLCPP_HPP_
