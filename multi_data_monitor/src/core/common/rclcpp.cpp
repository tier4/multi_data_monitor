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

#include "rclcpp.hpp"
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace multi_data_monitor::ros
{

std::string to_string(rmw_qos_reliability_policy_t reliability)
{
  switch (reliability)
  {
    case RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT:
      return "d";
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
      return "r";
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
      return "b";
    default:
      return "x";
  }
}

std::string to_string(rmw_qos_durability_policy_t durability)
{
  switch (durability)
  {
    case RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT:
      return "d";
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
      return "t";
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
      return "v";
    default:
      return "x";
  }
}

std::string to_string(const rclcpp::QoS & qos)
{
  const auto profile = qos.get_rmw_qos_profile();
  return to_string(profile.reliability) + to_string(profile.durability) + std::to_string(profile.depth);
}

}  // namespace multi_data_monitor::ros
