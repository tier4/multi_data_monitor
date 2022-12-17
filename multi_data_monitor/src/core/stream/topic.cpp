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
#include "common/exceptions.hpp"
#include "common/text.hpp"
#include "common/yaml.hpp"
#include <generic_type_utility/generic_type_utility.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace multi_data_monitor
{

rclcpp::QoS convert_qos(const std::string text)
{
  // clang-format off
  static const std::unordered_map<std::string, rmw_qos_reliability_policy_t> reliabilities =
  {
    {"d", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT},
    {"r", RMW_QOS_POLICY_RELIABILITY_RELIABLE},
    {"b", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT}
  };
  static const std::unordered_map<std::string, rmw_qos_durability_policy_t> durabilities =
  {
    {"d", RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT},
    {"t", RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL},
    {"v", RMW_QOS_POLICY_DURABILITY_VOLATILE}
  };
  // clang-format on

  if (text.length() < 3)
  {
    throw ConfigError("topic qos must be at least 3 characters");
  }

  const auto create_qos = [](const std::string & depth)
  {
    try
    {
      return rclcpp::QoS(std::stoi(depth));
    }
    catch (const std::exception &)
    {
      throw ConfigError("invalid qos depth: " + depth);
    }
  };

  rclcpp::QoS qos = create_qos(text.substr(2));
  const auto r = text.substr(0, 1);
  const auto d = text.substr(1, 1);

  if (reliabilities.count(r) == 0)
  {
    throw ConfigError("unknown qos reliability: " + r);
  }
  qos.reliability(reliabilities.at(r));

  if (durabilities.count(d) == 0)
  {
    throw ConfigError("unknown qos durability: " + d);
  }
  qos.durability(durabilities.at(d));

  return qos;
}

void TopicStream::setting(YAML::Node yaml)
{
  name_ = yaml::take_required(yaml, "name").as<std::string>("");
  type_ = yaml::take_optional(yaml, "type").as<std::string>("");
  qos_ = yaml::take_optional(yaml, "qos").as<std::string>("");
}

void TopicStream::message(const Packet & packet)
{
  outputs(packet);
}

void TopicStream::update(ros::Node node)
{
  if (!sub_)
  {
    create_subscription(node);
  }
}

void TopicStream::create_subscription(ros::Node node)
{
  auto type = type_;
  auto qos = qos_;

  if (type.empty() || qos.empty())
  {
    const auto infos = node->get_publishers_info_by_topic(name_);
    if (infos.empty())
    {
      RCLCPP_WARN_STREAM(node->get_logger(), "no topic info: " + name_);
      return;
    }

    std::unordered_set<std::string> types;
    std::unordered_set<std::string> qoses;
    for (const auto & info : infos)
    {
      types.insert(info.topic_type());
      qoses.insert(ros::to_string(info.qos_profile()));
    }

    if (type.empty())
    {
      if (types.size() != 1)
      {
        RCLCPP_WARN_STREAM(node->get_logger(), "topic type is not unique: " << text::join(types));
        return;
      }
      type = *types.begin();
    }

    if (qos.empty())
    {
      if (qoses.size() != 1)
      {
        RCLCPP_WARN_STREAM(node->get_logger(), "topic qos is not unique: " << text::join(qoses));
        return;
      }
      qos = *qoses.begin();
    }
  }

  const auto callback = [this](const std::shared_ptr<const rclcpp::SerializedMessage> serialized)
  {
    const auto value = generic_->ConvertYAML(*serialized);
    const auto attrs = std::unordered_map<std::string, std::string>();
    message(Packet{value, attrs});
  };

  generic_ = std::make_shared<generic_type_utility::GenericMessage>(type);
  sub_ = node->create_generic_subscription(name_, type, convert_qos(qos), callback);
}

}  // namespace multi_data_monitor
