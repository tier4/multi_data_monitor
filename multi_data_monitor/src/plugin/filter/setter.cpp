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

#include "conditions.hpp"
#include <multi_data_monitor/filter.hpp>
#include <optional>

//
#include <iostream>

namespace multi_data_monitor
{

struct SetIf : public BasicFilter
{
public:
  void setup(YAML::Node yaml) override;
  Packet apply(const Packet & packet) override;

private:
  using Condition = conditions::Condition;

  struct PacketAction
  {
    std::optional<YAML::Node> value;
    std::unordered_map<std::string, std::string> attrs;
  };
  struct SetterAction
  {
    Condition condition;
    PacketAction packet;
  };
  std::vector<SetterAction> actions_;
};

void SetIf::setup(YAML::Node yaml)
{
  const auto get_packet_action = [](const YAML::Node & yaml)
  {
    PacketAction action;
    if (yaml["value"])
    {
      action.value = yaml["value"];
    }
    if (yaml["attrs"])
    {
      for (const auto & pair : yaml["attrs"])
      {
        const auto name = pair.first.as<std::string>();
        const auto attr = pair.second.as<std::string>();
        action.attrs[name] = attr;
      }
    }
    return action;
  };

  if (yaml["list"].IsSequence())
  {
    for (const auto & item : yaml["list"])
    {
      auto action = get_packet_action(item);
      actions_.push_back({Condition(item), action});
    }
  }
}

Packet SetIf::apply(const Packet & packet)
{
  for (const auto & [condition, action] : actions_)
  {
    bool result = condition.eval(packet.value);
    std::cout << packet.value << " " << result << std::endl;
    if (result)
    {
      YAML::Node value = action.value.value_or(packet.value);
      Packet::Attrs attrs = action.attrs;
      for (const auto & pair : packet.attrs)
      {
        attrs.insert(pair);
      }
      return {value, attrs};
    }
  }
  return packet;
}

}  // namespace multi_data_monitor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(multi_data_monitor::SetIf, multi_data_monitor::BasicFilter)
