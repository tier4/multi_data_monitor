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

#include <multi_data_monitor/action.hpp>
#include <optional>
#include <string>
#include <unordered_map>

namespace multi_data_monitor
{

// TODO(Takagi, Isamu): utility
std::unordered_map<std::string, std::string> AttrsFrom(const YAML::Node yaml)
{
  std::unordered_map<std::string, std::string> attrs;
  for (const auto & node : yaml)
  {
    attrs[node.first.as<std::string>()] = node.second.as<std::string>();
  }
  return attrs;
}

// TODO(Takagi, Isamu): utility
MonitorValues MonitorValuesFrom(const YAML::Node yaml)
{
  return {yaml["value"], AttrsFrom(yaml["attrs"])};
}

class Switch : public multi_data_monitor::Action
{
private:
  std::optional<MonitorValues> default_;
  std::unordered_map<std::string, MonitorValues> mapping_;

public:
  void Initialize(const YAML::Node & yaml)
  {
    for (const auto & node : yaml["mapping"])
    {
      const auto condition = node.first.as<std::string>();
      const auto execution = MonitorValuesFrom(node.second);
      mapping_.insert(std::make_pair(condition, execution));
    }
    if (yaml["default"])
    {
      default_ = MonitorValuesFrom(yaml["default"]);
    }
  }
  MonitorValues Apply(const MonitorValues & input) override
  {
    const auto iter = mapping_.find(input.value.as<std::string>());
    if (iter != mapping_.end())
    {
      return iter->second;
    }
    return default_.value_or(input);
  }
};

}  // namespace multi_data_monitor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(multi_data_monitor::Switch, multi_data_monitor::BasicFilter)
