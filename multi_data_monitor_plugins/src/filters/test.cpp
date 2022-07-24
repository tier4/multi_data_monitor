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
#include <string>

namespace multi_data_monitor
{

class TestFilter : public multi_data_monitor::Action
{
public:
  MonitorValues Apply(const MonitorValues & input) override
  {
    const auto text = "[" + input.value.as<std::string>() + "]";
    return MonitorValues{YAML::Node(text), input.attrs};
  }
};

}  // namespace multi_data_monitor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(multi_data_monitor::TestFilter, multi_data_monitor::Action)