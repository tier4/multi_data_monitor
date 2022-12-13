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
#include <cmath>
#include <string>

namespace multi_data_monitor
{

class Units : public multi_data_monitor::Action
{
private:
  double coefficient_;

public:
  void Initialize(const YAML::Node & yaml)
  {
    const auto type = yaml["type"].as<std::string>();
    coefficient_ = 1.0;
    // clang-format off
    if (type == "mps_to_kph") { coefficient_ = 1.0 * 3.6; return; }
    if (type == "kph_to_mps") { coefficient_ = 1.0 / 3.6; return; }
    if (type == "deg_to_rad") { coefficient_ = M_PI / 180.0; return; }
    if (type == "rad_to_deg") { coefficient_ = 180.0 / M_PI; return; }
    // clang-format on
    // TODO(Takagi, Isamu): warning
  }
  MonitorValues Apply(const MonitorValues & input) override
  {
    double value = coefficient_ * input.value.as<double>();
    return {YAML::Node(value), input.attrs};
  }
};

}  // namespace multi_data_monitor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(multi_data_monitor::Units, multi_data_monitor::Action)