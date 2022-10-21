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

#ifndef MULTI_DATA_MONITOR__VALUES_HPP_
#define MULTI_DATA_MONITOR__VALUES_HPP_

#include <yaml-cpp/yaml.h>
#include <string>
#include <unordered_map>

namespace multi_data_monitor
{

struct MonitorValues
{
  MonitorValues Clone() const { return MonitorValues{YAML::Clone(value), attrs}; }
  YAML::Node value;
  std::unordered_map<std::string, std::string> attrs;
};

}  // namespace multi_data_monitor

#endif  // MULTI_DATA_MONITOR__VALUES_HPP_
