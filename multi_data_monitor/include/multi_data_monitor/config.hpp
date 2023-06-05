// Copyright 2023 Takagi, Isamu
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

#ifndef MULTI_DATA_MONITOR__CONFIG_HPP_
#define MULTI_DATA_MONITOR__CONFIG_HPP_

#include <multi_data_monitor/errors.hpp>
#include <yaml-cpp/yaml.h>
#include <string>

namespace multi_data_monitor
{

struct ConfigObject
{
  ConfigObject(YAML::Node yaml, const std::string & track) : yaml(yaml), track(track) {}

  template <class T>
  T take_required_data(const std::string & field)
  {
    const auto node = yaml[field];
    if (node)
    {
      yaml.remove(field);
      return node.as<T>();
    }
    throw FieldNotFound(field, track);
  }

  YAML::Node yaml;
  const std::string track;
};

}  // namespace multi_data_monitor

#endif  // MULTI_DATA_MONITOR__CONFIG_HPP_
