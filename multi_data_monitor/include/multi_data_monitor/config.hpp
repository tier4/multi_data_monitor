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

// DEBUG
#include <iostream>

namespace multi_data_monitor
{

struct ConfigObject
{
  ConfigObject(YAML::Node yaml, const std::string & track, bool check = true) : yaml(yaml), track(track), check(check) {}

  YAML::Node take_node(const std::string & field, bool optional)
  {
    const auto node = yaml[field];
    if (node || optional)
    {
      yaml.remove(field);
      return node;
    }
    throw FieldNotFound(field, track);
  }
  YAML::Node take_required_node(const std::string & field) { return take_node(field, false); }
  YAML::Node take_optional_node(const std::string & field) { return take_node(field, true); }

  template <class T>
  T take_required_data(const std::string & field)
  {
    try
    {
      const auto node = take_required_node(field);
      return node.as<T>();
    }
    catch (const YAML::TypedBadConversion<T> &)
    {
      throw InvalidDataType(field, track);
    }
  }

  template <class T>
  T take_optional_data(const std::string & field, const T & fails)
  {
    try
    {
      const auto node = take_optional_node(field);
      return node ? node.as<T>() : fails;
    }
    catch (const YAML::TypedBadConversion<T> &)
    {
      throw InvalidDataType(field, track);
    }
  }

  ~ConfigObject()
  {
    if (check && yaml && yaml.size())
    {
      std::cout << "==================== " << track << " ====================" << std::endl;
      std::cout << YAML::Dump(yaml) << std::endl;
    }
  }

  YAML::Node yaml;
  const std::string track;
  bool check;
};

}  // namespace multi_data_monitor

#endif  // MULTI_DATA_MONITOR__CONFIG_HPP_
