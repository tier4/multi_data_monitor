// Copyright 2021 Takagi, Isamu
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

#ifndef CONFIG__CONFIG_HPP_
#define CONFIG__CONFIG_HPP_

#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

namespace monitors
{

class ConfigError : public std::runtime_error
{
  using std::runtime_error::runtime_error;
};

struct DefaultConfig
{
  DefaultConfig(YAML::Node node);
  std::string klass;
  YAML::Node temp;  // TODO: temp
};

struct TopicConfig
{
  TopicConfig() = default;
  TopicConfig(YAML::Node node);
  std::string name;
  std::string type;
  int depth;
  std::string reliability;
  std::string durability;
};

struct FieldConfig
{
  FieldConfig() = default;
  FieldConfig(YAML::Node node, const std::string & topic);
  std::string topic;
  std::string name;
  std::string type;
};

struct ObjectConfig
{
  ObjectConfig() = default;
  ObjectConfig(YAML::Node node);
  std::string klass;
  YAML::Node custom;
};

struct MonitorConfig
{
  MonitorConfig(YAML::Node node);
  ObjectConfig object;
  std::optional<TopicConfig> topic;
  std::optional<FieldConfig> field;
};

}  // namespace monitors

#endif  // CONFIG__CONFIG_HPP_
