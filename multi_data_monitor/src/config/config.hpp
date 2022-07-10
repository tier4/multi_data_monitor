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

struct TopicConfig
{
  TopicConfig() = default;
  TopicConfig(YAML::Node node);
  std::string name;
  std::string type;
};

struct MonitorConfig
{
  MonitorConfig(YAML::Node node);
  YAML::Node config;
  std::optional<TopicConfig> topic;
};

}  // namespace monitors

#endif  // CONFIG__CONFIG_HPP_
