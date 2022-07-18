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

#ifndef CONFIG_HPP_
#define CONFIG_HPP_

#include "errors.hpp"
#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>
#include <vector>

namespace multi_data_monitor
{

struct FieldConfig
{
  std::string data;
};

struct TopicConfig
{
  std::string name;
  std::string type;
  size_t depth;
  std::string reliability;
  std::string durability;
  std::vector<FieldConfig> fields;
};

struct ConfigNode
{
  ConfigNode(YAML::Node yaml, const std::string & path);
  ConfigError Error(const std::string message);
  void CheckUnknownKeys();
  YAML::Node TakeNode(const std::string & name, bool optional = false);

  YAML::Node yaml;
  std::string path;
  std::string type;
  std::string name;
  std::string data;

  ConfigNode * input = nullptr;
  std::vector<ConfigNode *> children;
};

class ConfigFile
{
public:
  ConfigFile(const std::string & package, const std::string & file);

  // private:
  ConfigNode * Parse(YAML::Node yaml, const std::string & path);
  std::vector<TopicConfig> topics_;
  std::vector<std::unique_ptr<ConfigNode>> nodes_;
};

}  // namespace multi_data_monitor

#endif  // CONFIG_HPP_
