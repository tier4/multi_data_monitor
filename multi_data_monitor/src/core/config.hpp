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

#ifndef CORE__CONFIG_HPP_
#define CORE__CONFIG_HPP_

#include "errors.hpp"
#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>
#include <unordered_map>
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

struct NodeConfig
{
  NodeConfig(YAML::Node yaml, const std::string & path, const std::string & mode);
  ConfigError Error(const std::string message);
  YAML::Node TakeNode(const std::string & name, bool optional = false);
  void CheckUnknownKeys();
  void RefreshTarget();
  NodeConfig * ResolveTarget();

  YAML::Node yaml;
  std::string path;
  std::string mode;
  std::string type;
  std::string name;
  std::string data;

  NodeConfig * target;
  NodeConfig * stream;
  std::vector<NodeConfig *> children;
};

class ConfigFile
{
public:
  explicit ConfigFile(const std::string & file);
  const NodeConfig * GetRoot() const;
  const std::vector<NodeConfig *> GetNodes(const std::string & mode = "") const;
  const std::vector<TopicConfig> & GetTopics() const;
  const std::string GetStyleSheet(const std::string & target = "") const;

private:
  NodeConfig * Parse(YAML::Node yaml, const std::string & path, const std::string & mode);
  NodeConfig * root_;
  std::vector<std::unique_ptr<NodeConfig>> nodes_;
  std::vector<TopicConfig> topics_;
  std::unordered_map<std::string, std::string> stylesheets;
};

}  // namespace multi_data_monitor

#endif  // CORE__CONFIG_HPP_
