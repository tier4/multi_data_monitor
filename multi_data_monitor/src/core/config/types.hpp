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

#ifndef CORE__CONFIG__TYPES_HPP_
#define CORE__CONFIG__TYPES_HPP_

#include <yaml-cpp/yaml.h>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace multi_data_monitor
{

using NodeClass = std::string;
using NodeLabel = std::string;

struct StreamData;
using StreamLink = std::shared_ptr<StreamData>;

struct StreamData
{
  static StreamLink Create(const NodeClass & klass, YAML::Node yaml);
  static StreamLink Create(const NodeClass & klass, const NodeLabel & label, YAML::Node yaml);
  void dump() const;

  const NodeClass klass;
  const NodeLabel label;
  YAML::Node yaml;
  StreamLink refer;
  StreamLink input;
};

struct TopicData
{
  enum class Reliability { Auto, Default, Reliable, BestEffort };
  enum class Durability { Auto, Default, Volatile, TransientLocal };
  std::string name;
  std::string type;
  size_t depth;
  Reliability reliability;
  Durability durability;
};

struct FieldData
{
  std::string name;
  std::string type;
};

class StyleSheetConfig
{
};

class StyleSheetStore
{
};

struct ConfigFile
{
  std::string version;
  std::vector<YAML::Node> stylesheets;
  std::vector<YAML::Node> widgets;
  std::vector<YAML::Node> streams;
  std::vector<YAML::Node> subscriptions;
};

struct ConfigData
{
  std::vector<std::shared_ptr<StreamData>> streams;
};

}  // namespace multi_data_monitor

#endif  // CORE__CONFIG__TYPES_HPP_
