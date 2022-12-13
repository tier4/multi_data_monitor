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

#ifndef CORE2__CONFIG__TYPES_HPP_
#define CORE2__CONFIG__TYPES_HPP_

#include <yaml-cpp/yaml.h>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace multi_data_monitor
{

using NodeIndex = size_t;
using NodeClass = std::string;
using NodeLabel = std::string;

struct NodeData
{
  void dump() const;
  static inline NodeIndex serial = 0;
  static inline NodeIndex NewIndex() { return ++serial; }
  const NodeIndex index;
  const NodeClass klass;
  const NodeLabel label;
};

template <class DataType>
struct NodeLink
{
  NodeLabel label;
  std::shared_ptr<DataType> link;
  YAML::Node yaml;
};

struct StreamData;
struct WidgetData;
using StreamLink = NodeLink<StreamData>;
using WidgetLink = NodeLink<WidgetData>;

struct StreamData
{
  void dump() const;
  const NodeClass klass;
  const NodeLabel label;
  std::optional<StreamLink> input;
  YAML::Node yaml;
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

struct ConfigData
{
  std::vector<std::shared_ptr<StreamData>> streams;
};

}  // namespace multi_data_monitor

#endif  // CORE2__CONFIG__TYPES_HPP_
