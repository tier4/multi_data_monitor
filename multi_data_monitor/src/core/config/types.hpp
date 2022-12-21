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

#include "common/typedef.hpp"
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>

namespace multi_data_monitor::builtin
{

constexpr char apply[] = "apply";
constexpr char print[] = "print";
constexpr char relay[] = "relay";
constexpr char subscription[] = "subscription";

constexpr char topic[] = "@topic";
constexpr char field[] = "@field";
constexpr char panel[] = "@panel";

}  // namespace multi_data_monitor::builtin

namespace multi_data_monitor::plugin::name
{

constexpr char package[] = "multi_data_monitor";
constexpr char widget[] = "multi_data_monitor::BasicWidget";

}  // namespace multi_data_monitor::plugin::name

namespace multi_data_monitor
{

using NodeClass = std::string;
using NodeLabel = std::string;

struct WidgetItem final
{
  YAML::Node yaml;
  WidgetLink link;
};

struct CommonData
{
  CommonData(const NodeClass & klass, const NodeLabel & label);
  virtual ~CommonData();
  static inline int created = 0;
  static inline int removed = 0;

  const NodeClass klass;
  const NodeLabel label;
  bool system = false;
  bool unused = false;
  // TODO(Takagi, Isamu): debug info for exception
};

struct StreamData final : public CommonData
{
  using CommonData::CommonData;
  void dump() const;
  YAML::Node yaml;
  StreamLink input;
  StreamLink refer;
  WidgetLink panel;
  ActionLink apply;
};

struct WidgetData final : public CommonData
{
  using CommonData::CommonData;
  void dump() const;
  YAML::Node yaml;
  WidgetLink refer;
  std::vector<WidgetItem> items;
};

struct DesignData final
{
  DesignData() { ++created; }
  ~DesignData() { ++removed; }
  static inline int created = 0;
  static inline int removed = 0;

  std::string klass;
  std::string stylesheet;
};

struct ConfigFile final
{
  std::string version;
  YAML::Node yaml;
};

struct ConfigData final
{
  StreamLink create_stream(const NodeClass & klass, const NodeLabel & label = {}, YAML::Node yaml = {});
  WidgetLink create_widget(const NodeClass & klass, const NodeLabel & label = {}, YAML::Node yaml = {});

  std::vector<StreamLink> streams;
  std::vector<WidgetLink> widgets;
  std::vector<DesignLink> designs;
};

class ConfigParserInterface
{
public:
  virtual ~ConfigParserInterface() = default;
  virtual std::string name() = 0;
  virtual ConfigData execute(const ConfigData & input) = 0;
};

template <class T>
struct NodeTraits;

template <>
struct NodeTraits<StreamLink>
{
  static constexpr auto Name = "stream";
};

template <>
struct NodeTraits<WidgetLink>
{
  static constexpr auto Name = "widget";
};

}  // namespace multi_data_monitor

#endif  // CORE__CONFIG__TYPES_HPP_
