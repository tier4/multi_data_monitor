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

namespace multi_data_monitor::builtin
{

const char relay[] = "relay";
const char print[] = "print";
const char subscription[] = "subscription";
const char topic[] = "@topic";
const char field[] = "@field";
const char panel[] = "@panel";

}  // namespace multi_data_monitor::builtin

namespace multi_data_monitor
{

using NodeClass = std::string;
using NodeLabel = std::string;

struct StreamData;
struct WidgetData;
using StreamLink = std::shared_ptr<StreamData>;
using WidgetLink = std::shared_ptr<WidgetData>;
using StreamList = std::vector<StreamLink>;

struct WidgetItem
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

struct StreamData : public CommonData
{
  using CommonData::CommonData;
  void dump() const;
  YAML::Node yaml;
  StreamLink input;
  StreamLink refer;
};

struct WidgetData : public CommonData
{
  using CommonData::CommonData;
  void dump() const;
  YAML::Node yaml;
  StreamLink input;
  WidgetLink refer;
  std::vector<WidgetItem> items;
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
  YAML::Node yaml;
};

struct ConfigData
{
  StreamLink create_stream(const NodeClass & klass, const NodeLabel & label = {}, YAML::Node yaml = {});
  WidgetLink create_widget(const NodeClass & klass, const NodeLabel & label = {}, YAML::Node yaml = {});

  std::vector<StreamLink> streams;
  std::vector<WidgetLink> widgets;
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
