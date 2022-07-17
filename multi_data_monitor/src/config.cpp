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

#include "config.hpp"
#include "errors.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fmt/format.h>
#include <filesystem>
#include <iostream>
#include <string>
using std::cout;
using std::endl;

namespace multi_data_monitor
{

YAML::Node LoadFile(const std::string & package, const std::string & source)
{
  try
  {
    auto path = std::filesystem::path();
    if (!package.empty())
    {
      path.append(ament_index_cpp::get_package_share_directory(package));
    }
    path.append(source);

    if (std::filesystem::exists(path))
    {
      return YAML::LoadFile(path);
    }
    throw ConfigError::LoadFile("file not found: " + path.string());
  }
  catch (const ament_index_cpp::PackageNotFoundError & error)
  {
    throw ConfigError::LoadFile("package not found: " + package);
  }
  catch (YAML::Exception & error)
  {
    throw ConfigError::LoadFile(error.what());
  }
}

ConfigFile::ConfigFile(const std::string & package, const std::string & path)
{
  try
  {
    const auto yaml = LoadFile(package, path);

    for (const auto & pair : yaml["monitors"])
    {
      const auto name = pair.first.as<std::string>();
      cout << "========================================================" << endl;
      cout << "monitor: " << name << endl;
      ParseNode(true, pair.second, name);
    }

    for (const auto & pair : yaml["messages"])
    {
      const auto name = pair.first.as<std::string>();
      cout << "========================================================" << endl;
      cout << "message: " << name << endl;
      ParseNode(false, pair.second, name);
    }
  }
  catch (YAML::Exception & error)
  {
    throw ConfigError::LoadFile(error.what());
  }
}

void ConfigFile::ParseNode(bool view, const YAML::Node & yaml, const std::string & parent)
{
  if (yaml.IsScalar())
  {
    return;
  }

  if (!yaml["class"])
  {
    throw ConfigError::Parse(fmt::format("node '{}{}.class' not found", parent, view ? "" : ".input"));
  }

  const auto name = yaml["class"].as<std::string>();
  cout << (view ? "  view: " : "  data: ") << name << " (" << parent << ")" << endl;

  const auto path = parent + (view ? "" : "." + name);
  if (yaml["input"])
  {
    ParseNode(false, yaml["input"], path);
  }

  const auto children = yaml["children"];
  if (children.IsDefined())
  {
    if (!children.IsSequence())
    {
      throw ConfigError::Parse(fmt::format("node '{}.children' is not an array", path));
    }
    for (size_t i = 0, n = children.size(); i < n; ++i)
    {
      ParseNode(true, children[i], fmt::format("{}[{}]", path, i));
    }
  }
}

TopicConfig::TopicConfig(const YAML::Node & yaml)
{
}

}  // namespace multi_data_monitor
