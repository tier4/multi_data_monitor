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
#include <yaml-cpp/yaml.h>
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

void ParseView(const YAML::Node & yaml, const std::string & path)
{
  if (yaml.IsScalar())
  {
    return;
  }

  if (!yaml["class"])
  {
    throw ConfigError::Parse("no class: " + path);  // TODO(Takagi, Isamu): message
  }

  const auto name = yaml["class"].as<std::string>();
  cout << "  panel: " << name << endl;

  if (yaml["children"])
  {
    for (const auto & node : yaml["children"])
    {
      ParseView(node, path);
    }
  }
}

void ParseData(const YAML::Node & yaml, const std::string & path)
{
  if (yaml.IsScalar())
  {
    return;
  }

  if (!yaml["class"])
  {
    throw ConfigError::Parse("no class: " + path);  // TODO(Takagi, Isamu): message
  }

  const auto name = yaml["class"].as<std::string>();
  cout << "  class: " << name << endl;

  if (yaml["input"])
  {
    ParseData(yaml["input"], path);
  }
}

ConfigFile::ConfigFile(const std::string & package, const std::string & path)
{
  try
  {
    const auto yaml = LoadFile(package, path);

    TopicConfig topics;
    for (const auto & pair : yaml["monitors"])
    {
      const auto name = pair.first.as<std::string>();
      cout << "========================================================" << endl;
      cout << "monitor: " << name << endl;
      ParseView(pair.second, name);
    }

    for (const auto & pair : yaml["messages"])
    {
      const auto name = pair.first.as<std::string>();
      cout << "========================================================" << endl;
      cout << "message: " << name << endl;
      ParseData(pair.second, name);
    }
  }
  catch (YAML::Exception & error)
  {
    throw ConfigError::LoadFile(error.what());
  }
}

}  // namespace multi_data_monitor
