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
#include <iostream>  // DEBUG
#include <string>
using std::cout;
using std::endl;

namespace multi_data_monitor
{

struct ParseError : public std::runtime_error
{
  using std::runtime_error::runtime_error;
};

void CheckUnused(const YAML::Node & yaml)
{
  std::string unused;
  for (const auto & pair : yaml)
  {
    unused += pair.first.as<std::string>() + " ";
  }
  if (!unused.empty())
  {
    unused.pop_back();
    throw ParseError(fmt::format(" has unused keys `{}`", unused));
  }
}

YAML::Node TakeNode(YAML::Node yaml, const std::string & name, bool optional = false)
{
  const auto node = yaml[name];
  if (optional || node.IsDefined())
  {
    yaml.remove(name);
    return node;
  }
  throw ParseError(fmt::format(" has no '{}'", name));
}

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

FilterConfig::FilterConfig(YAML::Node yaml)
{
  TakeNode(yaml, "rules", true);  // TODO(Takagi, Isamu)
}

TopicConfig::TopicConfig(YAML::Node yaml)
{
  name = TakeNode(yaml, "name").as<std::string>();
  type = TakeNode(yaml, "type").as<std::string>();
  data = TakeNode(yaml, "data").as<std::string>();
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

void ConfigFile::ParseNode(bool view, YAML::Node yaml, const std::string & path)
{
  std::string mark = path + (view ? "" : ".input");
  try
  {
    if (yaml.IsScalar())
    {
      return;
    }

    if (!yaml.IsMap())
    {
      throw ParseError(" is not a dict");
    }

    const auto name = TakeNode(yaml, "class").as<std::string>();
    cout << (view ? "  view: " : "  data: ") << name << " (" << path << ")" << endl;

    if (name == "filter")
    {
      const auto config = FilterConfig(yaml);
      (void)config;
    }
    if (name == "topic")
    {
      const auto config = TopicConfig(yaml);
      (void)config;
    }

    const auto children = TakeNode(yaml, "children", true);  // TODO(Takagi, Isamu)
    const auto input = TakeNode(yaml, "input", true);        // TODO(Takagi, Isamu)
    TakeNode(yaml, "param", true);                           // TODO(Takagi, Isamu)

    mark = path + (view ? "" : "." + name);
    CheckUnused(yaml);

    if (input.IsDefined())
    {
      ParseNode(false, input, mark);
    }

    if (children.IsDefined())
    {
      if (!children.IsSequence())
      {
        throw ParseError(".children is not a list");
      }
      for (size_t i = 0, n = children.size(); i < n; ++i)
      {
        ParseNode(true, children[i], fmt::format("{}[{}]", path, i));
      }
    }
  }
  catch (const ParseError & error)
  {
    throw ConfigError::Parse(mark + error.what());
  }
}

}  // namespace multi_data_monitor
