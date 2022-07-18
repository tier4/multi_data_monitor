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
#include <memory>
#include <optional>
#include <string>
using std::cout;
using std::endl;

namespace multi_data_monitor
{

/*
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
*/

ConfigNode::ConfigNode(YAML::Node yaml, const std::string & path)
{
  if (yaml.IsScalar())
  {
    this->path = path;
    this->yaml["class"] = "target";
    this->yaml["name"] = yaml;
  }
  else
  {
    this->path = path;
    this->yaml.reset(yaml);
  }
}

YAML::Node ConfigNode::TakeNode(const std::string & name, bool optional)
{
  const auto node = yaml[name];
  if (optional || node)
  {
    yaml.remove(name);
    return node;
  }
  throw ConfigError::ParseFile(fmt::format("{} has no '{}'", path, name));
}

ConfigNode * ConfigTrees::Parse(YAML::Node yaml, const std::string & path)
{
  const auto node = nodes.emplace_back(std::make_unique<ConfigNode>(yaml, path)).get();

  if (!node->yaml.IsMap())
  {
    throw ConfigError::ParseFile(node->path + " is not a dict");
  }

  node->type = node->TakeNode("class").as<std::string>();

  if (node->type == "topic")
  {
    node->name = node->TakeNode("name").as<std::string>();
    node->data = node->TakeNode("data").as<std::string>();
  }

  const auto input = node->TakeNode("input", true);
  if (input)
  {
    node->input = Parse(input, node->path + ".input");
  }

  for (const std::string field : {"children", "rules"})
  {
    const auto children = node->TakeNode(field, true);
    if (children)
    {
      const std::string path = node->path + "." + field;
      if (!children.IsSequence())
      {
        throw ConfigError::ParseFile(path + " is not a list");
      }
      for (size_t i = 0, n = children.size(); i < n; ++i)
      {
        node->children.push_back(Parse(children[i], fmt::format("{}[{}]", path, i)));
      }
    }
  }
  return node;
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

ConfigFile::ConfigFile(const std::string & package, const std::string & path)
{
  try
  {
    const auto yaml = LoadFile(package, path);

    ConfigTrees trees;

    cout << "========================================================" << endl;

    for (const auto & pair : yaml["monitors"])
    {
      const auto name = pair.first.as<std::string>();
      const auto temp = trees.Parse(pair.second, name);
      (void)temp;
    }

    for (const auto & pair : yaml["messages"])
    {
      const auto name = pair.first.as<std::string>();
      const auto temp = trees.Parse(pair.second, name);
      (void)temp;
    }

    for (const auto & node : trees.nodes)
    {
      cout << fmt::format("Node     {:50} ({}, {}, {})", node->path, node->type, node->name, node->data) << endl;
      if (node->input)
      {
        cout << fmt::format("  Input  {}", node->input->path) << endl;
      }
      for (const auto child : node->children)
      {
        cout << fmt::format("  Child  {}", child->path) << endl;
      }
    }

    cout << "========================================================" << endl;
  }
  catch (YAML::Exception & error)
  {
    throw ConfigError::LoadFile(error.what());
  }
}

}  // namespace multi_data_monitor
