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
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fmt/format.h>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace multi_data_monitor
{

class FieldMerge
{
public:
  void Add(ConfigNode * node);

private:
  std::string data;
};

class TopicMerge
{
public:
  void Add(ConfigNode * node);
  TopicConfig Convert();

private:
  std::string name;
  std::unordered_set<std::string> types;
  std::unordered_set<size_t> depth;
  std::unordered_set<std::string> reliability;
  std::unordered_set<std::string> durability;
  std::unordered_map<std::string, FieldMerge> fields;
};

void FieldMerge::Add(ConfigNode * node)
{
  data = node->data;
}

void TopicMerge::Add(ConfigNode * node)
{
  const auto type = node->TakeNode("type", true).as<std::string>("");
  if (!type.empty())
  {
    types.insert(type);
  }

  const auto qos = node->TakeNode("qos", true).as<std::string>("");
  if (!qos.empty())
  {
    try
    {
      depth.insert(std::stoul(qos.substr(2)));
      reliability.insert(qos.substr(0, 1));
      durability.insert(qos.substr(1, 1));
    }
    catch (...)
    {
      throw node->Error("has invalid qos settings");
    }
  }

  fields[node->data].Add(node);
  name = node->name;
  node->CheckUnknownKeys();
}

TopicConfig TopicMerge::Convert()
{
  const auto check_unique = [this](auto set, const std::string & var)
  {
    if (set.size() != 1)
    {
      throw ConfigError::ParseFile("topic " + var + " is not unique: " + name);
    }
  };

  // clang-format off
  if (depth.empty()) { depth.insert(1); }
  if (reliability.empty()) { reliability.insert("D"); }
  if (durability.empty()) { durability.insert("D"); }
  // clang-format on

  check_unique(types, "type");
  check_unique(depth, "depth");
  check_unique(reliability, "reliability");
  check_unique(durability, "durability");

  std::vector<FieldConfig> field_configs;
  for (const auto & pair : fields)
  {
    field_configs.push_back(FieldConfig{pair.first});
  }
  return {name, *types.begin(), *depth.begin(), *reliability.begin(), *durability.begin(), field_configs};
}

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
ConfigError ConfigNode::Error(const std::string message)
{
  return ConfigError::ParseFile(fmt::format("{} '{}' {}", path, type, message));
}

void ConfigNode::CheckUnknownKeys()
{
  std::string unknown;
  for (const auto & pair : yaml)
  {
    unknown += pair.first.as<std::string>() + " ";
  }
  if (!unknown.empty())
  {
    unknown.pop_back();
    throw Error(fmt::format("has unknown keys '{}'", unknown));
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
  throw Error(fmt::format("has no '{}'", name));
}

ConfigFile::ConfigFile(const std::string & package, const std::string & file)
{
  try
  {
    // resolve file path
    auto file_path = std::filesystem::path();
    if (!package.empty())
    {
      file_path.append(ament_index_cpp::get_package_share_directory(package));
    }
    file_path.append(file);

    // check to distinguish from errors in YAML::LoadFile
    if (!std::filesystem::exists(file_path))
    {
      throw ConfigError::LoadFile("file not found: " + file_path.string());
    }

    // load config and convert version if possible
    const auto yaml = YAML::LoadFile(file_path);
    if (yaml["version"].as<std::string>("") != "1")
    {
      throw ConfigError::LoadFile("this version is not supported");
    }

    // load widgets
    for (const auto & pair : yaml["widgets"])
    {
      const auto name = pair.first.as<std::string>();
      const auto temp = Parse(pair.second, name);
      (void)temp;
    }

    // load streams
    for (const auto & pair : yaml["streams"])
    {
      const auto name = pair.first.as<std::string>();
      const auto temp = Parse(pair.second, name);
      (void)temp;
    }

    // merge topic settings
    std::unordered_map<std::string, TopicMerge> topics;
    for (const auto & node : nodes_)
    {
      if (node->type == "topic")
      {
        topics[node->name].Add(node.get());
      }
    }
    for (auto & merge : topics)
    {
      topics_.push_back(merge.second.Convert());
    }
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

ConfigNode * ConfigFile::Parse(YAML::Node yaml, const std::string & path)
{
  // create config node
  const auto node = nodes_.emplace_back(std::make_unique<ConfigNode>(yaml, path)).get();
  if (!node->yaml.IsMap())
  {
    throw ConfigError::ParseFile(node->path + " is not a dict");
  }

  // get class name
  node->type = node->TakeNode("class").as<std::string>();

  // save the text for reference in some types
  if (node->type == "topic")
  {
    node->name = node->TakeNode("name").as<std::string>();
    node->data = node->TakeNode("data").as<std::string>();
  }
  if (node->type == "target")
  {
    node->name = node->TakeNode("name").as<std::string>();
  }

  // parse input node
  const auto input = node->TakeNode("input", true);
  if (input)
  {
    node->input = Parse(input, node->path + ".input");
  }

  // parse child nodes
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

}  // namespace multi_data_monitor
