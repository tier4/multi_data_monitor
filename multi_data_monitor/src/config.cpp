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
#include <filesystem>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

// clang-format off
#include <iostream>
#include <fstream>
#include <fmt/format.h>

namespace multi_data_monitor
{

// TODO(Takagi, Isamu): move to tools
void WriteFile(const std::string & path, std::vector<std::string> & lines)
{
  std::ofstream ofs(path);
  for (const auto & line : lines)
  {
    ofs << line << std::endl;
  }
}

// TODO(Takagi, Isamu): move to tools
void WriteConfig(const std::string & path, const std::vector<std::unique_ptr<NodeConfig>> & nodes)
{
  std::vector<std::string> plant;
  plant.push_back("@startuml");
  for (const auto & node : nodes)
  {
    const auto ptr = static_cast<void *>(node.get());
    std::string type = node->type;
    if (type == "target") { type = "[" + node->name + "]"; }
    plant.push_back(fmt::format("card \"{}\" as {}", type, ptr));
  }
  for (const auto & node : nodes)
  {
    const auto src = static_cast<void *>(node.get());
    if (node->stream)
    {
      const auto dst = static_cast<void *>(node->stream);
      plant.push_back(fmt::format("{} --> {}", src, dst));
    }
    if (src != node->target)
    {
      const auto dst = static_cast<void *>(node->target);
      plant.push_back(fmt::format("{} --> {}", src, dst));
    }
    for (const auto & child : node->children)
    {
      const auto dst = static_cast<void *>(child);
      plant.push_back(fmt::format("{} --> {}", src, dst));
    }
  }
  plant.push_back("@enduml");
  WriteFile(path, plant);
}
}  // namespace multi_data_monitor
// clang-format on

namespace multi_data_monitor
{

class FieldMerge
{
public:
  void Add(NodeConfig * node);

private:
  std::string data;
};

class TopicMerge
{
public:
  void Add(NodeConfig * node);
  TopicConfig Convert();

private:
  std::string name;
  std::unordered_set<std::string> types;
  std::unordered_set<size_t> depths;
  std::unordered_set<std::string> reliabilities;
  std::unordered_set<std::string> durabilities;
  std::unordered_map<std::string, FieldMerge> merge_fields;
};

void FieldMerge::Add(NodeConfig * node)
{
  data = node->data;
}

void TopicMerge::Add(NodeConfig * node)
{
  const auto type = node->TakeNode("type", true).as<std::string>("");
  if (!type.empty())
  {
    types.insert(type);
  }

  const auto qos = node->TakeNode("qos", true).as<std::string>("");
  if (!qos.empty())
  {
    static const auto valid_reliability = std::unordered_set<std::string>({"D", "R", "B"});
    static const auto valid_durability = std::unordered_set<std::string>({"D", "V", "T"});
    try
    {
      const auto depth = std::stoul(qos.substr(2));
      const auto reliability = qos.substr(0, 1);
      const auto durability = qos.substr(1, 1);

      if (valid_reliability.count(reliability) == 0 || valid_durability.count(durability) == 0)
      {
        throw std::logic_error("");
      }

      depths.insert(depth);
      reliabilities.insert(reliability);
      durabilities.insert(durability);
    }
    catch (const std::logic_error &)
    {
      throw node->Error("has invalid qos settings '" + qos + '"');
    }
  }

  merge_fields[node->data].Add(node);
  name = node->name;
  node->CheckUnknownKeys();
}

TopicConfig TopicMerge::Convert()
{
  const auto check_unique = [this](auto set, const std::string & var)
  {
    if (set.size() != 1)
    {
      throw ConfigError("topic " + var + " is not unique: " + name);
    }
  };

  // clang-format off
  if (depths.empty()) { depths.insert(1); }
  if (reliabilities.empty()) { reliabilities.insert("D"); }
  if (durabilities.empty()) { durabilities.insert("D"); }
  // clang-format on

  check_unique(types, "type");
  check_unique(depths, "depth");
  check_unique(reliabilities, "reliability");
  check_unique(durabilities, "durability");

  std::vector<FieldConfig> fields;
  for (const auto & pair : merge_fields)
  {
    fields.push_back(FieldConfig{pair.first});
  }
  const auto type = *types.begin();
  const auto depth = *depths.begin();
  const auto reliability = *reliabilities.begin();
  const auto durability = *durabilities.begin();

  return {name, type, depth, reliability, durability, fields};
}

NodeConfig::NodeConfig(YAML::Node yaml, const std::string & path, const std::string & mode)
{
  if (yaml.IsScalar())
  {
    this->mode = mode;
    this->path = path;
    this->yaml["class"] = "target";
    this->yaml["name"] = yaml;
  }
  else
  {
    this->mode = mode;
    this->path = path;
    this->yaml.reset(yaml);
  }

  this->target = this;
  this->stream = nullptr;
}

ConfigError NodeConfig::Error(const std::string message)
{
  return ConfigError(fmt::format("{} '{}' {}", path, type, message));
}

void NodeConfig::CheckUnknownKeys()
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

YAML::Node NodeConfig::TakeNode(const std::string & name, bool optional)
{
  const auto node = yaml[name];
  if (optional || node)
  {
    yaml.remove(name);
    return node;
  }
  throw Error(fmt::format("has no '{}'", name));
}

void NodeConfig::RefreshTarget()
{
  if (stream)
  {
    stream = stream->target;
  }
  for (auto & node : children)
  {
    node = node->target;
  }
}

NodeConfig * NodeConfig::ResolveTarget()
{
  if (target != target->target)
  {
    target = target->ResolveTarget();
  }
  return target;
}

std::filesystem::path ResolveFileSystemPath(const std::string & path)
{
  const std::string scheme = "file://";
  if (path.find(scheme) == 0)
  {
    return std::filesystem::path(path.substr(scheme.size()));
  }
  throw SystemError("invalid path: " + path);
}

std::filesystem::path ResolvePackagePath(const std::string & path)
{
  try
  {
    const std::string scheme = "package://";
    if (path.find(scheme) == 0)
    {
      size_t pos1 = scheme.size();
      size_t pos2 = path.find("/", pos1);
      if (pos2 == std::string::npos)
      {
        throw SystemError("invalid path: " + path);
      }
      const auto package_name = path.substr(pos1, pos2 - pos1);
      const auto package_path = ament_index_cpp::get_package_share_directory(package_name);
      return std::filesystem::path(package_path + path.substr(pos2));
    }
    return ResolveFileSystemPath(path);
  }
  catch (const ament_index_cpp::PackageNotFoundError & error)
  {
    throw SystemError("package not found: " + error.package_name);
  }
}

ConfigFile::ConfigFile(const std::string & file)
{
  try
  {
    // check to distinguish from errors in YAML::LoadFile
    const auto file_path = ResolvePackagePath(file);
    if (!std::filesystem::exists(file_path))
    {
      throw SystemError("file not found: " + file_path.string());
    }

    // load config and convert version if possible
    const auto yaml = YAML::LoadFile(file_path);
    if (yaml["version"].as<std::string>("") != "1")
    {
      throw ConfigError("this version is not supported");
    }

    // laod stylesheets
    for (const auto & style : yaml["stylesheets"])
    {
      NodeConfig config(style, "stylesheets", "stylesheets");
      const auto target = config.TakeNode("target", true).as<std::string>("");
      const auto path = config.TakeNode("path").as<std::string>("");
      const auto file = ResolvePackagePath(path);
      std::ifstream ifs(file);
      if (ifs)
      {
        std::cout << "load: " + file.string() << std::endl;
        std::stringstream buffer;
        buffer << ifs.rdbuf();
        stylesheets[target] += buffer.str();
      }
    }

    // load widgets
    std::unordered_map<std::string, NodeConfig *> widgets;
    for (const auto & pair : yaml["widgets"])
    {
      const auto name = pair.first.as<std::string>();
      const auto node = Parse(pair.second, name, "view");
      widgets.emplace(name, node);
    }

    // load streams
    std::unordered_map<std::string, NodeConfig *> streams;
    for (const auto & pair : yaml["streams"])
    {
      const auto name = pair.first.as<std::string>();
      const auto node = Parse(pair.second, name, "data");
      streams.emplace(name, node);
    }

    // connect targets
    for (const auto & node : nodes_)
    {
      // TODO(Takagi, Isamu): check namespace
      if (node->type == "target")
      {
        // clang-format off
        if (widgets.count(node->name)) { node->target = widgets.at(node->name); }
        if (streams.count(node->name)) { node->target = streams.at(node->name); }
        // clang-format on
      }
    }
    // WriteConfig("graph1.plantuml", nodes_);  // TODO(Takagi, Isamu): remove

    // resolve targets
    for (const auto & node : nodes_)
    {
      node->ResolveTarget();
    }
    for (const auto & node : nodes_)
    {
      node->RefreshTarget();
    }
    root_ = widgets["root"]->target;

    // release unused nodes
    std::vector<std::unique_ptr<NodeConfig>> temporary;
    for (auto & node : nodes_)
    {
      if (node->type != "target")
      {
        std::unique_ptr<NodeConfig> t;
        node.swap(t);
        temporary.push_back(std::move(t));
      }
    }
    nodes_.swap(temporary);
    // WriteConfig("graph2.plantuml", nodes_);  // TODO(Takagi, Isamu): remove

    // TODO(Takagi, Isamu): check tree

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
  catch (YAML::Exception & error)
  {
    throw SystemError(error.what());
  }
}

NodeConfig * ConfigFile::Parse(YAML::Node yaml, const std::string & path, const std::string & mode)
{
  // create config node
  const auto node = nodes_.emplace_back(std::make_unique<NodeConfig>(yaml, path, mode)).get();
  if (!node->yaml.IsMap())
  {
    throw ConfigError(node->path + " is not a dict");
  }

  // get class name
  const auto class_field = (mode == "data") ? "model" : "class";
  node->type = node->TakeNode(class_field).as<std::string>();

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
    node->stream = Parse(input, node->path + ".input", "data");
  }

  // parse child nodes
  const auto parse_children = [this](NodeConfig * node, const std::string & field, const std::string & mode)
  {
    const auto children = node->TakeNode(field, true);
    if (children)
    {
      const std::string path = node->path + "." + field;
      if (!children.IsSequence())
      {
        throw ConfigError(path + " is not a list");
      }
      for (size_t i = 0, n = children.size(); i < n; ++i)
      {
        node->children.push_back(Parse(children[i], fmt::format("{}[{}]", path, i), mode));
      }
    }
  };
  parse_children(node, "rules", "rule");
  parse_children(node, "children", "view");
  return node;
}

const NodeConfig * ConfigFile::GetRoot() const
{
  return root_;
}

const std::vector<TopicConfig> & ConfigFile::GetTopics() const
{
  return topics_;
}

const std::vector<NodeConfig *> ConfigFile::GetNodes(const std::string & mode) const
{
  std::vector<NodeConfig *> nodes;
  for (const auto & node : nodes_)
  {
    if (mode.empty() || mode == node->mode)
    {
      nodes.push_back(node.get());
    }
  }
  return nodes;
}

const std::string ConfigFile::GetStyleSheet(const std::string & target) const
{
  return stylesheets.count(target) ? stylesheets.at(target) : std::string();
}

}  // namespace multi_data_monitor
