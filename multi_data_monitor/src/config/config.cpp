// Copyright 2021 Takagi, Isamu
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
#include <iostream>

namespace monitors
{

YAML::Node GetNecessary(const YAML::Node & node, const std::string & name, const std::string & prefix)
{
  if (node[name]) { return node[name]; }
  throw ConfigError(prefix + " requires '" + name + "' field");
}

TopicConfig::TopicConfig(YAML::Node node)
{
  name = GetNecessary(node, "name", "topic").as<std::string>();
  type = GetNecessary(node, "type", "topic").as<std::string>();
  depth = node["qos"]["depth"].as<int>(1);
  reliability = node["qos"]["reliability"].as<std::string>("default");
  durability = node["qos"]["durability"].as<std::string>("default");
}

FieldConfig::FieldConfig(YAML::Node node, const std::string & topic): topic(topic)
{
  name = GetNecessary(node, "name", "field").as<std::string>();
  type = node["type"].as<std::string>("");
}

ObjectConfig::ObjectConfig(YAML::Node node) : custom(node)
{
  klass = GetNecessary(node, "class", "monitor").as<std::string>();
  node.remove("class");

  std::cout << "===========================================================================" << std::endl;
  std::cout << custom << std::endl;
}

MonitorConfig::MonitorConfig(YAML::Node node)
{
  if (node["topic"])
  {
    topic = TopicConfig(GetNecessary(node, "topic", "monitor"));
    field = FieldConfig(GetNecessary(node, "field", "monitor"), topic.value().name);
    node.remove("topic");
    node.remove("field");
  }
  object = ObjectConfig(node);
}

}  // namespace monitors
